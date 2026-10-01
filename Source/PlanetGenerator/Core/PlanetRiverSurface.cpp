#include "PlanetRiverSurface.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace PlanetCore {
	/**
	 * 最終河床と地点水位の交差を挟込み，水面の横断形状を求める
	 * @param Field 描画・衝突と共有する不変な最終地形
	 * @param ReachIndex 対象河道の番号
	 * @param Fraction 下流に向かう正規化した位置，０から１迄
	 * @param Stop 中断要求
	 * @return 中心と左右の岸又は接続境界，不正入力・乾いた中心・中断時は空
	 */
	std::optional<RiverWaterSection> SampleRiverWaterSection(
		const RiverTerrainField &Field, const std::size_t ReachIndex, const double Fraction, const std::stop_token Stop) {
		// 不正な河道又は中断要求に対する返戻
		if(ReachIndex >= Field.Channels().size() || Stop.stop_requested()) return {};
		const std::optional<RiverProfileSample> Profile = Field.Drainage().SampleReach(ReachIndex, Fraction);
		// 範囲外又は非有限の位置に対する返戻
		if(!Profile) return {};
		const double Radius = Field.Base().Settings().RadiusMeters;
		const double Sea = Field.Drainage().Settings().SeaLevelMeters;
		const RiverTerrainChannel &Channel = Field.Channels()[ReachIndex];
		const UnitDirection Across = *UnitDirection::FromVector(Profile->Direction.Cross(Profile->DownstreamTangent));
		RiverWaterSection Result;
		Result.DownstreamTangent = Profile->DownstreamTangent;
		/**
		 * 同じ球面上の横断位置で，地面と海・河川の水位を一緒に評価する
		 * @param Distance 中心からの符号付き弧長
		 * @return 地点と其の水面・地面の標高
		 */
		const auto Evaluate = [&](const double Distance) {
			const double Angle = Distance / Radius;
			const UnitDirection Direction =
				*UnitDirection::FromVector(Profile->Direction * std::cos(Angle) + Across.Vector() * std::sin(Angle));
			const RiverSurfaceSample Surface = Field.EvaluateSurface(Direction);
			RiverWaterEdge Edge;
			Edge.Direction = Direction.Vector();
			// 河道被覆の終了を陸岸と誤認しない為，自身の横断水位を未評価域へ延長する
			// 限界迄水没している断面は閉じずに返し，後段で水面の支持域を拡張する
			Edge.bUsesProfileExtension = Surface.WaterSupport <= 0 && Profile->WaterMeters > Sea;
			Edge.WaterMeters = std::max(Sea, Surface.WaterSupport > 0 ? Surface.WaterElevationMeters : Profile->WaterMeters);
			Edge.GroundMeters = Surface.Terrain.ElevationMeters;
			Edge.DistanceMeters = std::abs(Distance);
			Edge.Boundary =
				Edge.WaterMeters == Sea && Edge.GroundMeters < Sea ? RiverWaterBoundary::Ocean : RiverWaterBoundary::OpenWater;
			++Result.Evaluations;
			// 同じ位置で評価した水位と地面の返戻
			return Edge;
		};
		Result.Center = Evaluate(0);
		// 水面より上に地面が出る中心は，描画で隠さず拒否した結果の返戻
		if(!(Result.Center.WaterMeters > Result.Center.GroundMeters)) return {};
		for(const double Sign : {-1.0, 1.0}) {
			RiverWaterEdge Wet = Result.Center;
			RiverWaterEdge &Edge = Sign > 0 ? Result.Left : Result.Right;
			double StartDistance = 0, EndDistance = Channel.InnerHalfWidthMeters;
			bool bFoundBank = false;
			// 狭い川幅の外でも水没が続く場合は，同じ不変場の谷の支持範囲迄段階的に調べる
			// 有限標本間の任意の微小島の不存在は保証せず，最後迄水没なら開いた端を保持する
			while(!bFoundBank) {
				for(std::uint32_t i = 1; i <= 128; ++i) {
					// 探索中の中断要求に対する返戻
					if(Stop.stop_requested()) return {};
					RiverWaterEdge Dry = Evaluate(Sign * std::lerp(StartDistance, EndDistance, i / 128.0));
					if(Dry.GroundMeters < Dry.WaterMeters) {
						Wet = Dry;
						Edge = Wet;
						continue;
					}
					for(std::uint32_t j = 0; j < 64 && Dry.DistanceMeters - Wet.DistanceMeters > 1E-4; ++j) {
						// 挟込み中の中断要求に対する返戻
						if(Stop.stop_requested()) return {};
						const RiverWaterEdge Middle = Evaluate(Sign * std::midpoint(Wet.DistanceMeters, Dry.DistanceMeters));
						if(Middle.GroundMeters < Middle.WaterMeters)
							Wet = Middle;
						else
							Dry = Middle;
					}
					Edge = Wet;
					Edge.Boundary = RiverWaterBoundary::DryBank;
					Edge.BracketWidthMeters = Dry.DistanceMeters - Wet.DistanceMeters;
					bFoundBank = true;
					break;
				}
				if(bFoundBank || Edge.Boundary == RiverWaterBoundary::Ocean || EndDistance >= Channel.OuterHalfWidthMeters)
					break;
				StartDistance = EndDistance;
				EndDistance = std::min(2 * EndDistance, Channel.OuterHalfWidthMeters);
			}
		}
		// 実際の交差位置と，陸岸又は接続先の区別の返戻
		return Result;
	}
}
