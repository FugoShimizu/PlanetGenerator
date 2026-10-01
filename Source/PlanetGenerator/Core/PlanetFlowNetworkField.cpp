#include "PlanetFlowNetworkField.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace PlanetCore {
	namespace {
		/**
		 * 単位球上の保守的な閉じた箱で，照会候補を絞る
		 * @param Point 量子化していない照会方向
		 * @param Low 下端
		 * @param High 上端
		 * @return 三軸の全てを含む場合に真
		 */
		bool InsideFlowBounds(const Vec3 Point, const Vec3 Low, const Vec3 High) {
			// 保守的な包含の返戻
			return Point.X >= Low.X && Point.Y >= Low.Y && Point.Z >= Low.Z && Point.X <= High.X && Point.Y <= High.Y && Point.Z <= High.Z;
		}
	}

	/**
	 * 共有元場と全河道を所有し，湖内部と連続曲線の保守的な範囲を準備する
	 * @param Network 同じ元場で全流出口を準備した有限網
	 * @return 完成した不変場；欠落・重複・尺度不一致では空
	 */
	std::optional<FlowNetworkField> FlowNetworkField::Create(const PreparedFlowNetwork &Network) {
		// 有限網と元場が無い場合の返戻
		if(!Network.Source || Network.Channels.empty() || Network.Channels.size() > 1024) return {};
		FlowNetworkField Result;
		Result.Source = Network.Source;
		Result.SourceHeight = Network.SourceHeight;
		Result.OutsideSupport = -std::numeric_limits<double>::infinity();
		std::set<const FlowChannelField *> Unique;
		for(const FlowBasinChannel &Entry : Network.Channels) {
			// 同じ河道を二重に開削する入力への返戻
			if(!Entry.Channel.Field || !Unique.insert(Entry.Channel.Field.get()).second) return {};
			const double Radius = Entry.Channel.Field->GetRadiusMeters();
			if(Result.Channels.empty()) Result.RadiusMeters = Radius;
			// 異なる物理尺度を一つの場へ混在させない返戻
			if(Radius != Result.RadiusMeters) return {};
			const FlowWaterGeometry Geometry = Entry.Channel.Field->GetWaterGeometry();
			ChannelBounds Bounds { { 3, 3, 3 }, { -3, -3, -3 }, Entry.Channel.Field };
			const auto Extend = [&](const Vec3 Center, const double Extent) {
				Bounds.Low = {
					std::min(Bounds.Low.X, Center.X - Extent),
					std::min(Bounds.Low.Y, Center.Y - Extent),
					std::min(Bounds.Low.Z, Center.Z - Extent)
				};
				Bounds.High = {
					std::max(Bounds.High.X, Center.X + Extent),
					std::max(Bounds.High.Y, Center.Y + Extent),
					std::max(Bounds.High.Z, Center.Z + Extent)
				};
				// 保守的な箱の追加の終了
				return;
			};
			// FlowRoute と同じ五次曲線の曲率予算に基づく，３倍の弧長の包囲球
			const double Margin = (2 * Geometry.MaximumChannelHalfWidthMeters + 0.1) / Radius + 1E-14;
			for(const TerrainFlowSegment &Segment : Geometry.Route.GetSegments()) {
				Extend(Segment.Evaluate(0)->Direction.Vector(), 3 * Segment.GetParameterLengthMeters() / Radius + Margin);
			}
			// 正の中心投影を持つ岸の多角形内部は，頂点の最大偏角の球冠内に含まれる
			for(const SpillLakeShore *Shore : { &Geometry.Upstream, Geometry.Receiving }) {
				if(!Shore) continue;
				const std::vector<TerrainContourSample> &Points = Shore->GetContour().Samples;
				// 不完全な岸から範囲を捏造しない返戻
				if(Points.empty()) return {};
				const Vec3 Center = Points.front().Direction.Vector();
				double Extent = 0;
				for(const TerrainContourSample &Point : Points) Extent = std::max(Extent, (Point.Direction.Vector() - Center).Length());
				Extend(Center, Extent + Margin);
			}
			Result.OutsideSupport = std::max(Result.OutsideSupport, -Geometry.MaximumChannelHalfWidthMeters);
			Result.Channels.push_back(std::move(Bounds));
		}
		// 準備コンテナから独立した所有場の返戻
		return Result;
	}

	/**
	 * 全河道で確認済の共通半径を返す
	 * @return 物理半径
	 */
	double FlowNetworkField::GetRadiusMeters() const {
		return RadiusMeters;
	}

	/**
	 * 共通の描画準備へ，同じ湖岸を参照する全河道の幾何を渡す
	 * @return この場が所有する河道・岸への参照列
	 */
	std::vector<FlowWaterGeometry> FlowNetworkField::GetWaterGeometries() const {
		std::vector<FlowWaterGeometry> Result;
		Result.reserve(Channels.size());
		for(const ChannelBounds &Channel : Channels) Result.push_back(Channel.Field->GetWaterGeometry());
		// 所有元が同じ全幾何の返戻
		return Result;
	}

	/**
	 * 河道と湖の支持域の和集合を，同じ地点から照会する
	 * @param Direction 球面方向
	 * @return 内側で正となる支持値；元の照会が不正な場合は空
	 * @note 河道数 K に対して O(K)，包囲箱外では詳細場を評価しない
	 */
	std::optional<double> FlowNetworkField::WaterSupport(const UnitDirection Direction) const {
		double Support = OutsideSupport;
		std::optional<double> Elevation;
		for(const ChannelBounds &Channel : Channels) {
			if(!InsideFlowBounds(Direction.Vector(), Channel.Low, Channel.High)) continue;
			if(!Elevation) Elevation = SourceHeight ? SourceHeight(Direction) : Source(Direction).ElevationMeters;
			// 元場を失った所属照会を受理しない返戻
			if(!std::isfinite(*Elevation)) return {};
			const std::optional<double> Value = Channel.Field->WaterSupportOnHeight(Direction, *Elevation);
			// 不正な所属を乾いた領域に置換えない返戻
			if(!Value || !std::isfinite(*Value)) return {};
			Support = std::max(Support, *Value);
		}
		// 同じ支持域の和集合の返戻
		return Support;
	}

	/**
	 * 有限幅の開削量を二乗和平方根で連続合成し，同じ支持域の水位を照会する
	 * @param Direction 元場と同じ球面方向
	 * @param NeedsWater 水位を必要とする場合に真
	 * @param NeedsTerrainGradient 地面の接線勾配を必要とする場合に真
	 * @return 合成した地形と水位・所属；不整合な候補は空
	 * @note 開削量が全て零の境界では各微分も零の為，合成勾配も零へ接続する
	 * @note K 本の開削量の最大を C とすると，合成量は C 以上 sqrt(K)*C 以下
	 */
	std::optional<FlowWaterSample> FlowNetworkField::EvaluateSample(
		const UnitDirection Direction,
		const bool NeedsWater,
		const bool NeedsTerrainGradient
	) const {
		const TerrainSample Base =
		NeedsTerrainGradient || !SourceHeight ? Source(Direction) : TerrainSample{ SourceHeight(Direction), {} };
		// 同じ元場を失った地点を零標高へ置換えない返戻
		if(!std::isfinite(Base.ElevationMeters) || !std::isfinite(Base.TangentialGradientPerRadian.Length())) return {};
		FlowWaterSample Sample { { Base, {}, 0, false }, OutsideSupport };
		FlowChannelSection &Result = Sample.Section;
		double CutSquared = 0, Coverage = 0, NearestSupport = -std::numeric_limits<double>::infinity();
		bool HasSupportedWater = false;
		Vec3 WeightedGradient;
		for(const ChannelBounds &Channel : Channels) {
			if(!InsideFlowBounds(Direction.Vector(), Channel.Low, Channel.High)) continue;
			const std::optional<FlowChannelSection> Value =
			Channel.Field->EvaluateOnSource(Direction, Base, NeedsWater, NeedsTerrainGradient);
			// 不正な河道を合成から除外しない返戻
			if(!Value || !std::isfinite(Value->Terrain.ElevationMeters)) return {};
			const double Cut = Base.ElevationMeters - Value->Terrain.ElevationMeters;
			// 別の元場又は盛土を開削量として黙って丸めない返戻
			if(Cut < 0) return {};
			CutSquared += Cut * Cut;
			if(NeedsTerrainGradient) {
				WeightedGradient = WeightedGradient + (Base.TangentialGradientPerRadian - Value->Terrain.TangentialGradientPerRadian) * Cut;
			}
			Coverage = std::max(Coverage, Value->BedCoverage);
			if(!NeedsWater) continue;
			const std::optional<double> Support = Channel.Field->WaterSupportOnHeight(Direction, Base.ElevationMeters);
			// 支持域の水位を失った場合の返戻
			if(!Support || !std::isfinite(*Support) || *Support >= 0 && !Value->HasWater) return {};
			Sample.SupportMeters = std::max(Sample.SupportMeters, *Support);
			if(!Value->HasWater) continue;
			const bool FiniteWater =
			std::isfinite(Value->Water.ElevationMeters) && std::isfinite(Value->Water.TangentialGradientPerRadian.Length());
			// 非有限な水面を選択しない返戻
			if(!FiniteWater) return {};
			// 水際の補間と法線照会の為，支持域の直外にも同じ水位候補を保持
			const bool Select = *Support >= 0 ?
			!HasSupportedWater || Value->Water.ElevationMeters > Result.Water.ElevationMeters :
			!HasSupportedWater && *Support > NearestSupport;
			if(Select) {
				Result.Water = Value->Water;
				Result.HasWater = true;
				NearestSupport = *Support;
			}
			HasSupportedWater = HasSupportedWater || *Support >= 0;
		}
		const double Cut = std::sqrt(CutSquared);
		Result.Terrain.ElevationMeters -= Cut;
		if(NeedsTerrainGradient && Cut) {
			Result.Terrain.TangentialGradientPerRadian = Result.Terrain.TangentialGradientPerRadian - WeightedGradient * (1 / Cut);
		}
		Result.BedCoverage = Coverage;
		const bool Finite =
		std::isfinite(Result.Terrain.ElevationMeters) && std::isfinite(Result.Terrain.TangentialGradientPerRadian.Length());
		// 演算範囲を超えた合成を公開しない返戻
		if(!Finite) return {};
		// 同じ値と解析勾配及び計算済の所属を渡す返戻
		return Sample;
	}

	/**
	 * 要求された属性だけを計算し，従来の断面照会へ渡す
	 * @param Direction 元場と同じ球面方向
	 * @param NeedsWater 水位を必要とする場合に真
	 * @param NeedsTerrainGradient 地面の接線勾配を必要とする場合に真
	 * @return 同じ合成断面；不整合な元場又は水位は空
	 */
	std::optional<FlowChannelSection> FlowNetworkField::Evaluate(
		const UnitDirection Direction,
		const bool NeedsWater,
		const bool NeedsTerrainGradient
	) const {
		const std::optional<FlowWaterSample> Sample = EvaluateSample(Direction, NeedsWater, NeedsTerrainGradient);
		// 所属を再計算せず，従来の公開型へ戻した返戻
		return Sample ? std::optional<FlowChannelSection>(Sample->Section) : std::nullopt;
	}

	/**
	 * 水位の選択に使った所属を同時に返し，元場と河道の再照会を省く
	 * @param Direction 元場と同じ球面方向
	 * @param NeedsTerrainGradient 地面の接線勾配を使う場合に真；水位の勾配は常に有効
	 * @return 同じ断面と支持距離；不整合な元場又は水位は空
	 */
	std::optional<FlowWaterSample> FlowNetworkField::EvaluateWithSupport(
		const UnitDirection Direction,
		const bool NeedsTerrainGradient
	) const {
		// 同じ水位選択と開削合成の結果を一度で返す
		return EvaluateSample(Direction, true, NeedsTerrainGradient);
	}
}
