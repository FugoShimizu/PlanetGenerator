#include "PlanetErodedTerrain.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace PlanetCore {
	/**
	 * 単位球の索引座標を求める
	 * @param Coordinate 単位球又は検索境界の座標
	 * @return ０～３１のセル番号
	 */
	std::uint32_t PlanetErodedTerrain::GridCoordinate(const double Coordinate) {
		// 境界を含む索引番号の返戻
		return static_cast<std::uint32_t>(std::clamp(std::floor((Coordinate + 1) * 16), 0.0, 31.0));
	}

	/**
	 * 固定した地点順序を，格子毎に安定な順序で索引へ格納する
	 */
	void PlanetErodedTerrain::BuildIndex() {
		Offsets.resize(32769);
		for(const Site &Value : Sites) {
			const Vec3 Point = Value.Direction;
			const std::uint32_t Cell = GridCoordinate(Point.X) + (GridCoordinate(Point.Y) << 5) + (GridCoordinate(Point.Z) << 10);
			++Offsets[Cell + 1];
		}
		for(std::size_t i = 1; i < Offsets.size(); ++i) Offsets[i] += Offsets[i - 1];
		std::vector<std::uint32_t> Cursor = Offsets;
		Order.resize(Sites.size());
		for(std::uint32_t i = 0; i < Sites.size(); ++i) {
			const Vec3 Point = Sites[i].Direction;
			const std::uint32_t Cell = GridCoordinate(Point.X) + (GridCoordinate(Point.Y) << 5) + (GridCoordinate(Point.Z) << 10);
			Order[Cursor[Cell]++] = i;
		}
		// 終了
		return;
	}

	/**
	 * 全球を覆う固定網の侵食率を補間用の不変な場へ変換する
	 * @param Ocean 元の海陸・海底
	 * @param Network 侵食と同じ生成定義の６段階球面網
	 * @param Erosion 同じ排水網の離散的な侵食結果
	 * @param Stop 中断要求
	 * @return 有効な補間場，非対応の網・不正な結果・中断時は空
	 */
	std::optional<PlanetErodedTerrain> PlanetErodedTerrain::Create(
		std::shared_ptr<const OceanConnectivityField> Ocean,
		const DrainageNetwork &Network,
		const PlanetFluvialErosion &Erosion,
		const std::stop_token Stop
	) {
		// 異なる定義又は規模の入力の返戻
		if(
			!Ocean || Network.SourceDefinitionHash() != Ocean->DefinitionHash() ||
			Network.SourceDefinitionHash() != Erosion.GetSourceDefinitionHash() || Network.Nodes().size() != Erosion.GetNodes().size() ||
			Stop.stop_requested()
		) return {};
		const std::optional<SphereMesh> Sphere = BuildUniformSphere(6);
		// 被覆を保証する固定網と異なる場合の返戻
		if(!Sphere || Sphere->Directions.size() + Network.Statistics().CoastalOutletNodes != Network.Nodes().size()) return {};
		PlanetErodedTerrain Result;
		Result.Source = std::move(Ocean);
		Result.Sites.reserve(Sphere->Directions.size());
		for(std::size_t i = 0; i < Sphere->Directions.size(); ++i) {
			const DrainageNode &Node = Network.Nodes()[i];
			const FluvialErosionNode &Value = Erosion.GetNodes()[i];
			// 順序の異なる網や，有限な対数で扱えない侵食率の返戻
			if(
				Stop.stop_requested() || (Node.Direction - Sphere->Directions[i].Vector()).Length() > 1E-12 ||
				!std::isfinite(Value.RelativeReliefRatio) || Value.RelativeReliefRatio <= 0 || Value.RelativeReliefRatio > 1 ||
				Node.Ocean && Value.RelativeReliefRatio != 1
			) return {};
			Result.Sites.push_back({ Node.Direction, std::log(Value.RelativeReliefRatio) });
		}
		// 追加した河口は輸送の終点であり，固定球面網の地形補間標本には混ぜない
		for(std::size_t i = Sphere->Directions.size(); i < Network.Nodes().size(); ++i) {
			const DrainageNode &Node = Network.Nodes()[i];
			const FluvialErosionNode &Value = Erosion.GetNodes()[i];
			// 面積を持つ陸域や侵食値を，河口として補間から除外しない返戻
			if(
				Stop.stop_requested() || !Node.Ocean || Node.Downstream != i || Node.Outlet != i ||
				Node.OriginalElevationMeters > Network.Settings().SeaLevelMeters || Value.RelativeReliefRatio != 1 ||
				Value.ErodedCubicMeters != 0
			) return {};
		}
		// 球面三角形内の点と頂点の距離を覆い，支持端でも重みの和が消えない幅
		for(const std::array<std::uint32_t, 3> &Triangle : Sphere->Triangles) for(std::uint32_t i = 0; i < 3; ++i) {
			const Vec3 First = Result.Sites[Triangle[i]].Direction, Second = Result.Sites[Triangle[(i + 1) % 3]].Direction;
			Result.SupportChord = std::max(Result.SupportChord, (First - Second).Length() * 1.5);
		}
		const double Radius = Result.Source->Base().Settings().RadiusMeters;
		Result.CoastTransitionMeters = 10 * Radius / (Radius + 250000);
		Result.BuildIndex();
		// 公開直前に中断された場の返戻
		if(Stop.stop_requested()) return {};
		// 海岸の標高と微分を維持する侵食場の返戻
		return Result;
	}

	/**
	 * 対数起伏率を C2 の有限幅重みで補間し，海岸で侵食を滑らかに消す
	 * @tparam HasDerivatives 解析勾配を計算する場合に真
	 * @param Direction 評価する単位方向
	 * @param IsIndexed 空間索引を使う場合に真，全点探索との独立比較に用いる
	 * @return 侵食後の標高と接線勾配
	 */
	template<bool HasDerivatives> TerrainSample PlanetErodedTerrain::EvaluateImpl(
		const UnitDirection Direction,
		const bool IsIndexed
	) const {
		TerrainSample Original;
		if constexpr(HasDerivatives) Original = Source->Evaluate(Direction);
		else Original.ElevationMeters = Source->EvaluateElevation(Direction);
		const double Sea = Source->Base().Settings().SeaLevelMeters, Height = Original.ElevationMeters - Sea;
		// 海面以下の地点を其の儘返す，海底地形には陸上侵食を加えない
		if(Height <= 0) return Original;
		const Vec3 Point = Direction.Vector();
		const double SupportSquared = SupportChord * SupportChord;
		double WeightSum = 0, WeightedLogRatio = 0;
		Vec3 WeightGradient, RatioGradient;
		/**
		 * 一つの地点の滑らかな寄与を積算する
		 * @param Index 寄与する可能性が有る地点番号
		 */
		const auto Accumulate = [&](const std::uint32_t Index) {
			const Site &Value = Sites[Index];
			const Vec3 Difference = Point - Value.Direction;
			const double Distance = Difference.Dot(Difference) / SupportSquared;
			// 支持範囲外の地点の終了
			if(Distance >= 1) return;
			const double Rest = 1 - Distance, Weight = Rest * Rest * Rest;
			WeightSum += Weight;
			WeightedLogRatio += Weight * Value.LogReliefRatio;
			if constexpr(HasDerivatives) {
				const Vec3 Gradient = (Difference - Point * Difference.Dot(Point)) * (-6 * Rest * Rest / SupportSquared);
				WeightGradient = WeightGradient + Gradient;
				RatioGradient = RatioGradient + Gradient * Value.LogReliefRatio;
			}
			// 終了
			return;
		};
		if(IsIndexed) {
			const std::uint32_t MinX = GridCoordinate(Point.X - SupportChord), MaxX = GridCoordinate(Point.X + SupportChord);
			const std::uint32_t MinY = GridCoordinate(Point.Y - SupportChord), MaxY = GridCoordinate(Point.Y + SupportChord);
			const std::uint32_t MinZ = GridCoordinate(Point.Z - SupportChord), MaxZ = GridCoordinate(Point.Z + SupportChord);
			for(std::uint32_t z = MinZ; z <= MaxZ; ++z) {
				for(std::uint32_t y = MinY; y <= MaxY; ++y) for(std::uint32_t x = MinX; x <= MaxX; ++x) {
					const std::uint32_t Cell = x + (y << 5) + (z << 10);
					for(std::uint32_t i = Offsets[Cell]; i < Offsets[Cell + 1]; ++i) Accumulate(Order[i]);
				}
			}
		} else for(const std::uint32_t Index : Order) Accumulate(Index);
		// 被覆違反を元の地形で隠さない返戻，此の分岐は固定網の被覆検査で検出する
		if(WeightSum <= 0) return { std::numeric_limits<double>::quiet_NaN(), {} };
		// 侵食の無い領域では引算と再加算の丸めも加えない返戻
		if(WeightedLogRatio == 0) return Original;
		const double LogRatio = WeightedLogRatio / WeightSum, ScaledHeight = Height / CoastTransitionMeters;
		const double Denominator = 1 + ScaledHeight * ScaledHeight, CoastMask = ScaledHeight * ScaledHeight / Denominator;
		const double Ratio = std::exp(LogRatio * CoastMask);
		TerrainSample Result { Sea + Height * Ratio, {} };
		if constexpr(HasDerivatives) {
			const Vec3 LogGradient = (RatioGradient - WeightGradient * LogRatio) * (1 / WeightSum);
			const double CoastDerivative = 2 * ScaledHeight / (CoastTransitionMeters * Denominator * Denominator);
			Result.TangentialGradientPerRadian = Original.TangentialGradientPerRadian * Ratio +
			(LogGradient * CoastMask + Original.TangentialGradientPerRadian * (LogRatio * CoastDerivative)) * (Height * Ratio);
		}
		// 解析的な侵食地形の返戻
		return Result;
	}

	/**
	 * 標高と法線の元に為る勾配を同じ式で評価する
	 * @param Direction 惑星上の方向
	 * @return 連続な地形の標高と接線勾配
	 */
	TerrainSample PlanetErodedTerrain::Evaluate(const UnitDirection Direction) const {
		// 解析勾配付きの地形の返戻
		return EvaluateImpl<true>(Direction, true);
	}

	/**
	 * 勾配の演算を省いて標高だけを評価する
	 * @param Direction 惑星上の方向
	 * @return 同じ式の標高
	 */
	double PlanetErodedTerrain::EvaluateElevation(const UnitDirection Direction) const {
		// 標高の返戻
		return EvaluateImpl<false>(Direction, true).ElevationMeters;
	}

	/**
	 * 索引を使わず全地点を確認して地形を評価する
	 * @param Direction 惑星上の方向
	 * @return 索引の取りこぼしを検査する独立な検索結果
	 */
	TerrainSample PlanetErodedTerrain::EvaluateReference(const UnitDirection Direction) const {
		// 全点探索の地形の返戻
		return EvaluateImpl<true>(Direction, false);
	}
}
