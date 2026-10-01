#include "PlanetFlowRoute.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <vector>

namespace PlanetCore {
	namespace {
		/**
		 * 空間索引の分割に使う成分を読む
		 * @param V 座標
		 * @param Axis ０：X，１：Y，２：Z
		 * @return 指定した成分
		 */
		double Component(const Vec3 V, const unsigned Axis) {
			// 指定軸の成分の返戻
			return !Axis ? V.X : Axis == 1 ? V.Y : V.Z;
		}

		/**
		 * 照会の幅で拡張した包囲箱との交差を調べる
		 * @param D 単位球面の照会点
		 * @param Low 箱の最小座標
		 * @param High 箱の最大座標
		 * @param Margin 全軸に加える保守的な幅
		 * @return 拡張箱に含まれる場合に真
		 */
		bool Contains(const Vec3 D, const Vec3 Low, const Vec3 High, const double Margin) {
			// 三軸全ての包含判定の返戻
			return D.X >= Low.X - Margin && D.Y >= Low.Y - Margin && D.Z >= Low.Z - Margin && D.X <= High.X + Margin &&
			D.Y <= High.Y + Margin && D.Z <= High.Z + Margin;
		}
	}

	/**
	 * 元点列の接線から曲率を共有し，表示解像度に依らない区間と索引を準備する
	 * @param Knots ２〜８１９２点の位置・下流接線・厳密増加する元経路距離
	 * @param Radius 参照球面の半径
	 * @param Diagnostics 拒否理由と元区間の実寸法，省略可
	 * @return 準備済の経路；不正点列・曲率予算超過は空
	 */
	std::optional<TerrainFlowRoute> TerrainFlowRoute::Create(
		const std::span<const FlowRouteKnot> Knots,
		const double Radius,
		FlowRouteDiagnostics *const Diagnostics
	) {
		FlowRouteDiagnostics Local;
		FlowRouteDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const FlowRouteFailure Failure) -> std::optional<TerrainFlowRoute> {
			Report.Failure = Failure;
			// 不完全な区間群を公開しない返戻
			return {};
		};
		const bool Valid = Knots.size() > 1 && Knots.size() < 8193 && std::isfinite(Radius) && Radius >= 1000 && Radius <= 1E9;
		// 有界な点列と有効な惑星尺度以外への返戻
		if(!Valid) return Reject(FlowRouteFailure::InvalidInput);
		TerrainFlowRoute Route;
		Route.RadiusMeters = Radius;
		std::vector<Vec3> Tangents, Curvatures(Knots.size());
		Tangents.reserve(Knots.size());
		for(std::size_t i = 0; i < Knots.size(); ++i) {
			const FlowRouteKnot &Knot = Knots[i];
			Report.Index = static_cast<std::uint32_t>(i);
			Report.DistanceDifferenceMeters = i ? Knot.DistanceMeters - Knots[i - 1].DistanceMeters : 0;
			const bool HasDistance = std::isfinite(Knot.DistanceMeters) && (!i || Knot.DistanceMeters - Knots[i - 1].DistanceMeters > 1E-6);
			// 非有限又は退化した基準距離を曲率推定へ渡さない返戻
			if(!HasDistance) return Reject(FlowRouteFailure::Distance);
			const Vec3 D = Knot.Direction.Vector();
			const std::optional<UnitDirection> Tangent = UnitDirection::FromVector(Knot.Tangent - D * Knot.Tangent.Dot(D));
			// 有効でない下降方向への返戻
			if(!Tangent) return Reject(FlowRouteFailure::Tangent);
			Tangents.push_back(Tangent->Vector());
		}
		for(std::size_t i = 0; i < Knots.size(); ++i) {
			const double Before = i ? Knots[i].DistanceMeters - Knots[i - 1].DistanceMeters : 0;
			const double After = i + 1 < Knots.size() ? Knots[i + 1].DistanceMeters - Knots[i].DistanceMeters : 0;
			if(!i) Curvatures[i] = (Tangents[i + 1] - Tangents[i]) * (1 / After);
			else if(i + 1 == Knots.size()) Curvatures[i] = (Tangents[i] - Tangents[i - 1]) * (1 / Before);
			else {
				Curvatures[i] = (Tangents[i] - Tangents[i - 1]) * (After / (Before * (Before + After))) +
				(Tangents[i + 1] - Tangents[i]) * (Before / (After * (Before + After)));
			}
		}
		Route.Segments.reserve(Knots.size() - 1);
		Route.Starts.reserve(Knots.size() - 1);
		for(std::size_t i = 1; i < Knots.size(); ++i) {
			const std::optional<TerrainFlowSegment> Curve = TerrainFlowSegment::CreateWithCurvature(
				Knots[i - 1].Direction,
				Knots[i].Direction,
				Tangents[i - 1],
				Tangents[i],
				Curvatures[i - 1],
				Curvatures[i],
				Radius
			);
			// 一区間でも構成出来ない経路を部分公開しない返戻
			if(!Curve) {
				Report.Index = static_cast<std::uint32_t>(i - 1);
				Report.DistanceDifferenceMeters = Knots[i].DistanceMeters - Knots[i - 1].DistanceMeters;
				Report.Directions = { Knots[i - 1].Direction.Vector(), Knots[i].Direction.Vector() };
				const Vec3 Delta = Report.Directions[1] - Report.Directions[0];
				Report.ChordMeters =
				Radius * std::atan2(Report.Directions[0].Cross(Report.Directions[1]).Length(), Report.Directions[0].Dot(Report.Directions[1]));
				for(unsigned Side = 0; Side < 2; ++Side) {
					const Vec3 Normal = Report.Directions[Side], Tangent = Tangents[i - 1 + Side];
					const Vec3 Curvature = Curvatures[i - 1 + Side];
					const Vec3 Geodesic = Curvature - Normal * Curvature.Dot(Normal) - Tangent * Curvature.Dot(Tangent);
					Report.ForwardProducts[Side] = Tangent.Dot(Delta);
					Report.CurvatureProducts[Side] = Geodesic.Length() * Report.ChordMeters;
				}
				// 元の区間制約と同じ位置・接線・曲率を記録した拒否の返戻
				return Reject(FlowRouteFailure::Segment);
			}
			Route.Starts.push_back(Route.LengthMeters);
			Route.Segments.push_back(*Curve);
			Route.LengthMeters += Curve->GetParameterLengthMeters();
		}
		Route.Order.resize(Route.Segments.size());
		std::iota(Route.Order.begin(), Route.Order.end(), 0U);
		Route.Index.reserve(Route.Segments.size() * 2);
		Route.BuildIndex(0, static_cast<std::uint32_t>(Route.Segments.size()));
		Report = {};
		// 不変な経路と索引の返戻
		return Route;
	}

	/**
	 * 両端の曲率予算から保守的に包囲した区間群を二分する
	 * 五次 Bézier 制御点は始点から 1.451θ 以内，正規化後は 3θ 以内に在る
	 * @param Begin 索引順序の先頭
	 * @param Count 区間数
	 * @return 構築した節点番号
	 */
	std::uint32_t TerrainFlowRoute::BuildIndex(const std::uint32_t Begin, const std::uint32_t Count) {
		const std::uint32_t NodeIndex = static_cast<std::uint32_t>(Index.size());
		Index.push_back({});
		Vec3 Low { 2, 2, 2 }, High { -2, -2, -2 };
		for(std::uint32_t i = Begin; i < Begin + Count; ++i) {
			const TerrainFlowSegment &Curve = Segments[Order[i]];
			const Vec3 A = Curve.Evaluate(0)->Direction.Vector();
			const double Bound = 3 * Curve.GetParameterLengthMeters() / RadiusMeters;
			Low = { std::min(Low.X, A.X - Bound), std::min(Low.Y, A.Y - Bound), std::min(Low.Z, A.Z - Bound) };
			High = { std::max(High.X, A.X + Bound), std::max(High.Y, A.Y + Bound), std::max(High.Z, A.Z + Bound) };
		}
		Index[NodeIndex] = { Low, High, Begin, Count, 0, 0 };
		// 葉に入る少数の区間を其の儘保持した返戻
		if(Count < 9) return NodeIndex;
		const Vec3 Extent = High - Low;
		const unsigned Axis = Extent.X > Extent.Y ? Extent.X > Extent.Z ? 0 : 2 : Extent.Y > Extent.Z ? 1 : 2;
		const std::uint32_t LeftCount = Count / 2;
		std::nth_element(
			Order.begin() + Begin,
			Order.begin() + Begin + LeftCount,
			Order.begin() + Begin + Count,
			[&](const std::uint32_t A, const std::uint32_t B) -> bool {
				const double X = Component(Segments[A].Evaluate(0)->Direction.Vector(), Axis);
				const double Y = Component(Segments[B].Evaluate(0)->Direction.Vector(), Axis);
				// 同じ位置でも区間番号で順序を固定した比較結果の返戻
				return X < Y || X == Y && A < B;
			}
		);
		const std::uint32_t Left = BuildIndex(Begin, LeftCount), Right = BuildIndex(Begin + LeftCount, Count - LeftCount);
		Index[NodeIndex].Count = 0;
		Index[NodeIndex].Left = Left;
		Index[NodeIndex].Right = Right;
		// 完成した内部節点の返戻
		return NodeIndex;
	}

	/**
	 * 各区間の公称距離を累積した座標で経路を評価する
	 * @param DistanceMeters 経路の先頭を０とする公称距離
	 * @return 球面位置と解析微分；範囲外は空
	 */
	std::optional<TerrainFlowCurveSample> TerrainFlowRoute::Sample(const double DistanceMeters) const {
		// 範囲外の距離を黙って端点へ丸めない返戻
		if(!std::isfinite(DistanceMeters) || DistanceMeters < 0 || DistanceMeters > LengthMeters) return {};
		const std::size_t Offset =
		static_cast<std::size_t>(std::upper_bound(Starts.begin(), Starts.end(), DistanceMeters) - Starts.begin() - 1);
		const double Fraction = std::clamp((DistanceMeters - Starts[Offset]) / Segments[Offset].GetParameterLengthMeters(), 0.0, 1.0);
		// 一区間の解析評価の返戻
		return Segments[Offset].Evaluate(Fraction);
	}

	/**
	 * 公称区間長の累積を読む
	 * @return 厳密な曲線弧長とは区別した経路長
	 */
	double TerrainFlowRoute::GetLengthMeters() const {
		// 公称経路長の返戻
		return LengthMeters;
	}

	/**
	 * 独立な全区間比較用の読取専用区間列を公開する
	 * @return 経路の生存中だけ有効な参照
	 */
	std::span<const TerrainFlowSegment> TerrainFlowRoute::GetSegments() const {
		// 不変区間列の返戻
		return Segments;
	}

	/**
	 * 保守的な索引で候補を絞り，局所直交点の距離を比較する
	 * 区間内に複数の極小が有る場合の全解探索は保証しない
	 * @param Query 照会方向
	 * @param MaximumDistanceMeters 対象とする管状近傍の幅
	 * @param Candidates 実際に直交点を解いた区間数の出力先，省略可
	 * @param IncludeEndpoints 経路端も比較し，端の外では公称距離を固定する場合に真
	 * @return 候補内の最小距離の点；該当無し又は無効幅は空
	 */
	std::optional<FlowRouteProjection> TerrainFlowRoute::FindFootpoint(
		const UnitDirection Query,
		const double MaximumDistanceMeters,
		std::uint32_t *const Candidates,
		const bool IncludeEndpoints
	) const {
		if(Candidates) *Candidates = 0;
		const bool HasValidWidth =
		std::isfinite(MaximumDistanceMeters) && MaximumDistanceMeters > 0 && MaximumDistanceMeters < RadiusMeters * 0.05;
		// 有限の局所幅以外への返戻
		if(!HasValidWidth) return {};
		const double Margin = MaximumDistanceMeters / RadiusMeters + 64 * std::numeric_limits<double>::epsilon();
		const double Chord = 2 * std::sin(MaximumDistanceMeters / (2 * RadiusMeters)), Limit = Chord * Chord;
		const Vec3 D = Query.Vector();
		std::optional<FlowRouteProjection> Best;
		if(IncludeEndpoints) for(unsigned Side = 0; Side < 2; ++Side) {
			const std::uint32_t Id = Side ? static_cast<std::uint32_t>(Segments.size() - 1) : 0;
			const double Fraction = Side ? 1.0 : 0.0;
			const TerrainFlowCurveSample Point = *Segments[Id].Evaluate(Fraction);
			const Vec3 Difference = D - Point.Direction.Vector();
			const double DistanceSquared = Difference.Dot(Difference), Along = Difference.Dot(Point.DirectionDerivativePerMeter);
			// 内側へ進むと距離が減る端は，制約付きの最近点ではない
			if(Side ? Along < 0 : Along > 0) continue;
			if(DistanceSquared > Limit || Best && !(DistanceSquared < Best->Footpoint.SquaredChordDistance)) continue;
			const Vec3 DistanceGradient = (Difference - D * Difference.Dot(D)) * 2;
			const TerrainFlowFootpoint Foot { Point, Fraction, DistanceSquared, DistanceGradient, {} };
			Best = FlowRouteProjection{ Foot, Side ? LengthMeters : 0.0, Id };
		}
		// 最大８１９１区間の平衡木なので，同時待機する兄弟節点は１４個以下
		std::array<std::uint32_t, 64> Stack {};
		std::uint32_t Pending = 1;
		while(Pending) {
			const IndexNode &Node = Index[Stack[--Pending]];
			// 既知の最近点より外側の箱を除く
			// 方向座標の丸め余裕は元の探索と同じ値を維持する
			const double SearchMargin =
			Best ? std::min(Margin, std::sqrt(Best->Footpoint.SquaredChordDistance) + 64 * std::numeric_limits<double>::epsilon()) : Margin;
			if(!Contains(D, Node.Min, Node.Max, SearchMargin)) continue;
			if(!Node.Count) {
				const auto BoxDistance = [&](const std::uint32_t Id) -> double {
					const IndexNode &Box = Index[Id];
					// 箱迄の最大成分距離は探索順だけに使い，候補の同距離規則には使わない
					return std::max({ 0.0, Box.Min.X - D.X, D.X - Box.Max.X, Box.Min.Y - D.Y, D.Y - Box.Max.Y, Box.Min.Z - D.Z, D.Z - Box.Max.Z });
				};
				const bool LeftNearer = BoxDistance(Node.Left) <= BoxDistance(Node.Right);
				Stack[Pending++] = LeftNearer ? Node.Right : Node.Left;
				Stack[Pending++] = LeftNearer ? Node.Left : Node.Right;
				continue;
			}
			for(std::uint32_t i = Node.Begin; i < Node.Begin + Node.Count; ++i) {
				const std::uint32_t Id = Order[i];
				if(Candidates) ++*Candidates;
				const std::optional<TerrainFlowFootpoint> Foot = Segments[Id].FindInteriorFootpoint(Query);
				if(!Foot || Foot->SquaredChordDistance > Limit) continue;
				if(
					Best && (
						Foot->SquaredChordDistance > Best->Footpoint.SquaredChordDistance ||
						Foot->SquaredChordDistance == Best->Footpoint.SquaredChordDistance && Id > Best->SegmentIndex
					)
				) continue;
				Best = FlowRouteProjection{ *Foot, Starts[Id] + Foot->Fraction * Segments[Id].GetParameterLengthMeters(), Id };
			}
		}
		// 検査した候補の最小値の返戻
		return Best;
	}
}
