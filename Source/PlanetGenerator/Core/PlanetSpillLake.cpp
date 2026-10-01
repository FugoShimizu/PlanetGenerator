#include "PlanetSpillLake.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace PlanetCore {
	/**
	 * 岸の辺を閉矩形でまとめ，同じ面を保つ平衡索引を作る
	 * @param Begin 並替済辺番号の先頭
	 * @param Count 対象とする辺数
	 * @return 生成した節点番号
	 */
	std::uint32_t SpillLakeShore::BuildIndex(const std::uint32_t Begin, const std::uint32_t Count) {
		const double Infinite = std::numeric_limits<double>::infinity();
		std::array<double, 2> Low { Infinite, Infinite }, High { -Infinite, -Infinite };
		for(std::uint32_t I = Begin; I < Begin + Count; ++I) {
			const std::uint32_t Edge = Order[I];
			for(const std::size_t Vertex : { static_cast<std::size_t>(Edge), (Edge + 1) % PlanarVertices.size() }) {
				for(unsigned Axis = 0; Axis < 2; ++Axis) {
					Low[Axis] = std::min(Low[Axis], PlanarVertices[Vertex][Axis]);
					High[Axis] = std::max(High[Axis], PlanarVertices[Vertex][Axis]);
				}
			}
		}
		const std::uint32_t Id = static_cast<std::uint32_t>(Index.size());
		Index.push_back({ Low, High, Begin, Count, 0, 0 });
		// 少数の辺を保持する葉の返戻
		if(Count < 9) return Id;
		const unsigned Axis = High[0] - Low[0] > High[1] - Low[1] ? 0 : 1;
		const std::uint32_t LeftCount = Count / 2;
		std::nth_element(
			Order.begin() + Begin,
			Order.begin() + Begin + LeftCount,
			Order.begin() + Begin + Count,
			[&](const std::uint32_t A, const std::uint32_t B) -> bool {
				const double X = PlanarVertices[A][Axis] + PlanarVertices[(A + 1) % PlanarVertices.size()][Axis];
				const double Y = PlanarVertices[B][Axis] + PlanarVertices[(B + 1) % PlanarVertices.size()][Axis];
				// 同位置でも辺番号で固定する比較の返戻
				return X < Y || X == Y && A < B;
			}
		);
		const std::uint32_t Left = BuildIndex(Begin, LeftCount), Right = BuildIndex(Begin + LeftCount, Count - LeftCount);
		Index[Id].Count = 0;
		Index[Id].Left = Left;
		Index[Id].Right = Right;
		// 完成した内部節点の返戻
		return Id;
	}

	/**
	 * 追跡精度と元の等高点を読取専用で取得する
	 * @return 同じ不変な輪郭
	 */
	const TerrainContour &SpillLakeShore::GetContour() const {
		return Contour;
	}

	/**
	 * 独立な面積・距離検証に使う射影済の輪郭を取得する
	 * @return 閉鎖点を重複しない読取専用座標列
	 */
	std::span<const std::array<double, 2>> SpillLakeShore::GetPlanarVertices() const {
		return PlanarVertices;
	}

	/**
	 * 保守的な矩形索引で，所属判定の半直線と最短距離の候補を共に探索する
	 * @param Direction 同じ惑星の照会方向
	 * @param TestedEdges 実際に計算した辺数の出力先，省略可
	 * @return 内側が正の接平面距離；反対の半球は負の無限大
	 */
	double SpillLakeShore::SignedPlanarDistanceMeters(const UnitDirection Direction, std::uint32_t *const TestedEdges) const {
		if(TestedEdges) *TestedEdges = 0;
		const Vec3 D = Direction.Vector();
		const double Z = D.Dot(Origin);
		// 反対の半球又は未準備の岸への返戻
		if(!(Z > 0) || Index.empty()) return -std::numeric_limits<double>::infinity();
		const double X = RadiusMeters * D.Dot(East) / Z, Y = RadiusMeters * D.Dot(North) / Z;
		const auto BoundSquared = [&](const IndexNode &Node) -> double {
			const double DX = std::max({ Node.Min[0] - X, 0.0, X - Node.Max[0] }), DY = std::max({ Node.Min[1] - Y, 0.0, Y - Node.Max[1] });
			// 矩形への最短距離の下界を返す
			return DX * DX + DY * DY;
		};
		double MinimumSquared = std::numeric_limits<double>::infinity();
		bool Inside = false;
		// 最大６５５３６辺の平衡木なので，同時待機数は１７個以下
		std::array<std::uint32_t, 64> Stack {};
		std::uint32_t Pending = 1;
		while(Pending) {
			const IndexNode &Node = Index[Stack[--Pending]];
			const double Bound = BoundSquared(Node), Margin = 64 * std::numeric_limits<double>::epsilon() * std::max(1.0, MinimumSquared);
			const bool HasRay = Node.Min[1] <= Y && Y <= Node.Max[1] && X <= Node.Max[0];
			if(!HasRay && Bound > MinimumSquared + Margin) continue;
			if(!Node.Count) {
				// 近い子を先に調べ，距離候補を早期に絞る；偶奇判定の候補は残す
				const bool IsLeftNearer = BoundSquared(Index[Node.Left]) <= BoundSquared(Index[Node.Right]);
				Stack[Pending++] = IsLeftNearer ? Node.Right : Node.Left;
				Stack[Pending++] = IsLeftNearer ? Node.Left : Node.Right;
				continue;
			}
			for(std::uint32_t I = Node.Begin; I < Node.Begin + Node.Count; ++I) {
				if(TestedEdges) ++*TestedEdges;
				const std::uint32_t Edge = Order[I];
				const std::array<double, 2> &A = PlanarVertices[Edge], &B = PlanarVertices[(Edge + 1) % PlanarVertices.size()];
				const double Cross = (B[0] - A[0]) * (Y - A[1]) - (B[1] - A[1]) * (X - A[0]);
				const bool IsOnSegment =
				!Cross && X >= std::min(A[0], B[0]) && X <= std::max(A[0], B[0]) && Y >= std::min(A[1], B[1]) && Y <= std::max(A[1], B[1]);
				// 実際の折線上への返戻
				if(IsOnSegment) return 0;
				const double EX = B[0] - A[0], EY = B[1] - A[1], Squared = EX * EX + EY * EY;
				const double T = std::clamp(((X - A[0]) * EX + (Y - A[1]) * EY) / Squared, 0.0, 1.0), QX = X - A[0] - T * EX;
				const double QY = Y - A[1] - T * EY;
				MinimumSquared = std::min(MinimumSquared, QX * QX + QY * QY);
				if(A[1] > Y != B[1] > Y && X < A[0] + (Y - A[1]) * (B[0] - A[0]) / (B[1] - A[1])) Inside ^= true;
			}
		}
		// 元の折線に対する偶奇判定と最短距離の返戻
		return std::sqrt(MinimumSquared) * (Inside ? 1.0 : -1.0);
	}

	/**
	 * 準備済の単純な岸に対し，半球内の点の所属を判定する
	 * @param Direction 同じ惑星の照会方向
	 * @return 湖側又は折線境界上なら真；反対の半球は偽
	 */
	bool SpillLakeShore::Contains(const UnitDirection Direction) const {
		return SignedPlanarDistanceMeters(Direction) >= 0;
	}

	/**
	 * 鞍点の片側から，同じ水位の指定終点まで元地形の岸を辿る
	 * @param Pass 同じ元場から求めた非退化な開始鞍点
	 * @param IsPositiveLakeBranch 最小曲率方向の正側を湖とする場合に真
	 * @param Source 元地形の標高と解析勾配
	 * @param Settings 半径と岸の精度；水位は鞍点に一致，終点の省略時は同じ鞍点へ帰還
	 * @param Stop 中断要求
	 * @param Diagnostics 拒否段階と実際の追跡結果，省略可
	 * @return 始端の鞍点を含む有向の岸の弧；未到達・予算超過・不整合時は空
	 */
	std::optional<TerrainContour> TraceSpillLakeArc(
		const TerrainCriticalPoint &Pass,
		const bool IsPositiveLakeBranch,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainContourSettings &Settings,
		const std::stop_token Stop,
		SpillLakeDiagnostics *const Diagnostics
	) {
		SpillLakeDiagnostics Local;
		SpillLakeDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const SpillLakeFailure Failure) -> std::optional<TerrainContour> {
			Report.Failure = Stop.stop_requested() ? SpillLakeFailure::Cancelled : Failure;
			// 不完全な岸を公開しない返戻
			return {};
		};
		const bool HasSaddle = Pass.Kind == TerrainCriticalKind::Saddle && std::isfinite(Pass.ElevationMeters) &&
		std::isfinite(Pass.MinimumCurvaturePerMeter) && std::isfinite(Pass.MaximumCurvaturePerMeter) &&
		Pass.MinimumCurvaturePerMeter < 0 && Pass.MaximumCurvaturePerMeter > 0;
		// 片側の湖岸を定義出来ない入力への返戻
		if(!Source || !Settings.IsValid() || !HasSaddle || Stop.stop_requested()) return Reject(SpillLakeFailure::InvalidInput);
		const std::optional<UnitDirection> Axis = UnitDirection::FromVector(
			Pass.MinimumCurvatureDirection - Pass.Direction.Vector() * Pass.MinimumCurvatureDirection.Dot(Pass.Direction.Vector())
		);
		// 鞍点の接平面で有効な主方向が無い場合の返戻
		if(!Axis) return Reject(SpillLakeFailure::Direction);
		const Vec3 Along = Axis->Vector() * (IsPositiveLakeBranch ? 1.0 : -1.0);
		const Vec3 Across = Pass.Direction.Vector().Cross(Along);
		const double Ratio = std::sqrt(-Pass.MinimumCurvaturePerMeter / Pass.MaximumCurvaturePerMeter);
		const std::optional<UnitDirection> Arm = UnitDirection::FromVector(Along - Across * Ratio);
		// 曲率の演算範囲を超える枝への返戻
		if(!Arm) return Reject(SpillLakeFailure::Direction);
		TerrainContourSettings Trace = Settings;
		Trace.LevelMeters = Pass.ElevationMeters;
		if(Trace.Terminals.empty()) Trace.Terminals.push_back(Pass.Direction);
		Trace.TerminalRadiusMeters = Settings.ChordToleranceMeters;
		// 鞍点から離した正則な始点を，小さい勾配だけで停留点と扱わない
		Trace.GradientTolerance = std::min(
			Settings.GradientTolerance,
			std::min(-Pass.MinimumCurvaturePerMeter, Pass.MaximumCurvaturePerMeter) * Trace.TerminalRadiusMeters * 0.125
		);
		// 始点に追加する鞍点１個も，等高線の全保持上限へ含める
		Trace.MaximumVertices = Settings.MaximumVertices - 1;
		const double InitialAngle = Trace.TerminalRadiusMeters * 0.5 / Settings.RadiusMeters;
		const UnitDirection Initial =
		*UnitDirection::FromVector(Pass.Direction.Vector() * std::cos(InitialAngle) + Arm->Vector() * std::sin(InitialAngle));
		std::optional<TerrainContour> Contour = TraceTerrainContour(Initial, Source, Trace, Stop);
		if(Contour) {
			Report.End = Contour->End;
			Report.Vertices = static_cast<std::uint32_t>(Contour->Samples.size());
			Report.HeightResidualMeters = Contour->MaximumHeightResidualMeters;
			if(!Contour->Samples.empty()) {
				Report.LengthMeters = Contour->Samples.back().DistanceMeters;
				Report.LastDirection = Contour->Samples.back().Direction.Vector();
			}
		}
		const bool HasArrived = Contour && Contour->End == TerrainContourEnd::Terminal && Contour->Samples.size() > 3;
		// 始点への小さな閉鎖や追跡途中を，指定終点へ繋がる岸の弧として扱わない返戻
		if(!HasArrived) return Reject(SpillLakeFailure::Contour);
		const Vec3 First = Contour->Samples.front().Direction.Vector(), Origin = Pass.Direction.Vector();
		const double FirstLength = Settings.RadiusMeters * std::atan2(First.Cross(Origin).Length(), First.Dot(Origin));
		Report.FirstGapMeters = FirstLength;
		// 開始側にも同じ短い接続の精度を要求する返戻
		if(FirstLength > Trace.TerminalRadiusMeters) return Reject(SpillLakeFailure::StartGap);
		for(TerrainContourSample &Point : Contour->Samples) Point.DistanceMeters += FirstLength;
		Contour->Samples.insert(Contour->Samples.begin(), { Pass.Direction, Pass.ElevationMeters, 0 });
		Report.LengthMeters = Contour->Samples.back().DistanceMeters;
		Report.Vertices = static_cast<std::uint32_t>(Contour->Samples.size());
		if(Report.LengthMeters > Settings.MaximumLengthMeters) {
			Report.End = TerrainContourEnd::LengthLimit;
			// 開始側の短い接続も全経路の予算へ含める返戻
			return Reject(SpillLakeFailure::Contour);
		}
		// 到達番号と両端の実位置を保持した，閉水面とは区別する岸の返戻
		return Contour;
	}

	/**
	 * 鞍点の同じ水位を持つ片側の岸を辿り，元地形を変更せず所属領域を準備する
	 * @param Pass 同じ元場から求めた非退化な流出口
	 * @param IsPositiveLakeBranch 最小曲率方向の正側を湖とする場合に真
	 * @param Source 元地形の標高と解析勾配
	 * @param Settings 半径と岸の追跡精度；水位は流出口と一致させる
	 * @param Stop 中断要求
	 * @param Diagnostics 拒否段階と実際の追跡結果，省略可
	 * @return 流出口へ帰還した単純な岸；他の停留点・予算超過・不整合は空
	 */
	std::optional<SpillLakeShore> PrepareSpillLakeShore(
		const TerrainCriticalPoint &Pass,
		const bool IsPositiveLakeBranch,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainContourSettings &Settings,
		const std::stop_token Stop,
		SpillLakeDiagnostics *const Diagnostics
	) {
		SpillLakeDiagnostics Local;
		SpillLakeDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const SpillLakeFailure Failure) -> std::optional<SpillLakeShore> {
			Report.Failure = Stop.stop_requested() ? SpillLakeFailure::Cancelled : Failure;
			// 不完全な岸を公開しない返戻
			return {};
		};
		TerrainContourSettings ArcSettings = Settings;
		ArcSettings.Terminals = { Pass.Direction };
		ArcSettings.TerminalRadiusMeters = Settings.ChordToleranceMeters;
		std::optional<TerrainContour> Contour = TraceSpillLakeArc(Pass, IsPositiveLakeBranch, Source, ArcSettings, Stop, &Report);
		// 片側の鞍点へ帰還しない経路を，単一の湖岸へ渡さない返戻
		if(!Contour) return {};
		const Vec3 Origin = Pass.Direction.Vector();
		const Vec3 Axis =
		UnitDirection::FromVector(Pass.MinimumCurvatureDirection - Origin * Pass.MinimumCurvatureDirection.Dot(Origin))->Vector();
		const Vec3 Along = Axis * (IsPositiveLakeBranch ? 1.0 : -1.0), Across = Origin.Cross(Along);
		Contour->End = TerrainContourEnd::Closed;
		Contour->TerminalIndex.reset();
		SpillLakeShore Shore;
		Shore.Contour = std::move(*Contour);
		Shore.Origin = Origin;
		Shore.East = Along;
		Shore.North = Across;
		Shore.RadiusMeters = Settings.RadiusMeters;
		for(std::size_t i = 0; i + 1 < Shore.Contour.Samples.size(); ++i) {
			const Vec3 D = Shore.Contour.Samples[i].Direction.Vector();
			const double Z = D.Dot(Origin);
			// 一つの局所射影に収まらない岸への返戻
			if(!(Z > 0)) return Reject(SpillLakeFailure::Projection);
			Shore.PlanarVertices.push_back({ Settings.RadiusMeters * D.Dot(Along) / Z, Settings.RadiusMeters * D.Dot(Across) / Z });
		}
		const std::optional<double> Area = MeasureTerrainContourArea(Shore.Contour, Settings.RadiusMeters);
		Report.AreaMetersSquared = Area.value_or(0);
		const bool HasSimpleBoundary = Area && *Area > 0 && IsSimplePlanarContour(Shore.PlanarVertices);
		// 自交差又は反対側を囲む経路を湖の所属へ使わない返戻
		if(!HasSimpleBoundary || Stop.stop_requested()) return Reject(SpillLakeFailure::Boundary);
		Shore.Order.resize(Shore.PlanarVertices.size());
		std::iota(Shore.Order.begin(), Shore.Order.end(), 0U);
		Shore.Index.reserve(Shore.PlanarVertices.size() / 2);
		Shore.BuildIndex(0, static_cast<std::uint32_t>(Shore.Order.size()));
		// 索引準備中の中断を公開しない返戻
		if(Stop.stop_requested()) return Reject(SpillLakeFailure::Cancelled);
		// 視点に依存せず元地形と流出口だけで決まる岸の返戻
		return Shore;
	}
}
