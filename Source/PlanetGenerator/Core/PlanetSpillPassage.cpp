#include "PlanetSpillPassage.h"
#include "PlanetBilinearCritical.h"
#include "PlanetSpillGraph.h"
#include "PlanetSpillPath.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		// 両端通路と外周窓で，同じ実辺確認の責務を共有する解法
		using PathSolver = std::optional<MeasuredSpillPath>(*)(
			std::span<const SpillGraphSite>,
			std::span<const SpillGraphEdge>,
			std::uint32_t,
			std::uint32_t,
			const std::function<std::optional<double>(std::uint32_t)> &,
			std::stop_token
		);

		/**
		 * 物理間隔の標本と接線の符号反転を用い，選択通路の最高標高を測る
		 * 計算量：O(L / d + 24 K)，L は通路長，d は標本間隔，K は検出した極大数
		 * @param Source 変更しない元地形
		 * @param Start 通路の始点
		 * @param End 通路の終点
		 * @param StartValue 準備済の始点の元標高と勾配
		 * @param EndValue 準備済の終点の元標高と勾配
		 * @param Radius 惑星半径
		 * @param MaximumStepMeters 弧上の最大標本間隔
		 * @param Stop 中断要求
		 * @param PeakDirection 最高標高の位置を受取る場合の参照先，省略時は位置を返さない
		 * @return 測った最高標高，有限標本間の極値が全て検出された事の証明ではない
		 */
		std::optional<double> MeasureSpillPassageCrest(
			const std::function<TerrainSample(UnitDirection)> &Source,
			const UnitDirection Start,
			const UnitDirection End,
			const TerrainSample &StartValue,
			const TerrainSample &EndValue,
			const double Radius,
			const double MaximumStepMeters,
			const std::stop_token Stop,
			UnitDirection *const PeakDirection=nullptr
		) {
			// 大円上の位置と接線の準備
			const Vec3 A = Start.Vector(), B = End.Vector();
			const double Angle = std::atan2(A.Cross(B).Length(), A.Dot(B)), Length = Angle * Radius;
			const Vec3 Tangent = UnitDirection::FromVector(B - A * A.Dot(B))->Vector();
			const double RequiredSegments = std::ceil(Length / MaximumStepMeters);
			// 整数変換前に非有限又は過大な通路標本数を拒否する返戻
			if(!std::isfinite(RequiredSegments) || RequiredSegments > 100000000) return {};
			const std::uint32_t Segments = static_cast<std::uint32_t>(RequiredSegments);
			// 最大値の位置を要求された場合の記録
			double RecordedHeight = -std::numeric_limits<double>::infinity();
			bool HasFiniteSource = true;
			// 端点と同じ連続場の標高及び弧長方向の微分
			const auto Sample = [&](const double Fraction) -> std::array<double, 2> {
				const double Phase = Angle * Fraction;
				const Vec3 Position = A * std::cos(Phase) + Tangent * std::sin(Phase);
				const Vec3 Along = Tangent * std::cos(Phase) - A * std::sin(Phase);
				const UnitDirection Direction = !Fraction ? Start : Fraction == 1 ? End : *UnitDirection::FromVector(Position);
				const TerrainSample Value = !Fraction ? StartValue : Fraction == 1 ? EndValue : Source(Direction);
				HasFiniteSource =
				HasFiniteSource && std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
				if(PeakDirection && Value.ElevationMeters > RecordedHeight) {
					RecordedHeight = Value.ElevationMeters;
					*PeakDirection = Direction;
				}
				// 弧上の標高と，微分符号判定に使う方向微分の返戻
				return { Value.ElevationMeters, Value.TangentialGradientPerRadian.Dot(Along) };
			};
			std::array<double, 2> Previous = Sample(0);
			double Crest = Previous[0];
			for(std::uint32_t i = 1; i <= Segments; ++i) {
				// 中断又は非有限な場を通路上限に混ぜない返戻
				if(Stop.stop_requested() || !HasFiniteSource) return {};
				const double Fraction = static_cast<double>(i) / Segments;
				const std::array<double, 2> Current = Sample(Fraction);
				Crest = std::max(Crest, Current[0]);
				if(Previous[1] > 0 && Current[1] < 0) {
					double Low = static_cast<double>(i - 1) / Segments, High = Fraction;
					for(unsigned Step = 0; Step < 24 && (High - Low) * Length > 1E-4; ++Step) {
						const double Middle = std::midpoint(Low, High);
						const std::array<double, 2> Value = Sample(Middle);
						Crest = std::max(Crest, Value[0]);
						if(Value[1] > 0) Low = Middle;
						else High = Middle;
					}
				}
				Previous = Current;
			}
			// 最後の標本で場を失った場合の返戻
			if(Stop.stop_requested() || !HasFiniteSource) return {};
			// 端点と検出した極大の最大値の返戻
			return Crest;
		}
	}

	/**
	 * 任意の球面折線の全辺を，同じ物理間隔と微分符号の絞込みで実測する
	 * 計算量 O(V + L / d + 24 K)，保持量 O(V)
	 * V は折線頂点数，L は全長，d は測定間隔，K は検出した極大数
	 * @param Path 絞込み前又は後の有限な折線；端点の変更又は地形の補正はしない
	 * @param Source 所有する同じ元地形の標高と解析勾配
	 * @param Settings 物理尺度・標本間隔と実照会の有限予算
	 * @param Stop 下位の辺測定へ伝播する中断要求
	 * @return 元場で測った通過高度と候補；非有限・退化辺・予算不足では空
	 */
	std::optional<SpillPathCrest> MeasureSpillPathCrest(
		const std::span<const UnitDirection> Path,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillPassageSettings &Settings,
		const std::stop_token Stop
	) {
		const double Radius = Settings.RadiusMeters, Step = Settings.MaximumSampleStepMeters;
		const bool Valid =
		Source && Path.size() > 1 && Path.size() < 65537 && std::isfinite(Radius) && Radius >= 1000 && Radius <= 1E9 &&
		std::isfinite(Step) && Step > 0 && Step <= 1000 && Settings.MaximumEvaluations > 0 && Settings.MaximumEvaluations < 100000001;
		// 無効な尺度・点数・予算又は中断を，元場の測定へ渡さない返戻
		if(!Valid || Stop.stop_requested()) return {};
		SpillPathCrest Result { Path.front() };
		Result.CrestMeters = -std::numeric_limits<double>::infinity();
		bool HasSource = true, HasBudget = true;
		const auto Sample = [&](const UnitDirection D) -> TerrainSample {
			if(Stop.stop_requested() || Result.Evaluations >= Settings.MaximumEvaluations) {
				HasBudget = false;
				const double Invalid = std::numeric_limits<double>::quiet_NaN();
				// 予算超過後に元場を呼ばない返戻
				return { Invalid, { Invalid, Invalid, Invalid } };
			}
			++Result.Evaluations;
			const TerrainSample Value = Source(D);
			HasSource = HasSource && std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
			// 同じ元場の実標高・勾配だけを渡す返戻
			return Value;
		};
		TerrainSample Previous = Sample(Path.front());
		std::vector<double> Heights;
		std::vector<UnitDirection> Peaks;
		Heights.reserve(Path.size() - 1);
		Peaks.reserve(Path.size() - 1);
		for(std::size_t I = 1; I < Path.size(); ++I) {
			const Vec3 A = Path[I - 1].Vector(), B = Path[I].Vector();
			const double Angle = std::atan2(A.Cross(B).Length(), A.Dot(B));
			// 退化又は対蹠点の辺を，測定可能な大円弧と見做さない返戻
			if(!std::isfinite(Angle) || !(Angle > 0) || Angle >= 3.141592653589793 - 1E-6) return {};
			const TerrainSample Next = Sample(Path[I]);
			UnitDirection Peak = Path[I - 1];
			const std::optional<double> Crest =
			MeasureSpillPassageCrest(Sample, Path[I - 1], Path[I], Previous, Next, Radius, Step, Stop, &Peak);
			// 実測の欠落・中断・非有限又は全照会予算超過への返戻
			if(!Crest || !HasSource || !HasBudget || Stop.stop_requested()) return {};
			Heights.push_back(*Crest);
			Peaks.push_back(Peak);
			if(*Crest > Result.CrestMeters) {
				Result.CrestMeters = *Crest;
				Result.Peak = Peak;
			}
			Result.LengthMeters += Radius * Angle;
			Previous = Next;
		}
		for(std::size_t I = 0; I < Heights.size(); ++I) {
			if(I && Heights[I] <= Heights[I - 1]) continue;
			if(I + 1 < Heights.size() && Heights[I] < Heights[I + 1]) continue;
			Result.PassCandidates.push_back(Peaks[I]);
		}
		for(std::size_t I = 1; I < Path.size(); ++I) {
			const Vec3 A = Path[I - 1].Vector(), B = Path[I].Vector();
			const double Length = Radius * std::atan2(A.Cross(B).Length(), A.Dot(B));
			const unsigned Probes = std::max(1U, static_cast<unsigned>(std::ceil(Length / 10)));
			for(unsigned Probe = 0; Probe < Probes; ++Probe) {
				const double Fraction = (Probe + 0.5) / Probes;
				const UnitDirection D = *UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
				const double Height = Sample(D).ElevationMeters;
				// 辺測定と独立な標本でも，非有限・中断・予算超過を拒否する返戻
				if(!HasSource || !HasBudget || Stop.stop_requested()) return {};
				Result.MaximumIndependentExcessMeters = std::max(Result.MaximumIndependentExcessMeters, Height - Result.CrestMeters);
			}
		}
		// 有限標本の証跡を返し，全極値又は最小経路の証明へ置換えない
		return Result;
	}

	/**
	 * 元の辺の周囲で小窪地の接続を調べ，標本網内の最低通過標高を求める
	 * 計算量：O((N + E) log E + M L / d)；最悪の場合は全辺を実測する
	 * N は局所頂点数，E は全辺数，M は実測辺数，L は平均辺長，d は標本間隔
	 * @param Start 元の通路の一端
	 * @param End 元の通路の他端
	 * @param Source 変更しない解析地形
	 * @param Settings 半径，物理間隔，横幅と計算予算
	 * @param Stop 中断要求
	 * @return 有限標本内の通過標高と独立な通路残差，予算超過又は解なしは空
	 */
	std::optional<SpillPassage> FindSpillPassage(
		const UnitDirection Start,
		const UnitDirection End,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillPassageSettings &Settings,
		const std::stop_token Stop
	) {
		// 探索範囲と有限な予算の検査
		const double Radius = Settings.RadiusMeters, HalfWidthMeters = Settings.HalfWidthMeters;
		double SpacingMeters = Settings.SpacingMeters;
		const bool HasValidSettings =
		Source && std::isfinite(Radius) && !(Radius < 1000) && Radius <= 1E9 && std::isfinite(SpacingMeters) && SpacingMeters > 0 &&
		std::isfinite(HalfWidthMeters) && HalfWidthMeters >= SpacingMeters && HalfWidthMeters < Radius * 4 &&
		std::isfinite(Settings.MaximumSampleStepMeters) && Settings.MaximumSampleStepMeters >= 0.01 &&
		!(Settings.MaximumSampleStepMeters > 50) && Settings.MaximumNodes > 10 && Settings.MaximumNodes < 1000001 &&
		Settings.MaximumEvaluations > 0 && Settings.MaximumEvaluations < 100000001;
		// 不正な場・尺度・予算又は中断時の返戻
		if(!HasValidSettings || Stop.stop_requested()) return {};
		// 退化しない端点配置の確認
		const double Separation = Start.Vector().Dot(End.Vector());
		// 同一又は反対側の端点で退化する通路を作らない返戻
		if(!Start.Vector().Cross(End.Vector()).Length() || Separation < 0.5) return {};
		// 元の辺を軸とした球面の局所座標
		const Vec3 A = Start.Vector(), B = End.Vector(), Up = UnitDirection::FromVector(A + B)->Vector();
		const Vec3 Along = UnitDirection::FromVector(B - A)->Vector(), Across = Up.Cross(Along);
		// 全ての標本間隔で同じ物理範囲を使う為の固定外縁
		const double EndpointX = Radius * B.Dot(Along) / B.Dot(Up);
		const double HalfLengthMeters = std::ceil((EndpointX + HalfWidthMeters * 0.5) / 500) * 500;
		// 長い候補通路を元場照会前に捨てず，同じ有限な保持枠へ収める
		// 二進拡大は最大３２回；候補の疎密だけを変え，選択辺は下位で同じ精度で実測する
		for(unsigned Attempt = 0; Attempt < 32; ++Attempt) {
			const double Columns = std::ceil(HalfLengthMeters / SpacingMeters), Rows = std::ceil(HalfWidthMeters / SpacingMeters);
			const double Required = (2 * Columns + 1) * (2 * Rows + 1) + 2;
			if(std::isfinite(Required) && Required <= Settings.MaximumNodes) break;
			// 比較用の固定格子，又は横幅迄拡げても保持不能な通路への返戻
			if(!Settings.AdaptCandidateSpacingToBudget || SpacingMeters >= HalfWidthMeters) return {};
			SpacingMeters = std::min(SpacingMeters * 2, HalfWidthMeters);
		}
		const double HalfColumns = std::ceil(HalfLengthMeters / SpacingMeters), HalfRows = std::ceil(HalfWidthMeters / SpacingMeters);
		// 整数変換前に過大な格子を拒否する返戻
		if(!std::isfinite(HalfColumns) || HalfColumns > 499999 || HalfRows > 499999) return {};
		const std::uint32_t HalfX = static_cast<std::uint32_t>(HalfColumns), HalfY = static_cast<std::uint32_t>(HalfRows);
		const std::uint32_t Width = 2 * HalfX + 1, Height = 2 * HalfY + 1;
		// 測定用の有界な局所網
		if(static_cast<std::uint64_t>(Width) * Height + 2 > Settings.MaximumNodes) return {};
		// 全ての準備経路の評価予算の共有
		SpillPassage Result { Start };
		Result.CandidateSpacingMeters = SpacingMeters;
		const auto SampleSource = [&](const UnitDirection Direction) -> TerrainSample {
			if(Stop.stop_requested() || Result.Evaluations >= Settings.MaximumEvaluations) {
				const double Invalid = std::numeric_limits<double>::quiet_NaN();
				// 計算予算を超えて元場を呼出さない返戻
				return { Invalid, { Invalid, Invalid, Invalid } };
			}
			++Result.Evaluations;
			// 全ての準備経路の費用を集計した元場の返戻
			return Source(Direction);
		};
		// 接続検査専用の標本網の準備
		std::vector<UnitDirection> Directions;
		std::vector<TerrainSample> Samples;
		std::vector<SpillGraphSite> Sites;
		std::vector<SpillGraphEdge> Edges;
		std::vector<UnitDirection> Peaks;
		Directions.reserve(Width * Height + 2);
		Samples.reserve(Width * Height + 2);
		Sites.reserve(Width * Height + 2);
		const std::uint32_t EndpointEdges = Settings.UseEndpointNeighborRing ? 32 : 8;
		Edges.reserve(2 * Width * Height + EndpointEdges);
		Peaks.reserve(2 * Width * Height + EndpointEdges);
		for(std::uint32_t Y = 0; Y < Height; ++Y) for(std::uint32_t X = 0; X < Width; ++X) {
			// 描画又は地形補間へ流用しない接続検査用の標本
			const double LocalX = (static_cast<double>(X) - HalfX) * SpacingMeters;
			const double LocalY = (static_cast<double>(Y) - HalfY) * SpacingMeters;
			const UnitDirection Direction = *UnitDirection::FromVector(Up + (Along * LocalX + Across * LocalY) * (1 / Radius));
			Directions.push_back(Direction);
			const TerrainSample Value = SampleSource(Direction);
			const bool IsFinite = std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
			// 非有限な標本又は途中の中断への返戻
			if(!IsFinite || Stop.stop_requested()) return {};
			Samples.push_back(Value);
			Sites.push_back({ Value.ElevationMeters, false });
		}
		// 両端は格子点へ丸めず，元の正確な方向を追加
		const std::uint32_t First = static_cast<std::uint32_t>(Directions.size()), Last = First + 1;
		for(const UnitDirection Direction : { Start, End }) {
			Directions.push_back(Direction);
			const TerrainSample Value = SampleSource(Direction);
			const bool IsFinite = std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
			// 非有限な標本又は途中の中断への返戻
			if(!IsFinite || Stop.stop_requested()) return {};
			Samples.push_back(Value);
			Sites.push_back({ Value.ElevationMeters, false });
		}
		// 同じ最小最大路を使う為の単一の診断用根，大洋の実在を主張しない
		Sites[First].IsOcean = true;
		const auto AddEdge = [&](const std::uint32_t U, const std::uint32_t V) -> void {
			// 両端の最大標高を，同じ連続辺の通過高度の下限として保持
			Edges.push_back({ { U, V }, std::max(Sites[U].ElevationMeters, Sites[V].ElevationMeters) });
			Peaks.push_back(Sites[U].ElevationMeters > Sites[V].ElevationMeters ? Directions[U] : Directions[V]);
			// 終了
			return;
		};
		for(std::uint32_t Y = 0; Y < Height; ++Y) for(std::uint32_t X = 0; X < Width; ++X) {
			// ４近傍の接続は水位だけの診断に使用
			const std::uint32_t Index = Y * Width + X;
			// 水平の候補辺を追加
			if(X + 1 < Width) AddEdge(Index, Index + 1);
			// 垂直の候補辺を追加
			if(Y + 1 < Height) AddEdge(Index, Index + Width);
		}
		for(const std::uint32_t Endpoint : { First, Last }) {
			// 元の端点を丸めず，周囲セルへ実測する候補辺で接続
			const Vec3 Direction = Directions[Endpoint].Vector();
			const double X = Radius * Direction.Dot(Along) / Direction.Dot(Up) / SpacingMeters + HalfX;
			const double Y = Radius * Direction.Dot(Across) / Direction.Dot(Up) / SpacingMeters + HalfY;
			const std::uint32_t CellX = static_cast<std::uint32_t>(std::floor(X)), CellY = static_cast<std::uint32_t>(std::floor(Y));
			// 元の四隅を保持し，指定時は格子線の両側を含む周囲セルへも実候補辺を追加する
			const std::uint32_t FirstX = Settings.UseEndpointNeighborRing && CellX ? CellX - 1 : CellX;
			const std::uint32_t FirstY = Settings.UseEndpointNeighborRing && CellY ? CellY - 1 : CellY;
			const std::uint32_t LastX = std::min(Width - 1, CellX + (Settings.UseEndpointNeighborRing ? 2U : 1U));
			const std::uint32_t LastY = std::min(Height - 1, CellY + (Settings.UseEndpointNeighborRing ? 2U : 1U));
			for(std::uint32_t GridY = FirstY; GridY <= LastY; ++GridY) {
				for(std::uint32_t GridX = FirstX; GridX <= LastX; ++GridX) AddEdge(Endpoint, GridY * Width + GridX);
			}
		}
		// 確定に必要な実辺だけを，従来と同じ標本間隔と微分符号で測る
		Result.Nodes = static_cast<std::uint32_t>(Directions.size());
		Result.TotalEdges = static_cast<std::uint32_t>(Edges.size());
		std::vector<bool> Measured(Edges.size(), false);
		const auto Measure = [&](const std::uint32_t I) -> std::optional<double> {
			SpillGraphEdge &Edge = Edges[I];
			// 同じ辺の元場照会を重ねない返戻
			if(Measured[I]) return Edge.CrestMeters;
			const std::uint32_t U = Edge.Nodes[0], V = Edge.Nodes[1];
			double Crest = Edge.CrestMeters;
			if(Directions[U].Vector().Cross(Directions[V].Vector()).Length() > 1E-14) {
				const std::optional<double> Value = MeasureSpillPassageCrest(
					SampleSource,
					Directions[U],
					Directions[V],
					Samples[U],
					Samples[V],
					Radius,
					Settings.MaximumSampleStepMeters,
					Stop,
					&Peaks[I]
				);
				// 元場・中断・共通予算を失った辺を，下限の儘確定しない返戻
				if(!Value) return {};
				Crest = std::max(Crest, *Value);
			}
			Edge.CrestMeters = Crest;
			Measured[I] = true;
			++Result.EvaluatedEdges;
			// 両端と辺内部で確認した同じ通過高度の返戻
			return Crest;
		};
		// 比較用の全辺版も，同じ元場・辺精度・予算を守る
		if(Settings.MeasureAllPassageEdges) for(std::uint32_t I = 0; I < Edges.size(); ++I) if(!Measure(I)) return {};
		const PathSolver Solve = Settings.UsePathEdgeSelection ? FindPathMeasuredSpillPath : FindMeasuredSpillPath;
		const std::optional<MeasuredSpillPath> Optimal = Solve(Sites, Edges, First, Last, Measure, Stop);
		// 未実測の候補だけでは，両端の通過高度を確定しない返戻
		if(!Optimal) return {};
		Result.CrestMeters = Optimal->CrestMeters;
		// 外周への両側の最低通過高度も実測する；未評価辺の下限を完成した値にしない
		std::vector<SpillGraphSite> BoundarySites = Sites;
		std::vector<SpillGraphEdge> BoundaryEdges = Edges;
		double BoundaryLevel = std::numeric_limits<double>::infinity();
		const std::uint32_t BoundaryRoot = Result.Nodes;
		for(std::uint32_t Y = 0; Y < Height; ++Y) for(std::uint32_t X = 0; X < Width; ++X) {
			if(X && Y && X + 1 < Width && Y + 1 < Height) continue;
			const std::uint32_t I = Y * Width + X;
			BoundaryLevel = std::min(BoundaryLevel, Sites[I].ElevationMeters);
			BoundaryEdges.push_back({ { I, BoundaryRoot }, Sites[I].ElevationMeters });
		}
		BoundarySites.push_back({ BoundaryLevel, true });
		const auto MeasureBoundary = [&](const std::uint32_t I) -> std::optional<double> {
			// 仮想外周辺に架空の物理通路を付けず，実辺だけを同じ保持枠で測る返戻
			return I < Edges.size() ? Measure(I) : std::optional<double>{ BoundaryEdges[I].CrestMeters };
		};
		for(unsigned Side = 0; Side < 2; ++Side) {
			const std::optional<MeasuredSpillPath> Boundary =
			Solve(BoundarySites, BoundaryEdges, Side ? Last : First, BoundaryRoot, MeasureBoundary, Stop);
			// 片側の外周到達を失った候補を，窓内の最低解として扱わない返戻
			if(!Boundary) return {};
			Result.BoundaryEscapeMeters[Side] = Boundary->CrestMeters;
		}
		Result.OutsideLowerBoundMeters = std::max(Result.BoundaryEscapeMeters[0], Result.BoundaryEscapeMeters[1]);
		Result.IsGraphInteriorMinimum = Result.OutsideLowerBoundMeters >= Result.CrestMeters;
		// 最低通過標高を変えずに最短路を取り，水位同率の木の遠回りを除く
		std::vector<std::vector<std::uint32_t>> Incident(Result.Nodes);
		for(std::uint32_t EdgeIndex = 0; EdgeIndex < Edges.size(); ++EdgeIndex) {
			for(const std::uint32_t Node : Edges[EdgeIndex].Nodes) Incident[Node].push_back(EdgeIndex);
		}
		using QueueEntry = std::pair<double, std::uint32_t>;
		std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> Queue;
		std::vector<double> Distances(Result.Nodes, std::numeric_limits<double>::infinity());
		std::vector<std::uint32_t> Parents(Result.Nodes, MissingSpillIndex), ParentEdges(Result.Nodes, MissingSpillIndex);
		Distances[First] = 0;
		Queue.push({ 0, First });
		while(!Queue.empty()) {
			// 経路探索中の中断への返戻
			if(Stop.stop_requested()) return {};
			// 確定済の距離より古い候補は再処理しない
			const QueueEntry Entry = Queue.top();
			Queue.pop();
			if(Entry.first != Distances[Entry.second]) continue;
			if(Entry.second == Last) break;
			for(const std::uint32_t EdgeIndex : Incident[Entry.second]) {
				// 解いた水位以下の辺だけを移動可能とする
				const SpillGraphEdge &Edge = Edges[EdgeIndex];
				if(Edge.CrestMeters > Result.CrestMeters) continue;
				const std::uint32_t Next = Edge.Nodes[0] == Entry.second ? Edge.Nodes[1] : Edge.Nodes[0];
				const Vec3 P = Directions[Entry.second].Vector(), Q = Directions[Next].Vector();
				const double Distance = Entry.first + Radius * std::atan2(P.Cross(Q).Length(), P.Dot(Q));
				if(Distance < Distances[Next]) {
					const std::optional<double> Actual = Measure(EdgeIndex);
					// 最短路の全辺も，同じ確定水位以下の実測を要求する返戻
					if(!Actual) return {};
					if(*Actual > Result.CrestMeters) continue;
					Distances[Next] = Distance;
					Parents[Next] = Entry.second;
					ParentEdges[Next] = EdgeIndex;
					Queue.push({ Distance, Next });
				}
			}
		}
		// 元の最小最大路が通る水位でも再接続出来ない場合の返戻
		if(!std::isfinite(Distances[Last])) return {};
		double HighestEdge = Sites[First].ElevationMeters;
		std::vector<std::uint32_t> PathEdges;
		std::uint32_t Current = Last;
		Result.Path.push_back(End);
		while(Current != First && Result.PathNodes < Result.Nodes) {
			// 確定順が下がる経路と探索範囲への接触
			const std::uint32_t Next = Parents[Current], EdgeIndex = ParentEdges[Current];
			// 有効な隣接先を失った場合の返戻
			if(Next >= Result.Nodes || EdgeIndex >= Edges.size()) return {};
			PathEdges.push_back(EdgeIndex);
			if(Current < First) {
				const std::uint32_t X = Current % Width, Y = Current / Width;
				Result.TouchesBoundary = Result.TouchesBoundary || !X || !Y || X + 1 == Width || Y + 1 == Height;
			}
			if(Edges[EdgeIndex].CrestMeters > HighestEdge) {
				HighestEdge = Edges[EdgeIndex].CrestMeters;
				Result.Peak = Peaks[EdgeIndex];
			}
			// 準備間隔より細かい独立な標本による通路の検査
			const Vec3 P = Directions[Current].Vector(), Q = Directions[Next].Vector();
			const double Length = Radius * std::atan2(P.Cross(Q).Length(), P.Dot(Q));
			const unsigned Probes = std::max(1U, static_cast<unsigned>(std::ceil(Length / 10)));
			Result.PathLengthMeters += Length;
			for(unsigned Probe = 0; Probe < Probes; ++Probe) {
				const double Fraction = (Probe + 0.5) / Probes;
				const double Elevation = SampleSource(*UnitDirection::FromVector(P * (1 - Fraction) + Q * Fraction)).ElevationMeters;
				// 独立検査でも場の欠落を成功としない返戻
				if(!std::isfinite(Elevation) || Stop.stop_requested()) return {};
				Result.MaximumIndependentExcessMeters = std::max(Result.MaximumIndependentExcessMeters, Elevation - Result.CrestMeters);
			}
			Current = Next;
			Result.Path.push_back(Directions[Current]);
			++Result.PathNodes;
		}
		// 診断用の根へ到達しなかった場合の返戻
		if(Current != First) return {};
		std::reverse(Result.Path.begin(), Result.Path.end());
		std::reverse(PathEdges.begin(), PathEdges.end());
		// 測定済の各辺の極大から，経路上の局所最高点を始点順に保持
		for(std::size_t I = 0; I < PathEdges.size(); ++I) {
			const std::uint32_t Edge = PathEdges[I];
			const double Crest = Edges[Edge].CrestMeters;
			if(I && Crest <= Edges[PathEdges[I - 1]].CrestMeters) continue;
			if(I + 1 < PathEdges.size() && Crest < Edges[PathEdges[I + 1]].CrestMeters) continue;
			Result.PassCandidates.push_back(Peaks[Edge]);
		}
		// 地形を変更しない局所通路の診断結果の返戻
		return Result;
	}

	/**
	 * 元場を変更せず，中心から有限の外周へ達する最小最大路を探す
	 * @param Center 同じ連続地形上の窪地又は照会点
	 * @param Source 不変な元標高と勾配
	 * @param Settings 方形窓の半幅・物理間隔と有限な準備予算
	 * @param Stop 中断要求
	 * @return 外周迄の候補経路；実鞍点・両終端・大洋接続の確認は別途必要
	 */
	std::optional<SpillWindowExit> FindSpillWindowExit(
		const UnitDirection Center,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillPassageSettings &Settings,
		const std::stop_token Stop
	) {
		// 物理的な探索範囲と有界な計算量の検査
		const double Radius = Settings.RadiusMeters, HalfExtent = Settings.HalfWidthMeters, Spacing = Settings.SpacingMeters;
		const bool Valid =
		Source && std::isfinite(Radius) && Radius >= 1000 && Radius <= 1E9 && std::isfinite(HalfExtent) && HalfExtent > 0 &&
		HalfExtent < Radius * 0.5 && std::isfinite(Spacing) && Spacing >= 0.01 && Spacing <= HalfExtent &&
		std::isfinite(Settings.MaximumSampleStepMeters) && Settings.MaximumSampleStepMeters >= 0.01 &&
		!(Settings.MaximumSampleStepMeters > 50) && Settings.MaximumNodes > 10 && Settings.MaximumNodes < 1000001 &&
		Settings.MaximumEvaluations > 0 && Settings.MaximumEvaluations < 100000001;
		// 不正入力又は開始前の中断への返戻
		if(!Valid || Stop.stop_requested()) return {};
		const double HalfCells = std::ceil(HalfExtent / Spacing);
		// 整数変換前に節点予算を照合する返戻
		if(!std::isfinite(HalfCells) || HalfCells > 499) return {};
		const std::uint32_t Half = static_cast<std::uint32_t>(HalfCells), Width = 2 * Half + 1;
		const std::uint32_t Count = Width * Width, Root = Count, Start = Half * Width + Half;
		// 元点列と仮想外周根を合わせた予算の検査
		if(Count + 1 > Settings.MaximumNodes) return {};
		const double Step = HalfExtent / Half;
		const Vec3 Up = Center.Vector(), Axis = std::abs(Up.Z) < 0.9 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
		const Vec3 East = UnitDirection::FromVector(Axis.Cross(Up))->Vector(), North = Up.Cross(East);
		// 指定方向に向く外周だけを根へ結び，既知の内陸へ戻る最安路を繰返さない
		std::optional<UnitDirection> Forward;
		if(Settings.WindowExitTarget) {
			const Vec3 Target = Settings.WindowExitTarget->Vector();
			Forward = UnitDirection::FromVector(Up.Cross(Target).Cross(Up));
			// 中心又は対蹠点では前方を定義出来ない為の返戻
			if(!Forward) return {};
		}
		SpillWindowExit Result { Center };
		Result.Nodes = Count;
		Result.MinimumBoundaryMeters = std::numeric_limits<double>::infinity();
		const auto Sample = [&](const UnitDirection D) -> TerrainSample {
			// 予算を使切った後に元場を呼出さない返戻
			if(Stop.stop_requested() || Result.Evaluations >= Settings.MaximumEvaluations) {
				const double Invalid = std::numeric_limits<double>::quiet_NaN();
				// 未計測を伝える非有限値の返戻
				return { Invalid, { Invalid, Invalid, Invalid } };
			}
			++Result.Evaluations;
			// 計数した元場の返戻
			return Source(D);
		};
		// 描画へ使わない接続検査の標本
		std::vector<UnitDirection> Directions;
		std::vector<TerrainSample> Samples;
		std::vector<SpillGraphSite> Sites;
		std::vector<bool> Boundary;
		Directions.reserve(Count);
		Samples.reserve(Count);
		Sites.reserve(Count + 1);
		Boundary.reserve(Count);
		for(std::uint32_t Y = 0; Y < Width; ++Y) for(std::uint32_t X = 0; X < Width; ++X) {
			const double LocalX = (static_cast<double>(X) - Half) * Step, LocalY = (static_cast<double>(Y) - Half) * Step;
			const UnitDirection D =
			X == Half && Y == Half ? Center : *UnitDirection::FromVector(Up + (East * LocalX + North * LocalY) * (1 / Radius));
			const TerrainSample Value = Sample(D);
			// 無効な元場又は予算超過を乾いた節点としない返戻
			if(!std::isfinite(Value.ElevationMeters) || !std::isfinite(Value.TangentialGradientPerRadian.Length())) return {};
			const bool Outer = !X || !Y || X + 1 == Width || Y + 1 == Width;
			Directions.push_back(D);
			Samples.push_back(Value);
			Sites.push_back({ Value.ElevationMeters, false });
			const bool ExitBoundary = Outer && (!Forward || (East * LocalX + North * LocalY).Dot(Forward->Vector()) > 0);
			Boundary.push_back(ExitBoundary);
			if(ExitBoundary) Result.MinimumBoundaryMeters = std::min(Result.MinimumBoundaryMeters, Value.ElevationMeters);
		}
		// 最小最大路の単一根を外周へ結ぶだけで，大洋の実在は意味しない
		Sites.push_back({ Result.MinimumBoundaryMeters, true });
		std::vector<SpillGraphEdge> Edges;
		std::vector<UnitDirection> Peaks;
		Edges.reserve(2 * Count + 4 * Width);
		Peaks.reserve(Edges.capacity());
		const auto AddEdge = [&](const std::uint32_t A, const std::uint32_t B) -> void {
			// 端点の最高標高は弧全体の最高標高の下限
			Edges.push_back({ { A, B }, std::max(Samples[A].ElevationMeters, Samples[B].ElevationMeters) });
			Peaks.push_back(Samples[A].ElevationMeters > Samples[B].ElevationMeters ? Directions[A] : Directions[B]);
			// 終了
			return;
		};
		for(std::uint32_t Y = 0; Y < Width; ++Y) for(std::uint32_t X = 0; X < Width; ++X) {
			const std::uint32_t I = Y * Width + X;
			if(X + 1 < Width) AddEdge(I, I + 1);
			if(Y + 1 < Width) AddEdge(I, I + Width);
			if(Boundary[I]) {
				Edges.push_back({ { I, Root }, Samples[I].ElevationMeters });
				Peaks.push_back(Directions[I]);
			}
		}
		// 必要な実辺だけを同じ物理間隔・微分の符号反転で測る
		std::vector<bool> Measured(Edges.size(), false);
		const auto Measure = [&](const std::uint32_t I) -> std::optional<double> {
			SpillGraphEdge &Edge = Edges[I];
			const std::uint32_t A = Edge.Nodes[0], B = Edge.Nodes[1];
			// 仮想外周辺と実測済の辺には元場の照会を重ねない返戻
			if(B == Root || Measured[I]) return Edge.CrestMeters;
			const std::optional<double> Crest = MeasureSpillPassageCrest(
				Sample,
				Directions[A],
				Directions[B],
				Samples[A],
				Samples[B],
				Radius,
				Settings.MaximumSampleStepMeters,
				Stop,
				&Peaks[I]
			);
			// 未計測を通過高度の下限で代用しない返戻
			if(!Crest) return {};
			Edge.CrestMeters = *Crest;
			Measured[I] = true;
			++Result.EvaluatedEdges;
			// 同じ連続場で確認した辺の最高標高の返戻
			return *Crest;
		};
		// 比較用の全辺評価でも計算予算と中断を守る返戻
		if(Settings.MeasureAllWindowEdges) for(std::uint32_t I = 0; I < Edges.size(); ++I) if(!Measure(I)) return {};
		const PathSolver Solve = Settings.UsePathEdgeSelection ? FindPathMeasuredSpillPath : FindMeasuredSpillPath;
		const std::optional<MeasuredSpillPath> Path = Solve(Sites, Edges, Start, Root, Measure, Stop);
		// 未確定の候補経路を外周接続として扱わない返戻
		if(!Path) return {};
		Result.CrestMeters = Path->CrestMeters;
		double Highest = -std::numeric_limits<double>::infinity();
		std::vector<std::uint32_t> PathEdges;
		const auto IsBoundaryPeak = [&](const UnitDirection D) -> bool {
			const Vec3 P = D.Vector();
			const double Denominator = P.Dot(Up), X = Radius * P.Dot(East) / Denominator, Y = Radius * P.Dot(North) / Denominator;
			// 外周と区別出来ない最高点の判定の返戻
			return std::max(std::abs(X), std::abs(Y)) >= HalfExtent - Step * 0.25;
		};
		// 仮想根を除いた実方向の経路と最高点の復元
		for(std::size_t I = 0; I < Path->Edges.size(); ++I) {
			// 途中の中断又は範囲外の経路への返戻
			if(Stop.stop_requested() || Path->Nodes[I] >= Count) return {};
			Result.Path.push_back(Directions[Path->Nodes[I]]);
			const std::uint32_t Edge = Path->Edges[I];
			// 未定義の通路を使わない返戻
			if(Edge >= Edges.size()) return {};
			if(Edges[Edge].CrestMeters > Highest) {
				Highest = Edges[Edge].CrestMeters;
				Result.Peak = Peaks[Edge];
				Result.PeakOnBoundary = IsBoundaryPeak(Result.Peak);
			}
			if(Path->Nodes[I + 1] != Root) PathEdges.push_back(Edge);
		}
		// 仮想根へ到達していない有限長の復元を成功にしない返戻
		if(Path->Nodes.back() != Root || Result.Path.empty()) return {};
		// 最高の既知峠の先に在る，より低い未発見の峠も候補へ保持
		for(std::size_t I = 0; I < PathEdges.size(); ++I) {
			const std::uint32_t Edge = PathEdges[I];
			const double Crest = Edges[Edge].CrestMeters;
			if(I && Crest <= Edges[PathEdges[I - 1]].CrestMeters) continue;
			if(I + 1 < PathEdges.size() && Crest < Edges[PathEdges[I + 1]].CrestMeters) continue;
			if(!IsBoundaryPeak(Peaks[Edge])) Result.PassCandidates.push_back(Peaks[Edge]);
		}
		// 同じ標本の解析勾配を，正規化球面写像の鎖則で窓座標の微分へ変換する
		if(Settings.UseCellCriticalCandidates) {
			const auto WindowGradient = [&](const std::uint32_t Index) -> std::array<double, 2> {
				const Vec3 D = Directions[Index].Vector(), G = Samples[Index].TangentialGradientPerRadian;
				const Vec3 Tangential = G - D * G.Dot(D);
				// |P| の逆数は D・Up；物理座標に関する共変成分の返戻
				return { Tangential.Dot(East) * D.Dot(Up) / Radius, Tangential.Dot(North) * D.Dot(Up) / Radius };
			};
			for(std::uint32_t Y = 0; Y + 1 < Width; ++Y) for(std::uint32_t X = 0; X + 1 < Width; ++X) {
				// セル走査も中断した部分候補を公開しない返戻
				if(Stop.stop_requested()) return {};
				const std::uint32_t I = Y * Width + X;
				const std::array<std::array<double, 2>, 4> Corners {
					WindowGradient(I),
					WindowGradient(I + 1),
					WindowGradient(I + Width),
					WindowGradient(I + Width + 1)
				};
				const BilinearCriticalCandidates Candidates = FindBilinearCriticalCandidates(Corners);
				for(std::uint32_t Candidate = 0; Candidate < Candidates.Count; ++Candidate) {
					const BilinearCriticalCandidate &Point = Candidates.Points[Candidate];
					if(!(Point.JacobianDeterminant < 0)) continue;
					const double LocalX = (static_cast<double>(X) - Half + Point.Coordinates[0]) * Step;
					const double LocalY = (static_cast<double>(Y) - Half + Point.Coordinates[1]) * Step;
					const UnitDirection D = *UnitDirection::FromVector(Up + (East * LocalX + North * LocalY) * (1 / Radius));
					Result.PassCandidates.push_back(D);
					++Result.CellCriticalCandidates;
				}
			}
		}
		// 元の値と有限な外周への到達条件の返戻
		return Result;
	}
}
