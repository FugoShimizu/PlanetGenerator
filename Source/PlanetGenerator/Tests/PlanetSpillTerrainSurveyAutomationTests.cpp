#include "Core/PlanetCore.h"
#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetDepressionHierarchy.h"
#include "Core/PlanetDescentPath.h"
#include "Core/PlanetFlowChannel.h"
#include "Core/PlanetFlowCurve.h"
#include "Core/PlanetFlowRoute.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillGraph.h"
#include "Core/PlanetTerrainContour.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	// 元地形の有限幅通路を調べる診断値
	struct SpillCorridorSurvey {
		// 局所通過条件と場の評価費用
		PlanetCore::UnitDirection Peak;
		double CrestMeters = 0;
		double PathLengthMeters = 0;
		double MaximumIndependentExcessMeters = 0;
		std::array<double, 2> BoundaryEscapeMeters {};
		double OutsideLowerBoundMeters = 0;
		bool IsGraphInteriorMinimum = false;
		std::uint64_t Evaluations = 0;
		std::uint32_t Nodes = 0;
		std::uint32_t PathNodes = 0;
		bool TouchesBoundary = false;
	};

	/**
	 * 物理間隔の標本と接線の符号反転を用い，選択通路の最高標高を測る
	 * 計算量：O(L / d + 24 K)，L は通路長，d は標本間隔，K は検出した極大数
	 * @param Source 変更しない元地形
	 * @param Start 通路の始点
	 * @param End 通路の終点
	 * @param Radius 惑星半径
	 * @param Evaluations 実際の地形照会数を加算する参照
	 * @param PeakDirection 最高標高の位置を受取る場合の参照先，省略時は位置を返さない
	 * @return 測った最高標高，有限標本間の極値が全て検出された事の証明ではない
	 */
	double MeasureSpillPassageCrest(
		const PlanetCore::OceanConnectivityField &Source,
		const PlanetCore::UnitDirection Start,
		const PlanetCore::UnitDirection End,
		const double Radius,
		std::uint64_t &Evaluations,
		PlanetCore::UnitDirection *const PeakDirection=nullptr
	) {
		// 大円上の位置と接線の準備
		const PlanetCore::Vec3 A = Start.Vector(), B = End.Vector();
		const double Angle = std::atan2(A.Cross(B).Length(), A.Dot(B)), Length = Angle * Radius;
		const PlanetCore::Vec3 Tangent = PlanetCore::UnitDirection::FromVector(B - A * A.Dot(B))->Vector();
		const std::uint32_t Segments = static_cast<std::uint32_t>(std::ceil(Length / 50));
		// 最大値の位置を要求された場合の記録
		double RecordedHeight = -std::numeric_limits<double>::infinity();
		// 端点と同じ連続場の標高及び弧長方向の微分
		const auto Sample = [&](const double Fraction) -> std::array<double, 2> {
			const double Phase = Angle * Fraction;
			const PlanetCore::Vec3 Position = A * std::cos(Phase) + Tangent * std::sin(Phase);
			const PlanetCore::Vec3 Along = Tangent * std::cos(Phase) - A * std::sin(Phase);
			const PlanetCore::TerrainSample Value = Source.Evaluate(*PlanetCore::UnitDirection::FromVector(Position));
			++Evaluations;
			if(PeakDirection && Value.ElevationMeters > RecordedHeight) {
				RecordedHeight = Value.ElevationMeters;
				*PeakDirection = *PlanetCore::UnitDirection::FromVector(Position);
			}
			// 弧上の標高と，微分符号判定に使う方向微分の返戻
			return { Value.ElevationMeters, Value.TangentialGradientPerRadian.Dot(Along) };
		};
		std::array<double, 2> Previous = Sample(0);
		double Crest = Previous[0];
		for(std::uint32_t i = 1; i <= Segments; ++i) {
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
		// 端点と検出した極大の最大値の返戻
		return Crest;
	}

	/**
	 * 元の辺の周囲で小窪地の接続を調べ，標本網内の最低通過標高を求める
	 * 計算量：O(N log N + E L / 50)，N は局所頂点数，E は辺数，L は平均辺長
	 * @param Source 変更しない解析地形
	 * @param Start 元の通路の一端
	 * @param End 元の通路の他端
	 * @param Radius 惑星半径
	 * @param SpacingMeters 局所網の物理間隔
	 * @param HalfWidthMeters 通路の横方向の探索範囲
	 * @return 有限標本内の通過標高と独立な通路残差，予算超過又は解なしは空
	 */
	std::optional<SpillCorridorSurvey> InspectSpillCorridor(
		const PlanetCore::OceanConnectivityField &Source,
		const PlanetCore::UnitDirection Start,
		const PlanetCore::UnitDirection End,
		const double Radius,
		const double SpacingMeters,
		const double HalfWidthMeters
	) {
		using PlanetCore::UnitDirection;
		using PlanetCore::Vec3;
		// 元の辺を軸とした球面の局所座標
		const Vec3 A = Start.Vector(), B = End.Vector(), Up = UnitDirection::FromVector(A + B)->Vector();
		const Vec3 Along = UnitDirection::FromVector(B - A)->Vector(), Across = Up.Cross(Along);
		// 全ての標本間隔で同じ物理範囲を使う為の固定外縁
		const double EndpointX = Radius * B.Dot(Along) / B.Dot(Up);
		const double HalfLengthMeters = std::ceil((EndpointX + HalfWidthMeters * 0.5) / 500) * 500;
		const std::uint32_t HalfX = static_cast<std::uint32_t>(std::ceil(HalfLengthMeters / SpacingMeters));
		const std::uint32_t HalfY = static_cast<std::uint32_t>(std::ceil(HalfWidthMeters / SpacingMeters));
		const std::uint32_t Width = 2 * HalfX + 1, Height = 2 * HalfY + 1;
		// 測定用の有界な局所網
		if(Width > 512 || Height > 512 || Width * Height > 65534) return {};
		SpillCorridorSurvey Result { Start };
		std::vector<UnitDirection> Directions;
		std::vector<PlanetCore::SpillGraphSite> Sites;
		std::vector<PlanetCore::SpillGraphEdge> Edges;
		std::vector<UnitDirection> Peaks;
		for(std::uint32_t Y = 0; Y < Height; ++Y) for(std::uint32_t X = 0; X < Width; ++X) {
			// 描画又は地形補間へ流用しない接続検査用の標本
			const double LocalX = (static_cast<double>(X) - HalfX) * SpacingMeters;
			const double LocalY = (static_cast<double>(Y) - HalfY) * SpacingMeters;
			const UnitDirection Direction = *UnitDirection::FromVector(Up + (Along * LocalX + Across * LocalY) * (1 / Radius));
			Directions.push_back(Direction);
			Sites.push_back({ Source.EvaluateElevation(Direction), false });
			++Result.Evaluations;
		}
		// 両端は格子点へ丸めず，元の正確な方向を追加
		const std::uint32_t First = static_cast<std::uint32_t>(Directions.size()), Last = First + 1;
		for(const UnitDirection Direction : { Start, End }) {
			Directions.push_back(Direction);
			Sites.push_back({ Source.EvaluateElevation(Direction), false });
			++Result.Evaluations;
		}
		// 同じ最小最大路を使う為の単一の診断用根，大洋の実在を主張しない
		Sites[First].IsOcean = true;
		const auto AddEdge = [&](const std::uint32_t U, const std::uint32_t V) -> void {
			// 通路全体の標高と最高点の記録
			UnitDirection Peak = Directions[U];
			double Crest = std::max(Sites[U].ElevationMeters, Sites[V].ElevationMeters);
			if(Directions[U].Vector().Cross(Directions[V].Vector()).Length() > 1E-14) {
				Crest = std::max(Crest, MeasureSpillPassageCrest(Source, Directions[U], Directions[V], Radius, Result.Evaluations, &Peak));
			}
			Edges.push_back({ { U, V }, Crest });
			Peaks.push_back(Peak);
			// 終了
			return;
		};
		for(std::uint32_t Y = 0; Y < Height; ++Y) for(std::uint32_t X = 0; X < Width; ++X) {
			// ４近傍の接続は水位だけの診断に使用
			const std::uint32_t Index = Y * Width + X;
			if(X + 1 < Width) AddEdge(Index, Index + 1);
			if(Y + 1 < Height) AddEdge(Index, Index + Width);
		}
		for(const std::uint32_t Endpoint : { First, Last }) {
			// 元の端点を周囲の４頂点へ接続
			const Vec3 Direction = Directions[Endpoint].Vector();
			const double X = Radius * Direction.Dot(Along) / Direction.Dot(Up) / SpacingMeters + HalfX;
			const double Y = Radius * Direction.Dot(Across) / Direction.Dot(Up) / SpacingMeters + HalfY;
			const std::uint32_t CellX = static_cast<std::uint32_t>(std::floor(X)), CellY = static_cast<std::uint32_t>(std::floor(Y));
			for(unsigned DY = 0; DY < 2; ++DY) for(unsigned DX = 0; DX < 2; ++DX) AddEdge(Endpoint, (CellY + DY) * Width + CellX + DX);
		}
		// 根の標高を基準にした最小最大路，大洋面や湖の面積には使わない
		const std::optional<PlanetCore::SpillGraph> Graph = PlanetCore::BuildSpillGraph(Sites, Edges, Sites[First].ElevationMeters);
		// 有効な診断経路が無い場合の返戻
		if(!Graph) return {};
		Result.Nodes = static_cast<std::uint32_t>(Directions.size());
		Result.CrestMeters = Graph->Nodes[Last].EscapeMeters;
		// 逆向きの同じ最小最大路と，外へ迂回する為に必要な水位の下限
		Sites[First].IsOcean = false;
		Sites[Last].IsOcean = true;
		const std::optional<PlanetCore::SpillGraph> Reverse = PlanetCore::BuildSpillGraph(Sites, Edges, Sites[Last].ElevationMeters);
		// 両端の順序を反転しても最低通過標高が一致する事の検査
		if(!Reverse || Reverse->Nodes[First].EscapeMeters != Result.CrestMeters) return {};
		Result.BoundaryEscapeMeters.fill(std::numeric_limits<double>::infinity());
		for(std::uint32_t Y = 0; Y < Height; ++Y) for(std::uint32_t X = 0; X < Width; ++X) {
			if(X && Y && X + 1 < Width && Y + 1 < Height) continue;
			// 両端から境界へ出る最小の通過標高
			const std::uint32_t Index = Y * Width + X;
			Result.BoundaryEscapeMeters[0] = std::min(Result.BoundaryEscapeMeters[0], Graph->Nodes[Index].EscapeMeters);
			Result.BoundaryEscapeMeters[1] = std::min(Result.BoundaryEscapeMeters[1], Reverse->Nodes[Index].EscapeMeters);
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
		std::vector<std::uint32_t> Parents(Result.Nodes, PlanetCore::MissingSpillIndex);
		std::vector<std::uint32_t> ParentEdges(Result.Nodes, PlanetCore::MissingSpillIndex);
		Distances[First] = 0;
		Queue.push({ 0, First });
		while(!Queue.empty()) {
			// 確定済の距離より古い候補は再処理しない
			const QueueEntry Entry = Queue.top();
			Queue.pop();
			if(Entry.first != Distances[Entry.second]) continue;
			if(Entry.second == Last) break;
			for(const std::uint32_t EdgeIndex : Incident[Entry.second]) {
				// 解いた水位以下の辺だけを移動可能とする
				const PlanetCore::SpillGraphEdge &Edge = Edges[EdgeIndex];
				if(Edge.CrestMeters > Result.CrestMeters) continue;
				const std::uint32_t Next = Edge.Nodes[0] == Entry.second ? Edge.Nodes[1] : Edge.Nodes[0];
				const Vec3 P = Directions[Entry.second].Vector(), Q = Directions[Next].Vector();
				const double Distance = Entry.first + Radius * std::atan2(P.Cross(Q).Length(), P.Dot(Q));
				if(Distance < Distances[Next]) {
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
		std::uint32_t Current = Last;
		while(Current != First && Result.PathNodes < Result.Nodes) {
			// 確定順が下がる経路と探索範囲への接触
			const std::uint32_t Next = Parents[Current], EdgeIndex = ParentEdges[Current];
			// 有効な隣接先を失った場合の返戻
			if(Next >= Result.Nodes || EdgeIndex >= Edges.size()) return {};
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
				const double Elevation = Source.EvaluateElevation(*UnitDirection::FromVector(P * (1 - Fraction) + Q * Fraction));
				++Result.Evaluations;
				Result.MaximumIndependentExcessMeters = std::max(Result.MaximumIndependentExcessMeters, Elevation - Result.CrestMeters);
			}
			Current = Next;
			++Result.PathNodes;
		}
		// 診断用の根へ到達しなかった場合の返戻
		if(Current != First) return {};
		// 地形を変更しない局所通路の診断結果の返戻
		return Result;
	}
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FPlanetSpillTerrainSurvey, "PlanetGenerator.SpillGraph.TerrainSurvey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 端点間８区間と，選択通路だけを精査する条件を列挙する
 * @param Names 表示名
 * @param Commands 実行引数
 */
void FPlanetSpillTerrainSurvey::GetTests(TArray<FString> &Names, TArray<FString> &Commands) const {
	Names.Add(TEXT("Coarse"));
	Commands.Add(TEXT("Coarse"));
	Names.Add(TEXT("RefinedRoutes"));
	Commands.Add(TEXT("RefinedRoutes"));
	Names.Add(TEXT("Saddles"));
	Commands.Add(TEXT("Saddles"));
	Names.Add(TEXT("Descent"));
	Commands.Add(TEXT("Descent"));
	Names.Add(TEXT("Topology"));
	Commands.Add(TEXT("Topology"));
	Names.Add(TEXT("Passages"));
	Commands.Add(TEXT("Passages"));
	Names.Add(TEXT("WidePassages"));
	Commands.Add(TEXT("WidePassages"));
	Names.Add(TEXT("BoundaryPassages"));
	Commands.Add(TEXT("BoundaryPassages"));
	Names.Add(TEXT("Hierarchy"));
	Commands.Add(TEXT("Hierarchy"));
	Names.Add(TEXT("Contours"));
	Commands.Add(TEXT("Contours"));
	Names.Add(TEXT("LocalLake"));
	Commands.Add(TEXT("LocalLake"));
	Names.Add(TEXT("FlowCurves"));
	Commands.Add(TEXT("FlowCurves"));
	Names.Add(TEXT("CurvatureFlowCurves"));
	Commands.Add(TEXT("CurvatureFlowCurves"));
	Names.Add(TEXT("FlowChannels"));
	Commands.Add(TEXT("FlowChannels"));
	Names.Add(TEXT("FlowChannelWater"));
	Commands.Add(TEXT("FlowChannelWater"));
	// 終了
	return;
}

/**
 * 元の惑星地形に於ける飽和湖の候補と，未解像の堰を測定する
 * @param Parameters 選択経路の精査条件
 * @return 診断の入力・出口証拠が正しい場合に真，湖候補の採用又は地形の合格ではない
 */
bool FPlanetSpillTerrainSurvey::RunTest(const FString &Parameters) {
	// 今回の比較条件
	const bool ShouldRefineRoutes = Parameters == TEXT("RefinedRoutes"), ShouldInspectTopology = Parameters == TEXT("Topology");
	const bool ShouldInspectContours = Parameters == TEXT("Contours");
	const bool ShouldInspectDescent = ShouldInspectTopology || ShouldInspectContours || Parameters == TEXT("Descent");
	const bool ShouldInspectBoundary = Parameters == TEXT("BoundaryPassages");
	const bool ShouldExpandPassages = ShouldInspectBoundary || Parameters == TEXT("WidePassages");
	const bool ShouldInspectPassages = ShouldExpandPassages || Parameters == TEXT("Passages");
	const bool ShouldInspectSaddles = ShouldInspectDescent || ShouldInspectPassages || Parameters == TEXT("Saddles");
	using PlanetCore::OceanConnectivityField;
	using PlanetCore::SpillGraph;
	using PlanetCore::SpillGraphEdge;
	using PlanetCore::SpillGraphSite;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	// 同一の元地質・造山・細部起伏
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = 1000000;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元場の準備に失敗した場合の返戻
	if(!TestTrue(TEXT("湖の診断用地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 山稜の準備に失敗した場合の返戻
	if(!TestTrue(TEXT("湖の診断用山稜を準備"), Ridges.has_value())) return false;
	const std::optional<OceanConnectivityField> Ocean = OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 連続した元地形を取得出来ない場合の返戻
	if(!TestTrue(TEXT("湖の診断用の連続地形を準備"), Ocean.has_value())) return false;
	// 表示した小窪地で，近い鞍点より低い出口を独立な局所網で調べる
	const bool ShouldInspectChannelWater = Parameters == TEXT("FlowChannelWater");
	const bool ShouldInspectChannels = ShouldInspectChannelWater || Parameters == TEXT("FlowChannels");
	const bool ShouldInspectCurvature = ShouldInspectChannels || Parameters == TEXT("CurvatureFlowCurves");
	const bool ShouldInspectFlowCurves = ShouldInspectCurvature || Parameters == TEXT("FlowCurves");
	if(ShouldInspectFlowCurves || Parameters == TEXT("LocalLake")) {
		const UnitDirection Minimum = *UnitDirection::FromVector({ 0.54187601419366105, 0.81191502638215118, -0.2171736060769405 });
		const UnitDirection Receiver = *UnitDirection::FromVector({ 0.54165180349932751, 0.81202943116404547, -0.21730514648604751 });
		const auto Field = [&](const UnitDirection Direction) -> PlanetCore::TerrainSample {
			// 不変な元地形の返戻
			return Ocean->Evaluate(Direction);
		};
		PlanetCore::TerrainCriticalSettings CriticalSettings;
		CriticalSettings.RadiusMeters = Definition.RadiusMeters;
		std::optional<PlanetCore::TerrainCriticalPoint> BestPass;
		double BestCrest = std::numeric_limits<double>::infinity();
		if(ShouldInspectFlowCurves) {
			// 検査済の出口候補を同じ元場で再精査し，未変更の格子探索を繰返さない
			const UnitDirection KnownPass = *UnitDirection::FromVector({ 0.54180127453542282, 0.81192128360203597, -0.21733662403243018 });
			BestPass = PlanetCore::RefineTerrainCriticalPoint(KnownPass, Field, CriticalSettings);
			if(BestPass) BestCrest = BestPass->ElevationMeters;
		}
		// 標本間隔の収束と，領域を広げた際の別出口を区別する
		if(!ShouldInspectFlowCurves) {
			for(const std::array<double, 2> Variant : { std::array<double, 2>{ 4, 128 }, { 2, 128 }, { 4, 512 }, { 8, 1024 } }) {
				const std::optional<SpillCorridorSurvey> Passage =
				InspectSpillCorridor(*Ocean, Minimum, Receiver, Definition.RadiusMeters, Variant[0], Variant[1]);
				if(!TestTrue(TEXT("小窪地の連続場の通路を測定出来る"), Passage.has_value())) continue;
				const std::optional<PlanetCore::TerrainCriticalPoint> Critical =
				PlanetCore::RefineTerrainCriticalPoint(Passage->Peak, Field, CriticalSettings);
				AddInfo(
					FString::Printf(
						TEXT("Local lake passage: spacing=%.9g halfWidth=%.9g nodes=%u crest=%.12g independentExcess=%.12g ")
						TEXT("boundaryStart=%.12g boundaryEnd=%.12g criticalKind=%d criticalHeight=%.12g peak=(%.17g,%.17g,%.17g)"),
						Variant[0],
						Variant[1],
						Passage->Nodes,
						Passage->CrestMeters,
						Passage->MaximumIndependentExcessMeters,
						Passage->BoundaryEscapeMeters[0],
						Passage->BoundaryEscapeMeters[1],
						Critical ? static_cast<int>(Critical->Kind) : -1,
						Critical ? Critical->ElevationMeters : Passage->CrestMeters,
						Passage->Peak.Vector().X,
						Passage->Peak.Vector().Y,
						Passage->Peak.Vector().Z
					)
				);
				if(Critical && Critical->Kind == PlanetCore::TerrainCriticalKind::Saddle && Passage->CrestMeters < BestCrest) {
					BestPass = Critical;
					BestCrest = Passage->CrestMeters;
				}
			}
		}
		// 測定した通路の峠が鞍点に対応しない場合の返戻
		if(!TestTrue(TEXT("低い通路の鞍点を取得出来る"), BestPass.has_value())) return false;
		TestTrue(TEXT("最初の近い鞍点を最低出口と誤認しない"), BestCrest < 846.55540493);
		const Vec3 Saddle = BestPass->Direction.Vector();
		AddInfo(
			FString::Printf(
				TEXT("Local lake saddle: height=%.12g direction=(%.17g,%.17g,%.17g) minimumCurvature=%.12g maximumCurvature=%.12g"),
				BestPass->ElevationMeters,
				Saddle.X,
				Saddle.Y,
				Saddle.Z,
				BestPass->MinimumCurvaturePerMeter,
				BestPass->MaximumCurvaturePerMeter
			)
		);
		unsigned ReturningBranches = 0;
		for(const double Sign : { -1.0, 1.0 }) {
			// 新しい鞍点の両側を元場に沿って下降し，実際の窪地との対応を検査
			const UnitDirection Start =
			*UnitDirection::FromVector(Saddle + BestPass->MinimumCurvatureDirection * (Sign * 5 / Definition.RadiusMeters));
			PlanetCore::TerrainDescentSettings FlowSettings;
			FlowSettings.RadiusMeters = Definition.RadiusMeters;
			if(ShouldInspectFlowCurves) FlowSettings.PositionToleranceMeters = 1E-5;
			const std::optional<PlanetCore::TerrainDescentPath> Flow = PlanetCore::TraceTerrainDescent(Start, Field, FlowSettings);
			if(!TestTrue(TEXT("新しい鞍点から下降曲線を取得出来る"), Flow.has_value())) continue;
			if(!TestTrue(TEXT("曲線化する下降点列に２点以上有る"), Flow->Samples.size() > 1)) continue;
			const PlanetCore::TerrainDescentSample &Last = Flow->Samples.back();
			const std::optional<PlanetCore::TerrainCriticalPoint> End =
			PlanetCore::RefineTerrainCriticalPoint(Last.Direction, Field, CriticalSettings);
			const double Distance = Definition.RadiusMeters *
			std::atan2(Last.Direction.Vector().Cross(Minimum.Vector()).Length(), Last.Direction.Vector().Dot(Minimum.Vector()));
			if(Distance < 0.1 && End && End->Kind == PlanetCore::TerrainCriticalKind::Minimum) ++ReturningBranches;
			AddInfo(
				FString::Printf(
					TEXT("Local lake branch: sign=%.0f end=%u points=%llu length=%.12g height=%.12g knownMinimumDistance=%.12g ")
					TEXT("criticalKind=%d direction=(%.17g,%.17g,%.17g)"),
					Sign,
					static_cast<unsigned>(Flow->End),
					static_cast<unsigned long long>(Flow->Samples.size()),
					Last.DistanceMeters,
					Last.ElevationMeters,
					Distance,
					End ? static_cast<int>(End->Kind) : -1,
					Last.Direction.Vector().X,
					Last.Direction.Vector().Y,
					Last.Direction.Vector().Z
				)
			);
			// 同じ下降接線を共有する曲線の内部も元場を下るか検査する
			std::uint32_t RejectedCurves = 0, CurveSamples = 0, UphillSamples = 0;
			double MaximumUphillDerivative = 0, MaximumCurveRise = 0, MaximumDirectionError = 0, WorstHeight = 0, WorstSlope = 0;
			double WorstLength = 0, WorstDistance = 0, WorstFraction = 0, MaximumCurvatureJoin = 0;
			std::uint32_t ChannelPoints = 0, MissingFootpoints = 0, SubmergedEdges = 0;
			std::uint32_t RouteQueries = 0, RouteCandidates = 0, MaximumRouteCandidates = 0;
			double MaximumIndexedDistanceError = 0;
			std::optional<PlanetCore::TerrainFlowRoute> Route;
			if(ShouldInspectChannels) {
				std::vector<PlanetCore::FlowRouteKnot> Knots;
				for(const PlanetCore::TerrainDescentSample &Point : Flow->Samples) {
					Knots.push_back({ Point.Direction, Field(Point.Direction).TangentialGradientPerRadian * -1, Point.DistanceMeters });
				}
				Route = PlanetCore::TerrainFlowRoute::Create(Knots, Definition.RadiusMeters);
				if(!TestTrue(TEXT("実地形の経路索引を準備出来る"), Route.has_value())) continue;
			}
			double MaximumCut = 0, MaximumEdgeWaterExcess = 0, CurveDistanceMeters = 5;
			std::optional<PlanetCore::TerrainFlowCurveSample> PreviousEnd;
			std::vector<Vec3> Tangents, Curvatures(Flow->Samples.size());
			for(const PlanetCore::TerrainDescentSample &Point : Flow->Samples) {
				Tangents.push_back(UnitDirection::FromVector(Field(Point.Direction).TangentialGradientPerRadian * -1)->Vector());
			}
			if(ShouldInspectCurvature) for(std::size_t i = 0; i < Flow->Samples.size(); ++i) {
				// 同じ節点の曲率は一度だけ推定し，両側の区間へ共有する
				const double Before = i ? Flow->Samples[i].DistanceMeters - Flow->Samples[i - 1].DistanceMeters : 0;
				const double After = i + 1 < Flow->Samples.size() ? Flow->Samples[i + 1].DistanceMeters - Flow->Samples[i].DistanceMeters : 0;
				if(!i) Curvatures[i] = (Tangents[i + 1] - Tangents[i]) * (1 / After);
				else if(i + 1 == Flow->Samples.size()) Curvatures[i] = (Tangents[i] - Tangents[i - 1]) * (1 / Before);
				else {
					Curvatures[i] = (Tangents[i] - Tangents[i - 1]) * (After / (Before * (Before + After))) +
					(Tangents[i + 1] - Tangents[i]) * (Before / (After * (Before + After)));
				}
			}
			for(std::size_t i = 1; i < Flow->Samples.size(); ++i) {
				const UnitDirection A = Flow->Samples[i - 1].Direction, B = Flow->Samples[i].Direction;
				const std::optional<PlanetCore::TerrainFlowSegment> Curve = ShouldInspectCurvature ?
				PlanetCore::TerrainFlowSegment::CreateWithCurvature(
					A,
					B,
					Tangents[i - 1],
					Tangents[i],
					Curvatures[i - 1],
					Curvatures[i],
					Definition.RadiusMeters
				) :
				PlanetCore::TerrainFlowSegment::Create(A, B, Tangents[i - 1], Tangents[i], Definition.RadiusMeters);
				if(!Curve) {
					++RejectedCurves;
					continue;
				}
				if(PreviousEnd) {
					MaximumCurvatureJoin = std::max(
						MaximumCurvatureJoin,
						Definition.RadiusMeters *
						(Curve->Evaluate(0)->DirectionSecondDerivativePerMeterSquared - PreviousEnd->DirectionSecondDerivativePerMeterSquared).Length()
					);
				}
				PreviousEnd = Curve->Evaluate(1);
				if(ShouldInspectChannels && Sign > 0 && Field(B).ElevationMeters > 681.98759) {
					// 下流の試験水位より上の枝で，狭い開削と両岸の成立を別に測る
					PlanetCore::FlowChannelShape Shape;
					Shape.RadiusMeters = Definition.RadiusMeters;
					for(const double Fraction : { 0.125, 0.5, 0.875 }) {
						const PlanetCore::TerrainFlowCurveSample Center = *Curve->Evaluate(Fraction);
						const Vec3 Tangent = UnitDirection::FromVector(Center.DirectionDerivativePerMeter)->Vector();
						const Vec3 Side = Center.Direction.Vector().Cross(Tangent);
						for(const double Across : { -6.0, -3.0, 0.0, 3.0, 6.0 }) {
							const double Angle = Across / Definition.RadiusMeters;
							const UnitDirection Query = *UnitDirection::FromVector(Center.Direction.Vector() * std::cos(Angle) + Side * std::sin(Angle));
							const std::optional<PlanetCore::TerrainFlowFootpoint> Foot = Curve->FindInteriorFootpoint(Query);
							if(!Foot) {
								++MissingFootpoints;
								continue;
							}
							std::uint32_t Candidates = 0;
							const std::optional<PlanetCore::FlowRouteProjection> Indexed = Route->FindFootpoint(Query, 6.1, &Candidates);
							if(!TestTrue(TEXT("実河道の岸まで経路索引から照会出来る"), Indexed.has_value())) continue;
							RouteCandidates += Candidates;
							MaximumRouteCandidates = std::max(MaximumRouteCandidates, Candidates);
							if(!(RouteQueries % 11)) {
								double BestDistance = std::numeric_limits<double>::infinity();
								for(const PlanetCore::TerrainFlowSegment &Segment : Route->GetSegments()) {
									const std::optional<PlanetCore::TerrainFlowFootpoint> Reference = Segment.FindInteriorFootpoint(Query);
									if(Reference) BestDistance = std::min(BestDistance, Reference->SquaredChordDistance);
								}
								const double Error =
								std::abs(Indexed->Footpoint.SquaredChordDistance - BestDistance) * Definition.RadiusMeters * Definition.RadiusMeters;
								TestTrue(TEXT("実河道でも索引と全区間走査が一致"), Error < 1E-8);
								MaximumIndexedDistanceError = std::max(MaximumIndexedDistanceError, Error);
							}
							++RouteQueries;
							const PlanetCore::TerrainSample Here = Field(Query);
							const PlanetCore::TerrainSample AtFoot = Field(Foot->Curve.Direction);
							PlanetCore::FlowChannelWaterLevel WaterLevel {
								AtFoot.ElevationMeters,
								AtFoot.TangentialGradientPerRadian.Dot(Foot->Curve.DirectionDerivativePerMeter)
							};
							if(ShouldInspectChannelWater) {
								PlanetCore::FlowChannelWaterSettings WaterSettings;
								WaterSettings.DownstreamWaterMeters = 681.98759;
								const std::optional<PlanetCore::FlowChannelWaterLevel> Profile = PlanetCore::SampleFlowWaterLevel(
									WaterLevel,
									CurveDistanceMeters + Foot->Fraction * Curve->GetParameterLengthMeters(),
									WaterSettings
								);
								if(!TestTrue(TEXT("実河道の下降縦断を水位へ接続出来る"), Profile.has_value())) continue;
								WaterLevel = *Profile;
							}
							const std::optional<PlanetCore::FlowChannelSection> Section = PlanetCore::SampleFlowChannel(*Foot, Here, WaterLevel, Shape);
							if(!TestTrue(TEXT("実地形の有効な投影から断面を構成出来る"), Section.has_value())) continue;
							MaximumCut = std::max(MaximumCut, Here.ElevationMeters - Section->Terrain.ElevationMeters);
							if(std::abs(Across) == 6) {
								const double Excess = Section->Water.ElevationMeters - Section->Terrain.ElevationMeters;
								if(Excess > 1E-6) ++SubmergedEdges;
								MaximumEdgeWaterExcess = std::max(MaximumEdgeWaterExcess, Excess);
							}
							++ChannelPoints;
						}
					}
				}
				double PreviousHeight = Field(A).ElevationMeters;
				for(unsigned SampleIndex = 0; SampleIndex < 33; ++SampleIndex) {
					const PlanetCore::TerrainFlowCurveSample Sample = *Curve->Evaluate(SampleIndex / 32.0);
					const PlanetCore::TerrainSample Terrain = Field(Sample.Direction);
					const double Slope = Terrain.TangentialGradientPerRadian.Dot(Sample.DirectionDerivativePerMeter);
					if(Slope > 1E-10) ++UphillSamples;
					MaximumUphillDerivative = std::max(MaximumUphillDerivative, Slope);
					MaximumCurveRise = std::max(MaximumCurveRise, Terrain.ElevationMeters - PreviousHeight);
					PreviousHeight = Terrain.ElevationMeters;
					const Vec3 Tangent = UnitDirection::FromVector(Sample.DirectionDerivativePerMeter)->Vector();
					const Vec3 Down = UnitDirection::FromVector(Terrain.TangentialGradientPerRadian * -1)->Vector();
					const double AngleError = std::atan2(Tangent.Cross(Down).Length(), Tangent.Dot(Down));
					if(AngleError > MaximumDirectionError) {
						MaximumDirectionError = AngleError;
						WorstHeight = Terrain.ElevationMeters;
						WorstSlope = Terrain.TangentialGradientPerRadian.Length() / Definition.RadiusMeters;
						WorstLength = Curve->GetParameterLengthMeters();
						WorstFraction = SampleIndex / 32.0;
						WorstDistance = Flow->Samples[i - 1].DistanceMeters + WorstLength * WorstFraction;
					}
					++CurveSamples;
				}
				CurveDistanceMeters += Curve->GetParameterLengthMeters();
			}
			if(ShouldInspectCurvature) {
				TestTrue(TEXT("実地形上でも共有曲率の継ぎ目が無い"), MaximumCurvatureJoin < 1E-8);
				AddInfo(FString::Printf(TEXT("Local lake curvature join: sign=%.0f maximum=%.12g"), Sign, MaximumCurvatureJoin));
			}
			if(ShouldInspectChannels && Sign > 0) {
				AddInfo(
					FString::Printf(
						TEXT("Source channel index: queries=%u candidates=%u maximumCandidates=%u segments=%llu squaredDistanceError=%.12g"),
						RouteQueries,
						RouteCandidates,
						MaximumRouteCandidates,
						static_cast<unsigned long long>(Route->GetSegments().size()),
						MaximumIndexedDistanceError
					)
				);
				TestEqual(TEXT("試験河道の両岸まで投影を取得出来る"), MissingFootpoints, 0U);
				if(ShouldInspectChannelWater) TestEqual(TEXT("支持領域の両外縁が水面より低くない"), SubmergedEdges, 0U);
				TestTrue(TEXT("元地形の開削が指定深さを超えない"), MaximumCut <= 1.5 + 1E-8);
				AddInfo(
					FString::Printf(
						TEXT("Source channel sections: points=%u missingFootpoints=%u maximumCut=%.12g submergedEdges=%u maximumEdgeWaterExcess=%.12g"),
						ChannelPoints,
						MissingFootpoints,
						MaximumCut,
						SubmergedEdges,
						MaximumEdgeWaterExcess
					)
				);
			}
			TestEqual(TEXT("実際の下降点列を全区間で曲線化出来る"), RejectedCurves, 0U);
			TestEqual(TEXT("実河道候補の曲線内部に正の勾配を作らない"), UphillSamples, 0U);
			TestTrue(TEXT("曲線の元標高に上り返しを作らない"), MaximumCurveRise < 1E-8);
			AddInfo(
				FString::Printf(
					TEXT("Local lake flow curve: sign=%.0f rejected=%u samples=%u uphill=%u maximumSlope=%.12g ")
					TEXT("maximumRise=%.12g maximumDirectionError=%.12g"),
					Sign,
					RejectedCurves,
					CurveSamples,
					UphillSamples,
					MaximumUphillDerivative,
					MaximumCurveRise,
					MaximumDirectionError
				)
			);
			AddInfo(
				FString::Printf(
					TEXT("Local lake curve discrepancy: sign=%.0f tolerance=%.12g height=%.12g slope=%.12g segmentLength=%.12g ")
					TEXT("distance=%.12g fraction=%.12g"),
					Sign,
					FlowSettings.PositionToleranceMeters,
					WorstHeight,
					WorstSlope,
					WorstLength,
					WorstDistance,
					WorstFraction
				)
			);
			if(Distance >= 0.1) continue;
			// 真の溢流・流量は別途必要；鞍点直下の同じ盆地の岸だけを測る
			const double Water = BestPass->ElevationMeters - 0.01;
			UnitDirection Dry = BestPass->Direction;
			std::optional<UnitDirection> Wet;
			for(const PlanetCore::TerrainDescentSample &Sample : Flow->Samples) {
				if(Sample.ElevationMeters <= Water) {
					Wet = Sample.Direction;
					break;
				}
				Dry = Sample.Direction;
			}
			if(!TestTrue(TEXT("同じ湖の岸を挟む下降曲線が有る"), Wet.has_value())) continue;
			for(unsigned Iteration = 0; Iteration < 48; ++Iteration) {
				const UnitDirection Middle = *UnitDirection::FromVector(Dry.Vector() + Wet->Vector());
				if(Field(Middle).ElevationMeters > Water) Dry = Middle;
				else Wet = Middle;
			}
			PlanetCore::TerrainContourSettings ShoreSettings;
			ShoreSettings.RadiusMeters = Definition.RadiusMeters;
			ShoreSettings.LevelMeters = Water;
			ShoreSettings.MaximumLengthMeters = 5000;
			ShoreSettings.MaximumVertices = 4096;
			const std::optional<PlanetCore::TerrainContour> Shore = PlanetCore::TraceTerrainContour(*Wet, Field, ShoreSettings);
			if(!TestTrue(TEXT("流出口近くの岸を取得出来る"), Shore.has_value())) continue;
			const std::optional<double> Area = PlanetCore::MeasureTerrainContourArea(*Shore, Definition.RadiusMeters);
			AddInfo(
				FString::Printf(
					TEXT("Local lake shore: water=%.12g end=%u vertices=%llu length=%.12g signedArea=%.12g ")
					TEXT("heightResidual=%.12g start=(%.17g,%.17g,%.17g)"),
					Water,
					static_cast<unsigned>(Shore->End),
					static_cast<unsigned long long>(Shore->Samples.size()),
					Shore->Samples.back().DistanceMeters,
					Area.value_or(0),
					Shore->MaximumHeightResidualMeters,
					Wet->Vector().X,
					Wet->Vector().Y,
					Wet->Vector().Z
				)
			);
		}
		TestEqual(TEXT("鞍点の一方だけが表示した小窪地へ戻る"), ReturningBranches, 1U);
		// 局所的な接続の検査結果の返戻；大洋接続又は正の流量の保証ではない
		return !HasAnyErrors();
	}
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 対応する標本網が無い場合の返戻
	if(!TestTrue(TEXT("湖の診断用全球網を準備"), Mesh.has_value())) return false;
	const std::size_t Count = Mesh->Directions.size();
	const double Sea = Definition.SeaLevelMeters;
	std::vector<SpillGraphSite> Sites;
	std::vector<std::array<std::uint32_t, 2>> Pairs;
	std::vector<double> Areas(Count, 0);
	for(const UnitDirection Direction : Mesh->Directions) {
		const double Height = Ocean->EvaluateElevation(Direction);
		Sites.push_back({ Height, Height <= Sea });
	}
	// 同じ球面三角形の面積と接続の抽出
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
		const Vec3 A = Mesh->Directions[Triangle[0]].Vector(), B = Mesh->Directions[Triangle[1]].Vector();
		const Vec3 C = Mesh->Directions[Triangle[2]].Vector();
		const double Area = 2 * std::atan2(std::abs(A.Dot(B.Cross(C))), 1 + A.Dot(B) + B.Dot(C) + C.Dot(A)) / 3;
		for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t First = Triangle[Side], Second = Triangle[(Side + 1) % 3];
			Areas[First] += Area;
			Pairs.push_back({ std::min(First, Second), std::max(First, Second) });
		}
	}
	std::sort(Pairs.begin(), Pairs.end());
	Pairs.erase(std::unique(Pairs.begin(), Pairs.end()), Pairs.end());
	// 端点だけの比較と，途中７点を加えた通路の標本上限
	std::vector<SpillGraphEdge> Edges, EndpointEdges;
	std::vector<std::vector<std::uint32_t>> OceanNeighbors(Count);
	std::uint32_t RaisedCrests = 0;
	double MaximumCrestDifference = 0;
	for(const std::array<std::uint32_t, 2> &Pair : Pairs) {
		const Vec3 A = Mesh->Directions[Pair[0]].Vector(), B = Mesh->Directions[Pair[1]].Vector();
		const double EndpointCrest = std::max(Sites[Pair[0]].ElevationMeters, Sites[Pair[1]].ElevationMeters);
		double Crest = EndpointCrest;
		for(unsigned Sample = 1; Sample < 8; ++Sample) {
			const double Fraction = Sample / 8.0;
			Crest = std::max(Crest, Ocean->EvaluateElevation(*UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction)));
		}
		Edges.push_back({ Pair, Crest });
		EndpointEdges.push_back({ Pair, EndpointCrest });
		if(Crest > EndpointCrest + 1E-7) {
			++RaisedCrests;
			MaximumCrestDifference = std::max(MaximumCrestDifference, Crest - EndpointCrest);
		}
		if(Crest <= Sea) {
			OceanNeighbors[Pair[0]].push_back(Pair[1]);
			OceanNeighbors[Pair[1]].push_back(Pair[0]);
		}
	}
	// 元海域の非接続を湖と呼替えず，別の不合格条件として計数
	std::vector<bool> Seen(Count, false);
	std::vector<std::uint32_t> Queue, LargestOcean;
	std::uint32_t OceanComponents = 0, OceanNodes = 0;
	for(std::uint32_t i = 0; i < Count; ++i) {
		if(!Sites[i].IsOcean) continue;
		++OceanNodes;
		if(Seen[i]) continue;
		++OceanComponents;
		Queue.clear();
		Queue.push_back(i);
		Seen[i] = true;
		for(std::size_t j = 0; j < Queue.size(); ++j) for(const std::uint32_t Adjacent : OceanNeighbors[Queue[j]]) {
			if(Seen[Adjacent]) continue;
			Seen[Adjacent] = true;
			Queue.push_back(Adjacent);
		}
		if(Queue.size() > LargestOcean.size()) LargestOcean = Queue;
	}
	const std::optional<SpillGraph> AllOcean = PlanetCore::BuildSpillGraph(Sites, Edges, Sea);
	TestEqual(TEXT("途中の堰で非接続な大洋指定は受理しない"), AllOcean.has_value(), OceanComponents == 1);
	for(SpillGraphSite &Site : Sites) Site.IsOcean = false;
	for(const std::uint32_t Index : LargestOcean) Sites[Index].IsOcean = true;
	// 主大洋への飽和限界を診断し，副水域の非接続数は別に保持
	const std::chrono::steady_clock::time_point SolveStarted = std::chrono::steady_clock::now();
	std::optional<SpillGraph> Graph = PlanetCore::BuildSpillGraph(Sites, Edges, Sea);
	const double SolveMilliseconds =
	std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - SolveStarted).count();
	const std::optional<SpillGraph> EndpointGraph = PlanetCore::BuildSpillGraph(Sites, EndpointEdges, Sea);
	// 同一の有効な主大洋から準備出来ない場合の返戻
	if(!TestTrue(TEXT("主大洋へ向かう静水位の診断を準備"), Graph.has_value() && EndpointGraph.has_value())) return false;
	// 包含木と溢流接続の実地形上の比較
	if(Parameters == TEXT("Hierarchy")) {
		const std::chrono::steady_clock::time_point HierarchyStarted = std::chrono::steady_clock::now();
		const std::optional<PlanetCore::DepressionHierarchy> Hierarchy = PlanetCore::BuildDepressionHierarchy(Sites, Edges, Sea);
		const double HierarchyMilliseconds =
		std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - HierarchyStarted).count();
		// 同じ有効な元地形の階層を構成出来ない場合の返戻
		if(!TestTrue(TEXT("実地形の包含と溢流を分離"), Hierarchy.has_value())) return false;
		// 根の水位と，別実装の最小最大路の照合
		std::vector<std::uint32_t> Depths(Hierarchy->Nodes.size(), 0);
		std::uint32_t Roots = 0, WetRoots = 0, NestedNodes = 0, MaximumTreeDepth = 0;
		double MaximumWaterDifference = 0;
		for(std::uint32_t i = 0; i < Count; ++i) {
			MaximumWaterDifference = std::max(
				MaximumWaterDifference,
				std::abs(Hierarchy->Nodes[Hierarchy->SiteRoots[i]].SpillMeters - Graph->Nodes[i].EscapeMeters)
			);
		}
		for(std::uint32_t End = static_cast<std::uint32_t>(Hierarchy->Nodes.size()); End > 1; --End) {
			const std::uint32_t Index = End - 1;
			const PlanetCore::DepressionHierarchyNode &Node = Hierarchy->Nodes[Index];
			if(Node.Parent != PlanetCore::MissingSpillIndex) {
				// 不正な包含順が有る場合の返戻
				if(!TestTrue(TEXT("実地形の包含順は非循環"), Node.Parent > Index && Node.Parent < Hierarchy->Nodes.size())) return false;
				Depths[Index] = Depths[Node.Parent] + 1;
				MaximumTreeDepth = std::max(MaximumTreeDepth, Depths[Index]);
				++NestedNodes;
				continue;
			}
			++Roots;
			if(Node.MinimumMeters < Node.SpillMeters) ++WetRoots;
			// 実出口の欠落時の返戻
			if(!TestTrue(TEXT("実地形の根に流出口が在る"), Node.OutletSite < Count && Node.ReceivingSite < Count)) return false;
			// 下流へ向かう水位と接続順
			const PlanetCore::DepressionHierarchyNode &Receiver = Hierarchy->Nodes[Hierarchy->SiteRoots[Node.ReceivingSite]];
			TestEqual(TEXT("実流出口の所属は元の根"), Hierarchy->SiteRoots[Node.OutletSite], Index);
			TestTrue(TEXT("実地形の下流接続は非循環"), Receiver.OceanLinkRank < Node.OceanLinkRank);
			TestTrue(TEXT("実地形の水位は下流へ非増加"), Receiver.SpillMeters <= Node.SpillMeters);
		}
		TestEqual(TEXT("全地点の飽和位が別算法と一致"), MaximumWaterDifference, 0.0);
		AddInfo(
			FString::Printf(
				TEXT("Spill hierarchy survey: sites=%llu treeNodes=%llu roots=%u wetRoots=%u nestedNodes=%u ")
				TEXT("maximumTreeDepth=%u maximumWaterDifference=%.12g solveMs=%.9g"),
				static_cast<unsigned long long>(Count),
				static_cast<unsigned long long>(Hierarchy->Nodes.size()),
				Roots,
				WetRoots,
				NestedNodes,
				MaximumTreeDepth,
				MaximumWaterDifference,
				HierarchyMilliseconds
			)
		);
	}
	// 選択経路だけを再測定し，出口が変われば新しい経路も検査する比較
	std::uint64_t RefinedEvaluations = 0;
	std::uint32_t RefinedEdges = 0, RefinementPasses = 0;
	bool HasStableRefinement = !ShouldRefineRoutes;
	if(ShouldRefineRoutes) {
		std::vector<bool> IsMeasured(Edges.size(), false);
		for(unsigned Pass = 0; Pass < 32; ++Pass) {
			++RefinementPasses;
			std::vector<std::uint32_t> Pending;
			for(std::uint32_t i = 0; i < Count; ++i) {
				if(Sites[i].IsOcean) continue;
				const std::uint32_t EdgeIndex = Graph->Nodes[i].DownstreamEdge;
				if(IsMeasured[EdgeIndex]) continue;
				IsMeasured[EdgeIndex] = true;
				Pending.push_back(EdgeIndex);
			}
			if(Pending.empty()) {
				HasStableRefinement = true;
				break;
			}
			for(const std::uint32_t EdgeIndex : Pending) {
				SpillGraphEdge &Edge = Edges[EdgeIndex];
				Edge.CrestMeters = std::max(
					Edge.CrestMeters,
					MeasureSpillPassageCrest(
						*Ocean,
						Mesh->Directions[Edge.Nodes[0]],
						Mesh->Directions[Edge.Nodes[1]],
						Definition.RadiusMeters,
						RefinedEvaluations
					)
				);
			}
			RefinedEdges += static_cast<std::uint32_t>(Pending.size());
			Graph = PlanetCore::BuildSpillGraph(Sites, Edges, Sea);
			// 同じ主大洋へ接続する改訂経路を準備出来ない場合の返戻
			if(!TestTrue(TEXT("通路を詳しく測った排水を準備"), Graph.has_value())) return false;
		}
	}
	TestTrue(TEXT("選択経路の再測定が予算内で安定"), HasStableRefinement);
	std::uint32_t WetLandNodes = 0, Lakes = 0, DeepLakes = 0, ChangedLevels = 0;
	double MaximumDepth = 0, LandArea = 0, WetArea = 0, MaximumLevelDifference = 0, MaximumSourceChange = 0;
	for(const PlanetCore::SpillGraphRegion &Region : Graph->Regions) if(!Region.IsOcean) ++Lakes;
	std::vector<double> RegionDepths(Graph->Regions.size(), 0);
	for(std::uint32_t i = 0; i < Count; ++i) {
		const PlanetCore::SpillGraphNode &Node = Graph->Nodes[i];
		const double Depth = Node.EscapeMeters - Sites[i].ElevationMeters;
		if(Sites[i].ElevationMeters > Sea) {
			LandArea += Areas[i];
			if(Depth > 0) {
				++WetLandNodes;
				WetArea += Areas[i];
				MaximumDepth = std::max(MaximumDepth, Depth);
			}
		}
		if(Node.Region != PlanetCore::MissingSpillIndex) RegionDepths[Node.Region] = std::max(RegionDepths[Node.Region], Depth);
		if(Node.EscapeMeters > EndpointGraph->Nodes[i].EscapeMeters + 1E-7) {
			++ChangedLevels;
			MaximumLevelDifference = std::max(MaximumLevelDifference, Node.EscapeMeters - EndpointGraph->Nodes[i].EscapeMeters);
		}
		if(!Sites[i].IsOcean) TestTrue(TEXT("実惑星でも出口側の確定順は常に先"), Graph->Nodes[Node.Downstream].SettlementRank < Node.SettlementRank);
		if(!(i % 127)) {
			MaximumSourceChange =
			std::max(MaximumSourceChange, std::abs(Ocean->EvaluateElevation(Mesh->Directions[i]) - Sites[i].ElevationMeters));
		}
	}
	for(std::size_t i = 0; i < Graph->Regions.size(); ++i) if(!Graph->Regions[i].IsOcean && RegionDepths[i] > 100) ++DeepLakes;
	TestEqual(TEXT("元の連続標高を変更しない"), MaximumSourceChange, 0.0);
	// 準備標本と一致しない内部位置で，通路上限と水位の未達を診断
	std::uint32_t IndependentSamples = 0, MissedCrests = 0, BlockedPassages = 0;
	double MaximumMissedCrest = 0, MaximumWaterDeficit = 0;
	for(std::uint32_t i = 0; i < Count; i += 17) {
		if(Sites[i].IsOcean) continue;
		const PlanetCore::SpillGraphNode &Node = Graph->Nodes[i];
		const Vec3 A = Mesh->Directions[i].Vector(), B = Mesh->Directions[Node.Downstream].Vector();
		for(unsigned Sample = 1; Sample < 32; Sample += 2) {
			const double Fraction = Sample / 32.0;
			const double Height = Ocean->EvaluateElevation(*UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction));
			const double CrestDeficit = Height - Edges[Node.DownstreamEdge].CrestMeters, WaterDeficit = Height - Node.EscapeMeters;
			if(CrestDeficit > 1E-7) {
				++MissedCrests;
				MaximumMissedCrest = std::max(MaximumMissedCrest, CrestDeficit);
			}
			if(WaterDeficit > 1E-7) {
				++BlockedPassages;
				MaximumWaterDeficit = std::max(MaximumWaterDeficit, WaterDeficit);
			}
			++IndependentSamples;
		}
	}
	// 大きな窪地の通路上の峠を，元の連続地形の停留点と比較する診断
	if(ShouldInspectSaddles) {
		std::vector<std::uint32_t> Candidates;
		for(std::uint32_t i = 0; i < Graph->Regions.size(); ++i) {
			if(!Graph->Regions[i].IsOcean && RegionDepths[i] > 100) Candidates.push_back(i);
		}
		std::sort(
			Candidates.begin(),
			Candidates.end(),
			[&](const std::uint32_t A, const std::uint32_t B) -> bool {
				// 深い候補から安定な順に検査する為の比較結果の返戻
				return RegionDepths[A] != RegionDepths[B] ? RegionDepths[A] > RegionDepths[B] : A < B;
			}
		);
		if(Candidates.size() > 16) Candidates.resize(16);
		std::uint32_t Saddles = 0, OtherCriticalPoints = 0, Unresolved = 0, CurvatureDisagreements = 0;
		std::uint64_t PeakEvaluations = 0;
		double MaximumMove = 0, MaximumSpillDifference = 0;
		const auto Field = [&](const UnitDirection Direction) -> PlanetCore::TerrainSample {
			// 変更しない元の連続地形の返戻
			return Ocean->Evaluate(Direction);
		};
		for(const std::uint32_t Index : Candidates) {
			const PlanetCore::SpillGraphRegion &Region = Graph->Regions[Index];
			const SpillGraphEdge &Edge = Edges[Region.OutletEdge];
			UnitDirection Peak = Mesh->Directions[Edge.Nodes[0]];
			const double Crest =
			MeasureSpillPassageCrest(*Ocean, Peak, Mesh->Directions[Edge.Nodes[1]], Definition.RadiusMeters, PeakEvaluations, &Peak);
			PlanetCore::TerrainCriticalSettings Settings;
			Settings.RadiusMeters = Definition.RadiusMeters;
			const std::optional<PlanetCore::TerrainCriticalPoint> Critical = PlanetCore::RefineTerrainCriticalPoint(Peak, Field, Settings);
			if(!Critical) {
				++Unresolved;
				continue;
			}
			if(Critical->Kind != PlanetCore::TerrainCriticalKind::Saddle) {
				++OtherCriticalPoints;
				continue;
			}
			++Saddles;
			const Vec3 Point = Critical->Direction.Vector(), Along = Critical->MinimumCurvatureDirection, Across = Point.Cross(Along);
			const double Move = Definition.RadiusMeters * std::atan2(Peak.Vector().Cross(Point).Length(), Peak.Vector().Dot(Point));
			MaximumMove = std::max(MaximumMove, Move);
			MaximumSpillDifference = std::max(MaximumSpillDifference, std::abs(Crest - Critical->ElevationMeters));
			bool HasCorrectHeightOrder = true;
			for(const double Sign : { -1.0, 1.0 }) {
				const double DownHeight =
				Ocean->EvaluateElevation(*UnitDirection::FromVector(Point + Along * (Sign * 0.25 / Definition.RadiusMeters)));
				const double UpHeight =
				Ocean->EvaluateElevation(*UnitDirection::FromVector(Point + Across * (Sign * 0.25 / Definition.RadiusMeters)));
				HasCorrectHeightOrder = HasCorrectHeightOrder && DownHeight < Critical->ElevationMeters && UpHeight > Critical->ElevationMeters;
			}
			if(!HasCorrectHeightOrder) ++CurvatureDisagreements;
			AddInfo(
				FString::Printf(
					TEXT("Spill saddle: region=%u depth=%.12g arcCrest=%.12g saddleHeight=%.12g move=%.12g slope=%.12g ")
					TEXT("minimumCurvature=%.12g maximumCurvature=%.12g direction=(%.17g,%.17g,%.17g) valueOrder=%d"),
					Index,
					RegionDepths[Index],
					Crest,
					Critical->ElevationMeters,
					Move,
					Critical->GradientNorm,
					Critical->MinimumCurvaturePerMeter,
					Critical->MaximumCurvaturePerMeter,
					Point.X,
					Point.Y,
					Point.Z,
					HasCorrectHeightOrder
				)
			);
			if(ShouldInspectPassages) for(unsigned Variant = 0; Variant < 3; ++Variant) {
				// 幅を広げる比較では標本間隔を揃え，解像度比較とは混同しない
				const double Spacing = ShouldExpandPassages ? 250 : 500.0 / (1U << Variant);
				const double HalfWidth = ShouldExpandPassages ? (ShouldInspectBoundary ? 4000.0 : 8000.0) * (1U << Variant) : 4000;
				// 両端を保持した有限幅通路で小窪地の接続高さを測定
				const std::chrono::steady_clock::time_point PatchStarted = std::chrono::steady_clock::now();
				const std::optional<SpillCorridorSurvey> Patch = InspectSpillCorridor(
					*Ocean,
					Mesh->Directions[Edge.Nodes[0]],
					Mesh->Directions[Edge.Nodes[1]],
					Definition.RadiusMeters,
					Spacing,
					HalfWidth
				);
				if(!TestTrue(TEXT("局所通路の診断を構築出来る"), Patch.has_value())) continue;
				// 経路上の最大点と連続場の停留点は別に検査
				const std::optional<PlanetCore::TerrainCriticalPoint> Pass =
				PlanetCore::RefineTerrainCriticalPoint(Patch->Peak, Field, Settings);
				const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - PatchStarted).count();
				AddInfo(
					FString::Printf(
						TEXT("Spill corridor: region=%u spacing=%.0f halfWidth=%.0f nodes=%u pathNodes=%u evaluations=%llu seconds=%.9g ")
						TEXT("crest=%.12g length=%.12g independentExcess=%.12g boundary=%d criticalKind=%d criticalHeight=%.12g ")
						TEXT("peak=(%.17g,%.17g,%.17g) boundaryStart=%.12g boundaryEnd=%.12g outsideLowerBound=%.12g graphInterior=%d"),
						Index,
						Spacing,
						HalfWidth,
						Patch->Nodes,
						Patch->PathNodes,
						static_cast<unsigned long long>(Patch->Evaluations),
						Seconds,
						Patch->CrestMeters,
						Patch->PathLengthMeters,
						Patch->MaximumIndependentExcessMeters,
						Patch->TouchesBoundary,
						Pass ? static_cast<int>(Pass->Kind) : -1,
						Pass ? Pass->ElevationMeters : Patch->CrestMeters,
						Patch->Peak.Vector().X,
						Patch->Peak.Vector().Y,
						Patch->Peak.Vector().Z,
						Patch->BoundaryEscapeMeters[0],
						Patch->BoundaryEscapeMeters[1],
						Patch->OutsideLowerBoundMeters,
						Patch->IsGraphInteriorMinimum
					)
				);
			}
			// 鞍点両側の極小を流域の対応付けに使用
			std::array<std::optional<PlanetCore::TerrainCriticalPoint>, 2> BranchMinima;
			if(ShouldInspectDescent) for(const double Sign : { -1.0, 1.0 }) {
				// 鞍点の主方向へ５ m 離してから元場の下降曲線を追跡
				const UnitDirection Start = *UnitDirection::FromVector(Point + Along * (Sign * 5 / Definition.RadiusMeters));
				PlanetCore::TerrainDescentSettings FlowSettings;
				FlowSettings.RadiusMeters = Definition.RadiusMeters;
				const std::optional<PlanetCore::TerrainDescentPath> Path = PlanetCore::TraceTerrainDescent(Start, Field, FlowSettings);
				if(!Path) {
					AddWarning(TEXT("鞍点からの下降曲線を取得出来ない"));
					continue;
				}
				// 停止点を極小と断定せず，別の停留点検査を実施
				const PlanetCore::TerrainDescentSample &Last = Path->Samples.back();
				const std::optional<PlanetCore::TerrainCriticalPoint> End =
				PlanetCore::RefineTerrainCriticalPoint(Last.Direction, Field, Settings);
				const double MoveToCritical = End ?
				Definition.RadiusMeters * std::atan2(
					Last.Direction.Vector().Cross(End->Direction.Vector()).Length(),
					Last.Direction.Vector().Dot(End->Direction.Vector())
				) :
				-1;
				if(
					End && End->Kind == PlanetCore::TerrainCriticalKind::Minimum && MoveToCritical < 0.1 &&
					Path->End == PlanetCore::TerrainDescentEnd::GradientLimit
				) BranchMinima[Sign < 0 ? 0 : 1] = End;
				// 粗い網の近傍ラベルは参考値として別に記録
				std::uint32_t Nearest = 0;
				double NearestDot = -1;
				for(std::uint32_t Site = 0; Site < Count; ++Site) {
					const double Dot = Mesh->Directions[Site].Vector().Dot(Last.Direction.Vector());
					if(Dot > NearestDot) {
						NearestDot = Dot;
						Nearest = Site;
					}
				}
				// 元場を保持する事と既知の別流域候補との位置関係
				double SourceChange = 0;
				for(const PlanetCore::TerrainDescentSample &Sample : Path->Samples) {
					SourceChange = std::max(SourceChange, std::abs(Field(Sample.Direction).ElevationMeters - Sample.ElevationMeters));
				}
				TestEqual(TEXT("下降曲線も元地形を変更しない"), SourceChange, 0.0);
				const Vec3 LastDirection = Last.Direction.Vector();
				const double NearestDistance =
				Definition.RadiusMeters * std::atan2(LastDirection.Cross(Mesh->Directions[Nearest].Vector()).Length(), NearestDot);
				AddInfo(
					FString::Printf(
						TEXT("Spill descent: region=%u sign=%.0f points=%llu evaluations=%u rejected=%u reason=%u ")
						TEXT("length=%.12g height=%.12g slope=%.12g endCritical=%d nearestNode=%u nearestRegion=%u nearestDistance=%.12g ")
						TEXT("direction=(%.17g,%.17g,%.17g) sourceChange=%.12g"),
						Index,
						Sign,
						static_cast<unsigned long long>(Path->Samples.size()),
						Path->Evaluations,
						Path->RejectedSteps,
						static_cast<unsigned>(Path->End),
						Last.DistanceMeters,
						Last.ElevationMeters,
						Field(Last.Direction).TangentialGradientPerRadian.Length() / Definition.RadiusMeters,
						End ? static_cast<int>(End->Kind) : -1,
						Nearest,
						Graph->Nodes[Nearest].Region,
						NearestDistance,
						LastDirection.X,
						LastDirection.Y,
						LastDirection.Z,
						SourceChange
					)
				);
				if(ShouldInspectContours) {
					// 鞍点その物の分岐を避けた診断水位；通過湖の最終水位とはしない
					const double Water = Critical->ElevationMeters - 0.5;
					UnitDirection Dry = Critical->Direction;
					std::optional<UnitDirection> Wet;
					for(const PlanetCore::TerrainDescentSample &Sample : Path->Samples) {
						if(Sample.ElevationMeters <= Water) {
							Wet = Sample.Direction;
							break;
						}
						Dry = Sample.Direction;
					}
					if(!Wet) {
						AddWarning(TEXT("診断水位を挟む下降経路が無い"));
						continue;
					}
					// 同じ元場の交差へ絞り，三角形の辺を岸として用いない
					for(unsigned Iteration = 0; Iteration < 48; ++Iteration) {
						const UnitDirection Middle = *UnitDirection::FromVector(Dry.Vector() + Wet->Vector());
						if(Field(Middle).ElevationMeters > Water) Dry = Middle;
						else Wet = Middle;
					}
					PlanetCore::TerrainContourSettings ShoreSettings;
					ShoreSettings.RadiusMeters = Definition.RadiusMeters;
					ShoreSettings.LevelMeters = Water;
					const std::chrono::steady_clock::time_point ShoreStarted = std::chrono::steady_clock::now();
					const std::optional<PlanetCore::TerrainContour> Shore = PlanetCore::TraceTerrainContour(*Wet, Field, ShoreSettings);
					const double ShoreMilliseconds =
					std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ShoreStarted).count();
					if(!Shore) {
						AddWarning(TEXT("診断水位の等高線を準備出来ない"));
						continue;
					}
					// 元標高の保持と，中点以外も含む独立な折線標本
					double MaximumHeight = 0, ContourSourceChange = 0, MaximumIndependentOffset = 0;
					for(std::uint32_t Sample = 0; Sample < Shore->Samples.size(); ++Sample) {
						const PlanetCore::TerrainContourSample &ShorePoint = Shore->Samples[Sample];
						const double Height = Field(ShorePoint.Direction).ElevationMeters;
						MaximumHeight = std::max(MaximumHeight, std::abs(Height - Water));
						ContourSourceChange = std::max(ContourSourceChange, std::abs(Height - ShorePoint.ElevationMeters));
						if(!Sample) continue;
						const Vec3 A = Shore->Samples[Sample - 1].Direction.Vector(), B = ShorePoint.Direction.Vector();
						for(const double Fraction : { 0.25, 0.5, 0.75 }) {
							const UnitDirection Probe = *UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
							const PlanetCore::TerrainSample Value = Field(Probe);
							const double Slope = Value.TangentialGradientPerRadian.Length() / Definition.RadiusMeters;
							if(Slope > ShoreSettings.GradientTolerance) {
								MaximumIndependentOffset = std::max(MaximumIndependentOffset, std::abs(Value.ElevationMeters - Water) / Slope);
							}
						}
					}
					// 閉曲線の小さい側が低地か高地かを区別する符号付面積
					const std::optional<double> Area = PlanetCore::MeasureTerrainContourArea(*Shore, Definition.RadiusMeters);
					TestEqual(TEXT("実際の等高線も元地形を変更しない"), ContourSourceChange, 0.0);
					TestTrue(TEXT("実際の岸の頂点が指定水位へ一致"), MaximumHeight <= ShoreSettings.HeightToleranceMeters);
					AddInfo(
						FString::Printf(
							TEXT("Spill contour area: region=%u sign=%.0f measured=%d signedSquareMeters=%.12g"),
							Index,
							Sign,
							Area.has_value(),
							Area.value_or(0)
						)
					);
					AddInfo(
						FString::Printf(
							TEXT("Spill contour: region=%u sign=%.0f water=%.12g end=%u vertices=%llu length=%.12g evaluations=%u ")
							TEXT("rejected=%u maxHeight=%.12g independentOffset=%.12g sourceChange=%.12g solveMs=%.9g start=(%.17g,%.17g,%.17g)"),
							Index,
							Sign,
							Water,
							static_cast<unsigned>(Shore->End),
							static_cast<unsigned long long>(Shore->Samples.size()),
							Shore->Samples.back().DistanceMeters,
							Shore->Evaluations,
							Shore->RejectedSteps,
							MaximumHeight,
							MaximumIndependentOffset,
							ContourSourceChange,
							ShoreMilliseconds,
							Wet->Vector().X,
							Wet->Vector().Y,
							Wet->Vector().Z
						)
					);
				}
			}
			if(ShouldInspectTopology) {
				// 元の網の両端も同じ連続勾配で降ろし，単なる最近傍ラベルと区別
				std::array<std::optional<PlanetCore::TerrainCriticalPoint>, 2> EndpointMinima;
				for(unsigned Side = 0; Side < 2; ++Side) {
					// 元の辺の片側から開始する独立な経路
					PlanetCore::TerrainDescentSettings FlowSettings;
					FlowSettings.RadiusMeters = Definition.RadiusMeters;
					const std::optional<PlanetCore::TerrainDescentPath> Path =
					PlanetCore::TraceTerrainDescent(Mesh->Directions[Edge.Nodes[Side]], Field, FlowSettings);
					if(!Path) continue;
					const PlanetCore::TerrainDescentSample &Last = Path->Samples.back();
					const std::optional<PlanetCore::TerrainCriticalPoint> End =
					PlanetCore::RefineTerrainCriticalPoint(Last.Direction, Field, Settings);
					const double MoveToCritical = End ?
					Definition.RadiusMeters * std::atan2(
						Last.Direction.Vector().Cross(End->Direction.Vector()).Length(),
						Last.Direction.Vector().Dot(End->Direction.Vector())
					) :
					-1;
					if(
						End && End->Kind == PlanetCore::TerrainCriticalKind::Minimum && MoveToCritical < 0.1 &&
						Path->End == PlanetCore::TerrainDescentEnd::GradientLimit
					) EndpointMinima[Side] = End;
					AddInfo(
						FString::Printf(
							TEXT("Spill endpoint descent: region=%u side=%u node=%u reason=%u length=%.12g height=%.12g ")
							TEXT("moveToCritical=%.12g minimum=%d direction=(%.17g,%.17g,%.17g)"),
							Index,
							Side,
							Edge.Nodes[Side],
							static_cast<unsigned>(Path->End),
							Last.DistanceMeters,
							Last.ElevationMeters,
							MoveToCritical,
							EndpointMinima[Side].has_value(),
							Last.Direction.Vector().X,
							Last.Direction.Vector().Y,
							Last.Direction.Vector().Z
						)
					);
				}
				// 同じ極小へ着く枝の対応は１０ cm の位置精度で診断
				std::array<std::array<double, 2>, 2> Distances { { { -1, -1 }, { -1, -1 } } };
				for(unsigned Side = 0; Side < 2; ++Side) for(unsigned Branch = 0; Branch < 2; ++Branch) {
					if(!EndpointMinima[Side] || !BranchMinima[Branch]) continue;
					const Vec3 A = EndpointMinima[Side]->Direction.Vector(), B = BranchMinima[Branch]->Direction.Vector();
					Distances[Side][Branch] = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
				}
				const auto Matches = [&](const unsigned Side, const unsigned Branch) -> bool {
					// 独立に精査した同じ極小の判定結果の返戻
					return Distances[Side][Branch] >= 0 && Distances[Side][Branch] < 0.1;
				};
				const bool ConnectsEndpoints = Matches(0, 0) && Matches(1, 1) || Matches(0, 1) && Matches(1, 0);
				AddInfo(
					FString::Printf(
						TEXT("Spill saddle topology: region=%u d00=%.12g d01=%.12g d10=%.12g d11=%.12g paired=%d"),
						Index,
						Distances[0][0],
						Distances[0][1],
						Distances[1][0],
						Distances[1][1],
						ConnectsEndpoints
					)
				);
			}
		}
		AddInfo(
			FString::Printf(
				TEXT("Spill saddle survey: candidates=%llu saddles=%u otherCritical=%u unresolved=%u valueDisagreements=%u ")
				TEXT("peakEvaluations=%llu maxMove=%.12g maxSpillDifference=%.12g"),
				static_cast<unsigned long long>(Candidates.size()),
				Saddles,
				OtherCriticalPoints,
				Unresolved,
				CurvatureDisagreements,
				static_cast<unsigned long long>(PeakEvaluations),
				MaximumMove,
				MaximumSpillDifference
			)
		);
	}
	const double Elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Spill terrain survey: seed=42 radius=1000000 nodes=%llu edges=%llu seconds=%.9g solveMs=%.9g"),
			static_cast<unsigned long long>(Count),
			static_cast<unsigned long long>(Edges.size()),
			Elapsed,
			SolveMilliseconds
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spill source oceans: components=%u nodes=%u mainNodes=%llu disconnectedNodes=%llu"),
			OceanComponents,
			OceanNodes,
			static_cast<unsigned long long>(LargestOcean.size()),
			static_cast<unsigned long long>(OceanNodes - LargestOcean.size())
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spill lake candidates: lakes=%u wetLandNodes=%u wetNodeAreaFraction=%.12g maxDepth=%.12g deepLakes=%u dryFlatEdges=%u"),
			Lakes,
			WetLandNodes,
			WetArea / LandArea,
			MaximumDepth,
			DeepLakes,
			Graph->DryFlatEdges
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spill crest comparison: raisedEdges=%u maxDifference=%.12g changedLevels=%u maxLevelDifference=%.12g sourceChange=%.12g"),
			RaisedCrests,
			MaximumCrestDifference,
			ChangedLevels,
			MaximumLevelDifference,
			MaximumSourceChange
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spill independent samples: count=%u missedCrests=%u maxMiss=%.12g blocked=%u maxDeficit=%.12g"),
			IndependentSamples,
			MissedCrests,
			MaximumMissedCrest,
			BlockedPassages,
			MaximumWaterDeficit
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spill route refinement: enabled=%d passes=%u edges=%u evaluations=%llu stable=%d"),
			ShouldRefineRoutes,
			RefinementPasses,
			RefinedEdges,
			static_cast<unsigned long long>(RefinedEvaluations),
			HasStableRefinement
		)
	);
	// 網の診断結果の返戻
	return !HasAnyErrors();
}

#endif
