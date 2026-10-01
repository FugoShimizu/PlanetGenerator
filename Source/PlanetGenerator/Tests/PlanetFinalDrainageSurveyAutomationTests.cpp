#include "Core/PlanetAeolianField.h"
#include "Core/PlanetDrainage.h"
#include "Core/PlanetErodedTerrain.h"
#include "Core/PlanetErosionPreparation.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetOutletPotential.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <span>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS
namespace {
	struct DrainageSurveyStatistics {
		std::uint32_t DepressionNodes = 0;
		std::array<std::uint32_t, 5> FillDepthCounts{}, CutDepthCounts{};
		double MaximumFillMeters = 0;
		double SumFillMeters = 0;
		std::uint32_t CutNodes = 0;
		double MaximumCutMeters = 0;
		double SumCutMeters = 0;
	};

	/**
	 * 窪地と開削の深さを物理距離の階級へ写す
	 * @param Meters 正の深さ又は標高差
	 * @return １／５／２０／１００mを境界とする番号
	 */
	std::size_t DepthBucket(const double Meters) {
		// 各階級の代表番号の返戻
		return Meters < 1 ? 0 : Meters < 5 ? 1 : Meters < 20 ? 2 : Meters < 100 ? 3 : 4;
	}
	/**
	 * 元の大洋を起点とする最小越流高と零勾配の開削包絡を測る
	 * @param Heights 評価段階の標高
	 * @param Nodes 元の大洋を示す排水節点
	 * @param Adjacency 固定球面網の辺
	 * @param Sea 基準海面
	 * @return 全頂点へ経路が有る場合の試料統計
	 */
	std::optional<DrainageSurveyStatistics> SurveyDrainagePaths(
		const std::span<const double> Heights, const std::span<const PlanetCore::DrainageNode> Nodes,
		const std::vector<std::vector<std::uint32_t>> &Adjacency, const double Sea) {
		if(Adjacency.size() > std::numeric_limits<std::uint32_t>::max() || Heights.size() < Adjacency.size() ||
		   Nodes.size() < Adjacency.size()) return {};
		const std::uint32_t Count = static_cast<std::uint32_t>(Adjacency.size());
		using QueueEntry = std::pair<double, std::uint32_t>;
		std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> Frontier;
		std::vector<double> SpillHeight(Count, std::numeric_limits<double>::infinity());
		std::vector<std::uint32_t> Parent(Count, std::numeric_limits<std::uint32_t>::max()), PopOrder;
		PopOrder.reserve(Count);
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(!Nodes[Index].Ocean || Heights[Index] >= Sea) continue;
			SpillHeight[Index] = Sea;
			Parent[Index] = Index;
			Frontier.emplace(Sea, Index);
		}
		if(Frontier.empty()) return {};
		while(!Frontier.empty()) {
			const QueueEntry Current = Frontier.top();
			Frontier.pop();
			if(Current.first != SpillHeight[Current.second]) continue;
			PopOrder.push_back(Current.second);
			for(const std::uint32_t Neighbor : Adjacency[Current.second]) {
				const double Candidate = std::max(Current.first, Heights[Neighbor]);
				if(Candidate >= SpillHeight[Neighbor]) continue;
				SpillHeight[Neighbor] = Candidate;
				Parent[Neighbor] = Current.second;
				Frontier.emplace(Candidate, Neighbor);
			}
		}
		if(PopOrder.size() != Count) return {};
		std::vector<double> CutHeight(Heights.begin(), Heights.begin() + Count);
		for(std::size_t Position = PopOrder.size(); Position > 0; --Position) {
			const std::uint32_t Index = PopOrder[Position - 1], Downstream = Parent[Index];
			if(Index != Downstream) CutHeight[Downstream] = std::min(CutHeight[Downstream], CutHeight[Index]);
		}
		DrainageSurveyStatistics Result;
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(!std::isfinite(SpillHeight[Index])) return {};
			if(Heights[Index] <= Sea + 0.01) continue;
			const double Fill = SpillHeight[Index] - Heights[Index];
			if(Fill <= 0.01) continue;
			++Result.DepressionNodes;
			++Result.FillDepthCounts[DepthBucket(Fill)];
			Result.MaximumFillMeters = std::max(Result.MaximumFillMeters, Fill);
			Result.SumFillMeters += Fill;
		}
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			const double Cut = Heights[Index] - CutHeight[Index];
			if(Cut <= 0.01) continue;
			++Result.CutNodes;
			++Result.CutDepthCounts[DepthBucket(Cut)];
			Result.MaximumCutMeters = std::max(Result.MaximumCutMeters, Cut);
			Result.SumCutMeters += Cut;
		}
		return Result;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetFinalDrainageSurvey, "PlanetGenerator.FinalDrainage.Survey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 最終 H の格子上の下流逆転と局所窪地を数え，受入保証との差を可視化する
 * @param Parameters UE 自動検査の引数
 * @return 計測を完了出来た場合に真，排水条件の合格は意味しない
 */
bool FPlanetFinalDrainageSurvey::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Started = FPlatformTime::Seconds();
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
	if(!TestTrue(TEXT("地質場を準備"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	if(!TestTrue(TEXT("海域接続場を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
		std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	if(!TestTrue(TEXT("元の排水網を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
		std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
	const std::optional<PlanetCore::RiverTerrainField> Field =
		PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	if(!TestTrue(TEXT("侵食を含む最終 H を準備"), Field.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	if(!TestTrue(TEXT("排水と同じ全球格子を準備"), Mesh.has_value())) return false;
	const std::uint32_t OriginalCount = static_cast<std::uint32_t>(Mesh->Directions.size());
	const std::span<const PlanetCore::DrainageNode> Nodes = Field->Drainage().Nodes();
	if(!TestTrue(TEXT("元の格子地点が排水網に保持される"), Nodes.size() >= Mesh->Directions.size())) return false;
	const std::shared_ptr<const PlanetCore::PlanetErosionPreparation> Erosion = Field->GetErosionPreparation();
	if(!TestTrue(TEXT("侵食の中間標高を保持"), static_cast<bool>(Erosion))) return false;
	const std::optional<PlanetCore::PlanetMesoRelief> Waves =
		PlanetCore::PlanetMesoRelief::Create(Settings.Seed, Settings.RadiusMeters);
	const std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(Field->Base());
	if(!TestTrue(TEXT("比較用の２種類の中距離場を準備"), Waves.has_value() && Ridges.has_value())) return false;
	std::uint32_t UndefinedRidgeWeightSites = 0;
	double MaximumRidgeAdditionMeters = 0;
	std::array<std::vector<double>, 8> StageHeights;
	for(std::vector<double> &Stage : StageHeights) Stage.reserve(Nodes.size());
	std::vector<double> Heights;
	Heights.reserve(Nodes.size());
	for(const PlanetCore::DrainageNode &Node : Nodes) {
		const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector(Node.Direction);
		if(!TestTrue(TEXT("排水地点の方向が有効"), Direction.has_value())) return false;
		StageHeights[0].push_back(Ocean->EvaluateElevation(*Direction));
		StageHeights[1].push_back(Erosion->GetTerrain()->EvaluateElevation(*Direction));
		StageHeights[2].push_back(Erosion->GetAeolianField()->EvaluateElevation(*Direction));
		const PlanetCore::RiverElevationStages Elevations = Field->EvaluateElevationStages(*Direction);
		StageHeights[3].push_back(Elevations.AfterValleyMeters);
		StageHeights[4].push_back(Elevations.AfterFineReliefMeters);
		const double WaveHeight = Waves->EvaluateElevation(*Direction);
		const double WaveContribution = Elevations.FinalMeters - Elevations.AfterFineReliefMeters;
		double DetailWeight = 0;
		if(WaveHeight > 1E-8) DetailWeight = WaveContribution / WaveHeight;
		else {
			if(!TestTrue(TEXT("零の波群から有限の付加を推定しない"), std::abs(WaveContribution) <= 1E-8)) return false;
			++UndefinedRidgeWeightSites;
		}
		const double MaximumWeight = Settings.RadiusMeters / (Settings.RadiusMeters + 250000);
		if(!TestTrue(TEXT("中距離の付加率が０から半径縮尺内"),
			std::isfinite(DetailWeight) && DetailWeight >= -1E-7 && DetailWeight <= MaximumWeight + 1E-7)) return false;
		const double RidgeAddition = Ridges->EvaluateElevation(*Direction) * std::clamp(DetailWeight, 0.0, MaximumWeight);
		MaximumRidgeAdditionMeters = std::max(MaximumRidgeAdditionMeters, RidgeAddition);
		StageHeights[5].push_back(Elevations.AfterFineReliefMeters + 0.25 * RidgeAddition);
		StageHeights[6].push_back(Elevations.AfterFineReliefMeters + 0.5 * RidgeAddition);
		StageHeights[7].push_back(Elevations.AfterFineReliefMeters + RidgeAddition);
		const double Height = Elevations.FinalMeters;
		if((Heights.size() & 255) == 0 &&
		   !TestTrue(TEXT("診断標高と公開標高がビット一致"),
		             std::bit_cast<std::uint64_t>(Height) ==
		                 std::bit_cast<std::uint64_t>(Field->EvaluateElevation(*Direction)))) return false;
		if(!TestTrue(TEXT("最終標高が有限"), std::isfinite(Height))) return false;
		for(const std::vector<double> &Stage : StageHeights) if(!TestTrue(TEXT("中間標高が有限"), std::isfinite(Stage.back()))) return false;
		Heights.push_back(Height);
	}
	std::vector<PlanetCore::DrainageSite> SourceSites(OriginalCount), OrogenicSites(OriginalCount);
	std::vector<std::array<std::uint32_t, 2>> Edges;
	Edges.reserve(Mesh->Triangles.size() * 3);
	const double SeaLevel = Field->Drainage().Settings().SeaLevelMeters;
	for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) {
		const PlanetCore::UnitDirection Direction = Mesh->Directions[Index];
		PlanetCore::DrainageSite &Source = SourceSites[Index];
		Source.Direction = Direction.Vector();
		Source.ElevationMeters = StageHeights[0][Index];
		Source.Ocean = Source.ElevationMeters <= SeaLevel;
		const double T = std::clamp((Source.ElevationMeters - SeaLevel) / 500.0, 0.0, 1.0);
		const double CoastalWeight = T * T * T * (10 + T * (-15 + 6 * T));
		OrogenicSites[Index] = Source;
		OrogenicSites[Index].ElevationMeters += CoastalWeight * Ridges->EvaluateElevation(Direction);
		if(!TestTrue(TEXT("前置造山でも海陸の分類を保存"),
			(OrogenicSites[Index].ElevationMeters <= SeaLevel) == Source.Ocean)) return false;
	}
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
		const PlanetCore::Vec3 A = SourceSites[Triangle[0]].Direction;
		const PlanetCore::Vec3 B = SourceSites[Triangle[1]].Direction;
		const PlanetCore::Vec3 C = SourceSites[Triangle[2]].Direction;
		const double Share = 2 * std::atan2(std::abs(A.Dot((B - A).Cross(C - A))),
			1 + A.Dot(B) + B.Dot(C) + C.Dot(A)) * Settings.RadiusMeters * Settings.RadiusMeters / 3;
		for(std::uint32_t Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t Site = Triangle[Corner], Neighbor = Triangle[(Corner + 1) % 3];
			SourceSites[Site].AreaSquareMeters += Share;
			OrogenicSites[Site].AreaSquareMeters += Share;
			Edges.push_back({std::min(Site, Neighbor), std::max(Site, Neighbor)});
		}
	}
	std::sort(Edges.begin(), Edges.end());
	Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
	const PlanetCore::DrainageSettings RouteSettings = Field->Drainage().Settings();
	const std::optional<PlanetCore::DrainageNetwork> BaseRoutes =
		PlanetCore::DrainageNetwork::Solve(SourceSites, Edges, RouteSettings);
	const std::optional<PlanetCore::DrainageNetwork> OrogenicRoutes =
		PlanetCore::DrainageNetwork::Solve(OrogenicSites, Edges, RouteSettings);
	if(!TestTrue(TEXT("同じ全球網で造山前後の排水を解く"), BaseRoutes.has_value() && OrogenicRoutes.has_value())) return false;
	std::uint32_t ChangedRouteSites = 0;
	for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) {
		if(BaseRoutes->Nodes()[Index].Downstream != OrogenicRoutes->Nodes()[Index].Downstream) ++ChangedRouteSites;
	}
	AddInfo(FString::Printf(
		TEXT("Pre-drainage orogenic graph: changedRoutes=%u baseCut=%.6fm ridgeCut=%.6fm ")
		TEXT("baseRaise=%.6fm ridgeRaise=%.6fm baseReaches=%u ridgeReaches=%u. ")
		TEXT("Fixed graph, no erosion or final continuous surface."),
		ChangedRouteSites, BaseRoutes->Statistics().MaximumRequiredWaterCutMeters,
		OrogenicRoutes->Statistics().MaximumRequiredWaterCutMeters,
		BaseRoutes->Statistics().MaximumRequiredBankRaiseMeters,
		OrogenicRoutes->Statistics().MaximumRequiredBankRaiseMeters,
		BaseRoutes->Statistics().Reaches, OrogenicRoutes->Statistics().Reaches));
std::vector<double> LowestNeighbor(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
	std::vector<std::vector<std::uint32_t>> Adjacency(Mesh->Directions.size());
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
		for(std::uint32_t Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
			LowestNeighbor[A] = std::min(LowestNeighbor[A], Heights[B]);
			LowestNeighbor[B] = std::min(LowestNeighbor[B], Heights[A]);
			Adjacency[A].push_back(B);
			Adjacency[B].push_back(A);
		}
	}
	std::uint32_t LocalSinks = 0, PlannedRouteRises = 0, NewlyFloodedSites = 0;
	std::array<std::uint32_t, 5> FinalSinkDepthCounts{};
	std::uint32_t DeepestSinkIndex = 0, SteepestRiseIndex = 0;
	double DeepestSinkMeters = 0, SteepestRiseMeters = 0;
	AddInfo(FString::Printf(TEXT("Orogenic comparison: undefinedWaveWeights=%u descriptors=%u maximumAddition=%.6fm. ")
		TEXT("Same inferred river mask; survey only."),
		UndefinedRidgeWeightSites, Ridges->Statistics().Descriptors, MaximumRidgeAdditionMeters));
	const double Sea = Field->Drainage().Settings().SeaLevelMeters;
	for(std::size_t Index = 0; Index < Nodes.size(); ++Index) {
		const PlanetCore::DrainageNode &Node = Nodes[Index];
		if(Node.Ocean) continue;
		if(Index < LowestNeighbor.size() && Heights[Index] > Sea + 0.01) {
			const double Depth = LowestNeighbor[Index] - Heights[Index];
			if(Depth > 0.01) {
				++LocalSinks;
				++FinalSinkDepthCounts[DepthBucket(Depth)];
				if(Depth > DeepestSinkMeters) {
					DeepestSinkMeters = Depth;
					DeepestSinkIndex = static_cast<std::uint32_t>(Index);
				}
			}
		}
		if(Heights[Index] < Sea - 0.01) ++NewlyFloodedSites;
		if(Node.Downstream >= Heights.size() || Node.Downstream == Index) continue;
		const double Rise = Heights[Node.Downstream] - Heights[Index];
		if(Rise > 0.01) {
			++PlannedRouteRises;
			if(Rise > SteepestRiseMeters) {
				SteepestRiseMeters = Rise;
				SteepestRiseIndex = static_cast<std::uint32_t>(Index);
			}
		}
	}
	const std::optional<DrainageSurveyStatistics> Spill = SurveyDrainagePaths(Heights, Nodes, Adjacency, Sea);
	if(!TestTrue(TEXT("最終格子から大洋への最小越流高を準備"), Spill.has_value())) return false;
	AddInfo(FString::Printf(
		TEXT("Final sampled spill survey: depressedNodes=%u maximumFill=%.6fm summedFill=%.6fm. ")
		TEXT("Graph-only minimax fill; no continuous drainage guarantee."),
		Spill->DepressionNodes, Spill->MaximumFillMeters, Spill->SumFillMeters));
	AddInfo(FString::Printf(
		TEXT("Final sampled cut envelope: cutNodes=%u maximumCut=%.6fm summedCut=%.6fm. ")
		TEXT("Zero-grade graph bound; not a continuous channel."),
		Spill->CutNodes, Spill->MaximumCutMeters, Spill->SumCutMeters));
	AddInfo(FString::Printf(
		TEXT("Final drainage survey: seed=42 radius=1000000 graph=%llu original=%llu localSinks=%u deepest=%.6fm at %u ")
		TEXT("plannedRouteRises=%u steepest=%.6fm at %u newlyFlooded=%u elapsed=%.3fs. Survey only; no global guarantee."),
		static_cast<unsigned long long>(Nodes.size()), static_cast<unsigned long long>(Mesh->Directions.size()),
		LocalSinks, DeepestSinkMeters, DeepestSinkIndex, PlannedRouteRises, SteepestRiseMeters, SteepestRiseIndex,
		NewlyFloodedSites, FPlatformTime::Seconds() - Started));
	AddInfo(FString::Printf(
		TEXT("Final depth bins [0.01,1,5,20,100,+inf)m: sinks=%u/%u/%u/%u/%u ")
		TEXT("fills=%u/%u/%u/%u/%u cuts=%u/%u/%u/%u/%u"),
		FinalSinkDepthCounts[0], FinalSinkDepthCounts[1], FinalSinkDepthCounts[2],
		FinalSinkDepthCounts[3], FinalSinkDepthCounts[4],
		Spill->FillDepthCounts[0], Spill->FillDepthCounts[1], Spill->FillDepthCounts[2],
		Spill->FillDepthCounts[3], Spill->FillDepthCounts[4],
		Spill->CutDepthCounts[0], Spill->CutDepthCounts[1], Spill->CutDepthCounts[2],
		Spill->CutDepthCounts[3], Spill->CutDepthCounts[4]));
	const TCHAR *const StageNames[] = {
		TEXT("ocean"), TEXT("eroded"), TEXT("aeolian"), TEXT("river_valley"), TEXT("fine_relief"),
		TEXT("orogenic_quarter"), TEXT("orogenic_half"), TEXT("orogenic_full")
	};
	for(std::size_t Stage = 0; Stage < StageHeights.size(); ++Stage) {
		const std::vector<double> &Values = StageHeights[Stage];
		std::vector<double> Neighbors(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
			for(std::uint32_t Corner = 0; Corner < 3; ++Corner) {
				const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
				Neighbors[A] = std::min(Neighbors[A], Values[B]);
				Neighbors[B] = std::min(Neighbors[B], Values[A]);
			}
		}
		std::uint32_t Sinks = 0, Rises = 0, Flooded = 0;
		std::array<std::uint32_t, 5> SinkDepthCounts{};
		double Deepest = 0, Steepest = 0;
		for(std::size_t Index = 0; Index < Nodes.size(); ++Index) {
			const PlanetCore::DrainageNode &Node = Nodes[Index];
			if(Node.Ocean) continue;
			if(Index < Neighbors.size() && Values[Index] > Sea + 0.01) {
				const double Depth = Neighbors[Index] - Values[Index];
				if(Depth > 0.01) {
					++Sinks;
					++SinkDepthCounts[DepthBucket(Depth)];
					Deepest = std::max(Deepest, Depth);
				}
			}
			if(Values[Index] < Sea - 0.01) ++Flooded;
			if(Node.Downstream >= Values.size() || Node.Downstream == Index) continue;
			const double Rise = Values[Node.Downstream] - Values[Index];
			if(Rise > 0.01) {
				++Rises;
				Steepest = std::max(Steepest, Rise);
			}
		}
		const std::optional<DrainageSurveyStatistics> StageSpill = SurveyDrainagePaths(Values, Nodes, Adjacency, Sea);
		if(!TestTrue(TEXT("中間標高から大洋への最小越流高を準備"), StageSpill.has_value())) return false;
		AddInfo(FString::Printf(TEXT("Drainage spill stage=%s depressedNodes=%u maximumFill=%.6fm summedFill=%.6fm"),
			StageNames[Stage], StageSpill->DepressionNodes, StageSpill->MaximumFillMeters, StageSpill->SumFillMeters));
		AddInfo(FString::Printf(TEXT("Drainage cut stage=%s cutNodes=%u maximumCut=%.6fm summedCut=%.6fm"),
			StageNames[Stage], StageSpill->CutNodes, StageSpill->MaximumCutMeters, StageSpill->SumCutMeters));
		AddInfo(FString::Printf(
			TEXT("Drainage stage=%s localSinks=%u deepest=%.6fm plannedRouteRises=%u steepest=%.6fm newlyFlooded=%u"),
			StageNames[Stage], Sinks, Deepest, Rises, Steepest, Flooded));
		AddInfo(FString::Printf(
			TEXT("Drainage depth stage=%s sinks=%u/%u/%u/%u/%u ")
			TEXT("fills=%u/%u/%u/%u/%u cuts=%u/%u/%u/%u/%u"),
			StageNames[Stage], SinkDepthCounts[0], SinkDepthCounts[1], SinkDepthCounts[2],
			SinkDepthCounts[3], SinkDepthCounts[4],
			StageSpill->FillDepthCounts[0], StageSpill->FillDepthCounts[1], StageSpill->FillDepthCounts[2],
			StageSpill->FillDepthCounts[3], StageSpill->FillDepthCounts[4],
			StageSpill->CutDepthCounts[0], StageSpill->CutDepthCounts[1], StageSpill->CutDepthCounts[2],
			StageSpill->CutDepthCounts[3], StageSpill->CutDepthCounts[4]));
	}
	std::vector<PlanetCore::DrainageSite> FinalSites(SourceSites.begin(), SourceSites.end());
	std::vector<std::vector<std::uint32_t>> UniqueNeighbors(OriginalCount);
	for(const std::array<std::uint32_t, 2> &Edge : Edges) {
		UniqueNeighbors[Edge[0]].push_back(Edge[1]);
		UniqueNeighbors[Edge[1]].push_back(Edge[0]);
	}
	std::vector<double> PositiveAltitudes(OriginalCount), CurvatureSources(OriginalCount, 1);
	std::vector<double> GeologicSources(OriginalCount, 1);
	double MaximumLandAltitude = 0, MaximumPositiveCurvature = 0;
	for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) {
		PositiveAltitudes[Index] = std::max(0.0, Heights[Index] - Sea);
		FinalSites[Index].Ocean = Heights[Index] <= Sea;
		MaximumLandAltitude = std::max(MaximumLandAltitude, PositiveAltitudes[Index]);
		GeologicSources[Index] += Ridges->EvaluateElevation(Mesh->Directions[Index]) / 100;
	}
	for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) {
		if(FinalSites[Index].Ocean) continue;
		double Curvature = UniqueNeighbors[Index].size() * PositiveAltitudes[Index];
		for(const std::uint32_t Neighbor : UniqueNeighbors[Index]) Curvature -= PositiveAltitudes[Neighbor];
		MaximumPositiveCurvature = std::max(MaximumPositiveCurvature, Curvature);
		CurvatureSources[Index] = std::max(1.0, Curvature);
	}
	const std::optional<PlanetCore::PlanetOutletPotential> CurvaturePotential =
		PlanetCore::PlanetOutletPotential::Create(FinalSites, Edges, CurvatureSources, MaximumLandAltitude);
	const std::optional<PlanetCore::PlanetOutletPotential> GeologicPotential =
		PlanetCore::PlanetOutletPotential::Create(FinalSites, Edges, GeologicSources, MaximumLandAltitude);
	if(!TestTrue(TEXT("最終海岸に合わせた正の隆起源を解く"),
		CurvaturePotential.has_value() && GeologicPotential.has_value())) return false;
	double CurvatureAbsoluteSum = 0, GeologicAbsoluteSum = 0, CurvatureMaximumChange = 0, GeologicMaximumChange = 0;
	std::uint32_t ComparedLandNodes = 0;
	for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) {
		if(FinalSites[Index].Ocean) continue;
		const double CurvatureError = std::abs(CurvaturePotential->Elevations()[Index] - PositiveAltitudes[Index]);
		const double GeologicError = std::abs(GeologicPotential->Elevations()[Index] - PositiveAltitudes[Index]);
		CurvatureAbsoluteSum += CurvatureError;
		GeologicAbsoluteSum += GeologicError;
		CurvatureMaximumChange = std::max(CurvatureMaximumChange, CurvatureError);
		GeologicMaximumChange = std::max(GeologicMaximumChange, GeologicError);
		++ComparedLandNodes;
	}
	AddInfo(FString::Printf(
		TEXT("Final-H Poisson alternatives: land=%u targetPeak=%.6fm positiveCurvature=%.6fm ")
		TEXT("curvatureMeanChange=%.6fm curvatureMaxChange=%.6fm geologyMeanChange=%.6fm ")
		TEXT("geologyMaxChange=%.6fm curvatureIterations=%u geologyIterations=%u. ")
		TEXT("Sampled graph comparison only; no final H replacement."),
		ComparedLandNodes, MaximumLandAltitude, MaximumPositiveCurvature,
		CurvatureAbsoluteSum / ComparedLandNodes, CurvatureMaximumChange,
		GeologicAbsoluteSum / ComparedLandNodes, GeologicMaximumChange,
		CurvaturePotential->Statistics().Iterations, GeologicPotential->Statistics().Iterations));
	std::uint32_t ReachGroundRises = 0;
	double MaximumReachRise = 0;
	for(const PlanetCore::RiverReach &Reach : Field->Drainage().Reaches()) {
		if(!TestTrue(TEXT("選択河道の両端が有効"), Reach.Upstream < Heights.size() && Reach.Downstream < Heights.size())) return false;
		const double Rise = Heights[Reach.Downstream] - Heights[Reach.Upstream];
		if(Rise > 0.01) {
			++ReachGroundRises;
			MaximumReachRise = std::max(MaximumReachRise, Rise);
		}
	}
	std::vector<std::uint32_t> Parents(OriginalCount);
	for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) Parents[Index] = Index;
	const auto FindRoot = [&Parents](std::uint32_t Index) {
		while(Parents[Index] != Index) {
			Parents[Index] = Parents[Parents[Index]];
			Index = Parents[Index];
		}
		return Index;
	};
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
		for(std::uint32_t Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
			if(Heights[A] >= Sea || Heights[B] >= Sea) continue;
			const std::uint32_t RootA = FindRoot(A), RootB = FindRoot(B);
			if(RootA != RootB) Parents[RootB] = RootA;
		}
	}
	std::vector<bool> HasOriginalOcean(OriginalCount, false);
	for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) {
		if(Heights[Index] < Sea && Nodes[Index].Ocean) HasOriginalOcean[FindRoot(Index)] = true;
	}
	std::uint32_t WetComponents = 0, DetachedComponents = 0, DetachedWetNodes = 0;
	for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) {
		if(Heights[Index] >= Sea) continue;
		const std::uint32_t Root = FindRoot(Index);
		if(Root == Index) {
			++WetComponents;
			if(!HasOriginalOcean[Root]) ++DetachedComponents;
		}
		if(!HasOriginalOcean[Root]) ++DetachedWetNodes;
	}
	AddInfo(FString::Printf(
		TEXT("Final water graph: originalNodes=%u wetComponents=%u detachedComponents=%u detachedWetNodes=%u ")
		TEXT("selectedReaches=%llu reachGroundRises=%u maximumReachRise=%.6fm. Sampled graph only."),
		OriginalCount, WetComponents, DetachedComponents, DetachedWetNodes,
		static_cast<unsigned long long>(Field->Drainage().Reaches().size()), ReachGroundRises, MaximumReachRise));
	return true;
}
#endif
