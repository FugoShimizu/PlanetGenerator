#include "Core/PlanetSpillGraph.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSpillGraphContracts, "PlanetGenerator.SpillGraph.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 地形を保持する湖の接続，未標本の堰，循環の不在と不正入力の拒否を検査する
 * @param Parameters 未使用の UE 引数
 * @return 指定した網の排水条件を満たす場合に真，連続水面又は景観の合格ではない
 */
bool FPlanetSpillGraphContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::SpillGraph;
	using PlanetCore::SpillGraphEdge;
	using PlanetCore::SpillGraphSite;
	// 高さ４，７，９の湖が順に低い水系へ溢れる既知の断面
	const std::array<SpillGraphSite, 10> Sites {
		{
			{ -2, true },
			{ 4, false },
			{ 1, false },
			{ 2, false },
			{ 7, false },
			{ 3, false },
			{ 6, false },
			{ 9, false },
			{ 5, false },
			{ 11, false }
		}
	};
	std::vector<SpillGraphEdge> Edges;
	for(std::uint32_t i = 1; i < Sites.size(); ++i) {
		Edges.push_back({ { i - 1, i }, std::max(Sites[i - 1].ElevationMeters, Sites[i].ElevationMeters) });
	}
	const std::optional<SpillGraph> Graph = PlanetCore::BuildSpillGraph(Sites, Edges, 0);
	// 正規の入力を受理しない場合の返戻
	if(!TestTrue(TEXT("段差状の通過湖を準備"), Graph.has_value())) return false;
	const std::array<double, 10> Expected { 0, 4, 4, 4, 7, 7, 7, 9, 9, 11 };
	for(std::uint32_t i = 0; i < Sites.size(); ++i) {
		TestEqual(TEXT("独立に指定した溢流位"), Graph->Nodes[i].EscapeMeters, Expected[i]);
		if(i) TestTrue(TEXT("出口側の確定順は常に先"), Graph->Nodes[Graph->Nodes[i].Downstream].SettlementRank < Graph->Nodes[i].SettlementRank);
	}
	TestEqual(TEXT("大洋と３個の通過湖"), Graph->Regions.size(), std::size_t{ 4 });
	TestTrue(
		TEXT("谷の両側を同じ水位へ接続"),
		Graph->Nodes[1].Region == Graph->Nodes[2].Region && Graph->Nodes[2].Region == Graph->Nodes[3].Region
	);
	TestEqual(TEXT("水面より高い斜面は湖でない"), Graph->Nodes[9].Region, PlanetCore::MissingSpillIndex);
	for(const PlanetCore::SpillGraphRegion &Region : Graph->Regions) {
		if(Region.IsOcean) continue;
		TestTrue(TEXT("湖は実在する辺へ溢れる"), Region.OutletEdge < Edges.size());
		TestTrue(TEXT("湖の出口は別の低い水域又は斜面"), Graph->Nodes[Region.OutletNeighbor].EscapeMeters < Region.WaterMeters);
		TestTrue(TEXT("出口の堰を水面が越えられる限界"), Edges[Region.OutletEdge].CrestMeters <= Region.WaterMeters);
	}
	// 端点より高い途中の堰を無視しない回帰
	const std::array<SpillGraphSite, 3> HiddenSites { { { -1, true }, { 3, false }, { 1, false } } };
	const std::array<SpillGraphEdge, 2> HiddenEdges { { { { 0, 1 }, 3 }, { { 1, 2 }, 8 } } };
	const std::optional<SpillGraph> Hidden = PlanetCore::BuildSpillGraph(HiddenSites, HiddenEdges, 0);
	// 有効な通路上限を受理しない場合の返戻
	if(!TestTrue(TEXT("途中の堰を持つ経路を受理"), Hidden.has_value())) return false;
	TestEqual(TEXT("端点間の高さ８の堰が必要水位を決める"), Hidden->Nodes[2].EscapeMeters, 8.0);
	TestEqual(TEXT("下流の高さ３は持上げない"), Hidden->Nodes[1].EscapeMeters, 3.0);
	// 水深の無い平坦面を ID 順の水位差で偽装しない検査
	const std::array<SpillGraphSite, 4> FlatSites { { { -1, true }, { 5, false }, { 5, false }, { 5, false } } };
	const std::array<SpillGraphEdge, 3> FlatEdges { { { { 0, 1 }, 5 }, { { 1, 2 }, 5 }, { { 2, 3 }, 5 } } };
	const std::optional<SpillGraph> Flat = PlanetCore::BuildSpillGraph(FlatSites, FlatEdges, 0);
	// 有効な平坦面を受理しない場合の返戻
	if(!TestTrue(TEXT("乾いた平坦面の診断を受理"), Flat.has_value())) return false;
	TestEqual(TEXT("平坦面を湖として登録しない"), Flat->Regions.size(), std::size_t{ 1 });
	TestEqual(TEXT("未解決の乾いた平坦辺を明示"), Flat->DryFlatEdges, std::uint32_t{ 2 });
	TestEqual(TEXT("最小勾配の為の人工的な標高加算が無い"), Flat->Nodes[3].EscapeMeters, 5.0);
	// 出口証拠の無い入力と中断の検査
	std::vector<SpillGraphSite> InvalidSites(HiddenSites.begin(), HiddenSites.end());
	InvalidSites[2] = { -1, true };
	TestFalse(TEXT("陸の堰で隔てられた水域を大洋と呼ばない"), PlanetCore::BuildSpillGraph(InvalidSites, HiddenEdges, 0).has_value());
	InvalidSites[0].IsOcean = false;
	InvalidSites[2].IsOcean = false;
	TestFalse(TEXT("大洋の無い網を拒否"), PlanetCore::BuildSpillGraph(InvalidSites, HiddenEdges, 0).has_value());
	TestFalse(
		TEXT("大洋へ到達しない成分を拒否"),
		PlanetCore::BuildSpillGraph(HiddenSites, std::span<const SpillGraphEdge>(HiddenEdges).first(1), 0).has_value()
	);
	std::vector<SpillGraphEdge> InvalidEdges(HiddenEdges.begin(), HiddenEdges.end());
	InvalidEdges[1].CrestMeters = 2;
	TestFalse(TEXT("端点標高未満の通路上限を拒否"), PlanetCore::BuildSpillGraph(HiddenSites, InvalidEdges, 0).has_value());
	InvalidEdges[1] = { { 1, 3 }, 8 };
	TestFalse(TEXT("範囲外の接続を拒否"), PlanetCore::BuildSpillGraph(HiddenSites, InvalidEdges, 0).has_value());
	InvalidEdges[1] = { { 1, 1 }, 8 };
	TestFalse(TEXT("自己辺を拒否"), PlanetCore::BuildSpillGraph(HiddenSites, InvalidEdges, 0).has_value());
	InvalidEdges[1] = { { 1, 2 }, std::numeric_limits<double>::quiet_NaN() };
	TestFalse(TEXT("非有限な通路上限を拒否"), PlanetCore::BuildSpillGraph(HiddenSites, InvalidEdges, 0).has_value());
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(TEXT("中断済の準備を拒否"), PlanetCore::BuildSpillGraph(Sites, Edges, 0, Cancelled.get_token()).has_value());
	// 独立な全対経路法及び辺の逆順との比較
	constexpr std::uint32_t Count = 19;
	std::uint32_t RandomState = 5317, Compared = 0;
	const auto Next = [&RandomState]() -> std::uint32_t {
		RandomState = 1664525 * RandomState + 1013904223;
		// 固定した整数列の返戻
		return RandomState;
	};
	for(unsigned Trial = 0; Trial < 32; ++Trial) {
		std::vector<SpillGraphSite> RandomSites;
		std::vector<SpillGraphEdge> RandomEdges;
		for(std::uint32_t i = 0; i < Count; ++i) RandomSites.push_back({ i ? static_cast<double>(Next() % 15) : -1.0, !i });
		const auto AddEdge = [&](const std::uint32_t A, const std::uint32_t B) -> void {
			RandomEdges.push_back(
				{ { A, B }, std::max(RandomSites[A].ElevationMeters, RandomSites[B].ElevationMeters) + static_cast<double>(Next() % 4) }
			);
			// 終了
			return;
		};
		for(std::uint32_t i = 1; i < Count; ++i) AddEdge(i - 1, i);
		for(std::uint32_t i = 2; i < Count; ++i) AddEdge(i, Next() % (i - 1));
		std::array<std::array<double, Count>, Count> Distances {};
		for(std::uint32_t i = 0; i < Count; ++i) {
			Distances[i].fill(std::numeric_limits<double>::infinity());
			Distances[i][i] = std::max(0.0, RandomSites[i].ElevationMeters);
		}
		for(const SpillGraphEdge &Edge : RandomEdges) {
			const std::uint32_t A = Edge.Nodes[0], B = Edge.Nodes[1];
			Distances[A][B] = Distances[B][A] = std::min(Distances[A][B], Edge.CrestMeters);
		}
		for(std::uint32_t k = 0; k < Count; ++k) for(std::uint32_t i = 0; i < Count; ++i) {
			for(std::uint32_t j = 0; j < Count; ++j) {
				Distances[i][j] = std::min(Distances[i][j], std::max(Distances[i][k], Distances[k][j]));
			}
		}
		const std::optional<SpillGraph> RandomGraph = PlanetCore::BuildSpillGraph(RandomSites, RandomEdges, 0);
		// 正規の連結網を解けない場合の返戻
		if(!TestTrue(TEXT("独立比較用の連結網を受理"), RandomGraph.has_value())) return false;
		std::reverse(RandomEdges.begin(), RandomEdges.end());
		for(SpillGraphEdge &Edge : RandomEdges) std::swap(Edge.Nodes[0], Edge.Nodes[1]);
		const std::optional<SpillGraph> Reordered = PlanetCore::BuildSpillGraph(RandomSites, RandomEdges, 0);
		// 順序変更した同じ接続を解けない場合の返戻
		if(!TestTrue(TEXT("逆順の同じ接続を受理"), Reordered.has_value())) return false;
		for(std::uint32_t i = 0; i < Count; ++i) {
			TestEqual(TEXT("全対経路の独立解と一致"), RandomGraph->Nodes[i].EscapeMeters, Distances[i][0]);
			TestEqual(TEXT("辺の列挙順に依らない水位"), RandomGraph->Nodes[i].EscapeMeters, Reordered->Nodes[i].EscapeMeters);
			TestEqual(TEXT("辺の列挙順に依らない流出方向"), RandomGraph->Nodes[i].Downstream, Reordered->Nodes[i].Downstream);
			std::uint32_t Index = i, Steps = 0;
			while(Index && Steps < Count) {
				const std::uint32_t Downstream = RandomGraph->Nodes[Index].Downstream;
				TestTrue(TEXT("下流への逆水位が無い"), RandomGraph->Nodes[Downstream].EscapeMeters <= RandomGraph->Nodes[Index].EscapeMeters);
				Index = Downstream;
				++Steps;
			}
			TestEqual(TEXT("全ての地点が唯一の大洋へ到達"), Index, std::uint32_t{ 0 });
			++Compared;
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Spill graph contracts: nestedLakes=3 independentMinimaxNodes=%u hiddenBarrier=8 dryFlatEdges=2"),
			Compared
		)
	);
	// 独立な排水条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
