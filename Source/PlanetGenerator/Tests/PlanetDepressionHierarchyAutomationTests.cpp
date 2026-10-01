#include "Core/PlanetDepressionHierarchy.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetDepressionHierarchyContracts, "PlanetGenerator.DepressionHierarchy.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 包含と溢流を区別し，既知の湖・独立な全対経路・順序変更で検査する
 * @param Parameters 未使用の UE 引数
 * @return 網の階層条件の検査結果；有限水量又は連続湖岸の合格ではない
 */
bool FPlanetDepressionHierarchyContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::DepressionHierarchy;
	using PlanetCore::DepressionHierarchyNode;
	using PlanetCore::MissingSpillIndex;
	using PlanetCore::SpillGraphEdge;
	using PlanetCore::SpillGraphSite;
	// ２個の窪地は高さ６で合体して高さ１０で溢れ，上流の別湖は高さ１２を保つ
	const std::array<SpillGraphSite, 4> Sites { { { -1, true }, { 2, false }, { 1, false }, { 5, false } } };
	const std::array<SpillGraphEdge, 3> Edges { { { { 0, 1 }, 10 }, { { 1, 2 }, 6 }, { { 2, 3 }, 12 } } };
	const std::optional<DepressionHierarchy> Known = PlanetCore::BuildDepressionHierarchy(Sites, Edges, 0);
	// 正規の階層を構成出来ない場合の返戻
	if(!TestTrue(TEXT("既知の包含・溢流を受理"), Known.has_value())) return false;
	const std::uint32_t Low = Known->SiteRoots[1], High = Known->SiteRoots[3];
	TestEqual(TEXT("低い２窪地を同じ包含木へ合体"), Low, Known->SiteRoots[2]);
	TestTrue(TEXT("高い湖を低い湖の包含子にしない"), High != Low && Known->Nodes[High].Parent == MissingSpillIndex);
	TestEqual(TEXT("下流の水位を１０に保持"), Known->Nodes[Low].SpillMeters, 10.0);
	TestEqual(TEXT("上流の水位を１２に保持"), Known->Nodes[High].SpillMeters, 12.0);
	TestEqual(TEXT("上流の実流出口は地点３"), Known->Nodes[High].OutletSite, std::uint32_t{ 3 });
	TestEqual(TEXT("下流の実流入先は地点２"), Known->Nodes[High].ReceivingSite, std::uint32_t{ 2 });
	TestEqual(TEXT("包含子は高さ６で合体"), Known->Nodes[Known->SiteLeaves[1]].SpillMeters, 6.0);
	TestEqual(TEXT("下流水域への順序は先"), Known->Nodes[Low].OceanLinkRank, std::uint32_t{ 1 });
	TestEqual(TEXT("上流水域の順序は後"), Known->Nodes[High].OceanLinkRank, std::uint32_t{ 2 });
	TestEqual(TEXT("元の標高を親の水位へ変更しない"), Known->Nodes[Known->SiteLeaves[2]].MinimumMeters, 1.0);
	// 海面以下の内陸窪地も，大洋へ通じる堰を保持する
	const std::array<SpillGraphSite, 4> MarineSites { { { -4, true }, { -3, true }, { -2, false }, { 4, false } } };
	const std::array<SpillGraphEdge, 3> MarineEdges { { { { 0, 1 }, -1 }, { { 1, 2 }, 3 }, { { 2, 3 }, 4 } } };
	const std::optional<DepressionHierarchy> Marine = PlanetCore::BuildDepressionHierarchy(MarineSites, MarineEdges, 0);
	// 連結した大洋を受理しない場合の返戻
	if(!TestTrue(TEXT("複数の連結した大洋地点を受理"), Marine.has_value())) return false;
	TestEqual(TEXT("大洋地点を０番へ集約"), Marine->SiteLeaves[1], std::uint32_t{ 0 });
	TestEqual(TEXT("陸で隔てられた海面下窪地の水位は３"), Marine->Nodes[Marine->SiteRoots[2]].SpillMeters, 3.0);
	TestEqual(TEXT("乾いた斜面は水深０のまま"), Marine->Nodes[Marine->SiteRoots[3]].SpillMeters, 4.0);
	TestEqual(TEXT("元の負標高を維持"), Marine->Nodes[Marine->SiteLeaves[2]].MinimumMeters, -2.0);
	// 入力と中断の拒否は正規経路の受理と対にする
	std::vector<SpillGraphSite> InvalidSites(MarineSites.begin(), MarineSites.end());
	InvalidSites[2].IsOcean = true;
	TestFalse(TEXT("堰で隔てた内陸を大洋と指定した場合は拒否"), PlanetCore::BuildDepressionHierarchy(InvalidSites, MarineEdges, 0).has_value());
	TestFalse(
		TEXT("切離された地点を拒否"),
		PlanetCore::BuildDepressionHierarchy(Sites, std::span<const SpillGraphEdge>(Edges).first(2), 0).has_value()
	);
	InvalidSites[2].IsOcean = false;
	InvalidSites[0].IsOcean = InvalidSites[1].IsOcean = false;
	TestFalse(TEXT("大洋の無い入力を拒否"), PlanetCore::BuildDepressionHierarchy(InvalidSites, MarineEdges, 0).has_value());
	std::vector<SpillGraphEdge> InvalidEdges(MarineEdges.begin(), MarineEdges.end());
	InvalidEdges[2].CrestMeters = 3;
	TestFalse(TEXT("端点より低い堰の指定を拒否"), PlanetCore::BuildDepressionHierarchy(MarineSites, InvalidEdges, 0).has_value());
	InvalidEdges[2].CrestMeters = std::numeric_limits<double>::infinity();
	TestFalse(TEXT("非有限な堰を拒否"), PlanetCore::BuildDepressionHierarchy(MarineSites, InvalidEdges, 0).has_value());
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(TEXT("中断済の処理を拒否"), PlanetCore::BuildDepressionHierarchy(Sites, Edges, 0, Cancelled.get_token()).has_value());
	// 入力１地点だけの大洋も正規の状態
	const std::array<SpillGraphSite, 1> OceanOnly { { { -2, true } } };
	const std::optional<DepressionHierarchy> Ocean = PlanetCore::BuildDepressionHierarchy(OceanOnly, {}, 0);
	TestTrue(TEXT("単独の大洋は木１個"), Ocean.has_value() && Ocean->Nodes.size() == 1 && !Ocean->SiteRoots[0]);
	// 同水位の複数経路を含む全対最小最大路の独立解
	constexpr std::uint32_t Count = 23;
	std::uint32_t RandomState = 6751, ComparedSites = 0, ComparedPairs = 0, ComparedNodes = 0;
	const auto Next = [&RandomState]() -> std::uint32_t {
		RandomState = 1664525 * RandomState + 1013904223;
		// 固定した整数列の返戻
		return RandomState;
	};
	for(unsigned Trial = 0; Trial < 64; ++Trial) {
		// 負の海面を含む，独立した地点と通路の入力
		const double Sea = Trial % 2 ? -3.0 : 0.0;
		std::vector<SpillGraphSite> RandomSites;
		std::vector<SpillGraphEdge> RandomEdges;
		for(std::uint32_t i = 0; i < Count; ++i) RandomSites.push_back({ i ? static_cast<double>(Next() % 19) - 5 : Sea - 1, !i });
		const auto AddEdge = [&](const std::uint32_t A, const std::uint32_t B) -> void {
			RandomEdges.push_back(
				{ { A, B }, std::max(RandomSites[A].ElevationMeters, RandomSites[B].ElevationMeters) + static_cast<double>(Next() % 5) }
			);
			// 終了
			return;
		};
		for(std::uint32_t i = 1; i < Count; ++i) AddEdge(i - 1, i);
		for(std::uint32_t i = 2; i < Count; ++i) AddEdge(i, Next() % (i - 1));
		std::array<std::array<double, Count>, Count> Distances {};
		for(std::uint32_t i = 0; i < Count; ++i) {
			Distances[i].fill(std::numeric_limits<double>::infinity());
			Distances[i][i] = std::max(Sea, RandomSites[i].ElevationMeters);
		}
		for(const SpillGraphEdge &Edge : RandomEdges) {
			const std::uint32_t A = Edge.Nodes[0], B = Edge.Nodes[1];
			Distances[A][B] = Distances[B][A] = std::min(Distances[A][B], std::max(Sea, Edge.CrestMeters));
		}
		for(std::uint32_t k = 0; k < Count; ++k) for(std::uint32_t i = 0; i < Count; ++i) {
			for(std::uint32_t j = 0; j < Count; ++j) {
				Distances[i][j] = std::min(Distances[i][j], std::max(Distances[i][k], Distances[k][j]));
			}
		}
		const std::optional<DepressionHierarchy> Hierarchy = PlanetCore::BuildDepressionHierarchy(RandomSites, RandomEdges, Sea);
		// 正規の比較用入力を拒否した場合の返戻
		if(!TestTrue(TEXT("比較用の網を受理"), Hierarchy.has_value())) return false;
		// 列挙順・向き・同一通路の重複に依る位相の不変性
		std::vector<SpillGraphEdge> ReorderedEdges(RandomEdges.rbegin(), RandomEdges.rend());
		for(SpillGraphEdge &Edge : ReorderedEdges) std::swap(Edge.Nodes[0], Edge.Nodes[1]);
		ReorderedEdges.push_back(ReorderedEdges[0]);
		const std::optional<DepressionHierarchy> Reordered = PlanetCore::BuildDepressionHierarchy(RandomSites, ReorderedEdges, Sea);
		// 正規の並換入力を拒否した場合の返戻
		if(!TestTrue(TEXT("同じ網の並換・同一辺の重複を受理"), Reordered.has_value())) return false;
		// 比較範囲が異なる場合の返戻
		if(!TestEqual(TEXT("包含木の数は不変"), Reordered->Nodes.size(), Hierarchy->Nodes.size())) return false;
		for(std::uint32_t i = 0; i < Hierarchy->Nodes.size(); ++i) {
			// 範囲・証拠を先に確認し，失敗を添字アクセスで隠さない
			const DepressionHierarchyNode &Node = Hierarchy->Nodes[i], &Other = Reordered->Nodes[i];
			TestEqual(TEXT("包含親は不変"), Node.Parent, Other.Parent);
			TestTrue(TEXT("包含子は不変"), Node.Children == Other.Children);
			TestEqual(TEXT("溢流位は不変"), Node.SpillMeters, Other.SpillMeters);
			TestEqual(TEXT("流出元は不変"), Node.OutletSite, Other.OutletSite);
			TestEqual(TEXT("流入先は不変"), Node.ReceivingSite, Other.ReceivingSite);
			TestEqual(TEXT("大洋接続順は不変"), Node.OceanLinkRank, Other.OceanLinkRank);
			if(!i) continue;
			// 範囲検査を終えてから実通路を参照する条件
			const bool HasOutletEvidence = Node.OutletEdge < RandomEdges.size() && Node.OutletSite < Count && Node.ReceivingSite < Count;
			// 実通路の証拠が不正な場合の返戻
			if(!TestTrue(TEXT("実通路と流出入地点を保持"), HasOutletEvidence)) return false;
			// 位相上の出口と比較する実通路
			const SpillGraphEdge &Passage = RandomEdges[Node.OutletEdge];
			TestEqual(TEXT("端点途中の堰を含む溢流位"), Node.SpillMeters, std::max(Sea, Passage.CrestMeters));
			TestTrue(
				TEXT("出口は指定した実通路の端点"),
				Passage.Nodes[0] == Node.OutletSite && Passage.Nodes[1] == Node.ReceivingSite ||
				Passage.Nodes[1] == Node.OutletSite && Passage.Nodes[0] == Node.ReceivingSite
			);
			if(Node.Parent != MissingSpillIndex) {
				// 包含順又は親の範囲が不正な場合の返戻
				if(!TestTrue(TEXT("親は後方の有効な節点"), Node.Parent > i && Node.Parent < Hierarchy->Nodes.size())) return false;
				TestEqual(TEXT("包含子を下流接続と混同しない"), Node.OceanLinkRank, MissingSpillIndex);
				TestTrue(TEXT("包含に従い水位は非減少"), Node.SpillMeters <= Hierarchy->Nodes[Node.Parent].SpillMeters);
			} else {
				// 飽和時に別の水域として保つ下流
				const std::uint32_t Receiver = Hierarchy->SiteRoots[Node.ReceivingSite];
				TestEqual(TEXT("流出元は根の領域に所属"), Hierarchy->SiteRoots[Node.OutletSite], i);
				TestTrue(TEXT("下流の根は別の水域"), Receiver != i);
				TestTrue(TEXT("下流接続順の減少に因り循環しない"), Hierarchy->Nodes[Receiver].OceanLinkRank < Node.OceanLinkRank);
				TestTrue(TEXT("下流の水位を持上げない"), Hierarchy->Nodes[Receiver].SpillMeters <= Node.SpillMeters);
			}
			if(Node.Site == MissingSpillIndex) {
				for(const std::uint32_t Child : Node.Children) {
					// 子の範囲又は順序が不正な場合の返戻
					if(!TestTrue(TEXT("子は有効な先行節点"), Child && Child < i)) return false;
					TestEqual(TEXT("包含の双方向が一致"), Hierarchy->Nodes[Child].Parent, i);
				}
				TestEqual(
					TEXT("最低標高を保持"),
					Node.MinimumMeters,
					std::min(Hierarchy->Nodes[Node.Children[0]].MinimumMeters, Hierarchy->Nodes[Node.Children[1]].MinimumMeters)
				);
			} else TestEqual(TEXT("入力標高を一切持上げない"), Node.MinimumMeters, RandomSites[Node.Site].ElevationMeters);
			++ComparedNodes;
		}
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 入力地点が飽和時に属する水域
			const std::uint32_t Root = Hierarchy->SiteRoots[i];
			TestEqual(TEXT("独立な全対経路の溢流位"), Hierarchy->Nodes[Root].SpillMeters, Distances[i][0]);
			TestEqual(TEXT("根の対応は順序に依らない"), Root, Reordered->SiteRoots[i]);
			++ComparedSites;
			for(std::uint32_t j = i + 1; j < Count; ++j) {
				// 同じ包含木の最初の共通祖先，又は大洋経由の接続位
				double Merge = std::max(Distances[i][0], Distances[j][0]);
				if(Root == Hierarchy->SiteRoots[j]) {
					std::vector<bool> Ancestors(Hierarchy->Nodes.size(), false);
					for(std::uint32_t Node = Hierarchy->SiteLeaves[i]; Node != MissingSpillIndex; Node = Hierarchy->Nodes[Node].Parent) {
						Ancestors[Node] = true;
					}
					// 他方の葉から辿る最初の共通祖先
					std::uint32_t Common = Hierarchy->SiteLeaves[j];
					while(Common != MissingSpillIndex && !Ancestors[Common]) Common = Hierarchy->Nodes[Common].Parent;
					// 共通祖先が欠ける場合の返戻
					if(!TestTrue(TEXT("同じ水域には共通祖先が在る"), Common < Hierarchy->Nodes.size())) return false;
					Merge = Common ? Hierarchy->Nodes[Hierarchy->Nodes[Common].Children[0]].SpillMeters : Sea;
				}
				TestEqual(TEXT("独立な全対経路と包含の合流水位が一致"), Merge, Distances[i][j]);
				++ComparedPairs;
			}
		}
	}
	AddInfo(
		FString::Printf(
			TEXT(
				"Depression hierarchy contracts: independentSites=%u independentPairs=%u topologyNodes=%u nestedSpill=6 lowLake=10 highLake=12"
			),
			ComparedSites,
			ComparedPairs,
			ComparedNodes
		)
	);
	// 包含と溢流の検査結果の返戻
	return !HasAnyErrors();
}

#endif
