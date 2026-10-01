#include "Core/PlanetFlowNetwork.h"
#include "Core/PlanetLakeTopology.h"
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
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLakeTopologyContracts,
	"PlanetGenerator.LakeTopology.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ水位の連結湖と独立湖を区別し，飽和の独立解と元場の追跡前の拒否を検査する
 * @param Parameters 未使用の自動試験引数
 * @return 全ての位相・水位・循環・正規入力・拒否条件が成立した場合に真
 */
bool FPlanetLakeTopologyContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::DepressionHierarchy;
	using PlanetCore::LakeComplex;
	using PlanetCore::LakeTopology;
	using PlanetCore::MissingSpillIndex;
	using PlanetCore::SpillGraphEdge;
	using PlanetCore::SpillGraphSite;
	const std::array<SpillGraphSite, 5> Sites { { { -1, true }, { 1, false }, { 2, false }, { 3, false }, { 4, false } } };
	const std::array<SpillGraphEdge, 4> Edges { { { { 0, 1 }, 10 }, { { 1, 2 }, 10 }, { { 2, 3 }, 12 }, { { 0, 4 }, 10 } } };
	const std::optional<DepressionHierarchy> Hierarchy = PlanetCore::BuildDepressionHierarchy(Sites, Edges, 0);
	// 正規の根拠となる階層を準備出来ない場合の返戻
	if(!TestTrue(TEXT("同水位と別水位を含む階層を準備"), Hierarchy.has_value())) return false;
	const std::optional<LakeTopology> Known = PlanetCore::BuildLakeTopology(*Hierarchy);
	// 同じ入力からの対応を拒否した場合の返戻
	if(!TestTrue(TEXT("連結湖の位相を準備"), Known.has_value())) return false;
	const std::uint32_t Lake = Known->SiteComplexes[1], Higher = Known->SiteComplexes[3];
	TestEqual(TEXT("大洋・連結湖・高い湖・独立湖を保持"), Known->Complexes.size(), std::size_t{ 4 });
	TestEqual(TEXT("同水位の実接続を集約"), Lake, Known->SiteComplexes[2]);
	TestTrue(TEXT("同じ水位だけの別湖は分離"), Lake != Known->SiteComplexes[4]);
	TestTrue(TEXT("高い湖の水位と所属を維持"), Lake != Higher && Known->Complexes[Higher].WaterMeters == 12);
	TestEqual(TEXT("既存の包含根を二つとも保持"), Known->Complexes[Lake].Basins.size(), std::size_t{ 2 });
	TestEqual(TEXT("湖内部の接続を一つ保持"), Known->Complexes[Lake].InternalSpillBasins.size(), std::size_t{ 1 });
	TestEqual(TEXT("下流への実流出口を一つ保持"), Known->Complexes[Lake].OutletBasins.size(), std::size_t{ 1 });
	TestEqual(TEXT("低い湖の水位を変更しない"), Known->Complexes[Lake].WaterMeters, 10.0);
	// 同じ水位でも二つの実出口を持つ湖は，一方を勝手に削除しない
	std::vector<SpillGraphEdge> MultipleEdges { { { 0, 1 }, 10 }, { { 0, 2 }, 10 }, { { 1, 2 }, 10 } };
	const std::optional<DepressionHierarchy> Multiple =
	PlanetCore::BuildDepressionHierarchy(std::span<const SpillGraphSite>(Sites).first(3), MultipleEdges, 0);
	// 正規の複数出口を拒否した場合の返戻
	if(!TestTrue(TEXT("複数出口の階層を準備"), Multiple.has_value())) return false;
	const std::optional<LakeTopology> Multi = PlanetCore::BuildLakeTopology(*Multiple);
	// 複数出口の連結を見落す場合の返戻
	if(!TestTrue(TEXT("採用した下流木の外の同水位辺も水域へ反映"), Multi.has_value())) return false;
	const LakeComplex &MultiLake = Multi->Complexes[Multi->SiteComplexes[1]];
	TestEqual(TEXT("二つの実流出口を維持"), MultiLake.OutletBasins.size(), std::size_t{ 2 });
	TestEqual(TEXT("木の外で繋がる場合に架空の内部枝を作らない"), MultiLake.InternalSpillBasins.size(), std::size_t{ 0 });
	TestEqual(TEXT("別々の根も実際の通路で同じ湖"), Multi->SiteComplexes[1], Multi->SiteComplexes[2]);
	// 海面下の接続と，水深０の乾いた平坦面を区別
	const std::array<SpillGraphSite, 4> MarineSites { { { -2, true }, { -1, false }, { 5, false }, { 5, false } } };
	const std::array<SpillGraphEdge, 3> MarineEdges { { { { 0, 1 }, 0 }, { { 0, 2 }, 5 }, { { 2, 3 }, 5 } } };
	const std::optional<DepressionHierarchy> MarineHierarchy = PlanetCore::BuildDepressionHierarchy(MarineSites, MarineEdges, 0);
	// 正規の海面以下の接続を拒否した場合の返戻
	if(!TestTrue(TEXT("海面下と乾いた平坦面の階層を準備"), MarineHierarchy.has_value())) return false;
	const std::optional<LakeTopology> Marine = PlanetCore::BuildLakeTopology(*MarineHierarchy);
	// 大洋に接続する水域の準備失敗への返戻
	if(!TestTrue(TEXT("海面下の実接続を大洋の水域へ対応"), Marine.has_value())) return false;
	TestEqual(TEXT("海面で繋がる水域は大洋に所属"), Marine->SiteComplexes[0], Marine->SiteComplexes[1]);
	TestEqual(TEXT("乾いた平坦面を湖にしない"), Marine->SiteComplexes[2], MissingSpillIndex);
	TestEqual(TEXT("乾いた根を明示的に保持"), Marine->DryBasins.size(), std::size_t{ 2 });
	// １ ULP の水位差も，同水位へ丸めずに保持
	std::vector<SpillGraphEdge> NearEdges(MultipleEdges.begin(), MultipleEdges.begin() + 2);
	NearEdges[1] = { { 1, 2 }, std::nextafter(10.0, 20.0) };
	const std::optional<DepressionHierarchy> Near =
	PlanetCore::BuildDepressionHierarchy(std::span<const SpillGraphSite>(Sites).first(3), NearEdges, 0);
	// 異なる有限水位の正規入力を拒否した場合の返戻
	if(!TestTrue(TEXT("隣接する浮動小数点水位を受理"), Near.has_value())) return false;
	const std::optional<LakeTopology> NearLakes = PlanetCore::BuildLakeTopology(*Near);
	TestTrue(TEXT("許容誤差で別水位を併合しない"), NearLakes && NearLakes->SiteComplexes[1] != NearLakes->SiteComplexes[2]);
	// 対応の破損と中断は，正規入力の受理と対に検査
	DepressionHierarchy Invalid = *Hierarchy;
	Invalid.Saturation.Nodes.pop_back();
	TestFalse(TEXT("異なる入力個数を拒否"), PlanetCore::BuildLakeTopology(Invalid).has_value());
	Invalid = *Hierarchy;
	Invalid.Nodes[Invalid.SiteRoots[2]].OceanLinkRank = 0;
	TestFalse(TEXT("下流の順位より小さい偽の接続を拒否"), PlanetCore::BuildLakeTopology(Invalid).has_value());
	Invalid = *Hierarchy;
	++Invalid.Saturation.Nodes[1].EscapeMeters;
	TestFalse(TEXT("水位の異なる対応を拒否"), PlanetCore::BuildLakeTopology(Invalid).has_value());
	Invalid = *Hierarchy;
	Invalid.Saturation.Nodes[1].Region = MissingSpillIndex;
	TestFalse(TEXT("湿った水域の対応欠落を拒否"), PlanetCore::BuildLakeTopology(Invalid).has_value());
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(TEXT("中断済の位相準備を拒否"), PlanetCore::BuildLakeTopology(*Hierarchy, Cancelled.get_token()).has_value());
	const std::array<SpillGraphSite, 1> OceanSite { { { -1, true } } };
	const std::optional<DepressionHierarchy> Ocean = PlanetCore::BuildDepressionHierarchy(OceanSite, {}, 0);
	TestTrue(TEXT("単独の大洋も正規の位相"), Ocean && PlanetCore::BuildLakeTopology(*Ocean).has_value());
	// 不完全な接続と複数外岸の未対応を，高価な元場照会前に識別
	for(const DepressionHierarchy *const Input : { &*Hierarchy, &*Multiple }) {
		PlanetCore::SpillNetwork Network;
		Network.Graph.Hierarchy = *Input;
		Network.Graph.Sites.assign(Sites.begin(), Sites.begin() + Input->SiteRoots.size());
		Network.Graph.Minima.resize(Input->SiteRoots.size());
		std::uint32_t Calls = 0;
		const auto Source = [&Calls](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			static_cast<void>(Direction);
			++Calls;
			// 拒否後に呼ばれた事を検出する元場の返戻
			return { 0, {} };
		};
		PlanetCore::FlowNetworkDiagnostics Report;
		TestFalse(TEXT("複数岸を単一の河道へ捏造しない"), PlanetCore::PrepareFlowNetwork(Network, Source, {}, {}, {}, &Report).has_value());
		TestTrue(
			TEXT("欠落した元通路又は複数の実出口を識別"),
			Report.Failure ==
			(Input == &*Multiple ? PlanetCore::FlowNetworkFailure::EqualWaterConnection : PlanetCore::FlowNetworkFailure::Outlet)
		);
		TestEqual(TEXT("未対応の水域数を保持"), Report.CompoundWaterBodies, std::uint32_t{ 1 });
		TestEqual(TEXT("岸の追跡前に拒否"), Calls, std::uint32_t{ 0 });
	}
	// 独立な全対最小最大路と，同じ水位だけを通る到達可能性を比較
	constexpr std::uint32_t Count = 23;
	std::uint32_t RandomState = 7951, ComparedSites = 0, ComparedPairs = 0, ComparedOutlets = 0;
	const auto Next = [&RandomState]() -> std::uint32_t {
		RandomState = 1664525 * RandomState + 1013904223;
		// 固定した比較用整数列の返戻
		return RandomState;
	};
	for(unsigned Trial = 0; Trial < 64; ++Trial) {
		const double Sea = Trial % 2 ? -3.0 : 0.0;
		std::vector<SpillGraphSite> RandomSites;
		std::vector<SpillGraphEdge> RandomEdges;
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			RandomSites.push_back({ Index ? static_cast<double>(Next() % 19) - 5 : Sea - 1, !Index });
		}
		const auto AddEdge = [&](const std::uint32_t A, const std::uint32_t B) -> void {
			RandomEdges.push_back(
				{ { A, B }, std::max(RandomSites[A].ElevationMeters, RandomSites[B].ElevationMeters) + static_cast<double>(Next() % 5) }
			);
			// 終了
			return;
		};
		for(std::uint32_t Index = 1; Index < Count; ++Index) AddEdge(Index - 1, Index);
		for(std::uint32_t Index = 2; Index < Count; ++Index) AddEdge(Index, Next() % (Index - 1));
		std::array<std::array<double, Count>, Count> Distances {};
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			Distances[Index].fill(std::numeric_limits<double>::infinity());
			Distances[Index][Index] = std::max(Sea, RandomSites[Index].ElevationMeters);
		}
		for(const SpillGraphEdge &Edge : RandomEdges) {
			const std::uint32_t A = Edge.Nodes[0], B = Edge.Nodes[1];
			Distances[A][B] = Distances[B][A] = std::min(Distances[A][B], std::max(Sea, Edge.CrestMeters));
		}
		for(std::uint32_t K = 0; K < Count; ++K) for(std::uint32_t I = 0; I < Count; ++I) {
			for(std::uint32_t J = 0; J < Count; ++J) {
				Distances[I][J] = std::min(Distances[I][J], std::max(Distances[I][K], Distances[K][J]));
			}
		}
		std::array<std::array<bool, Count>, Count> Connected {};
		for(std::uint32_t Index = 0; Index < Count; ++Index) Connected[Index][Index] = true;
		for(const SpillGraphEdge &Edge : RandomEdges) {
			const std::uint32_t A = Edge.Nodes[0], B = Edge.Nodes[1];
			if(Distances[A][0] == Distances[B][0] && Edge.CrestMeters <= Distances[A][0]) Connected[A][B] = Connected[B][A] = true;
		}
		for(std::uint32_t K = 0; K < Count; ++K) {
			for(std::uint32_t I = 0; I < Count; ++I) {
				for(std::uint32_t J = 0; J < Count; ++J) Connected[I][J] = Connected[I][J] || Connected[I][K] && Connected[K][J];
			}
		}
		const std::optional<DepressionHierarchy> RandomHierarchy = PlanetCore::BuildDepressionHierarchy(RandomSites, RandomEdges, Sea);
		// 比較用の正規入力を拒否した場合の返戻
		if(!TestTrue(TEXT("無作為な網の階層を準備"), RandomHierarchy.has_value())) return false;
		const std::optional<LakeTopology> Lakes = PlanetCore::BuildLakeTopology(*RandomHierarchy);
		// 同じ入力の派生位相を拒否した場合の返戻
		if(!TestTrue(TEXT("無作為な網の水域を準備"), Lakes.has_value())) return false;
		std::reverse(RandomEdges.begin(), RandomEdges.end());
		for(SpillGraphEdge &Edge : RandomEdges) std::swap(Edge.Nodes[0], Edge.Nodes[1]);
		RandomEdges.push_back(RandomEdges[0]);
		const std::optional<DepressionHierarchy> Reordered = PlanetCore::BuildDepressionHierarchy(RandomSites, RandomEdges, Sea);
		// 列挙順変更後の正規入力を拒否した場合の返戻
		if(!TestTrue(TEXT("並換えと重複後も階層を準備"), Reordered.has_value())) return false;
		const std::optional<LakeTopology> ReorderedLakes = PlanetCore::BuildLakeTopology(*Reordered);
		// 同一入力の並換えで位相を失う場合の返戻
		if(!TestTrue(TEXT("並換えと重複後も水域を準備"), ReorderedLakes.has_value())) return false;
		for(std::uint32_t I = 0; I < Count; ++I) {
			bool HasWater = false;
			for(std::uint32_t J = 0; J < Count; ++J) {
				HasWater = HasWater || Connected[I][J] && (RandomSites[J].IsOcean || RandomSites[J].ElevationMeters < Distances[I][0]);
			}
			const std::uint32_t Region = Lakes->SiteComplexes[I];
			TestEqual(TEXT("正の水深又は大洋を持つ成分だけが水域"), Region != MissingSpillIndex, HasWater);
			TestEqual(TEXT("独立な全対最小最大路の水位"), RandomHierarchy->Saturation.Nodes[I].EscapeMeters, Distances[I][0]);
			TestEqual(TEXT("並換えと重複で地点の水域は不変"), Region, ReorderedLakes->SiteComplexes[I]);
			++ComparedSites;
			if(!HasWater || Region == MissingSpillIndex) continue;
			for(std::uint32_t J = I + 1; J < Count; ++J) {
				TestEqual(TEXT("同水位で実際に到達可能な地点だけを集約"), Region == Lakes->SiteComplexes[J], Connected[I][J]);
				++ComparedPairs;
			}
		}
		for(const LakeComplex &Complex : Lakes->Complexes) {
			TestEqual(
				TEXT("各陸上根は内部接続又は外部流出口に一度だけ対応"),
				Complex.InternalSpillBasins.size() + Complex.OutletBasins.size(),
				Complex.Basins.size() - (Complex.IsOcean ? 1 : 0)
			);
			for(const std::uint32_t Root : Complex.OutletBasins) {
				const std::uint32_t Receiver = RandomHierarchy->SiteRoots[RandomHierarchy->Nodes[Root].ReceivingSite];
				TestTrue(TEXT("連結湖の外では水位が厳密に下がる"), RandomHierarchy->Nodes[Receiver].SpillMeters < Complex.WaterMeters);
				++ComparedOutlets;
			}
		}
	}
	AddInfo(
		FString::Printf(
			TEXT(
				"Lake topology contracts: independentSites=%u connectedPairs=%u strictOutlets=%u equalLakeRoots=2 outlets=1 multipleOutlets=2"
			),
			ComparedSites,
			ComparedPairs,
			ComparedOutlets
		)
	);
	// 位相と拒否条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
