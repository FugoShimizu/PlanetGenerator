#include "Core/PlanetSpillNetwork.h"
#include "Core/PlanetSpillSampleCache.h"
#include "Misc/AutomationTest.h"
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillSourceReuse,
	"PlanetGenerator.ContinuousSpill.SourceReuse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 再利用枠の衝突・無効化で値を変えず，大洋へ繋がる実経路と排水階層を維持する
 * @param Parameters 未使用の実行引数
 * @return 三半径の実接続がビット一致し，評価と保持量の上限・中断を守る場合に真
 */
bool FPlanetSpillSourceReuse::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 診断回数を除き，実経路と排水関係を各成分の値として比較
	const auto Values = [](const PlanetCore::SpillNetwork &Network) -> std::vector<std::uint64_t> {
		std::vector<std::uint64_t> Data;
		const auto Number = [&](const double Value) -> void {
			Data.push_back(std::bit_cast<std::uint64_t>(Value));
			// 値の追加の終了
			return;
		};
		const auto Vector = [&](const PlanetCore::Vec3 V) -> void {
			Number(V.X);
			Number(V.Y);
			Number(V.Z);
			// 成分の追加の終了
			return;
		};
		const auto Critical = [&](const PlanetCore::TerrainCriticalPoint &P) -> void {
			Vector(P.Direction.Vector());
			Number(P.ElevationMeters);
			Number(P.GradientNorm);
			Number(P.MinimumCurvaturePerMeter);
			Number(P.MaximumCurvaturePerMeter);
			Vector(P.MinimumCurvatureDirection);
			Data.push_back(static_cast<unsigned>(P.Kind));
			// 実停留点の追加の終了
			return;
		};
		Data.push_back(Network.Connections.size());
		for(const PlanetCore::SpillConnection &Connection : Network.Connections) {
			Critical(Connection.Pass);
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
				Data.push_back(static_cast<unsigned>(Branch.Kind));
				Data.push_back(Branch.Minimum.has_value());
				if(Branch.Minimum) Critical(*Branch.Minimum);
				Data.push_back(static_cast<unsigned>(Branch.Path.End));
				Data.push_back(Branch.Path.RejectedSteps);
				Number(Branch.Path.MaximumEstimatedErrorMeters);
				Data.push_back(Branch.Path.Samples.size());
				for(const PlanetCore::TerrainDescentSample &Sample : Branch.Path.Samples) {
					Vector(Sample.Direction.Vector());
					Number(Sample.ElevationMeters);
					Number(Sample.DistanceMeters);
				}
			}
		}
		Data.push_back(Network.OceanWitnesses.size());
		for(const PlanetCore::SpillOceanWitness &Witness : Network.OceanWitnesses) {
			Vector(Witness.Terminal.Vector());
			Vector(Witness.Anchor.Vector());
			Number(Witness.CheckedMaximumMeters);
		}
		const PlanetCore::ContinuousSpillGraph &Graph = Network.Graph;
		Data.push_back(Graph.Sites.size());
		for(const PlanetCore::SpillGraphSite &Site : Graph.Sites) {
			Number(Site.ElevationMeters);
			Data.push_back(Site.IsOcean);
		}
		Data.push_back(Graph.Edges.size());
		for(const PlanetCore::SpillGraphEdge &Edge : Graph.Edges) {
			Data.push_back(Edge.Nodes[0]);
			Data.push_back(Edge.Nodes[1]);
			Number(Edge.CrestMeters);
		}
		for(const std::uint32_t Value : Graph.EdgeConnections) Data.push_back(Value);
		for(const std::array<std::uint32_t, 2> &Pair : Graph.BranchSites) {
			Data.push_back(Pair[0]);
			Data.push_back(Pair[1]);
		}
		for(const std::uint32_t Value : Graph.Hierarchy.SiteRoots) Data.push_back(Value);
		for(const std::uint32_t Value : Graph.Hierarchy.SiteLeaves) Data.push_back(Value);
		Data.push_back(Graph.Hierarchy.Nodes.size());
		for(const PlanetCore::DepressionHierarchyNode &Node : Graph.Hierarchy.Nodes) {
			for(
				const std::uint32_t Value :
				{
					Node.Parent,
					Node.Children[0],
					Node.Children[1],
					Node.Site,
					Node.OutletSite,
					Node.ReceivingSite,
					Node.OutletEdge,
					Node.OceanLinkRank
				}
			) Data.push_back(Value);
			Number(Node.MinimumMeters);
			Number(Node.SpillMeters);
		}
		const PlanetCore::SpillGraph &Saturation = Graph.Hierarchy.Saturation;
		Data.push_back(Saturation.Nodes.size());
		for(const PlanetCore::SpillGraphNode &Node : Saturation.Nodes) {
			Number(Node.EscapeMeters);
			for(const std::uint32_t Value : { Node.Downstream, Node.DownstreamEdge, Node.SettlementRank, Node.Region }) {
				Data.push_back(Value);
			}
		}
		Data.push_back(Saturation.Regions.size());
		for(const PlanetCore::SpillGraphRegion &Region : Saturation.Regions) {
			Number(Region.WaterMeters);
			for(
				const std::uint32_t Value :
				{ Region.OutletNode, Region.OutletNeighbor, Region.OutletEdge, Region.WetNodes, Region.ShoreNodes }
			) Data.push_back(Value);
			Data.push_back(Region.IsOcean);
		}
		Data.push_back(Saturation.DryFlatEdges);
		// 型の詰物を読まない成分列の返戻
		return Data;
	};
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			// 同じ解析場の標高と全微分の返戻
			return {
				4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400,
				{ Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) }
			};
		};
		const PlanetCore::UnitDirection Initial =
		*PlanetCore::UnitDirection::FromVector({ 40 * (4 * std::acos(-1.0) + std::asin(0.2)) / Radius, 0, 1 });
		const std::array<PlanetCore::UnitDirection, 1> Anchors { *PlanetCore::UnitDirection::FromVector({ -140 / Radius, 0, 1 }) };
		const auto IsOcean = [&](const PlanetCore::UnitDirection D) -> bool {
			const PlanetCore::Vec3 A = D.Vector(), B = Anchors[0].Vector();
			// 指定した同じ海域標本だけへの所属の返戻
			return A.X == B.X && A.Y == B.Y && A.Z == B.Z;
		};
		PlanetCore::SpillNetworkSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.MaximumCachedSamples = 0;
		PlanetCore::SpillNetworkDiagnostics OriginalReport;
		const std::optional<PlanetCore::SpillNetwork> Original =
		PlanetCore::PrepareSpillNetwork(Initial, Source, Anchors, IsOcean, Settings, {}, &OriginalReport);
		// 比較基準を準備出来ない場合の返戻
		if(!TestTrue(TEXT("再利用無の正規水系"), Original.has_value())) return false;
		TestEqual(TEXT("基準の元場評価回数"), OriginalReport.Evaluations, Calls);
		const std::vector<std::uint64_t> Expected = Values(*Original);
		for(const std::uint32_t Capacity : { 1U, 17U, 262144U, PlanetCore::SpillSampleCache::MaximumCapacity }) {
			Settings.MaximumCachedSamples = Capacity;
			Calls = 0;
			PlanetCore::SpillNetworkDiagnostics Report;
			const std::optional<PlanetCore::SpillNetwork> Reused =
			PlanetCore::PrepareSpillNetwork(Initial, Source, Anchors, IsOcean, Settings, {}, &Report);
			// 衝突の有る枠でも正規の水系を準備出来ない場合の返戻
			if(!TestTrue(TEXT("有界な枠で正規水系を準備"), Reused.has_value())) return false;
			TestTrue(TEXT("全経路・鞍点・実終端・大洋通路・排水階層がビット一致"), Values(*Reused) == Expected);
			TestEqual(TEXT("元場を呼んだ実回数を報告"), Report.Evaluations, Calls);
			TestTrue(TEXT("保持量は枠の上限以内"), Report.CachedSamples <= Capacity);
			if(Capacity > 262143) TestTrue(TEXT("元場の再評価を削減"), Report.CacheHits > 0 && Calls < OriginalReport.Evaluations);
			AddInfo(
				FString::Printf(
					TEXT("Source reuse R=%.0f capacity=%u evaluations=%llu/%llu hits=%llu retained=%u valueComponents=%llu"),
					Radius,
					Capacity,
					Calls,
					OriginalReport.Evaluations,
					Report.CacheHits,
					Report.CachedSamples,
					static_cast<unsigned long long>(Expected.size())
				)
			);
		}
		Settings.MaximumCachedSamples = PlanetCore::SpillSampleCache::MaximumCapacity + 1;
		Calls = 0;
		PlanetCore::SpillNetworkDiagnostics Invalid;
		TestFalse(
			TEXT("過大な保持枠を確保前に拒否"),
			PlanetCore::PrepareSpillNetwork(Initial, Source, Anchors, IsOcean, Settings, {}, &Invalid).has_value()
		);
		TestEqual(TEXT("過大な枠では元場を呼出さない"), Calls, UINT64_C(0));
		Settings.MaximumCachedSamples = 262144;
		Settings.MaximumEvaluations = 1;
		TestFalse(
			TEXT("再利用で元場の評価上限を超えない"),
			PlanetCore::PrepareSpillNetwork(Initial, Source, Anchors, IsOcean, Settings, {}, &Invalid).has_value()
		);
		TestEqual(TEXT("元場の実評価は指定上限"), Calls, UINT64_C(1));
		Settings.MaximumEvaluations = 32000000;
		Calls = 0;
		std::stop_source Stop;
		Stop.request_stop();
		TestFalse(
			TEXT("中断後は再利用枠も準備しない"),
			PlanetCore::PrepareSpillNetwork(Initial, Source, Anchors, IsOcean, Settings, Stop.get_token(), &Invalid).has_value()
		);
		TestEqual(TEXT("中断後の元場照会は零"), Calls, UINT64_C(0));
	}
	// 再利用の正確さと有界な準備の検査結果の返戻
	return !HasAnyErrors();
}

#endif
