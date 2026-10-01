#include "Core/PlanetContinuousSpillGraph.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetContinuousSpillContracts,
	"PlanetGenerator.ContinuousSpill.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 解析場で準備した峠と極小を，標高を変えずに下流順序へ組込む
 * @param Parameters 未使用の実行引数
 * @return 実経路・水位・並換不変性と不完全な網の拒否を確認した場合に真
 */
bool FPlanetContinuousSpillContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto Field = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 下流ほど低い窪地と峠を持つ，変更しない解析場の返戻
			return { 4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400, Gradient };
		};
		const auto IsOcean = [&](const PlanetCore::UnitDirection Direction) -> bool {
			const PlanetCore::Vec3 D = Direction.Vector();
			// 解析試験で指定した下流の一水域への所属の返戻
			return D.Z > 0 && D.X < 0 && !(Field(Direction).ElevationMeters > -1);
		};
		PlanetCore::SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.RadiusMeters = Radius;
		ConnectionSettings.SeaLevelMeters = -1;
		std::vector<PlanetCore::SpillConnection> Connections;
		const double Pi = std::acos(-1.0), FirstPass = 40 * std::asin(0.2);
		for(unsigned i = 0; i < 3; ++i) {
			const double X = FirstPass + i * 80 * Pi;
			const PlanetCore::UnitDirection Candidate = *PlanetCore::UnitDirection::FromVector({ X / Radius, 0, 1 });
			PlanetCore::SpillConnectionDiagnostics Diagnostic;
			std::optional<PlanetCore::SpillConnection> Connection =
			PlanetCore::PrepareSpillConnection(Candidate, Field, ConnectionSettings, {}, &Diagnostic);
			AddInfo(
				FString::Printf(
					TEXT("Continuous input R=%.0f pass=%u accepted=%d failure=%u side=%u end=%u move=%.12g"),
					Radius,
					i,
					static_cast<int>(Connection.has_value()),
					static_cast<unsigned>(Diagnostic.Failure),
					Diagnostic.Branch,
					static_cast<unsigned>(Diagnostic.TraceEnd),
					Diagnostic.TerminalMoveMeters
				)
			);
			// 両枝の接続が未確定の場合の返戻
			if(!TestTrue(TEXT("解析場の実下降から接続を準備"), Connection.has_value())) return false;
			Connections.push_back(std::move(*Connection));
		}
		PlanetCore::ContinuousSpillSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -1;
		PlanetCore::ContinuousSpillDiagnostics Diagnostic;
		const std::optional<PlanetCore::ContinuousSpillGraph> Graph =
		PlanetCore::BuildContinuousSpillGraph(Connections, IsOcean, Settings, {}, &Diagnostic);
		AddInfo(
			FString::Printf(
				TEXT("Continuous graph R=%.0f accepted=%d failure=%u connection=%u"),
				Radius,
				static_cast<int>(Graph.has_value()),
				static_cast<unsigned>(Diagnostic.Failure),
				Diagnostic.Connection
			)
		);
		// 部分的な接続を完成した階層としない返戻
		if(!TestTrue(TEXT("元場の３つの峠を大洋へ繋ぐ"), Graph.has_value())) return false;
		TestEqual(TEXT("同じ極小を共有した４地点"), Graph->Sites.size(), std::size_t{ 4 });
		TestEqual(TEXT("実際の３つの峠を保持"), Graph->Edges.size(), std::size_t{ 3 });
		for(unsigned i = 0; i < 3; ++i) {
			const PlanetCore::SpillConnection &Connection = Connections[i];
			const unsigned UpstreamSide = Connection.Branches[0].Path.Samples.back().Direction.Vector().X >
			Connection.Branches[1].Path.Samples.back().Direction.Vector().X ? 0 : 1;
			const std::uint32_t Site = Graph->BranchSites[i][UpstreamSide], Receiver = Graph->BranchSites[i][1 - UpstreamSide];
			const std::uint32_t Root = Graph->Hierarchy.SiteRoots[Site], ReceivingRoot = Graph->Hierarchy.SiteRoots[Receiver];
			const PlanetCore::DepressionHierarchyNode &Node = Graph->Hierarchy.Nodes[Root];
			TestEqual(TEXT("水位は粗い格子でなく実鞍点の高度"), Node.SpillMeters, Connection.Pass.ElevationMeters);
			TestEqual(TEXT("下流は実際の下降枝の終端"), Node.ReceivingSite, Receiver);
			TestTrue(TEXT("下流へ水位が低下"), Graph->Hierarchy.Nodes[ReceivingRoot].SpillMeters < Node.SpillMeters);
			TestTrue(TEXT("大洋への接続順は非循環"), Graph->Hierarchy.Nodes[ReceivingRoot].OceanLinkRank < Node.OceanLinkRank);
			const PlanetCore::TerrainCriticalPoint &Minimum = *Graph->Minima[Site];
			TestEqual(TEXT("網への統合で元標高を変更しない"), Graph->Sites[Site].ElevationMeters, Field(Minimum.Direction).ElevationMeters);
		}
		std::vector<PlanetCore::SpillConnection> Reversed(Connections.rbegin(), Connections.rend());
		const std::optional<PlanetCore::ContinuousSpillGraph> Other =
		PlanetCore::BuildContinuousSpillGraph(Reversed, IsOcean, Settings);
		// 入力順を変えた正規経路が失敗した場合の返戻
		if(!TestTrue(TEXT("並換えた接続も受理"), Other.has_value())) return false;
		TestEqual(TEXT("極小の個数は列挙順に依らない"), Other->Sites.size(), Graph->Sites.size());
		for(std::size_t i = 0; i < Graph->Sites.size(); ++i) {
			TestEqual(TEXT("元標高と地点番号は不変"), Other->Sites[i].ElevationMeters, Graph->Sites[i].ElevationMeters);
			TestEqual(TEXT("所属水域は不変"), Other->Hierarchy.SiteRoots[i], Graph->Hierarchy.SiteRoots[i]);
		}
		const auto NotOcean = [](const PlanetCore::UnitDirection) -> bool {
			// 未確認の海域所属の返戻
			return false;
		};
		TestFalse(
			TEXT("海面高度だけで大洋へ接続しない"),
			PlanetCore::BuildContinuousSpillGraph(Connections, NotOcean, Settings, {}, &Diagnostic).has_value()
		);
		TestTrue(TEXT("未検証の大洋を通知"), Diagnostic.Failure == PlanetCore::ContinuousSpillFailure::UnverifiedOcean);
		std::vector<PlanetCore::SpillConnection> Broken { Connections[0], Connections[2] };
		TestFalse(TEXT("中間の接続が欠けた網を拒否"), PlanetCore::BuildContinuousSpillGraph(Broken, IsOcean, Settings, {}, &Diagnostic).has_value());
		TestTrue(TEXT("切離された窪地を通知"), Diagnostic.Failure == PlanetCore::ContinuousSpillFailure::UnconnectedNetwork);
		std::stop_source Cancel;
		Cancel.request_stop();
		TestFalse(
			TEXT("中断した階層を公開しない"),
			PlanetCore::BuildContinuousSpillGraph(Connections, IsOcean, Settings, Cancel.get_token(), &Diagnostic).has_value()
		);
		TestTrue(TEXT("中断理由を保持"), Diagnostic.Failure == PlanetCore::ContinuousSpillFailure::Cancelled);
		Settings.MinimumMergeDistanceMeters = 0;
		TestFalse(
			TEXT("無効な照合距離を拒否"),
			PlanetCore::BuildContinuousSpillGraph(Connections, IsOcean, Settings, {}, &Diagnostic).has_value()
		);
		TestTrue(TEXT("入力不正を通知"), Diagnostic.Failure == PlanetCore::ContinuousSpillFailure::InvalidInput);
	}
	// 有限な解析場での統合検査結果の返戻
	return !HasAnyErrors();
}

#endif
