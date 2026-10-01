#include "Core/PlanetSpillDiscovery.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillConnectionReuse,
	"PlanetGenerator.ContinuousSpill.ConnectionReuse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ元場で準備済の内部接続を再利用し，新しい接続と評価予算を独立に比較する
 * @param Parameters 未使用の実行引数
 * @return 物理結果を保って再追跡を省略し，不正な省略指定を拒否した場合に真
 */
bool FPlanetSpillConnectionReuse::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			const PlanetCore::Vec3 G { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 解析的な峠が周期的に並ぶ同じ不変場の返戻
			return { 4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400, G };
		};
		const double X = 40 * std::asin(0.2) + 160 * std::acos(-1.0);
		PlanetCore::SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.RadiusMeters = Radius;
		ConnectionSettings.SeaLevelMeters = -100;
		const std::optional<PlanetCore::SpillConnection> Initial =
		PlanetCore::PrepareSpillConnection(*PlanetCore::UnitDirection::FromVector({ X / Radius, 0, 1 }), Source, ConnectionSettings);
		// 再利用する実接続を準備出来ない場合の返戻
		if(!TestTrue(TEXT("再利用前の実接続を準備"), Initial.has_value())) return false;
		std::vector<PlanetCore::TerrainCriticalPoint> Minima;
		for(const PlanetCore::SpillConnectionBranch &Branch : Initial->Branches) {
			// 内部の両終端が無い場合の返戻
			if(!TestTrue(TEXT("内部接続の両極小を確認"), Branch.Minimum.has_value())) return false;
			Minima.push_back(*Branch.Minimum);
		}
		const std::array<PlanetCore::SpillConnection, 1> Completed { *Initial };
		PlanetCore::SpillDiscoverySettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -100;
		Settings.InitialHalfWidthMeters = 400;
		Settings.MaximumHalfWidthMeters = 400;
		Settings.OutletTarget = Initial->Pass.Direction;
		PlanetCore::SpillDiscoveryDiagnostics PlainReport, ReusedReport;
		Calls = 0;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Plain =
		PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &PlainReport);
		TestEqual(TEXT("通常探索の実評価回数"), Calls, PlainReport.Evaluations);
		Calls = 0;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Reused =
		PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &ReusedReport, Completed);
		TestEqual(TEXT("再利用探索の実評価回数"), Calls, ReusedReport.Evaluations);
		// 比較する両探索の失敗への返戻
		if(!TestTrue(TEXT("両探索が新しい接続を発見"), Plain.has_value() && Reused.has_value())) return false;
		TestTrue(TEXT("両枝の再追跡を省略"), ReusedReport.ReusedConnections > 0);
		TestTrue(TEXT("元場評価を削減"), ReusedReport.Evaluations < PlainReport.Evaluations);
		TestEqual(TEXT("大洋到達の判定を保持"), Reused->HasSeaHeightTerminal, Plain->HasSeaHeightTerminal);
		TestEqual(TEXT("新しい極小の数を保持"), Reused->NewMinima.size(), Plain->NewMinima.size());
		for(std::size_t I = 0; I < std::min(Plain->NewMinima.size(), Reused->NewMinima.size()); ++I) {
			const PlanetCore::TerrainCriticalPoint &A = Plain->NewMinima[I], &B = Reused->NewMinima[I];
			TestEqual(TEXT("新しい極小の標高を保持"), A.ElevationMeters, B.ElevationMeters, 0);
			TestEqual(TEXT("新しい極小の方向を保持"), (A.Direction.Vector() - B.Direction.Vector()).Length(), 0.0, 0);
		}
		std::size_t NewConnections = 0;
		for(const PlanetCore::SpillConnection &A : Plain->Connections) {
			if((A.Pass.Direction.Vector() - Initial->Pass.Direction.Vector()).Length() * Radius < 0.001) continue;
			++NewConnections;
			const std::vector<PlanetCore::SpillConnection>::const_iterator Found = std::find_if(
				Reused->Connections.begin(),
				Reused->Connections.end(),
				[&](const PlanetCore::SpillConnection &B) -> bool {
					// 新たに発見した同一の峠の照合結果の返戻
					return (A.Pass.Direction.Vector() - B.Pass.Direction.Vector()).Length() * Radius < 0.001;
				}
			);
			// 新たな接続を省略していない事の検査
			if(!TestTrue(TEXT("全ての新しい実接続を保持"), Found != Reused->Connections.end())) return false;
			for(unsigned Side = 0; Side < 2; ++Side) {
				const PlanetCore::TerrainDescentPath &P = A.Branches[Side].Path, &Q = Found->Branches[Side].Path;
				TestEqual(TEXT("新経路の標本数を保持"), P.Samples.size(), Q.Samples.size());
				for(std::size_t I = 0; I < std::min(P.Samples.size(), Q.Samples.size()); ++I) {
					TestEqual(TEXT("元標高を完全一致で保持"), P.Samples[I].ElevationMeters, Q.Samples[I].ElevationMeters, 0);
					TestEqual(TEXT("下降距離を保持"), P.Samples[I].DistanceMeters, Q.Samples[I].DistanceMeters, 0);
					TestEqual(TEXT("下降方向を保持"), (P.Samples[I].Direction.Vector() - Q.Samples[I].Direction.Vector()).Length(), 0.0, 0);
				}
			}
		}
		TestEqual(TEXT("再利用後も新しい接続だけを返す"), Reused->Connections.size(), NewConnections);
		AddInfo(
			FString::Printf(
				TEXT("Spill reuse: radius=%.0f evaluations=%llu/%llu reused=%u newConnections=%llu"),
				Radius,
				ReusedReport.Evaluations,
				PlainReport.Evaluations,
				ReusedReport.ReusedConnections,
				static_cast<unsigned long long>(NewConnections)
			)
		);
		// 入力集合外の接続を省略する誤用は，元場の照会前に拒否する
		Calls = 0;
		const std::span<const PlanetCore::TerrainCriticalPoint> Incomplete(Minima.data(), 1);
		TestFalse(
			TEXT("未接続の極小を探索から省略しない"),
			PlanetCore::FindSpillComponentExits(Incomplete, Source, Settings, {}, &ReusedReport, Completed).has_value()
		);
		TestTrue(TEXT("不正な再利用の理由"), ReusedReport.Failure == PlanetCore::SpillDiscoveryFailure::InvalidInput);
		TestEqual(TEXT("不正指定後の元場照会は零"), Calls, UINT64_C(0));
		// 再利用が有っても全窓共通の評価予算を増やさない
		Settings.MaximumEvaluations = 100;
		TestFalse(
			TEXT("再利用時も共通予算を守る"),
			PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &ReusedReport, Completed).has_value()
		);
		TestEqual(TEXT("再利用時の有限な実評価"), Calls, UINT64_C(100));
		TestTrue(TEXT("再利用時の予算不足を区別"), ReusedReport.Failure == PlanetCore::SpillDiscoveryFailure::BudgetLimit);
	}
	// 同じ物理結果と有限費用の検査結果の返戻
	return !HasAnyErrors();
}

#endif
