#include "Core/PlanetSpillConnection.h"
#include "Misc/AutomationTest.h"
#include <cmath>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetShallowSpillConnection,
	"PlanetGenerator.SpillConnection.ShallowMinimum",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 緩い極小の手前で，終端確認より粗い勾配閾値に依り停止する回帰を検出する
 * @param Parameters 未使用の実行引数
 * @return 全半径で両側の実極小へ到達した場合に真
 */
bool FPlanetShallowSpillConnection::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Origin = *PlanetCore::UnitDirection::FromVector({ 0, 0, 1 });
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		// 高さだけを千分の一にした既知の二重井戸
		const auto Source = [Radius](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, U = X / 40, V = Y / 20, DX = (4 * U * U * U - 4 * U) / 40000;
			const double DY = V / 10000;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 元の幾何を保ち，物理勾配だけを小さくした場の返戻
			return { (U * U * U * U - 2 * U * U + V * V + 1) * 0.001, Gradient };
		};
		PlanetCore::SpillConnectionSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -1;
		Settings.TerminalPositionToleranceMeters = 0.01;
		PlanetCore::SpillConnectionDiagnostics Report;
		const std::optional<PlanetCore::SpillConnection> Connection =
		PlanetCore::PrepareSpillConnection(Origin, Source, Settings, {}, &Report);
		AddInfo(
			FString::Printf(
				TEXT("Shallow connection: radius=%.0f accepted=%d failure=%u end=%u move=%.12g"),
				Radius,
				static_cast<int>(Connection.has_value()),
				static_cast<unsigned>(Report.Failure),
				static_cast<unsigned>(Report.TraceEnd),
				Report.TerminalMoveMeters
			)
		);
		TestTrue(TEXT("緩い地形でも極小への接続を失わない"), Connection.has_value());
		if(!Connection) continue;
		for(const PlanetCore::SpillConnectionBranch &Branch : Connection->Branches) {
			TestTrue(TEXT("両側の極小を確認"), Branch.Minimum.has_value());
			if(!Branch.Minimum) continue;
			const PlanetCore::Vec3 End = Branch.Minimum->Direction.Vector();
			TestTrue(TEXT("解析解の極小位置に一センチメートル以内"), std::abs(std::abs(Radius * End.X / End.Z) - 40) < 0.01);
			TestTrue(TEXT("確認した物理勾配を保持"), Branch.Minimum->GradientNorm < 1E-8);
			for(std::size_t I = 1; I < Branch.Path.Samples.size(); ++I) {
				TestTrue(TEXT("緩い地形も元場上を下降"), Branch.Path.Samples[I].ElevationMeters <= Branch.Path.Samples[I - 1].ElevationMeters);
			}
		}
	}
	// 全条件の回帰検査の返戻
	return !HasAnyErrors();
}

#endif
