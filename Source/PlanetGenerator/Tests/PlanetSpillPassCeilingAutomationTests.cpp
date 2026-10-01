#include "Core/PlanetSpillDiscovery.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillPassCeilingContracts,
	"PlanetGenerator.ContinuousSpill.PassCeiling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 高い前方出口を指定した解析場で，呼出側の実峠上限と逆側の低い出口を検証する
 * @param Parameters 未使用の実行引数
 * @return 正規の低い接続・上限による未発見・不正入力を確認した場合に真
 */
bool FPlanetSpillPassCeilingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			const PlanetCore::Vec3 G { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 峠の位置と高さを独立に計算出来る，傾いた周期場の返戻
			return { 4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400, G };
		};
		const double Pi = std::acos(-1.0), Phase = std::asin(0.2), MinimumX = 40 * (5 * Pi - Phase);
		PlanetCore::TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Radius;
		Critical.MaximumDisplacementMeters = std::min(Critical.MaximumDisplacementMeters, Radius * 0.125);
		const std::optional<PlanetCore::TerrainCriticalPoint> Minimum =
		PlanetCore::RefineTerrainCriticalPoint(*PlanetCore::UnitDirection::FromVector({ MinimumX / Radius, 0, 1 }), Source, Critical);
		// 独立な出発極小の欠落への返戻
		if(!TestTrue(TEXT("解析極小を準備"), Minimum && Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum)) return false;
		const std::array<PlanetCore::TerrainCriticalPoint, 1> Minima { *Minimum };
		const double LowX = 40 * (4 * Pi + Phase), LowHeight = 4 * std::sqrt(0.96) + 0.02 * LowX + 2, Ceiling = LowHeight + 1;
		PlanetCore::SpillDiscoverySettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -1;
		Settings.InitialHalfWidthMeters = Settings.MaximumHalfWidthMeters = 400;
		Settings.InitialWindowOnly = true;
		Settings.OutletTarget = *PlanetCore::UnitDirection::FromVector({ (MinimumX + 400) / Radius, 0, 1 });
		Calls = 0;
		PlanetCore::SpillDiscoveryDiagnostics UnboundedReport;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Unbounded =
		PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &UnboundedReport);
		// 正規の上限無しの経路を先に確認する返戻
		if(!TestTrue(TEXT("前方の高い出口も既定では正規"), Unbounded.has_value())) return false;
		const bool HasHighPass = std::any_of(
			Unbounded->Connections.begin(),
			Unbounded->Connections.end(),
			[Ceiling](const PlanetCore::SpillConnection &Connection) -> bool {
				// 独立な低い峠より高い，前方の実接続を確認した返戻
				return Connection.Pass.ElevationMeters >= Ceiling;
			}
		);
		TestTrue(TEXT("前方候補は独立な峠上限より高い"), HasHighPass);
		const std::uint64_t UnboundedCalls = Calls;
		Settings.MaximumPassMeters = Ceiling;
		Calls = 0;
		PlanetCore::SpillDiscoveryDiagnostics BoundedReport;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Bounded =
		PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &BoundedReport);
		// 高い標高候補の後に，逆側の低い解析勾配候補へ進める返戻
		if(!TestTrue(TEXT("同じ窓の低い実出口を保持"), Bounded.has_value())) return false;
		bool HasLowPass = false;
		for(const PlanetCore::SpillConnection &Connection : Bounded->Connections) {
			const PlanetCore::Vec3 P = Connection.Pass.Direction.Vector();
			HasLowPass = HasLowPass || std::abs(Radius * P.X / P.Z - LowX) < 0.001;
			TestTrue(TEXT("準備した実峠は呼出側の上限未満"), Connection.Pass.ElevationMeters < Ceiling);
			TestTrue(
				TEXT("元の H を変更せず実峠を保持"),
				std::abs(Source(Connection.Pass.Direction).ElevationMeters - Connection.Pass.ElevationMeters) < 1E-6
			);
		}
		TestTrue(TEXT("独立な解析解の低い峠を発見"), HasLowPass);
		TestTrue(TEXT("高い峠を未解決又は正規出口へ混同しない"), BoundedReport.ExcludedHighPasses > 0);
		TestTrue(TEXT("有界な実評価を記録"), BoundedReport.Evaluations <= Settings.MaximumEvaluations);
		AddInfo(
			FString::Printf(
				TEXT("Pass ceiling R=%.0f highExcluded=%u lowX=%.17g unboundedCalls=%llu boundedCalls=%llu"),
				Radius,
				BoundedReport.ExcludedHighPasses,
				LowX,
				UnboundedCalls,
				BoundedReport.Evaluations
			)
		);
		Settings.MaximumPassMeters = Minimum->ElevationMeters + 0.1;
		PlanetCore::SpillDiscoveryDiagnostics TooLow;
		TestFalse(TEXT("元盆地より低い無関係の峠を出口にしない"), PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &TooLow).has_value());
		TestTrue(TEXT("低い上限での未発見を入力不正にしない"), TooLow.Failure == PlanetCore::SpillDiscoveryFailure::NoIncidentExit);
		for(
			const double Invalid :
			{ Settings.SeaLevelMeters, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() }
		) {
			Settings.MaximumPassMeters = Invalid;
			Calls = 0;
			PlanetCore::SpillDiscoveryDiagnostics InvalidReport;
			TestFalse(
				TEXT("非有限又は海面以下の峠上限を拒否"),
				PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &InvalidReport).has_value()
			);
			TestTrue(TEXT("上限の入力不正を報告"), InvalidReport.Failure == PlanetCore::SpillDiscoveryFailure::InvalidInput);
			TestEqual(TEXT("不正な上限では元場照会無し"), Calls, UINT64_C(0));
		}
	}
	// 独立な解析解・正規入力・未発見・不正入力の全結果の返戻
	return !HasAnyErrors();
}

#endif
