#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetSpillDiscovery.h"
#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillCandidateScale, "PlanetGenerator.ContinuousSpill.CandidateScale",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 格子より小さい固定精査範囲の不足を再現し，広い解析盆地の鞍点を元場で確認する
 * @param Parameters 未使用の実行引数
 * @return 二半径・二方向で正しい峠と両下降終端，元の精度と共通予算を確認した場合に真
 */
bool FPlanetSpillCandidateScale::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const double Pi = std::acos(-1.0), Phase = std::asin(0.03);
	constexpr double Wavelength = 30000;
	std::uint32_t Cases = 0, OldRangeFailures = 0;
	for(const double Radius : { 1E6, 1E9 }) for(const double Turn : { 0.37, 0.83 }) {
		std::uint64_t Calls = 0;
		const double Cosine = std::cos(Turn), Sine = std::sin(Turn);
		const auto DirectionAt = [&](const double U, const double V) -> PlanetCore::UnitDirection {
			// 同じ解析座標から得る球面方向の返戻
			return *PlanetCore::UnitDirection::FromVector({ (Cosine * U - Sine * V) / Radius, (Sine * U + Cosine * V) / Radius, 1 });
		};
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, U = Cosine * X + Sine * Y, V = -Sine * X + Cosine * Y;
			const double DU = -100 / Wavelength * std::sin(U / Wavelength) + 0.0001, DV = 100 * V / (Wavelength * Wavelength);
			const double DX = Cosine * DU - Sine * DV, DY = Sine * DU + Cosine * DV;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 軸を格子から回した広い傾斜盆地の元標高と解析微分の返戻
			return { 10 + 100 * std::cos(U / Wavelength) + 0.0001 * U + 50 * V * V / (Wavelength * Wavelength), Gradient };
		};
		PlanetCore::TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Radius;
		const PlanetCore::UnitDirection Initial = DirectionAt((Pi - Phase) * Wavelength, 0);
		const std::optional<PlanetCore::TerrainCriticalPoint> Minimum =
		PlanetCore::RefineTerrainCriticalPoint(Initial, Source, Critical);
		// 独立な解析極小の欠落への返戻
		if(!TestTrue(TEXT("解析的な広い盆地の初期極小"), Minimum && Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum)) return false;
		PlanetCore::SpillDiscoverySettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -1000;
		Settings.InitialHalfWidthMeters = 140000;
		Settings.MaximumHalfWidthMeters = 140000;
		Settings.HalfCells = 16;
		PlanetCore::SpillPassageSettings WindowSettings;
		WindowSettings.RadiusMeters = Radius;
		WindowSettings.HalfWidthMeters = Settings.InitialHalfWidthMeters;
		WindowSettings.SpacingMeters = Settings.InitialHalfWidthMeters / Settings.HalfCells;
		const std::optional<PlanetCore::SpillWindowExit> Window =
		PlanetCore::FindSpillWindowExit(Minimum->Direction, Source, WindowSettings);
		// 実候補を欠いた旧条件の比較への返戻
		if(!TestTrue(TEXT("同じ元場と格子の候補を保持"), Window && !Window->PassCandidates.empty())) return false;
		const std::array<PlanetCore::UnitDirection, 2> Expected {
			DirectionAt(Phase * Wavelength, 0),
			DirectionAt((2 * Pi + Phase) * Wavelength, 0)
		};
		const auto Distance = [Radius](const PlanetCore::UnitDirection A, const PlanetCore::UnitDirection B) -> double {
			const PlanetCore::Vec3 P = A.Vector(), Q = B.Vector();
			// 独立な物理弧長の返戻
			return Radius * std::atan2(P.Cross(Q).Length(), P.Dot(Q));
		};
		double Best = std::numeric_limits<double>::infinity();
		PlanetCore::UnitDirection Closest = Window->PassCandidates.front();
		for(const PlanetCore::UnitDirection Candidate : Window->PassCandidates) for(const PlanetCore::UnitDirection Pass : Expected) {
			const double Separation = Distance(Candidate, Pass);
			if(Separation < Best) {
				Best = Separation;
				Closest = Candidate;
			}
		}
		PlanetCore::TerrainCriticalDiagnostics Previous;
		const std::optional<PlanetCore::TerrainCriticalPoint> OldPass =
		PlanetCore::RefineTerrainCriticalPoint(Closest, Source, Critical, {}, &Previous);
		if(Best > Critical.MaximumDisplacementMeters) {
			TestFalse(TEXT("固定２ｋｍ範囲外の鞍点を受理しない"), OldPass.has_value());
			++OldRangeFailures;
		} else {
			TestTrue(TEXT("範囲内の正規の旧精査も保持"), OldPass && OldPass->Kind == PlanetCore::TerrainCriticalKind::Saddle);
			if(OldPass) {
				const double Error = std::min(Distance(OldPass->Direction, Expected[0]), Distance(OldPass->Direction, Expected[1]));
				TestTrue(TEXT("範囲内の旧結果も独立な解析鞍点へ一致"), Error <= 0.001);
			}
		}
		const std::array<PlanetCore::TerrainCriticalPoint, 1> Minima { *Minimum };
		PlanetCore::SpillDiscoveryDiagnostics Report;
		Calls = 0;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Batch =
		PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report);
		AddInfo(
			FString::Printf(
				TEXT("Candidate scale R=%.0f turn=%.9g candidates=%llu closestMeters=%.17g oldFailure=%u accepted=%d ")
				TEXT("newFailure=%u source=%llu criticalStep=%.17g criticalRange=%.17g connections=%llu minima=%llu"),
				Radius,
				Turn,
				static_cast<unsigned long long>(Window->PassCandidates.size()),
				Best,
				static_cast<unsigned>(Previous.Failure),
				static_cast<int>(Batch.has_value()),
				static_cast<unsigned>(Report.Failure),
				static_cast<unsigned long long>(Calls),
				Report.MaximumCriticalStepMeters,
				Report.MaximumCriticalSearchRadiusMeters,
				static_cast<unsigned long long>(Batch ? Batch->Connections.size() : 0),
				static_cast<unsigned long long>(Batch ? Batch->NewMinima.size() : 0)
			)
		);
		TestEqual(TEXT("全候補と枝の元場照会を同じ予算に計上"), Report.Evaluations, Calls);
		// 未確認の候補を新接続へ置換えない返戻
		if(!TestTrue(TEXT("格子尺度から元の連続鞍点と両枝を確認"), Batch && !Batch->Connections.empty() && !Batch->NewMinima.empty())) return false;
		TestTrue(TEXT("間隔から選ぶ有界な探索範囲"), Report.MaximumCriticalSearchRadiusMeters == 2 * WindowSettings.SpacingMeters);
		for(const PlanetCore::SpillConnection &Connection : Batch->Connections) {
			const double Error =
			std::min(Distance(Connection.Pass.Direction, Expected[0]), Distance(Connection.Pass.Direction, Expected[1]));
			TestTrue(TEXT("独立な解析鞍点へ元の位置精度で一致"), Error <= 0.001);
			TestTrue(
				TEXT("元場の標高と勾配精度を維持"),
				std::abs(Source(Connection.Pass.Direction).ElevationMeters - Connection.Pass.ElevationMeters) <= 1E-6 &&
				Connection.Pass.GradientNorm <= Critical.GradientTolerance
			);
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
				TestTrue(TEXT("実下降終端は元場の極小"), Branch.Minimum && Branch.Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum);
			}
		}
		Settings.MaximumEvaluations = 25;
		Calls = 0;
		TestFalse(
			TEXT("大きな探索範囲でも共通予算を超えて公開しない"),
			PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report).has_value()
		);
		TestEqual(TEXT("元場呼出は小さい共通予算で停止"), Calls, UINT64_C(25));
		std::stop_source Stop;
		Stop.request_stop();
		Calls = 0;
		TestFalse(
			TEXT("中断後に候補を公開しない"),
			PlanetCore::FindSpillComponentExits(Minima, Source, Settings, Stop.get_token(), &Report).has_value()
		);
		TestEqual(TEXT("中断後に元場を呼出さない"), Calls, UINT64_C(0));
		++Cases;
	}
	TestTrue(TEXT("固定範囲の不足を実候補から再現"), OldRangeFailures > 0);
	AddInfo(FString::Printf(TEXT("Candidate scale cases=%u oldRangeFailures=%u"), Cases, OldRangeFailures));
	// 独立な位置・両下降枝・共通予算の検査結果の返戻
	return !HasAnyErrors();
}

#endif
