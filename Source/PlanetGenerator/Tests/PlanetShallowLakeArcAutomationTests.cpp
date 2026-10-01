#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetSpillLake.h"
#include "Core/PlanetTerrainContour.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetShallowLakeArc, "PlanetGenerator.SpillLake.ShallowDeparture",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 鞍点近傍の小さい勾配でも，物理的な閉鎖精度を維持して正則な湖岸を辿る
 * @param Parameters 未使用の実行引数
 * @return 三半径・二曲率・二標高・両側で独立な輪郭と湖側の所属を確認した場合に真
 */
bool FPlanetShallowLakeArc::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Origin = *PlanetCore::UnitDirection::FromVector({ 1, 0, 0 });
	std::uint32_t Cases = 0;
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		for(const double Scale : { 0.001, 0.00001 }) for(const double Baseline : { 0.0, 100.0 }) {
			const auto Source = [Radius, Scale, Baseline](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
				const PlanetCore::Vec3 D = Direction.Vector();
				const double X = Radius * D.Y, Y = Radius * D.Z, Squared = X * X;
				const double H = Baseline + Scale * (-0.00005 * Squared + 6.25E-8 * Squared * Squared + 0.0001 * Y * Y);
				const PlanetCore::Vec3 Gradient { 0, Scale * Radius * X * (-0.0001 + 2.5E-7 * Squared), Scale * Radius * 0.0002 * Y };
				// 高さの基準と曲率を独立に変える既知の二重窪地の返戻
				return { H, Gradient - D * Gradient.Dot(D) };
			};
			PlanetCore::TerrainCriticalSettings Critical;
			Critical.RadiusMeters = Radius;
			Critical.MaximumDisplacementMeters = std::min(Critical.MaximumDisplacementMeters, Radius * 0.125);
			const std::optional<PlanetCore::TerrainCriticalPoint> Pass = PlanetCore::RefineTerrainCriticalPoint(Origin, Source, Critical);
			// 非退化な解析鞍点の欠落への返戻
			if(!TestTrue(TEXT("浅い解析鞍点を精査"), Pass && Pass->Kind == PlanetCore::TerrainCriticalKind::Saddle)) return false;
			for(const bool IsPositive : { false, true }) {
				const PlanetCore::Vec3 Along = Pass->MinimumCurvatureDirection * (IsPositive ? 1.0 : -1.0);
				const PlanetCore::Vec3 Across = Origin.Vector().Cross(Along);
				const double Ratio = std::sqrt(-Pass->MinimumCurvaturePerMeter / Pass->MaximumCurvaturePerMeter);
				const PlanetCore::Vec3 Arm = PlanetCore::UnitDirection::FromVector(Along - Across * Ratio)->Vector();
				PlanetCore::TerrainContourSettings Settings;
				Settings.RadiusMeters = Radius;
				Settings.MaximumLengthMeters = std::min(10000.0, Radius * 4);
				Settings.LevelMeters = Baseline;
				Settings.Terminals.push_back(Origin);
				const double Angle = Settings.TerminalRadiusMeters * 0.5 / Radius;
				const PlanetCore::UnitDirection Initial =
				*PlanetCore::UnitDirection::FromVector(Origin.Vector() * std::cos(Angle) + Arm * std::sin(Angle));
				const std::optional<PlanetCore::TerrainContour> OldContour = PlanetCore::TraceTerrainContour(Initial, Source, Settings);
				TestTrue(
					TEXT("小さい始点勾配を停留とする旧条件を独立に再現"),
					OldContour && OldContour->Samples.size() == 1 && OldContour->End == PlanetCore::TerrainContourEnd::CriticalPoint
				);
				PlanetCore::SpillLakeDiagnostics Report;
				const std::optional<PlanetCore::SpillLakeShore> Shore =
				PlanetCore::PrepareSpillLakeShore(*Pass, IsPositive, Source, Settings, {}, &Report);
				AddInfo(
					FString::Printf(
						TEXT("Shallow shore radius=%.0f scale=%.9g base=%.0f side=%d accepted=%d failure=%u end=%u ")
						TEXT("vertices=%u gap=%.17g area=%.17g residual=%.17g"),
						Radius,
						Scale,
						Baseline,
						static_cast<int>(IsPositive),
						static_cast<int>(Shore.has_value()),
						static_cast<unsigned>(Report.Failure),
						static_cast<unsigned>(Report.End),
						Report.Vertices,
						Report.FirstGapMeters,
						Report.AreaMetersSquared,
						Report.HeightResidualMeters
					)
				);
				// 開いた岸又は追跡失敗を所属済の湖へ読み替えない返戻
				if(!TestTrue(TEXT("閉鎖精度を保って同じ鞍点へ帰還"), Shore.has_value())) return false;
				TestTrue(
					TEXT("元の閉鎖精度と高さ精度を維持"),
					Report.FirstGapMeters <= Settings.ChordToleranceMeters && Report.HeightResidualMeters <= Settings.HeightToleranceMeters &&
					Report.Vertices <= Settings.MaximumVertices
				);
				// 連続式の平面上の面積は８００√２／３；球面歪と折線近似だけを許容
				const double ExpectedArea = 800 * std::sqrt(2.0) / 3;
				TestTrue(TEXT("解析的な片側の面積と一致"), std::abs(Report.AreaMetersSquared - ExpectedArea) < 5);
				const PlanetCore::UnitDirection Inside =
				*PlanetCore::UnitDirection::FromVector(Origin.Vector() * std::cos(20 / Radius) + Along * std::sin(20 / Radius));
				const PlanetCore::UnitDirection Opposite =
				*PlanetCore::UnitDirection::FromVector(Origin.Vector() * std::cos(20 / Radius) - Along * std::sin(20 / Radius));
				TestTrue(TEXT("同じ元地形の窪地側の所属"), Source(Inside).ElevationMeters < Baseline && Shore->Contains(Inside));
				TestFalse(TEXT("鞍点の反対側を湖へ混ぜない"), Shore->Contains(Opposite));
				++Cases;
			}
		}
	}
	AddInfo(FString::Printf(TEXT("Shallow lake departure cases=%u"), Cases));
	// 独立な両側輪郭と物理精度の検査結果の返戻
	return !HasAnyErrors();
}

#endif
