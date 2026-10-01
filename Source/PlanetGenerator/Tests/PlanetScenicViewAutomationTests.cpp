#include "Core/PlanetScenicView.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetScenicViewContracts,
	"PlanetGenerator.ScenicView.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 中間の尾根に隠れた山と露出した山を，同じ遠方標高から区別する
 * @param Parameters Automation の追加引数，未使用
 * @return 曲率・遮蔽・視線回転・不正入力の条件が全て成立する場合に真
 */
bool FPlanetScenicViewContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	constexpr double Elevation = 100;
	unsigned Cases = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		const double Span = std::min(5000.0, Radius * 0.1);
		for(const bool Rotated : { false, true }) {
			const PlanetCore::Vec3 Up = Rotated ? PlanetCore::Vec3{ 1, 0, 0 } : PlanetCore::Vec3{ 0, 0, 1 };
			const PlanetCore::Vec3 Forward = Rotated ? PlanetCore::Vec3{ 0, 0, 1 } : PlanetCore::Vec3{ 1, 0, 0 };
			const PlanetCore::Vec3 Right = Up.Cross(Forward);
			const PlanetCore::UnitDirection Anchor = *PlanetCore::UnitDirection::FromVector(Up);
			std::size_t Calls = 0;
			// 遠方の山は固定し，前景の遮蔽物だけを切替える独立な解析場
			const auto Height = [&](const PlanetCore::UnitDirection Direction, const bool Blocked) -> double {
				++Calls;
				const PlanetCore::Vec3 D = Direction.Vector();
				const double X = Radius * D.Dot(Forward) / D.Dot(Up), Y = Radius * D.Dot(Right) / D.Dot(Up), PeakX = (X - Span) / (Span * 0.2);
				const double PeakY = Y / (Span * 0.2), RidgeX = (X - Span * 0.08) / (Span * 0.04), RidgeY = Y / (Span * 0.4);
				// 同じ山体に前景の滑らかな尾根だけを加えた標高の返戻
				return Elevation + Span * 0.2 * std::exp(-PeakX * PeakX - PeakY * PeakY) +
				(Blocked ? Span * 0.06 * std::exp(-RidgeX * RidgeX - RidgeY * RidgeY) : 0);
			};
			const std::function<double(PlanetCore::UnitDirection)> OpenHeight = [&](const PlanetCore::UnitDirection D) -> double {
				// 尾根を加えない遠方の山体の返戻
				return Height(D, false);
			};
			const std::function<double(PlanetCore::UnitDirection)> BlockedHeight = [&](const PlanetCore::UnitDirection D) -> double {
				// 同じ山体と遮る前景の返戻
				return Height(D, true);
			};
			const std::optional<PlanetCore::ScenicView> Open =
			PlanetCore::EvaluateScenicView(Anchor, Forward, Radius, Elevation, 1.65, OpenHeight);
			const std::optional<PlanetCore::ScenicView> Hidden =
			PlanetCore::EvaluateScenicView(Anchor, Forward, Radius, Elevation, 1.65, BlockedHeight);
			const std::optional<PlanetCore::ScenicView> Flat = PlanetCore::EvaluateScenicView(
				Anchor,
				Forward,
				Radius,
				Elevation,
				1.65,
				[Elevation](const PlanetCore::UnitDirection) -> double {
					// 一定標高の参照球面の返戻
					return Elevation;
				}
			);
			// 不完全な視線評価の返戻
			if(!TestTrue(TEXT("Scenic profiles remain finite at all supported test radii"), Open && Hidden && Flat)) return false;
			TestTrue(TEXT("Visible mountain outranks constant-height sphere"), Open->Score > Flat->Score + 0.15);
			TestTrue(TEXT("Foreground ridge does not count its hidden mountain"), Hidden->Score < Open->Score * 0.1);
			TestTrue(TEXT("Sphere curvature alone receives no mountain bonus"), Flat->Score < 1E-10);
			TestTrue(TEXT("Occluder lies above the far silhouette"), Hidden->CenterNearAngleRadians > Hidden->CenterFarAngleRadians);
			TestTrue(TEXT("A visible peak rises beyond foreground"), Open->CenterFarAngleRadians > Open->CenterNearAngleRadians);
			TestTrue(
				TEXT("Height queries remain bounded"),
				Calls == 150 && Open->HeightEvaluations == 75 && Hidden->HeightEvaluations == 75
			);
			TestFalse(
				TEXT("Radial forward direction is rejected"),
				PlanetCore::EvaluateScenicView(Anchor, Up, Radius, Elevation, 1.65, OpenHeight).has_value()
			);
			TestFalse(
				TEXT("Nonfinite height is rejected"),
				PlanetCore::EvaluateScenicView(
					Anchor,
					Forward,
					Radius,
					Elevation,
					1.65,
					[](const PlanetCore::UnitDirection) -> double {
						// 不正な元場を模擬する非有限値の返戻
						return std::numeric_limits<double>::quiet_NaN();
					}
				).has_value()
			);
			AddInfo(
				FString::Printf(
					TEXT("Scenic radius=%.0f rotated=%d open=%.9g hidden=%.9g flat=%.9g exposed=%.6g"),
					Radius,
					Rotated,
					Open->Score,
					Hidden->Score,
					Flat->Score,
					Open->MaximumExposedRiseMeters
				)
			);
			++Cases;
		}
	}
	AddInfo(
		FString::Printf(TEXT("Scenic independent analytic cases=%u; finite visibility heuristic, not an occlusion proof"), Cases)
	);
	// Automation が各条件の失敗も保持する全条件終了の返戻
	return true;
}

#endif
