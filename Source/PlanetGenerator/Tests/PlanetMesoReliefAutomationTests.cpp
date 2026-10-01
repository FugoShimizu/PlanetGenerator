#include "Core/PlanetMesoRelief.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetMesoReliefContracts, "PlanetGenerator.Geology.MesoReliefContracts",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 半径境界，標高の再現性と解析勾配を ４条件で検査する
 * @param Parameters UE 自動検査の未使用引数
 * @return 全試料を検査出来た場合に真，失敗は Automation の記録へ残す
 */
bool FPlanetMesoReliefContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	TestFalse(TEXT("Radius below 1 km rejected"), PlanetCore::PlanetMesoRelief::Create(42, 999).has_value());
	TestFalse(TEXT("Nonfinite radius rejected"), PlanetCore::PlanetMesoRelief::Create(42, INFINITY).has_value());
	for(const std::pair<std::uint64_t, double> &Case :
	    std::array<std::pair<std::uint64_t, double>, 4>{{{42, 1000}, {42, 1000000}, {42, 1E9}, {7, 6371000}}}) {
		const std::optional<PlanetCore::PlanetMesoRelief> Field = PlanetCore::PlanetMesoRelief::Create(Case.first, Case.second);
		const std::optional<PlanetCore::PlanetMesoRelief> Repeat = PlanetCore::PlanetMesoRelief::Create(Case.first, Case.second);
		// 場を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("Meso-scale field prepared"), Field.has_value() && Repeat.has_value())) return false;
		// 大半径では方向の正規化丸め誤差より大きい差分幅を使う
		const double Step = std::max(0.02, Case.second * 1E-10) / Case.second;
		for(unsigned Index = 0; Index < 128; ++Index) {
			const double Z = 1 - 2 * (Index + 0.5) / 128, Angle = Index * 2.399963229728653;
			const double Width = std::sqrt(1 - Z * Z);
			const PlanetCore::UnitDirection Direction =
			    *PlanetCore::UnitDirection::FromVector({ Width * std::cos(Angle), Width * std::sin(Angle), Z });
			const PlanetCore::TerrainSample Sample = Field->Evaluate(Direction);
			const double Scalar = Field->EvaluateElevation(Direction);
			const double Repeated = Repeat->EvaluateElevation(Direction);
			if(!TestTrue(TEXT("Finite, bounded, repeatable scalar height"),
			             std::isfinite(Scalar) && Scalar >= 0 && Scalar <= 335 &&
			                 std::bit_cast<std::uint64_t>(Scalar) == std::bit_cast<std::uint64_t>(Sample.ElevationMeters) &&
			                 std::bit_cast<std::uint64_t>(Scalar) == std::bit_cast<std::uint64_t>(Repeated))) {
				// 標高の不正又は再現性不一致の返戻
				return false;
			}
			const PlanetCore::Vec3 Point = Direction.Vector(), Axis = Point.Cross({ 0, 0, 1 }), Tangent = Axis * (1 / Axis.Length());
			const PlanetCore::UnitDirection Before = *PlanetCore::UnitDirection::FromVector(Point - Tangent * Step);
			const PlanetCore::UnitDirection After = *PlanetCore::UnitDirection::FromVector(Point + Tangent * Step);
			const double Difference = (Field->EvaluateElevation(After) - Field->EvaluateElevation(Before)) / (2 * Step);
			const double Analytic = Sample.TangentialGradientPerRadian.Dot(Tangent);
			const double Tolerance = std::max(0.05, std::abs(Analytic) * 0.0005);
			if(!TestTrue(TEXT("Analytic tangent derivative"),
			             std::isfinite(Analytic) && std::abs(Difference - Analytic) <= Tolerance)) {
				// 接線勾配の不一致の返戻
				return false;
			}
		}
	}
	// 全条件の検査完了の返戻
	return true;
}
#endif
