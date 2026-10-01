#include "Core/PlanetTreePalette.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetTreePaletteContracts, "PlanetGenerator.TreePalette.Contracts",
								 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 常緑林の樹種と固定散布候補の受入れが一致する事を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真
 */
bool FPlanetTreePaletteContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Start = FPlatformTime::Seconds();
	PlanetCore::BiomeHabitat Habitat;
	Habitat.TreeSuitability = 0.8;
	Habitat.Weights[static_cast<unsigned>(PlanetCore::LandBiome::Taiga)] = 1;
	TestEqual(TEXT("Taiga forest-floor share"), PlanetCore::ConiferFraction(Habitat), 1.0);
	TestTrue(TEXT("Taiga accepts conifer"), PlanetCore::SelectTreeKind(Habitat, 0.4, 0.6) == PlanetCore::TreeKind::Conifer);
	TestFalse(TEXT("Taiga rejects an unsuitable candidate"), PlanetCore::SelectTreeKind(Habitat, 0.81, 0.6).has_value());
	Habitat.Weights[static_cast<unsigned>(PlanetCore::LandBiome::Taiga)] = 0;
	Habitat.Weights[static_cast<unsigned>(PlanetCore::LandBiome::TropicalRainforest)] = 1;
	TestEqual(TEXT("Tropical broadleaf floor"), PlanetCore::ConiferFraction(Habitat), 0.0);
	TestTrue(TEXT("Tropical forest accepts broadleaf"),
			 PlanetCore::SelectTreeKind(Habitat, 0.4, 0.2) == PlanetCore::TreeKind::Broadleaf);
	Habitat.Weights[static_cast<unsigned>(PlanetCore::LandBiome::TropicalRainforest)] = 0;
	Habitat.Weights[static_cast<unsigned>(PlanetCore::LandBiome::TemperateRainforest)] = 1;
	TestEqual(TEXT("Temperate mixed forest floor"), PlanetCore::ConiferFraction(Habitat), 0.5);
	TestTrue(TEXT("Temperate rainforest mixes conifer"),
			 PlanetCore::SelectTreeKind(Habitat, 0.4, 0.2) == PlanetCore::TreeKind::Conifer);
	TestTrue(TEXT("Temperate rainforest mixes broadleaf"),
			 PlanetCore::SelectTreeKind(Habitat, 0.4, 0.8) == PlanetCore::TreeKind::Broadleaf);
	Habitat.TreeSuitability = 0;
	TestFalse(TEXT("Water or unsuitable ground carries no trees"), PlanetCore::SelectTreeKind(Habitat, 0, 0).has_value());
	Habitat.TreeSuitability = 1;
	TestFalse(TEXT("Reject nonfinite canonical selection"),
			  PlanetCore::SelectTreeKind(Habitat, std::numeric_limits<double>::quiet_NaN(), 0).has_value());
	AddInfo(FString::Printf(TEXT("Tree palette contracts passed in %.3f ms"), (FPlatformTime::Seconds() - Start) * 1000));
	return true;
}
#endif
