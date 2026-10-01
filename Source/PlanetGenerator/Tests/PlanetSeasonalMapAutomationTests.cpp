#include "Core/PlanetSeasonalMap.h"
#include "Core/PlanetTreePalette.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSeasonalMapContracts, "PlanetGenerator.SeasonalMap.Contracts",
								 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 季節配列の実気候との対応・極・経度境界・周期を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，不一致は UE に記録する
 */
bool FPlanetSeasonalMapContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Start = FPlatformTime::Seconds();
	PlanetCore::GeologySettings Settings;
	Settings.RadiusMeters = 1000;
	const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
	// 実地質の準備に失敗した場合の返戻
	if(!TestTrue(TEXT("季節配列の地質を準備"), Geology.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
	// 海陸の準備に失敗した場合の返戻
	if(!TestTrue(TEXT("季節配列の海陸を準備"), Ocean.has_value())) return false;
	const std::optional<PlanetCore::PlanetClimate> Climate = PlanetCore::PlanetClimate::Create(*Ocean);
	// 全球気候の準備に失敗した場合の返戻
	if(!TestTrue(TEXT("季節配列の実気候を準備"), Climate.has_value())) return false;
	const double MapStart = FPlatformTime::Seconds();
	const std::optional<PlanetCore::PlanetSeasonalMap> Map = PlanetCore::PlanetSeasonalMap::Create(*Climate);
	const double MapSeconds = FPlatformTime::Seconds() - MapStart;
	// 配列の準備に失敗した場合の返戻
	if(!TestTrue(TEXT("１２期の全配列を準備"), Map.has_value())) return false;
	TestEqual(TEXT("配列容量を固定"), Map->Data().size(),
			  std::size_t(PlanetCore::PlanetSeasonalMap::Width * PlanetCore::PlanetSeasonalMap::Height *
						  PlanetCore::PlanetSeasonalMap::Layers));
	double MaximumCenterError = 0, MaximumConiferError = 0, MaximumSeamDifference = 0, MaximumCover = 0;
	std::uint32_t Samples = 0;
	for(std::uint32_t y = 0; y < PlanetCore::PlanetSeasonalMap::Height; y += 4) {
		for(std::uint32_t x = 0; x < PlanetCore::PlanetSeasonalMap::Width; x += 7) {
			const PlanetCore::UnitDirection Direction = PlanetCore::PlanetSeasonalMap::PixelDirection(x, y);
			const std::optional<PlanetCore::BiomeProfile> Profile = Climate->SampleBiome(Direction);
			// 元の気候応答が欠けた場合の返戻
			if(!TestTrue(TEXT("独立に元の気候を評価"), Profile.has_value())) return false;
			const std::optional<PlanetCore::BiomeHabitat> Habitat = Profile->Habitat({});
			if(!TestTrue(TEXT("Biome habitat is available"), Habitat.has_value())) return false;
			MaximumConiferError = std::max(
				MaximumConiferError, std::abs(PlanetCore::ConiferFraction(*Habitat) - Map->SampleConiferFraction(Direction)));
			for(std::uint32_t Period = 0; Period < PlanetCore::ClimatePeriodCount; ++Period) {
				const double Phase = static_cast<double>(Period) / PlanetCore::ClimatePeriodCount;
				const std::optional<PlanetCore::SeasonalAppearance> Appearance = Profile->Appearance(Phase, {});
				const std::optional<std::array<double, 8>> Sample = Map->Sample(Direction, Phase);
				// 配列の中心又は応答が評価出来ない場合の返戻
				if(!TestTrue(TEXT("テクセル中心の元の応答と描画参照が存在"), Appearance && Sample)) return false;
				const std::array<double, 6> Expected{Appearance->GreenGrass, Appearance->DryGrass, Appearance->LeafLitter,
													 Appearance->Snow,		 Appearance->Ice,	   Appearance->Wetness};
				for(std::size_t i = 0; i < Expected.size(); ++i) {
					MaximumCenterError = std::max(MaximumCenterError, std::abs(Expected[i] - (*Sample)[i]));
				}
				++Samples;
			}
		}
	}
	TestTrue(TEXT("単精度の格納誤差以内で元の応答と一致"), MaximumCenterError < 1.0E-7);
	TestTrue(TEXT("Conifer atlas matches canonical species mix"), MaximumConiferError < 1.0E-7);
	for(std::uint32_t i = 0; i < 257; ++i) {
		const double Z = -1 + 2 * (i + 0.5) / 257, Angle = i * 2.399963229728653;
		const double Width = std::sqrt(1 - Z * Z);
		const PlanetCore::UnitDirection Direction =
			*PlanetCore::UnitDirection::FromVector({Width * std::cos(Angle), Width * std::sin(Angle), Z});
		for(const double Phase : {-0.3, 0.0, 0.333, 0.75, 0.99999}) {
			const std::optional<std::array<double, 8>> Sample = Map->Sample(Direction, Phase),
													   Again = Map->Sample(Direction, Phase + 1);
			// 正規な地点の被覆が欠けた場合の返戻
			if(!TestTrue(TEXT("任意の地点と年周位相を評価"), Sample && Again)) return false;
			for(std::size_t j = 0; j < Sample->size(); ++j) {
				TestTrue(TEXT("被覆率と適性は有限且つ０～１"),
						 std::isfinite((*Sample)[j]) && (*Sample)[j] >= 0 && (*Sample)[j] <= 1);
				TestTrue(TEXT("一年を跨いで同じ被覆を再現"), std::abs((*Sample)[j] - (*Again)[j]) < 1.0E-12);
			}
			const double Cover = (*Sample)[0] + (*Sample)[1] + (*Sample)[2] + (*Sample)[3] + (*Sample)[4];
			MaximumCover = std::max(MaximumCover, Cover);
			++Samples;
		}
		const double Conifer = Map->SampleConiferFraction(Direction);
		TestTrue(TEXT("Conifer fraction remains finite and bounded"), std::isfinite(Conifer) && Conifer >= 0 && Conifer <= 1);
		const PlanetCore::UnitDirection Left = *PlanetCore::UnitDirection::FromVector({-Width, -1.0E-9, Z});
		const PlanetCore::UnitDirection Right = *PlanetCore::UnitDirection::FromVector({-Width, 1.0E-9, Z});
		const std::optional<std::array<double, 8>> First = Map->Sample(Left, 0.17), Second = Map->Sample(Right, 0.17);
		// 経度の継ぎ目で配列が欠けた場合の返戻
		if(!TestTrue(TEXT("経度境界の両側を評価"), First && Second)) return false;
		for(std::size_t j = 0; j < First->size(); ++j) {
			MaximumSeamDifference = std::max(MaximumSeamDifference, std::abs((*First)[j] - (*Second)[j]));
		}
		MaximumSeamDifference =
			std::max(MaximumSeamDifference, std::abs(Map->SampleConiferFraction(Left) - Map->SampleConiferFraction(Right)));
	}
	TestTrue(TEXT("被覆面積を重複しない"), MaximumCover <= 1 + 1.0E-7);
	TestTrue(TEXT("経度境界が連続"), MaximumSeamDifference < 1.0E-6);
	for(const std::uint32_t y : {0U, 64U})
		for(std::uint32_t x = 1; x < 128; ++x) {
			const PlanetCore::Vec3 First = PlanetCore::PlanetSeasonalMap::PixelDirection(0, y).Vector();
			const PlanetCore::Vec3 Second = PlanetCore::PlanetSeasonalMap::PixelDirection(x, y).Vector();
			TestTrue(TEXT("両極に経度依存の穴を作らない"), (First - Second).Length() == 0);
		}
	const PlanetCore::UnitDirection Direction = PlanetCore::PlanetSeasonalMap::PixelDirection(0, 0);
	TestFalse(TEXT("非有限な年周位相を拒否"), Map->Sample(Direction, std::numeric_limits<double>::infinity()).has_value());
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(TEXT("中断された季節配列を公開しない"),
			  PlanetCore::PlanetSeasonalMap::Create(*Climate, Stop.get_token()).has_value());
	AddInfo(FString::Printf(TEXT("Seasonal map: samples=%u centerError=%.9g coniferError=%.9g seamError=%.9g maximumCover=%.9g "
								 "bytes=%llu mapSeconds=%.6f total=%.6f"),
							Samples, MaximumCenterError, MaximumConiferError, MaximumSeamDifference, MaximumCover,
							static_cast<unsigned long long>(Map->Data().size_bytes()), MapSeconds,
							FPlatformTime::Seconds() - Start));
	// 検査実行結果の返戻
	return true;
}
#endif
