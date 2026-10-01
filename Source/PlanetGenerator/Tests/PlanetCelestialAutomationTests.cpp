#include "Core/PlanetCelestial.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <limits>
#include <numbers>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetCelestialContracts, "PlanetGenerator.Celestial.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 独立な JPL Horizons の天体中心と，暦・地理座標の拒否条件を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，不一致は UE へ記録する
 */
bool FPlanetCelestialContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Started = FPlatformTime::Seconds();
	struct Reference {
		double Days, Latitude, Longitude, Height;
		bool IsMoon;
		double Azimuth, Altitude, IlluminatedFraction;
	};
	// 取得条件・未加工の応答は Documents/Research/Metadata/Horizons_Celestial_Reference.json
	// 地球地理座標・UTC・屈折なし・天体中心，四地点の昼夜を含む独立データ
	constexpr std::array<Reference, 32> References{{
		{ 9755.5, 35.6, 139.7, 0.0, false, 123.704282, 40.939837, 1.0 },
		{ 9755.75, 35.6, 139.7, 0.0, false, 246.743181, 32.234920, 1.0 },
		{ 9756.0, 35.6, 139.7, 0.0, false, 307.560913, -37.621963, 1.0 },
		{ 9756.25, 35.6, 139.7, 0.0, false, 63.115854, -29.316740, 1.0 },
		{ 9755.5, 35.6, 139.7, 0.0, true, 104.320494, -27.144152, 0.3336387 },
		{ 9755.75, 35.6, 139.7, 0.0, true, 162.168713, 24.755815, 0.3532329 },
		{ 9756.0, 35.6, 139.7, 0.0, true, 236.266537, -1.138573, 0.3682092 },
		{ 9756.25, 35.6, 139.7, 0.0, true, 285.113137, -68.250197, 0.3939778 },
		{ 8844.5, 0.0, 0.0, 0.0, false, 268.430897, -88.135113, 1.0 },
		{ 8844.75, 0.0, 0.0, 0.0, false, 89.952242, -1.848273, 1.0 },
		{ 8845.0, 0.0, 0.0, 0.0, false, 85.415287, 88.166712, 1.0 },
		{ 8845.25, 0.0, 0.0, 0.0, false, 270.245389, 1.806194, 1.0 },
		{ 8844.5, 0.0, 0.0, 0.0, true, 300.040310, 33.069255, 0.7620143 },
		{ 8844.75, 0.0, 0.0, 0.0, true, 304.030249, -44.599300, 0.7843044 },
		{ 8845.0, 0.0, 0.0, 0.0, true, 59.776453, -40.111188, 0.8117541 },
		{ 8845.25, 0.0, 0.0, 0.0, true, 60.899632, 39.058939, 0.8290647 },
		{ 8936.5, 89.9, 45.0, 0.0, false, 44.629035, 23.362303, 1.0 },
		{ 8936.75, 89.9, 45.0, 0.0, false, 134.615085, 23.504980, 1.0 },
		{ 8937.0, 89.9, 45.0, 0.0, false, 224.663057, 23.506714, 1.0 },
		{ 8937.25, 89.9, 45.0, 0.0, false, 314.649901, 23.365723, 1.0 },
		{ 8936.5, 89.9, 45.0, 0.0, true, 252.512436, -25.626177, 0.9517801 },
		{ 8936.75, 89.9, 45.0, 0.0, true, 339.309938, -26.499383, 0.9619618 },
		{ 8937.0, 89.9, 45.0, 0.0, true, 66.079199, -27.118249, 0.9709591000000001 },
		{ 8937.25, 89.9, 45.0, 0.0, true, 152.704213, -27.577135, 0.9787121999999999 },
		{ 9485.5, -33.9, 151.2, 2000.0, false, 74.506412, 63.182252, 1.0 },
		{ 9485.75, -33.9, 151.2, 2000.0, false, 263.822834, 35.663113, 1.0 },
		{ 9486.0, -33.9, 151.2, 2000.0, false, 209.166332, -26.641503, 1.0 },
		{ 9486.25, -33.9, 151.2, 2000.0, false, 125.384512, -8.055001, 1.0 },
		{ 9485.5, -33.9, 151.2, 2000.0, true, 90.354000, 54.850710, 0.0102766 },
		{ 9485.75, -33.9, 151.2, 2000.0, true, 266.496176, 48.673212, 0.013207100000000001 },
		{ 9486.0, -33.9, 151.2, 2000.0, true, 221.900118, -16.204381, 0.018942 },
		{ 9486.25, -33.9, 151.2, 2000.0, true, 141.744301, -19.473481, 0.028962599999999998 }
	}};
	double MaximumAngleDegrees = 0, MaximumIlluminationError = 0;
	for(const Reference &Expected : References) {
		const std::optional<PlanetCore::CelestialState> Actual = PlanetCore::PlanetCelestial::Evaluate(
			Expected.Days, { Expected.Latitude, Expected.Longitude, Expected.Height }
		);
		// 有効な参照条件の計算失敗では検査を終了する返戻
		if(!TestTrue(TEXT("有効な日時と地点の天体位置を計算"), Actual.has_value())) return false;
		const PlanetCore::CelestialBodyState &Body = Expected.IsMoon ? Actual->Moon : Actual->Sun;
		const double Azimuth = Expected.Azimuth * std::numbers::pi / 180;
		const double Altitude = Expected.Altitude * std::numbers::pi / 180;
		const PlanetCore::Vec3 Direction{
			std::cos(Altitude) * std::sin(Azimuth), std::cos(Altitude) * std::cos(Azimuth), std::sin(Altitude)
		};
		const double Angle = std::atan2(Body.Direction.Cross(Direction).Length(), Body.Direction.Dot(Direction)) *
			180 / std::numbers::pi;
		MaximumAngleDegrees = std::max(MaximumAngleDegrees, Angle);
		TestTrue(TEXT("天体中心の角距離が暫定目標０．１度以内"), std::isfinite(Angle) && Angle < 0.1);
		TestTrue(TEXT("見掛けの角直径が有限で正"), Body.AngularDiameterDegrees > 0.4 && Body.AngularDiameterDegrees < 0.6);
		TestTrue(TEXT("季節の位相が０以上１未満"), Actual->SolarYearPhase >= 0 && Actual->SolarYearPhase < 1);
		TestTrue(TEXT("恒星時が０以上２４時間未満"), Actual->GreenwichSiderealHours >= 0 && Actual->GreenwichSiderealHours < 24);
		TestTrue(TEXT("表示方向の長さが一"), std::abs(Body.Direction.Length() - 1) < 1.0E-12);
		const double AltitudeCheck = std::asin(Body.Direction.Z) * 180 / std::numbers::pi;
		TestTrue(TEXT("地平座標と表示用の方向が一致"), std::abs(AltitudeCheck - Body.AltitudeDegrees) < 1.0E-9);
		if(Expected.IsMoon) {
			const double Error = std::abs(Actual->MoonIlluminatedFraction - Expected.IlluminatedFraction);
			MaximumIlluminationError = std::max(MaximumIlluminationError, Error);
			TestTrue(TEXT("月の明部面積が独立参照と０．２パーセント以内"), Error < 0.002);
		}
	}
	TestTrue(TEXT("２０００年の正午が原点"), PlanetCore::PlanetCelestial::UtcDays(2000, 1, 1, 12) == 0);
	TestTrue(TEXT("２０００年の閏日を受理"), PlanetCore::PlanetCelestial::UtcDays(2000, 2, 29).has_value());
	TestFalse(TEXT("１９００年に存在しない閏日を拒否"), PlanetCore::PlanetCelestial::UtcDays(1900, 2, 29).has_value());
	TestFalse(TEXT("２１００年に存在しない閏日を拒否"), PlanetCore::PlanetCelestial::UtcDays(2100, 2, 29).has_value());
	TestFalse(TEXT("存在しない月を拒否"), PlanetCore::PlanetCelestial::UtcDays(2026, 13, 1).has_value());
	TestFalse(TEXT("繰上げを伴う時刻を拒否"), PlanetCore::PlanetCelestial::UtcDays(2026, 1, 1, 24).has_value());
	TestFalse(TEXT("上限年代外を拒否"), PlanetCore::PlanetCelestial::Evaluate(36889.5, {}).has_value());
	TestTrue(TEXT("上限年代の最終日を受理"), PlanetCore::PlanetCelestial::Evaluate(36888.5, {}).has_value());
	TestTrue(TEXT("下限年代の初日を受理"), PlanetCore::PlanetCelestial::Evaluate(-36524.5, {}).has_value());
	TestFalse(TEXT("非有限の時刻を拒否"), PlanetCore::PlanetCelestial::Evaluate(std::numeric_limits<double>::infinity(), {}).has_value());
	TestFalse(TEXT("範囲外の緯度を拒否"), PlanetCore::PlanetCelestial::Evaluate(0, { 91, 0, 0 }).has_value());
	const std::optional<PlanetCore::CelestialState> West = PlanetCore::PlanetCelestial::Evaluate(0, { 0, -180, 0 });
	const std::optional<PlanetCore::CelestialState> East = PlanetCore::PlanetCelestial::Evaluate(0, { 0, 180, 0 });
	TestTrue(TEXT("日付変更線の同一点を再現"), West && East && (West->Moon.Direction - East->Moon.Direction).Length() < 1.0E-12);
	AddInfo(FString::Printf(TEXT("Celestial references=%llu maximumAngleDegrees=%.9g illuminationError=%.9g seconds=%.6f"),
		static_cast<unsigned long long>(References.size()), MaximumAngleDegrees, MaximumIlluminationError,
		FPlatformTime::Seconds() - Started));
	// 検査実行の返戻
	return true;
}
#endif
