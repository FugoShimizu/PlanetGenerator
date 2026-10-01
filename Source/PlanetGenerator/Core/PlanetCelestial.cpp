#include "PlanetCelestial.h"
#include "ThirdParty/AstronomyEngine/astronomy.h"
#include <chrono>
#include <mutex>
#include <numbers>

namespace PlanetCore {
	/**
	 * 地球参照の地理座標と高度の範囲を確認する
	 * @return 有限で有効な座標の場合に真
	 */
	bool CelestialObserver::IsValid() const {
		// 方位の曖昧な経度範囲や地球中心に近い高度を使わない判定の返戻
		return std::isfinite(LatitudeDegrees) && std::abs(LatitudeDegrees) <= 90 &&
			std::isfinite(LongitudeDegrees) && std::abs(LongitudeDegrees) <= 180 &&
			std::isfinite(HeightMeters) && HeightMeters >= -12000 && HeightMeters <= 100000;
	}

	/**
	 * グレゴリオ暦の明示日時を，２０００年１月１日正午からの UTC 日数へ変換する
	 * @param Year 西暦１９００～２１００年
	 * @param Month １～１２月
	 * @param Day 暦に存在する日
	 * @param Hour ０～２３時
	 * @param Minute ０～５９分
	 * @param Second ０以上６０未満の秒，閏秒の表記は此の入口では受け付けない
	 * @return 日数，範囲外又は不正な日付の場合は空
	 */
	std::optional<double> PlanetCelestial::UtcDays(
		const int Year, const unsigned Month, const unsigned Day,
		const unsigned Hour, const unsigned Minute, const double Second
	) {
		// 無効な日付を別の日へ繰り上げない返戻
		if(Year < 1900 || Year > 2100 || Month < 1 || Month > 12 || Day < 1 || Day > 31 ||
			Hour > 23 || Minute > 59 || !std::isfinite(Second) || Second < 0 || Second >= 60) return {};
		const std::chrono::year_month_day Date{ std::chrono::year(Year), std::chrono::month(Month), std::chrono::day(Day) };
		// 存在しない日を拒否する返戻
		if(!Date.ok()) return {};
		const std::chrono::sys_days Epoch = std::chrono::year(2000) / 1 / 1;
		// 暦日と日内の秒を分け，正午を原点にする返戻
		return static_cast<double>((std::chrono::sys_days(Date) - Epoch).count()) - 0.5 +
			(Hour * 3600 + Minute * 60 + Second) / 86400;
	}

	/**
	 * 地球の地理座標に於ける，屈折を含まない太陽・月の見掛け位置を計算する
	 * @param UtcDaysSinceJ2000 ２０００年１月１日正午からの UTC 日数
	 * @param Observer 地球の緯度・経度・海抜高度，手続惑星の半径を代入しない
	 * @return 天体中心・月の明部・太陽黄経の年周位相，不正入力又は暦の失敗では空
	 */
	std::optional<CelestialState> PlanetCelestial::Evaluate(
		const double UtcDaysSinceJ2000, const CelestialObserver Observer
	) {
		// 検証対象外の年代や非有限入力を渡さない返戻
		if(!Observer.IsValid() || !std::isfinite(UtcDaysSinceJ2000) ||
			UtcDaysSinceJ2000 < -36524.5 || UtcDaysSinceJ2000 >= 36889.5) return {};
		// 上流の月計算回数に共有の書込みが有る為，ライブラリへの入口を直列化
		static std::mutex EvaluationMutex;
		const std::lock_guard<std::mutex> Lock(EvaluationMutex);
		astro_time_t Time = Astronomy_TimeFromDays(UtcDaysSinceJ2000);
		const astro_observer_t Site = Astronomy_MakeObserver(
			Observer.LatitudeDegrees, Observer.LongitudeDegrees, Observer.HeightMeters
		);
		const astro_equatorial_t Sun = Astronomy_Equator(BODY_SUN, &Time, Site, EQUATOR_OF_DATE, ABERRATION);
		const astro_equatorial_t Moon = Astronomy_Equator(BODY_MOON, &Time, Site, EQUATOR_OF_DATE, ABERRATION);
		const astro_ecliptic_t SolarPosition = Astronomy_SunPosition(Time);
		// 天体暦の失敗を有効な方向へ置き換えない返戻
		if(Sun.status != ASTRO_SUCCESS || Moon.status != ASTRO_SUCCESS || SolarPosition.status != ASTRO_SUCCESS) return {};
		const astro_horizon_t SunHorizon = Astronomy_Horizon(&Time, Site, Sun.ra, Sun.dec, REFRACTION_NONE);
		const astro_horizon_t MoonHorizon = Astronomy_Horizon(&Time, Site, Moon.ra, Moon.dec, REFRACTION_NONE);
		const astro_rotation_t Rotation = Astronomy_Rotation_EQD_HOR(&Time, Site);
		// 座標変換に失敗した場合の返戻
		if(Rotation.status != ASTRO_SUCCESS) return {};
		const astro_vector_t SunLocal = Astronomy_RotateVector(Rotation, Sun.vec);
		const astro_vector_t MoonLocal = Astronomy_RotateVector(Rotation, Moon.vec);
		// ライブラリの北・西・上を，地形の接平面で使う東・北・上へ変換
		const Vec3 SunVector{ -SunLocal.y, SunLocal.x, SunLocal.z }, MoonVector{ -MoonLocal.y, MoonLocal.x, MoonLocal.z };
		const std::optional<UnitDirection> SunDirection = UnitDirection::FromVector(SunVector);
		const std::optional<UnitDirection> MoonDirection = UnitDirection::FromVector(MoonVector);
		const std::optional<UnitDirection> LightDirection = UnitDirection::FromVector(SunVector - MoonVector);
		// 零ベクトル又は非有限な方向の場合の返戻
		if(!SunDirection || !MoonDirection || !LightDirection) return {};
		constexpr double Degrees = 180 / std::numbers::pi, AstronomicalUnitMeters = KM_PER_AU * 1000;
		CelestialState Result;
		Result.Sun = {
			SunDirection->Vector(), SunHorizon.azimuth, SunHorizon.altitude, Sun.dist * AstronomicalUnitMeters,
			2 * Degrees * std::asin(695700000 / (Sun.dist * AstronomicalUnitMeters))
		};
		Result.Moon = {
			MoonDirection->Vector(), MoonHorizon.azimuth, MoonHorizon.altitude, Moon.dist * AstronomicalUnitMeters,
			2 * Degrees * std::asin(1737400 / (Moon.dist * AstronomicalUnitMeters))
		};
		Result.MoonToSunDirection = LightDirection->Vector();
		Result.MoonIlluminatedFraction = std::clamp((1 - Result.Moon.Direction.Dot(Result.MoonToSunDirection)) * 0.5, 0.0, 1.0);
		Result.SolarYearPhase = SolarPosition.elon / 360;
		Result.GreenwichSiderealHours = Astronomy_SiderealTime(&Time);
		// 共通の時刻・観測地点で揃えた結果の返戻
		return Result;
	}
}
