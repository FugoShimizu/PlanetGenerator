#pragma once

#include "PlanetCore.h"

namespace PlanetCore {
	/** 地球上の対応観測地点，半径や表示視点とは独立した参照座標 */
	struct CelestialObserver {
		double LatitudeDegrees = 0, LongitudeDegrees = 0, HeightMeters = 0;
		bool IsValid() const;
	};

	/** 観測地点から見た天体中心と距離，東・北・上の右手系 */
	struct CelestialBodyState {
		Vec3 Direction;
		double AzimuthDegrees = 0, AltitudeDegrees = 0, DistanceMeters = 0, AngularDiameterDegrees = 0;
	};

	/** 一つの日時と地点に対する太陽・月・季節・恒星時 */
	struct CelestialState {
		CelestialBodyState Sun, Moon;
		Vec3 MoonToSunDirection;
		double MoonIlluminatedFraction = 0, SolarYearPhase = 0, GreenwichSiderealHours = 0;
	};

	/** 日時を暗黙に取得せず，固定した天体暦から天体の見掛け位置を評価する */
	class PlanetCelestial {
	public:

		static std::optional<double> UtcDays(
			const int Year, const unsigned Month, const unsigned Day,
			const unsigned Hour = 0, const unsigned Minute = 0, const double Second = 0
		);
		static std::optional<CelestialState> Evaluate(const double UtcDaysSinceJ2000, const CelestialObserver Observer);
	};
}
