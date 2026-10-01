#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <optional>

namespace PlanetCore {
	struct SphericalWarpSettings {
		std::uint64_t Seed = 42;
		double RadiusMeters = 1000000;
		double LargestWavelengthMeters = 40000;
		double DerivativeBudget = 1.2;
		/** 有効時だけ，惑星規模から細部迄の２７個の可逆捩りを用いる */
		bool ShouldUseContinentalWavelengths = false;
		bool IsValid() const;
	};

	struct SphericalWarpMapping {
		UnitDirection Direction;
		std::array<Vec3, 3> Derivatives;
		Vec3 PullbackGradient(const Vec3 SourceGradient) const;
	};

	/** 海域の面積と接続を保存し，逆写像を持つ球面上の座標変形 */
	class PlanetSphericalWarp {
	private:

		struct Twist {
			Vec3 Axis;
			double Frequency = 0;
			double Amplitude = 0;
			double Phase = 0;
			double HarmonicPhase = 0;
			std::array<double, 2> AngleAndDerivative(const Vec3 Point) const;
		};

		static constexpr unsigned LegacyTwistCount = 9;
		static constexpr unsigned MaximumTwistCount = 27;
		std::array<Twist, MaximumTwistCount> Twists;
		unsigned TwistCount = LegacyTwistCount;
		double MaximumFrequency = 0;
		static std::uint64_t Mix(const std::uint64_t Value);
		static double Phase(const std::uint64_t Value);
		static Vec3 Rotate(const Vec3 Point, const Vec3 Axis, const double Angle);
		SphericalWarpMapping MapImpl(UnitDirection Direction, bool IsInverse) const;

		PlanetSphericalWarp() = default;

	public:

		static std::optional<PlanetSphericalWarp> Create(const SphericalWarpSettings &Settings);
		UnitDirection ToSource(const UnitDirection Direction) const;
		UnitDirection ToWorld(const UnitDirection Direction) const;
		SphericalWarpMapping MapWithDerivatives(const UnitDirection Direction) const;
		SphericalWarpMapping MapToWorldWithDerivatives(UnitDirection Direction) const;
		double GetMaximumFrequency() const;
	};
}
