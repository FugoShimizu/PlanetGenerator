#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <optional>

namespace PlanetCore {
	struct RiverPlanformSample {
		UnitDirection Direction;
		Vec3 DerivativePerFraction;
		Vec3 SecondDerivativePerFractionSquared;
	};

	/**
	 * 端点と二階迄の接続を保つ球面上の有限な蛇行
	 * 角度座標は厳密に前進し，区間内で曲線自身が交差又は停止しない
	 * 水位と海岸の拘束は此の同じ曲線を使って準備する
	 */
	class RiverPlanform {
	private:

		Vec3 Start;
		Vec3 Along;
		Vec3 Across;
		double Arc = 0;
		double Amplitude = 0;
		double Frequency = 0;
		double Phase = 0;
		double SecondaryPhase = 0;
		std::array<double, 3> Offset(double Fraction) const;

		RiverPlanform() = default;

	public:

		static std::optional<RiverPlanform> Create(
			UnitDirection First,
			UnitDirection Last,
			double RadiusMeters,
			double BedHalfWidthMeters,
			std::uint64_t Seed
		);

		RiverPlanformSample Evaluate(double Fraction) const;
		RiverPlanformSample EvaluateAngle(double Angle) const;
		double ArcRadians() const;
		double MaximumDeviationRadians() const;
		double MaximumTangentStretch() const;

		bool IsCoveredByLinearWater(
			double EndAngle,
			const std::array<UnitDirection, 3> &Triangle,
			const std::array<double, 3> &Depths
		) const;
	};
}
