#pragma once

#include "PlanetCore.h"
#include <optional>
#include <span>

namespace PlanetCore {
	/** 原点を含む上下限へ単位接線方向の微分を制限する条件 */
	struct SphericalGradientConstraint {
		Vec3 Tangent;
		double MaximumDerivative = 0;
		double MinimumDerivative = 0;
	};

	std::optional<Vec3> ProjectSphericalGradient(
		const UnitDirection Point,
		const Vec3 Gradient,
		const std::span<const SphericalGradientConstraint> Constraints
	);
}
