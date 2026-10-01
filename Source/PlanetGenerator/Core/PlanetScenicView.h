#pragma once

#include "PlanetCore.h"
#include <cstddef>
#include <functional>
#include <optional>

namespace PlanetCore {
	struct ScenicView {
		// 有限な視線標本による景観順位；連続な遮蔽の証明ではない
		double Score = 0;
		double MaximumExposedRiseMeters = 0;
		double CenterNearAngleRadians = 0;
		double CenterFarAngleRadians = 0;
		std::size_t HeightEvaluations = 0;
	};

	std::optional<ScenicView> EvaluateScenicView(
		const UnitDirection Anchor,
		const Vec3 Forward,
		const double RadiusMeters,
		const double ElevationMeters,
		const double EyeHeightMeters,
		const std::function<double(UnitDirection)> &Height
	);
}
