#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/** 標本尺度で推定した接線と，局所解法の実際の選択数 */
	struct SphericalGradientEstimate {
		// 節点順の接線と近似次数
		std::vector<Vec3> Gradients;
		std::uint32_t QuadraticVertices = 0;
		std::uint32_t LinearVertices = 0;
	};

	std::optional<SphericalGradientEstimate> EstimateSphericalGradients(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> ElevationsMeters,
		const std::stop_token Stop={}
	);
}
