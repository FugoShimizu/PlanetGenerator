#pragma once

#include "PlanetCriticalPoint.h"
#include "PlanetDescentPath.h"
#include <cstddef>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>

namespace PlanetCore {
	struct TerrainMinimumLink {
		// 元場で確認した接続開始点と，其の前に保持する既存標本数
		TerrainDescentSample Anchor;
		std::size_t RetainedSamples = 0;
	};

	std::optional<TerrainMinimumLink> PrepareTerrainMinimumLink(
		const std::span<const TerrainDescentSample> Path,
		const TerrainCriticalPoint &Minimum,
		const double RadiusMeters,
		const double PositionToleranceMeters,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::stop_token Stop= {}
	);

	bool DescendsToTerrainMinimum(
		const TerrainDescentSample &Last,
		const TerrainCriticalPoint &Minimum,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::stop_token Stop= {}
	);
}
