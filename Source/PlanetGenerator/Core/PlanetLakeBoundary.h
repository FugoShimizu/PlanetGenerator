#pragma once

#include "PlanetCriticalPoint.h"
#include "PlanetTerrainContour.h"
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct LakeBoundaryLoop {
		// 確認した弧の閉鎖列と，元の入力弧への対応
		TerrainContour Contour;
		std::vector<std::uint32_t> Arcs;
		double SignedAreaMetersSquared = 0;
	};

	// 指定した弧だけから輪郭を組立てる；未発見の島や全水域の列挙とは区別する
	std::optional<std::vector<LakeBoundaryLoop>> AssembleLakeBoundary(
		std::span<const TerrainCriticalPoint> Passes,
		std::span<const TerrainContour> Arcs,
		const TerrainContourSettings &Settings,
		std::stop_token Stop= {}
	);
}
