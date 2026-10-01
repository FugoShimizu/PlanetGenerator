#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/** 段階６の元頂点を保持し，指定した共有辺だけを一度細分化した排水網 */
	struct DrainageRefinementMesh {
		std::vector<UnitDirection> Directions;
		std::vector<std::array<std::uint32_t, 3>> Triangles;
		// 追加頂点の順序に対応する，正規化・整列済みの元辺
		std::vector<std::array<std::uint32_t, 2>> SplitEdges;
	};

	std::optional<DrainageRefinementMesh> BuildDrainageRefinement(
		const std::span<const std::array<std::uint32_t, 2>> Edges,
		const std::stop_token Stop={}
	);
}
