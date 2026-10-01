#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/** 元頂点を保持し，指定地点の周囲だけを分割した排水網 */
	struct DrainageSamplingMesh {
		// 元の節点番号を保持する球面の位置と接続
		std::vector<UnitDirection> Directions;
		std::vector<std::array<std::uint32_t, 3>> Triangles;
		// 入力順の標本に対応する頂点番号，重複や既存頂点も対応を保持する
		std::vector<std::uint32_t> SampleNodes;
	};

	std::optional<DrainageSamplingMesh> BuildDrainageSamplingMesh(
		const std::span<const UnitDirection> Samples,
		const std::stop_token Stop = {}
	);
	/** 段階６の元頂点・元面を保持した適応網へ，既存辺を組替えずに標本を挿入する */
	std::optional<DrainageSamplingMesh> BuildDrainageSamplingMeshOnMesh(
		const std::span<const UnitDirection> InitialDirections,
		const std::span<const std::array<std::uint32_t, 3>> InitialTriangles,
		const std::span<const UnitDirection> Samples,
		const std::stop_token Stop = {}
	);

}
