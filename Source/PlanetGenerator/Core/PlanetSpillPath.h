#pragma once

#include "PlanetSpillGraph.h"
#include <functional>

namespace PlanetCore {
	struct MeasuredSpillPath {
		// 始点から指定根への実測済経路；他の辺の下限を実測値とは扱わない
		std::vector<std::uint32_t> Nodes;
		std::vector<std::uint32_t> Edges;
		double CrestMeters = 0;
		std::uint32_t EvaluatedEdges = 0;
	};

	std::optional<MeasuredSpillPath> FindMeasuredSpillPath(
		std::span<const SpillGraphSite> Sites,
		std::span<const SpillGraphEdge> LowerBounds,
		std::uint32_t Start,
		std::uint32_t Root,
		const std::function<std::optional<double>(std::uint32_t)> &MeasureEdge,
		std::stop_token Stop= {}
	);

	std::optional<MeasuredSpillPath> FindPathMeasuredSpillPath(
		std::span<const SpillGraphSite> Sites,
		std::span<const SpillGraphEdge> LowerBounds,
		std::uint32_t Start,
		std::uint32_t Root,
		const std::function<std::optional<double>(std::uint32_t)> &MeasureEdge,
		std::stop_token Stop= {}
	);
}
