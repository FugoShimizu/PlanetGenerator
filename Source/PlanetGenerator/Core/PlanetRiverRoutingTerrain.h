#pragma once

#include "PlanetCore.h"
#include "PlanetRiverRouting.h"
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct RiverRoutingTerrainSettings {
		double RadiusMeters = 1000000;
		double BankHalfWidthMeters = 20;
		double MaximumSampleStepMeters = 25;
		double MinimumDepthMeters = 0.3;
		double MaximumDepthMeters = 10;
		double MaximumCutMeters = 20;
		double FreeboardMeters = 0.3;
		double MinimumDownstreamSlope = 1E-7;
		std::uint64_t MaximumEvaluations = 1000000;
	};

	enum class RiverRoutingTerrainError : std::uint8_t {
		None,
		InvalidInput,
		EvaluationBudget,
		InvalidTerrain,
		NumericFailure,
		Cancelled
	};

	struct RiverRoutingTerrainGraph {
		std::vector<RiverRoutingWindow> Sites;
		std::vector<RiverRoutingEdge> Edges;
		std::uint64_t Evaluations = 0;
		std::uint32_t RejectedUndirectedEdges = 0;
	};

	/** 実地形の中心・左右岸を同じ球面場から採り，既存水域が所有する端点の外岸拘束を分離する */
	std::optional<RiverRoutingTerrainGraph> PrepareRiverRoutingTerrainGraph(
		std::span<const UnitDirection> Directions,
		std::span<const std::array<std::uint32_t, 2>> UndirectedEdges,
		const std::function<double(UnitDirection)> &Height,
		const RiverRoutingTerrainSettings &Settings,
		RiverRoutingTerrainError &Error,
		std::stop_token Stop={},
		std::span<const std::uint32_t> WaterBodyOwnedNodes=std::span<const std::uint32_t>(),
		std::span<const double> EdgeBankHalfWidthsMeters=std::span<const double>()
	);
}
