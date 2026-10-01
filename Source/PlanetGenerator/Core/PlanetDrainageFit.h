#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t NoDrainageFitDownstream = std::numeric_limits<std::uint32_t>::max();

	struct DrainageFitSite {
		std::uint32_t Downstream = NoDrainageFitDownstream;
		double TargetMeters = 0;
		double Weight = 1;
	};

	struct DrainageFitResult {
		std::vector<double> Levels;
		std::uint64_t HeapMoves = 0;
		std::uint32_t PooledKnots = 0;
		double WeightedSquaredChange = 0;
		double MinimumTheoreticalMaxChangeMeters = 0;
		std::uint32_t CriticalAncestor = NoDrainageFitDownstream;
		std::uint32_t CriticalSite = NoDrainageFitDownstream;
	};

	struct DrainageTerrainSite {
		std::uint32_t Downstream = NoDrainageFitDownstream;
		double DistanceToOceanMeters = 0;
		double TargetMeters = 0;
		double Weight = 1;
	};

	/** 大洋への距離に比例する厳密な下降余裕を持つ排水木の節点標高を最小二乗で割当てる */
	std::optional<DrainageFitResult> FitDrainageTerrain(
		std::span<const DrainageTerrainSite> Sites,
		double SeaLevelMeters,
		double MinimumSlope,
		std::stop_token Stop ={}
	);

	std::optional<DrainageFitResult> FitDrainageTree(
		std::span<const DrainageFitSite> Sites,
		double LowerMeters,
		std::stop_token Stop ={}
	);
}
