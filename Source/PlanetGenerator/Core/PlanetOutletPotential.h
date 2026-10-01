#pragma once

#include "PlanetDrainage.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t OutletPotentialPrototypeVersion = 1;

	struct OutletPotentialStatistics {
		std::uint32_t Nodes = 0, LandNodes = 0, OceanNodes = 0, Iterations = 0;
		double MaximumElevationMeters = 0, MinimumDownhillMeters = 0, MaximumResidual = 0;
	};

	/** 大洋を零境界とする正の隆起源の離散 Poisson 試作．最終の連続地形ではない */
	class PlanetOutletPotential {
	private:
		std::vector<double> Heights;
		OutletPotentialStatistics Stats;

	public:
		static std::optional<PlanetOutletPotential> Create(std::span<const DrainageSite> Sites,
			std::span<const std::array<std::uint32_t, 2>> Edges, std::span<const double> PositiveSources,
			double TargetMaximumMeters, std::stop_token Stop = {});
		std::span<const double> Elevations() const;
		const OutletPotentialStatistics &Statistics() const;
	};
} // namespace PlanetCore
