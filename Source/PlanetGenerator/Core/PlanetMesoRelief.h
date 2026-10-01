#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <optional>

namespace PlanetCore {
	/** Seed と半径だけから球面上の中距離起伏を評価する不変な解析場 */
	class PlanetMesoRelief {
	private:

		struct Wave {
			Vec3 Axis;
			double Frequency = 0;
			double Phase = 0;
		};

		std::array<Wave, 12> Waves;
		std::array<double, 4> BandAmplitudesMeters {};

		static std::optional<PlanetMesoRelief> CreateImpl(std::uint64_t Seed, double RadiusMeters, bool GlobalWavelengths);
		template<bool Derivatives> TerrainSample EvaluateImpl(UnitDirection Direction) const;

	public:

		static std::optional<PlanetMesoRelief> Create(std::uint64_t Seed, double RadiusMeters);
		static std::optional<PlanetMesoRelief> CreateForDrainage(std::uint64_t Seed, double RadiusMeters);
		TerrainSample Evaluate(UnitDirection Direction) const;
		double EvaluateElevation(UnitDirection Direction) const;
	};
} // namespace PlanetCore
