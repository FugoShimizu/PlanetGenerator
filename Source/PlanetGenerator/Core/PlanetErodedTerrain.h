#pragma once

#include "PlanetFluvialErosion.h"

namespace PlanetCore {
	inline constexpr std::uint32_t ErodedTerrainVersion = 1;

	/** 固定排水の侵食を有限範囲の滑らかな関数へ再構成する，比較用の基盤 */
	class PlanetErodedTerrain {
	private:
		struct Site {
			Vec3 Direction;
			double LogReliefRatio = 0;
		};

		std::shared_ptr<const OceanConnectivityField> Source;
		std::vector<Site> Sites;
		std::vector<std::uint32_t> Order, Offsets;
		double SupportChord = 0, CoastTransitionMeters = 0;
		PlanetErodedTerrain() = default;
		static std::uint32_t GridCoordinate(const double Coordinate);
		void BuildIndex();
		template <bool HasDerivatives> TerrainSample EvaluateImpl(const UnitDirection Direction, const bool IsIndexed) const;

	public:
		static std::optional<PlanetErodedTerrain> Create(std::shared_ptr<const OceanConnectivityField> Ocean,
			const DrainageNetwork &Network, const PlanetFluvialErosion &Erosion, const std::stop_token Stop = {});

		TerrainSample Evaluate(const UnitDirection Direction) const;
		double EvaluateElevation(const UnitDirection Direction) const;
		TerrainSample EvaluateReference(const UnitDirection Direction) const;
	};
}
