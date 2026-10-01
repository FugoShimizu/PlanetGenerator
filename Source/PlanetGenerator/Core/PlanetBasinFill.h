#pragma once

#include "PlanetBasinReconstruction.h"
#include <optional>
#include <span>
#include <stop_token>

namespace PlanetCore {
	/** 元の海域を保持し，陸上の非負な盛土だけを C2 の遷移で加える */
	class PlanetBasinFill {
	private:

		PlanetBasinReconstruction Surface;
		double SeaLevelMeters = 0;
		double CoastBandMeters = 0;
		double RelativeRaiseLimit = 0;

		static std::optional<PlanetBasinFill> CreateImpl(
			const unsigned Level,
			const std::span<const double> OriginalElevations,
			const std::span<const double> MinimumElevations,
			const double Sea,
			const double CoastBand,
			const double RelativeLimit,
			const std::stop_token Stop
		);
		template<bool Derivatives> TerrainSample ApplyImpl(const UnitDirection Direction, const TerrainSample Original) const;

		PlanetBasinFill(
			PlanetBasinReconstruction Reconstruction, const double Sea, const double CoastBand, const double RelativeLimit
		);

	public:

		static std::optional<PlanetBasinFill> Create(
			const unsigned Level,
			const std::span<const double> OriginalElevations,
			const std::span<const double> MinimumElevations,
			const double Sea,
			const double CoastBand,
			const std::stop_token Stop={}
		);

		static std::optional<PlanetBasinFill> CreateWithCoastalLimit(
			const unsigned Level,
			const std::span<const double> OriginalElevations,
			const std::span<const double> MinimumElevations,
			const double Sea,
			const double CoastBand,
			const double RelativeLimit,
			const std::stop_token Stop={}
		);
		static double CoastalRaiseCapacity(const double HeightAboveSea, const double CoastBand, const double RelativeLimit);

		/** 元地形と補間面の高い方を平滑化が越える標高上限 */
		static double SmoothingOvershootBoundMeters();

		std::optional<PlanetBasinFill> WithMinimumElevations(
			const std::span<const UnitDirection> Directions,
			const std::span<const double> OriginalElevations,
			const std::span<const double> MinimumElevations,
			const std::stop_token Stop={}
		) const;

		/** 元地形・海岸との接続前の補間面を診断する */
		TerrainSample EvaluateReconstruction(const UnitDirection Direction) const;
		TerrainSample Apply(const UnitDirection Direction, const TerrainSample Original) const;
		double ApplyElevation(const UnitDirection Direction, const double OriginalElevation) const;
		const BasinReconstructionStatistics &Statistics() const;
	};
}
