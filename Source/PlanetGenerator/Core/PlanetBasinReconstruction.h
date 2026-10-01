#pragma once

#include "PlanetCore.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct BasinReconstructionStatistics {
		std::uint32_t Sites = 0;
		std::uint32_t Iterations = 0;
		std::uint64_t Couplings = 0;
		double MaximumResidualMeters = 0;
		double AbsoluteBoundMeters = 0;
		std::uint32_t LowerBounds = 0;
		double MaximumLowerBoundDeficitMeters = 0;
	};

	/** 盆地の標高補正を C2 の有限支持関数へ再構成する，海岸と排水の拘束は生成側が検査する */
	class PlanetBasinReconstruction {
	private:

		struct Site {
			Vec3 Direction;
			double CoefficientMeters = 0;
		};

		std::vector<Site> Sites;
		std::vector<std::uint32_t> Order;
		std::vector<std::uint32_t> Offsets;
		double SupportChord = 0;
		bool Normalized = false;
		BasinReconstructionStatistics Stats;

		PlanetBasinReconstruction() = default;
		static std::optional<PlanetBasinReconstruction> CreateImpl(
			const std::span<const UnitDirection> Directions,
			const std::span<const double> CorrectionsMeters,
			const double Width,
			const bool Normalize,
			const std::stop_token Stop
		);
		static std::uint32_t GridCoordinate(const double Coordinate);
		void BuildIndex();
		void Gather(const Vec3 Point, std::vector<std::uint32_t> &Candidates) const;
		template<bool Derivatives> TerrainSample EvaluateImpl(const UnitDirection Direction, const bool Indexed) const;

	public:

		static std::optional<PlanetBasinReconstruction> Create(
			const std::span<const UnitDirection> Directions,
			const std::span<const double> CorrectionsMeters,
			const double Width,
			const std::stop_token Stop={}
		);

		static std::optional<PlanetBasinReconstruction> CreateGlobal(
			const unsigned Level,
			const std::span<const double> CorrectionsMeters,
			const std::stop_token Stop={}
		);

		std::optional<PlanetBasinReconstruction> WithLowerBounds(
			const std::span<const UnitDirection> Directions,
			const std::span<const double> MinimumElevations,
			const std::stop_token Stop={}
		) const;

		TerrainSample Evaluate(const UnitDirection Direction) const;
		double EvaluateElevation(const UnitDirection Direction) const;
		TerrainSample EvaluateReference(const UnitDirection Direction) const;
		const BasinReconstructionStatistics &Statistics() const;
	};
}
