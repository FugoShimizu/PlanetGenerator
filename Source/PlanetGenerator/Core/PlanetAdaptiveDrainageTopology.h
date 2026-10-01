#pragma once

#include "PlanetMinimalDrainageFill.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	enum class AdaptiveDrainageError : std::uint8_t {
		None,
		Cancelled,
		FillRejected,
		NumericFailure,
		InfeasibleBound,
		DrainageRejected
	};

	struct AdaptiveDrainageStatistics {
		std::uint32_t Vertices = 0;
		std::uint32_t Edges = 0;
		std::uint32_t OceanVertices = 0;
		std::uint32_t ChangedLandVertices = 0;
		std::uint32_t ChangedParentVertices = 0;
		std::uint32_t FeasibilityRuns = 0;
		double MaximumAbsoluteChangeMeters = 0;
		double MaximumRaiseMeters = 0;
		double MaximumCutMeters = 0;
		double SumAbsoluteChangeMeters = 0;
		double MinimumLandDropMeters = 0;
		double MinimumFeasibleBoundMeters = 0;
		double FixedTreeFillMaxRaiseMeters = 0;
	};

	/**
	 * 大洋を根に，流路木と節点標高を共に選び，最大絶対標高補正を最小化する
	 * 海底と元の海陸区分を保存する
	 * 有限網上の保証であり，細分後の滑らかな H や河川 W の完成保証ではない
	 */
	class AdaptiveDrainageTopology {
	private:

		std::vector<double> HeightsMeters;
		std::vector<std::uint32_t> DownstreamVertices;
		AdaptiveDrainageStatistics Stats;

		AdaptiveDrainageTopology() = default;

	public:

		static std::optional<AdaptiveDrainageTopology> Create(
			std::span<const UnitDirection> Directions,
			std::span<const std::array<std::uint32_t, 3>> Triangles,
			std::span<const double> OriginalHeightsMeters,
			const MinimalDrainageFillSettings &Settings,
			AdaptiveDrainageError &Error,
			std::stop_token Stop ={}
		);

		std::span<const double> GetHeightsMeters() const;
		std::span<const std::uint32_t> GetDownstreamVertices() const;
		const AdaptiveDrainageStatistics &GetStatistics() const;
	};
}
