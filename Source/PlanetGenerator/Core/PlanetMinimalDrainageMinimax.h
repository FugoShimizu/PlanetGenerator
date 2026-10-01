#pragma once

#include "PlanetMinimalDrainageFill.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t MinimalDrainageMinimaxVersion = 1;

	enum class MinimalDrainageMinimaxError : std::uint8_t {
		None,
		FillRejected,
		NumericFailure,
		DrainageRejected,
		Cancelled,
		CoastUpper,
		EmptyWindow,
		FinalSlope,
		LevelLandEdge,
		LandSink
	};

	struct MinimalDrainageMinimaxStatistics {
		std::uint32_t ChangedLandVertices = 0;
		double MaximumAbsoluteChangeMeters = 0;
		double MaximumRaiseMeters = 0;
		double MaximumCutMeters = 0;
		double SumAbsoluteChangeMeters = 0;
		double MinimumLandDropMeters = 0;
		double MaximumTieBreakMeters = 0;
	};

	/**
	 * 固定した大洋根の排水木で，各節点の絶対標高補正の最大値を最小化する
	 * 元の海底と海陸区分を保持する
	 * 連続面と景観の品質は別途検証する
	 */
	class MinimalDrainageMinimax {
	private:

		std::vector<double> HeightsMeters;
		std::vector<std::uint32_t> DownstreamVertices;
		MinimalDrainageMinimaxStatistics Stats;

		MinimalDrainageMinimax() = default;

	public:

		static std::optional<MinimalDrainageMinimax> Create(
			std::span<const UnitDirection> Directions,
			std::span<const std::array<std::uint32_t, 3>> Triangles,
			std::span<const double> OriginalHeightsMeters,
			const MinimalDrainageFillSettings &Settings,
			MinimalDrainageMinimaxError &Error,
			std::stop_token Stop={}
		);

		std::span<const double> GetHeightsMeters() const;
		std::span<const std::uint32_t> GetDownstreamVertices() const;
		const MinimalDrainageMinimaxStatistics &GetStatistics() const;
	};
}
