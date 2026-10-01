#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t MinimalDrainageFillVersion = 1;

	struct MinimalDrainageFillSettings {
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		double MinimumLandSlope = 1E-6;
		bool IsValid() const;
	};

	enum class MinimalDrainageFillError : std::uint8_t {
		None,
		Cancelled,
		InvalidInput,
		InvalidMesh,
		NoOcean,
		NoLand,
		DisconnectedOcean,
		NumericFailure
	};

	struct MinimalDrainageFillStatistics {
		std::uint32_t Vertices = 0;
		std::uint32_t Edges = 0;
		std::uint32_t OceanVertices = 0;
		std::uint32_t RaisedLandVertices = 0;
		double MaximumRaiseMeters = 0;
		double SumRaiseMeters = 0;
		double MinimumLandDropMeters = 0;
	};

	/**
	 * 連結大洋を根に，元の地質標高を下界とする最小越流標高を構成する
	 * 陸を下げず海底を変えない
	 * 有限網の全陸下降だけを保証し，C1 又は
	 * 目視上の平坦さ・河道水位・細分後の排水は別途検証する
	 */
	class MinimalDrainageFill {
	private:

		std::vector<double> HeightsMeters;
		std::vector<std::uint32_t> DownstreamVertices;
		MinimalDrainageFillStatistics Stats;

		MinimalDrainageFill() = default;

	public:

		static std::optional<MinimalDrainageFill> Create(
			std::span<const UnitDirection> Directions,
			std::span<const std::array<std::uint32_t, 3>> Triangles,
			std::span<const double> OriginalHeightsMeters,
			const MinimalDrainageFillSettings &Settings,
			MinimalDrainageFillError &Error,
			std::stop_token Stop={}
		);

		std::span<const double> GetHeightsMeters() const;
		std::span<const std::uint32_t> GetDownstreamVertices() const;
		const MinimalDrainageFillStatistics &GetStatistics() const;
	};
}
