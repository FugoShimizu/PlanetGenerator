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
	inline constexpr std::uint32_t OceanDistanceTerrainVersion = 1;

	struct OceanDistanceTerrainSettings {
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		double MaximumReliefMeters = 5000;
		bool IsValid() const;
	};

	enum class OceanDistanceTerrainError : std::uint8_t {
		None,
		Cancelled,
		InvalidInput,
		InvalidMesh,
		NoOcean,
		NoLand,
		DisconnectedOcean,
		NumericFailure
	};

	struct OceanDistanceTerrainStatistics {
		std::uint32_t Vertices = 0;
		std::uint32_t Edges = 0;
		std::uint32_t OceanVertices = 0;
		std::uint32_t LandVertices = 0;
		double MaximumWeightedDistanceMeters = 0;
		double MaximumLandHeightMeters = 0;
		double MinimumLandDropMeters = 0;
	};

	/**
	 * 連結大洋を根に正の重み付き球面距離を解き，最終節点標高を下流経路から構成する
	 * 負の海底標高を維持し，陸の元標高は海陸分類と費用の入力に限る
	 * 区分線形基準面の構成用であり，C1・侵食・写実的な最終地形は別途必要
	 */
	class OceanDistanceTerrain {
	private:

		std::vector<double> HeightsMeters;
		std::vector<std::uint32_t> DownstreamVertices;
		OceanDistanceTerrainStatistics Stats;

		OceanDistanceTerrain() = default;

	public:

		static std::optional<OceanDistanceTerrain> Create(
			std::span<const UnitDirection> Directions,
			std::span<const std::array<std::uint32_t, 3>> Triangles,
			std::span<const double> OriginalHeightsMeters,
			std::span<const double> TravelCosts,
			const OceanDistanceTerrainSettings &Settings,
			OceanDistanceTerrainError &Error,
			std::stop_token Stop={}
		);

		std::span<const double> GetHeightsMeters() const;
		std::span<const std::uint32_t> GetDownstreamVertices() const;
		const OceanDistanceTerrainStatistics &GetStatistics() const;
	};
}
