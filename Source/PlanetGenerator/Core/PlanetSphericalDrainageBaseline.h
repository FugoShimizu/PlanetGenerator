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
	inline constexpr std::uint32_t SphericalDrainageBaselineVersion = 1;
	/** 既存の共有球面網が検証出来る最大頂点数 */
	inline constexpr std::size_t SphericalDrainageBaselineMaximumVertices = 167938;

	enum class SphericalDrainageBaselineError : std::uint8_t {
		None,
		Cancelled,
		InvalidInput,
		InvalidMesh,
		NoOcean,
		NoLand,
		DisconnectedOcean,
		LevelLandEdge,
		LandSink
	};

	struct SphericalDrainageBaselineStatistics {
		std::uint32_t Vertices = 0;
		std::uint32_t Triangles = 0;
		std::uint32_t Edges = 0;
		std::uint32_t OceanVertices = 0;
		std::uint32_t NonOceanVertices = 0;
	};

	/**
	 * 閉球面の区分線形標高を，連結した大洋と全陸上点の下降経路で拘束する基準
	 * 全陸頂点が低い隣接点を持ち，同高度の乾いた辺を除けば，
	 * 面内から最低頂点へ向かう重みの線形経路も厳密に下る
	 * 辺の法線は連続でなく，完成した地表面や河川水位には使用しない
	 */
	class SphericalDrainageBaseline {
	private:

		std::vector<std::array<std::uint32_t, 3>> Triangles;
		std::vector<double> ElevationsMeters;
		std::vector<std::uint32_t> DownstreamVertices;
		SphericalDrainageBaselineStatistics Stats;
		double SeaLevelMeters = 0;

		SphericalDrainageBaseline() = default;

	public:

		static std::optional<SphericalDrainageBaseline> Create(
			std::span<const UnitDirection> Directions,
			std::span<const std::array<std::uint32_t, 3>> Triangles,
			std::span<const double> ElevationsMeters,
			double SeaLevelMeters,
			SphericalDrainageBaselineError &Error,
			std::stop_token Stop={}
		);

		std::optional<double> EvaluateFace(std::size_t Face, std::array<double, 3> Weights) const;
		std::span<const std::uint32_t> GetDownstreamVertices() const;
		const SphericalDrainageBaselineStatistics &GetStatistics() const;
	};
}
