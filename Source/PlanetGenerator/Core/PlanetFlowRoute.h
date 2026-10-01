#pragma once

#include "PlanetCore.h"
#include "PlanetFlowCurve.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace PlanetCore {
	struct FlowRouteKnot {
		UnitDirection Direction;
		Vec3 Tangent;
		double DistanceMeters = 0;
	};

	struct FlowRouteProjection {
		TerrainFlowFootpoint Footpoint;
		double DistanceMeters = 0;
		std::uint32_t SegmentIndex = 0;
	};

	enum class FlowRouteFailure : std::uint8_t { None, InvalidInput, Distance, Tangent, Segment };

	struct FlowRouteDiagnostics {
		// 制約を緩めず，拒否した元区間と其の実寸法を特定
		FlowRouteFailure Failure = FlowRouteFailure::None;
		std::uint32_t Index = 0;
		double DistanceDifferenceMeters = 0;
		double ChordMeters = 0;
		std::array<double, 2> ForwardProducts {};
		std::array<double, 2> CurvatureProducts {};
		std::array<Vec3, 2> Directions;
	};

	/** 共有曲率の区間群と保守的な空間索引を持つ不変な経路 */
	class TerrainFlowRoute {
	private:

		struct IndexNode {
			Vec3 Min;
			Vec3 Max;
			std::uint32_t Begin = 0;
			std::uint32_t Count = 0;
			std::uint32_t Left = 0;
			std::uint32_t Right = 0;
		};

		std::vector<TerrainFlowSegment> Segments;
		std::vector<double> Starts;
		std::vector<std::uint32_t> Order;
		std::vector<IndexNode> Index;
		double RadiusMeters = 0;
		double LengthMeters = 0;

		TerrainFlowRoute() = default;

		std::uint32_t BuildIndex(const std::uint32_t Begin, const std::uint32_t Count);

	public:

		static std::optional<TerrainFlowRoute> Create(
			const std::span<const FlowRouteKnot> Knots,
			const double Radius,
			FlowRouteDiagnostics *const Diagnostics=nullptr
		);

		std::optional<TerrainFlowCurveSample> Sample(const double DistanceMeters) const;
		double GetLengthMeters() const;
		std::span<const TerrainFlowSegment> GetSegments() const;

		std::optional<FlowRouteProjection> FindFootpoint(
			const UnitDirection Query,
			const double MaximumDistanceMeters,
			std::uint32_t *const Candidates=nullptr,
			const bool IncludeEndpoints=false
		) const;
	};
}
