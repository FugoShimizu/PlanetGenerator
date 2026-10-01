#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct SphericalReliefSettings {
		double RadiusMeters = 1000000, SeaLevelMeters = 0, MinimumSlope = 1E-7, TargetErrorMeters = 25;
		std::uint32_t MaximumAddedVertices = 4096;
		std::uint8_t MaximumDepth = 4;
		bool ShouldRefineEdges = false, ShouldPreserveVertexSlopes = false, ShouldLimitToDrainageEdges = false,
		ShouldRelaxAddedHeights = false;

		bool IsValid() const;
	};

	struct SphericalReliefStatistics {
		std::uint32_t OriginalVertices = 0, AddedVertices = 0, PreservedSlopeVertices = 0, ProtectedAddedVertices = 0,
		RelaxedVertices = 0;
		std::uint64_t SourceEvaluations = 0;
		double MaximumConditioningRaiseMeters = 0, MaximumConditioningCutMeters = 0, MaximumRelaxationMeters = 0;
		Vec3 MaximumRaiseDirection, MaximumCutDirection;
		double LargestRemainingPriorityMeters = 0, MinimumCreatedTriangleQuality = 1;
	};

	/** 排水をまだ解かず，元標高を改変せずに準備した共有網 */
	struct SphericalReliefSamplingMesh {
		std::vector<UnitDirection> Directions;
		std::vector<std::array<std::uint32_t, 3>> Triangles;
		std::vector<double> ElevationsMeters;
		SphericalReliefStatistics Statistics;
	};
	/** 陸上起伏の適応細分化と，共有網に保持する下降経路 */
	class PlanetSphericalRefinement {
	private:

		struct EdgeWorkspace;

		struct Face {
			std::array<std::uint32_t, 3> Vertices {};
			std::uint8_t Depth = 0;
			bool IsActive = true;
		};

		struct Candidate {
			Vec3 Direction;
			double ElevationMeters = 0, SourceMeters = 0, ErrorMeters = 0, TriangleQuality = 1;
			std::uint32_t FaceIndex = 0, DownstreamVertex = 0;
		};

		static constexpr double MinimumChildQuality = 0.25;

		std::vector<UnitDirection> Directions;
		std::vector<std::array<std::uint32_t, 3>> Triangles;
		std::vector<double> Heights;
		std::vector<std::uint32_t> Downstream;
		SphericalReliefStatistics Stats;

		std::optional<std::vector<Vec3>> ConstrainGradientsImpl(
			const std::span<const Vec3> Gradients,
			const double SeaLevelMeters,
			const std::span<const std::uint32_t> RequiredDownstream,
			const std::stop_token Stop
		) const;

		std::optional<Candidate> SelectCandidate(
			const Face &Triangle,
			const std::uint32_t FaceIndex,
			const SphericalReliefSettings &Settings,
			const std::function<double(UnitDirection)> &Source
		);

		PlanetSphericalRefinement() = default;

	public:

		static std::optional<PlanetSphericalRefinement> Create(
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::span<const double> ElevationsMeters,
			const std::function<double(UnitDirection)> &Source,
			const SphericalReliefSettings &Settings,
			const std::stop_token Stop={}
		);

		static std::optional<SphericalReliefSamplingMesh> CreateSamplingMesh(
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::span<const double> ElevationsMeters,
			const std::function<double(UnitDirection)> &Source,
			const SphericalReliefSettings &Settings,
			const std::stop_token Stop={}
		);

		static std::optional<PlanetSphericalRefinement> CreateWithRefinedEdges(
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::span<const double> ElevationsMeters,
			const std::span<const std::uint32_t> RequiredDownstream,
			const std::function<double(UnitDirection)> &Source,
			const SphericalReliefSettings &Settings,
			const std::stop_token Stop={}
		);

		std::optional<std::vector<Vec3>> ConstrainGradients(
			const std::span<const Vec3> Gradients,
			const double SeaLevelMeters,
			const std::stop_token Stop={}
		) const;

		std::optional<std::vector<Vec3>> ConstrainDrainageGradients(
			const std::span<const Vec3> Gradients,
			const double SeaLevelMeters,
			const std::span<const std::uint32_t> RequiredDownstream,
			const std::stop_token Stop={}
		) const;

		std::span<const UnitDirection> GetDirections() const;
		std::span<const std::array<std::uint32_t, 3>> GetTriangles() const;
		std::span<const double> GetElevations() const;
		std::span<const std::uint32_t> GetDownstreamVertices() const;
		const SphericalReliefStatistics &GetStatistics() const;
	};
}
