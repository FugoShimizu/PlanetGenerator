#pragma once
#include "PlanetView.h"
#include "PlanetGround.h"
namespace PlanetCore {
	struct OceanSurfaceMesh {
		SphereMesh Sphere;
		std::optional<SphereRefinementHierarchy> Hierarchy;
		GroundFrame Frame;
		std::vector<Vec3> Positions;
		std::size_t LeafCount = 0, CachedTiles = 0;
		std::uint64_t CachedProbeHits = 0;
		std::uint32_t RefinementPasses = 0, RefinedTerrainParents = 0;
		double NearRadiusMeters = 0, MaximumFloatErrorMeters = 0, MaximumProjectionErrorPixels = 0;
		bool ReusedTopology = false, ReusedPreviousTopology = false;
	};
	struct OceanReuseDiagnostics;
	class OceanSurfaceCache {
	public:
		OceanSurfaceCache(double Radius, double SeaLevel);
		bool Uses(double Radius, double SeaLevel) const;

	private:
		double Radius, SeaLevel;
		LodSurfaceCache Tiles;
		// One complete validated partition; single-worker ownership, never serialized.
		std::optional<LodTree> TopologyTree;
		std::optional<AdaptiveSphereMesh> TopologyMesh;
		friend std::optional<OceanSurfaceMesh> BuildOceanSurfaceFromTerrain(const LodView &, double, const LodTree &,
			const AdaptiveSphereMesh &, const std::optional<GroundFrame> &, std::stop_token, OceanReuseDiagnostics *,
			OceanSurfaceCache *);
		friend std::optional<OceanSurfaceMesh> BuildOceanSurface(
			const LodView &, double, const std::optional<GroundFrame> &, std::stop_token, OceanSurfaceCache *);
	};
	// Minimum unit-sphere chord distance to a minor-arc spherical triangle.
	// Relative differences retain precision for metre-sized triangles on huge spheres.
	double SphericalTriangleChordDistance(UnitDirection Focus, const std::array<UnitDirection, 3> &Triangle);
	struct OceanReuseDiagnostics {
		double NearEdgeMeters = 0, NearErrorMeters = 0, FloatErrorMeters = 0, ProjectionErrorPixels = 0;
		bool RejectedNear = false, RejectedProjection = false;
		std::size_t RejectedNearTriangles = 0, RejectedProjectionTriangles = 0;
	};
	// Candidate must come from BuildAdaptiveSphere (complete, balanced sphere).
	// Reuse only after checking its near edges, float positions and a conservative
	// sphere chord-error metric against one pixel; otherwise use independent LOD.
	std::optional<OceanSurfaceMesh> ReuseOceanSurface(const LodView &View, double SeaLevel, const SphereMesh &Candidate,
		const std::optional<GroundFrame> &PreviousFrame = {}, std::stop_token Stop = {},
		OceanReuseDiagnostics *Diagnostics = nullptr);
	// Reuse a matching terrain tree/mesh, refining only failed owners on a private
	// tree copy. At most four passes; budget/depth/cancellation failures return no
	// candidate so the caller can use independent sea LOD or retain its old surface.
	// A cache first tries its last complete partition under the NEW view/precision
	// constraints, then the terrain partition. Only successful bounded results replace it.
	std::optional<OceanSurfaceMesh> BuildOceanSurfaceFromTerrain(const LodView &View, double SeaLevel,
		const LodTree &TerrainTree, const AdaptiveSphereMesh &Candidate, const std::optional<GroundFrame> &PreviousFrame = {},
		std::stop_token Stop = {}, OceanReuseDiagnostics *InitialDiagnostics = nullptr, OceanSurfaceCache *Cache = nullptr);
	/**
	 * 固定骨格を含む分割を，通常の視点選択の葉数上限とは分けて投影・細分化する
	 * @param View
	 * 有効な視点と精度条件，視点の上限は緩和しない
	 * @param SeaLevel 一定の海面標高
	 * @param SkeletonTree 最大
	 * LodTree::MaximumLeaves の整合した分割
	 * @param Candidate 同じ木から作った球面三角形
	 * @param PreviousFrame
	 * 前回の局所原点
	 * @param Stop 中断要求
	 * @param InitialDiagnostics 初回の精度不足の記録
	 * @return
	 * 精度と全球分割の上限を満たした候補，失敗時は空
	 */
	std::optional<OceanSurfaceMesh> BuildOceanSurfaceFromSkeleton(const LodView &View, double SeaLevel,
		const LodTree &SkeletonTree, const AdaptiveSphereMesh &Candidate, const std::optional<GroundFrame> &PreviousFrame = {},
		std::stop_token Stop = {}, OceanReuseDiagnostics *InitialDiagnostics = nullptr);

	// Independent constant-height LOD: deep seabed relief does not force the
	// visible sea to use a coarse planetary-coordinate mesh or refine the bottom.
	std::optional<OceanSurfaceMesh> BuildOceanSurface(const LodView &View, double SeaLevel,
		const std::optional<GroundFrame> &PreviousFrame = {}, std::stop_token Stop = {}, OceanSurfaceCache *Cache = nullptr);
}
