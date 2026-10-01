#pragma once

#include "PlanetCore.h"
#include <cstddef>
#include <functional>
#include <stop_token>

namespace PlanetCore {
	// 半径に依らない地表の加速度，単位は m/s²
	inline constexpr double SurfaceGravityMetersPerSecondSquared = 9.80665;
	std::optional<Vec3> SurfaceGravity(Vec3 PositionFromCenterMeters);

	struct GroundFrame {
		Vec3 Up;
		Vec3 East;
		Vec3 North;
		double RadiusMeters = 0;
		double AnchorElevationMeters = 0;
		static std::optional<GroundFrame> Create(UnitDirection Anchor, double Radius, double Elevation);
		UnitDirection DirectionAt(double EastMeters, double NorthMeters) const;
		Vec3 SurfaceToLocal(UnitDirection Direction, double Elevation) const;
		Vec3 VectorToLocal(Vec3 Vector) const;
		Vec3 LocalToPlanet(Vec3 Position) const;
	};

	struct GroundPatchSettings {
		double HalfExtentMeters = 16;
		std::uint32_t CellsPerSide = 32;
		double MaximumProbeErrorMeters = 0.02;
		// 同じ候補内だけの共有辺再利用であり，形状又は誤差基準を変えない
		bool ReuseSharedEdgeProbes = true;
		bool IsValid(double Radius) const;
	};

	struct GroundPatch {
		GroundFrame Frame;
		std::vector<Vec3> Positions;
		std::vector<Vec3> Normals;
		std::vector<std::array<std::uint32_t, 3>> Triangles;
		double MaximumProbeErrorMeters = 0;
		std::uint64_t Evaluations = 0;
		std::uint64_t SharedEdgeProbeHits = 0;
	};

	enum class GroundPatchFailure : std::uint8_t {
		None, InvalidInput, InvalidFrame, InvalidSample, InvalidNormal, InvertedTriangle,
		InvalidProbe, RefinementLimit, VertexLimit, TriangleLimit, Cancelled
	};

	/** 拒否理由と最後の反復を保持し，精度と資源制限を区別する */
	struct GroundPatchDiagnostics {
		GroundPatchFailure Failure = GroundPatchFailure::None;
		unsigned RefinementPass = 0;
		std::size_t Vertices = 0;
		std::size_t Triangles = 0;
		double MaximumErrorMeters = 0;
		std::array<Vec3, 3> WorstTriangle {};
		std::array<double, 7> WorstProbeErrors {};
	};

	using GroundEvaluator = std::function<TerrainSample(UnitDirection)>;
	using GroundHeightEvaluator = std::function<double(UnitDirection)>;
	// 有限な内部検査は全点の誤差上界の証明ではない
	// 検査に失敗した候補又は中断した候補は公開しない
	std::optional<GroundPatch> BuildGroundPatch(
		UnitDirection Anchor,
		double Radius,
		const GroundPatchSettings &Settings,
		const GroundEvaluator &Evaluate,
		std::stop_token Stop={},
		const GroundHeightEvaluator &EvaluateHeight={},
		GroundPatchDiagnostics *Diagnostics=nullptr
	);
}
