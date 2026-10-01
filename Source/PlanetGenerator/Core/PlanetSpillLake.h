#pragma once

#include "PlanetCriticalPoint.h"
#include "PlanetTerrainContour.h"
#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace PlanetCore {
	struct SpillLakeDiagnostics;

	/** 流出口の片側を囲む元地形の不変な湖岸；大洋迄の水系接続は別の契約 */
	class SpillLakeShore {
	private:

		friend std::optional<SpillLakeShore> PrepareSpillLakeShore(
			const TerrainCriticalPoint &Pass,
			bool IsPositiveLakeBranch,
			const std::function<TerrainSample(UnitDirection)> &Source,
			const TerrainContourSettings &Settings,
			std::stop_token Stop,
			SpillLakeDiagnostics *Diagnostics
		);

		struct IndexNode {
			// 辺群の閉矩形と，葉又は子への対応
			std::array<double, 2> Min;
			std::array<double, 2> Max;
			std::uint32_t Begin = 0;
			std::uint32_t Count = 0;
			std::uint32_t Left = 0;
			std::uint32_t Right = 0;
		};

		// 同じ球面と接平面の岸，構築後に変更しない索引
		TerrainContour Contour;
		Vec3 Origin;
		Vec3 East;
		Vec3 North;
		double RadiusMeters = 0;
		std::vector<std::array<double, 2>> PlanarVertices;
		std::vector<std::uint32_t> Order;
		std::vector<IndexNode> Index;
		std::uint32_t BuildIndex(std::uint32_t Begin, std::uint32_t Count);

		SpillLakeShore() = default;

	public:

		const TerrainContour &GetContour() const;
		std::span<const std::array<double, 2>> GetPlanarVertices() const;
		double SignedPlanarDistanceMeters(UnitDirection Direction, std::uint32_t *TestedEdges=nullptr) const;
		bool Contains(UnitDirection Direction) const;
	};

	enum class SpillLakeFailure : std::uint8_t {
		None,
		InvalidInput,
		Direction,
		Contour,
		StartGap,
		Projection,
		Boundary,
		Cancelled
	};

	struct SpillLakeDiagnostics {
		// 最後の追跡結果と拒否した段階
		SpillLakeFailure Failure = SpillLakeFailure::None;
		TerrainContourEnd End = TerrainContourEnd::StepLimit;
		std::uint32_t Vertices = 0;
		double LengthMeters = 0;
		double HeightResidualMeters = 0;
		double FirstGapMeters = 0;
		double AreaMetersSquared = 0;
		Vec3 LastDirection;
	};

	std::optional<TerrainContour> TraceSpillLakeArc(
		const TerrainCriticalPoint &Pass,
		bool IsPositiveLakeBranch,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainContourSettings &Settings,
		std::stop_token Stop= {},
		SpillLakeDiagnostics *Diagnostics=nullptr
	);

	std::optional<SpillLakeShore> PrepareSpillLakeShore(
		const TerrainCriticalPoint &Pass,
		bool IsPositiveLakeBranch,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainContourSettings &Settings,
		std::stop_token Stop= {},
		SpillLakeDiagnostics *Diagnostics=nullptr
	);
}
