#pragma once

#include "PlanetFlowChannelField.h"
#include "PlanetGround.h"
#include "PlanetSurfaceMaterial.h"
#include "PlanetView.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	enum class FlowViewFailure : std::uint8_t {
		None,
		InvalidView,
		Sample,
		Selection,
		Resolution,
		Topology,
		FloatPrecision,
		Interpolation,
		Cancelled
	};

	struct FlowViewDiagnostics {
		// 有効な候補を作れない理由と，今回の実費用
		FlowViewFailure Failure = FlowViewFailure::None;
		std::size_t Leaves = 0;
		std::uint64_t HeightEvaluations = 0;
		std::uint64_t ProbeCacheHits = 0;
		std::uint64_t ValidationSamples = 0;
		std::uint64_t ValidationEvaluations = 0;
		std::uint64_t ValidationReuses = 0;
		std::uint64_t VertexSampleEvaluations = 0;
		std::uint64_t VertexSampleReuses = 0;
		double MaximumSelectionErrorMeters = 0;
		double MaximumFloatErrorMeters = 0;
		double MaximumSurfaceErrorMeters = 0;
	};

	struct FlowTerrainMesh {
		// 完全な球面分割と，同じ場から得た局所座標・解析法線
		std::shared_ptr<const FlowSurfaceField> SourceField;
		LodTree Tree;
		GroundFrame Frame;
		AdaptiveSphereMesh Sphere;
		std::vector<double> Heights;
		std::vector<Vec3> Positions;
		std::vector<Vec3> Normals;
		std::vector<Vec3> PlanetNormals;
		std::vector<SurfaceMaterialWeights> Materials;
		// 被覆へ飽和する前の水深と符号付き支持距離，単位 m
		std::vector<std::array<double, 2>> WaterCoordinates;
		Vec3 TextureOrigin;
	};

	std::optional<FlowTerrainMesh> BuildFlowTerrainPartition(
		std::shared_ptr<const FlowSurfaceField> Field,
		const LodView &View,
		AdaptiveSphereMesh Sphere,
		LodTree Tree,
		const std::stop_token Stop,
		FlowViewDiagnostics &Report,
		const FlowTerrainMesh *const Previous=nullptr
	);

	/** 一つの不変場と有界な標高再利用を，一件の生成ワーカーだけが所有する */
	class FlowTerrainView {
	private:

		// 異なる生成定義の標高を混ぜない所有権
		std::shared_ptr<const FlowSurfaceField> Field;
		std::unique_ptr<LodSurfaceCache> Cache;

		explicit FlowTerrainView(std::shared_ptr<const FlowSurfaceField> Input);

	public:

		static std::unique_ptr<FlowTerrainView> Create(std::shared_ptr<const FlowSurfaceField> Field);
		bool Uses(const std::shared_ptr<const FlowSurfaceField> &Candidate) const;

		std::optional<FlowTerrainMesh> Build(
			const LodView &View,
			const std::stop_token Stop= {},
			FlowViewDiagnostics *const Diagnostics=nullptr
		);
	};
}
