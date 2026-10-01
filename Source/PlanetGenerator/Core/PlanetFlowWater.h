#pragma once

#include "PlanetFlowView.h"
#include "PlanetWaterCoverage.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct FlowWaterMesh {
		// 河床と同じ座標基準に置く水面；海面は別の一定球面
		GroundFrame Frame;
		WaterCoverageMesh Surface;
		std::vector<Vec3> Normals;
		std::vector<Vec3> PlanetNormals;
		Vec3 TextureOrigin;
	};

	struct FlowWaterReuseStatistics {
		// 同じ不変場に対する再利用の累積回数と現在の保持量
		std::uint64_t BankProbeHits = 0;
		std::uint64_t BankProbeEvaluations = 0;
		std::size_t BankSamples = 0;
		std::size_t BankChecks = 0;
		std::uint64_t PartitionReuses = 0;
		std::size_t RetainedLeaves = 0;
		std::size_t WaterSamples = 0;
		std::uint64_t WaterSampleHits = 0;
		std::uint64_t WaterSampleEvaluations = 0;
		std::uint64_t WaterSupportEvaluations = 0;
		// 累積費用：水位照会・河道発見・河岸残差・水面法線・共通地面，単位はミリ秒
		std::array<double, 5> StageMilliseconds {};
		// 保持上限に依る河岸残差の追出し回数
		std::uint64_t BankCheckEvictions = 0;
	};

	/** 同じ不変場の河道・湖岸を索引化し，唯一の生成ワーカーから視点毎に描画を準備する */
	class FlowWaterView {
	private:

		struct Guide {
			// 発見用の短い大円弧と保守的な包囲球
			UnitDirection Start;
			UnitDirection End;
			Vec3 Center;
			double Cap = 0;
			double ElevationMeters = 0;
			bool IsRiver = false;
		};

		struct IndexNode {
			// 包囲箱と葉範囲又は二つの子
			Vec3 Low;
			Vec3 High;
			std::uint32_t Begin = 0;
			std::uint32_t Count = 0;
			std::uint32_t Left = 0;
			std::uint32_t Right = 0;
		};

		// 視点から独立した河岸の標本と補間誤差を，有界な RAM にだけ保持
		struct ReuseState;
		std::unique_ptr<ReuseState> Reuse;
		// 場の寿命を保持し，生成後は変更しない発見用索引
		std::shared_ptr<const FlowSurfaceField> Field;
		std::vector<Guide> Guides;
		std::vector<std::uint32_t> Order;
		std::vector<IndexNode> Index;
		double HalfWidthMeters = 0;
		double BankSupportChord = 0;
		double MaximumGuideErrorMeters = 0;

		explicit FlowWaterView(std::shared_ptr<const FlowSurfaceField> Input);

		void AddGuide(const UnitDirection Start, const UnitDirection End, const double ElevationMeters, const bool IsRiver);
		std::uint32_t BuildIndex(const std::uint32_t Begin, const std::uint32_t Count);

		double RequiredEdge(
			const UnitDirection Center,
			const double Cap,
			const std::array<UnitDirection, 3> &Triangle,
			const std::array<double, 3> &Depths,
			const LodView &View,
			bool &TouchesBank
		) const;

	public:

		~FlowWaterView();

		FlowWaterReuseStatistics GetReuseStatistics() const;

		static std::unique_ptr<FlowWaterView> Create(
			std::shared_ptr<const FlowSurfaceField> Field,
			const std::stop_token Stop= {}
		);

		std::optional<FlowWaterMesh> Build(
			FlowTerrainMesh &Terrain,
			const LodView &View,
			WaterRefinementDiagnostics &Diagnostics,
			const std::stop_token Stop= {},
			FlowViewDiagnostics *const GroundDiagnostics=nullptr
		);

		std::size_t GuideCount() const;
		double GetMaximumGuideErrorMeters() const;
	};
}
