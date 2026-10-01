#pragma once
#include "PlanetRiverTerrain.h"
#include "PlanetGround.h"
#include <memory>
namespace PlanetCore {
	struct EvaluationCacheStats {
		std::uint64_t Samples = 0, SampleHits = 0, Heights = 0, HeightHits = 0, WaterSamples = 0, WaterHits = 0;
	};
	// Requires a non-null immutable field and one worker. Exact direction bits, never rounded
	// spatial keys. Bounded RAM only; eviction cannot change any terrain value.
	class TerrainEvaluationCache {
	public:
		static constexpr std::size_t DefaultSamples = 65536, MaximumSamples = 262144, MaximumHeights = 131072;
		explicit TerrainEvaluationCache(std::shared_ptr<const RiverTerrainField> Field,
			std::size_t SampleCapacity = DefaultSamples, std::size_t HeightCapacity = MaximumHeights);
		~TerrainEvaluationCache();
		TerrainEvaluationCache(const TerrainEvaluationCache &) = delete;
		TerrainEvaluationCache &operator=(const TerrainEvaluationCache &) = delete;
		TerrainSample Evaluate(UnitDirection Direction);
		RiverSurfaceSample EvaluateSurface(UnitDirection Direction);
		RiverWaterHeights EvaluateWater(UnitDirection Direction);
		double EvaluateElevation(UnitDirection Direction);
		EvaluationCacheStats Statistics() const;
		std::size_t SampleCount() const;
		std::size_t SampleCapacity() const;
		/** 既存の値を保って保持枠を拡張する．唯一のワーカーから呼び，上限を超える要求は上限へ制限する */
		void GrowSamples(std::size_t Capacity);
		std::size_t HeightCount() const;
		/** 水面用は必要時だけ枠を確保し，地表の法線用標本を追出さない */
		void GrowWaterSamples(std::size_t Capacity);
		std::size_t WaterSampleCount() const;
		std::size_t WaterSampleCapacity() const;

	private:
		struct Impl;
		std::unique_ptr<Impl> State;
	};
	// The numerical origin may stay still while the mandatory disk follows the
	// body. Rebase before its surface focus moves 128 m in local coordinates.
	// Actual near-vertex float error must still be checked by the renderer.
	std::optional<GroundFrame> SelectRenderFrame(
		UnitDirection Anchor, double Radius, double Elevation, const std::optional<GroundFrame> &Previous = {});
}
