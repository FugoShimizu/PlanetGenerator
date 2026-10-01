#pragma once

#include "PlanetLod.h"
#include "PlanetSurfaceCorridor.h"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	struct LodView {
		static constexpr std::size_t MaximumViewLeaves = 32768;
		Vec3 ObserverMeters { 0, 0, 1100000 };
		double RadiusMeters = 1000000;
		double ReliefBoundMeters = 40000;
		double FocalLengthPixels = 540;
		double TargetEdgePixels = 32;
		std::size_t LeafBudget = 2048;
		// 辺長に依る描画密度の追加は此の総葉数で停止
		// 足元・植生・標高の投影誤差は LeafBudget 迄を使用
		std::size_t PixelLeafBudget = 8192;
		std::uint8_t MaximumLevel = 32;
		// 身体の精細中心は描画視点と分離可能；省略時は描画視点
		std::optional<Vec3> SurfaceObserverMeters;
		// 進行方向の短い通路が有る場合だけ終点を指定
		std::optional<Vec3> SurfacePathEndMeters;
		// 身体の精細中心から参照球面上で測る距離；半径０で必須領域を無効化
		double SurfaceRadiusMeters = 0;
		double SurfaceEdgeMeters = 2;
		double SurfaceErrorMeters = 0.01;
		// 同じ地面の広い領域に植生の接地精度を予約；半径０で無効化
		double FoliageRadiusMeters = 0;
		double FoliageEdgeMeters = 8;
		double FoliageErrorMeters = 0.02;
		// 粗い候補の照会を遅延；最終葉の全標本検査は維持
		// 偽は全候補を照会する比較用の基準経路
		bool DeferCoarseSurfaceProbes = true;
		bool IsValid() const;
	};

	struct LodSelection {
		LodTree Tree;
		std::uint32_t RefinedParents = 0;
		std::uint8_t DeepestLevel = 0;
		double MaximumRemainingEdgePixels = 0;
		double MaximumSurfaceEdgeMeters = 0;
		double MaximumSurfaceErrorMeters = 0;
		double MaximumStitchErrorMeters = 0;
		double MaximumFoliageEdgeMeters = 0;
		double MaximumFoliageErrorMeters = 0;
		double MaximumFoliageStitchErrorMeters = 0;
		std::uint32_t StitchRefinementPasses = 0;
		std::uint32_t StitchRefinedParents = 0;
		std::uint32_t ReclaimedDistantParents = 0;
		std::uint64_t StitchProbeCacheHits = 0;
		std::uint64_t SurfaceHeightEvaluations = 0;
		std::uint64_t SurfaceProbeCacheHits = 0;
		std::size_t CachedTiles = 0;
		std::size_t CachedNeighborTiles = 0;
		std::uint64_t NeighborCacheHits = 0;
		bool SurfaceResolutionSatisfied = true;
		bool FoliageResolutionSatisfied = true;
	};

	// 不変な H の有限標本で補間残差を調べる；連続領域の誤差上界ではない
	// 視点選択は標高を変えず，全球で同じ位相を維持
	using SurfaceHeightEvaluator = std::function<double(UnitDirection)>;

	class LodSurfaceCache;

	std::optional<LodSelection> SelectLodForView(
		const LodView &View,
		std::stop_token Stop={},
		const SurfaceHeightEvaluator &Height={},
		LodSurfaceCache *Cache=nullptr
	);

	// 同じ不変場と半径に対し，単一ワーカーからだけ使う有界な RAM
	// 場が変わる場合は再作成し，評価関数が元の場の寿命を保持
	class LodSurfaceCache {
	private:

		friend class LodSurfacePreparation;

		friend std::optional<LodSelection> SelectLodForView(
			const LodView &,
			std::stop_token,
			const SurfaceHeightEvaluator &,
			LodSurfaceCache *
		);

		struct Impl;
		std::unique_ptr<Impl> State;

	public:

		static constexpr std::size_t MaximumTiles = 65536;
		static constexpr std::size_t MaximumStitchVariants = 16384;
		std::size_t TileCount() const;

		LodSurfaceCache &operator=(const LodSurfaceCache &) = delete;

		LodSurfaceCache(double Radius, SurfaceHeightEvaluator Height, std::size_t Capacity=MaximumTiles);

		LodSurfaceCache(const LodSurfaceCache &) = delete;

		~LodSurfaceCache();
	};

	struct SurfacePreparationProgress {
		std::uint64_t VisitedTiles = 0;
		std::uint64_t HeightEvaluations = 0;
		std::uint64_t ProbeHits = 0;
		bool Complete = false;
		bool Valid = true;
		bool BudgetExhausted = false;
	};

	// 不変な標本の再開可能な準備；分割や描画面，精度の認証は公開しない
	// 選択時は最終葉と接合を別途検査し，同じ単一ワーカーの保持器を使用
	class LodSurfacePreparation {
	private:

		struct Impl;
		std::unique_ptr<Impl> State;

		explicit LodSurfacePreparation(std::unique_ptr<Impl> State);

	public:

		static std::shared_ptr<LodSurfacePreparation> Create(const LodView &View, std::shared_ptr<LodSurfaceCache> Cache);
		const LodView &TargetView() const;
		bool Uses(const std::shared_ptr<LodSurfaceCache> &Cache) const;
		SurfacePreparationProgress Advance(std::size_t TileBudget, std::stop_token Stop={});

		~LodSurfacePreparation();
	};
}
