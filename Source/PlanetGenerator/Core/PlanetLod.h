#pragma once

#include "PlanetCore.h"
#include <cstddef>
#include <set>
#include <memory>
#include <stop_token>

namespace PlanetCore {
	struct TileOrder {
		bool operator()(TileId A, TileId B) const noexcept;
	};

	enum class LodEditStatus {
		Applied,
		NotLeaf,
		DepthLimit,
		BudgetExceeded,
		InvalidBudget,
		ChildrenNotLeaves,
		NeighborTooFine
	};
	struct LodEdit {
		LodEditStatus Status = LodEditStatus::NotLeaf;
		// Parents replaced by four children. Empty on any rejected edit.
		std::vector<TileId> SplitParents;
	};

	// Pure same-level adjacency, independent of the field and view. One worker
	// owns the cache; tree copies share it. Never serialize this derived geometry.
	class LodNeighborCache {
	public:
		static constexpr std::size_t MaximumTiles = 65536;
		explicit LodNeighborCache(std::size_t Capacity = MaximumTiles);
		~LodNeighborCache();
		std::optional<Neighbor> Find(TileId Tile, std::uint8_t Edge);
		std::size_t TileCount() const;
		std::uint64_t Hits() const;

	private:
		struct Impl;
		std::unique_ptr<Impl> State;
	};

	// Always a complete partition of all 20 roots, balanced across every edge.
	// Build/edit on a worker-owned copy; publish a const snapshot only when its
	// mesh and boundary data are ready. This class does not publish GPU resources.
	class LodTree {
	public:
		static constexpr std::size_t MaximumLeaves = 262144;
		explicit LodTree(std::shared_ptr<LodNeighborCache> Neighbors = {});
		const std::set<TileId, TileOrder> &Leaves() const {
			return LeafTiles;
		}
		bool IsLeaf(TileId Tile) const;
		std::optional<TileId> CoveringLeaf(TileId Tile) const;
		bool HasFinerNeighbor(TileId Tile, std::uint8_t Edge) const;
		/** 別ワーカーへ渡す前に隣接キャッシュだけを新規にし，葉を変えない */
		void DetachNeighborCache();
		// Includes every balancing dependency in the same transaction. A budget
		// failure leaves the old complete partition untouched.
		LodEdit Refine(TileId Tile, std::size_t LeafBudget);
		LodEditStatus Coarsen(TileId Parent);

	private:
		std::set<TileId, TileOrder> LeafTiles;
		std::shared_ptr<LodNeighborCache> Neighbors;
		std::optional<Neighbor> AdjacentTile(TileId Tile, std::uint8_t Edge) const;
	};

	// Indices 0..2 are corners; 3..5 are midpoints opposite each corner.
	// Shared by topology emission and height-error selection.
	struct StitchPattern {
		std::array<std::array<std::uint8_t, 3>, 4> Triangles{};
		std::uint8_t Count = 0;
	};
	std::optional<StitchPattern> GetStitchPattern(std::uint8_t Mask);

	/** 接合面を次の親にせず，元の所有葉を細分化する為の対応 */
	struct SphereRefinementHierarchy {
		LodTree Tree;
		std::vector<TileId> TriangleOwners;
		std::array<std::size_t, 8> StitchMaskCounts{};
	};

	struct AdaptiveSphereMesh {
		SphereMesh Mesh;
		std::vector<TileId> TriangleOwners;
		std::array<std::size_t, 8> StitchMaskCounts{};
	};

	// Coarse leaves add the canonical midpoint where a finer neighbor needs it.
	// The eight red/green templates therefore share actual H(d) vertices, not a
	// skirt or a second overlapping surface. Every output vertex is evaluated once.
	// Temporal morphing and per-tile GPU publication are separate responsibilities.
	std::optional<AdaptiveSphereMesh> BuildAdaptiveSphere(const LodTree &Tree);
	std::optional<AdaptiveSphereMesh> UpdateAdaptiveSphere(
		const LodTree &Tree, const AdaptiveSphereMesh &Previous, std::stop_token Stop = {});
}
