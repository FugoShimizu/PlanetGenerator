#include "PlanetLod.h"

#include <map>
#include <list>
#include <algorithm>
#include <tuple>

namespace PlanetCore {
	bool TileOrder::operator()(TileId A, TileId B) const noexcept {
		return std::tie(A.RootFace, A.Level, A.Path) < std::tie(B.RootFace, B.Level, B.Path);
	}

	struct LodNeighborCache::Impl {
		struct Entry {
			std::array<std::optional<Neighbor>, 3> Adjacent;
			std::list<TileId>::iterator Recent;
		};
		std::size_t Capacity;
		std::uint64_t Hits = 0;
		std::map<TileId, Entry, TileOrder> Tiles;
		std::list<TileId> Recent;
		explicit Impl(std::size_t C) : Capacity(std::clamp(C, std::size_t{1}, MaximumTiles)) {
		}
	};
	LodNeighborCache::LodNeighborCache(std::size_t Capacity) : State(std::make_unique<Impl>(Capacity)) {
	}
	LodNeighborCache::~LodNeighborCache() = default;
	std::size_t LodNeighborCache::TileCount() const {
		return State->Tiles.size();
	}
	std::uint64_t LodNeighborCache::Hits() const {
		return State->Hits;
	}
	std::optional<Neighbor> LodNeighborCache::Find(TileId Tile, std::uint8_t Edge) {
		if(!Tile.IsValid() || Edge > 2) return {};
		auto &S = *State;
		auto It = S.Tiles.find(Tile);
		if(It == S.Tiles.end()) {
			if(S.Tiles.size() >= S.Capacity) {
				S.Tiles.erase(S.Recent.back());
				S.Recent.pop_back();
			}
			S.Recent.push_front(Tile);
			It = S.Tiles.emplace(Tile, Impl::Entry{{}, S.Recent.begin()}).first;
		} else
			S.Recent.splice(S.Recent.begin(), S.Recent, It->second.Recent);
		auto &Value = It->second.Adjacent[Edge];
		if(Value)
			++S.Hits;
		else
			Value = GetNeighbor(Tile, Edge);
		return Value;
	}
	std::optional<Neighbor> LodTree::AdjacentTile(TileId Tile, std::uint8_t Edge) const {
		return Neighbors ? Neighbors->Find(Tile, Edge) : GetNeighbor(Tile, Edge);
	}

	LodTree::LodTree(std::shared_ptr<LodNeighborCache> Cache) : Neighbors(std::move(Cache)) {
		for(std::uint8_t Root = 0; Root < 20; ++Root) LeafTiles.insert({0, Root, 0});
	}

	bool LodTree::IsLeaf(TileId Tile) const {
		return LeafTiles.contains(Tile);
	}

	std::optional<TileId> LodTree::CoveringLeaf(TileId Tile) const {
		if(!Tile.IsValid()) return {};
		for(;;) {
			if(IsLeaf(Tile)) return Tile;
			if(Tile.Level == 0) return {};
			Tile = *Tile.Parent();
		}
	}

	bool LodTree::HasFinerNeighbor(TileId Tile, std::uint8_t Edge) const {
		if(!IsLeaf(Tile)) return false;
		const auto Adjacent = AdjacentTile(Tile, Edge);
		return Adjacent && !CoveringLeaf(Adjacent->Tile);
	}

	/** 別ワーカーの葉分割へ可変な探索状態を渡さない */
	void LodTree::DetachNeighborCache() {
		if(Neighbors) Neighbors = std::make_shared<LodNeighborCache>();
		// 葉とその順序を保ったまま所有者を切離す終了
		return;
	}
	LodEdit LodTree::Refine(TileId Tile, std::size_t LeafBudget) {
		if(LeafBudget < 20 || LeafBudget > MaximumLeaves) return {LodEditStatus::InvalidBudget, {}};
		if(!IsLeaf(Tile)) return {LodEditStatus::NotLeaf, {}};
		if(Tile.Level == MaxTileLevel) return {LodEditStatus::DepthLimit, {}};
		std::set<TileId, TileOrder> Pending;
		const auto Gather = [&](auto &&Self, TileId Current) -> void {
			if(!Pending.insert(Current).second) return;
			for(std::uint8_t Edge = 0; Edge < 3; ++Edge) {
				const auto Adjacent = AdjacentTile(Current, Edge);
				const auto Coarse = CoveringLeaf(Adjacent->Tile);
				if(Coarse && Coarse->Level < Current.Level) Self(Self, *Coarse);
			}
		};
		Gather(Gather, Tile);
		if(LeafTiles.size() > LeafBudget || Pending.size() > (LeafBudget - LeafTiles.size()) / 3)
			return {LodEditStatus::BudgetExceeded, {}};

		// Allocate the new nodes and result before touching the old partition.
		// Node transfers below allocate nothing, preserving the old coverage even
		// if allocation of a candidate fails with std::bad_alloc.
		LodEdit Result{LodEditStatus::Applied, {Pending.begin(), Pending.end()}};
		std::set<TileId, TileOrder> NewLeaves;
		for(const auto Parent : Pending)
			for(std::uint8_t Child = 0; Child < 4; ++Child) NewLeaves.insert(*Parent.Child(Child));
		for(const auto Parent : Pending) LeafTiles.erase(Parent);
		LeafTiles.merge(NewLeaves);
		return Result;
	}

	LodEditStatus LodTree::Coarsen(TileId Parent) {
		if(!Parent.IsValid() || Parent.Level == MaxTileLevel) return LodEditStatus::ChildrenNotLeaves;
		for(std::uint8_t Child = 0; Child < 4; ++Child)
			if(!IsLeaf(*Parent.Child(Child))) return LodEditStatus::ChildrenNotLeaves;
		for(std::uint8_t Edge = 0; Edge < 3; ++Edge) {
			const auto Adjacent = AdjacentTile(Parent, Edge);
			if(CoveringLeaf(Adjacent->Tile)) continue;
			// Only the two children touching this edge matter. Refinement on the
			// far side of a neighbor must not prevent an otherwise legal merge.
			for(std::uint8_t Endpoint = 1; Endpoint <= 2; ++Endpoint) {
				const auto Child = *Adjacent->Tile.Child((Adjacent->Edge + Endpoint) % 3);
				if(!IsLeaf(Child)) return LodEditStatus::NeighborTooFine;
			}
		}
		// Insertion can throw; do it while all four children are still present.
		LeafTiles.insert(Parent);
		for(std::uint8_t Child = 0; Child < 4; ++Child) LeafTiles.erase(*Parent.Child(Child));
		return LodEditStatus::Applied;
	}

	namespace {
		struct VertexOrder {
			bool operator()(const VertexKey &A, const VertexKey &B) const {
				return std::tie(A.Kind, A.Owner, A.Other, A.Level, A.U, A.V) <
					std::tie(B.Kind, B.Owner, B.Other, B.Level, B.U, B.V);
			}
		};
	}

	std::optional<StitchPattern> GetStitchPattern(std::uint8_t Mask) {
		if(Mask > 7) return {};
		StitchPattern Result;
		constexpr std::array<std::uint8_t, 3> V{0, 1, 2}, M{3, 4, 5};
		const auto Emit = [&](std::uint8_t A, std::uint8_t B, std::uint8_t C) { Result.Triangles[Result.Count++] = {A, B, C}; };
		if(Mask == 0)
			Emit(V[0], V[1], V[2]);
		else if(Mask == 7) {
			Emit(V[0], M[2], M[1]);
			Emit(M[2], V[1], M[0]);
			Emit(M[1], M[0], V[2]);
			Emit(M[2], M[0], M[1]);
		} else if(Mask == 1 || Mask == 2 || Mask == 4) {
			const std::uint8_t Edge = Mask == 1 ? 0 : (Mask == 2 ? 1 : 2);
			Emit(V[Edge], V[(Edge + 1) % 3], M[Edge]);
			Emit(V[Edge], M[Edge], V[(Edge + 2) % 3]);
		} else {
			const std::uint8_t Unsplit = Mask == 6 ? 0 : (Mask == 5 ? 1 : 2);
			const auto AB = M[(Unsplit + 2) % 3], CA = M[(Unsplit + 1) % 3];
			Emit(V[Unsplit], AB, CA);
			Emit(AB, V[(Unsplit + 1) % 3], V[(Unsplit + 2) % 3]);
			Emit(AB, V[(Unsplit + 2) % 3], CA);
		}
		return Result;
	}

	std::optional<AdaptiveSphereMesh> BuildAdaptiveSphere(const LodTree &Tree) {
		AdaptiveSphereMesh Output;
		auto &Mesh = Output.Mesh;
		Mesh.Triangles.reserve(Tree.Leaves().size() * 2);
		Output.TriangleOwners.reserve(Mesh.Triangles.capacity());
		Mesh.Directions.reserve(Tree.Leaves().size());
		Mesh.Keys.reserve(Tree.Leaves().size());
		// 完全な２対１平衡分割では，辺の中点が他の葉の角に有る事と，対岸が一段細かい事が同値
		// 出力頂点は従来の順序で遅延追加し，先行収集する鍵だけを隣接判定へ使う
		std::set<VertexKey, VertexOrder> Corners;
		for(const TileId Tile : Tree.Leaves())
			for(std::uint8_t Corner = 0; Corner < 3; ++Corner) Corners.insert(*GetVertexKey(Tile, Corner));
		std::map<VertexKey, std::uint32_t, VertexOrder> Vertices;
		const auto Vertex = [&](VertexKey Key) -> std::optional<std::uint32_t> {
			if(const auto It = Vertices.find(Key); It != Vertices.end()) return It->second;
			const auto Direction = GetDirection(Key);
			if(!Direction) return {};
			const auto Index = static_cast<std::uint32_t>(Mesh.Directions.size());
			Vertices.emplace(Key, Index);
			Mesh.Directions.push_back(*Direction);
			Mesh.Keys.push_back(Key);
			return Index;
		};
		for(const auto Tile : Tree.Leaves()) {
			std::array<std::uint32_t, 3> V{}, M{};
			std::uint8_t Mask = 0;
			for(std::uint8_t I = 0; I < 3; ++I) {
				const auto Index = Vertex(*GetVertexKey(Tile, I));
				if(!Index) return {};
				V[I] = *Index;
				const std::optional<TileId> CenterChild = Tile.Child(3);
				if(CenterChild) {
					const VertexKey Midpoint = *GetVertexKey(*CenterChild, (I + 1) % 3);
					if(!Corners.contains(Midpoint)) continue;
					const std::optional<std::uint32_t> Mid = Vertex(Midpoint);
					if(!Mid) return {};
					M[I] = *Mid;
					Mask |= static_cast<std::uint8_t>(1 << I);
				}
			}
			++Output.StitchMaskCounts[Mask];
			const auto Emit = [&](std::uint32_t A, std::uint32_t B, std::uint32_t C) {
				Mesh.Triangles.push_back({A, B, C});
				Output.TriangleOwners.push_back(Tile);
			};
			const auto Pattern = *GetStitchPattern(Mask);
			const std::array<std::uint32_t, 6> VerticesByRole{V[0], V[1], V[2], M[0], M[1], M[2]};
			for(unsigned I = 0; I < Pattern.Count; ++I) {
				const auto T = Pattern.Triangles[I];
				Emit(VerticesByRole[T[0]], VerticesByRole[T[1]], VerticesByRole[T[2]]);
			}
		}
		return Output;
	}
}
