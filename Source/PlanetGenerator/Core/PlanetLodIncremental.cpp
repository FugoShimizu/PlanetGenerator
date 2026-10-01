#include "PlanetLod.h"
#include <limits>
#include <unordered_map>

namespace PlanetCore {
	namespace {
		/** 正規頂点の各属性を混合し，番号の挿入順序とは独立に探索する */
		struct LodVertexHash {
			/** 等しい正規鍵に同じ探索値を与える */
			std::size_t operator()(const VertexKey &Key) const noexcept {
				const auto Mix = [](std::uint64_t Value) {
					Value = (Value ^ (Value >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
					Value = (Value ^ (Value >> 27)) * UINT64_C(0x94d049bb133111eb);
					// 上位と下位の偏りを混合した値の返戻
					return Value ^ (Value >> 31);
				};
				const std::uint64_t Meta = static_cast<std::uint64_t>(Key.Kind) | (static_cast<std::uint64_t>(Key.Owner) << 8) |
					(static_cast<std::uint64_t>(Key.Other) << 16) | (static_cast<std::uint64_t>(Key.Level) << 24);
				// 一致判定自体は VertexKey の全属性に依る等値比較へ委ねる
				return static_cast<std::size_t>(Mix(Meta ^ Mix(Key.U) ^ (Mix(Key.V) >> 1)));
			}
		};

		constexpr std::uint32_t MissingVertex = std::numeric_limits<std::uint32_t>::max();

		struct PreviousLeaf {
			TileId Tile;
			std::array<std::uint32_t, 6> Roles{};
			std::uint8_t Mask = 0;
		};

		/**
		 * 元の葉に属する接合面から，角と中点の参照を復元する
		 * @param Previous 標準の接合順序で組立てた既存面
		 * @param Stop 中断要求
		 * @return 葉順の参照表，不正又は中断時は空
		 */
		std::optional<std::vector<PreviousLeaf>> ReadPreviousLeaves(const AdaptiveSphereMesh &Previous, std::stop_token Stop) {
			const SphereMesh &Mesh = Previous.Mesh;
			// 参照数の不整合を表す空値の返戻
			if(Mesh.Keys.size() != Mesh.Directions.size() || Previous.TriangleOwners.size() != Mesh.Triangles.size() ||
				Mesh.Triangles.empty())
				return {};
			std::vector<PreviousLeaf> Leaves;
			std::array<std::size_t, 8> Counts{};
			Leaves.reserve(Mesh.Triangles.size());
			for(std::size_t Begin = 0; Begin < Mesh.Triangles.size();) {
				// 中断時の未完成結果の返戻
				if(Stop.stop_requested()) return {};
				const TileId Tile = Previous.TriangleOwners[Begin];
				// 不正な葉又は順序違反の返戻
				if(!Tile.IsValid() || (!Leaves.empty() && !TileOrder{}(Leaves.back().Tile, Tile))) return {};
				std::size_t End = Begin + 1;
				while(End < Mesh.Triangles.size() && Previous.TriangleOwners[End] == Tile) ++End;
				const std::size_t Count = End - Begin;
				// 標準の接合形状で表現出来ない面数の返戻
				if(Count > 4) return {};
				for(std::size_t Face = Begin; Face < End; ++Face)
					for(const std::uint32_t Index : Mesh.Triangles[Face]) {
						// 範囲外参照の返戻
						if(Index >= Mesh.Keys.size()) return {};
					}
				PreviousLeaf Leaf;
				Leaf.Tile = Tile;
				Leaf.Roles.fill(MissingVertex);
				if(Count == 4)
					Leaf.Mask = 7;
				else if(Count == 2 || Count == 3) {
					std::uint8_t Corner = 0;
					while(Corner < 3 && *GetVertexKey(Tile, Corner) != Mesh.Keys[Mesh.Triangles[Begin][0]]) ++Corner;
					// 先頭の角が所有葉に属さない面の返戻
					if(Corner == 3) return {};
					Leaf.Mask = static_cast<std::uint8_t>(Count == 2 ? (1 << Corner) : (7 ^ (1 << Corner)));
				}
				const StitchPattern Pattern = *GetStitchPattern(Leaf.Mask);
				for(std::size_t Face = 0; Face < Count; ++Face)
					for(std::size_t Corner = 0; Corner < 3; ++Corner) {
						std::uint32_t &Role = Leaf.Roles[Pattern.Triangles[Face][Corner]];
						const std::uint32_t Index = Mesh.Triangles[Begin + Face][Corner];
						// 同じ役割に異なる頂点を割当てた不整合の返戻
						if(Role != MissingVertex && Role != Index) return {};
						Role = Index;
					}
				++Counts[Leaf.Mask];
				Leaves.push_back(Leaf);
				Begin = End;
			}
			// 接合数の不整合の返戻
			if(Counts != Previous.StitchMaskCounts) return {};
			// 元の面と同順の参照表の返戻
			return Leaves;
		}
	}

	/**
	 * 不変な葉の接合と頂点方向を再利用し，全組立と同じ順序の球面を作る
	 * @param Tree 更新後の完全な平衡木
	 * @param Previous BuildAdaptiveSphere 又は本関数で作った変更前の面
	 * @param Stop 中断要求
	 * @return 更新後の球面，参照不整合又は中断時は空
	 */
	std::optional<AdaptiveSphereMesh> UpdateAdaptiveSphere(
		const LodTree &Tree, const AdaptiveSphereMesh &Previous, std::stop_token Stop) {
		const std::optional<std::vector<PreviousLeaf>> Old = ReadPreviousLeaves(Previous, Stop);
		// 使用出来ない既存面の返戻
		if(!Old) return {};
		std::set<TileId, TileOrder> ChangedNeighbors;
		std::size_t Cursor = 0;
		for(const TileId Tile : Tree.Leaves()) {
			// 中断時の未完成結果の返戻
			if(Stop.stop_requested()) return {};
			while(Cursor < Old->size() && TileOrder{}((*Old)[Cursor].Tile, Tile)) ++Cursor;
			if(Cursor < Old->size() && (*Old)[Cursor].Tile == Tile) continue;
			for(std::uint8_t Edge = 0; Edge < 3; ++Edge) {
				const Neighbor Adjacent = *GetNeighbor(Tile, Edge);
				if(const std::optional<TileId> Cover = Tree.CoveringLeaf(Adjacent.Tile))
					ChangedNeighbors.insert(*Cover);
				else {
					// 縮約時は隣の細かい葉も接合中点を失う為，接辺する２子を含める
					for(std::uint8_t Endpoint = 1; Endpoint <= 2; ++Endpoint) {
						const std::optional<TileId> Child = Adjacent.Tile.Child((Adjacent.Edge + Endpoint) % 3);
						if(Child && Tree.IsLeaf(*Child)) ChangedNeighbors.insert(*Child);
					}
				}
			}
		}
		AdaptiveSphereMesh Output;
		SphereMesh &Mesh = Output.Mesh;
		Mesh.Keys.reserve(Tree.Leaves().size());
		Mesh.Directions.reserve(Tree.Leaves().size());
		Mesh.Triangles.reserve(Tree.Leaves().size() * 2);
		Output.TriangleOwners.reserve(Mesh.Triangles.capacity());
		std::unordered_map<VertexKey, std::uint32_t, LodVertexHash> Vertices;
		Vertices.reserve(Tree.Leaves().size());
		const auto Vertex = [&](VertexKey Key, std::optional<UnitDirection> Known) -> std::optional<std::uint32_t> {
			// 共有頂点の既存番号の返戻
			if(const std::unordered_map<VertexKey, std::uint32_t, LodVertexHash>::const_iterator Found =
					Vertices.find(Key);
				Found != Vertices.end())
				return Found->second;
			const std::optional<UnitDirection> Direction = Known ? Known : GetDirection(Key);
			// 不正な正規鍵の返戻
			if(!Direction) return {};
			const std::uint32_t Index = static_cast<std::uint32_t>(Mesh.Keys.size());
			Vertices.emplace(Key, Index);
			Mesh.Keys.push_back(Key);
			Mesh.Directions.push_back(*Direction);
			// 全組立と同じ遅延挿入順序の番号の返戻
			return Index;
		};
		Cursor = 0;
		for(const TileId Tile : Tree.Leaves()) {
			// 中断時の未完成結果の返戻
			if(Stop.stop_requested()) return {};
			while(Cursor < Old->size() && TileOrder{}((*Old)[Cursor].Tile, Tile)) ++Cursor;
			const PreviousLeaf *Reuse = Cursor < Old->size() && (*Old)[Cursor].Tile == Tile && !ChangedNeighbors.contains(Tile)
				? &(*Old)[Cursor]
				: nullptr;
			std::array<std::uint32_t, 6> Roles{};
			std::uint8_t Mask = Reuse ? Reuse->Mask : 0;
			for(std::uint8_t Corner = 0; Corner < 3; ++Corner) {
				if(!Reuse && Tree.HasFinerNeighbor(Tile, Corner)) Mask |= static_cast<std::uint8_t>(1 << Corner);
				for(const std::uint8_t Role : {Corner, static_cast<std::uint8_t>(Corner + 3)}) {
					if(Role >= 3 && !(Mask & (1 << Corner))) continue;
					std::optional<std::uint32_t> Index;
					if(Reuse) {
						const std::uint32_t OldIndex = Reuse->Roles[Role];
						Index = Vertex(Previous.Mesh.Keys[OldIndex], Previous.Mesh.Directions[OldIndex]);
					} else {
						const VertexKey Key =
							Role < 3 ? *GetVertexKey(Tile, Corner) : *GetVertexKey(*Tile.Child(3), (Corner + 1) % 3);
						Index = Vertex(Key, {});
					}
					// 頂点を復元出来ない候補の返戻
					if(!Index) return {};
					Roles[Role] = *Index;
				}
			}
			++Output.StitchMaskCounts[Mask];
			const StitchPattern Pattern = *GetStitchPattern(Mask);
			for(std::uint8_t Face = 0; Face < Pattern.Count; ++Face) {
				const std::array<std::uint8_t, 3> &Triangle = Pattern.Triangles[Face];
				Mesh.Triangles.push_back({Roles[Triangle[0]], Roles[Triangle[1]], Roles[Triangle[2]]});
				Output.TriangleOwners.push_back(Tile);
			}
		}
		// 不変部分と変更部分を同じ接合規則で組立てた面の返戻
		return Output;
	}
}
