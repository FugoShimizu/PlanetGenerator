#include "PlanetDrainageRefinement.h"
#include "PlanetDrainage.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 元の球面網の共有辺を一度だけ分割し，八種類の red/green 分割で閉じた面を保つ
	 * 計算量：O(n log n ＋ n log(k ＋ 1))，記憶量：O(n ＋ k)，n は元の網，k は指定辺数
	 * @param Edges 段階６の元頂点番号で指定する辺，逆順・重複・入力順には依存しない
	 * @param Stop 中断要求
	 * @return 元頂点と面積被覆を保つ局所網，不正な辺・過大入力・中断時は空
	 */
	std::optional<DrainageRefinementMesh> BuildDrainageRefinement(
		const std::span<const std::array<std::uint32_t, 2>> Edges,
		const std::stop_token Stop
	) {
		// 最大段階の全辺より大きい要求や中断の返戻
		if(Edges.size() > 122880 || Stop.stop_requested()) return {};
		const std::optional<SphereMesh> Base = BuildUniformSphere(6);
		// 固定した元網の準備失敗時の返戻
		if(!Base) return {};
		DrainageRefinementMesh Result;
		Result.Directions = Base->Directions;
		Result.SplitEdges.assign(Edges.begin(), Edges.end());
		for(std::array<std::uint32_t, 2> &Edge : Result.SplitEdges) {
			// 元の頂点を参照しない又は退化した指定の返戻
			if(Edge[0] >= Base->Directions.size() || Edge[1] >= Base->Directions.size() || Edge[0] == Edge[1]) return {};
			if(Edge[0] > Edge[1]) std::swap(Edge[0], Edge[1]);
		}
		std::sort(Result.SplitEdges.begin(), Result.SplitEdges.end());
		Result.SplitEdges.erase(std::unique(Result.SplitEdges.begin(), Result.SplitEdges.end()), Result.SplitEdges.end());
		std::vector<std::array<std::uint32_t, 2>> Available;
		Available.reserve(Base->Triangles.size() * 3);
		for(const std::array<std::uint32_t, 3> &Triangle : Base->Triangles) for(unsigned Edge = 0; Edge < 3; ++Edge) {
			const std::uint32_t A = Triangle[Edge], B = Triangle[(Edge + 1) % 3];
			Available.push_back({ std::min(A, B), std::max(A, B) });
		}
		std::sort(Available.begin(), Available.end());
		for(const std::array<std::uint32_t, 2> &Edge : Result.SplitEdges) {
			// 単に存在する二頂点だけを辺と誤認しない返戻
			if(!std::binary_search(Available.begin(), Available.end(), Edge) || Stop.stop_requested()) return {};
			const std::optional<UnitDirection> Middle =
			UnitDirection::FromVector(Base->Directions[Edge[0]].Vector() + Base->Directions[Edge[1]].Vector());
			// 球面上の共有中点を作れない場合の返戻
			if(!Middle) return {};
			Result.Directions.push_back(*Middle);
		}
		Result.Triangles.reserve(Base->Triangles.size() + Result.SplitEdges.size() * 2);
		for(const std::array<std::uint32_t, 3> &V : Base->Triangles) {
			// 面の構成を中断した場合の返戻
			if(Stop.stop_requested()) return {};
			std::array<std::uint32_t, 3> M {};
			unsigned Mask = 0;
			for(unsigned Edge = 0; Edge < 3; ++Edge) {
				const std::uint32_t A = V[(Edge + 1) % 3], B = V[(Edge + 2) % 3];
				const std::array<std::uint32_t, 2> Key { std::min(A, B), std::max(A, B) };
				const std::vector<std::array<std::uint32_t, 2>>::iterator Found =
				std::lower_bound(Result.SplitEdges.begin(), Result.SplitEdges.end(), Key);
				if(Found == Result.SplitEdges.end() || *Found != Key) continue;
				Mask |= 1U << Edge;
				M[Edge] = static_cast<std::uint32_t>(Base->Directions.size() + (Found - Result.SplitEdges.begin()));
			}
			const auto Emit = [&Result](const std::uint32_t A, const std::uint32_t B, const std::uint32_t C) -> void {
				Result.Triangles.push_back({ A, B, C });
				// 一面を追加した終端
				return;
			};
			if(!Mask) Emit(V[0], V[1], V[2]);
			else if(Mask == 7) {
				Emit(V[0], M[2], M[1]);
				Emit(M[2], V[1], M[0]);
				Emit(M[1], M[0], V[2]);
				Emit(M[2], M[0], M[1]);
			} else if(Mask == 1 || Mask == 2 || Mask == 4) {
				const unsigned Edge = Mask == 1 ? 0 : Mask == 2 ? 1 : 2;
				Emit(V[Edge], V[(Edge + 1) % 3], M[Edge]);
				Emit(V[Edge], M[Edge], V[(Edge + 2) % 3]);
			} else {
				const unsigned Edge = Mask == 6 ? 0 : Mask == 5 ? 1 : 2;
				const std::uint32_t AB = M[(Edge + 2) % 3], CA = M[(Edge + 1) % 3];
				Emit(V[Edge], AB, CA);
				Emit(AB, V[(Edge + 1) % 3], V[(Edge + 2) % 3]);
				Emit(AB, V[(Edge + 2) % 3], CA);
			}
		}
		// 隣接面も同じ中点を共有する局所網の返戻
		return Result;
	}

	/**
	 * 見落とした窪地の指定辺だけを細分化し，同じ海岸・面積・排水解法で準備する
	 * 計算量：O(n log n ＋ m p)，n は局所網，m は河道数，p は縦断標本数
	 * @param Ocean 海陸と地形の共通の場
	 * @param Edges 段階６の元頂点を参照する分割辺
	 * @param Stop 中断要求
	 * @return 局所網からの排水と河川縦断，不正な辺・準備失敗・中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::CreateWithRefinedEdges(
		const OceanConnectivityField &Ocean,
		const std::span<const std::array<std::uint32_t, 2>> Edges,
		const std::stop_token Stop
	) {
		const std::optional<DrainageRefinementMesh> Mesh = BuildDrainageRefinement(Edges, Stop);
		// 正規の共有網を準備出来ない場合の返戻
		if(!Mesh) return {};
		const auto Height = [&Ocean](const UnitDirection Direction) -> double {
			// 同じ元地形から標高を取得した返戻
			return Ocean.EvaluateElevation(Direction);
		};
		// 面積と沿岸接続を既存の正規経路と共有した排水の返戻
		return CreateOnMesh(Ocean, Mesh->Directions, Mesh->Triangles, Height, true, Stop);
	}
}
