#include "PlanetSphericalMesh.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 公開された任意網の全辺・向き・連結と球面被覆を検査する
	 * @param Directions 同じ単位球の頂点
	 * @param Triangles 外向き三角形
	 * @param Stop 中断要求
	 * @param MaximumNodes 検証済の網の種別に対応する上限，最大１６７９３８点
	 * @return 有界な閉球面として接続処理へ渡せる場合に真
	 */
	bool IsClosedSphericalSamplingMesh(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::stop_token Stop,
		const std::size_t MaximumNodes
	) {
		const std::size_t Count = Directions.size();
		const bool ValidBounds = MaximumNodes > 11 && MaximumNodes < 167939 && Count > 11 && Count <= MaximumNodes;
		const bool ValidInput = ValidBounds && Triangles.size() == 2 * (Count - 2) && !Stop.stop_requested();
		// 範囲外・非球面の面数・中断に対する返戻
		if(!ValidInput) return false;
		std::vector<std::pair<std::uint64_t, int>> Edges;
		std::vector<std::uint32_t> Parents(Count);
		std::iota(Parents.begin(), Parents.end(), 0U);
		const auto Root = [&Parents](std::uint32_t Index) -> std::uint32_t {
			while(Parents[Index] != Index) {
				Parents[Index] = Parents[Parents[Index]];
				Index = Parents[Index];
			}
			// 経路圧縮した代表番号の返戻
			return Index;
		};
		Edges.reserve(Triangles.size() * 3);
		double Area = 0;
		std::uint32_t Visited = 0;
		for(const std::array<std::uint32_t, 3> &Triangle : Triangles) {
			// 全頂点番号の検査前に添字を参照しない返戻
			if(!(Visited++ & 255) && Stop.stop_requested()) return false;
			// 範囲外の頂点番号を含む面の返戻
			for(const std::uint32_t Vertex : Triangle) if(Vertex >= Count) return false;
			const Vec3 A = Directions[Triangle[0]].Vector(), B = Directions[Triangle[1]].Vector();
			const Vec3 C = Directions[Triangle[2]].Vector();
			const double Determinant = A.Dot((B - A).Cross(C - A));
			// 外向きの非退化局所面だけを受理する返戻
			if(!(Determinant > 0) || !(A.Dot(B) > 0) || !(B.Dot(C) > 0) || !(C.Dot(A) > 0)) return false;
			Area += 2 * std::atan2(Determinant, 1 + A.Dot(B) + B.Dot(C) + C.Dot(A));
			for(unsigned Side = 0; Side < 3; ++Side) {
				const std::uint32_t First = Triangle[Side], Second = Triangle[(Side + 1) % 3];
				const std::uint64_t Key = static_cast<std::uint64_t>(std::min(First, Second)) << 32 | std::max(First, Second);
				Edges.emplace_back(Key, First < Second ? 1 : -1);
				const std::uint32_t R = Root(First), S = Root(Second);
				if(R != S) Parents[std::max(R, S)] = std::min(R, S);
			}
		}
		std::sort(Edges.begin(), Edges.end());
		for(std::size_t Index = 0; Index < Edges.size(); Index += 2) {
			// 両側の向きが反対の共有辺だけを受理する返戻
			if(Edges[Index].first != Edges[Index + 1].first || Edges[Index].second + Edges[Index + 1].second) return false;
			// 同じ辺に３面以上が接する入力の返戻
			if(Index + 2 < Edges.size() && Edges[Index].first == Edges[Index + 2].first) return false;
		}
		const std::uint32_t First = Root(0);
		// 孤立した頂点又は別の球面を混合しない返戻
		for(std::uint32_t Index = 0; Index < Count; ++Index) if(Root(Index) != First) return false;
		// 全球面の面積との一致を含む返戻
		return std::isfinite(Area) && std::abs(Area - 4 * std::acos(-1.0)) < 1E-8 && !Stop.stop_requested();
	}
}
