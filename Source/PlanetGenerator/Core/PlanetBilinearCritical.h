#pragma once

#include <array>
#include <cstdint>

namespace PlanetCore {
	struct BilinearCriticalCandidate {
		// 単位セル内の座標と補間勾配場の局所行列式；元地形の停留点ではない
		std::array<double, 2> Coordinates {};
		double JacobianDeterminant = 0;
	};

	struct BilinearCriticalCandidates {
		// 双線形の二成分場の孤立零点は最大二つ；退化曲線は候補にしない
		std::array<BilinearCriticalCandidate, 2> Points {};
		std::uint32_t Count = 0;
		bool HasDegenerateElimination = false;
	};

	// 頂点順は（０，０），（１，０），（０，１），（１，１）；有限な入力を呼出側で保証する
	BilinearCriticalCandidates FindBilinearCriticalCandidates(const std::array<std::array<double, 2>, 4> &Corners);
}
