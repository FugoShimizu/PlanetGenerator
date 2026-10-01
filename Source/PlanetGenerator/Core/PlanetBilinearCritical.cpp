#include "PlanetBilinearCritical.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace PlanetCore {
	/**
	 * 双線形の二成分場を消去してセル内の孤立零点候補を求める
	 * 計算量 O(1)，作業領域 O(1)；退化曲線や補間外の元場の完全探索ではない
	 * @param Corners 単位セル四隅の二成分；有限値以外は候補を返さない
	 * @return 正規化した場の行列式付き候補；退化した消去式には印を付ける
	 */
	BilinearCriticalCandidates FindBilinearCriticalCandidates(const std::array<std::array<double, 2>, 4> &Corners) {
		BilinearCriticalCandidates Result;
		double Scale = 0;
		for(const std::array<double, 2> &Corner : Corners) for(const double Value : Corner) {
			// 非有限値を有効な零点へ変換しない返戻
			if(!std::isfinite(Value)) return Result;
			Scale = std::max(Scale, std::abs(Value));
		}
		if(!Scale) {
			Result.HasDegenerateElimination = true;
			// 全セルが零の時に孤立点を捏造しない返戻
			return Result;
		}
		std::array<std::array<double, 4>, 2> Coefficients {};
		for(std::uint32_t Component = 0; Component < 2; ++Component) {
			const double A = Corners[0][Component] / Scale, B = Corners[1][Component] / Scale, C = Corners[2][Component] / Scale;
			const double D = Corners[3][Component] / Scale;
			Coefficients[Component] = { A, B - A, C - A, D - B - C + A };
		}
		constexpr double Tolerance = 256 * std::numeric_limits<double>::epsilon();
		// 二軸の消去で，片方の分母が零となる正規の零点も保持する
		for(unsigned Axis = 0; Axis < 2; ++Axis) {
			const std::array<double, 4> &P = Coefficients[0], &Q = Coefficients[1];
			const double A = P[0], B = P[1 + Axis], C = P[2 - Axis], D = P[3], E = Q[0], F = Q[1 + Axis], G = Q[2 - Axis], H = Q[3];
			const double Quadratic = B * H - F * D, Linear = A * H + B * G - E * D - F * C, Constant = A * G - E * C;
			std::array<double, 2> Roots {};
			unsigned Count = 0;
			if(!Quadratic) {
				if(!Linear) {
					if(!Constant) Result.HasDegenerateElimination = true;
					continue;
				}
				Roots[Count++] = -Constant / Linear;
			} else {
				const double Discriminant = Linear * Linear - 4 * Quadratic * Constant;
				if(Discriminant < 0) continue;
				const double Stable = -0.5 * (Linear + std::copysign(std::sqrt(Discriminant), Linear));
				if(!Stable) Roots[Count++] = -Linear / (2 * Quadratic);
				else {
					Roots[Count++] = Stable / Quadratic;
					if(Discriminant > 0) Roots[Count++] = Constant / Stable;
				}
			}
			for(unsigned Index = 0; Index < Count; ++Index) {
				const double Raw = Roots[Index];
				if(!std::isfinite(Raw) || Raw < -Tolerance || Raw > 1 + Tolerance) continue;
				const double U = std::clamp(Raw, 0.0, 1.0), FirstDenominator = C + D * U, SecondDenominator = G + H * U;
				if(!FirstDenominator && !SecondDenominator) continue;
				const double Other =
				std::abs(FirstDenominator) >= std::abs(SecondDenominator) ? -(A + B * U) / FirstDenominator : -(E + F * U) / SecondDenominator;
				if(!std::isfinite(Other) || Other < -Tolerance || Other > 1 + Tolerance) continue;
				const double V = std::clamp(Other, 0.0, 1.0);
				if(std::abs(A + B * U + C * V + D * U * V) > Tolerance || std::abs(E + F * U + G * V + H * U * V) > Tolerance) continue;
				const std::array<double, 2> Point { Axis ? V : U, Axis ? U : V };
				bool Duplicate = false;
				for(std::uint32_t I = 0; I < Result.Count; ++I) {
					Duplicate = Duplicate || std::abs(Point[0] - Result.Points[I].Coordinates[0]) <= Tolerance &&
					std::abs(Point[1] - Result.Points[I].Coordinates[1]) <= Tolerance;
				}
				if(Duplicate || Result.Count == Result.Points.size()) continue;
				const double Determinant =
				(P[1] + P[3] * Point[1]) * (Q[2] + Q[3] * Point[0]) - (P[2] + P[3] * Point[0]) * (Q[1] + Q[3] * Point[1]);
				// 非孤立又は特異な根は，実鞍点の候補へ昇格しない
				if(!Determinant) continue;
				Result.Points[Result.Count++] = { Point, Determinant };
			}
		}
		// 補間場に限定した有界な候補の返戻
		return Result;
	}
}
