#include "PlanetGradientProjection.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace PlanetCore {
	/**
	 * 球面接線上の凸制約へ元勾配を最短距離で射影し，不要な横断方向の平坦化を避ける
	 * 原点は常に許容される為，最短点は内部・単一境界への直交射影・２境界の交点の何れかとなる
	 * @param Point 接線平面を定める単位方向
	 * @param Gradient 元の有限な勾配，半径成分は除外する
	 * @param Constraints 有限な単位接線と，０を含む微分の上下限
	 * @return 制約を満たす最近点，不正又は演算範囲を超えた入力では空
	 */
	std::optional<Vec3> ProjectSphericalGradient(
		const UnitDirection Point,
		const Vec3 Gradient,
		const std::span<const SphericalGradientConstraint> Constraints
	) {
		// 非有限な元勾配を比較へ渡さない返戻
		if(!std::isfinite(Gradient.X) || !std::isfinite(Gradient.Y) || !std::isfinite(Gradient.Z)) return {};
		const Vec3 P = Point.Vector(), Original = Gradient - P * P.Dot(Gradient);
		const double Magnitude = Original.Length();
		// 有限成分からの内積が範囲を超えた場合の返戻
		if(!std::isfinite(Magnitude)) return {};
		const Vec3 Axis = std::abs(P.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
		const Vec3 East = UnitDirection::FromVector(P.Cross(Axis))->Vector(), North = P.Cross(East);
		const double Scale = std::max(1.0, Magnitude), GX = Original.Dot(East) / Scale, GY = Original.Dot(North) / Scale;
		// 各半平面は x X + y Y <= 上限
		std::vector<std::array<double, 3>> Planes;
		Planes.reserve(2 * Constraints.size());
		for(const SphericalGradientConstraint &Constraint : Constraints) {
			const Vec3 T = Constraint.Tangent;
			const bool HasValidTangent =
			std::isfinite(T.X) && std::isfinite(T.Y) && std::isfinite(T.Z) && std::abs(T.Dot(P)) < 1E-10 && std::abs(T.Dot(T) - 1) < 1E-10;
			const bool HasInvalidBounds = !std::isfinite(Constraint.MaximumDerivative) || Constraint.MaximumDerivative < 0 ||
			!std::isfinite(Constraint.MinimumDerivative) || Constraint.MinimumDerivative > 0;
			// 原点を含まない制約，又は不正な方向を黙って補正しない返戻
			if(!HasValidTangent || HasInvalidBounds) return {};
			const double X = T.Dot(East), Y = T.Dot(North), Bound = Constraint.MaximumDerivative / Scale;
			Planes.push_back({ X, Y, Bound });
			Planes.push_back({ -X, -Y, -Constraint.MinimumDerivative / Scale });
		}
		constexpr double Roundoff = 32 * std::numeric_limits<double>::epsilon();
		double BestX = 0, BestY = 0, BestSquared = GX * GX + GY * GY;
		const auto Consider = [&](const double X, const double Y) -> void {
			const double Squared = (X - GX) * (X - GX) + (Y - GY) * (Y - GY);
			// 非有限な交点又は既知の最近点より遠い候補の返戻
			if(!std::isfinite(Squared) || Squared >= BestSquared) return;
			for(const std::array<double, 3> &Plane : Planes) {
				const double Value = Plane[0] * X + Plane[1] * Y;
				// 正規化座標の演算丸めを越えて制約外なら不採用とする返戻
				if(Value - Plane[2] > Roundoff * (1 + std::abs(Value) + std::abs(Plane[2]))) return;
			}
			BestX = X;
			BestY = Y;
			BestSquared = Squared;
			// 終了
			return;
		};
		Consider(GX, GY);
		for(std::size_t First = 0; First < Planes.size() && BestSquared > 0; ++First) {
			const std::array<double, 3> A = Planes[First];
			const double Offset = (A[0] * GX + A[1] * GY - A[2]) / (A[0] * A[0] + A[1] * A[1]);
			Consider(GX - A[0] * Offset, GY - A[1] * Offset);
			for(std::size_t Second = 0; Second < First; ++Second) {
				const std::array<double, 3> B = Planes[Second];
				const double Determinant = A[0] * B[1] - A[1] * B[0];
				if(!Determinant) continue;
				Consider((A[2] * B[1] - A[1] * B[2]) / Determinant, (A[0] * B[2] - A[2] * B[0]) / Determinant);
			}
		}
		const Vec3 Result = (East * BestX + North * BestY) * Scale;
		// 径方向を持たず，下流制約の範囲で元勾配へ最も近い接線の返戻
		return Result;
	}
}
