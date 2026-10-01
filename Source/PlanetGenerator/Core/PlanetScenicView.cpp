#include "PlanetScenicView.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>

namespace PlanetCore {
	/**
	 * 身体の前景に隠れない山体と稜線の変化から，安全な着地点の景観順位を求める
	 * 計算量：O(1)，固定した５方位・１５距離の７５回の標高照会
	 * @param Anchor 身体の惑星方向
	 * @param Forward 初期視線の接線方向；半径方向の成分は射影して除く
	 * @param RadiusMeters 参照球半径，１０００≦ R ≦１０億，単位 m
	 * @param ElevationMeters 同じ元場の身体位置の標高，単位 m
	 * @param EyeHeightMeters 地表からの眼高，正の有限値，単位 m
	 * @param Height 同じ不変場の標高照会；水没部は一定海面０として評価
	 * @return ０〜１の景観順位と診断値；不正入力又は非有限な標高では空
	 */
	std::optional<ScenicView> EvaluateScenicView(
		const UnitDirection Anchor,
		const Vec3 Forward,
		const double RadiusMeters,
		const double ElevationMeters,
		const double EyeHeightMeters,
		const std::function<double(UnitDirection)> &Height
	) {
		const Vec3 Up = Anchor.Vector();
		const std::optional<UnitDirection> Tangent = UnitDirection::FromVector(Forward - Up * Forward.Dot(Up));
		const bool Valid = Height && Tangent && std::isfinite(RadiusMeters) && RadiusMeters >= 1000 && RadiusMeters <= 1E9 &&
		std::isfinite(ElevationMeters) && RadiusMeters + ElevationMeters > 0 && std::isfinite(EyeHeightMeters) && EyeHeightMeters > 0 &&
		std::isfinite(RadiusMeters + ElevationMeters + EyeHeightMeters);
		// 視線又は眼高を定義出来ない入力への返戻
		if(!Valid) return {};
		const Vec3 Right = Up.Cross(Tangent->Vector());
		const double Span = std::min(5000.0, RadiusMeters * 0.1), AngleScale = std::atan(0.1);
		constexpr std::array<double, 15> Fractions { 0.004, 0.01, 0.02, 0.04, 0.08, 0.14, 0.2, 0.3, 0.4, 0.6, 1, 1.5, 2, 3, 4 };
		double ExposureSum = 0, VisibleWeight = 0, LowestSkyline = std::numeric_limits<double>::infinity();
		double HighestSkyline = -LowestSkyline;
		ScenicView Result;
		for(int Ray = -2; Ray < 3; ++Ray) {
			const double Yaw = Ray * 0.26179938779914943654;
			const Vec3 Axis = Tangent->Vector() * std::cos(Yaw) + Right * std::sin(Yaw);
			double NearAngle = -1.57079632679489661923, FarAngle = NearAngle, FarReferenceAngle = 0, FarDistance = 0;
			for(const double Fraction : Fractions) {
				const double Distance = Span * Fraction, Length = std::hypot(RadiusMeters, Distance);
				const UnitDirection Direction = *UnitDirection::FromVector(Up * RadiusMeters + Axis * Distance);
				const double RawHeight = Height(Direction);
				++Result.HeightEvaluations;
				// 欠けた標高を平地として採点しない返戻
				if(!std::isfinite(RawHeight)) return {};
				const double H = std::max(0.0, RawHeight), Cosine = RadiusMeters / Length;
				const double Sagitta = RadiusMeters * Distance * Distance / (Length * (Length + RadiusMeters));
				const double Rise = H * Cosine - ElevationMeters - Sagitta - EyeHeightMeters, Along = Distance * ((RadiusMeters + H) / Length);
				const double Angle = std::atan2(Rise, Along);
				// 有限な標高でも座標変換が表現範囲を超えた場合の返戻
				if(!std::isfinite(Rise) || !std::isfinite(Along)) return {};
				if(Fraction <= 0.2) NearAngle = std::max(NearAngle, Angle);
				else if(Angle > FarAngle) {
					FarAngle = Angle;
					FarDistance = Along;
					// 一定標高の球の地平線を山体の露出として加点しない
					const double ReferenceRise = ElevationMeters * Cosine - ElevationMeters - Sagitta - EyeHeightMeters;
					const double ReferenceAlong = Distance * ((RadiusMeters + ElevationMeters) / Length);
					FarReferenceAngle = std::atan2(ReferenceRise, ReferenceAlong);
				}
			}
			const double Exposure = std::max(0.0, std::min(FarAngle - NearAngle, FarAngle - FarReferenceAngle)), Weight = Ray ? 1 : 2;
			ExposureSum += Weight * std::clamp(Exposure / AngleScale, 0.0, 1.0);
			if(Exposure > 0) VisibleWeight += Weight;
			LowestSkyline = std::min(LowestSkyline, FarAngle);
			HighestSkyline = std::max(HighestSkyline, FarAngle);
			Result.MaximumExposedRiseMeters = std::max(Result.MaximumExposedRiseMeters, std::sin(Exposure) * FarDistance);
			if(!Ray) {
				Result.CenterNearAngleRadians = NearAngle;
				Result.CenterFarAngleRadians = FarAngle;
			}
		}
		// 前景より高い実稜線の露出を主とし，同じ視野内の輪郭差を補助順位に使用
		const double Variation = std::clamp((HighestSkyline - LowestSkyline) / AngleScale, 0.0, 1.0);
		Result.Score = std::clamp((0.7 * ExposureSum + 0.3 * VisibleWeight * Variation) / 6, 0.0, 1.0);
		// 地形を変更せずに評価した有限標本の景観順位の返戻
		return Result;
	}
}
