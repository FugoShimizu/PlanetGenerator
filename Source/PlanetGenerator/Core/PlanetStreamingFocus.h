#pragma once
#include "PlanetCore.h"

namespace PlanetCore {
	/** 身体の周囲と進行方向の通路を同じ近景LODで覆う */
	struct StreamingFocus {
		static constexpr double BodyRadiusMeters = 24;
		static constexpr double MaximumLeadMeters = 36;
		Vec3 StartMeters, EndMeters;
		double RadiusMeters = BodyRadiusMeters;
	};

	/**
	 * 許容速度と生成遅延から，球面上の身体位置から進行方向へ延びる近景通路を求める
	 * @param PositionMeters 現在の身体の惑星座標
	 * @param VelocityMeters 惑星座標の秒速
	 * @param ForwardMeters 停止時に用いる身体の前方
	 * @param PlanetRadiusMeters 参照球の半径
	 * @param HorizonSeconds 観測した生成遅延を含む先読み秒数
	 * @param AllowedSpeedMetersPerSecond 設定で許可する最大接線速度
	 * @return 身体周囲を含む有限な通路，不正な入力の場合は空
	 */
	std::optional<StreamingFocus> PredictStreamingFocus(Vec3 PositionMeters, Vec3 VelocityMeters,
		Vec3 ForwardMeters, double PlanetRadiusMeters, double HorizonSeconds, double AllowedSpeedMetersPerSecond);
}
