#pragma once
#include "PlanetCore.h"
#include <optional>

namespace PlanetCore {
	/** 参照球面上の短い弧を半径一定の近景通路で囲む */
	struct SurfaceCorridor {
		Vec3 StartDirection, EndDirection, Span;
		double SphereRadiusMeters = 0, TubeRadiusMeters = 0, SagittaMeters = 0, SpanSquared = 0;
		/**
		 * 球面上の短い通路を準備する
		 * @param StartMeters 身体側の惑星座標
		 * @param EndMeters 進行側の惑星座標
		 * @param SphereRadiusMeters 参照球半径
		 * @param TubeRadiusMeters 全方位の精細半径
		 * @return 有効な通路，不正な入力の場合は空
		 */
		static std::optional<SurfaceCorridor> Create(Vec3 StartMeters, Vec3 EndMeters,
			double SphereRadiusMeters, double TubeRadiusMeters);
		/**
		 * 単位方向から通路の弦までの距離を求める
		 * @param Direction 単位球面方向
		 * @return 参照球面上の弦への距離
		 */
		double DistanceMeters(Vec3 Direction) const;
		/**
		 * 安全余裕を残して方向が近景通路内か判定する
		 * @param Direction 単位球面方向
		 * @param SafetyMeters 通路境界に残す余裕
		 * @return 近景通路に含まれる時に真
		 */
		bool Contains(Vec3 Direction, double SafetyMeters = 0) const;
		/**
		 * タイルの包囲球が通路と交わるか保守的に判定する
		 * @param Center 単位球面上のタイル中心
		 * @param CapMeters タイルの包囲半径
		 * @return 交差する可能性がある時に真
		 */
		bool Intersects(Vec3 Center, double CapMeters) const;
	};
}
