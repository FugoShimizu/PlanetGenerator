#pragma once
#include "PlanetJet.h"
#include <cstdint>

namespace PlanetCore {
	inline constexpr double LandReliefCellMeters = 400;
	inline constexpr double LandReliefAmplitudeMeters = 5;

	/**
	 * 惑星固定メートル座標の連続な起伏を同じ計算順で評価する
	 * @param Direction 惑星中心からの単位方向
	 * @param Radius 惑星半径，単位は m
	 * @param Seed 惑星の Seed から導いた値
	 * @return 起伏の標高差と，要求時の三次元方向微分
	 */
	template<bool Derivatives> TerrainJet<Derivatives> SampleLandRelief(
		const Vec3 Direction,
		const double Radius,
		const std::uint64_t Seed
	);

	extern template TerrainJet<true> SampleLandRelief<true>(const Vec3 Direction, const double Radius, const std::uint64_t Seed);
	extern template TerrainJet<false> SampleLandRelief<false>(const Vec3 Direction, const double Radius, const std::uint64_t Seed);
}
