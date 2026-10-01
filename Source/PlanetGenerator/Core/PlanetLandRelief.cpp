#include "PlanetLandRelief.h"
#include <cmath>
#include <cstdint>

namespace PlanetCore {
	namespace {
		/**
		 * 固定整数格子の頂点に用いる整数ビットを混合する
		 * @param Value 入力ビット
		 * @return 混合したビット
		 */
		std::uint64_t MixReliefBits(std::uint64_t Value) {
			Value ^= Value >> 30;
			Value *= UINT64_C(0XBF58476D1CE4E5B9);
			Value ^= Value >> 27;
			Value *= UINT64_C(0X94D049BB133111EB);
			// 固定したハッシュ値の返戻
			return Value ^ Value >> 31;
		}

		/**
		 * 固定 Seed と格子座標に対する－１～１の値を取得する
		 * @param X 格子の X 座標
		 * @param Y 格子の Y 座標
		 * @param Z 格子の Z 座標
		 * @param Seed 惑星の Seed から導いた値
		 * @return 格子頂点の値
		 */
		double ReliefCorner(const std::int64_t X, const std::int64_t Y, const std::int64_t Z, const std::uint64_t Seed) {
			const std::uint64_t Bits = MixReliefBits(
				Seed ^ static_cast<std::uint64_t>(X) * UINT64_C(0X9E3779B97F4A7C15) ^
				static_cast<std::uint64_t>(Y) * UINT64_C(0XBF58476D1CE4E5B9) ^ static_cast<std::uint64_t>(Z) * UINT64_C(0X94D049BB133111EB)
			);
			// 上位 53bit を有限な振幅へ写した値の返戻
			return static_cast<double>(Bits >> 11) * (1.0 / 4503599627370496.0) - 1;
		}
	}

	/**
	 * 惑星固定メートル座標の連続な格子起伏と解析勾配を評価する
	 * @param Direction 惑星中心からの単位方向
	 * @param Radius 惑星半径，単位は m
	 * @param Seed 惑星の Seed から導いた値
	 * @return 起伏の標高差と方向微分
	 */
	template<bool Derivatives> TerrainJet<Derivatives> SampleLandRelief(
		const Vec3 Direction,
		const double Radius,
		const std::uint64_t Seed
	) {
		using Jet = TerrainJet<Derivatives>;
		const Vec3 Coordinate = Direction * (Radius / LandReliefCellMeters);
		const std::int64_t X = static_cast<std::int64_t>(std::floor(Coordinate.X));
		const std::int64_t Y = static_cast<std::int64_t>(std::floor(Coordinate.Y));
		const std::int64_t Z = static_cast<std::int64_t>(std::floor(Coordinate.Z));
		const Vec3 Fraction { Coordinate.X - X, Coordinate.Y - Y, Coordinate.Z - Z };
		const auto Fade = [](const double T) {
			// 両端で二階迄接続する補間率の返戻
			return T * T * T * (10 + T * (-15 + 6 * T));
		};
		const auto Slope = [](const double T) {
			// 同じ補間率の一階微分の返戻
			return 30 * T * T * (1 - T) * (1 - T);
		};
		Vec3 GX, GY, GZ;
		if constexpr(Derivatives) {
			const double Scale = Radius / LandReliefCellMeters;
			GX = { Scale * Slope(Fraction.X), 0, 0 };
			GY = { 0, Scale * Slope(Fraction.Y), 0 };
			GZ = { 0, 0, Scale * Slope(Fraction.Z) };
		}
		const Jet FX { Fade(Fraction.X), GX }, FY { Fade(Fraction.Y), GY }, FZ { Fade(Fraction.Z), GZ };
		const Jet One { 1, {} };
		Jet Sum;
		for(std::int64_t z = 0; z < 2; ++z) for(std::int64_t y = 0; y < 2; ++y) for(std::int64_t x = 0; x < 2; ++x) {
			const Jet Weight = (x ? FX : One - FX) * (y ? FY : One - FY) * (z ? FZ : One - FZ);
			Sum = Sum + Weight * ReliefCorner(X + x, Y + y, Z + z, Seed);
		}
		// 最大５ m の緩やかな林床・平原起伏の返戻
		return Sum * LandReliefAmplitudeMeters;
	}

	template TerrainJet<true> SampleLandRelief<true>(const Vec3 Direction, const double Radius, const std::uint64_t Seed);
	template TerrainJet<false> SampleLandRelief<false>(const Vec3 Direction, const double Radius, const std::uint64_t Seed);
}
