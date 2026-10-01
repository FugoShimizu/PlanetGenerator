#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t ContinentalDiskWarpVersion = 1;

	struct ContinentalDiskTwist {
		double X = 0;
		double Y = 0;
		double Radius = 0.2;
		double AngleRadians = 0;
	};

	struct ContinentalDiskWarpSettings {
		std::vector<ContinentalDiskTwist> Twists;
	};

	struct ContinentalDiskWarpMapping {
		std::array<double, 2> Coordinates {};
		/** 行優先の第１・第２座標に依る解析微分 */
		std::array<double, 4> Derivatives {};
		std::array<double, 2> Pushforward(std::array<double, 2> Tangent) const;
		std::array<double, 2> Pullback(std::array<double, 2> Gradient) const;
	};

	/**
	 * 円板内で支持が完結する捩りを合成し，海岸の位置と内部極値の種類を保つ
	 * 山地形状を比較する座標変形であり，侵食又は自然な完成地形ではない
	 */
	class ContinentalDiskWarp {
	private:

		std::vector<ContinentalDiskTwist> Twists;
		std::uint64_t Hash = 0;

		ContinentalDiskWarp() = default;

		std::optional<ContinentalDiskWarpMapping> MapImpl(double X, double Y, bool IsInverse) const;

	public:

		static constexpr std::size_t MaximumTwists = 32;
		static std::optional<ContinentalDiskWarp> Create(const ContinentalDiskWarpSettings &Settings);
		std::optional<ContinentalDiskWarpMapping> MapToSource(double X, double Y) const;
		std::optional<ContinentalDiskWarpMapping> MapToWorld(double X, double Y) const;
		std::size_t GetTwistCount() const;
		std::uint64_t GetDefinitionHash() const;
	};
}
