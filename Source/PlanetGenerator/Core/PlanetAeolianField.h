#pragma once

#include "PlanetClimate.h"
#include "PlanetErodedTerrain.h"
#include <array>
#include <memory>
#include <stop_token>

namespace PlanetCore {
	inline constexpr std::uint32_t AeolianFieldVersion = 5;

	/** 気候で選んだ乾燥陸へ有限支持の固定砂丘を加え，元の侵食地形を共有する */
	class PlanetAeolianField {
	private:
		static constexpr unsigned LongitudeCount = 64, LatitudeCount = 32;
		std::shared_ptr<const PlanetErodedTerrain> Base;
		std::array<double, LongitudeCount * LatitudeCount> DesertWeights{};
		std::uint64_t Seed = 0;
		double RadiusMeters = 0, SeaLevelMeters = 0;

		PlanetAeolianField() = default;
		double DesertWeight(UnitDirection Direction) const;
		template <bool Derivatives>
		TerrainSample EvaluateImpl(UnitDirection Direction) const;

	public:
		static constexpr double MaximumAddedHeightMeters = 18;

		/**
		 * 固定気候から砂丘適性を準備する
		 * @param BaseTerrain 侵食後の不変な標高場
		 * @param Climate 同じ生成定義から準備した不変な気候
		 * @param Settings 惑星の半径・海面・Seed
		 * @param Stop 中断要求
		 * @return 有効な不変場，準備失敗又は中断時は空
		 */
		static std::optional<PlanetAeolianField> Create(std::shared_ptr<const PlanetErodedTerrain> BaseTerrain,
			const PlanetClimate &Climate, const GeologySettings &Settings, std::stop_token Stop = {});

		/**
		 * 解析勾配と共に固定砂丘を評価する
		 * @param Direction 球面の評価方向
		 * @return 最終砂丘を加えた標高と接線勾配
		 */
		TerrainSample Evaluate(UnitDirection Direction) const;

		/**
		 * 勾配を省いて同じ標高を評価する
		 * @param Direction 球面の評価方向
		 * @return 最終砂丘を加えた標高
		 */
		double EvaluateElevation(UnitDirection Direction) const;
	};
}
