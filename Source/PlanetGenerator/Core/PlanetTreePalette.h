#pragma once

#include "PlanetBiome.h"
#include <algorithm>
#include <cmath>
#include <optional>

namespace PlanetCore {
enum class TreeKind : std::uint8_t { Broadleaf, Conifer };

struct TreeSpeciesWeights {
	double Broadleaf = 0, Conifer = 0;
};

/**
 * 樹種別の重みを散布と季節材質で共有する
 * @param Habitat 地点のバイオーム混合と樹木適性
 * @return 広葉樹と針葉樹の重み
 */
inline TreeSpeciesWeights EvaluateTreeSpeciesWeights(const BiomeHabitat &Habitat) {
	const auto Weight = [&](const LandBiome Biome) { return Habitat.Weights[static_cast<unsigned>(Biome)]; };
	const double TemperateRainforest = Weight(LandBiome::TemperateRainforest);
	return {Weight(LandBiome::TropicalRainforest) + Weight(LandBiome::TropicalSeasonalForest) + Weight(LandBiome::Savanna) +
				Weight(LandBiome::TemperateDeciduousForest) + Weight(LandBiome::Mediterranean) + TemperateRainforest * 0.5,
			Weight(LandBiome::Taiga) + TemperateRainforest * 0.5};
}

/**
 * 樹木が生えない地点でも連続的な林床の針葉樹比率を算出する
 * @param Habitat 地点のバイオーム混合と樹木適性
 * @return ０以上１以下の針葉樹比率
 */
inline double ConiferFraction(const BiomeHabitat &Habitat) {
	const TreeSpeciesWeights Species = EvaluateTreeSpeciesWeights(Habitat);
	const double Total = Species.Broadleaf + Species.Conifer;
	return std::isfinite(Total) && Total > 0 && std::isfinite(Species.Conifer) ? std::clamp(Species.Conifer / Total, 0.0, 1.0)
																			   : 0.0;
}

/**
 * 視点と季節に依らない樹種を決定する
 * @param Habitat 地点のバイオーム混合と樹木適性
 * @param Selection 樹木の受入れを決める固定乱数
 * @param SpeciesSelection 受入れ後の樹種を決める固定乱数
 * @return 樹種，受入れない地点又は不正な入力では空
 */
inline std::optional<TreeKind> SelectTreeKind(const BiomeHabitat &Habitat, const double Selection,
											  const double SpeciesSelection) {
	if(!std::isfinite(Selection) || !std::isfinite(SpeciesSelection) || Selection < 0 || Selection >= 1 ||
	   SpeciesSelection < 0 || SpeciesSelection >= 1 || !std::isfinite(Habitat.TreeSuitability))
		return {};
	const TreeSpeciesWeights Species = EvaluateTreeSpeciesWeights(Habitat);
	const double Total = Species.Broadleaf + Species.Conifer;
	if(!std::isfinite(Total) || Total <= 0 || Selection >= std::clamp(Habitat.TreeSuitability * Total, 0.0, 1.0)) return {};
	// 樹種の乱数は樹木の受入れ後に適用し，森の境界でも候補セルを変えない
	return SpeciesSelection < Species.Conifer / Total ? TreeKind::Conifer : TreeKind::Broadleaf;
}
} // namespace PlanetCore
