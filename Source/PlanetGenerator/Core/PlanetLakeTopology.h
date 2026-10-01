#pragma once

#include "PlanetDepressionHierarchy.h"
#include <cstdint>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct LakeComplex {
		// 同じ水位で実際の通路が繋がる水域；零水深の連結も位相に含む
		double WaterMeters = 0;
		bool IsOcean = false;
		std::vector<std::uint32_t> Sites;
		std::vector<std::uint32_t> Basins;
		// 内部の同水位接続と，異なる水域へ下る実流出口の包含根番号
		std::vector<std::uint32_t> InternalSpillBasins;
		std::vector<std::uint32_t> OutletBasins;
	};

	/** 飽和時の水域と包含木を結ぶ位相；岸・有限幅の通水路・流量とは区別する */
	struct LakeTopology {
		// 大洋も同じ配列へ含め，元の領域番号と地点番号を保持
		std::vector<LakeComplex> Complexes;
		std::vector<std::uint32_t> SiteComplexes;
		std::vector<std::uint32_t> BasinComplexes;
		// 正の水深を持つ水域へ属さない根を，湖から明示的に分離
		std::vector<std::uint32_t> DryBasins;
	};

	std::optional<LakeTopology> BuildLakeTopology(
		const DepressionHierarchy &Hierarchy,
		const std::stop_token Stop= {}
	);
}
