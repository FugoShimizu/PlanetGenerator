#pragma once

#include "PlanetSpillGraph.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct DepressionHierarchyNode {
		// 子の包含関係と，葉に対応する入力地点
		std::uint32_t Parent = MissingSpillIndex;
		std::array<std::uint32_t, 2> Children { MissingSpillIndex, MissingSpillIndex };
		std::uint32_t Site = MissingSpillIndex;
		// 元標高の最低値と，親への合体又は下流水域への溢流位
		double MinimumMeters = 0;
		double SpillMeters = 0;
		// 流出口の内側・外側の実地点と，其の間の通路
		std::uint32_t OutletSite = MissingSpillIndex;
		std::uint32_t ReceivingSite = MissingSpillIndex;
		std::uint32_t OutletEdge = MissingSpillIndex;
		// 包含されない下流水域への接続順；大洋は０，包含子は未定義
		std::uint32_t OceanLinkRank = MissingSpillIndex;
	};

	/** 葉は網の地点であり，正の水深を持つ湖や連続的な極小の個数ではない */
	struct DepressionHierarchy {
		// ０番の大洋，親が子より後に並ぶ包含木と，地点からの対応
		std::vector<DepressionHierarchyNode> Nodes;
		std::vector<std::uint32_t> SiteLeaves;
		std::vector<std::uint32_t> SiteRoots;
		// 同じ入力で確認済の飽和水位と，同水位で連結する領域
		SpillGraph Saturation;
	};

	// 標高を変更せず，包含木と低い水域への接続を分離する
	std::optional<DepressionHierarchy> BuildDepressionHierarchy(
		const std::span<const SpillGraphSite> Sites,
		const std::span<const SpillGraphEdge> Edges,
		const double SeaLevelMeters,
		const std::stop_token Stop= {}
	);
}
