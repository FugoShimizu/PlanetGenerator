#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	// 未所属・未定義の番号
	inline constexpr std::uint32_t MissingSpillIndex = std::numeric_limits<std::uint32_t>::max();

	struct SpillGraphSite {
		// 変更しない元標高と，接続を検査する大洋の指定
		double ElevationMeters = 0;
		bool IsOcean = false;
	};

	struct SpillGraphEdge {
		// 球面又は局所網の接続と，端点間の通路全体の最高標高
		std::array<std::uint32_t, 2> Nodes {};
		double CrestMeters = 0;
	};

	struct SpillGraphNode {
		// 大洋へ溢れる為の最小水位と，循環しない出口の証拠
		double EscapeMeters = 0;
		std::uint32_t Downstream = MissingSpillIndex;
		std::uint32_t DownstreamEdge = MissingSpillIndex;
		std::uint32_t SettlementRank = MissingSpillIndex;
		std::uint32_t Region = MissingSpillIndex;
	};

	struct SpillGraphRegion {
		// 同じ水位で連結する水域と，一つの下流側流出口
		double WaterMeters = 0;
		std::uint32_t OutletNode = MissingSpillIndex;
		std::uint32_t OutletNeighbor = MissingSpillIndex;
		std::uint32_t OutletEdge = MissingSpillIndex;
		std::uint32_t WetNodes = 0;
		std::uint32_t ShoreNodes = 0;
		bool IsOcean = false;
	};

	/** 静的な飽和限界の診断結果であり，河道水深・流量・連続水面の完成形ではない */
	struct SpillGraph {
		// 元標高を保存又は補間しない排水結果
		std::vector<SpillGraphNode> Nodes;
		std::vector<SpillGraphRegion> Regions;
		std::uint32_t DryFlatEdges = 0;
	};

	// 不変な標高と通路の上限を受取る準備入口
	std::optional<SpillGraph> BuildSpillGraph(
		const std::span<const SpillGraphSite> Sites,
		const std::span<const SpillGraphEdge> Edges,
		const double SeaLevelMeters,
		const std::stop_token Stop={}
	);
}
