#include "PlanetSpillGraph.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 元標高を変更せず，通路の最高標高を越えて大洋へ溢れる最小水位を求める
	 * 計算量：O((V + E) log(V + E))，作業領域 O(V + E)，V は節点数，E は辺数
	 * @param Sites 有限な元標高と大洋指定，最大１０４８５７６点
	 * @param Edges 通路の最高標高を持つ無向辺，端点標高以上，最大で節点数の８倍
	 * @param SeaLevelMeters 接続した大洋の一定水位
	 * @param Stop 中断要求
	 * @return 網上の静水位・流出口，不正入力・大洋又は全網の非接続・中断時は空
	 */
	std::optional<SpillGraph> BuildSpillGraph(
		const std::span<const SpillGraphSite> Sites,
		const std::span<const SpillGraphEdge> Edges,
		const double SeaLevelMeters,
		const std::stop_token Stop
	) {
		// 有限な入力規模の検査
		const bool HasValidSize = !Sites.empty() && Sites.size() < 1048577 && Edges.size() <= Sites.size() * 8;
		// 準備を開始出来ない場合の返戻
		if(!HasValidSize || !std::isfinite(SeaLevelMeters) || Stop.stop_requested()) return {};
		// 接続の作業領域
		const std::uint32_t Count = static_cast<std::uint32_t>(Sites.size());
		struct Link {
			// 隣接節点と入力の通路番号
			std::uint32_t Neighbor;
			std::uint32_t Edge;
		};
		// 隣接表と大洋の個数
		std::vector<std::vector<Link>> Neighbors(Count);
		std::uint32_t OceanRoot = MissingSpillIndex, OceanCount = 0;
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 標高と大洋指定の検査
			const bool HasValidSite =
			std::isfinite(Sites[i].ElevationMeters) && (!Sites[i].IsOcean || Sites[i].ElevationMeters <= SeaLevelMeters);
			// 不正な地点又は中断時の返戻
			if(!HasValidSite || Stop.stop_requested()) return {};
			if(Sites[i].IsOcean) {
				if(OceanRoot == MissingSpillIndex) OceanRoot = i;
				++OceanCount;
			}
		}
		// 大洋の無い入力の返戻
		if(OceanRoot == MissingSpillIndex) return {};
		for(std::uint32_t i = 0; i < Edges.size(); ++i) {
			// 通路の端点と標高上限の検査
			const SpillGraphEdge &Edge = Edges[i];
			const std::uint32_t A = Edge.Nodes[0], B = Edge.Nodes[1];
			const bool HasValidEdge = A < Count && B < Count && A != B && std::isfinite(Edge.CrestMeters) &&
			Edge.CrestMeters >= std::max(Sites[A].ElevationMeters, Sites[B].ElevationMeters);
			// 範囲外又は元地形と矛盾する辺の返戻
			if(!HasValidEdge || Stop.stop_requested()) return {};
			Neighbors[A].push_back({ B, i });
			Neighbors[B].push_back({ A, i });
		}
		// 大洋指定の接続検査
		std::vector<bool> Seen(Count, false);
		std::vector<std::uint32_t> Queue { OceanRoot };
		Seen[OceanRoot] = true;
		for(std::size_t i = 0; i < Queue.size(); ++i) {
			// 中断時の返戻
			if(Stop.stop_requested()) return {};
			for(const Link Adjacent : Neighbors[Queue[i]]) {
				if(Seen[Adjacent.Neighbor] || !Sites[Adjacent.Neighbor].IsOcean || Edges[Adjacent.Edge].CrestMeters > SeaLevelMeters) continue;
				Seen[Adjacent.Neighbor] = true;
				Queue.push_back(Adjacent.Neighbor);
			}
		}
		// 離れた内陸水域を大洋として許可しない返戻
		if(Queue.size() != OceanCount) return {};
		// 最小最大路に依る水位の準備
		SpillGraph Result;
		Result.Nodes.resize(Count);
		using Entry = std::pair<double, std::uint32_t>;
		// 未確定の水位候補
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
		for(std::uint32_t i = 0; i < Count; ++i) {
			Result.Nodes[i].EscapeMeters = Sites[i].IsOcean ? SeaLevelMeters : std::numeric_limits<double>::infinity();
			if(Sites[i].IsOcean) {
				Result.Nodes[i].Downstream = i;
				Frontier.emplace(SeaLevelMeters, i);
			}
		}
		std::fill(Seen.begin(), Seen.end(), false);
		// 下流より必ず後に成る確定順位
		std::uint32_t Rank = 0;
		while(!Frontier.empty()) {
			// 中断時の返戻
			if(Stop.stop_requested()) return {};
			// 最小の候補水位
			const Entry Current = Frontier.top();
			Frontier.pop();
			// 候補の節点番号
			const std::uint32_t Index = Current.second;
			if(Seen[Index] || Current.first != Result.Nodes[Index].EscapeMeters) continue;
			Seen[Index] = true;
			Result.Nodes[Index].SettlementRank = Rank++;
			for(const Link Adjacent : Neighbors[Index]) {
				// 隣接側へ伝える水位候補
				SpillGraphNode &Next = Result.Nodes[Adjacent.Neighbor];
				const double Escape = std::max(Current.first, Edges[Adjacent.Edge].CrestMeters);
				if(Seen[Adjacent.Neighbor] || Escape >= Next.EscapeMeters) continue;
				Next.EscapeMeters = Escape;
				Next.Downstream = Index;
				Next.DownstreamEdge = Adjacent.Edge;
				Frontier.emplace(Escape, Adjacent.Neighbor);
			}
		}
		// 大洋へ到達しない成分が残る場合の返戻
		if(Rank != Count) return {};
		// 同一水位で繋がる湖と岸の抽出
		std::fill(Seen.begin(), Seen.end(), false);
		for(std::uint32_t Start = 0; Start < Count; ++Start) {
			if(Seen[Start]) continue;
			Queue.clear();
			Queue.push_back(Start);
			Seen[Start] = true;
			// 同一水位の領域
			SpillGraphRegion Region;
			Region.WaterMeters = Result.Nodes[Start].EscapeMeters;
			for(std::size_t i = 0; i < Queue.size(); ++i) {
				// 中断時の返戻
				if(Stop.stop_requested()) return {};
				// 領域内の地点
				const std::uint32_t Index = Queue[i];
				Region.IsOcean = Region.IsOcean || Sites[Index].IsOcean;
				if(Sites[Index].ElevationMeters < Region.WaterMeters) ++Region.WetNodes;
				else ++Region.ShoreNodes;
				if(
					Region.OutletNode == MissingSpillIndex || Result.Nodes[Index].SettlementRank < Result.Nodes[Region.OutletNode].SettlementRank
				) Region.OutletNode = Index;
				for(const Link Adjacent : Neighbors[Index]) {
					if(
						Seen[Adjacent.Neighbor] || Edges[Adjacent.Edge].CrestMeters > Region.WaterMeters ||
						Result.Nodes[Adjacent.Neighbor].EscapeMeters != Region.WaterMeters
					) continue;
					Seen[Adjacent.Neighbor] = true;
					Queue.push_back(Adjacent.Neighbor);
				}
			}
			// 水深の無い平坦面を湖に見せない処理
			if(!Region.IsOcean && !Region.WetNodes) continue;
			// 領域の流出口
			const SpillGraphNode &Outlet = Result.Nodes[Region.OutletNode];
			Region.OutletNeighbor = Outlet.Downstream;
			Region.OutletEdge = Outlet.DownstreamEdge;
			// 公開する領域番号
			const std::uint32_t RegionIndex = static_cast<std::uint32_t>(Result.Regions.size());
			for(const std::uint32_t Index : Queue) Result.Nodes[Index].Region = RegionIndex;
			Result.Regions.push_back(Region);
		}
		// 河道の正勾配を別途必要とする乾いた平坦辺の計数
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 乾いた辺の判定対象
			const SpillGraphNode &Node = Result.Nodes[i];
			if(
				Node.Region == MissingSpillIndex && i != Node.Downstream && Node.EscapeMeters == Result.Nodes[Node.Downstream].EscapeMeters
			) ++Result.DryFlatEdges;
		}
		// 元標高から分離した静的な排水診断の返戻
		return Result;
	}
}
