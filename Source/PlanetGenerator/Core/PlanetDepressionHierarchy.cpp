#include "PlanetDepressionHierarchy.h"
#include <algorithm>
#include <cstdint>
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 通路上限の昇順で包含木を作り，大洋へ接続済の成分とは水位を共有しない
	 * Barnes 等の窪地階層を，途中の堰を持つ一般の網へ適用する
	 * 計算量 O((V + E) log(V + E))，作業領域 O(V + E)，V は地点数，E は辺数
	 * @param Sites 元標高と，海面以下で相互に接続する大洋の指定
	 * @param Edges 両端の標高以上の通路上限
	 * @param SeaLevelMeters 一定の海面
	 * @param Stop 中断要求
	 * @return 包含と溢流を分けた階層；不正入力，非連結又は中断時は空
	 */
	std::optional<DepressionHierarchy> BuildDepressionHierarchy(
		const std::span<const SpillGraphSite> Sites,
		const std::span<const SpillGraphEdge> Edges,
		const double SeaLevelMeters,
		const std::stop_token Stop
	) {
		std::optional<SpillGraph> Saturation = BuildSpillGraph(Sites, Edges, SeaLevelMeters, Stop);
		// 不正な入力・大洋の非連結・中断時の返戻
		if(!Saturation) return {};
		// 互いに素な成分と，成分が現在表す包含木
		const std::uint32_t Count = static_cast<std::uint32_t>(Sites.size());
		std::vector < std::uint32_t > Parents(Count), Sizes(Count, 1), Tops(Count), Order(Edges.size());
		std::iota(Parents.begin(), Parents.end(), 0U);
		std::iota(Order.begin(), Order.end(), 0U);
		// 成分代表の探索と経路圧縮
		const auto Find = [&Parents](std::uint32_t Index) -> std::uint32_t {
			while(Parents[Index] != Index) {
				Parents[Index] = Parents[Parents[Index]];
				Index = Parents[Index];
			}
			// 圧縮した成分代表の返戻
			return Index;
		};
		// 大洋の０番への集約と陸上地点の葉の用意
		DepressionHierarchy Result;
		Result.Saturation = std::move(*Saturation);
		Result.Nodes.reserve(2 * Count);
		Result.Nodes.emplace_back();
		Result.Nodes[0].MinimumMeters = Result.Nodes[0].SpillMeters = SeaLevelMeters;
		Result.Nodes[0].OceanLinkRank = 0;
		Result.SiteLeaves.resize(Count);
		Result.SiteRoots.resize(Count);
		// 連結済の大洋代表と溢流の確定順
		std::uint32_t OceanRepresentative = MissingSpillIndex, LinkRank = 0;
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 中断時の返戻
			if(Stop.stop_requested()) return {};
			if(Sites[i].IsOcean) {
				Result.Nodes[0].MinimumMeters = std::min(Result.Nodes[0].MinimumMeters, Sites[i].ElevationMeters);
				if(OceanRepresentative == MissingSpillIndex) OceanRepresentative = i;
				else {
					Parents[i] = OceanRepresentative;
					++Sizes[OceanRepresentative];
				}
				continue;
			}
			Tops[i] = Result.SiteLeaves[i] = static_cast<std::uint32_t>(Result.Nodes.size());
			// 元標高を保持する葉
			DepressionHierarchyNode Leaf;
			Leaf.Site = i;
			Leaf.MinimumMeters = Sites[i].ElevationMeters;
			Result.Nodes.push_back(Leaf);
		}
		// 通路の向きと列挙順から独立した整列キー
		const auto Key = [&](const std::uint32_t Index) -> std::tuple<double, double, std::uint32_t, std::uint32_t> {
			// 海面以下の通路は同じ水位で処理し，実際の上限で同順位を区別する
			return {
				std::max(SeaLevelMeters, Edges[Index].CrestMeters),
				Edges[Index].CrestMeters,
				std::min(Edges[Index].Nodes[0], Edges[Index].Nodes[1]),
				std::max(Edges[Index].Nodes[0], Edges[Index].Nodes[1])
			};
		};
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::uint32_t A, const std::uint32_t B) -> bool {
				// 並行する同一通路の番号だけは元配列の順序を用いる
				return Key(A) != Key(B) ? Key(A) < Key(B) : A < B;
			}
		);
		for(const std::uint32_t EdgeIndex : Order) {
			// 中断時の返戻
			if(Stop.stop_requested()) return {};
			// 端点の向きを固定した現在の成分
			const SpillGraphEdge &Edge = Edges[EdgeIndex];
			const std::uint32_t A = std::min(Edge.Nodes[0], Edge.Nodes[1]), B = std::max(Edge.Nodes[0], Edge.Nodes[1]);
			std::uint32_t RootA = Find(A), RootB = Find(B);
			if(RootA == RootB) continue;
			// 接続前の包含木と，今回の合流水位
			const std::uint32_t TopA = Tops[RootA], TopB = Tops[RootB];
			const double Level = std::max(SeaLevelMeters, Edge.CrestMeters);
			std::uint32_t MergedTop = 0;
			if(!TopA || !TopB) {
				// 包含親とは区別する，下流の実地点への接続
				DepressionHierarchyNode &Upstream = Result.Nodes[TopA ? TopA : TopB];
				Upstream.SpillMeters = Level;
				Upstream.OutletSite = TopA ? A : B;
				Upstream.ReceivingSite = TopA ? B : A;
				Upstream.OutletEdge = EdgeIndex;
				Upstream.OceanLinkRank = ++LinkRank;
			} else {
				// 同じ水域へ合体する２成分だけを包含する親
				DepressionHierarchyNode Parent;
				Parent.Children = { TopA, TopB };
				Parent.MinimumMeters = std::min(Result.Nodes[TopA].MinimumMeters, Result.Nodes[TopB].MinimumMeters);
				MergedTop = static_cast<std::uint32_t>(Result.Nodes.size());
				Result.Nodes.push_back(Parent);
				Result.Nodes[TopA].Parent = Result.Nodes[TopB].Parent = MergedTop;
				Result.Nodes[TopA].SpillMeters = Result.Nodes[TopB].SpillMeters = Level;
				Result.Nodes[TopA].OutletSite = Result.Nodes[TopB].ReceivingSite = A;
				Result.Nodes[TopA].ReceivingSite = Result.Nodes[TopB].OutletSite = B;
				Result.Nodes[TopA].OutletEdge = Result.Nodes[TopB].OutletEdge = EdgeIndex;
			}
			// 成分の大きさに依る併合と，包含木への対応の更新
			if(Sizes[RootA] < Sizes[RootB]) std::swap(RootA, RootB);
			Parents[RootB] = RootA;
			Sizes[RootA] += Sizes[RootB];
			Tops[RootA] = MergedTop;
		}
		// 親が後方に在る順序を用いた，飽和時の所属水域の逆順計算
		std::vector<std::uint32_t> Roots(Result.Nodes.size());
		for(std::uint32_t End = static_cast<std::uint32_t>(Result.Nodes.size()); End; --End) {
			// 中断時の返戻
			if(Stop.stop_requested()) return {};
			// 先に処理済の親，又は下流へ溢れる木の根
			const std::uint32_t Index = End - 1, Parent = Result.Nodes[Index].Parent;
			Roots[Index] = Parent == MissingSpillIndex ? Index : Roots[Parent];
		}
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 中断時の返戻
			if(Stop.stop_requested()) return {};
			Result.SiteRoots[i] = Roots[Result.SiteLeaves[i]];
		}
		// 流量・連続湖岸・水体積とは分離した位相情報の返戻
		return Result;
	}
}
