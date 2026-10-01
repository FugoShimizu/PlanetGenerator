#include "PlanetLakeTopology.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 検証済の飽和領域を包含根へ対応させ，同水位の湖内接続と外部への流出を区別する
	 * @param Hierarchy 同じ入力から準備した包含木と飽和結果
	 * @param Stop 地点・根の走査に伝播する中断要求
	 * @return 全地点・根の対応と実流出口；対応不整合又は中断時は空
	 * @note 計算量と作業領域は O(N+B+L)，N は地点数，B は包含節点数，L は水域数
	 */
	std::optional<LakeTopology> BuildLakeTopology(const DepressionHierarchy &Hierarchy, const std::stop_token Stop) {
		const SpillGraph &Saturation = Hierarchy.Saturation;
		const std::size_t SiteCount = Hierarchy.SiteRoots.size(), BasinCount = Hierarchy.Nodes.size();
		const bool Valid =
		SiteCount && SiteCount < 1048577 && BasinCount && BasinCount <= 2 * SiteCount && Hierarchy.SiteLeaves.size() == SiteCount &&
		Saturation.Nodes.size() == SiteCount && !Saturation.Regions.empty() && Saturation.Regions.size() <= SiteCount;
		// 未準備又は異なる入力を組合わせた状態への返戻
		if(!Valid || Stop.stop_requested()) return {};
		LakeTopology Result;
		Result.Complexes.resize(Saturation.Regions.size());
		Result.SiteComplexes.resize(SiteCount, MissingSpillIndex);
		Result.BasinComplexes.resize(BasinCount, MissingSpillIndex);
		std::vector<bool> Seen(BasinCount, false);
		std::uint32_t OceanCount = 0;
		for(std::size_t Index = 0; Index < Saturation.Regions.size(); ++Index) {
			const SpillGraphRegion &Region = Saturation.Regions[Index];
			// 不正な水位又は乾いた面の水域指定への返戻
			if(!std::isfinite(Region.WaterMeters) || !Region.IsOcean && !Region.WetNodes || Stop.stop_requested()) return {};
			LakeComplex &Complex = Result.Complexes[Index];
			Complex.WaterMeters = Region.WaterMeters;
			Complex.IsOcean = Region.IsOcean;
			if(Region.IsOcean) ++OceanCount;
		}
		// 全水域が一つの大洋へ流れる前提を失った入力への返戻
		if(OceanCount != 1) return {};
		for(std::uint32_t Site = 0; Site < SiteCount; ++Site) {
			const std::uint32_t Root = Hierarchy.SiteRoots[Site], Region = Saturation.Nodes[Site].Region;
			// 範囲外の対応又は中断への返戻
			if(Root >= BasinCount || Region != MissingSpillIndex && Region >= Result.Complexes.size() || Stop.stop_requested()) return {};
			const DepressionHierarchyNode &Basin = Hierarchy.Nodes[Root];
			const bool HasRoot =
			Basin.Parent == MissingSpillIndex && Basin.OceanLinkRank != MissingSpillIndex && std::isfinite(Basin.SpillMeters) &&
			Basin.SpillMeters == Saturation.Nodes[Site].EscapeMeters;
			// 別の水位又は包含子を根として流用した入力への返戻
			if(!HasRoot || Seen[Root] && Result.BasinComplexes[Root] != Region) return {};
			Result.SiteComplexes[Site] = Region;
			if(Region != MissingSpillIndex) {
				LakeComplex &Complex = Result.Complexes[Region];
				// 同じ番号へ別水位の地点を混入させた場合の返戻
				if(Basin.SpillMeters != Complex.WaterMeters || !Root && !Complex.IsOcean) return {};
				Complex.Sites.push_back(Site);
				if(!Seen[Root]) Complex.Basins.push_back(Root);
			} else if(!Seen[Root]) Result.DryBasins.push_back(Root);
			Result.BasinComplexes[Root] = Region;
			Seen[Root] = true;
		}
		// 大洋が欠落，又は乾いた領域へ対応する入力への返戻
		if(!Seen[0] || Result.BasinComplexes[0] == MissingSpillIndex || Hierarchy.Nodes[0].OceanLinkRank) return {};
		for(std::uint32_t Root = 1; Root < BasinCount; ++Root) {
			const DepressionHierarchyNode &Basin = Hierarchy.Nodes[Root];
			if(Basin.Parent != MissingSpillIndex) continue;
			const bool HasOutlet = Seen[Root] && Basin.OutletSite < SiteCount && Basin.ReceivingSite < SiteCount;
			// 未対応の根又は流出先を失った入力への返戻
			if(!HasOutlet || Stop.stop_requested()) return {};
			const std::uint32_t Receiver = Hierarchy.SiteRoots[Basin.ReceivingSite];
			const DepressionHierarchyNode &Receiving = Hierarchy.Nodes[Receiver];
			const bool Descends = Receiving.OceanLinkRank < Basin.OceanLinkRank && Receiving.SpillMeters <= Basin.SpillMeters &&
			Hierarchy.SiteRoots[Basin.OutletSite] == Root;
			// 循環，水位の上昇，又は別水域から流出する偽の根への返戻
			if(!Descends) return {};
			const std::uint32_t Region = Result.BasinComplexes[Root], Downstream = Result.BasinComplexes[Receiver];
			if(Region == MissingSpillIndex) continue;
			LakeComplex &Complex = Result.Complexes[Region];
			if(Region == Downstream) Complex.InternalSpillBasins.push_back(Root);
			else {
				// 飽和時の同水位接続が別水域へ分離した不整合，又は大洋からの流出への返戻
				if(!(Receiving.SpillMeters < Basin.SpillMeters) || Complex.IsOcean) return {};
				Complex.OutletBasins.push_back(Root);
			}
		}
		for(const LakeComplex &Complex : Result.Complexes) {
			// 水域全体の所属と，湖から下流へ出る流出口の欠落への返戻
			if(Complex.Sites.empty() || !Complex.IsOcean && Complex.OutletBasins.empty() || Stop.stop_requested()) return {};
		}
		// 元場と既存の包含関係を変更しない派生位相の返戻
		return Result;
	}
}
