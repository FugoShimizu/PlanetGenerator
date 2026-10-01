#include "PlanetDrainageFit.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace PlanetCore {
	/**
	 * 親辺の実長を海までの距離差へ変換し，既存の共有木回帰で節点標高を決める
	 * この離散節点契約だけでは，三角面内部の窪地又は滑らかな補間の排水は証明しない
	 * @param Sites 陸上節点と大洋までの実距離，目標標高，費用重み
	 * @param SeaLevelMeters 固定した球状海面の基準標高
	 * @param MinimumSlope 各下流辺に要求する正の落差率
	 * @param Stop 中断要求
	 * @return 厳密な下流落差を持つ節点標高，不正な木又は丸めで保証不能なら空
	 */
	std::optional<DrainageFitResult> FitDrainageTerrain(
		const std::span<const DrainageTerrainSite> Sites,
		const double SeaLevelMeters,
		const double MinimumSlope,
		const std::stop_token Stop
	) {
		// 空入力，非有限な基準面，非正の落差率，中断の返戻
		if(
			Sites.empty() || !std::isfinite(SeaLevelMeters) || !std::isfinite(MinimumSlope) || !(MinimumSlope > 0) || Stop.stop_requested()
		) return {};
		std::vector<DrainageFitSite> Shifted;
		Shifted.reserve(Sites.size());
		for(std::size_t Index = 0; Index < Sites.size(); ++Index) {
			// 距離の前処理中に中断した場合の返戻
			if(!(Index % 512) && Stop.stop_requested()) return {};
			const DrainageTerrainSite &Site = Sites[Index];
			// 海への距離が増えない辺と非有限な尺度を拒否する返戻
			if(
				!std::isfinite(Site.DistanceToOceanMeters) || !(Site.DistanceToOceanMeters > 0) || Site.Downstream != NoDrainageFitDownstream &&
				(Site.Downstream >= Sites.size() || !(Site.DistanceToOceanMeters > Sites[Site.Downstream].DistanceToOceanMeters))
			) return {};
			const double Offset = MinimumSlope * Site.DistanceToOceanMeters, Target = Site.TargetMeters - Offset;
			// 距離に応じた変換が表現範囲を超える入力の返戻
			if(!std::isfinite(Offset) || !std::isfinite(Target)) return {};
			Shifted.push_back({ Site.Downstream, Target, Site.Weight });
		}
		std::optional<DrainageFitResult> Result = FitDrainageTree(Shifted, SeaLevelMeters, Stop);
		// 不正な木又は中断した回帰の返戻
		if(!Result) return {};
		std::vector<std::size_t> Order(Sites.size());
		std::iota(Order.begin(), Order.end(), 0);
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::size_t A, const std::size_t B) {
				const double X = Sites[A].DistanceToOceanMeters, Y = Sites[B].DistanceToOceanMeters;
				// 同じ海距離の別流域も地点番号で安定に並べる返戻
				return X != Y ? X < Y : A < B;
			}
		);
		// 親を先に処理する並べ替え中に中断された結果の返戻
		if(Stop.stop_requested()) return {};
		Result->WeightedSquaredChange = 0;
		std::vector<double> HighestAncestorTarget(Sites.size());
		std::vector<std::uint32_t> HighestAncestorSite(Sites.size());
		for(const std::size_t Index : Order) {
			// 物理標高と実費用の再構成中に中断した場合の返戻
			if(Stop.stop_requested()) return {};
			const DrainageTerrainSite &Site = Sites[Index];
			const std::uint32_t Downstream = Site.Downstream;
			const double Parent = Downstream == NoDrainageFitDownstream ? SeaLevelMeters : Result->Levels[Downstream];
			const double ParentDistance = Downstream == NoDrainageFitDownstream ? 0 : Sites[Downstream].DistanceToOceanMeters;
			// 海面と祖先からの二つの必要条件は，同じ木の最大変更量の実数上の下限を与える
			const double Target = Shifted[Index].TargetMeters, OceanBound = std::max(0.0, SeaLevelMeters - Target);
			const double PairBound =
			Downstream == NoDrainageFitDownstream ? 0 : std::max(0.0, 0.5 * (HighestAncestorTarget[Downstream] - Target));
			// 境界差が表現範囲を超えた場合の返戻
			if(!std::isfinite(OceanBound) || !std::isfinite(PairBound)) return {};
			if(OceanBound > Result->MinimumTheoreticalMaxChangeMeters) {
				Result->MinimumTheoreticalMaxChangeMeters = OceanBound;
				Result->CriticalAncestor = NoDrainageFitDownstream;
				Result->CriticalSite = static_cast<std::uint32_t>(Index);
			}
			if(PairBound > Result->MinimumTheoreticalMaxChangeMeters) {
				Result->MinimumTheoreticalMaxChangeMeters = PairBound;
				Result->CriticalAncestor = HighestAncestorSite[Downstream];
				Result->CriticalSite = static_cast<std::uint32_t>(Index);
			}
			HighestAncestorTarget[Index] = Target;
			HighestAncestorSite[Index] = static_cast<std::uint32_t>(Index);
			if(Downstream != NoDrainageFitDownstream && HighestAncestorTarget[Downstream] > Target) {
				HighestAncestorTarget[Index] = HighestAncestorTarget[Downstream];
				HighestAncestorSite[Index] = HighestAncestorSite[Downstream];
			}
			const double MinimumDrop = MinimumSlope * (Site.DistanceToOceanMeters - ParentDistance), Required = Parent + MinimumDrop;
			const double Unrounded = Result->Levels[Index] + MinimumSlope * Site.DistanceToOceanMeters;
			// 必要な落差が非有限又は零へ丸められた場合の返戻
			if(!std::isfinite(MinimumDrop) || !(MinimumDrop > 0) || !std::isfinite(Required) || !std::isfinite(Unrounded)) return {};
			double Level = std::max(Unrounded, Required);
			if(!(Level > Parent)) Level = std::nextafter(Parent, std::numeric_limits<double>::infinity());
			// 上側丸めでも最低勾配又は有限な標高を保てない場合の返戻
			if(!std::isfinite(Level) || !(Level > Parent) || Level < Required) return {};
			Result->Levels[Index] = Level;
			const double Change = Level - Site.TargetMeters;
			Result->WeightedSquaredChange += Site.Weight * Change * Change;
			// 物理標高での費用が非有限になる場合の返戻
			if(!std::isfinite(Result->WeightedSquaredChange)) return {};
		}
		// 全ての節点と下流辺で厳密下降を確認した返戻
		return Result;
	}

	/**
	 * 合流を共有する排水木で，元標高の重み付き二乗変更を最小化する
	 * 計算量：O(n log² n)，記憶量：O(n)，n は陸上標本数
	 * 小さい二分ヒープを大きい物へ移し，部分木の凸費用微分を統合する
	 * @param Sites 陸上地点の目標値，正の重み，下流参照，固定海口への参照は番兵値
	 * @param LowerMeters 固定海口の変換後水位，全地点の共通下限
	 * @param Stop 中断要求
	 * @return 下流以下へ逆転しない共有値，不正値，循環，中断，非有限な集計では空
	 */
	std::optional<DrainageFitResult> FitDrainageTree(
		const std::span<const DrainageFitSite> Sites,
		const double LowerMeters,
		const std::stop_token Stop
	) {
		// 空又は地点番号で表せない入力を拒否する返戻
		if(Sites.empty() || Sites.size() > NoDrainageFitDownstream || !std::isfinite(LowerMeters) || Stop.stop_requested()) return {};
		const std::uint32_t Count = static_cast<std::uint32_t>(Sites.size());
		std::vector<std::uint32_t> Children(Count), Order;
		Order.reserve(Count);
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			// 入力走査中の中断時の返戻
			if(!(Index % 512) && Stop.stop_requested()) return {};
			const DrainageFitSite &Site = Sites[Index];
			// 無効な費用や下流参照を拒否する返戻
			if(
				!std::isfinite(Site.TargetMeters) || !std::isfinite(Site.Weight) || !(Site.Weight > 0) || Site.Downstream == Index ||
				Site.Downstream != NoDrainageFitDownstream && Site.Downstream >= Count
			) return {};
			if(Site.Downstream != NoDrainageFitDownstream) ++Children[Site.Downstream];
		}
		for(std::uint32_t Index = 0; Index < Count; ++Index) if(!Children[Index]) Order.push_back(Index);
		for(std::size_t Position = 0; Position < Order.size(); ++Position) {
			// 接続検査中の中断時の返戻
			if(!(Position % 512) && Stop.stop_requested()) return {};
			const std::uint32_t Downstream = Sites[Order[Position]].Downstream;
			if(Downstream != NoDrainageFitDownstream && !--Children[Downstream]) Order.push_back(Downstream);
		}
		// 循環を含む接続を排水木として扱わない返戻
		if(Order.size() != Sites.size()) return {};
		struct Hinge {
			double Level = 0;
			double Weight = 0;
			std::uint32_t Order = 0;
		};
		const auto Later = [](const Hinge &A, const Hinge &B) {
			// 同じ折点でも加算順を固定する最小ヒープの優先順位の返戻
			return A.Level > B.Level || A.Level == B.Level && A.Order > B.Order;
		};
		std::vector<std::vector<Hinge>> Heaps(Count);
		DrainageFitResult Result;
		Result.Levels.resize(Count);
		for(const std::uint32_t Index : Order) {
			// 部分木の処理間で中断する返戻
			if(Stop.stop_requested()) return {};
			std::vector<Hinge> &Heap = Heaps[Index];
			double Weight = Sites[Index].Weight, Moment = Weight * Sites[Index].TargetMeters;
			// 費用の一次モーメントが溢れた場合の返戻
			if(!std::isfinite(Moment)) return {};
			while(!Heap.empty() && Heap.front().Level < Moment / Weight) {
				const Hinge Knot = Heap.front();
				std::pop_heap(Heap.begin(), Heap.end(), Later);
				Heap.pop_back();
				Weight += Knot.Weight;
				Moment += Knot.Weight * Knot.Level;
				++Result.PooledKnots;
				// 費用の有限性又は中断条件を満たせない場合の返戻
				if(!std::isfinite(Weight) || !std::isfinite(Moment) || !(Result.PooledKnots % 512) && Stop.stop_requested()) return {};
			}
			const double Mode = Moment / Weight;
			Result.Levels[Index] = Mode;
			Heap.push_back({ Mode, Weight, Index });
			std::push_heap(Heap.begin(), Heap.end(), Later);
			const std::uint32_t Downstream = Sites[Index].Downstream;
			if(Downstream == NoDrainageFitDownstream) {
				// 固定海口に達した費用表は再構成に不要である
				std::vector<Hinge>().swap(Heap);
				continue;
			}
			std::vector<Hinge> &ParentHeap = Heaps[Downstream];
			if(Heap.size() > ParentHeap.size()) Heap.swap(ParentHeap);
			for(const Hinge &Knot : Heap) {
				ParentHeap.push_back(Knot);
				std::push_heap(ParentHeap.begin(), ParentHeap.end(), Later);
				++Result.HeapMoves;
				// 大きい合流の統合中も中断を受け付ける返戻
				if(!(Result.HeapMoves % 512) && Stop.stop_requested()) return {};
			}
			std::vector<Hinge>().swap(Heap);
		}
		for(std::vector<std::uint32_t>::const_reverse_iterator It = Order.rbegin(); It != Order.rend(); ++It) {
			// 水位再構成を中断した場合の返戻
			if(Stop.stop_requested()) return {};
			const std::uint32_t Index = *It, Downstream = Sites[Index].Downstream;
			const double Floor = Downstream == NoDrainageFitDownstream ? LowerMeters : Result.Levels[Downstream];
			Result.Levels[Index] = std::max(Floor, Result.Levels[Index]);
			const double Change = Result.Levels[Index] - Sites[Index].TargetMeters;
			Result.WeightedSquaredChange += Sites[Index].Weight * Change * Change;
			// 最終目的値を有限に保持出来ない場合の返戻
			if(!std::isfinite(Result.WeightedSquaredChange)) return {};
		}
		// 固定海口と全合流点で同じ値を共有する最小二乗解の返戻
		return Result;
	}
}
