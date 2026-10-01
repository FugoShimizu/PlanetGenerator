#include "PlanetOutletPotential.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 正の離散 Laplacian 源を大洋零境界へ解き，有限網の下り隣接を確認する
	 * @param Sites 球面網の海陸と方向
	 * @param Edges 同じ網の無向辺
	 * @param PositiveSources 陸上で厳密に正の隆起源
	 * @param TargetMaximumMeters 陸の最高標高
	 * @param Stop 中断要求
	 * @return 全陸地点が下り隣接を持つ有限網の標高，解けない場合は空
	 */
	std::optional<PlanetOutletPotential> PlanetOutletPotential::Create(
		const std::span<const DrainageSite> Sites,
		const std::span<const std::array<std::uint32_t, 2>> Edges,
		const std::span<const double> PositiveSources,
		const double TargetMaximumMeters,
		const std::stop_token Stop
	) {
		const bool InvalidInput =
		Sites.empty() || Sites.size() != PositiveSources.size() || Sites.size() > std::numeric_limits<std::uint32_t>::max() ||
		!std::isfinite(TargetMaximumMeters) || !(TargetMaximumMeters > 0) || Stop.stop_requested();
		// 入力の大きさ・物理値・中断状態が不正な場合の返戻
		if(InvalidInput) return {};
		const std::uint32_t Count = static_cast<std::uint32_t>(Sites.size());
		PlanetOutletPotential Result;
		Result.Stats.Nodes = Count;
		Result.Heights.assign(Count, 0);
		std::vector<std::vector<std::uint32_t>> Neighbors(Count);
		for(const std::array<std::uint32_t, 2> Edge : Edges) {
			// 網の範囲外又は自己辺を拒否する返戻
			if(Edge[0] >= Count || Edge[1] >= Count || Edge[0] == Edge[1]) return {};
			Neighbors[Edge[0]].push_back(Edge[1]);
			Neighbors[Edge[1]].push_back(Edge[0]);
		}
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			std::vector<std::uint32_t> &Around = Neighbors[Index];
			std::sort(Around.begin(), Around.end());
			// 重複辺又は孤立地点を拒否する返戻
			if(std::adjacent_find(Around.begin(), Around.end()) != Around.end() || Around.empty()) return {};
			if(Sites[Index].Ocean) ++Result.Stats.OceanNodes;
			else {
				// 隆起源が非正又は非有限な陸を拒否する返戻
				if(!std::isfinite(PositiveSources[Index]) || !(PositiveSources[Index] > 0)) return {};
				++Result.Stats.LandNodes;
			}
		}
		// 境界又は陸が無い網を拒否する返戻
		if(!Result.Stats.OceanNodes || !Result.Stats.LandNodes) return {};
		std::vector<bool> Reached(Count, false);
		std::queue<std::uint32_t> Frontier;
		for(std::uint32_t Index = 0; Index < Count; ++Index) if(Sites[Index].Ocean) {
			Reached[Index] = true;
			Frontier.push(Index);
		}
		while(!Frontier.empty()) {
			const std::uint32_t Current = Frontier.front();
			Frontier.pop();
			for(const std::uint32_t Neighbor : Neighbors[Current]) if(!Reached[Neighbor]) {
				Reached[Neighbor] = true;
				Frontier.push(Neighbor);
			}
		}
		// 大洋へ連結しない地点を拒否する返戻
		for(const bool IsReached : Reached) if(!IsReached) return {};
		std::vector<double> Residual(Count, 0), Search(Count, 0), Product(Count, 0), Preconditioned(Count, 0);
		double MaximumSource = 0, DotResidual = 0;
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(Sites[Index].Ocean) continue;
			Residual[Index] = PositiveSources[Index];
			Preconditioned[Index] = Residual[Index] / Neighbors[Index].size();
			Search[Index] = Preconditioned[Index];
			DotResidual += Residual[Index] * Preconditioned[Index];
			MaximumSource = std::max(MaximumSource, Residual[Index]);
		}
		// 反復の初期内積が非正又は非有限な場合の返戻
		if(!std::isfinite(DotResidual) || !(DotResidual > 0)) return {};
		const double Tolerance = 1E-10 * MaximumSource;
		const std::uint32_t MaximumIterations = Count < 5000 ? 2 * Count : 10000;
		bool Converged = false;
		for(std::uint32_t Iteration = 0; Iteration < MaximumIterations; ++Iteration) {
			// 中断された反復を公開しない返戻
			if(!(Iteration & 63) && Stop.stop_requested()) return {};
			double Denominator = 0;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				if(Sites[Index].Ocean) continue;
				double Value = Neighbors[Index].size() * Search[Index];
				for(const std::uint32_t Neighbor : Neighbors[Index]) if(!Sites[Neighbor].Ocean) Value -= Search[Neighbor];
				Product[Index] = Value;
				Denominator += Search[Index] * Value;
			}
			// 正定値性が崩れた場合の返戻
			if(!std::isfinite(Denominator) || !(Denominator > 0)) return {};
			const double Step = DotResidual / Denominator;
			// 非有限な反復係数を拒否する返戻
			if(!std::isfinite(Step)) return {};
			double MaximumError = 0, NextDot = 0;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				if(Sites[Index].Ocean) continue;
				Result.Heights[Index] += Step * Search[Index];
				Residual[Index] -= Step * Product[Index];
				MaximumError = std::max(MaximumError, std::abs(Residual[Index]));
				Preconditioned[Index] = Residual[Index] / Neighbors[Index].size();
				NextDot += Residual[Index] * Preconditioned[Index];
			}
			Result.Stats.Iterations = Iteration + 1;
			if(MaximumError <= Tolerance) {
				Converged = true;
				break;
			}
			// 反復の内積が非正又は非有限な場合の返戻
			if(!std::isfinite(NextDot) || !(NextDot > 0)) return {};
			const double Beta = NextDot / DotResidual;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				if(!Sites[Index].Ocean) Search[Index] = Preconditioned[Index] + Beta * Search[Index];
			}
			DotResidual = NextDot;
		}
		// 指定精度へ収束しなかった場合の返戻
		if(!Converged) return {};
		double MaximumHeight = 0;
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(!Sites[Index].Ocean) MaximumHeight = std::max(MaximumHeight, Result.Heights[Index]);
		}
		// 正の最高標高を得られなかった場合の返戻
		if(!std::isfinite(MaximumHeight) || !(MaximumHeight > 0)) return {};
		const double HeightScale = TargetMaximumMeters / MaximumHeight;
		Result.Stats.MaximumElevationMeters = TargetMaximumMeters;
		Result.Stats.MinimumDownhillMeters = std::numeric_limits<double>::infinity();
		for(std::uint32_t Index = 0; Index < Count; ++Index) Result.Heights[Index] *= HeightScale;
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(Sites[Index].Ocean) continue;
			const double Height = Result.Heights[Index];
			// 陸の標高が非正又は非有限な場合の返戻
			if(!std::isfinite(Height) || !(Height > 0)) return {};
			double Lowest = std::numeric_limits<double>::infinity(), Laplacian = Neighbors[Index].size() * Height;
			for(const std::uint32_t Neighbor : Neighbors[Index]) {
				Lowest = std::min(Lowest, Result.Heights[Neighbor]);
				Laplacian -= Result.Heights[Neighbor];
			}
			const double Downhill = Height - Lowest;
			// 下降又は正の離散源を満たさない場合の返戻
			if(!(Downhill > 0) || !(Laplacian > 0)) return {};
			Result.Stats.MinimumDownhillMeters = std::min(Result.Stats.MinimumDownhillMeters, Downhill);
			const double Error = std::abs(Laplacian - PositiveSources[Index] * HeightScale);
			Result.Stats.MaximumResidual = std::max(Result.Stats.MaximumResidual, Error);
		}
		// 全地点で検査した試作場の返戻
		return Result;
	}

	/**
	 * 大洋を零とした有限網の標高を参照する
	 * @return 海域では零，陸上では正の標高
	 */
	std::span<const double> PlanetOutletPotential::Elevations() const {
		return Heights;
	}

	/**
	 * 離散 Poisson 解と下降条件の検査値を参照する
	 * @return 生成中の最大残差と最小下降差
	 */
	const OutletPotentialStatistics &PlanetOutletPotential::Statistics() const {
		return Stats;
	}
} // namespace PlanetCore
