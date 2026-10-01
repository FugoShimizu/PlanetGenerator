#include "PlanetMinimalDrainageMinimax.h"
#include "PlanetSphericalDrainageBaseline.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 盛土基準解の排水木に海岸と最小下降量を課し，最大絶対補正を最小化する
	 * @param Directions 閉じた球面網の方向
	 * @param Triangles 球面網の三角形
	 * @param OriginalHeightsMeters 海底を含む元の標高
	 * @param Settings 半径，海面，数値下降勾配
	 * @param Error 失敗理由
	 * @param Stop 中断要求
	 * @return 有限網で検証済みの補正標高
	 */
	std::optional<MinimalDrainageMinimax> MinimalDrainageMinimax::Create(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> OriginalHeightsMeters,
		const MinimalDrainageFillSettings &Settings,
		MinimalDrainageMinimaxError &Error,
		const std::stop_token Stop
	) {
		Error = MinimalDrainageMinimaxError::None;
		const auto Fail = [&](const MinimalDrainageMinimaxError Reason) -> std::optional<MinimalDrainageMinimax> {
			Error = Reason;
			// 部分的な補正を公開しない
			return {};
		};
		// 中断済みの準備を開始しない
		// 中断済みの計算を公開しない
		if(Stop.stop_requested()) return Fail(MinimalDrainageMinimaxError::Cancelled);
		MinimalDrainageFillError FillError = MinimalDrainageFillError::None;
		const std::optional<MinimalDrainageFill> Fill =
		MinimalDrainageFill::Create(Directions, Triangles, OriginalHeightsMeters, Settings, FillError, Stop);
		// 基準木の不正と中断を区別する
		if(!Fill) {
			// 基準木の準備に失敗した事を伝える
			return Fail(Stop.stop_requested() ? MinimalDrainageMinimaxError::Cancelled : MinimalDrainageMinimaxError::FillRejected);
		}
		std::vector<std::uint32_t> Order;
		Order.reserve(Directions.size() - Fill->GetStatistics().OceanVertices);
		std::vector<double> EdgeDrop(Directions.size());
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) {
			if(OriginalHeightsMeters[Index] < Settings.SeaLevelMeters) continue;
			const std::uint32_t Parent = Fill->GetDownstreamVertices()[Index];
			const Vec3 Difference = Directions[Index].Vector() - Directions[Parent].Vector();
			const double Arc = 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
			EdgeDrop[Index] = Settings.MinimumLandSlope * Settings.RadiusMeters * Arc;
			// 無効な距離を持つ排水木を補正しない
			// 不正な辺長を拒否する
			if(!std::isfinite(EdgeDrop[Index]) || !(EdgeDrop[Index] > 0)) return Fail(MinimalDrainageMinimaxError::NumericFailure);
			Order.push_back(Index);
		}
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::uint32_t A, const std::uint32_t B) {
				const double First = Fill->GetHeightsMeters()[A], Second = Fill->GetHeightsMeters()[B];
				// 下流から上流の順を固定する
				return First == Second ? A < B : First < Second;
			}
		);
		std::vector<double> Upper(Directions.size()), Candidate(OriginalHeightsMeters.begin(), OriginalHeightsMeters.end());
		MinimalDrainageMinimaxError FeasibilityError = MinimalDrainageMinimaxError::None;
		const auto Feasible = [&](const double Limit) -> bool {
			for(const std::uint32_t Vertex : Order) Upper[Vertex] = OriginalHeightsMeters[Vertex] + Limit;
			for(std::size_t Position = Order.size(); Position > 0; --Position) {
				const std::uint32_t Vertex = Order[Position - 1], Parent = Fill->GetDownstreamVertices()[Vertex];
				if(OriginalHeightsMeters[Parent] < Settings.SeaLevelMeters) {
					// 海岸の最小下降量を確保できない
					if(Upper[Vertex] < Settings.SeaLevelMeters + EdgeDrop[Vertex]) {
						FeasibilityError = MinimalDrainageMinimaxError::CoastUpper;
						// 海岸の下降可能範囲が空である
						return false;
					}
					continue;
				}
				Upper[Parent] = std::min(Upper[Parent], Upper[Vertex] - EdgeDrop[Vertex]);
			}
			for(const std::uint32_t Vertex : Order) {
				// 中断した候補は実行可能と扱わない
				if(Stop.stop_requested()) return false;
				const std::uint32_t Parent = Fill->GetDownstreamVertices()[Vertex];
				const double ParentHeight =
				OriginalHeightsMeters[Parent] < Settings.SeaLevelMeters ? Settings.SeaLevelMeters : Candidate[Parent];
				const double Lower =
				std::max({ OriginalHeightsMeters[Vertex] - Limit, Settings.SeaLevelMeters, ParentHeight + EdgeDrop[Vertex] });
				// 制約を満たす標高がない
				if(!std::isfinite(Lower) || !std::isfinite(Upper[Vertex]) || Lower > Upper[Vertex]) {
					FeasibilityError = MinimalDrainageMinimaxError::EmptyWindow;
					// 節点の許容標高範囲が空である
					return false;
				}
				Candidate[Vertex] = std::clamp(OriginalHeightsMeters[Vertex], Lower, Upper[Vertex]);
			}
			// 全地点が所定の標高帯と下降条件を満たす
			return true;
		};
		double LowerLimit = 0, UpperLimit = Fill->GetStatistics().MaximumRaiseMeters;
		// 盛土基準解を上界として確保する
		// 基準の盛土上界でも実行できない場合を拒否する
		if(!Feasible(UpperLimit)) return Fail(Stop.stop_requested() ? MinimalDrainageMinimaxError::Cancelled : FeasibilityError);
		for(unsigned Step = 0; Step < 60; ++Step) {
			// 二分探索中の中断を伝播する
			// 中断済みの計算を公開しない
			if(Stop.stop_requested()) return Fail(MinimalDrainageMinimaxError::Cancelled);
			const double Middle = std::midpoint(LowerLimit, UpperLimit);
			if(Middle == LowerLimit || Middle == UpperLimit) break;
			if(Feasible(Middle)) UpperLimit = Middle;
			else LowerLimit = Middle;
		}
		// 最後の失敗試行で上書きされた候補を確実に再構成する
		// 二分探索後の上界の喪失を拒否する
		// 二分探索後の許容範囲を再確認する
		if(!Feasible(UpperLimit)) return Fail(FeasibilityError);
		MinimalDrainageMinimax Result;
		Result.HeightsMeters = std::move(Candidate);
		Result.DownstreamVertices.assign(Fill->GetDownstreamVertices().begin(), Fill->GetDownstreamVertices().end());
		double Scale = 1, MinimumEdgeDrop = std::numeric_limits<double>::infinity();
		for(const std::uint32_t Vertex : Order) {
			Scale = std::max(Scale, std::abs(Result.HeightsMeters[Vertex]));
			MinimumEdgeDrop = std::min(MinimumEdgeDrop, EdgeDrop[Vertex]);
		}
		const double TieStep = 16 * std::numeric_limits<double>::epsilon() * Scale;
		const double MaximumTieBreak = TieStep * static_cast<double>(Order.size());
		// 下り勾配より十分小さな数値差だけで平坦陸上辺を解消する
		if(!std::isfinite(MaximumTieBreak) || MaximumTieBreak >= 0.25 * MinimumEdgeDrop) {
			// 数値差が実際の下降条件へ影響する設定を拒否する
			return Fail(MinimalDrainageMinimaxError::NumericFailure);
		}
		for(std::size_t Position = 0; Position < Order.size(); ++Position) {
			Result.HeightsMeters[Order[Position]] += TieStep * static_cast<double>(Position + 1);
		}
		Result.Stats.MaximumTieBreakMeters = MaximumTieBreak;
		Result.Stats.MinimumLandDropMeters = std::numeric_limits<double>::infinity();
		for(const std::uint32_t Vertex : Order) {
			const std::uint32_t Parent = Result.DownstreamVertices[Vertex];
			const double ParentHeight =
			OriginalHeightsMeters[Parent] < Settings.SeaLevelMeters ? Settings.SeaLevelMeters : Result.HeightsMeters[Parent];
			const double Drop = Result.HeightsMeters[Vertex] - ParentHeight;
			// 浮動小数点誤差を含む場合でも下降の符号は必ず正とする
			// 排水木上の下降条件の喪失を拒否する
			// 補正後の親辺の下降を拒否する
			if(!(Drop > 0) || Drop + 1E-9 < EdgeDrop[Vertex]) return Fail(MinimalDrainageMinimaxError::FinalSlope);
			Result.Stats.MinimumLandDropMeters = std::min(Result.Stats.MinimumLandDropMeters, Drop);
			const double Difference = Result.HeightsMeters[Vertex] - OriginalHeightsMeters[Vertex];
			if(std::abs(Difference) > 0.01) ++Result.Stats.ChangedLandVertices;
			Result.Stats.MaximumRaiseMeters = std::max(Result.Stats.MaximumRaiseMeters, Difference);
			Result.Stats.MaximumCutMeters = std::max(Result.Stats.MaximumCutMeters, -Difference);
			Result.Stats.MaximumAbsoluteChangeMeters = std::max(Result.Stats.MaximumAbsoluteChangeMeters, std::abs(Difference));
			Result.Stats.SumAbsoluteChangeMeters += std::abs(Difference);
		}
		SphericalDrainageBaselineError DrainageError = SphericalDrainageBaselineError::None;
		const std::optional<SphericalDrainageBaseline> Baseline =
		SphericalDrainageBaseline::Create(Directions, Triangles, Result.HeightsMeters, Settings.SeaLevelMeters, DrainageError, Stop);
		// 補正後の全陸上節点が連結大洋へ達する事を独立に確認する
		if(!Baseline) {
			// 独立した全球排水検査の失敗を伝える
			const MinimalDrainageMinimaxError Reason = Stop.stop_requested() ?
			MinimalDrainageMinimaxError::Cancelled :
			DrainageError == SphericalDrainageBaselineError::LevelLandEdge ?
			MinimalDrainageMinimaxError::LevelLandEdge :
			DrainageError == SphericalDrainageBaselineError::LandSink ?
			MinimalDrainageMinimaxError::LandSink :
			MinimalDrainageMinimaxError::DrainageRejected;
			// 独立検査の具体的な失敗を伝える
			return Fail(Reason);
		}
		// 有限網の最小最大補正解を公開する
		return Result;
	}

	/**
	 * 補正済み標高を取得する
	 * @return 海底を保存した有限網の標高
	 */
	std::span<const double> MinimalDrainageMinimax::GetHeightsMeters() const {
		// 不変な補正標高を返す
		return HeightsMeters;
	}

	/**
	 * 固定した大洋根の排水木を取得する
	 * @return 各節点の下流親
	 */
	std::span<const std::uint32_t> MinimalDrainageMinimax::GetDownstreamVertices() const {
		// 不変な排水木を返す
		return DownstreamVertices;
	}

	/**
	 * 地形変更量を取得する
	 * @return 最大補正，盛土，開削，合計の実測値
	 */
	const MinimalDrainageMinimaxStatistics &MinimalDrainageMinimax::GetStatistics() const {
		// 検証済みの統計を返す
		return Stats;
	}
}
