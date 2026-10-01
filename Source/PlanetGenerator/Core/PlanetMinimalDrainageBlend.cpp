#include "PlanetMinimalDrainageBlend.h"
#include "PlanetSphericalDrainageBaseline.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 最小盛土の木を共有して，切下げ包絡と線形結合を構成する
	 * @param Directions 閉球面の共有方向
	 * @param Triangles 外向き三角形
	 * @param OriginalHeightsMeters 海底を保持する元標高
	 * @param Settings 半径・海面・最低勾配
	 * @param FillWeight 盛土側の比率，０なら切下げだけ
	 * @param AdjustCoast 海岸条件を守れる比率へ引上げるか
	 * @param Error 拒否理由
	 * @param Stop 中断要求
	 * @return 海陸を保ち有限網で大洋へ下降する比較場
	 */
	std::optional<MinimalDrainageBlend> MinimalDrainageBlend::CreateImpl(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> OriginalHeightsMeters,
		const MinimalDrainageFillSettings &Settings,
		const double FillWeight,
		const bool AdjustCoast,
		MinimalDrainageBlendError &Error,
		const std::stop_token Stop
	) {
		Error = MinimalDrainageBlendError::None;
		const auto Fail = [&](const MinimalDrainageBlendError Reason) -> std::optional<MinimalDrainageBlend> {
			Error = Reason;
			// 部分的な比較場を公開しない返戻
			return {};
		};
		// 中断済みの準備を開始しない返戻
		if(Stop.stop_requested()) return Fail(MinimalDrainageBlendError::Cancelled);
		const bool Invalid = !Settings.IsValid() || !std::isfinite(FillWeight) || FillWeight < 0 || FillWeight > 1;
		// 誤った比率又は球面定義を使わない返戻
		if(Invalid) return Fail(MinimalDrainageBlendError::InvalidInput);
		MinimalDrainageFillError FillError = MinimalDrainageFillError::None;
		const std::optional<MinimalDrainageFill> Filled =
		MinimalDrainageFill::Create(Directions, Triangles, OriginalHeightsMeters, Settings, FillError, Stop);
		// 大洋又は有限網の準備失敗を区別する返戻
		if(!Filled) return Fail(Stop.stop_requested() ? MinimalDrainageBlendError::Cancelled : MinimalDrainageBlendError::FillRejected);
		std::vector<double> Cut(OriginalHeightsMeters.begin(), OriginalHeightsMeters.end());
		std::vector<std::uint32_t> ReverseOrder;
		ReverseOrder.reserve(Directions.size() - Filled->GetStatistics().OceanVertices);
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) {
			if(OriginalHeightsMeters[Index] >= Settings.SeaLevelMeters) ReverseOrder.push_back(Index);
		}
		std::sort(
			ReverseOrder.begin(),
			ReverseOrder.end(),
			[&](const std::uint32_t A, const std::uint32_t B) {
				const double First = Filled->GetHeightsMeters()[A], Second = Filled->GetHeightsMeters()[B];
				// 盛土木の子を親より先に切下げる決定的順序の返戻
				return First == Second ? A > B : First > Second;
			}
		);
		for(const std::uint32_t Vertex : ReverseOrder) {
			// 木を変更中に中断した場合の返戻
			if(!(Vertex & 255) && Stop.stop_requested()) return Fail(MinimalDrainageBlendError::Cancelled);
			const std::uint32_t Parent = Filled->GetDownstreamVertices()[Vertex];
			const Vec3 Difference = Directions[Vertex].Vector() - Directions[Parent].Vector();
			const double Length = Settings.RadiusMeters * 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
			const double Drop = Settings.MinimumLandSlope * Length;
			// 有限な辺長と下流を失った場合の返戻
			if(!std::isfinite(Drop) || !(Drop > 0)) return Fail(MinimalDrainageBlendError::NumericFailure);
			if(OriginalHeightsMeters[Parent] < Settings.SeaLevelMeters) continue;
			const double Upper = std::min(Cut[Vertex] - Drop, std::nextafter(Cut[Vertex], -std::numeric_limits<double>::infinity()));
			Cut[Parent] = std::min(Cut[Parent], Upper);
		}
		double AppliedFillWeight = FillWeight;
		if(AdjustCoast) {
			for(const std::uint32_t Vertex : ReverseOrder) {
				const double Lower = Cut[Vertex], Upper = Filled->GetHeightsMeters()[Vertex];
				if(Lower < Settings.SeaLevelMeters) {
					const double Required = (Settings.SeaLevelMeters - Lower) / (Upper - Lower);
					AppliedFillWeight = std::max(AppliedFillWeight, std::nextafter(Required, 1.0));
				}
				const std::uint32_t Parent = Filled->GetDownstreamVertices()[Vertex];
				if(OriginalHeightsMeters[Parent] >= Settings.SeaLevelMeters) continue;
				const Vec3 Difference = Directions[Vertex].Vector() - Directions[Parent].Vector();
				const double Length = Settings.RadiusMeters * 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
				const double RequiredDrop = Settings.MinimumLandSlope * Length, CutDrop = Lower - Settings.SeaLevelMeters;
				if(CutDrop >= RequiredDrop) continue;
				const double FillDrop = Upper - Settings.SeaLevelMeters, Required = (RequiredDrop - CutDrop) / (FillDrop - CutDrop);
				AppliedFillWeight = std::max(AppliedFillWeight, std::nextafter(Required, 1.0));
			}
			AppliedFillWeight = std::min(1.0, AppliedFillWeight);
			// 海岸を保てる有限な混合比を選べない場合の返戻
			if(!std::isfinite(AppliedFillWeight)) return Fail(MinimalDrainageBlendError::NumericFailure);
		}
		MinimalDrainageBlend Result;
		Result.Stats.AppliedFillWeight = AppliedFillWeight;
		Result.Stats.Vertices = static_cast<std::uint32_t>(Directions.size());
		Result.Stats.OceanVertices = Filled->GetStatistics().OceanVertices;
		Result.HeightsMeters.resize(Directions.size());
		Result.DownstreamVertices.assign(Filled->GetDownstreamVertices().begin(), Filled->GetDownstreamVertices().end());
		Result.Stats.MinimumLandDropMeters = std::numeric_limits<double>::infinity();
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) {
			const double Height = (1 - AppliedFillWeight) * Cut[Index] + AppliedFillWeight * Filled->GetHeightsMeters()[Index];
			const double Original = OriginalHeightsMeters[Index];
			// 元の海底をビット一致で保存する
			Result.HeightsMeters[Index] = Original < Settings.SeaLevelMeters ? Original : Height;
			if(Original < Settings.SeaLevelMeters) continue;
			// 非有限な標高，新しい海域又は半径方向の破綻を拒否する返戻
			// 元の陸を新しい海域に変えた混合比の返戻
			if(Height < Settings.SeaLevelMeters) return Fail(MinimalDrainageBlendError::NewOcean);
			// 非有限な半径又は標高を公開しない返戻
			if(!std::isfinite(Height) || !(Settings.RadiusMeters + Height > 0)) return Fail(MinimalDrainageBlendError::NumericFailure);
			const double Difference = Height - Original;
			if(std::abs(Difference) > 0.01) ++Result.Stats.ChangedLandVertices;
			Result.Stats.MaximumRaiseMeters = std::max(Result.Stats.MaximumRaiseMeters, Difference);
			Result.Stats.MaximumCutMeters = std::max(Result.Stats.MaximumCutMeters, -Difference);
			Result.Stats.SumAbsoluteChangeMeters += std::abs(Difference);
		}
		for(const std::uint32_t Vertex : ReverseOrder) {
			const std::uint32_t Parent = Result.DownstreamVertices[Vertex];
			const double ParentHeight =
			OriginalHeightsMeters[Parent] < Settings.SeaLevelMeters ? Settings.SeaLevelMeters : Result.HeightsMeters[Parent];
			const double Drop = Result.HeightsMeters[Vertex] - ParentHeight;
			const Vec3 Difference = Directions[Vertex].Vector() - Directions[Parent].Vector();
			const double Length = Settings.RadiusMeters * 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
			// 同じ木の親への厳密下降と指定した勾配を守れない場合の返戻
			// 海岸の厳密な水位差を失った混合比の返戻
			if(OriginalHeightsMeters[Parent] < Settings.SeaLevelMeters && Drop + 1E-9 < Settings.MinimumLandSlope * Length) {
				// 海岸での必要下降を失った混合比を拒否する返戻
				return Fail(MinimalDrainageBlendError::ShoreSlope);
			}
			// 同じ木の親への下降条件を失った比較面の返戻
			if(!(Drop > 0) || Drop + 1E-9 < Settings.MinimumLandSlope * Length) return Fail(MinimalDrainageBlendError::NumericFailure);
			Result.Stats.MinimumLandDropMeters = std::min(Result.Stats.MinimumLandDropMeters, Drop);
		}
		SphericalDrainageBaselineError DrainageError = SphericalDrainageBaselineError::None;
		const std::optional<SphericalDrainageBaseline> Baseline =
		SphericalDrainageBaseline::Create(Directions, Triangles, Result.HeightsMeters, Settings.SeaLevelMeters, DrainageError, Stop);
		if(!Baseline) {
			// 同じ地形の全球大洋到達を確認出来ない場合の返戻
			return Fail(Stop.stop_requested() ? MinimalDrainageBlendError::Cancelled : MinimalDrainageBlendError::DrainageRejected);
		}
		// 有限網だけの排水が確認された比較場の返戻
		return Result;
	}

	/**
	 * 指定した比率で同じ木の上下包絡を構成する
	 * @param Directions 閉球面の共有方向
	 * @param Triangles 外向き三角形
	 * @param OriginalHeightsMeters 元の海底と陸上標高
	 * @param Settings 半径・海面・最低勾配
	 * @param FillWeight 固定する盛土側比率
	 * @param Error 拒否理由
	 * @param Stop 中断要求
	 * @return 受理した有限網の比較場
	 */
	std::optional<MinimalDrainageBlend> MinimalDrainageBlend::Create(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> OriginalHeightsMeters,
		const MinimalDrainageFillSettings &Settings,
		const double FillWeight,
		MinimalDrainageBlendError &Error,
		const std::stop_token Stop
	) {
		// 要求した混合比を変えずに構成する返戻
		return CreateImpl(Directions, Triangles, OriginalHeightsMeters, Settings, FillWeight, false, Error, Stop);
	}

	/**
	 * 海陸と海岸勾配を守るために必要な分だけ盛土側へ寄せる
	 * @param Directions 閉球面の共有方向
	 * @param Triangles 外向き三角形
	 * @param OriginalHeightsMeters 元の海底と陸上標高
	 * @param Settings 半径・海面・最低勾配
	 * @param MinimumFillWeight 盛土側比率の下界
	 * @param Error 拒否理由
	 * @param Stop 中断要求
	 * @return 受理した有限網の比較場
	 */
	std::optional<MinimalDrainageBlend> MinimalDrainageBlend::CreateWithCoastalBalance(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> OriginalHeightsMeters,
		const MinimalDrainageFillSettings &Settings,
		const double MinimumFillWeight,
		MinimalDrainageBlendError &Error,
		const std::stop_token Stop
	) {
		// 海岸制約に必要な比率を構成時に選ぶ返戻
		return CreateImpl(Directions, Triangles, OriginalHeightsMeters, Settings, MinimumFillWeight, true, Error, Stop);
	}

	/**
	 * 共有節点の混合標高を取得する
	 * @return 検査済みの不変な標高列
	 */
	std::span<const double> MinimalDrainageBlend::GetHeightsMeters() const {
		// 検査済みの標高を変更させない返戻
		return HeightsMeters;
	}

	/**
	 * 同じ木の大洋方向の親を取得する
	 * @return 各節点の親番号
	 */
	std::span<const std::uint32_t> MinimalDrainageBlend::GetDownstreamVertices() const {
		// 検査済みの下流を変更させない返戻
		return DownstreamVertices;
	}

	/**
	 * 盛土・切下げの実改変量を取得する
	 * @return 実際の改変量と網の規模
	 */
	const MinimalDrainageBlendStatistics &MinimalDrainageBlend::GetStatistics() const {
		// 比較用統計を変更させない返戻
		return Stats;
	}
}
