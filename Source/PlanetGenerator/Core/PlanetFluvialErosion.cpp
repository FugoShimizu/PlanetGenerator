#include "PlanetFluvialErosion.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace PlanetCore {
	/**
	 * 参照縮尺と有限な侵食パラメータを確認する
	 * @return 対応する範囲内の場合に真
	 */
	bool FluvialErosionSettings::IsValid() const {
		// 有限な時間・速度・流量と，海陸を反転させない制約の返戻
		return std::isfinite(Years) && Years >= 0 && Years <= 200000000 && std::isfinite(ReferenceRadiusMeters) &&
			ReferenceRadiusMeters >= 1000 && ReferenceRadiusMeters <= 1E9 &&
			std::isfinite(ReferenceDischargeCubicMetersPerSecond) && ReferenceDischargeCubicMetersPerSecond > 0 &&
			std::isfinite(ReferenceCelerityMetersPerYear) && ReferenceCelerityMetersPerYear >= 0 &&
			ReferenceCelerityMetersPerYear <= 100 && std::isfinite(DischargeExponent) && DischargeExponent > 0 &&
			DischargeExponent <= 1 && std::isfinite(MaximumFractionalLowering) && MaximumFractionalLowering >= 0 &&
			MaximumFractionalLowering < 1;
	}

	/**
	 * 下流の高さを先に解き，削られた体積を同じ流路で海へ集める
	 * @param Network 同じ生成定義の固定排水
	 * @param Runoff その排水に沿う不変な流量と局所面積
	 * @param Settings 観測時間から独立した侵食時間・係数
	 * @param Stop 中断要求
	 * @return 離散的な侵食・輸送量，入力不正・中断・体積収支違反時は空
	 */
	std::optional<PlanetFluvialErosion> PlanetFluvialErosion::Create(const DrainageNetwork &Network, const PlanetRunoff &Runoff,
		const FluvialErosionSettings &Settings, const std::stop_token Stop) {
		const std::span<const DrainageNode> Graph = Network.Nodes();
		const double Radius = Network.Settings().RadiusMeters, Sea = Network.Settings().SeaLevelMeters;
		// 不正な定義又は異なる規模の場の返戻
		if(!Settings.IsValid() || !Network.Settings().IsValid() || Graph.empty() || Stop.stop_requested() ||
			Graph.size() != Runoff.GetPeriods().size() || Graph.size() != Runoff.GetNodeAreas().size() ||
			Graph.size() >= std::numeric_limits<std::uint32_t>::max() ||
			Runoff.GetSourceDefinitionHash() != Network.SourceDefinitionHash())
			return {};
		const std::uint32_t Count = static_cast<std::uint32_t>(Graph.size());
		std::vector<std::uint32_t> Offsets(Count + 1), Donors, Order;
		Order.reserve(Count);
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 不正な下流参照の返戻
			if(Graph[i].Downstream >= Count) return {};
			if(Graph[i].Ocean)
				Order.push_back(i);
			else
				++Offsets[Graph[i].Downstream + 1];
		}
		for(std::uint32_t i = 1; i <= Count; ++i) Offsets[i] += Offsets[i - 1];
		Donors.resize(Offsets.back());
		std::vector<std::uint32_t> Cursor = Offsets;
		for(std::uint32_t i = 0; i < Count; ++i) {
			if(!Graph[i].Ocean) Donors[Cursor[Graph[i].Downstream]++] = i;
		}
		PlanetFluvialErosion Result;
		Result.SourceHash = Network.SourceDefinitionHash();
		Result.Values.resize(Count);
		const double RadiusRatio = Settings.ReferenceRadiusMeters / Radius;
		for(std::size_t i = 0; i < Order.size(); ++i) {
			// 中断された途中結果の返戻
			if(Stop.stop_requested()) return {};
			const std::uint32_t Id = Order[i];
			const DrainageNode &Node = Graph[Id];
			FluvialErosionNode &Value = Result.Values[Id];
			Value.ElevationMeters = Node.OriginalElevationMeters;
			if(!Node.Ocean) {
				const double Original = Node.OriginalElevationMeters - Sea;
				// 海面以下の陸域を，侵食だけで修復したことにしない返戻
				if(!std::isfinite(Original) || Original <= 0) return {};
				const double Downstream = std::max(0.0, Result.Values[Node.Downstream].ElevationMeters - Sea);
				double MeanDischarge = 0;
				for(const double Period : Runoff.GetPeriods()[Id]) MeanDischarge += Period / ClimatePeriodCount;
				// 対応しない水量を使わない返戻
				if(!std::isfinite(MeanDischarge) || MeanDischarge < 0) return {};
				if(Original > Downstream && MeanDischarge > 0 && Settings.Years > 0 &&
					Settings.ReferenceCelerityMetersPerYear > 0 && Settings.MaximumFractionalLowering > 0) {
					const Vec3 To = Graph[Node.Downstream].Direction;
					const double Angle = std::atan2(Node.Direction.Cross(To).Length(), Node.Direction.Dot(To));
					const double ReferenceLength = Angle * Settings.ReferenceRadiusMeters;
					// 長さの無い流路の返戻
					if(!std::isfinite(ReferenceLength) || ReferenceLength <= 0) return {};
					const double DischargeRatio =
						MeanDischarge * RadiusRatio * RadiusRatio / Settings.ReferenceDischargeCubicMetersPerSecond;
					const double Alpha = Settings.Years * Settings.ReferenceCelerityMetersPerYear *
						std::pow(DischargeRatio, Settings.DischargeExponent) / ReferenceLength;
					// 有限な侵食更新を作れない場合の返戻
					if(!std::isfinite(Alpha)) return {};
					const double Predicted = Downstream + (Original - Downstream) / (1 + Alpha);
					const double Minimum = Original * (1 - Settings.MaximumFractionalLowering);
					if(Predicted < Minimum) ++Result.Stats.LoweringLimitedNodes;
					Value.ElevationMeters = std::min(Node.OriginalElevationMeters,
						std::max(
							std::nextafter(Sea, std::numeric_limits<double>::infinity()), Sea + std::max(Predicted, Minimum)));
					Value.RelativeReliefRatio = (Value.ElevationMeters - Sea) / Original;
				}
				const double Lowering = Node.OriginalElevationMeters - Value.ElevationMeters;
				const double Before = 1 + Node.OriginalElevationMeters / Radius, After = 1 + Value.ElevationMeters / Radius;
				Value.ErodedCubicMeters =
					Runoff.GetNodeAreas()[Id] * Lowering * (Before * Before + Before * After + After * After) / 3;
				// 球面の放射状土柱の体積を表現出来ない場合の返戻
				if(!std::isfinite(Value.ErodedCubicMeters) || Value.ErodedCubicMeters < 0) return {};
				Value.OutgoingSedimentCubicMeters = Value.ErodedCubicMeters;
				if(Lowering > 0) ++Result.Stats.ErodedNodes;
				Result.Stats.MaximumLoweringMeters = std::max(Result.Stats.MaximumLoweringMeters, Lowering);
			}
			for(std::uint32_t j = Offsets[Id]; j < Offsets[Id + 1]; ++j) Order.push_back(Donors[j]);
		}
		// 海へ終端しない閉じた経路の返戻
		if(Order.size() != Count) return {};
		std::vector<double> Compensation(Count);
		for(auto i = Order.rbegin(); i != Order.rend(); ++i) {
			// 体積の輸送を中断する返戻
			if(Stop.stop_requested()) return {};
			if(Graph[*i].Ocean) continue;
			const std::uint32_t Downstream = Graph[*i].Downstream;
			double &Total = Result.Values[Downstream].OutgoingSedimentCubicMeters, &Correction = Compensation[Downstream];
			const double Adjusted = Result.Values[*i].OutgoingSedimentCubicMeters - Correction, Next = Total + Adjusted;
			Correction = (Next - Total) - Adjusted;
			Total = Next;
		}
		double RemovedCorrection = 0, DeliveredCorrection = 0;
		for(std::uint32_t i = 0; i < Count; ++i) {
			const FluvialErosionNode &Value = Result.Values[i];
			const double Removed = Value.ErodedCubicMeters - RemovedCorrection;
			const double Next = Result.Stats.TotalErodedCubicMeters + Removed;
			RemovedCorrection = (Next - Result.Stats.TotalErodedCubicMeters) - Removed;
			Result.Stats.TotalErodedCubicMeters = Next;
			if(Graph[i].Ocean) {
				const double Delivered = Value.OutgoingSedimentCubicMeters - DeliveredCorrection;
				const double OceanNext = Result.Stats.OceanDeliveredCubicMeters + Delivered;
				DeliveredCorrection = (OceanNext - Result.Stats.OceanDeliveredCubicMeters) - Delivered;
				Result.Stats.OceanDeliveredCubicMeters = OceanNext;
			}
		}
		FluvialErosionStatistics &Stats = Result.Stats;
		Stats.SedimentRelativeResidual = std::abs(Stats.TotalErodedCubicMeters - Stats.OceanDeliveredCubicMeters) /
			std::max(1.0, Stats.TotalErodedCubicMeters);
		// 非有限又は体積収支を満たさない結果の返戻
		if(!std::isfinite(Stats.TotalErodedCubicMeters) || !std::isfinite(Stats.OceanDeliveredCubicMeters) ||
			Stats.SedimentRelativeResidual > 1E-10)
			return {};
		// 固定排水での陰解法と離散的な土砂輸送の返戻
		return Result;
	}

	/**
	 * 離散的な標高・体積を取得する
	 * @return 排水グラフと同じ番号の侵食結果
	 */
	std::span<const FluvialErosionNode> PlanetFluvialErosion::GetNodes() const {
		// 地点毎の侵食結果の返戻
		return Values;
	}

	/**
	 * 離散的な土砂の収支を取得する
	 * @return 削られた体積と海へ運ばれた体積の統計
	 */
	const FluvialErosionStatistics &PlanetFluvialErosion::GetStatistics() const {
		// 体積収支の返戻
		return Stats;
	}

	/**
	 * 侵食の元に為った海陸の生成定義を取得する
	 * @return 同じ排水と結合するための識別値
	 */
	std::uint64_t PlanetFluvialErosion::GetSourceDefinitionHash() const {
		// 元の生成定義の返戻
		return SourceHash;
	}
}
