#include "PlanetRunoff.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace PlanetCore {
	namespace {
		/** 桁の違う流域からの加算で失う低位桁を次の加算へ繰り越す */
		struct CompensatedSum {
			double Value = 0, Correction = 0;

			/**
			 * 一つの寄与を蓄積する
			 * @param Contribution 新たな寄与
			 */
			void Add(const double Contribution) {
				const double Adjusted = Contribution - Correction, Next = Value + Adjusted;
				Correction = (Next - Value) - Adjusted;
				Value = Next;
				// 終了
				return;
			}
		};
	}

	/**
	 * 同じ生成定義から準備した気候と排水を結合する
	 * @param Network 固定の排水グラフ
	 * @param Climate 同じ海陸から準備した気候
	 * @param Stop 中断要求
	 * @return 海への収支を満たす流量，未準備又は中断時は空
	 */
	std::optional<PlanetRunoff> PlanetRunoff::Create(
		const DrainageNetwork &Network, const PlanetClimate &Climate, const std::stop_token Stop) {
		// 各地点の不変な気候応答から求めた流量の返戻
		return FromProfiles(
			Network,
			/**
			 * 地点の気候応答を準備する
			 * @param Direction 惑星上の方向
			 * @return 年間の局所水収支を持つ応答
			 */
			[&Climate](const UnitDirection Direction) {
				// 同じ気候から得た応答の返戻
				return Climate.SampleBiome(Direction);
			},
			Stop);
	}

	/**
	 * 局所の年周期水収支を各枝へ集め，海へ到達する量と照合する
	 * @param Network 下流が確定した排水グラフ
	 * @param Provider 地点毎の気候応答，解析解付きの検査にも用いる
	 * @param Stop 中断要求
	 * @return 有効な流量，入力不正・収支違反・中断時は空
	 */
	std::optional<PlanetRunoff> PlanetRunoff::FromProfiles(
		const DrainageNetwork &Network, const ProfileProvider &Provider, const std::stop_token Stop) {
		const std::span<const DrainageNode> Nodes = Network.Nodes();
		// 未準備又は不正な入力の返戻
		if(Nodes.empty() || Nodes.size() > std::numeric_limits<std::uint32_t>::max() || !Provider || Stop.stop_requested())
			return {};
		const std::uint32_t Count = static_cast<std::uint32_t>(Nodes.size());
		std::vector<std::uint32_t> Incoming(Count);
		std::vector<CompensatedSum> UpstreamArea(Count);
		for(std::uint32_t i = 0; i < Count; ++i) {
			const DrainageNode &Node = Nodes[i];
			// 下流番号又は集水面積が不正な場合の返戻
			if(Node.Downstream >= Count || !std::isfinite(Node.ContributingAreaSquareMeters) ||
				Node.ContributingAreaSquareMeters < 0)
				return {};
			if(!Node.Ocean) {
				// 自己循環を持つ陸域の返戻
				if(Node.Downstream == i) return {};
				++Incoming[Node.Downstream];
				UpstreamArea[Node.Downstream].Add(Node.ContributingAreaSquareMeters);
			}
		}
		PlanetRunoff Result;
		Result.SourceHash = Network.SourceDefinitionHash();
		Result.Discharge.resize(Count);
		Result.LocalAreas.resize(Count);
		std::vector<std::array<double, ClimatePeriodCount>> Compensation(Count);
		std::vector<std::uint32_t> Order;
		Order.reserve(Count);
		CompensatedSum Area, Rain, Evaporation, Frozen, LocalRunoff, OceanRunoff;
		constexpr double SecondsPerPeriod = ReferenceYearSeconds / ClimatePeriodCount;
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 要求が不要になった場合の返戻
			if(Stop.stop_requested()) return {};
			if(!Incoming[i]) Order.push_back(i);
			const DrainageNode &Node = Nodes[i];
			if(Node.Ocean) continue;
			// 累積面積ではなく此の地点の面積だけで降水を積算する
			const double LocalArea = Node.ContributingAreaSquareMeters - UpstreamArea[i].Value;
			// 非正又は非有限な局所面積の返戻
			if(!std::isfinite(LocalArea) || LocalArea <= 0) return {};
			const std::optional<UnitDirection> Direction = UnitDirection::FromVector(Node.Direction);
			// 不正な地点方向の返戻
			if(!Direction) return {};
			const std::optional<BiomeProfile> Profile = Provider(*Direction);
			// 気候応答を準備出来なかった場合の返戻
			if(!Profile) return {};
			Result.LocalAreas[i] = LocalArea;
			++Result.Stats.LandNodes;
			Area.Add(LocalArea);
			Rain.Add(Profile->Statistics().AnnualPrecipitationMm * 0.001 * LocalArea);
			for(std::uint32_t j = 0; j < ClimatePeriodCount; ++j) {
				const BiomeWaterPeriod &Period = Profile->WaterCycle()[j];
				const double Volume = Period.RunoffMm * 0.001 * LocalArea;
				// 負又は非有限な水量を公開しない返戻
				if(!std::isfinite(Volume) || Volume < 0) return {};
				Result.Discharge[i][j] = Volume / SecondsPerPeriod;
				LocalRunoff.Add(Volume);
				Evaporation.Add(Period.ActualEvapotranspirationMm * 0.001 * LocalArea);
				Frozen.Add(Period.FrozenExportMm * 0.001 * LocalArea);
			}
		}
		for(std::size_t i = 0; i < Order.size(); ++i) {
			// 準備中断の返戻
			if(Stop.stop_requested()) return {};
			const std::uint32_t Node = Order[i];
			if(Nodes[Node].Ocean) continue;
			const std::uint32_t Downstream = Nodes[Node].Downstream;
			for(std::uint32_t j = 0; j < ClimatePeriodCount; ++j) {
				double &Value = Result.Discharge[Downstream][j], &Correction = Compensation[Downstream][j];
				const double Adjusted = Result.Discharge[Node][j] - Correction, Next = Value + Adjusted;
				// 流量が倍精度の範囲を超える場合の返戻
				if(!std::isfinite(Next)) return {};
				Correction = (Next - Value) - Adjusted;
				Value = Next;
			}
			if(--Incoming[Downstream] == 0) Order.push_back(Downstream);
		}
		// 海へ終端しない循環を持つ場合の返戻
		if(Order.size() != Count) return {};
		for(std::uint32_t i = 0; i < Count; ++i) {
			CompensatedSum Annual;
			for(const double Value : Result.Discharge[i]) Annual.Add(Value * SecondsPerPeriod);
			if(Nodes[i].Ocean) {
				OceanRunoff.Add(Annual.Value);
				if(Annual.Value > 0) ++Result.Stats.ReceivingOceanNodes;
			}
			Result.Stats.MaximumMeanDischargeCubicMetersPerSecond =
				std::max(Result.Stats.MaximumMeanDischargeCubicMetersPerSecond, Annual.Value / ReferenceYearSeconds);
		}
		RunoffStatistics &Stats = Result.Stats;
		Stats.LandAreaSquareMeters = Area.Value;
		Stats.AnnualPrecipitationCubicMeters = Rain.Value;
		Stats.AnnualEvaporationCubicMeters = Evaporation.Value;
		Stats.AnnualFrozenExportCubicMeters = Frozen.Value;
		Stats.AnnualLocalRunoffCubicMeters = LocalRunoff.Value;
		Stats.AnnualOceanRunoffCubicMeters = OceanRunoff.Value;
		Stats.RoutingRelativeResidual = std::abs(LocalRunoff.Value - OceanRunoff.Value) / std::max(1.0, LocalRunoff.Value);
		Stats.WaterBudgetRelativeResidual =
			std::abs(Rain.Value - Evaporation.Value - Frozen.Value - OceanRunoff.Value) / std::max(1.0, Rain.Value);
		Stats.AreaRelativeResidual =
			std::abs(Area.Value - Network.Statistics().LandAreaSquareMeters) / std::max(1.0, Area.Value);
		// 無視出来ない収支違反又は非有限な集計を公開しない返戻
		if(!std::isfinite(Area.Value) || !std::isfinite(Rain.Value) || !std::isfinite(Evaporation.Value) ||
			!std::isfinite(Frozen.Value) || !std::isfinite(LocalRunoff.Value) || !std::isfinite(OceanRunoff.Value) ||
			Stats.RoutingRelativeResidual > 1E-10 || Stats.WaterBudgetRelativeResidual > 1E-8 ||
			Stats.AreaRelativeResidual > 1E-10)
			return {};
		// 海へ到達する流量の返戻
		return Result;
	}

	/**
	 * 地形を更新せず，季節の流量を滑らかに補間する
	 * @param Node 排水グラフの地点番号
	 * @param SolarYearPhase 共通時計の年周位相
	 * @return 毎秒の立方メートル，無効な入力では空
	 */
	std::optional<double> PlanetRunoff::Sample(const std::uint32_t Node, const double SolarYearPhase) const {
		// 不正な入力の返戻
		if(Node >= Discharge.size() || !std::isfinite(SolarYearPhase)) return {};
		const double Position = (SolarYearPhase - std::floor(SolarYearPhase)) * ClimatePeriodCount;
		const std::uint32_t First = static_cast<std::uint32_t>(Position) % ClimatePeriodCount;
		const double Fraction = Position - std::floor(Position);
		const double Blend = Fraction * Fraction * Fraction * (10 + Fraction * (-15 + Fraction * 6));
		const std::uint32_t Second = (First + 1) % ClimatePeriodCount;
		// 各合流点でも加法性を保つ，同じ周期補間の返戻
		return Discharge[Node][First] * (1 - Blend) + Discharge[Node][Second] * Blend;
	}

	/**
	 * 不変な１２期の流量を参照する
	 * @return 地点番号順の流量，単位は毎秒の立方メートル
	 */
	std::span<const std::array<double, ClimatePeriodCount>> PlanetRunoff::GetPeriods() const {
		// 全地点の流量の返戻
		return Discharge;
	}

	/**
	 * 準備時の水収支を取得する
	 * @return 雨・蒸発散・河川流出・未モデル化の氷河輸送の集計
	 */
	const RunoffStatistics &PlanetRunoff::GetStatistics() const {
		// 年水収支の返戻
		return Stats;
	}

	/**
	 * 積算に用いた陸上の局所面積を共有する
	 * @return 地点毎の平方メートル，海の地点は０
	 */
	std::span<const double> PlanetRunoff::GetNodeAreas() const {
		// 後続の侵食で累積面積を二重に使わないための局所面積の返戻
		return LocalAreas;
	}
	/**
	 * 排水が由来する海陸の生成定義を取得する
	 * @return 元の排水の識別値，解析用の任意グラフでは０
	 */
	std::uint64_t PlanetRunoff::GetSourceDefinitionHash() const {
		// 地質・海陸が異なる場の混用を避ける識別値の返戻
		return SourceHash;
	}
}
