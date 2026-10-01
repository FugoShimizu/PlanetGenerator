#include "PlanetRiverReceiver.h"
#include "PlanetDrainage.h"
#include "PlanetRiverTerrain.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	namespace {
		/**
		 * 河道の元となる全排水節点が，循環せず厳密に低い水位で大洋へ着く事を確認する
		 * @param Network 最終地形と共有する排水網
		 * @param Stop 中断要求
		 * @return 全節点に大洋根がある場合に真
		 */
		bool AllNodesReachOcean(const DrainageNetwork &Network, const std::stop_token Stop) {
			const std::span<const DrainageNode> Nodes = Network.Nodes();
			// 空又は添字の範囲を超える排水網を受理しない返戻
			if(Nodes.empty() || Nodes.size() >= UINT32_MAX) return false;
			std::vector<std::uint8_t> Visit(Nodes.size());
			std::vector<std::uint32_t> Path;
			for(std::uint32_t Start = 0; Start < Nodes.size(); ++Start) {
				// 中断後に部分的な下降証明を返さない
				if(Stop.stop_requested()) return false;
				if(Visit[Start] == 2) continue;
				Path.clear();
				std::uint32_t Current = Start;
				while(true) {
					// 範囲外又は循環へ入った経路への返戻
					if(Current >= Nodes.size() || Visit[Current] == 1) return false;
					if(Visit[Current] == 2) break;
					if(Nodes[Current].Ocean) {
						Visit[Current] = 2;
						break;
					}
					const std::uint32_t Downstream = Nodes[Current].Downstream;
					// 上昇・停滞・範囲外の下流を拒否する返戻
					if(Downstream >= Nodes.size() || !(Nodes[Current].WaterMeters > Nodes[Downstream].WaterMeters)) return false;
					Visit[Current] = 1;
					Path.push_back(Current);
					Current = Downstream;
					// 経路の途中で中断した場合の返戻
					if(Stop.stop_requested()) return false;
				}
				for(const std::uint32_t Node : Path) Visit[Node] = 2;
			}
			// 全陸上節点の下降と大洋到達を確認した返戻
			return true;
		}
	}

	/**
	 * 最終地形で水面が支持される河道中心を，同じ大洋根の受入候補として準備する
	 * @param Field 表示・衝突と同じ最終地形
	 * @param SegmentsPerReach 一河道の等分数
	 * @param Error 失敗理由
	 * @param Stop 中断要求
	 * @return 全排水節点が大洋へ下り，支持された候補がある場合の索引
	 */
	std::optional<RiverReceiverIndex> RiverReceiverIndex::Create(
		const RiverTerrainField &Field,
		const std::uint32_t SegmentsPerReach,
		RiverReceiverError &Error,
		const std::stop_token Stop
	) {
		Error = RiverReceiverError::None;
		if(Stop.stop_requested()) {
			Error = RiverReceiverError::Cancelled;
			// 中断した準備結果を公開しない返戻
			return {};
		}
		const DrainageNetwork &Network = Field.Drainage();
		const std::span<const RiverReach> Reaches = Network.Reaches();
		const double Radius = Network.Settings().RadiusMeters;
		if(
			!(Radius >= 1000) || !std::isfinite(Radius) || !SegmentsPerReach || SegmentsPerReach > 256 || Reaches.empty() ||
			Reaches.size() >= UINT32_MAX
		) {
			Error = RiverReceiverError::InvalidInput;
			// 無効な尺度又は河道数への返戻
			return {};
		}
		if(!AllNodesReachOcean(Network, Stop)) {
			Error = Stop.stop_requested() ? RiverReceiverError::Cancelled : RiverReceiverError::InvalidDrainage;
			// 未証明の大洋経路への返戻
			return {};
		}
		RiverReceiverIndex Result;
		Result.RadiusMeters = Radius;
		Result.Stats.Reaches = static_cast<std::uint32_t>(Reaches.size());
		for(std::uint32_t Reach = 0; Reach < Reaches.size(); ++Reach) for(std::uint32_t Knot = 0; Knot < SegmentsPerReach + 1; ++Knot) {
			if(Stop.stop_requested()) {
				Error = RiverReceiverError::Cancelled;
				// 標本準備中の中断への返戻
				return {};
			}
			++Result.Stats.Sampled;
			const double Fraction = static_cast<double>(Knot) / SegmentsPerReach;
			const std::optional<RiverProfileSample> Profile = Network.SampleReach(Reach, Fraction);
			if(!Profile) {
				++Result.Stats.Rejected;
				continue;
			}
			const std::optional<UnitDirection> Direction = UnitDirection::FromVector(Profile->Direction);
			if(!Direction) {
				++Result.Stats.Rejected;
				continue;
			}
			const RiverWaterHeights Water = Field.EvaluateWater(*Direction);
			const bool Supported =
			std::isfinite(Water.WaterSupport) && Water.WaterSupport > 0 && std::isfinite(Water.WaterElevationMeters) &&
			std::isfinite(Water.TerrainElevationMeters) && std::isfinite(Profile->BedMeters) &&
			Water.TerrainElevationMeters <= Water.WaterElevationMeters + 0.001;
			if(!Supported) {
				++Result.Stats.Rejected;
				continue;
			}
			Result.Sites.push_back(
				{
					*Direction,
					Reach,
					Fraction,
					Water.WaterElevationMeters,
					Water.TerrainElevationMeters,
					Profile->BedMeters,
					Water.WaterSupport
				}
			);
			++Result.Stats.Supported;
		}
		if(Result.Sites.empty()) {
			Error = RiverReceiverError::NoSupportedSite;
			// 支持された受入点が無い場合の返戻
			return {};
		}
		// 同じ最終場で検証した受入候補の返戻
		return Result;
	}

	/**
	 * 水位上限を守る最寄りの既存河川を探索する
	 * @param Origin 補助流路の元盆地
	 * @param MaximumWaterMeters 元盆地から下降可能な受入水位の上限
	 * @param MaximumDistanceMeters 受入候補の物理距離上限
	 * @param Stop 中断要求
	 * @return 条件内で最も近い支持された河道中心
	 */
	std::optional<RiverReceiverNearest> RiverReceiverIndex::FindNearest(
		const UnitDirection Origin,
		const double MaximumWaterMeters,
		const double MaximumDistanceMeters,
		const std::stop_token Stop
	) const {
		if(std::isnan(MaximumWaterMeters) || std::isnan(MaximumDistanceMeters) || MaximumDistanceMeters < 0 || Stop.stop_requested()) {
			// 無効な水位・距離又は開始前の中断への返戻
			return {};
		}
		const Vec3 A = Origin.Vector();
		const double MaximumAngle = MaximumDistanceMeters / RadiusMeters;
		const double MaximumChordSquared = MaximumAngle >= std::numbers::pi ? 4.0 : 4 * std::pow(std::sin(MaximumAngle * 0.5), 2);
		const RiverReceiverSite *Best = nullptr;
		double BestChordSquared = std::numeric_limits<double>::infinity();
		for(const RiverReceiverSite &Site : Sites) {
			// 照会中の中断で候補を公開しない返戻
			if(Stop.stop_requested()) return {};
			if(Site.WaterMeters > MaximumWaterMeters) continue;
			const Vec3 Delta = Site.Direction.Vector() - A;
			const double ChordSquared = Delta.Dot(Delta);
			if(ChordSquared > MaximumChordSquared || ChordSquared >= BestChordSquared) continue;
			Best = &Site;
			BestChordSquared = ChordSquared;
		}
		// 制約を満たす候補が無い場合の返戻
		if(!Best) return {};
		const Vec3 B = Best->Direction.Vector();
		const double Distance = RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
		// 最近傍だけに球面角を評価した結果の返戻
		return RiverReceiverNearest{ *Best, Distance };
	}

	/**
	 * 水位と球面距離の双方で支配されない受入河川の標本を返す
	 * 計算量 O(n log n)，保持量 O(n)；通路の開削費用は未評価
	 * @param Origin 補助流路の元盆地
	 * @param MaximumWaterMeters 受入候補の水位上限
	 * @param MaximumDistanceMeters 受入候補の球面距離上限
	 * @param Stop 中断要求
	 * @return 距離昇順・水位厳密下降の候補；入力不正又は中断時は空
	 */
	std::optional<std::vector<RiverReceiverNearest>> RiverReceiverIndex::FindDistanceWaterPareto(
		const UnitDirection Origin,
		const double MaximumWaterMeters,
		const double MaximumDistanceMeters,
		const std::stop_token Stop
	) const {
		if(std::isnan(MaximumWaterMeters) || std::isnan(MaximumDistanceMeters) || MaximumDistanceMeters < 0 || Stop.stop_requested()) {
			// 無効な制約又は開始前の中断への返戻
			return {};
		}
		struct Candidate {
			double ChordSquared = 0;
			std::size_t SiteIndex = 0;
		};
		const Vec3 A = Origin.Vector();
		const double MaximumAngle = MaximumDistanceMeters / RadiusMeters;
		const double MaximumChordSquared = MaximumAngle >= std::numbers::pi ? 4.0 : 4 * std::pow(std::sin(MaximumAngle * 0.5), 2);
		std::vector<Candidate> Candidates;
		Candidates.reserve(Sites.size());
		for(std::size_t i = 0; i < Sites.size(); ++i) {
			// 照会中の中断で部分的な候補を公開しない返戻
			if(Stop.stop_requested()) return {};
			const RiverReceiverSite &Site = Sites[i];
			if(Site.WaterMeters > MaximumWaterMeters) continue;
			const Vec3 Delta = Site.Direction.Vector() - A;
			const double ChordSquared = Delta.Dot(Delta);
			if(ChordSquared > MaximumChordSquared) continue;
			Candidates.push_back({ ChordSquared, i });
		}
		std::sort(
			Candidates.begin(),
			Candidates.end(),
			[&](const Candidate &Left, const Candidate &Right) {
				// 距離が違う場合は近い候補を先にする返戻
				if(Left.ChordSquared != Right.ChordSquared) return Left.ChordSquared < Right.ChordSquared;
				const double LeftWater = Sites[Left.SiteIndex].WaterMeters, RightWater = Sites[Right.SiteIndex].WaterMeters;
				// 同距離では低い水位を先にして支配候補を取り除く返戻
				if(LeftWater != RightWater) return LeftWater < RightWater;
				// 同距離・同水位では準備済み候補の順序を保つ返戻
				return Left.SiteIndex < Right.SiteIndex;
			}
		);
		std::vector<RiverReceiverNearest> Result;
		double LowestWater = std::numeric_limits<double>::infinity();
		for(const Candidate &Entry : Candidates) {
			// 順位構成中の中断で部分的な候補を公開しない返戻
			if(Stop.stop_requested()) return {};
			const RiverReceiverSite &Site = Sites[Entry.SiteIndex];
			if(!(Site.WaterMeters < LowestWater)) continue;
			const Vec3 B = Site.Direction.Vector();
			const double Distance = RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
			// 距離を測れない候補を順位として公開しない返戻
			if(!std::isfinite(Distance)) return {};
			Result.push_back({ Site, Distance });
			LowestWater = Site.WaterMeters;
		}
		// 距離と水位で互いに支配されない候補集合の返戻
		return Result;
	}

	/**
	 * 保持した同じ最終場の標本を参照する
	 * @return 河道中心の不変な標本
	 */
	std::span<const RiverReceiverSite> RiverReceiverIndex::GetSites() const {
		// 不変な候補一覧の返戻
		return Sites;
	}

	/**
	 * 大洋下降と水位支持を確認した標本数を参照する
	 * @return 準備時の標本数と除外数
	 */
	const RiverReceiverStatistics &RiverReceiverIndex::GetStatistics() const {
		// 準備時に確定した統計の返戻
		return Stats;
	}
}
