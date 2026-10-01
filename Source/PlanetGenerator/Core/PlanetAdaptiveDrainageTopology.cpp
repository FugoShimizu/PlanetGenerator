#include "PlanetAdaptiveDrainageTopology.h"
#include "PlanetSphericalDrainageBaseline.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <utility>

namespace PlanetCore {
	namespace {
		struct AdaptiveNeighbor {
			std::uint32_t Vertex = 0;
			double DropMeters = 0;
		};

		enum class Feasibility : std::uint8_t { Yes, No, NumericFailure, Cancelled };
	}

	/**
	 * 流路木も変え，最大絶対補正幅を二分探索する
	 * 各候補幅では単調な辺緩和を使い，大洋から到達可能な最小標高を確定する
	 * @param Directions 閉じた球面網の共有節点方向
	 * @param Triangles 閉じた球面網の三角形
	 * @param OriginalHeightsMeters 元の地質・侵食標高
	 * @param Settings 球半径，海面と親辺の正勾配
	 * @param Error 失敗時の理由
	 * @param Stop 中断要求
	 * @return 全陸上節点の大洋到達を独立に確かめた補正結果
	 */
	std::optional<AdaptiveDrainageTopology> AdaptiveDrainageTopology::Create(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> OriginalHeightsMeters,
		const MinimalDrainageFillSettings &Settings,
		AdaptiveDrainageError &Error,
		const std::stop_token Stop
	) {
		Error = AdaptiveDrainageError::None;
		const auto Fail = [&](const AdaptiveDrainageError Reason) -> std::optional<AdaptiveDrainageTopology> {
			Error = Reason;
			// 不完全な排水木を公開しない返戻
			return {};
		};
		// 中断済みの地形準備を開始しない返戻
		if(Stop.stop_requested()) return Fail(AdaptiveDrainageError::Cancelled);
		MinimalDrainageFillError FillError = MinimalDrainageFillError::None;
		const std::optional<MinimalDrainageFill> Fill =
		MinimalDrainageFill::Create(Directions, Triangles, OriginalHeightsMeters, Settings, FillError, Stop);
		// 共有網又は大洋条件が不正な場合の返戻
		if(!Fill) return Fail(Stop.stop_requested() ? AdaptiveDrainageError::Cancelled : AdaptiveDrainageError::FillRejected);
		const std::uint32_t Count = static_cast<std::uint32_t>(Directions.size());
		AdaptiveDrainageTopology Result;
		Result.Stats.Vertices = Count;
		Result.Stats.OceanVertices = Fill->GetStatistics().OceanVertices;
		Result.Stats.FixedTreeFillMaxRaiseMeters = Fill->GetStatistics().MaximumRaiseMeters;
		std::vector<bool> IsOcean(Count);
		for(std::uint32_t Index = 0; Index < Count; ++Index) IsOcean[Index] = OriginalHeightsMeters[Index] < Settings.SeaLevelMeters;
		std::vector<std::array<std::uint32_t, 2>> Edges;
		Edges.reserve(Triangles.size() * 3);
		for(const std::array<std::uint32_t, 3> &Face : Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			Edges.push_back({ std::min(A, B), std::max(A, B) });
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		Result.Stats.Edges = static_cast<std::uint32_t>(Edges.size());
		std::vector<std::uint32_t> Offsets(Count + 1);
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			++Offsets[Edge[0] + 1];
			++Offsets[Edge[1] + 1];
		}
		for(std::size_t Index = 1; Index < Offsets.size(); ++Index) Offsets[Index] += Offsets[Index - 1];
		std::vector<std::uint32_t> Cursor = Offsets;
		std::vector<AdaptiveNeighbor> Neighbors(Offsets.back());
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			const std::uint32_t A = Edge[0], B = Edge[1];
			const Vec3 Difference = Directions[A].Vector() - Directions[B].Vector();
			const double Arc = 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
			const double Drop = Settings.MinimumLandSlope * Settings.RadiusMeters * Arc;
			// 親辺で正の有限な下降余裕を表現出来ない場合の返戻
			if(!std::isfinite(Drop) || !(Drop > 0)) return Fail(AdaptiveDrainageError::NumericFailure);
			Neighbors[Cursor[A]++] = { B, Drop };
			Neighbors[Cursor[B]++] = { A, Drop };
		}
		std::vector<double> ChosenHeights;
		std::vector<std::uint32_t> ChosenParents;
		std::vector<double> ChosenParentDrops;
		const auto Feasible = [&](const double Limit, const bool Store) -> Feasibility {
			++Result.Stats.FeasibilityRuns;
			// 浮動小数点の許容区間を構築出来ない候補の返戻
			if(!std::isfinite(Limit) || Limit < 0) return Feasibility::NumericFailure;
			std::vector<double> Lower(Count), Upper(Count), Heights(Count, std::numeric_limits<double>::infinity());
			std::vector<std::uint32_t> Parent(Count);
			std::vector<double> ParentDrop(Count);
			std::vector<bool> Settled(Count);
			using Entry = std::pair<double, std::uint32_t>;
			std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				Parent[Index] = Index;
				if(!IsOcean[Index]) {
					Lower[Index] = OriginalHeightsMeters[Index] - Limit;
					Upper[Index] = OriginalHeightsMeters[Index] + Limit;
					// 元地形の許容区間を有限に作れない候補の返戻
					if(!std::isfinite(Lower[Index]) || !std::isfinite(Upper[Index])) return Feasibility::NumericFailure;
					continue;
				}
				Heights[Index] = Settings.SeaLevelMeters;
				Frontier.emplace(Settings.SeaLevelMeters, Index);
			}
			std::uint32_t SettledLand = 0;
			while(!Frontier.empty()) {
				// 多数の候補辺の処理中にも中断を受け付ける返戻
				if(!(SettledLand % 256) && Stop.stop_requested()) return Feasibility::Cancelled;
				const Entry Current = Frontier.top();
				Frontier.pop();
				const std::uint32_t Index = Current.second;
				if(Settled[Index] || Current.first != Heights[Index]) continue;
				Settled[Index] = true;
				if(!IsOcean[Index]) ++SettledLand;
				for(std::uint32_t At = Offsets[Index]; At < Offsets[Index + 1]; ++At) {
					const AdaptiveNeighbor &Link = Neighbors[At];
					const std::uint32_t Next = Link.Vertex;
					if(IsOcean[Next] || Settled[Next]) continue;
					const double Required = Current.first + Link.DropMeters;
					double Candidate = std::max(Lower[Next], Required);
					if(!(Candidate > Current.first)) Candidate = std::nextafter(Current.first, std::numeric_limits<double>::infinity());
					// 丸めで失った親辺の落差を一段だけ回復する
					const double CandidateDrop = Candidate - Current.first;
					if(CandidateDrop < Link.DropMeters) Candidate = std::nextafter(Candidate, std::numeric_limits<double>::infinity());
					// 丸め補正後も親辺の下降余裕を表現出来ない候補の返戻
					if(
						!std::isfinite(Candidate) || !std::isfinite(Required) || !(Candidate > Current.first) ||
						Candidate - Current.first < Link.DropMeters
					) return Feasibility::NumericFailure;
					if(Candidate > Upper[Next] || Candidate >= Heights[Next]) continue;
					Heights[Next] = Candidate;
					Parent[Next] = Index;
					ParentDrop[Next] = Link.DropMeters;
					Frontier.emplace(Candidate, Next);
				}
			}
			// 全陸が大洋へ接続しなかった候補幅の返戻
			if(SettledLand != Count - Result.Stats.OceanVertices) return Feasibility::No;
			if(Store) {
				ChosenHeights = std::move(Heights);
				ChosenParents = std::move(Parent);
				ChosenParentDrops = std::move(ParentDrop);
			}
			// 具体的な下流木を持つ候補幅の返戻
			return Feasibility::Yes;
		};
		double LowerBound = 0, UpperBound = Result.Stats.FixedTreeFillMaxRaiseMeters;
		const Feasibility UpperReady = Feasible(UpperBound, false);
		// 既存の盛土上界でも数値上の候補を構成出来ない場合の返戻
		if(UpperReady != Feasibility::Yes) {
			// 上界として保証された盛土幅が成立しない理由を返す
			return Fail(
				UpperReady == Feasibility::Cancelled ?
				AdaptiveDrainageError::Cancelled :
				UpperReady == Feasibility::NumericFailure ? AdaptiveDrainageError::NumericFailure : AdaptiveDrainageError::InfeasibleBound
			);
		}
		for(unsigned Step = 0; Step < 60; ++Step) {
			// 上下界が同じ浮動小数点数へ収束した場合は再試行しない
			if(std::midpoint(LowerBound, UpperBound) == LowerBound || std::midpoint(LowerBound, UpperBound) == UpperBound) break;
			const double Middle = std::midpoint(LowerBound, UpperBound);
			const Feasibility FeasibleMiddle = Feasible(Middle, false);
			// 数値破綻又は中断を不可能な候補幅と混同しない返戻
			// 探索途中の中断理由を返す
			if(FeasibleMiddle == Feasibility::Cancelled) return Fail(AdaptiveDrainageError::Cancelled);
			// 探索途中の数値破綻理由を返す
			if(FeasibleMiddle == Feasibility::NumericFailure) return Fail(AdaptiveDrainageError::NumericFailure);
			if(FeasibleMiddle == Feasibility::Yes) UpperBound = Middle;
			else LowerBound = Middle;
		}
		const Feasibility FinalReady = Feasible(UpperBound, true);
		// 探索した上界で実際の親辺を再構成出来ない場合の返戻
		if(FinalReady != Feasibility::Yes) {
			// 最後の幅で親辺を再構成出来ない理由を返す
			return Fail(FinalReady == Feasibility::Cancelled ? AdaptiveDrainageError::Cancelled : AdaptiveDrainageError::NumericFailure);
		}
		Result.Stats.MinimumFeasibleBoundMeters = UpperBound;
		// 同じ高さの乾いた共有辺は面内で下降方向を失う
		// 親から子への順で
		// 隣接点と一致する場合だけ最小の表現可能な高さへ上げる
		std::vector<std::uint32_t> AscendingLand;
		AscendingLand.reserve(Count - Result.Stats.OceanVertices);
		for(std::uint32_t Index = 0; Index < Count; ++Index) if(!IsOcean[Index]) AscendingLand.push_back(Index);
		std::sort(
			AscendingLand.begin(),
			AscendingLand.end(),
			[&](const std::uint32_t A, const std::uint32_t B) {
				return ChosenHeights[A] == ChosenHeights[B] ? A < B : ChosenHeights[A] < ChosenHeights[B];
			}
		);
		std::vector<bool> HeightFinalized(Count);
		for(const std::uint32_t Index : AscendingLand) {
			double Height = ChosenHeights[Index];
			const std::uint32_t Parent = ChosenParents[Index];
			const double ParentHeight = IsOcean[Parent] ? Settings.SeaLevelMeters : ChosenHeights[Parent];
			if(Height - ParentHeight < ChosenParentDrops[Index]) {
				Height = std::nextafter(ParentHeight + ChosenParentDrops[Index], std::numeric_limits<double>::infinity());
			}
			bool Collision;
			do {
				Collision = false;
				for(std::uint32_t At = Offsets[Index]; At < Offsets[Index + 1]; ++At) {
					const std::uint32_t Next = Neighbors[At].Vertex;
					if(IsOcean[Next] || !HeightFinalized[Next] || ChosenHeights[Next] != Height) continue;
					Height = std::nextafter(Height, std::numeric_limits<double>::infinity());
					Collision = true;
					break;
				}
			} while(Collision);
			if(!std::isfinite(Height) || Height > OriginalHeightsMeters[Index] + UpperBound + 1E-8) {
				// 最小変更幅の数値許容を超える同高度解消は採用しない返戻
				return Fail(AdaptiveDrainageError::NumericFailure);
			}
			ChosenHeights[Index] = Height;
			HeightFinalized[Index] = true;
		}
		Result.HeightsMeters.assign(OriginalHeightsMeters.begin(), OriginalHeightsMeters.end());
		Result.DownstreamVertices = std::move(ChosenParents);
		Result.Stats.MinimumLandDropMeters = std::numeric_limits<double>::infinity();
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(IsOcean[Index]) continue;
			const std::uint32_t Parent = Result.DownstreamVertices[Index];
			const double ParentHeight = IsOcean[Parent] ? Settings.SeaLevelMeters : ChosenHeights[Parent], Height = ChosenHeights[Index];
			const double Drop = Height - ParentHeight;
			const bool InvalidSurface = !(Drop > 0) || Drop < ChosenParentDrops[Index] || !(Settings.RadiusMeters + Height > 0) ||
			!std::isfinite(Settings.RadiusMeters + Height) || !std::isfinite(Height);
			// 親辺の厳密下降又は有限な地表半径を失った結果の返戻
			if(InvalidSurface) return Fail(AdaptiveDrainageError::NumericFailure);
			Result.HeightsMeters[Index] = Height;
			Result.Stats.MinimumLandDropMeters = std::min(Result.Stats.MinimumLandDropMeters, Drop);
			const double Change = Height - OriginalHeightsMeters[Index];
			if(std::abs(Change) > 0.01) ++Result.Stats.ChangedLandVertices;
			if(Parent != Fill->GetDownstreamVertices()[Index]) ++Result.Stats.ChangedParentVertices;
			Result.Stats.MaximumRaiseMeters = std::max(Result.Stats.MaximumRaiseMeters, Change);
			Result.Stats.MaximumCutMeters = std::max(Result.Stats.MaximumCutMeters, -Change);
			Result.Stats.MaximumAbsoluteChangeMeters = std::max(Result.Stats.MaximumAbsoluteChangeMeters, std::abs(Change));
			Result.Stats.SumAbsoluteChangeMeters += std::abs(Change);
		}
		// 実際の補正量が探索した上界を超えた場合の返戻
		if(Result.Stats.MaximumAbsoluteChangeMeters > UpperBound + 1E-8) return Fail(AdaptiveDrainageError::NumericFailure);
		SphericalDrainageBaselineError BaselineError = SphericalDrainageBaselineError::None;
		const std::optional<SphericalDrainageBaseline> Baseline =
		SphericalDrainageBaseline::Create(Directions, Triangles, Result.HeightsMeters, Settings.SeaLevelMeters, BaselineError, Stop);
		// 独立した全球網で大洋到達が確認出来ない場合の返戻
		if(!Baseline) return Fail(Stop.stop_requested() ? AdaptiveDrainageError::Cancelled : AdaptiveDrainageError::DrainageRejected);
		// 流路木と標高を共に選んだ有限網解の返戻
		return Result;
	}

	/**
	 * 全球網上に準備した標高を返す
	 * @return 共有節点の地表標高
	 */
	std::span<const double> AdaptiveDrainageTopology::GetHeightsMeters() const {
		// 準備済みの全球節点標高を参照する返戻
		return HeightsMeters;
	}

	/**
	 * 各節点の下流親を返す
	 * @return 大洋へ到達する親節点番号
	 */
	std::span<const std::uint32_t> AdaptiveDrainageTopology::GetDownstreamVertices() const {
		// 準備済みの大洋へ至る親辺を参照する返戻
		return DownstreamVertices;
	}

	/**
	 * 最大補正と流路変更の統計を返す
	 * @return 準備済みの比較統計
	 */
	const AdaptiveDrainageStatistics &AdaptiveDrainageTopology::GetStatistics() const {
		// 補正と親辺の比較統計を参照する返戻
		return Stats;
	}
}
