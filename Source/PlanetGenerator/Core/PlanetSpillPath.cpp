#include "PlanetSpillPath.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <tuple>
#include <utility>

namespace PlanetCore {
	/**
	 * 辺の下限で候補を並べ，確定に必要な辺だけを実測する最小最大路探索
	 * 計算量：O((V + E) log E)；各辺の高価な実測は高々一回，記憶量 O(V + E)
	 * @param Sites 不変な節点標高；此の照会は指定根一つだけを使う
	 * @param LowerBounds 辺の通過高度の下限；各端点標高以上で，実測値以下
	 * @param Start 経路の出発点
	 * @param Root 外周又は実際の到達点
	 * @param MeasureEdge 指定辺の実際の通過高度；評価不足又は非有限な辺は空
	 * @param Stop 中断要求
	 * @return 全辺を実測した始点から根への最小最大路；部分的な経路は空
	 */
	std::optional<MeasuredSpillPath> FindMeasuredSpillPath(
		const std::span<const SpillGraphSite> Sites,
		const std::span<const SpillGraphEdge> LowerBounds,
		const std::uint32_t Start,
		const std::uint32_t Root,
		const std::function<std::optional<double>(std::uint32_t)> &MeasureEdge,
		const std::stop_token Stop
	) {
		// 記憶量・番号・照会関数の有効性
		const bool Valid =
		MeasureEdge && !Sites.empty() && Sites.size() < 1000001 && LowerBounds.size() < 4000001 && Start < Sites.size() &&
		Root < Sites.size();
		// 確保前の不正入力又は中断への返戻
		if(!Valid || Stop.stop_requested()) return {};
		using Link = std::pair<std::uint32_t, std::uint32_t>;
		std::vector<std::vector<Link>> Neighbors(Sites.size());
		// 非有限な節点を未到達と混同しない返戻
		for(const SpillGraphSite &Site : Sites) if(!std::isfinite(Site.ElevationMeters) || Stop.stop_requested()) return {};
		for(std::uint32_t I = 0; I < LowerBounds.size(); ++I) {
			const SpillGraphEdge &Edge = LowerBounds[I];
			const std::uint32_t A = Edge.Nodes[0], B = Edge.Nodes[1];
			// 不正な辺番号又は非有限の下限への返戻
			if(A >= Sites.size() || B >= Sites.size() || A == B || !std::isfinite(Edge.CrestMeters) || Stop.stop_requested()) return {};
			// 端点其の物より低い偽の通過高度への返戻
			if(Edge.CrestMeters < std::max(Sites[A].ElevationMeters, Sites[B].ElevationMeters)) return {};
			Neighbors[A].push_back({ B, I });
			Neighbors[B].push_back({ A, I });
		}
		// 同じ水位では節点番号，続いて元の辺投入順で確定し，全辺実測版の順序を維持する
		using Entry = std::tuple<double, std::uint32_t, std::uint64_t, std::uint32_t, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
		std::vector<bool> Settled(Sites.size(), false), Measured(LowerBounds.size(), false);
		std::vector<double> Levels(Sites.size(), 0), Crests(LowerBounds.size(), 0);
		std::vector<std::uint32_t> Parents(Sites.size(), MissingSpillIndex), ParentEdges(Sites.size(), MissingSpillIndex);
		std::uint64_t Order = 0;
		const auto Expand = [&](const std::uint32_t From) -> void {
			for(const Link Adjacent : Neighbors[From]) {
				if(Settled[Adjacent.first]) continue;
				const double Bound = Measured[Adjacent.second] ? Crests[Adjacent.second] : LowerBounds[Adjacent.second].CrestMeters;
				Frontier.emplace(std::max(Levels[From], Bound), Adjacent.first, Order++, From, Adjacent.second);
			}
			// 終了
			return;
		};
		Settled[Root] = true;
		Levels[Root] = Sites[Root].ElevationMeters;
		Expand(Root);
		MeasuredSpillPath Result;
		while(!Frontier.empty() && !Settled[Start]) {
			// 候補間の中断への返戻
			if(Stop.stop_requested()) return {};
			const Entry Candidate = Frontier.top();
			Frontier.pop();
			const double Level = std::get<0>(Candidate);
			const std::uint32_t Node = std::get<1>(Candidate), From = std::get<3>(Candidate), Edge = std::get<4>(Candidate);
			if(Settled[Node]) continue;
			if(!Measured[Edge]) {
				const double Lower = LowerBounds[Edge].CrestMeters;
				const std::optional<double> Value = MeasureEdge(Edge);
				++Result.EvaluatedEdges;
				// 下限違反又は未計測の辺を確定しない返戻
				if(!Value || !std::isfinite(*Value) || *Value < Lower || Stop.stop_requested()) return {};
				Crests[Edge] = *Value;
				Measured[Edge] = true;
			}
			const double Actual = std::max(Levels[From], Crests[Edge]);
			if(Actual > Level) {
				// 実測で上がった候補は並べ直し，他の低い経路を先に調べる
				Frontier.emplace(Actual, Node, std::get<2>(Candidate), From, Edge);
				continue;
			}
			Settled[Node] = true;
			Levels[Node] = Actual;
			Parents[Node] = From;
			ParentEdges[Node] = Edge;
			if(Node != Start) Expand(Node);
		}
		// 根へ届かない成分を公開しない返戻
		if(!Settled[Start] || Stop.stop_requested()) return {};
		Result.CrestMeters = Levels[Start];
		std::uint32_t Node = Start;
		for(std::size_t I = 0; I < Sites.size() && Node != Root; ++I) {
			Result.Nodes.push_back(Node);
			const std::uint32_t Edge = ParentEdges[Node];
			// 復元する辺も実測と有効番号を確認する返戻
			if(Edge >= Measured.size() || !Measured[Edge] || Parents[Node] >= Sites.size()) return {};
			Result.Edges.push_back(Edge);
			Node = Parents[Node];
		}
		// 循環した又は中断された復元への返戻
		if(Node != Root || Stop.stop_requested()) return {};
		Result.Nodes.push_back(Root);
		// 全体の排水階層とは区別した一経路の返戻
		return Result;
	}

	/**
	 * 下限で得た候補経路の未実測辺だけを測り，最低通過高度を確定する
	 * 計算量：O((V + E) log E) の探索を高々３２回；其の後は辺選択の既存算法へ移る
	 * @param Sites 不変な節点標高
	 * @param LowerBounds 同じ辺の端点標高以上・実際の通過高度以下の下限
	 * @param Start 経路の出発点
	 * @param Root 実終点又は仮想外周根
	 * @param MeasureEdge 元場で指定辺を実測する照会；各辺は高々一回
	 * @param Stop 元場照会前と探索中に伝播する中断要求
	 * @return 全辺を実測した有限網の最低通過経路；下限違反・欠落・中断では空
	 * @note 候補の下限と全実辺の最大値が一致すれば，最大演算の単調性から有限網内の最適性が成立する
	 */
	std::optional<MeasuredSpillPath> FindPathMeasuredSpillPath(
		const std::span<const SpillGraphSite> Sites,
		const std::span<const SpillGraphEdge> LowerBounds,
		const std::uint32_t Start,
		const std::uint32_t Root,
		const std::function<std::optional<double>(std::uint32_t)> &MeasureEdge,
		const std::stop_token Stop
	) {
		// 複製する記憶量と入力番号を，確保前に確認する
		const bool Valid =
		MeasureEdge && !Sites.empty() && Sites.size() < 1000001 && LowerBounds.size() < 4000001 && Start < Sites.size() &&
		Root < Sites.size();
		// 不正な入力又は開始前の中断への返戻
		if(!Valid || Stop.stop_requested()) return {};
		std::vector<SpillGraphEdge> Current(LowerBounds.begin(), LowerBounds.end());
		std::vector<bool> Measured(Current.size(), false);
		std::uint32_t Evaluations = 0;
		const auto Measure = [&](const std::uint32_t Edge) -> std::optional<double> {
			// 同じ辺を重ねて元場へ照会しない返戻
			if(Measured[Edge]) return Current[Edge].CrestMeters;
			// 中断後の元場照会を行わない返戻
			if(Stop.stop_requested()) return {};
			const std::optional<double> Actual = MeasureEdge(Edge);
			++Evaluations;
			// 未計測・非有限又は下限に反する辺を確定しない返戻
			if(!Actual || !std::isfinite(*Actual) || *Actual < Current[Edge].CrestMeters || Stop.stop_requested()) return {};
			Current[Edge].CrestMeters = *Actual;
			Measured[Edge] = true;
			// 元場で実測した同じ辺の返戻
			return *Actual;
		};
		const auto Estimate = [&](const std::uint32_t Edge) -> std::optional<double> {
			// 元場を呼出さず，既知の実測値又は成立する下限の返戻
			return Current[Edge].CrestMeters;
		};
		for(unsigned Round = 0; Round < 32; ++Round) {
			std::optional<MeasuredSpillPath> Candidate = FindMeasuredSpillPath(Sites, Current, Start, Root, Estimate, Stop);
			// 未接続・不正な辺又は中断への返戻
			if(!Candidate) return {};
			double ActualMaximum = Candidate->CrestMeters;
			for(const std::uint32_t Edge : Candidate->Edges) {
				const std::optional<double> Actual = Measure(Edge);
				// 部分的な実測経路を公開しない返戻
				if(!Actual) return {};
				ActualMaximum = std::max(ActualMaximum, *Actual);
			}
			if(ActualMaximum == Candidate->CrestMeters) {
				Candidate->EvaluatedEdges = Evaluations;
				// 全実辺の最大値が全経路の下限と一致した経路の返戻
				return Candidate;
			}
		}
		// 繰返す候補探索の費用を有界にし，既知の実測辺を保持して従来の確定順へ移る
		std::optional<MeasuredSpillPath> Result = FindMeasuredSpillPath(Sites, Current, Start, Root, Measure, Stop);
		if(Result) Result->EvaluatedEdges = Evaluations;
		// 精度を緩めずに，既存の辺選択で確定した経路の返戻
		return Result;
	}
}
