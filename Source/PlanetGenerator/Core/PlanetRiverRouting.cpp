#include "PlanetRiverRouting.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 元標高からの最大開削，最小・最大水深，左右岸の余裕高を水位上下限へまとめる
	 * @param OriginalGroundMeters 河床を切る前の中心標高
	 * @param MinimumBankMeters 左右岸の実標高の小さい方
	 * @param MinimumDepthMeters 最低水深
	 * @param MaximumDepthMeters 最大水深
	 * @param MaximumCutMeters 中心で許す最大開削
	 * @param FreeboardMeters 左右岸で必要な余裕高
	 * @return 全ての有限標本条件が両立する時の水位窓
	 */
	std::optional<RiverRoutingWindow> MakeRiverRoutingWindow(
		const double OriginalGroundMeters,
		const double MinimumBankMeters,
		const double MinimumDepthMeters,
		const double MaximumDepthMeters,
		const double MaximumCutMeters,
		const double FreeboardMeters
	) {
		const bool Valid =
		std::isfinite(OriginalGroundMeters) && std::isfinite(MinimumBankMeters) && std::isfinite(MinimumDepthMeters) &&
		MinimumDepthMeters > 0 && std::isfinite(MaximumDepthMeters) && MaximumDepthMeters >= MinimumDepthMeters &&
		std::isfinite(MaximumCutMeters) && MaximumCutMeters >= 0 && std::isfinite(FreeboardMeters) && FreeboardMeters >= 0;
		const double Lower = OriginalGroundMeters - MaximumCutMeters + MinimumDepthMeters;
		const double Upper = std::min(OriginalGroundMeters + MaximumDepthMeters, MinimumBankMeters - FreeboardMeters);
		const bool Feasible = Valid && std::isfinite(Lower) && std::isfinite(Upper) && Lower <= Upper;
		// 中心の開削・水深と左右岸の無盛土条件が両立した窓の返戻
		return Feasible ? std::optional<RiverRoutingWindow>(RiverRoutingWindow{ Lower, Upper }) : std::nullopt;
	}

	/**
	 * 内部標本の上下限を，受入側水位の上限と上流側水位の下限へ縮約する
	 * @param Upstream 有向辺の上流節点
	 * @param Downstream 有向辺の下流節点
	 * @param LengthMeters 辺の物理長
	 * @param Samples 内部標本を上流から下流へ並べた水位窓
	 * @param MinimumDownstreamSlope 水位の最低下降勾配
	 * @param Error 辺準備の拒否理由，省略可
	 * @param Stop 中断要求
	 * @return 全内部標本で両立する転送境界，無効又は不可能なら空
	 */
	std::optional<RiverRoutingEdge> PrepareRiverRoutingEdge(
		const std::uint32_t Upstream,
		const std::uint32_t Downstream,
		const double LengthMeters,
		const std::span<const RiverRoutingInteriorSample> Samples,
		const double MinimumDownstreamSlope,
		RiverRoutingEdgeError *const Error,
		const std::stop_token Stop
	) {
		if(Error) *Error = RiverRoutingEdgeError::None;
		if(Stop.stop_requested()) {
			if(Error) *Error = RiverRoutingEdgeError::Cancelled;
			// 中断した辺の部分境界を公開しない返戻
			return {};
		}
		if(
			Upstream == Downstream || !std::isfinite(LengthMeters) || !(LengthMeters > 0) || !std::isfinite(MinimumDownstreamSlope) ||
			!(MinimumDownstreamSlope > 0) || MinimumDownstreamSlope >= 1 || Samples.size() > 1000000
		) {
			// 退化辺又は非有限な勾配への返戻
			if(Error) *Error = RiverRoutingEdgeError::InvalidInput;
			// 無効な辺定義を公開しない返戻
			return {};
		}
		RiverRoutingEdge Result;
		Result.Upstream = Upstream;
		Result.Downstream = Downstream;
		Result.LengthMeters = LengthMeters;
		double RequiredAtNext = -std::numeric_limits<double>::infinity(), NextDistance = LengthMeters;
		for(std::size_t i = Samples.size(); i > 0; --i) {
			if(Stop.stop_requested()) {
				if(Error) *Error = RiverRoutingEdgeError::Cancelled;
				// 中断した内部標本の転送境界を公開しない返戻
				return {};
			}
			const RiverRoutingInteriorSample &Sample = Samples[i - 1];
			const double Distance = Sample.DistanceFromUpstreamMeters;
			const RiverRoutingWindow &Window = Sample.Water;
			if(
				!std::isfinite(Distance) || !(Distance > 0) || !(Distance < NextDistance) || !std::isfinite(Window.MinimumWaterMeters) ||
				!std::isfinite(Window.MaximumWaterMeters) || Window.MinimumWaterMeters > Window.MaximumWaterMeters
			) {
				// 窓の逆順又は破綻への返戻
				if(Error) *Error = RiverRoutingEdgeError::InvalidWindow;
				// 逆順又は破綻した窓を公開しない返戻
				return {};
			}
			const double Required =
			std::max(Window.MinimumWaterMeters, RequiredAtNext + MinimumDownstreamSlope * (NextDistance - Distance));
			if(!std::isfinite(Required) || Required > Window.MaximumWaterMeters) {
				// 下流の最低水位を伝播すると内部窓へ収まらない返戻
				if(Error) *Error = RiverRoutingEdgeError::IncompatibleWindows;
				// 内部で下降出来ない辺を公開しない返戻
				return {};
			}
			Result.MinimumUpstreamWaterMeters =
			std::max(Result.MinimumUpstreamWaterMeters, Window.MinimumWaterMeters + MinimumDownstreamSlope * Distance);
			Result.MaximumDownstreamWaterMeters =
			std::min(Result.MaximumDownstreamWaterMeters, Window.MaximumWaterMeters - MinimumDownstreamSlope * (LengthMeters - Distance));
			RequiredAtNext = Required;
			NextDistance = Distance;
		}
		if(
			std::isnan(Result.MinimumUpstreamWaterMeters) || std::isnan(Result.MaximumDownstreamWaterMeters) ||
			Result.MinimumUpstreamWaterMeters == std::numeric_limits<double>::infinity() ||
			Result.MaximumDownstreamWaterMeters == -std::numeric_limits<double>::infinity()
		) {
			// 算術桁溢れを経路網へ渡さない返戻
			if(Error) *Error = RiverRoutingEdgeError::NumericFailure;
			// 桁溢れの辺を公開しない返戻
			return {};
		}
		// 内部窓の下降拘束を保持した有向辺の返戻
		return Result;
	}

	/**
	 * 受入点から上流へ，厳密下降を守る最小水位を優先して伝播する
	 * 節点ごとの連続水位区間を保持し，下流端の最低水位での取りこぼしを防ぐ
	 * @param Sites 節点の最低水深・最大水深・最大開削・河岸から得た水位窓
	 * @param Edges 内部標本を条件付けた有向辺
	 * @param Source 解を求める水源
	 * @param Receivers 固定水位を持つ大洋接続済みの受入節点群
	 * @param MinimumDownstreamSlope 全辺に課す最低下降勾配
	 * @param Error 失敗理由
	 * @param Stop 中断要求
	 * @param Diagnostics 経路不成立時に受入点からの到達水位を受取る場合の参照先
	 * @return 有限網の水源から受入点への折線と節点水位，連続面の保証ではない
	 */
	std::optional<RiverRoutingResult> FindRiverRoutingPath(
		const std::span<const RiverRoutingWindow> Sites,
		const std::span<const RiverRoutingEdge> Edges,
		const std::uint32_t Source,
		const std::span<const RiverRoutingReceiver> Receivers,
		const double MinimumDownstreamSlope,
		RiverRoutingError &Error,
		const std::stop_token Stop,
		RiverRoutingDiagnostics *const Diagnostics
	) {
		Error = RiverRoutingError::None;
		if(Diagnostics) *Diagnostics = {};
		if(Stop.stop_requested()) {
			Error = RiverRoutingError::Cancelled;
			// 部分経路を公開しない返戻
			return {};
		}
		if(
			Sites.size() < 2 || Sites.size() > 1000000 || Edges.empty() || Edges.size() > 8000000 || Source >= Sites.size() ||
			Receivers.empty() || Receivers.size() > Sites.size() || !std::isfinite(MinimumDownstreamSlope) ||
			!(MinimumDownstreamSlope > 0) || MinimumDownstreamSlope >= 1
		) {
			Error = RiverRoutingError::InvalidInput;
			// 無効な網又は境界条件への返戻
			return {};
		}
		// 逆向きの閉路判定に使う入次数
		std::vector<std::uint32_t> ReverseIndegree(Sites.size(), 0);
		for(const RiverRoutingWindow &Site : Sites) {
			if(Stop.stop_requested()) {
				Error = RiverRoutingError::Cancelled;
				// 水位窓の検証中も部分網を公開しない返戻
				return {};
			}
			if(
				!std::isfinite(Site.MinimumWaterMeters) || !std::isfinite(Site.MaximumWaterMeters) ||
				Site.MinimumWaterMeters > Site.MaximumWaterMeters
			) {
				Error = RiverRoutingError::InvalidInput;
				// 破綻した節点水位窓への返戻
				return {};
			}
		}
		std::vector<std::uint32_t> Heads(Sites.size(), UINT32_MAX), Next(Edges.size(), UINT32_MAX);
		for(std::uint32_t i = 0; i < Edges.size(); ++i) {
			if(Stop.stop_requested()) {
				Error = RiverRoutingError::Cancelled;
				// 辺の検証中も部分網を公開しない返戻
				return {};
			}
			const RiverRoutingEdge &Edge = Edges[i];
			if(
				Edge.Upstream >= Sites.size() || Edge.Downstream >= Sites.size() || Edge.Upstream == Edge.Downstream ||
				!std::isfinite(Edge.LengthMeters) || !(Edge.LengthMeters > 0) || std::isnan(Edge.MinimumUpstreamWaterMeters) ||
				Edge.MinimumUpstreamWaterMeters == std::numeric_limits<double>::infinity() || std::isnan(Edge.MaximumDownstreamWaterMeters) ||
				Edge.MaximumDownstreamWaterMeters == -std::numeric_limits<double>::infinity() || std::isnan(Edge.MaximumUpstreamWaterMeters) ||
				Edge.MaximumUpstreamWaterMeters == -std::numeric_limits<double>::infinity() || std::isnan(Edge.MinimumDownstreamWaterMeters) ||
				Edge.MinimumDownstreamWaterMeters == std::numeric_limits<double>::infinity() ||
				Edge.MinimumUpstreamWaterMeters > Edge.MaximumUpstreamWaterMeters ||
				Edge.MinimumDownstreamWaterMeters > Edge.MaximumDownstreamWaterMeters
			) {
				Error = RiverRoutingError::InvalidEdge;
				// 不正辺を経路の一部として黙認しない返戻
				return {};
			}
			Next[i] = Heads[Edge.Downstream];
			Heads[Edge.Downstream] = i;
			++ReverseIndegree[Edge.Upstream];
		}
		std::vector<std::uint32_t> Topological;
		Topological.reserve(Sites.size());
		for(std::uint32_t Node = 0; Node < Sites.size(); ++Node) if(!ReverseIndegree[Node]) Topological.push_back(Node);
		for(std::size_t Position = 0; Position < Topological.size(); ++Position) {
			if(Stop.stop_requested()) {
				Error = RiverRoutingError::Cancelled;
				// 巡回判定中も部分経路を公開しない
				return {};
			}
			for(std::uint32_t i = Heads[Topological[Position]]; i != UINT32_MAX; i = Next[i]) {
				const std::uint32_t Upstream = Edges[i].Upstream;
				if(!--ReverseIndegree[Upstream]) Topological.push_back(Upstream);
			}
		}
		const bool IsAcyclic = Topological.size() == Sites.size();
		struct Label {
			double MinimumWaterMeters = 0;
			double MaximumWaterMeters = 0;
			std::uint32_t Node = UINT32_MAX;
			std::uint32_t ParentLabel = UINT32_MAX;
			std::uint32_t ParentEdge = UINT32_MAX;
			bool Active = true;
		};
		using State = std::pair<double, std::uint32_t>;
		std::priority_queue<State, std::vector<State>, std::greater<State>> Queue;
		std::vector<Label> Labels;
		std::vector<std::vector<std::uint32_t>> LabelsAtNode(Sites.size());
		std::vector<double> MinimumWater(Sites.size(), std::numeric_limits<double>::infinity());
		std::vector<std::uint8_t> IsReceiver(Sites.size(), 0);
		std::vector<std::uint32_t> PathMarks(Sites.size(), 0);
		std::uint32_t MarkSerial = 0;
		for(const RiverRoutingReceiver &Receiver : Receivers) {
			if(Stop.stop_requested()) {
				Error = RiverRoutingError::Cancelled;
				// 受入口の検証中も部分経路を公開しない
				return {};
			}
			if(
				Receiver.Node >= Sites.size() || IsReceiver[Receiver.Node] || Receiver.Node == Source || !std::isfinite(Receiver.WaterMeters) ||
				Receiver.WaterMeters < Sites[Receiver.Node].MinimumWaterMeters || Receiver.WaterMeters > Sites[Receiver.Node].MaximumWaterMeters
			) {
				Error = RiverRoutingError::InvalidInput;
				// 受入口の重複・水位窓の破綻への返戻
				return {};
			}
			IsReceiver[Receiver.Node] = 1;
			const std::uint32_t Id = static_cast<std::uint32_t>(Labels.size());
			Labels.push_back({ Receiver.WaterMeters, Receiver.WaterMeters, Receiver.Node });
			LabelsAtNode[Receiver.Node].push_back(Id);
			MinimumWater[Receiver.Node] = Receiver.WaterMeters;
			Queue.push({ Receiver.WaterMeters, Id });
		}
		const std::size_t MaximumLabels = std::min<std::size_t>(1000000, Edges.size() * 8 + Receivers.size());
		RiverRoutingResult Result;
		std::uint32_t SourceLabel = UINT32_MAX;
		while(!Queue.empty()) {
			if(Stop.stop_requested()) {
				Error = RiverRoutingError::Cancelled;
				// 中断した探索から部分経路を公開しない
				return {};
			}
			const State StateValue = Queue.top();
			Queue.pop();
			const Label Current = Labels[StateValue.second];
			if(!Current.Active) continue;
			++Result.SettledNodes;
			if(Current.Node == Source) {
				SourceLabel = StateValue.second;
				break;
			}
			for(std::uint32_t i = Heads[Current.Node]; i != UINT32_MAX; i = Next[i]) {
				if(Stop.stop_requested()) {
					Error = RiverRoutingError::Cancelled;
					// 高次数節点の探索中も部分経路を公開しない
					return {};
				}
				const RiverRoutingEdge &Edge = Edges[i];
				if(IsReceiver[Edge.Upstream]) continue;
				const double DownstreamMinimum = std::max(Current.MinimumWaterMeters, Edge.MinimumDownstreamWaterMeters);
				const double DownstreamMaximum = std::min(Current.MaximumWaterMeters, Edge.MaximumDownstreamWaterMeters);
				if(DownstreamMinimum > DownstreamMaximum) continue;
				const double UpstreamMinimum = std::max(
					{
						Sites[Edge.Upstream].MinimumWaterMeters,
						Edge.MinimumUpstreamWaterMeters,
						DownstreamMinimum + MinimumDownstreamSlope * Edge.LengthMeters
					}
				);
				const double UpstreamMaximum = std::min(Sites[Edge.Upstream].MaximumWaterMeters, Edge.MaximumUpstreamWaterMeters);
				if(!std::isfinite(UpstreamMinimum)) {
					Error = RiverRoutingError::NumericFailure;
					// 非有限の水位伝播への返戻
					return {};
				}
				if(!(UpstreamMinimum > DownstreamMinimum) || UpstreamMinimum > UpstreamMaximum) continue;
				if(!IsAcyclic) {
					if(!++MarkSerial) {
						std::fill(PathMarks.begin(), PathMarks.end(), 0);
						MarkSerial = 1;
					}
					bool Cycle = false;
					for(std::uint32_t Ancestor = StateValue.second; Ancestor != UINT32_MAX; Ancestor = Labels[Ancestor].ParentLabel) {
						const std::uint32_t Node = Labels[Ancestor].Node;
						PathMarks[Node] = MarkSerial;
						if(Node == Edge.Upstream) Cycle = true;
					}
					if(Cycle) continue;
					PathMarks[Edge.Upstream] = MarkSerial;
				}
				std::vector<std::uint32_t> &AtNode = LabelsAtNode[Edge.Upstream];
				if(IsAcyclic && AtNode.size() > 256) {
					AtNode.erase(
						std::remove_if(
							AtNode.begin(),
							AtNode.end(),
							[&Labels](const std::uint32_t Id) {
								return !Labels[Id].Active;
							}
						),
						AtNode.end()
					);
				}
				bool Dominated = false;
				for(const std::uint32_t Id : AtNode) {
					const Label &Prior = Labels[Id];
					if(!Prior.Active || Prior.MinimumWaterMeters > UpstreamMinimum || Prior.MaximumWaterMeters < UpstreamMaximum) continue;
					if(IsAcyclic) {
						Dominated = true;
						break;
					}
					bool PriorPathSubset = true;
					for(std::uint32_t Ancestor = Id; Ancestor != UINT32_MAX; Ancestor = Labels[Ancestor].ParentLabel) {
						if(PathMarks[Labels[Ancestor].Node] != MarkSerial) {
							PriorPathSubset = false;
							break;
						}
					}
					if(PriorPathSubset) {
						Dominated = true;
						break;
					}
				}
				if(Dominated) continue;
				if(Labels.size() == MaximumLabels || AtNode.size() > 1023) {
					Error = RiverRoutingError::BudgetExceeded;
					// 状態予算の不足を経路不存在と混同しない
					return {};
				}
				if(IsAcyclic) for(const std::uint32_t Id : AtNode) {
					Label &Prior = Labels[Id];
					if(Prior.Active && UpstreamMinimum <= Prior.MinimumWaterMeters && UpstreamMaximum >= Prior.MaximumWaterMeters) {
						Prior.Active = false;
					}
				}
				const std::uint32_t Id = static_cast<std::uint32_t>(Labels.size());
				Labels.push_back({ UpstreamMinimum, UpstreamMaximum, Edge.Upstream, StateValue.second, i });
				AtNode.push_back(Id);
				MinimumWater[Edge.Upstream] = std::min(MinimumWater[Edge.Upstream], UpstreamMinimum);
				Queue.push({ UpstreamMinimum, Id });
			}
		}
		if(SourceLabel == UINT32_MAX) {
			if(Diagnostics) {
				Diagnostics->MinimumRequiredWaterMeters = std::move(MinimumWater);
				Diagnostics->SettledNodes = Result.SettledNodes;
			}
			Error = RiverRoutingError::NoRoute;
			// 制約を満たす接続が無い返戻
			return {};
		}
		double Water = Labels[SourceLabel].MinimumWaterMeters;
		for(std::uint32_t Id = SourceLabel; Id != UINT32_MAX; Id = Labels[Id].ParentLabel) {
			const Label &Current = Labels[Id];
			if(!std::isfinite(Water) || Water < Current.MinimumWaterMeters || Water > Current.MaximumWaterMeters) {
				Error = RiverRoutingError::NumericFailure;
				// 復元した水位が状態窓から外れた場合の返戻
				return {};
			}
			Result.Nodes.push_back(Current.Node);
			Result.WaterMeters.push_back(Water);
			if(IsReceiver[Current.Node]) {
				Result.SelectedReceiver = Current.Node;
				break;
			}
			if(Current.ParentEdge == UINT32_MAX || Current.ParentLabel == UINT32_MAX || Result.Nodes.size() > Sites.size()) {
				Error = RiverRoutingError::NumericFailure;
				// 不完全な親鎖を公開しない返戻
				return {};
			}
			const RiverRoutingEdge &Edge = Edges[Current.ParentEdge];
			const Label &Downstream = Labels[Current.ParentLabel];
			const double NextWater = std::max(Downstream.MinimumWaterMeters, Edge.MinimumDownstreamWaterMeters);
			if(
				NextWater > Downstream.MaximumWaterMeters || NextWater > Edge.MaximumDownstreamWaterMeters ||
				!(NextWater + MinimumDownstreamSlope * Edge.LengthMeters <= Water)
			) {
				Error = RiverRoutingError::NumericFailure;
				// 各辺の水位と下降制約を保てない復元への返戻
				return {};
			}
			Result.LengthMeters += Edge.LengthMeters;
			Water = NextWater;
		}
		if(Result.SelectedReceiver == UINT32_MAX) {
			Error = RiverRoutingError::NumericFailure;
			// 受入口へ到達しない親鎖への返戻
			return {};
		}
		// 水位窓の有限網で成立した経路の返戻
		return Result;
	}

	/**
	 * 単一受入端の既存 API を複数境界の同じ探索へ渡す
	 * @param Sites 節点ごとの水位窓
	 * @param Edges 方向別の内部標本と岸端点の制約
	 * @param Source 水源節点
	 * @param Receiver 大河又は大洋へ接続済みの受入節点
	 * @param ReceiverWaterMeters 受入節点の既存水位
	 * @param MinimumDownstreamSlope 最低下降勾配
	 * @param Error 失敗理由
	 * @param Stop 中断要求
	 * @param Diagnostics 経路不成立時の到達水位
	 * @return 水源から固定受入点への有効な有限網上の経路
	 */
	std::optional<RiverRoutingResult> FindRiverRoutingPath(
		const std::span<const RiverRoutingWindow> Sites,
		const std::span<const RiverRoutingEdge> Edges,
		const std::uint32_t Source,
		const std::uint32_t Receiver,
		const double ReceiverWaterMeters,
		const double MinimumDownstreamSlope,
		RiverRoutingError &Error,
		const std::stop_token Stop,
		RiverRoutingDiagnostics *const Diagnostics
	) {
		if(Source == Receiver) {
			Error = RiverRoutingError::InvalidInput;
			if(Diagnostics) *Diagnostics = {};
			// 単一受入経路の退化を拒否する返戻
			return {};
		}
		const RiverRoutingReceiver Boundary { Receiver, ReceiverWaterMeters };
		// 既存の単一境界も同じ制約探索へ渡す返戻
		return FindRiverRoutingPath(
			Sites,
			Edges,
			Source,
			std::span<const RiverRoutingReceiver>(&Boundary, 1),
			MinimumDownstreamSlope,
			Error,
			Stop,
			Diagnostics
		);
	}
}
