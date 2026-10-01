#include "PlanetRiverRoutingTerrain.h"
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
	 * 同じ連続地形の節点と辺の両岸を物理尺度で測り，水位窓を保持する
	 * 有限標本網であり，標本間の全極値又は最終河川面の接続証明ではない
	 * @param Directions 球面上の候補節点
	 * @param UndirectedEdges 往復を調べる候補辺
	 * @param Height 補正前の同じ解析標高場
	 * @param Settings 物理尺度・河床・河岸・照会予算
	 * @param Error 失敗理由
	 * @param Stop 中断要求
	 * @param WaterBodyOwnedNodes 既存水域と共有する証明済み節点，新河道の岸拘束を端点で課さない
	 * @param EdgeBankHalfWidthsMeters 候補辺ごとの岸半幅，空なら共通設定を使う
	 * @return 各向きの岸端点と内部標本を拘束した網
	 */
	std::optional<RiverRoutingTerrainGraph> PrepareRiverRoutingTerrainGraph(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 2>> UndirectedEdges,
		const std::function<double(UnitDirection)> &Height,
		const RiverRoutingTerrainSettings &Settings,
		RiverRoutingTerrainError &Error,
		const std::stop_token Stop,
		const std::span<const std::uint32_t> WaterBodyOwnedNodes,
		const std::span<const double> EdgeBankHalfWidthsMeters
	) {
		Error = RiverRoutingTerrainError::None;
		if(Stop.stop_requested()) {
			Error = RiverRoutingTerrainError::Cancelled;
			// 中断した候補網を公開しない返戻
			return {};
		}
		const bool Valid = Height && Directions.size() > 1 && Directions.size() < 1000001 && !UndirectedEdges.empty() &&
		UndirectedEdges.size() < 4000001 && std::isfinite(Settings.RadiusMeters) && !(Settings.RadiusMeters < 1000) &&
		!(Settings.RadiusMeters > 1E9) && std::isfinite(Settings.BankHalfWidthMeters) && Settings.BankHalfWidthMeters > 0 &&
		Settings.BankHalfWidthMeters < Settings.RadiusMeters * 0.01 && std::isfinite(Settings.MaximumSampleStepMeters) &&
		Settings.MaximumSampleStepMeters > 0 && !(Settings.MaximumSampleStepMeters > 1000) &&
		std::isfinite(Settings.MinimumDownstreamSlope) && Settings.MinimumDownstreamSlope > 0 && Settings.MinimumDownstreamSlope < 1 &&
		std::isfinite(Settings.MinimumDepthMeters) && Settings.MinimumDepthMeters > 0 && std::isfinite(Settings.MaximumDepthMeters) &&
		Settings.MaximumDepthMeters >= Settings.MinimumDepthMeters && std::isfinite(Settings.MaximumCutMeters) &&
		!(Settings.MaximumCutMeters < 0) && std::isfinite(Settings.FreeboardMeters) && !(Settings.FreeboardMeters < 0) &&
		Settings.MaximumEvaluations > 0 && Settings.MaximumEvaluations < 100000001;
		if(!Valid) {
			Error = RiverRoutingTerrainError::InvalidInput;
			// 不正尺度又は候補数への返戻
			return {};
		}
		if(!EdgeBankHalfWidthsMeters.empty() && EdgeBankHalfWidthsMeters.size() != UndirectedEdges.size()) {
			Error = RiverRoutingTerrainError::InvalidInput;
			// 候補辺と幅の対応が不正な返戻
			return {};
		}
		for(const double Width : EdgeBankHalfWidthsMeters) {
			if(std::isfinite(Width) && Width > 0 && Width < Settings.RadiusMeters * 0.01) continue;
			Error = RiverRoutingTerrainError::InvalidInput;
			// 不正な岸幅で地形場を呼ばない返戻
			return {};
		}
		std::vector<std::uint8_t> Owned(Directions.size(), 0);
		for(const std::uint32_t Node : WaterBodyOwnedNodes) {
			if(Stop.stop_requested()) {
				Error = RiverRoutingTerrainError::Cancelled;
				// 水域所有の検査中に中断した返戻
				return {};
			}
			if(Node >= Directions.size() || Owned[Node]) {
				Error = RiverRoutingTerrainError::InvalidInput;
				// 水域の節点指定が不正又は重複した返戻
				return {};
			}
			Owned[Node] = 1;
		}
		RiverRoutingTerrainGraph Result;
		Result.Sites.reserve(Directions.size());
		Result.Edges.reserve(UndirectedEdges.size() * 2);
		std::vector<double> Ground;
		Ground.reserve(Directions.size());
		const auto SampleHeight = [&](const UnitDirection Direction) -> std::optional<double> {
			// 中断又は照会予算の超過時に地形場を呼ばない返戻
			if(Stop.stop_requested() || Result.Evaluations >= Settings.MaximumEvaluations) return {};
			++Result.Evaluations;
			const double Value = Height(Direction);
			// 非有限な場を網へ渡さない返戻
			return std::isfinite(Value) ? std::optional<double>(Value) : std::nullopt;
		};
		for(const UnitDirection Direction : Directions) {
			if(Stop.stop_requested()) {
				Error = RiverRoutingTerrainError::Cancelled;
				// 中断した節点準備への返戻
				return {};
			}
			const std::optional<double> Value = SampleHeight(Direction);
			if(!Value) {
				Error = Result.Evaluations >= Settings.MaximumEvaluations ?
				RiverRoutingTerrainError::EvaluationBudget :
				RiverRoutingTerrainError::InvalidTerrain;
				// 照会予算又は場の不正への返戻
				return {};
			}
			const double Lower = *Value - Settings.MaximumCutMeters + Settings.MinimumDepthMeters;
			const double Upper = *Value + Settings.MaximumDepthMeters;
			if(!std::isfinite(Lower) || !std::isfinite(Upper) || Lower > Upper) {
				Error = RiverRoutingTerrainError::InvalidInput;
				// 水深と開削上限が両立しない節点への返戻
				return {};
			}
			Ground.push_back(*Value);
			Result.Sites.push_back({ Lower, Upper });
		}
		for(std::size_t EdgeIndex = 0; EdgeIndex < UndirectedEdges.size(); ++EdgeIndex) {
			const std::array<std::uint32_t, 2> Nodes = UndirectedEdges[EdgeIndex];
			const double Width = EdgeBankHalfWidthsMeters.empty() ? Settings.BankHalfWidthMeters : EdgeBankHalfWidthsMeters[EdgeIndex];
			const double Probe = Width / Settings.RadiusMeters, BankCos = std::cos(Probe), BankSin = std::sin(Probe);
			if(Stop.stop_requested()) {
				Error = RiverRoutingTerrainError::Cancelled;
				// 中断した辺準備への返戻
				return {};
			}
			if(Nodes[0] >= Directions.size() || Nodes[1] >= Directions.size() || Nodes[0] == Nodes[1]) {
				Error = RiverRoutingTerrainError::InvalidInput;
				// 不正な辺番号への返戻
				return {};
			}
			const Vec3 A = Directions[Nodes[0]].Vector(), B = Directions[Nodes[1]].Vector();
			const double Dot = A.Dot(B), Angle = std::atan2(A.Cross(B).Length(), Dot), Length = Angle * Settings.RadiusMeters;
			if(!std::isfinite(Length) || !(Length > 0) || Angle >= 3.141592653589793 - 1E-6) {
				Error = RiverRoutingTerrainError::InvalidInput;
				// 退化又は対蹠辺への返戻
				return {};
			}
			const double Count = std::ceil(Length / Settings.MaximumSampleStepMeters);
			if(!std::isfinite(Count) || Count > 1000000) {
				Error = RiverRoutingTerrainError::InvalidInput;
				// 過大な辺標本への返戻
				return {};
			}
			const std::uint32_t Segments = std::max(1U, static_cast<std::uint32_t>(Count));
			const Vec3 Along = UnitDirection::FromVector(B - A * Dot)->Vector();
			std::vector<RiverRoutingInteriorSample> Samples;
			Samples.reserve(Segments - 1);
			std::array<RiverRoutingWindow, 2> EndWindows {};
			bool Passable = true;
			for(std::uint32_t i = 0; i <= Segments; ++i) {
				if(Stop.stop_requested()) {
					Error = RiverRoutingTerrainError::Cancelled;
					// 中断した横断標本への返戻
					return {};
				}
				const double Fraction = static_cast<double>(i) / Segments, Phase = Angle * Fraction;
				const Vec3 P = A * std::cos(Phase) + Along * std::sin(Phase);
				const Vec3 Tangent = Along * std::cos(Phase) - A * std::sin(Phase);
				const Vec3 Across = P.Cross(Tangent);
				const bool OwnedEndpoint = !i ? Owned[Nodes[0]] != 0 : i == Segments && Owned[Nodes[1]] != 0;
				std::optional<double> LeftHeight, RightHeight;
				if(!OwnedEndpoint) {
					const UnitDirection Left = *UnitDirection::FromVector(P * BankCos + Across * BankSin);
					const UnitDirection Right = *UnitDirection::FromVector(P * BankCos - Across * BankSin);
					LeftHeight = SampleHeight(Left);
					RightHeight = SampleHeight(Right);
					if(!LeftHeight || !RightHeight) {
						Error = Stop.stop_requested() ? RiverRoutingTerrainError::Cancelled : Result.Evaluations >= Settings.MaximumEvaluations ?
						RiverRoutingTerrainError::EvaluationBudget :
						RiverRoutingTerrainError::InvalidTerrain;
						// 外岸の標本欠落時に一部だけ測った辺を返さない返戻
						return {};
					}
				}
				const std::optional<double> Center =
				!i ? Ground[Nodes[0]] : i == Segments ? Ground[Nodes[1]] : SampleHeight(*UnitDirection::FromVector(P));
				if(!Center) {
					Error = Stop.stop_requested() ? RiverRoutingTerrainError::Cancelled : Result.Evaluations >= Settings.MaximumEvaluations ?
					RiverRoutingTerrainError::EvaluationBudget :
					RiverRoutingTerrainError::InvalidTerrain;
					// 中心標本欠落への返戻
					return {};
				}
				const std::optional<RiverRoutingWindow> Window = OwnedEndpoint ?
				std::optional<RiverRoutingWindow>(
					RiverRoutingWindow{ *Center - Settings.MaximumCutMeters + Settings.MinimumDepthMeters, *Center + Settings.MaximumDepthMeters }
				) :
				MakeRiverRoutingWindow(
					*Center,
					std::min(*LeftHeight, *RightHeight),
					Settings.MinimumDepthMeters,
					Settings.MaximumDepthMeters,
					Settings.MaximumCutMeters,
					Settings.FreeboardMeters
				);
				if(!Window) {
					Passable = false;
					break;
				}
				if(!i) EndWindows[0] = *Window;
				else if(i == Segments) EndWindows[1] = *Window;
				else Samples.push_back({ Fraction * Length, *Window });
			}
			if(!Passable) {
				++Result.RejectedUndirectedEdges;
				continue;
			}
			RiverRoutingEdgeError ForwardError = RiverRoutingEdgeError::None, ReverseError = RiverRoutingEdgeError::None;
			std::optional<RiverRoutingEdge> Forward =
			PrepareRiverRoutingEdge(Nodes[0], Nodes[1], Length, Samples, Settings.MinimumDownstreamSlope, &ForwardError, Stop);
			if(ForwardError == RiverRoutingEdgeError::Cancelled) {
				Error = RiverRoutingTerrainError::Cancelled;
				// 中断辺への返戻
				return {};
			}
			std::vector<RiverRoutingInteriorSample> ReverseSamples;
			ReverseSamples.reserve(Samples.size());
			for(std::vector<RiverRoutingInteriorSample>::const_reverse_iterator It = Samples.crbegin(); It != Samples.crend(); ++It) {
				ReverseSamples.push_back({ Length - It->DistanceFromUpstreamMeters, It->Water });
			}
			std::optional<RiverRoutingEdge> Reverse =
			PrepareRiverRoutingEdge(Nodes[1], Nodes[0], Length, ReverseSamples, Settings.MinimumDownstreamSlope, &ReverseError, Stop);
			if(ReverseError == RiverRoutingEdgeError::Cancelled) {
				Error = RiverRoutingTerrainError::Cancelled;
				// 中断辺への返戻
				return {};
			}
			if(
				!Forward && ForwardError != RiverRoutingEdgeError::IncompatibleWindows ||
				!Reverse && ReverseError != RiverRoutingEdgeError::IncompatibleWindows
			) {
				Error = RiverRoutingTerrainError::NumericFailure;
				// 不正な転送境界への返戻
				return {};
			}
			if(Forward) {
				Forward->MinimumUpstreamWaterMeters = std::max(Forward->MinimumUpstreamWaterMeters, EndWindows[0].MinimumWaterMeters);
				Forward->MaximumUpstreamWaterMeters = EndWindows[0].MaximumWaterMeters;
				Forward->MinimumDownstreamWaterMeters = EndWindows[1].MinimumWaterMeters;
				Forward->MaximumDownstreamWaterMeters = std::min(Forward->MaximumDownstreamWaterMeters, EndWindows[1].MaximumWaterMeters);
				if(
					Forward->MinimumUpstreamWaterMeters <= Forward->MaximumUpstreamWaterMeters &&
					Forward->MinimumDownstreamWaterMeters <= Forward->MaximumDownstreamWaterMeters
				) Result.Edges.push_back(*Forward);
			}
			if(Reverse) {
				Reverse->MinimumUpstreamWaterMeters = std::max(Reverse->MinimumUpstreamWaterMeters, EndWindows[1].MinimumWaterMeters);
				Reverse->MaximumUpstreamWaterMeters = EndWindows[1].MaximumWaterMeters;
				Reverse->MinimumDownstreamWaterMeters = EndWindows[0].MinimumWaterMeters;
				Reverse->MaximumDownstreamWaterMeters = std::min(Reverse->MaximumDownstreamWaterMeters, EndWindows[0].MaximumWaterMeters);
				if(
					Reverse->MinimumUpstreamWaterMeters <= Reverse->MaximumUpstreamWaterMeters &&
					Reverse->MinimumDownstreamWaterMeters <= Reverse->MaximumDownstreamWaterMeters
				) Result.Edges.push_back(*Reverse);
			}
			if(!Forward && !Reverse) ++Result.RejectedUndirectedEdges;
		}
		// 全有限辺で岸と水位を確認した網の返戻
		return Result;
	}
}
