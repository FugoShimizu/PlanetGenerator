#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct RiverRoutingWindow {
		double MinimumWaterMeters = 0;
		double MaximumWaterMeters = 0;
	};

	struct RiverRoutingInteriorSample {
		double DistanceFromUpstreamMeters = 0;
		RiverRoutingWindow Water;
	};

	struct RiverRoutingEdge {
		std::uint32_t Upstream = 0;
		std::uint32_t Downstream = 0;
		double LengthMeters = 0;
		double MinimumUpstreamWaterMeters = -std::numeric_limits<double>::infinity();
		double MaximumDownstreamWaterMeters = std::numeric_limits<double>::infinity();
		double MaximumUpstreamWaterMeters = std::numeric_limits<double>::infinity();
		double MinimumDownstreamWaterMeters = -std::numeric_limits<double>::infinity();
	};

	enum class RiverRoutingEdgeError : std::uint8_t {
		None,
		InvalidInput,
		InvalidWindow,
		IncompatibleWindows,
		NumericFailure,
		Cancelled
	};

	enum class RiverRoutingError : std::uint8_t {
		None,
		InvalidInput,
		InvalidEdge,
		NoRoute,
		NumericFailure,
		Cancelled,
		BudgetExceeded
	};

	struct RiverRoutingReceiver {
		std::uint32_t Node = 0;
		double WaterMeters = 0;
	};

	struct RiverRoutingDiagnostics {
		std::vector<double> MinimumRequiredWaterMeters;
		std::uint32_t SettledNodes = 0;
	};

	struct RiverRoutingResult {
		std::vector<std::uint32_t> Nodes;
		std::vector<double> WaterMeters;
		double LengthMeters = 0;
		std::uint32_t SettledNodes = 0;
		std::uint32_t SelectedReceiver = UINT32_MAX;
	};

	/** 元地面・河岸・水深・開削の有限標本制約を一つの水位窓へ変換する */
	std::optional<RiverRoutingWindow> MakeRiverRoutingWindow(
		double OriginalGroundMeters,
		double MinimumBankMeters,
		double MinimumDepthMeters,
		double MaximumDepthMeters,
		double MaximumCutMeters,
		double FreeboardMeters
	);

	/** 各辺の内部水位窓から，厳密下降を守る逆向きの水位転送境界を構成する */
	std::optional<RiverRoutingEdge> PrepareRiverRoutingEdge(
		std::uint32_t Upstream,
		std::uint32_t Downstream,
		double LengthMeters,
		std::span<const RiverRoutingInteriorSample> Samples,
		double MinimumDownstreamSlope,
		RiverRoutingEdgeError *Error=nullptr,
		std::stop_token Stop={}
	);

	/** 水位窓付き有向網で，固定受入水位から到達可能な最小上流水位の経路を探す */
	/** 複数の大洋接続済み水域端から，下降制約を守る受入端を選ぶ */
	std::optional<RiverRoutingResult> FindRiverRoutingPath(
		std::span<const RiverRoutingWindow> Sites,
		std::span<const RiverRoutingEdge> Edges,
		std::uint32_t Source,
		std::span<const RiverRoutingReceiver> Receivers,
		double MinimumDownstreamSlope,
		RiverRoutingError &Error,
		std::stop_token Stop={},
		RiverRoutingDiagnostics *Diagnostics=nullptr
	);

	std::optional<RiverRoutingResult> FindRiverRoutingPath(
		std::span<const RiverRoutingWindow> Sites,
		std::span<const RiverRoutingEdge> Edges,
		std::uint32_t Source,
		std::uint32_t Receiver,
		double ReceiverWaterMeters,
		double MinimumDownstreamSlope,
		RiverRoutingError &Error,
		std::stop_token Stop={},
		RiverRoutingDiagnostics *Diagnostics=nullptr
	);
}
