#pragma once

#include "PlanetFlowNetworkField.h"
#include "PlanetSpillBypass.h"
#include "PlanetSpillSeed.h"
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	class OceanConnectivityField;

	enum class FlowWatershedFailure : std::uint8_t {
		None,
		InvalidOcean,
		OceanAnchors,
		Seed,
		Network,
		Channels,
		Field,
		Cancelled,
		Bypass
	};

	struct FlowWatershedDiagnostics {
		// 地点・大洋・水系・河床の各段階を区別した準備結果
		FlowWatershedFailure Failure = FlowWatershedFailure::None;
		std::uint32_t OceanAnchors = 0;
		SpillSeedDiagnostics Seed;
		SpillNetworkDiagnostics Network;
		FlowNetworkDiagnostics Channels;
		// 補完前の失敗も残し，実接続を追加した回数と最後の費用を区別
		std::uint32_t TopologyCompletions = 0;
		std::uint64_t BypassEvaluations = 0;
		std::uint64_t BypassCacheHits = 0;
		std::uint64_t BypassReusedCandidates = 0;
		std::uint32_t PreferredContinuousAttempts = 0;
		std::uint32_t AcceptedPreferredContinuous = 0;
		std::uint64_t PreferredContinuousEvaluations = 0;
		std::uint32_t PreferredGridFallbacks = 0;
		std::uint64_t BypassConnectionRelaxations = 0;
		std::uint64_t BypassConnectionRelaxationEvaluations = 0;
		std::uint64_t BypassConnectionIterations = 0;
		std::uint64_t BypassConnectionCheckpointChecks = 0;
		std::uint64_t BypassAcceptedConnectionCheckpoints = 0;
		// 格子未発見後の実候補の照会と，両大洋出口を確認した最初の実例
		std::uint32_t ContinuousConnectionFallbacks = 0;
		std::uint32_t AcceptedContinuousConnectionFallbacks = 0;
		Vec3 FirstContinuousFallbackPass;
		double FirstContinuousFallbackPassMeters = 0;
		std::array<double, 2> FirstContinuousFallbackEscapeMeters {};
		// 現在の全補完を合算した実準備時間と河道の再利用数
		double ChannelPreparationMilliseconds = 0;
		double BypassPreparationMilliseconds = 0;
		std::uint64_t ReusedChannels = 0;
		std::uint64_t RebuiltChannels = 0;
		FlowNetworkDiagnostics FirstTopologyFailure;
		SpillBypassDiagnostics Bypass;
	};

	struct PreparedFlowWatershed {
		// 同じ不変な元場と，全流出口の河床・水位・岸を所有
		std::shared_ptr<const OceanConnectivityField> Ocean;
		std::shared_ptr<const FlowNetworkField> Field;
		UnitDirection InitialMinimum;
		UnitDirection InitialPass;
	};

	// 一地点の有限水系を準備；全球の全盆地の列挙は呼出側の別責務
	std::optional<PreparedFlowWatershed> PrepareFlowWatershed(
		UnitDirection Location,
		std::shared_ptr<const OceanConnectivityField> Ocean,
		std::stop_token Stop= {},
		FlowWatershedDiagnostics *Diagnostics=nullptr
	);
}
