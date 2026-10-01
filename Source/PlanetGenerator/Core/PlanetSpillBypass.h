#pragma once

#include "PlanetSpillNetwork.h"
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>

namespace PlanetCore {
	struct SpillBypassSettings {
		// 元の水系と同じ尺度・海面；全試行と再接続を合算した有限な作業量
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		std::uint32_t MaximumEvaluations = 4000000;
		// 同じ不変な元場の準備にだけ属する厳密な標本；０は比較用の再利用無し
		std::uint32_t MaximumCachedSamples = 262144;
		std::uint32_t MaximumConnections = 1024;
		std::uint32_t MaximumBranchSamples = 524288;
		// 実接続で低い大洋出口が未確認の場合だけ緩和する；偽は従来経路の比較用
		bool RelaxOnlyWhenNeeded = true;
		// 繰返しの途中でも同じ実接続を確認；偽は最大反復後だけ確認する比較用
		bool UseConnectionCheckpoints = true;
		// 同じ元場で確認済の峠の両枝を再追跡しない；偽は比較用
		bool ReusePreparedConnections = true;
		// 格子通路の未発見後も実極小から連続候補を調べる；偽は従来の早期拒否
		bool UseContinuousConnectionFallback = true;
		// 格子を先行する比較と，連続候補だけの費用・実接続の比較を切替える
		bool UseGridPassages = true;
		// 広い格子より先に，実成分の連続候補を短い有界試行で確認する
		bool PreferContinuousCandidates = false;
		std::uint32_t ContinuousEvaluationLimit = 131072;
		// 全ての成分極小の小窓・登坂を尽くした後，窓だけを此の有界範囲まで拡げる
		// ２５６ｍは従来の固定小窓の比較用
		double MaximumLocalHalfWidthMeters = 2048;
	};

	enum class SpillBypassFailure : std::uint8_t {
		None,
		InvalidInput,
		Source,
		Budget,
		NoLowerPassage,
		NoNewConnection,
		Ocean,
		Graph,
		Cancelled,
		Window
	};

	struct SpillBypassDiagnostics {
		// 元場の照会と，全窓を通した実際の追加数；失敗でも入力網を変更しない
		SpillBypassFailure Failure = SpillBypassFailure::None;
		std::uint64_t Evaluations = 0;
		std::uint64_t CacheHits = 0;
		std::uint32_t CachedSamples = 0;
		std::uint32_t Windows = 0;
		std::uint32_t MeasuredWindows = 0;
		// 格子と候補経路の実照会を制限し，成分の出口探索に同じ総予算の残りを確保する
		std::uint64_t PassageCandidateEvaluations = 0;
		bool PassageCandidateBudgetExhausted = false;
		// 格子で未確認だった候補の連続な絞込みと，全辺再測定の証跡
		std::uint32_t RelaxationAttempts = 0;
		std::uint32_t MeasuredRelaxationAttempts = 0;
		std::uint64_t RelaxationEvaluations = 0;
		double RelaxedCrestMeters = 0;
		bool HasLowerRelaxedPath = false;
		// 格子の証拠とは区別し，全実接続の網で確認した低い大洋出口
		bool UsedContinuousConnectionFallback = false;
		bool HasLowerConnectionRoute = false;
		// 実鞍点を補う緩和の実作業；低い経路自体を探す上記の緩和とは区別
		std::uint32_t ConnectionRelaxationAttempts = 0;
		std::uint64_t ConnectionRelaxationEvaluations = 0;
		std::uint32_t ConnectionRelaxationIterations = 0;
		std::uint32_t ConnectionCheckpointChecks = 0;
		std::uint32_t AcceptedConnectionIteration = 0;
		std::uint32_t AddedConnections = 0;
		std::uint32_t RejectedCandidates = 0;
		std::uint32_t ReusedCandidates = 0;
		std::uint32_t ClimbingAttempts = 0;
		std::uint32_t LocalWindows = 0;
		// 新たな海面終端の所属確認；候補格子の拒否と実際の乾いた通路を区別する
		std::uint32_t OceanVerificationAttempts = 0;
		std::uint64_t OceanVerificationEvaluations = 0;
		double LastOceanDistanceMeters = 0;
		double LastOceanCrestMeters = 0;
		double LastOceanSpacingMeters = 0;
		double LastOceanHalfWidthMeters = 0;
		std::optional<UnitDirection> LastOceanTerminal;
		std::optional<UnitDirection> LastOceanAnchor;
		std::optional<UnitDirection> LastOceanPeak;
		std::uint32_t LastOceanNodes = 0;
		bool HasLastOceanPassage = false;
		// 連続候補の先行試行と，未確認の時だけ使う格子の費用を別に残す
		std::uint32_t PreferredContinuousAttempts = 0;
		bool AcceptedPreferredContinuous = false;
		SpillBypassFailure FirstPreferredFailure = SpillBypassFailure::None;
		std::uint64_t PreferredContinuousEvaluations = 0;
		std::uint32_t PreferredGridFallbacks = 0;
		std::array<double, 2> EscapeMeters {};
		std::optional<UnitDirection> FirstRejectedCandidate;
		SpillConnectionDiagnostics FirstRejectedConnection;
		double LastHalfWidthMeters = 0;
		double LastSpacingMeters = 0;
		double MeasuredBypassMeters = 0;
		double OriginalPassMeters = 0;
		ContinuousSpillDiagnostics Graph;
		SpillDiscoveryDiagnostics LastWindow;
	};

	// 一つの不変な元場へ束縛し，逐次補完の間だけ厳密標本を共有する
	class SpillBypassPreparation {
	private:

		struct Impl;
		std::unique_ptr<Impl> State;

		std::optional<SpillNetwork> PrepareCandidates(
			const SpillNetwork &Original,
			const std::uint32_t OutletConnection,
			const SpillBypassSettings &Settings,
			const std::stop_token Stop,
			SpillBypassDiagnostics *const Diagnostics
		);

	public:

		std::optional<SpillNetwork> Prepare(
			const SpillNetwork &Original,
			const std::uint32_t OutletConnection,
			const SpillBypassSettings &Settings,
			const std::stop_token Stop= {},
			SpillBypassDiagnostics *const Diagnostics=nullptr
		);

		SpillBypassPreparation &operator=(const SpillBypassPreparation &) = delete;

		SpillBypassPreparation(
			const std::function<TerrainSample(UnitDirection)> &Source,
			const std::uint32_t Capacity,
			const std::span<const UnitDirection> ConnectedOceanAnchors,
			const std::function<bool(UnitDirection)> &IsConnectedOceanAnchor
		);

		SpillBypassPreparation(const std::function<TerrainSample(UnitDirection)> &Source, const std::uint32_t Capacity);

		SpillBypassPreparation(const SpillBypassPreparation &) = delete;

		~SpillBypassPreparation();
	};

	std::optional<SpillNetwork> CompleteSpillNetworkBypass(
		const SpillNetwork &Original,
		const std::uint32_t OutletConnection,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillBypassSettings &Settings,
		const std::stop_token Stop= {},
		SpillBypassDiagnostics *const Diagnostics=nullptr
	);
}
