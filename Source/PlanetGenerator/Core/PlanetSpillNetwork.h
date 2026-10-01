#pragma once

#include "PlanetContinuousSpillGraph.h"
#include "PlanetSpillDiscovery.h"
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	// 元場の探索と海域通路の確認を合わせた準備手順の版
	std::uint32_t GetSpillNetworkVersion();

	struct SpillNetworkSettings {
		// 同じ元場の尺度・海面と全準備の計算量
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		std::uint32_t MaximumEvaluations = 32000000;
		std::uint32_t MaximumSteps = 512;
		std::uint32_t MaximumConnections = 1024;
		std::uint32_t MaximumBranchSamples = 524288;
		// 準備中だけの厳密な地点評価の再利用枠；０は比較用の再利用無
		std::uint32_t MaximumCachedSamples = 262144;
		// 小窓・登坂・広窓を順に試し，全入力成分の未試行の極小へ進む；偽は従来経路の比較用
		bool UseClimbingFrontier = true;
	};

	enum class SpillNetworkFailure : std::uint8_t {
		None,
		InvalidInput,
		InvalidSource,
		BudgetLimit,
		InitialPass,
		NoOceanAnchor,
		Discovery,
		OceanPassage,
		Graph,
		Cancelled
	};

	struct SpillNetworkDiagnostics {
		// 全体の失敗理由と実費用，最後の下位処理の診断
		SpillNetworkFailure Failure = SpillNetworkFailure::None;
		std::uint64_t Evaluations = 0;
		std::uint64_t CacheHits = 0;
		std::uint32_t CachedSamples = 0;
		std::uint32_t Steps = 0;
		// 上限時にも残る実保持量と，同じ海域目標への実距離
		std::uint32_t PreparedConnections = 0;
		std::uint32_t PreparedMinima = 0;
		std::uint32_t RetainedBranchSamples = 0;
		double InitialOceanDistanceMeters = 0;
		double ClosestFrontierDistanceMeters = 0;
		// 一つの初期海域目標とは区別する，最寄りの確認済海域への距離
		double ClosestOceanDistanceMeters = 0;
		// 一段階内の有限な中心照会数と，有限方向又は窓で未発見だった極小
		std::uint32_t CenterQueries = 0;
		std::uint32_t ExhaustedInitialWindowCenters = 0;
		std::uint32_t ExhaustedClimbingCenters = 0;
		std::uint32_t ExhaustedWindowCenters = 0;
		std::uint32_t OceanPassages = 0;
		SpillDiscoveryDiagnostics Discovery;
		ContinuousSpillDiagnostics Graph;
	};

	struct SpillOceanWitness {
		// 同じ元場で水没を検査した実終端と，所属を別途確認した海域標本
		UnitDirection Terminal;
		UnitDirection Anchor;
		double CheckedMaximumMeters = 0;
	};

	/** 一つの初期鞍点から発見した有限網；全惑星の全鞍点を列挙した物ではない */
	struct SpillNetwork {
		// 発見した接続と大洋の実終端，同じ網から求めた包含関係
		std::vector<SpillConnection> Connections;
		std::vector<SpillOceanWitness> OceanWitnesses;
		ContinuousSpillGraph Graph;
	};

	struct SpillNetworkOutlet {
		// 包含根と，元の下降枝への対応
		std::uint32_t Basin = 0;
		std::uint32_t ReceivingBasin = 0;
		std::uint32_t Connection = 0;
		bool IsPositiveDownstream = false;
		// 同じ水系で求めた実流出口と受入水域の水位
		double UpstreamWaterMeters = 0;
		double DownstreamWaterMeters = 0;
	};

	std::optional<SpillNetworkOutlet> GetSpillNetworkOutlet(const SpillNetwork &Network, const std::uint32_t Site);

	std::optional<SpillNetwork> PrepareSpillNetwork(
		const UnitDirection InitialPass,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::span<const UnitDirection> OceanAnchors,
		const std::function<bool(UnitDirection)> &IsConnectedOceanAnchor,
		const SpillNetworkSettings &Settings,
		const std::stop_token Stop= {},
		SpillNetworkDiagnostics *const Diagnostics=nullptr
	);
}
