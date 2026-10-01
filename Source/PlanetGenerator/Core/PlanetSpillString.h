#pragma once

#include "PlanetCriticalPoint.h"
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct SpillStringSettings {
		// 物理尺度，標本間隔と下降の擬似時間
		double RadiusMeters = 1000000;
		double SpacingMeters = 25;
		double TimeStepMeters = 500;
		double MaximumMoveMeters = 12.5;
		double MotionToleranceMeters = 0.001;
		// 一回の候補準備の有限予算
		std::uint32_t MaximumVertices = 8192;
		std::uint32_t MaximumIterations = 128;
		std::uint32_t MaximumEvaluations = 2000000;
		// 外部の実接続条件を確認する間隔；受理関数が無い従来経路では未使用
		std::uint32_t AcceptanceStride = 16;
	};

	/** 元場を変えない経路候補；最小エネルギー経路又は大洋接続の証明ではない */
	struct SpillString {
		// 同じ端点を保つ球面経路と，後段で実鞍点を確認すべき候補
		std::vector<UnitDirection> Path;
		std::vector<UnitDirection> PassCandidates;
		// 最終反復の移動量と経路に直交する元場の物理勾配
		double MaximumMotionMeters = 0;
		double MaximumNormalSlope = 0;
		std::uint32_t Iterations = 0;
		std::uint64_t Evaluations = 0;
		// 外部条件で終了した事を，反復変位の収束と区別する
		bool AcceptedCheckpoint = false;
	};

	using SpillStringAcceptance = std::function<bool(const SpillString &)>;

	std::optional<SpillString> RelaxSpillString(
		const std::span<const UnitDirection> Initial,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillStringSettings &Settings,
		const std::stop_token Stop= {},
		const SpillStringAcceptance &Accept= {}
	);

	struct SpillClimbSettings {
		// 球面尺度，反射力の擬似時間と局所更新の上限
		double RadiusMeters = 1000000;
		double TimeStepMeters = 5000;
		double MaximumMoveMeters = 12.5;
		double MaximumLengthMeters = 2000;
		double GradientTolerance = 1E-5;
		// 弧長で再配置する固定点数と，一回の候補準備の有限予算
		std::uint32_t Vertices = 17;
		std::uint32_t MaximumIterations = 512;
		std::uint32_t MaximumEvaluations = 100000;
	};

	enum class SpillClimbEnd : std::uint8_t { Converged, IterationLimit, LengthLimit };

	/** 谷底を固定した登坂経路；終点は別途，実鞍点と両下降枝で検証する */
	struct SpillClimb {
		// 変更しない谷底から登坂端点迄の候補経路
		std::vector<UnitDirection> Path;
		SpillClimbEnd End = SpillClimbEnd::IterationLimit;
		double LengthMeters = 0;
		double MaximumNormalSlope = 0;
		double TipGradientNorm = 0;
		std::uint32_t Iterations = 0;
		std::uint32_t Truncations = 0;
		std::uint64_t Evaluations = 0;
	};

	std::optional<SpillClimb> ClimbSpillString(
		const TerrainCriticalPoint &Minimum,
		const UnitDirection InitialTip,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillClimbSettings &Settings,
		const std::stop_token Stop= {}
	);
}
