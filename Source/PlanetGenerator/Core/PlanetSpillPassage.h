#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct SpillPassageSettings {
		// 物理的な探索範囲と有限な準備予算
		double RadiusMeters = 1000000;
		double SpacingMeters = 50;
		double HalfWidthMeters = 2000;
		double MaximumSampleStepMeters = 50;
		std::uint32_t MaximumNodes = 65536;
		std::uint32_t MaximumEvaluations = 4000000;
		// 窓探索のみで使う出口の前方，通常の両端通路には適用しない
		std::optional<UnitDirection> WindowExitTarget;
		// 窓の比較検証用；通常は必要な辺を確定時に実測する
		bool MeasureAllWindowEdges = false;
		// 両端通路の比較検証用；通常は通過高度・外周・最短路に必要な実辺だけを測る
		bool MeasureAllPassageEdges = false;
		// 両端通路又は外周窓の候補経路を先に選び，経路上の実辺を測る；偽は従来順との比較
		bool UsePathEdgeSelection = false;
		// 候補格子だけを保持上限に合わせ，選択辺の実測間隔は変更しない
		bool AdaptCandidateSpacingToBudget = false;
		// 岸の正確な端点を含むセルの周囲一リングにも実辺を結び，格子線の片側だけへ拘束しない
		bool UseEndpointNeighborRing = false;
		// 窓の四隅の解析勾配から，補間場の鞍点候補も拾う；実鞍点・両枝は後段で確認する
		bool UseCellCriticalCandidates = false;
	};

	struct SpillPassage {
		// 局所通過条件と場の評価費用
		UnitDirection Peak;
		double CrestMeters = 0;
		double PathLengthMeters = 0;
		double MaximumIndependentExcessMeters = 0;
		std::array<double, 2> BoundaryEscapeMeters {};
		double OutsideLowerBoundMeters = 0;
		bool IsGraphInteriorMinimum = false;
		std::uint64_t Evaluations = 0;
		std::uint32_t Nodes = 0;
		// 実際に使用した候補格子の間隔；元場の評価精度とは区別する
		double CandidateSpacingMeters = 0;
		std::uint32_t PathNodes = 0;
		std::uint32_t EvaluatedEdges = 0;
		std::uint32_t TotalEdges = 0;
		bool TouchesBoundary = false;
		std::vector<UnitDirection> Path;
		// 最高の峠だけで，中間の低い接続を取りこぼさない候補列
		std::vector<UnitDirection> PassCandidates;
	};

	/** 有限の探索窓の外周へ出る為の通過高度；大洋への接続又は全球最低出口の証明ではない */
	struct SpillWindowExit {
		// 元の場での最高点，通過高度と外周の最低標高
		UnitDirection Peak;
		double CrestMeters = 0;
		double MinimumBoundaryMeters = 0;
		// 有限な準備費用と，窓拡張の判断に使う最高点の位置
		std::uint64_t Evaluations = 0;
		std::uint32_t Nodes = 0;
		std::uint32_t EvaluatedEdges = 0;
		bool PeakOnBoundary = false;
		// 標高の局所最高点とは独立に，同じ窓の勾配から得た候補数
		std::uint32_t CellCriticalCandidates = 0;
		std::vector<UnitDirection> Path;
		// 経路内の局所最高点候補；実鞍点である事は元場で別途確認
		std::vector<UnitDirection> PassCandidates;
	};

	/** 元場上の有限折線を測った結果；格子内の最小経路又は全極大の証明ではない */
	struct SpillPathCrest {
		// 経路上で測った最高点と，物理長・元場の実照会数
		UnitDirection Peak;
		double CrestMeters = 0;
		double MaximumIndependentExcessMeters = 0;
		double LengthMeters = 0;
		std::uint64_t Evaluations = 0;
		// 隣接辺より高い実測最高点；実鞍点の確認は後段で行う
		std::vector<UnitDirection> PassCandidates;
	};

	std::optional<SpillPathCrest> MeasureSpillPathCrest(
		const std::span<const UnitDirection> Path,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillPassageSettings &Settings,
		const std::stop_token Stop= {}
	);

	std::optional<SpillWindowExit> FindSpillWindowExit(
		const UnitDirection Center,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillPassageSettings &Settings,
		const std::stop_token Stop= {}
	);

	std::optional<SpillPassage> FindSpillPassage(
		const UnitDirection Start,
		const UnitDirection End,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillPassageSettings &Settings,
		const std::stop_token Stop= {}
	);
}
