#pragma once

#include "PlanetSpillConnection.h"
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct SpillDiscoverySettings {
		// 同じ元場の尺度・水位と探索窓
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		// 指定時は此の高さ未満の実峠だけを探索し，高い接続で候補探索を終了しない
		std::optional<double> MaximumPassMeters;
		// 探索の前線を進める任意の方向，大洋所属の証明には使わない
		std::optional<UnitDirection> OutletTarget;
		// 入力集合内の実極小を明示して，既知の峠だけを持つ中心から別の前線へ進める
		std::optional<UnitDirection> PreferredCenter;
		// 真の時は有限方向の登坂候補だけを準備し，二次元窓の未発見と混同しない
		bool ClimbingOnly = false;
		// 真の時は最初の小窓だけを検査し，登坂・経路緩和・窓拡大へ進めない
		bool InitialWindowOnly = false;
		// 既知の異なる極小同士を結ぶ未登録の峠も保持する；単一の集合外出口の照会では偽
		bool IncludeInternalConnections = false;
		double InitialHalfWidthMeters = 256;
		double MaximumHalfWidthMeters = 16384;
		std::uint32_t HalfCells = 64;
		// 正の時は此の物理間隔を目標に節点数を決め，HalfCells を上限にする；０は固定密度
		double CandidateSpacingMeters = 0;
		// 接続群を探す為の全窓共通の評価・記憶予算
		std::uint32_t MaximumEvaluations = 2000000;
		std::uint32_t MaximumConnections = 1024;
		// 入力と新たな実極小を合算した保持量；通常網は接続数の予算に合わせる
		std::uint32_t MaximumKnownMinima = 1024;
		std::uint32_t MaximumBranchSamples = 262144;
		// 分離した実極小間だけを再探索する，全窓共通の上限
		std::uint32_t MaximumBridgeCorridors = 16;
		// 窓の候補が繋がらない時，実極小から方向を変えて登坂する有限な回数；０は比較用の従来経路
		std::uint32_t MaximumClimbingAttempts = 0;
	};

	enum class SpillDiscoveryFailure : std::uint8_t { None, InvalidInput, WindowFailed, BudgetLimit, NoIncidentExit, Cancelled };

	struct SpillDiscoveryDiagnostics {
		// 実際の探索費用と最後に調べた窓
		SpillDiscoveryFailure Failure = SpillDiscoveryFailure::None;
		std::uint64_t Evaluations = 0;
		std::uint32_t Windows = 0;
		// 全窓で同じ標本の解析勾配から拾った補間鞍点の候補数
		std::uint32_t CellCriticalCandidates = 0;
		// 元場で精査済だが，呼出側の峠上限を満たさなかった候補数
		std::uint32_t ExcludedHighPasses = 0;
		std::uint32_t RepeatedConnections = 0;
		std::uint32_t NewInternalConnections = 0;
		std::uint32_t ReusedConnections = 0;
		std::uint32_t UnrelatedConnections = 0;
		std::uint32_t UnresolvedPasses = 0;
		// 候補の停留点探索と，確認済の峠からの下降失敗を区別
		std::uint32_t UnresolvedCriticalPoints = 0;
		std::uint32_t UnresolvedConnections = 0;
		// 実際の停留点拒否と収束した種類を区別；追加の元場照会は行わない
		std::array<std::uint32_t, 10> CriticalFailures {};
		std::array<std::uint32_t, 3> CriticalKinds {};
		// 実候補の間隔から選んだ探索範囲；最終位置の許容値とは区別する
		double MaximumCriticalStepMeters = 0;
		double MaximumCriticalSearchRadiusMeters = 0;
		std::optional<UnitDirection> FirstFailedCriticalCandidate;
		TerrainCriticalDiagnostics FirstFailedCritical;
		std::optional<UnitDirection> CenterDirection;
		std::optional<UnitDirection> FirstFailedConnectionCandidate;
		SpillConnectionDiagnostics FirstFailedConnection;
		std::uint32_t BridgeCorridors = 0;
		std::uint32_t RejectedBridges = 0;
		// 探索予算内の補助通路の端点を保持し，長い水系探索を繰返さず再現する
		std::vector<std::array<UnitDirection, 2>> BridgeEndpoints;
		// 最初に拒否された補助通路を，同じ元場で再現する端点
		std::optional<UnitDirection> FirstRejectedBridgeStart;
		std::optional<UnitDirection> FirstRejectedBridgeEnd;
		double FirstRejectedBridgeLengthMeters = 0;
		// 連続場での候補経路の緩和と，残った幾何学的な誤差
		std::uint32_t RelaxedWindows = 0;
		std::uint32_t RelaxedBridges = 0;
		std::uint32_t RelaxationFailures = 0;
		// 一端登坂の試行と停止状態；候補の収束と実接続の確認は別々に扱う
		std::uint32_t ClimbingAttempts = 0;
		std::uint32_t ClimbingCandidates = 0;
		std::uint32_t ClimbingFailures = 0;
		std::uint32_t LastClimbEnd = 0;
		double LastClimbNormalSlope = 0;
		double LastClimbTipGradientNorm = 0;
		double LastRelaxedNormalSlope = 0;
		double LastRelaxedMotionMeters = 0;
		double LastHalfWidthMeters = 0;
		std::uint32_t LastHalfCells = 0;
	};

	struct SpillDiscoveryBatch {
		// 入力の水系へ実際に接続した未登録の峠と，新たな極小；既知極小間の別の峠も指定時に含む
		std::vector<SpillConnection> Connections;
		std::vector<TerrainCriticalPoint> NewMinima;
		// 共通の大洋への所属とは区別する海面高度への到達
		bool HasSeaHeightTerminal = false;
	};

	std::optional<SpillDiscoveryBatch> FindSpillComponentExits(
		const std::span<const TerrainCriticalPoint> Minima,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillDiscoverySettings &Settings,
		const std::stop_token Stop= {},
		SpillDiscoveryDiagnostics *const Diagnostics=nullptr,
		const std::span<const SpillConnection> CompletedConnections= {}
	);

	// 元場で確認済の有限な極小集合から，外側への一つの実接続を探す
	std::optional<SpillConnection> FindSpillComponentExit(
		const std::span<const TerrainCriticalPoint> Minima,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillDiscoverySettings &Settings,
		const std::stop_token Stop= {},
		SpillDiscoveryDiagnostics *const Diagnostics=nullptr,
		const std::span<const SpillConnection> CompletedConnections= {}
	);
}
