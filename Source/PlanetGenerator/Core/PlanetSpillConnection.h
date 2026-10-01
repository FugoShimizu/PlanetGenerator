#pragma once

#include "PlanetCriticalPoint.h"
#include "PlanetDescentPath.h"
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	// 海面高度への到達だけでは，一つの大洋への接続を証明しない
	enum class SpillTerminalKind : std::uint8_t { Minimum, SeaLevel };

	struct SpillConnectionSettings {
		// 球面尺度，停止水位と両枝の計算範囲
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		double MaximumBranchLengthMeters = 300000;
		double TerminalPositionToleranceMeters = 0.1;
		std::uint32_t MaximumBranchVertices = 8192;
	};

	struct SpillConnectionBranch {
		// 元場の下降経路と確認済の終端
		TerrainDescentPath Path;
		SpillTerminalKind Kind = SpillTerminalKind::Minimum;
		std::optional<TerrainCriticalPoint> Minimum; // 海面高度への終端では空
	};

	/** ０番は鞍点の最小曲率の負側，１番は正側；下流順序は湖の階層が決める */
	struct SpillConnection {
		// 同じ元場の鞍点と両側の接続
		TerrainCriticalPoint Pass;
		std::array<SpillConnectionBranch, 2> Branches;
		std::uint64_t Evaluations = 0;
	};

	enum class SpillConnectionFailure : std::uint8_t {
		None,
		InvalidInput,
		SaddleNotFound,
		InvalidDeparture,
		TraceFailed,
		UnresolvedTerminal,
		Cancelled
	};

	struct SpillConnectionDiagnostics {
		// 拒否理由と検査中の枝
		SpillConnectionFailure Failure = SpillConnectionFailure::None;
		std::uint32_t Branch = 2;
		TerrainDescentEnd TraceEnd = TerrainDescentEnd::BudgetLimit;
		double TerminalMoveMeters = 0;
		// 最後の実追跡位置と，局所極小の未制限な精査結果
		Vec3 TerminalDirection;
		double TerminalHeightMeters = 0;
		double TraceLengthMeters = 0;
		TerrainCriticalDiagnostics TerminalCritical;
	};

	std::optional<SpillConnection> PrepareSpillConnection(
		UnitDirection Candidate,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillConnectionSettings &Settings,
		std::stop_token Stop= {},
		SpillConnectionDiagnostics *Diagnostics=nullptr
	);
}
