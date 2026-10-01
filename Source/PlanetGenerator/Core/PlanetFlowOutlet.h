#pragma once

#include "PlanetCriticalPoint.h"
#include "PlanetDescentPath.h"
#include "PlanetFlowRoute.h"
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	struct FlowOutletSettings {
		// 物理尺度，両側の追跡範囲，受入水位と下降枝
		double RadiusMeters = 1000000;
		double UpstreamLengthMeters = 100;
		// 鞍点の精査位置から両下降を開始する局所距離；大きな未追跡区間を作らない
		double LaunchDistanceMeters = 1;
		double MaximumDownstreamLengthMeters = 300000;
		double DownstreamWaterMeters = 0;
		bool IsPositiveBranch = true;
		// 枝の符号を付けた元の主軸；再精査した固有ベクトルの任意な符号と混同しない
		std::optional<Vec3> ReferenceMinimumDirection;
	};

	/** 同じ鞍点を共有する湖側と河川側の曲線，水系全体の接続とは区別する */
	struct FlowOutletRoute {
		// 共有曲線と鞍点の対応，元の追跡結果
		TerrainFlowRoute Route;
		TerrainCriticalPoint Pass;
		double PassDistanceMeters = 0;
		TerrainDescentEnd UpstreamEnd = TerrainDescentEnd::LengthLimit;
		std::uint32_t Evaluations = 0;
	};

	enum class FlowOutletFailure : std::uint8_t { None, InvalidInput, Critical, Upstream, Downstream, Curve, Cancelled };

	/** 河道準備の拒否箇所と，上流・下流の追跡終端 */
	struct FlowOutletDiagnostics {
		// 準備の段階と追跡理由
		FlowOutletFailure Failure = FlowOutletFailure::None;
		TerrainDescentEnd UpstreamEnd = TerrainDescentEnd::BudgetLimit;
		TerrainDescentEnd DownstreamEnd = TerrainDescentEnd::BudgetLimit;
		std::uint32_t UpstreamVertices = 0;
		std::uint32_t DownstreamVertices = 0;
		// 停止した上流端の実値と，選択した下降軸
		Vec3 UpstreamDirection;
		Vec3 DownstreamAxis;
		double UpstreamDistanceMeters = 0;
		double UpstreamHeightMeters = 0;
		double UpstreamMinimumMoveMeters = 0;
		bool HasUpstreamMinimum = false;
		TerrainCriticalDiagnostics UpstreamCritical;
		FlowRouteDiagnostics Curve;
	};

	std::optional<FlowOutletRoute> PrepareFlowOutletRoute(
		const UnitDirection Candidate,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const FlowOutletSettings &Settings,
		const std::stop_token Stop= {},
		FlowOutletDiagnostics *const Diagnostics=nullptr,
		const std::function<double(UnitDirection)> &StopDomainDistance= {}
	);
}
