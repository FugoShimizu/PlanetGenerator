#pragma once

#include "PlanetFlowChannelField.h"
#include "PlanetFlowOutlet.h"
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	/** 視点を含めない河道の生成条件 */
	struct FlowPreparationSettings {
		// 元場と同じ尺度・下流向き・受入水位，生成時の淵配置
		FlowOutletSettings Outlet;
		std::uint64_t Seed = 42;
	};

	enum class FlowPreparationFailure : std::uint8_t { None, InvalidInput, Outlet, Shore, Profile, Field, Cancelled };

	struct FlowPreparationDiagnostics {
		// 拒否段階と，下位の追跡・湖岸の実結果
		FlowPreparationFailure Failure = FlowPreparationFailure::None;
		FlowOutletDiagnostics Outlet;
		SpillLakeDiagnostics Shore;
		FlowWaterProfileDiagnostics Water;
	};

	/** 元場を所有する不変場と，同じ曲線上の距離対応 */
	struct PreparedFlowChannel {
		// 表示・衝突・水面で共有する生成済の定義
		std::shared_ptr<const FlowChannelField> Field;
		std::shared_ptr<const TerrainFlowRoute> Route;
		double PassDistanceMeters = 0;
		double UpstreamWaterMeters = 0;
		std::uint32_t PoolCount = 0;
	};

	std::optional<PreparedFlowChannel> PrepareFlowChannel(
		const UnitDirection Pass,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::function<double(UnitDirection)> &SourceHeight,
		const FlowPreparationSettings &Settings,
		std::shared_ptr<const SpillLakeShore> ReceivingLake= {},
		const std::stop_token Stop= {},
		FlowPreparationDiagnostics *const Diagnostics=nullptr
	);
}
