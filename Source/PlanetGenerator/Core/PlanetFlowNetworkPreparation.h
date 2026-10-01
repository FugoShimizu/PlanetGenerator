#pragma once

#include "PlanetFlowNetwork.h"
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	// 一つの不変な元場・生成条件へ束縛した，有界な下流側の準備
	class FlowNetworkPreparation {
	private:

		struct Impl;
		std::unique_ptr<Impl> State;

	public:

		std::optional<PreparedFlowNetwork> Prepare(
			const SpillNetwork &Network,
			const std::stop_token Stop= {},
			FlowNetworkDiagnostics *const Diagnostics=nullptr
		);

		FlowNetworkPreparation &operator=(const FlowNetworkPreparation &) = delete;

		FlowNetworkPreparation(
			const std::function<TerrainSample(UnitDirection)> &Source,
			const std::function<double(UnitDirection)> &SourceHeight,
			const FlowNetworkSettings &Settings
		);

		FlowNetworkPreparation(const FlowNetworkPreparation &) = delete;

		~FlowNetworkPreparation();
	};
}
