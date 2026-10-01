#pragma once

#include "PlanetFlowPreparation.h"
#include "PlanetSpillNetwork.h"
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct FlowNetworkSettings {
		// 同じ元場の尺度・乱数条件と，一度に準備する有限な水系
		FlowPreparationSettings Channel;
		std::uint32_t MaximumChannels = 64;
		// 水系全体の岸・河道区間・水位節点の合算上限；視点の描画頂点数とは別
		std::uint32_t MaximumGeometrySamples = 262144;
	};

	enum class FlowNetworkFailure : std::uint8_t {
		None,
		InvalidInput,
		Outlet,
		ChannelBudget,
		ReceivingOrder,
		EqualWaterConnection,
		LakeSill,
		Channel,
		WaterLevel,
		Membership,
		OceanPassage,
		Cancelled,
		GeometryBudget
	};

	enum class FlowMembershipFailure : std::uint8_t { None, UpstreamMinimum, MissingRouteEnd, ReceivingLake, OceanHeight };

	struct FlowNetworkDiagnostics {
		// 部分的な水系を公開しない為の拒否位置と，下位の準備結果
		FlowNetworkFailure Failure = FlowNetworkFailure::None;
		std::uint32_t Basin = 0;
		// 受入水域を捨てずに数えた全流出口と，現在準備している実鞍点
		std::uint32_t RequiredChannels = 0;
		Vec3 OutletPassDirection;
		double UpstreamWaterMeters = 0;
		double DownstreamWaterMeters = 0;
		bool IsPositiveDownstream = false;
		std::uint32_t PreparedChannels = 0;
		// 固定した生成定義の連続部分の再利用；現在の所属・保持量は再検査する
		std::uint32_t ReusedChannels = 0;
		std::uint32_t RebuiltChannels = 0;
		std::uint32_t RetainedGeometrySamples = 0;
		std::uint32_t CoastalChannels = 0;
		// 単一湖岸では表せない同水位の連結を，準備前に識別
		std::uint32_t LakeComplexes = 0;
		std::uint32_t EqualWaterConnections = 0;
		std::uint32_t CompoundWaterBodies = 0;
		std::uint32_t LoweredSills = 0;
		// 所属拒否を上流の実極小・河道の末尾・受入側へ分け，同じ元場の位置を保持
		FlowMembershipFailure Membership = FlowMembershipFailure::None;
		std::uint32_t MembershipSite = MissingSpillIndex;
		Vec3 MembershipDirection;
		double MembershipHeightMeters = 0;
		double MembershipSignedDistanceMeters = 0;
		FlowPreparationDiagnostics Channel;
	};

	struct FlowBasinChannel {
		// 包含根・実下流と，同じ水位・岸を共有する不変な河道
		SpillNetworkOutlet Outlet;
		PreparedFlowChannel Channel;
	};

	/** 下流から上流へ準備した有限水系；複数河床の合成及び全球の全水系列挙とは区別する */
	struct PreparedFlowNetwork {
		// 大洋へ近い水域を先に置き，受入湖の岸を所有権付きで共有
		std::vector<FlowBasinChannel> Channels;
		// 準備時の補正も含む同じ元場を，公開後の地面・水面へ引継ぐ
		std::function<TerrainSample(UnitDirection)> Source;
		std::function<double(UnitDirection)> SourceHeight;
	};

	std::optional<PreparedFlowNetwork> PrepareFlowNetwork(
		const SpillNetwork &Network,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::function<double(UnitDirection)> &SourceHeight,
		const FlowNetworkSettings &Settings,
		const std::stop_token Stop= {},
		FlowNetworkDiagnostics *const Diagnostics=nullptr
	);
}
