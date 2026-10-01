#include "PlanetFlowWatershed.h"
#include "PlanetFlowNetworkPreparation.h"
#include "PlanetGeology.h"
#include "PlanetOcean.h"
#include "PlanetSpillSampleCache.h"
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * エディタの診断アクター無しで，地点から大洋接続と共有河床を準備する
	 * 計算量：時間 O(AlogA+N+F+U)，A は海域標本数，N は有限網，F は河道，U は位相補完の各累計準備費用
	 * 各下位の有界予算を保持し，接続・枝標本・水面幾何の追加空間も其の実上限で制限する
	 * @param Location 元場上の任意の陸上地点
	 * @param Ocean 所属を確認した大洋を含む不変な元場
	 * @param Stop 下位の全段階へ伝播する中断要求
	 * @param Diagnostics 公開出来ない段階と各準備費用
	 * @return 元場と全河道を所有する一水系；未確認・部分的な準備は空
	 */
	std::optional<PreparedFlowWatershed> PrepareFlowWatershed(
		const UnitDirection Location,
		std::shared_ptr<const OceanConnectivityField> Ocean,
		const std::stop_token Stop,
		FlowWatershedDiagnostics *const Diagnostics
	) {
		FlowWatershedDiagnostics Local;
		FlowWatershedDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const FlowWatershedFailure Reason) -> std::optional<PreparedFlowWatershed> {
			Report.Failure = Stop.stop_requested() ? FlowWatershedFailure::Cancelled : Reason;
			// 未完成の定義を呼出側へ公開しない返戻
			return {};
		};
		// 水位だけで孤立海域の所属を認めない返戻
		if(!Ocean || Ocean->Statistics().ComponentsAfter != 1 || Stop.stop_requested()) {
			// 未確認の大洋又は中断への返戻
			return Reject(FlowWatershedFailure::InvalidOcean);
		}
		const GeologySettings &Definition = Ocean->Base().Settings();
		// 河道の現行の大洋終端は基準水位零に対応
		if(Definition.SeaLevelMeters) return Reject(FlowWatershedFailure::InvalidOcean);
		// 公開後も存続する捕捉は共有所有権のみとし，準備中の参照や予算を残さない
		const auto Source = [Ocean](const UnitDirection D) -> TerrainSample {
			// 同じ元場の標高と勾配の返戻
			return Ocean->Evaluate(D);
		};
		const auto Height = [Ocean](const UnitDirection D) -> double {
			// 同じ元場の標高専用照会の返戻
			return Ocean->EvaluateElevation(D);
		};
		SpillSeedSettings SeedSettings;
		SeedSettings.RadiusMeters = Definition.RadiusMeters;
		SeedSettings.SeaLevelMeters = Definition.SeaLevelMeters;
		const std::optional<SpillSeed> Seed = FindSpillSeed(Location, Source, SeedSettings, Stop, &Report.Seed);
		// 直接海面へ降る地点と未発見の盆地を，架空の湖で置換えない返戻
		if(!Seed) return Reject(FlowWatershedFailure::Seed);
		std::vector<UnitDirection> Anchors;
		std::set<std::tuple<double, double, double>> Keys;
		for(const UnitDirection D : Ocean->ConnectedOceanAnchors()) {
			// 標本を収集する途中の中断への返戻
			if(Stop.stop_requested()) return Reject(FlowWatershedFailure::Cancelled);
			if(!(Height(D) < Definition.SeaLevelMeters - 1)) continue;
			Anchors.push_back(D);
			const Vec3 P = D.Vector();
			Keys.emplace(P.X, P.Y, P.Z);
		}
		Report.OceanAnchors = static_cast<std::uint32_t>(Anchors.size());
		// 水没を確認した大洋の標本が無い場合の返戻
		if(Anchors.empty()) return Reject(FlowWatershedFailure::OceanAnchors);
		const auto IsOcean = [&Keys](const UnitDirection D) -> bool {
			const Vec3 P = D.Vector();
			// 同じ準備で水没を確認した代表方向の返戻
			return Keys.contains({ P.X, P.Y, P.Z });
		};
		SpillNetworkSettings NetworkSettings;
		NetworkSettings.RadiusMeters = Definition.RadiusMeters;
		NetworkSettings.SeaLevelMeters = Definition.SeaLevelMeters;
		// 大陸内の長い接続で，実評価・保持量より先に段階数だけを使切る不整合を避ける
		NetworkSettings.MaximumSteps = 2048;
		NetworkSettings.MaximumConnections = 2048;
		// 表示の場へ持越さない厳密な準備標本；同時に一つの６４ MiB 枠だけを使用
		NetworkSettings.MaximumCachedSamples = SpillSampleCache::MaximumCapacity;
		std::optional<SpillNetwork> Network =
		PrepareSpillNetwork(Seed->Connection.Pass.Direction, Source, Anchors, IsOcean, NetworkSettings, Stop, &Report.Network);
		// 全接続が同じ大洋へ至らない場合の返戻
		if(!Network) return Reject(FlowWatershedFailure::Network);
		FlowNetworkSettings ChannelSettings;
		// 確認済の有限網に対応した全流出口を扱い，実保持量は別の合算上限で制限する
		ChannelSettings.MaximumChannels = 1024;
		// 岸・河道・水位の全標本にも，下降枝と同じ有界な保持枠を予約する
		ChannelSettings.MaximumGeometrySamples = NetworkSettings.MaximumBranchSamples;
		ChannelSettings.Channel.Seed = Definition.Seed;
		ChannelSettings.Channel.Outlet.RadiusMeters = Definition.RadiusMeters;
		FlowNetworkPreparation Preparation(Source, Height, ChannelSettings);
		SpillBypassPreparation BypassPreparation(Source, NetworkSettings.MaximumCachedSamples, Anchors, IsOcean);
		std::optional<PreparedFlowNetwork> Channels;
		while(true) {
			const std::chrono::steady_clock::time_point ChannelStart = std::chrono::steady_clock::now();
			Channels = Preparation.Prepare(*Network, Stop, &Report.Channels);
			Report.ChannelPreparationMilliseconds +=
			std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ChannelStart).count();
			Report.ReusedChannels += Report.Channels.ReusedChannels;
			Report.RebuiltChannels += Report.Channels.RebuiltChannels;
			if(Channels) break;
			const FlowNetworkDiagnostics &Failure = Report.Channels;
			const bool HasIncompleteShore = Failure.Channel.Shore.Failure == SpillLakeFailure::Contour &&
			(Failure.Channel.Shore.End == TerrainContourEnd::VertexLimit || Failure.Channel.Shore.End == TerrainContourEnd::LengthLimit);
			const bool HasInvalidShore =
			Failure.Channel.Shore.Failure == SpillLakeFailure::Boundary && Failure.Channel.Shore.AreaMetersSquared < 0;
			const bool CanCheckLowerPassage =
			Failure.Failure == FlowNetworkFailure::Channel && Failure.Channel.Failure == FlowPreparationFailure::Shore &&
			(HasIncompleteShore || HasInvalidShore) && !Failure.LoweredSills && Failure.Basin < Network->Graph.Hierarchy.Nodes.size();
			// 非帰還を低い通路の証明にせず，元場で測った補完を試みる範囲だけを選ぶ返戻
			if(!CanCheckLowerPassage) return Reject(FlowWatershedFailure::Channels);
			if(!Report.TopologyCompletions) Report.FirstTopologyFailure = Failure;
			const DepressionHierarchyNode &Basin = Network->Graph.Hierarchy.Nodes[Failure.Basin];
			const bool HasConnection = Basin.OutletEdge < Network->Graph.EdgeConnections.size() &&
			Network->Graph.EdgeConnections[Basin.OutletEdge] < Network->Connections.size();
			// 拒否した実流出口に対応しない修正を試みない返戻
			if(!HasConnection) return Reject(FlowWatershedFailure::Bypass);
			const bool HasBypassBudget = Report.Network.Evaluations < NetworkSettings.MaximumEvaluations &&
			Network->Connections.size() < NetworkSettings.MaximumConnections;
			// 元の網と追加探索に共通する予算を超える場合の返戻
			if(!HasBypassBudget) return Reject(FlowWatershedFailure::Bypass);
			SpillBypassSettings BypassSettings;
			BypassSettings.RadiusMeters = Definition.RadiusMeters;
			BypassSettings.SeaLevelMeters = Definition.SeaLevelMeters;
			BypassSettings.MaximumConnections = NetworkSettings.MaximumConnections;
			BypassSettings.MaximumCachedSamples = NetworkSettings.MaximumCachedSamples;
			BypassSettings.MaximumBranchSamples = NetworkSettings.MaximumBranchSamples;
			// 大陸規模の元場では，小窓だけで出口不存在と判定しない；実評価の共通予算は維持
			BypassSettings.MaximumLocalHalfWidthMeters = 262144;
			// 連続先行の広域比較は悪化した為，正規の格子先行を採用する
			BypassSettings.PreferContinuousCandidates = false;
			BypassSettings.MaximumEvaluations =
			static_cast<std::uint32_t>(std::min<std::uint64_t>(4000000, NetworkSettings.MaximumEvaluations - Report.Network.Evaluations));
			const std::chrono::steady_clock::time_point BypassStart = std::chrono::steady_clock::now();
			std::optional<SpillNetwork> Completed =
			BypassPreparation.Prepare(*Network, Network->Graph.EdgeConnections[Basin.OutletEdge], BypassSettings, Stop, &Report.Bypass);
			Report.BypassPreparationMilliseconds +=
			std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - BypassStart).count();
			Report.Network.Evaluations += Report.Bypass.Evaluations;
			Report.BypassEvaluations += Report.Bypass.Evaluations;
			Report.BypassCacheHits += Report.Bypass.CacheHits;
			Report.BypassReusedCandidates += Report.Bypass.ReusedCandidates;
			Report.PreferredContinuousAttempts += Report.Bypass.PreferredContinuousAttempts;
			Report.AcceptedPreferredContinuous += Report.Bypass.AcceptedPreferredContinuous;
			Report.PreferredContinuousEvaluations += Report.Bypass.PreferredContinuousEvaluations;
			Report.PreferredGridFallbacks += Report.Bypass.PreferredGridFallbacks;
			Report.BypassConnectionRelaxations += Report.Bypass.ConnectionRelaxationAttempts;
			Report.BypassConnectionRelaxationEvaluations += Report.Bypass.ConnectionRelaxationEvaluations;
			Report.BypassConnectionIterations += Report.Bypass.ConnectionRelaxationIterations;
			Report.BypassConnectionCheckpointChecks += Report.Bypass.ConnectionCheckpointChecks;
			if(Report.Bypass.AcceptedConnectionIteration) ++Report.BypassAcceptedConnectionCheckpoints;
			if(Report.Bypass.UsedContinuousConnectionFallback) {
				++Report.ContinuousConnectionFallbacks;
				if(Completed && Report.Bypass.HasLowerConnectionRoute) {
					++Report.AcceptedContinuousConnectionFallbacks;
					if(Report.AcceptedContinuousConnectionFallbacks == 1) {
						const TerrainCriticalPoint &Pass = Network->Connections[Network->Graph.EdgeConnections[Basin.OutletEdge]].Pass;
						Report.FirstContinuousFallbackPass = Pass.Direction.Vector();
						Report.FirstContinuousFallbackPassMeters = Pass.ElevationMeters;
						Report.FirstContinuousFallbackEscapeMeters = Report.Bypass.EscapeMeters;
					}
				}
			}
			// 元網を変えず，同じ失敗だけを際限無く再試行しない返戻
			if(!Completed || !Report.Bypass.AddedConnections) return Reject(FlowWatershedFailure::Bypass);
			Network = std::move(Completed);
			++Report.TopologyCompletions;
			Report.Network.PreparedConnections = static_cast<std::uint32_t>(Network->Connections.size());
			Report.Network.PreparedMinima = static_cast<std::uint32_t>(Network->Graph.Minima.size() - 1);
			Report.Network.RetainedBranchSamples = 0;
			for(const SpillConnection &Connection : Network->Connections) {
				Report.Network.RetainedBranchSamples +=
				static_cast<std::uint32_t>(Connection.Branches[0].Path.Samples.size() + Connection.Branches[1].Path.Samples.size());
			}
			Report.Network.Graph = Report.Bypass.Graph;
		}
		std::optional<FlowNetworkField> Field = FlowNetworkField::Create(*Channels);
		// 合成定義の欠落又は公開前の中断への返戻
		if(!Field || Stop.stop_requested()) return Reject(FlowWatershedFailure::Field);
		// 診断アクターの寿命と独立した準備結果の返戻
		return PreparedFlowWatershed{
			std::move(Ocean),
			std::make_shared<const FlowNetworkField>(std::move(*Field)),
			Seed->Minimum.Direction,
			Seed->Connection.Pass.Direction
		};
	}
}
