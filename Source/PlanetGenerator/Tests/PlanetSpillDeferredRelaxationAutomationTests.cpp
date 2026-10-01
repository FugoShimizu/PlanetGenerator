#include "Core/PlanetSpillBypass.h"
#include "Core/PlanetSpillPassage.h"
#include "Core/PlanetSpillSampleCache.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	struct BypassFixture {
		double Radius;
		std::function<PlanetCore::TerrainSample(PlanetCore::UnitDirection)> Source;
		PlanetCore::SpillNetwork Network;
		PlanetCore::UnitDirection Direction(const double X, const double Y) const;
		static std::optional<BypassFixture> Create(const double Radius, FString &Failure);
	};

	/**
	 * 解析場の独立な物理位置を球面上へ写す
	 * @param X 東西の無次元座標；一単位は１００ｍ
	 * @param Y 南北の無次元座標；一単位は１００ｍ
	 * @return 同じ球上の正規方向
	 */
	PlanetCore::UnitDirection BypassFixture::Direction(const double X, const double Y) const {
		// 同じ射影座標の返戻
		return *PlanetCore::UnitDirection::FromVector({ 100 * X / Radius, 100 * Y / Radius, 1 });
	}

	/**
	 * 高い北峠だけで閉じた元網を作り，未投入の低い南峠を持つ解析場で検証する
	 * @param Radius 物理半径
	 * @param Failure 入力準備に失敗した実段階
	 * @return 元場・三つの実鞍点・独立な水没通路を持つ網；準備不足は空
	 */
	std::optional<BypassFixture> BypassFixture::Create(const double Radius, FString &Failure) {
		BypassFixture Result { Radius, {}, {} };
		Result.Source = [Radius](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double Scale = Radius / 100, X = Scale * D.X / D.Z, Y = Scale * D.Y / D.Z, Q = X * X - 1, T = Y * Y - 1;
			const double H = (10 + 3 * Y) * Q * Q + 4 * T * T + 5 * X + 2 * Y, DX = 4 * (10 + 3 * Y) * X * Q + 5;
			const double DY = 3 * Q * Q + 16 * Y * T + 2;
			const PlanetCore::Vec3 G { Scale * DX / D.Z, Scale * DY / D.Z, -Scale * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 射影座標の全連鎖律から求めた解析標高と接線勾配の返戻
			return { H, G - D * G.Dot(D) };
		};
		PlanetCore::SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.RadiusMeters = Radius;
		ConnectionSettings.SeaLevelMeters = -6;
		for(const std::array<double, 2> Initial : { std::array<double, 2>{ 0, 1 }, { -1, 0 }, { 1, 0 } }) {
			PlanetCore::SpillConnectionDiagnostics Report;
			std::optional<PlanetCore::SpillConnection> Connection =
			PlanetCore::PrepareSpillConnection(Result.Direction(Initial[0], Initial[1]), Result.Source, ConnectionSettings, {}, &Report);
			if(!Connection) {
				Failure =
				FString::Printf(TEXT("connection at (%.0f,%.0f), failure=%u"), Initial[0], Initial[1], static_cast<unsigned>(Report.Failure));
				// 独立な元網の実鞍点を作成出来ない場合の返戻
				return {};
			}
			Result.Network.Connections.push_back(std::move(*Connection));
		}
		const PlanetCore::UnitDirection Anchor = Result.Direction(-1.1, -1.1);
		if(!(Result.Source(Anchor).ElevationMeters < -6)) {
			Failure = TEXT("ocean anchor height");
			// 水没していない標本を大洋へ置換えない返戻
			return {};
		}
		for(const PlanetCore::SpillConnection &Connection : Result.Network.Connections) {
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
				if(Branch.Kind != PlanetCore::SpillTerminalKind::SeaLevel) continue;
				const PlanetCore::UnitDirection Terminal = Branch.Path.Samples.back().Direction;
				PlanetCore::SpillPassageSettings Settings;
				Settings.RadiusMeters = Radius;
				Settings.HalfWidthMeters = 32;
				Settings.SpacingMeters = 4;
				Settings.MaximumSampleStepMeters = 1;
				const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(Terminal, Anchor, Result.Source, Settings);
				if(!Passage || Passage->CrestMeters + Passage->MaximumIndependentExcessMeters > -6) {
					Failure = TEXT("independent wet ocean passage");
					// 同じ元場の水没通路を確認出来ない場合の返戻
					return {};
				}
				Result.Network.OceanWitnesses.push_back({ Terminal, Anchor, Passage->CrestMeters + Passage->MaximumIndependentExcessMeters });
			}
		}
		PlanetCore::ContinuousSpillSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -6;
		const auto IsOcean = [&Result](const PlanetCore::UnitDirection D) -> bool {
			const PlanetCore::Vec3 P = D.Vector();
			for(const PlanetCore::SpillOceanWitness &Witness : Result.Network.OceanWitnesses) {
				const PlanetCore::Vec3 Q = Witness.Terminal.Vector();
				// 実際に水没を検査した終端の返戻
				if(P.X == Q.X && P.Y == Q.Y && P.Z == Q.Z) return true;
			}
			// 水位だけの海面終端を受理しない返戻
			return false;
		};
		PlanetCore::ContinuousSpillDiagnostics Report;
		std::optional<PlanetCore::ContinuousSpillGraph> Graph =
		PlanetCore::BuildContinuousSpillGraph(Result.Network.Connections, IsOcean, Settings, {}, &Report);
		if(!Graph || Result.Network.OceanWitnesses.empty()) {
			Failure = FString::Printf(TEXT("input graph, failure=%u"), static_cast<unsigned>(Report.Failure));
			// 大洋の証拠又は元網の包含関係を確認出来ない場合の返戻
			return {};
		}
		Result.Network.Graph = std::move(*Graph);
		// 元の高い出口を含む，独立に検査した有限網の返戻
		return Result;
	}

	/**
	 * 診断回数以外の実経路・停留点・大洋と排水階層を成分のビット列へ写す
	 * @param Network 同じ元場で準備した有限の水系
	 * @return 詰物を含まない全物理成分；時間・追加記憶量は O（枝の標本＋頂点＋辺＋階層節点）
	 */
	std::vector<std::uint64_t> BypassNetworkValues(const PlanetCore::SpillNetwork &Network) {
		std::vector<std::uint64_t> Data;
		const auto Number = [&](const double Value) -> void {
			Data.push_back(std::bit_cast<std::uint64_t>(Value));
			// 値の追加の終了
			return;
		};
		const auto Vector = [&](const PlanetCore::Vec3 V) -> void {
			Number(V.X);
			Number(V.Y);
			Number(V.Z);
			// 成分の追加の終了
			return;
		};
		const auto Critical = [&](const PlanetCore::TerrainCriticalPoint &P) -> void {
			Vector(P.Direction.Vector());
			Number(P.ElevationMeters);
			Number(P.GradientNorm);
			Number(P.MinimumCurvaturePerMeter);
			Number(P.MaximumCurvaturePerMeter);
			Vector(P.MinimumCurvatureDirection);
			Data.push_back(static_cast<unsigned>(P.Kind));
			// 実停留点の追加の終了
			return;
		};
		Data.push_back(Network.Connections.size());
		for(const PlanetCore::SpillConnection &Connection : Network.Connections) {
			Critical(Connection.Pass);
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
				Data.push_back(static_cast<unsigned>(Branch.Kind));
				Data.push_back(Branch.Minimum.has_value());
				if(Branch.Minimum) Critical(*Branch.Minimum);
				Data.push_back(static_cast<unsigned>(Branch.Path.End));
				Data.push_back(Branch.Path.RejectedSteps);
				Number(Branch.Path.MaximumEstimatedErrorMeters);
				Data.push_back(Branch.Path.Samples.size());
				for(const PlanetCore::TerrainDescentSample &Sample : Branch.Path.Samples) {
					Vector(Sample.Direction.Vector());
					Number(Sample.ElevationMeters);
					Number(Sample.DistanceMeters);
				}
			}
		}
		Data.push_back(Network.OceanWitnesses.size());
		for(const PlanetCore::SpillOceanWitness &Witness : Network.OceanWitnesses) {
			Vector(Witness.Terminal.Vector());
			Vector(Witness.Anchor.Vector());
			Number(Witness.CheckedMaximumMeters);
		}
		const PlanetCore::ContinuousSpillGraph &Graph = Network.Graph;
		Data.push_back(Graph.Minima.size());
		for(const std::optional<PlanetCore::TerrainCriticalPoint> &Minimum : Graph.Minima) {
			Data.push_back(Minimum.has_value());
			if(Minimum) Critical(*Minimum);
		}
		Data.push_back(Graph.Sites.size());
		for(const PlanetCore::SpillGraphSite &Site : Graph.Sites) {
			Number(Site.ElevationMeters);
			Data.push_back(Site.IsOcean);
		}
		Data.push_back(Graph.Edges.size());
		for(const PlanetCore::SpillGraphEdge &Edge : Graph.Edges) {
			Data.push_back(Edge.Nodes[0]);
			Data.push_back(Edge.Nodes[1]);
			Number(Edge.CrestMeters);
		}
		for(const std::uint32_t Value : Graph.EdgeConnections) Data.push_back(Value);
		for(const std::array<std::uint32_t, 2> &Pair : Graph.BranchSites) {
			Data.push_back(Pair[0]);
			Data.push_back(Pair[1]);
		}
		for(const std::uint32_t Value : Graph.Hierarchy.SiteRoots) Data.push_back(Value);
		for(const std::uint32_t Value : Graph.Hierarchy.SiteLeaves) Data.push_back(Value);
		Data.push_back(Graph.Hierarchy.Nodes.size());
		for(const PlanetCore::DepressionHierarchyNode &Node : Graph.Hierarchy.Nodes) {
			for(
				const std::uint32_t Value :
				{
					Node.Parent,
					Node.Children[0],
					Node.Children[1],
					Node.Site,
					Node.OutletSite,
					Node.ReceivingSite,
					Node.OutletEdge,
					Node.OceanLinkRank
				}
			) Data.push_back(Value);
			Number(Node.MinimumMeters);
			Number(Node.SpillMeters);
		}
		const PlanetCore::SpillGraph &Saturation = Graph.Hierarchy.Saturation;
		Data.push_back(Saturation.Nodes.size());
		for(const PlanetCore::SpillGraphNode &Node : Saturation.Nodes) {
			Number(Node.EscapeMeters);
			for(const std::uint32_t Value : { Node.Downstream, Node.DownstreamEdge, Node.SettlementRank, Node.Region }) {
				Data.push_back(Value);
			}
		}
		Data.push_back(Saturation.Regions.size());
		for(const PlanetCore::SpillGraphRegion &Region : Saturation.Regions) {
			Number(Region.WaterMeters);
			for(
				const std::uint32_t Value :
				{ Region.OutletNode, Region.OutletNeighbor, Region.OutletEdge, Region.WetNodes, Region.ShoreNodes }
			) Data.push_back(Value);
			Data.push_back(Region.IsOcean);
		}
		Data.push_back(Saturation.DryFlatEdges);
		// 型の詰物を読まない成分列の返戻
		return Data;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetBypassDeferredRelaxation,
	"PlanetGenerator.SpillNetwork.DeferredRelaxation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ元場の実接続で低い大洋出口を確認してから，不要な緩和を省く
 * @param Parameters 未使用の実行引数
 * @return 正規の三半径，元網不変，有限予算・元場の失敗・中断と従来順の比較が成立した場合に真
 */
bool FPlanetBypassDeferredRelaxation::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		FString Failure;
		std::optional<BypassFixture> Fixture = BypassFixture::Create(Radius, Failure);
		AddInfo(FString::Printf(TEXT("Bypass fixture R=%.0f prepared=%d stage=%s"), Radius, Fixture.has_value(), *Failure));
		// 未準備の解析網を補完検査へ渡さない返戻
		if(!TestTrue(TEXT("高い元峠と実大洋を持つ解析網"), Fixture.has_value())) return false;
		PlanetCore::SpillBypassSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -6;
		const PlanetCore::SpillConnection &OldPass = Fixture->Network.Connections[0];
		TestTrue(TEXT("元峠の両終端は実極小"), OldPass.Branches[0].Minimum && OldPass.Branches[1].Minimum);
		const std::vector<std::uint64_t> OldValues = BypassNetworkValues(Fixture->Network);
		const std::vector<std::uint32_t> OldRoots = Fixture->Network.Graph.Hierarchy.SiteRoots;
		const std::size_t OldWitnesses = Fixture->Network.OceanWitnesses.size();
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			// 所有する同じ解析標高と勾配の返戻
			return Fixture->Source(D);
		};
		Settings.MaximumLocalHalfWidthMeters = 0;
		PlanetCore::SpillBypassDiagnostics InvalidWidthReport;
		TestFalse(
			TEXT("無効な局所範囲を拒否"),
			PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &InvalidWidthReport).has_value()
		);
		TestTrue(TEXT("局所範囲の実拒否理由"), InvalidWidthReport.Failure == PlanetCore::SpillBypassFailure::InvalidInput);
		TestEqual(TEXT("無効入力で元場を呼ばない"), Calls, UINT64_C(0));
		Settings.MaximumLocalHalfWidthMeters = 2048;
		PlanetCore::SpillBypassDiagnostics DeferredReport, EagerReport;
		const std::optional<PlanetCore::SpillNetwork> Deferred =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &DeferredReport);
		TestEqual(TEXT("繰延経路の元場の実照会数"), Calls, DeferredReport.Evaluations);
		AddInfo(
			FString::Printf(
				TEXT("Deferred R=%.0f accepted=%d failure=%u added=%u calls=%llu relax=%u escapes=(%.17g,%.17g) old=%.17g"),
				Radius,
				Deferred.has_value(),
				static_cast<unsigned>(DeferredReport.Failure),
				DeferredReport.AddedConnections,
				Calls,
				DeferredReport.ConnectionRelaxationAttempts,
				DeferredReport.EscapeMeters[0],
				DeferredReport.EscapeMeters[1],
				OldPass.Pass.ElevationMeters
			)
		);
		// 低い出口の確認不足を費用削減の成功にしない返戻
		if(!TestTrue(TEXT("実接続から両終端の低い大洋出口を確認"), Deferred.has_value())) return false;
		TestTrue(TEXT("実際に新しい接続を投入"), DeferredReport.AddedConnections > 0);
		TestEqual(TEXT("確認済の出口では緩和不要"), DeferredReport.ConnectionRelaxationAttempts, 0U);
		for(unsigned Side = 0; Side < 2; ++Side) {
			TestTrue(TEXT("元峠を越えない両大洋出口"), DeferredReport.EscapeMeters[Side] < OldPass.Pass.ElevationMeters - 1E-6);
		}
		Settings.RelaxOnlyWhenNeeded = false;
		Calls = 0;
		const std::optional<PlanetCore::SpillNetwork> Eager =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &EagerReport);
		AddInfo(
			FString::Printf(
				TEXT("Eager R=%.0f accepted=%d failure=%u added=%u calls=%llu relax=%u relaxationCalls=%llu"),
				Radius,
				Eager.has_value(),
				static_cast<unsigned>(EagerReport.Failure),
				EagerReport.AddedConnections,
				Calls,
				EagerReport.ConnectionRelaxationAttempts,
				EagerReport.ConnectionRelaxationEvaluations
			)
		);
		TestTrue(TEXT("従来順の正規補完も維持"), Eager.has_value());
		TestEqual(TEXT("従来順は同じ経路を緩和"), EagerReport.ConnectionRelaxationAttempts, 1U);
		TestEqual(TEXT("従来順の実照会数"), Calls, EagerReport.Evaluations);
		TestTrue(TEXT("直接確認時の実作業を削減"), DeferredReport.Evaluations < EagerReport.Evaluations);
		TestEqual(TEXT("元の三接続を保持"), Fixture->Network.Connections.size(), std::size_t(3));
		TestEqual(TEXT("元の大洋証拠を保持"), Fixture->Network.OceanWitnesses.size(), OldWitnesses);
		TestTrue(TEXT("元の包含関係を保持"), Fixture->Network.Graph.Hierarchy.SiteRoots == OldRoots);
		// 格子を省略する明示した比較で，未発見を早期拒否する経路と実接続まで検証する経路を切分ける
		Settings.RelaxOnlyWhenNeeded = true;
		Settings.MaximumLocalHalfWidthMeters = 256;
		Settings.UseGridPassages = false;
		Settings.UseContinuousConnectionFallback = false;
		Calls = 0;
		PlanetCore::SpillBypassDiagnostics LegacyReport, FallbackReport;
		const std::optional<PlanetCore::SpillNetwork> Legacy =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &LegacyReport);
		TestFalse(TEXT("格子通路の未発見で拒否する従来条件"), Legacy.has_value());
		TestTrue(TEXT("早期拒否の理由"), LegacyReport.Failure == PlanetCore::SpillBypassFailure::NoLowerPassage);
		TestEqual(TEXT("比較条件では格子を準備しない"), LegacyReport.Windows, 0U);
		Settings.UseContinuousConnectionFallback = true;
		Calls = 0;
		const std::optional<PlanetCore::SpillNetwork> Fallback =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &FallbackReport);
		AddInfo(
			FString::Printf(
				TEXT("Continuous connection R=%.0f accepted=%d failure=%u calls=%llu windows=%u climbs=%u added=%u route=%d"),
				Radius,
				Fallback.has_value(),
				static_cast<unsigned>(FallbackReport.Failure),
				Calls,
				FallbackReport.LocalWindows,
				FallbackReport.ClimbingAttempts,
				FallbackReport.AddedConnections,
				FallbackReport.HasLowerConnectionRoute
			)
		);
		TestEqual(TEXT("連続候補の全実照会"), Calls, FallbackReport.Evaluations);
		// 実接続を確認出来ない候補を成功にしない返戻
		if(!TestTrue(TEXT("格子未準備でも実接続で両大洋出口を確認"), Fallback.has_value())) return false;
		TestTrue(TEXT("格子とは別の連続経路を使用"), FallbackReport.UsedContinuousConnectionFallback);
		TestTrue(TEXT("全実接続で低い大洋出口を確認"), FallbackReport.HasLowerConnectionRoute);
		TestFalse(TEXT("未測定の格子通路を証拠にしない"), FallbackReport.HasLowerRelaxedPath);
		TestTrue(TEXT("低い実接続を追加"), FallbackReport.AddedConnections > 0);
		for(unsigned Side = 0; Side < 2; ++Side) {
			TestTrue(TEXT("連続候補の両大洋出口は元峠より低い"), FallbackReport.EscapeMeters[Side] < OldPass.Pass.ElevationMeters - 1E-6);
		}
		TestTrue(TEXT("連続候補でも元網の全物理成分を保持"), BypassNetworkValues(Fixture->Network) == OldValues);
		Settings.UseGridPassages = true;
		Settings.MaximumLocalHalfWidthMeters = 2048;
		for(const bool UseGrid : { true, false }) {
			Settings.UseGridPassages = UseGrid;
			Settings.RelaxOnlyWhenNeeded = true;
			Settings.MaximumEvaluations = 1;
			Calls = 0;
			TestFalse(
				TEXT("探索途中の予算不足を拒否"),
				PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &DeferredReport).has_value()
			);
			TestTrue(TEXT("予算不足の実原因"), DeferredReport.Failure == PlanetCore::SpillBypassFailure::Budget);
			TestEqual(TEXT("予算後に元場を呼ばない"), Calls, UINT64_C(1));
			Settings.MaximumEvaluations = 4000000;
			Calls = 0;
			const auto InvalidSource = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
				++Calls;
				if(Calls > 20) {
					const double Invalid = std::numeric_limits<double>::quiet_NaN();
					// 探索途中の欠落した元場の返戻
					return { Invalid, { Invalid, Invalid, Invalid } };
				}
				// 初期は正規な同じ元場の返戻
				return Fixture->Source(D);
			};
			TestFalse(
				TEXT("途中の元場欠落を拒否"),
				PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, InvalidSource, Settings, {}, &DeferredReport).has_value()
			);
			TestTrue(TEXT("元場欠落の実原因"), DeferredReport.Failure == PlanetCore::SpillBypassFailure::Source);
			std::stop_source Stop;
			Calls = 0;
			const auto CancelSource = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
				++Calls;
				if(Calls == 30) Stop.request_stop();
				// 中断を要求しても元場の定義自体は変えない返戻
				return Fixture->Source(D);
			};
			TestFalse(
				TEXT("探索途中の中断を拒否"),
				PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, CancelSource, Settings, Stop.get_token(), &DeferredReport)
				.has_value()
			);
			TestTrue(TEXT("中断の実原因"), DeferredReport.Failure == PlanetCore::SpillBypassFailure::Cancelled);
			TestEqual(TEXT("中断後に元場を呼ばない"), Calls, UINT64_C(30));
		}
	}
	// 実接続と費用及び失敗経路の確認結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetBypassPreparedReuse,
	"PlanetGenerator.SpillNetwork.PreparedBranchReuse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 元場で同じ峠を精査した時だけ，既存の両下降枝の再追跡を省く
 * @param Parameters 未使用の実行引数
 * @return 三半径で全物理成分のビット一致・実作業削減・元網不変が成立する場合に真
 */
bool FPlanetBypassPreparedReuse::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		FString Failure;
		const std::optional<BypassFixture> Fixture = BypassFixture::Create(Radius, Failure);
		AddInfo(FString::Printf(TEXT("Prepared-branch fixture R=%.0f ready=%d failure=%s"), Radius, Fixture.has_value(), *Failure));
		// 独立な元網を準備出来ない場合の返戻
		if(!TestTrue(TEXT("三半径の実大洋を持つ元網"), Fixture.has_value())) return false;
		const std::vector<std::uint64_t> Original = BypassNetworkValues(Fixture->Network);
		PlanetCore::SpillBypassSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -6;
		// 繰返し途中の同じ峠候補も比較する為，両経路で緩和を実行
		Settings.RelaxOnlyWhenNeeded = false;
		Settings.ReusePreparedConnections = false;
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			// 値を変えない同じ解析元場の返戻
			return Fixture->Source(D);
		};
		PlanetCore::SpillBypassDiagnostics ColdReport, WarmReport;
		const std::optional<PlanetCore::SpillNetwork> Cold =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &ColdReport);
		TestEqual(TEXT("比較元の実照会数"), Calls, ColdReport.Evaluations);
		// 比較基準が失敗した場合の返戻
		if(!TestTrue(TEXT("再追跡する従来経路が成功"), Cold.has_value())) return false;
		Calls = 0;
		Settings.ReusePreparedConnections = true;
		const std::optional<PlanetCore::SpillNetwork> Warm =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &WarmReport);
		TestEqual(TEXT("再利用する実照会数"), Calls, WarmReport.Evaluations);
		AddInfo(
			FString::Printf(
				TEXT("Prepared branches R=%.0f accepted=%d cold=%llu warm=%llu reused=%u added=(%u,%u) checks=(%u,%u) iterations=(%u,%u)"),
				Radius,
				Warm.has_value(),
				ColdReport.Evaluations,
				WarmReport.Evaluations,
				WarmReport.ReusedCandidates,
				ColdReport.AddedConnections,
				WarmReport.AddedConnections,
				ColdReport.ConnectionCheckpointChecks,
				WarmReport.ConnectionCheckpointChecks,
				ColdReport.ConnectionRelaxationIterations,
				WarmReport.ConnectionRelaxationIterations
			)
		);
		// 部分的な候補群を再利用成功にしない返戻
		if(!TestTrue(TEXT("準備済の両枝を再利用する経路が成功"), Warm.has_value())) return false;
		TestTrue(TEXT("同じ実峠の候補が再発見された"), WarmReport.ReusedCandidates > 0);
		TestTrue(TEXT("実元場の照会が減少"), WarmReport.Evaluations < ColdReport.Evaluations);
		TestTrue(TEXT("全停留点・実下降経路・大洋・排水階層がビット一致"), BypassNetworkValues(*Warm) == BypassNetworkValues(*Cold));
		TestEqual(TEXT("実追加接続数を維持"), WarmReport.AddedConnections, ColdReport.AddedConnections);
		TestEqual(TEXT("実接続を確認する反復回数を維持"), WarmReport.ConnectionRelaxationIterations, ColdReport.ConnectionRelaxationIterations);
		TestTrue(TEXT("元網の全物理成分が不変"), BypassNetworkValues(Fixture->Network) == Original);
	}
	// 正規な実物比較と費用の検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetBypassSamplePreparation,
	"PlanetGenerator.SpillNetwork.SamplePreparation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 逐次補完の厳密標本を共有し，元場の寿命・容量・中断と全物理量を検査する
 * @param Parameters 未使用の実行引数
 * @return 三半径・両候補方式で物理量一致・照会削減・失敗の保持が成立する場合に真
 */
bool FPlanetBypassSamplePreparation::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		FString Failure;
		const std::optional<BypassFixture> Fixture = BypassFixture::Create(Radius, Failure);
		// 独立な解析網の欠落への返戻
		if(!TestTrue(TEXT("共有準備の独立な三半径の元網"), Fixture.has_value())) return false;
		const std::vector<std::uint64_t> Original = BypassNetworkValues(Fixture->Network);
		for(const bool UseGrid : { false, true }) {
			PlanetCore::SpillBypassSettings Settings;
			Settings.RadiusMeters = Radius;
			Settings.SeaLevelMeters = -6;
			Settings.UseGridPassages = UseGrid;
			std::weak_ptr<std::uint64_t> Lifetime;
			std::optional<PlanetCore::SpillNetwork> Saved;
			{
				const std::shared_ptr<std::uint64_t> Calls = std::make_shared<std::uint64_t>();
				Lifetime = Calls;
				std::function<PlanetCore::TerrainSample(PlanetCore::UnitDirection)> Source =
				[Base = Fixture->Source, Calls](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
					++*Calls;
					// 所有する元場の値だけを計測する返戻
					return Base(D);
				};
				PlanetCore::SpillBypassPreparation Preparation(Source, Settings.MaximumCachedSamples);
				Source = {};
				PlanetCore::SpillBypassDiagnostics ColdReport, WarmReport, Rejected;
				const std::optional<PlanetCore::SpillNetwork> Cold = Preparation.Prepare(Fixture->Network, 0, Settings, {}, &ColdReport);
				TestEqual(TEXT("初回の元場照会を実回数と一致"), *Calls, ColdReport.Evaluations);
				// 単独基準を欠く場合の返戻
				if(!TestTrue(TEXT("呼出側の関数を解放しても初回準備"), Cold.has_value())) return false;
				const std::optional<PlanetCore::SpillNetwork> Independent =
				PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Fixture->Source, Settings);
				TestTrue(TEXT("単独の準備と全物理量が一致"), Independent && BypassNetworkValues(*Independent) == BypassNetworkValues(*Cold));
				*Calls = 0;
				const std::optional<PlanetCore::SpillNetwork> Warm = Preparation.Prepare(Fixture->Network, 0, Settings, {}, &WarmReport);
				TestEqual(TEXT("再準備の照会を実回数と一致"), *Calls, WarmReport.Evaluations);
				TestTrue(TEXT("同じ実網の全成分がビット一致"), Warm && BypassNetworkValues(*Warm) == BypassNetworkValues(*Cold));
				TestTrue(TEXT("逐次準備で元場照会を削減"), WarmReport.Evaluations < ColdReport.Evaluations);
				TestTrue(TEXT("共有標本の実再利用"), WarmReport.CacheHits > ColdReport.CacheHits);
				TestTrue(TEXT("共有標本も設定の上限内"), WarmReport.CachedSamples <= Settings.MaximumCachedSamples);
				std::stop_source Stop;
				Stop.request_stop();
				*Calls = 0;
				TestFalse(
					TEXT("保持済の標本でも中断後に公開しない"),
					Preparation.Prepare(Fixture->Network, 0, Settings, Stop.get_token(), &Rejected).has_value()
				);
				TestTrue(TEXT("共有準備の中断理由"), Rejected.Failure == PlanetCore::SpillBypassFailure::Cancelled);
				TestEqual(TEXT("中断後の元場照会は零"), *Calls, UINT64_C(0));
				Settings.MaximumCachedSamples = 1;
				TestFalse(TEXT("異なる容量を古い枠で黙って処理しない"), Preparation.Prepare(Fixture->Network, 0, Settings, {}, &Rejected).has_value());
				TestTrue(TEXT("容量不一致を入力拒否"), Rejected.Failure == PlanetCore::SpillBypassFailure::InvalidInput);
				TestEqual(TEXT("容量不一致で元場を呼ばない"), *Calls, UINT64_C(0));
				Settings.MaximumCachedSamples = 262144;
				Saved = Preparation.Prepare(Fixture->Network, 0, Settings);
				TestTrue(TEXT("中断・不一致後も正規準備が通る"), Saved && BypassNetworkValues(*Saved) == BypassNetworkValues(*Cold));
				AddInfo(
					FString::Printf(
						TEXT("Bypass preparation R=%.0f grid=%d cold=%llu warm=%llu hits=%llu retained=%u"),
						Radius,
						UseGrid,
						ColdReport.Evaluations,
						WarmReport.Evaluations,
						WarmReport.CacheHits,
						WarmReport.CachedSamples
					)
				);
			}
			TestTrue(TEXT("公開済の実網は元場関数と標本領域を所有しない"), Saved.has_value() && Lifetime.expired());
			// 元場が違う新しい準備へ，古い元場の標本を流用しない
			const auto Different = [Base = Fixture->Source](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
				PlanetCore::TerrainSample Value = Base(D);
				++Value.ElevationMeters;
				// 独立な異なる元場の返戻
				return Value;
			};
			PlanetCore::SpillBypassPreparation Foreign(Different, Settings.MaximumCachedSamples);
			PlanetCore::SpillBypassDiagnostics Rejected;
			TestFalse(TEXT("異なる元場の旧網を拒否"), Foreign.Prepare(Fixture->Network, 0, Settings, {}, &Rejected).has_value());
			TestTrue(TEXT("異なる元場を実値で識別"), Rejected.Failure == PlanetCore::SpillBypassFailure::Source);
			Settings.MaximumCachedSamples = PlanetCore::SpillSampleCache::MaximumCapacity + 1;
			std::uint64_t InvalidCalls = 0;
			const auto InvalidSource = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
				++InvalidCalls;
				// 無効な容量で呼出された回数だけを計測する返戻
				return Fixture->Source(D);
			};
			PlanetCore::SpillBypassPreparation Oversized(InvalidSource, Settings.MaximumCachedSamples);
			TestFalse(TEXT("共有枠の実容量上限を超える準備を拒否"), Oversized.Prepare(Fixture->Network, 0, Settings, {}, &Rejected).has_value());
			TestTrue(TEXT("過大な容量の入力拒否"), Rejected.Failure == PlanetCore::SpillBypassFailure::InvalidInput);
			TestEqual(TEXT("過大な容量で元場を呼ばない"), InvalidCalls, UINT64_C(0));
			Settings.MaximumCachedSamples = 1;
			PlanetCore::SpillBypassPreparation Bounded(Fixture->Source, 1);
			TestTrue(TEXT("小さい共有枠でも初回準備が成功"), Bounded.Prepare(Fixture->Network, 0, Settings).has_value());
			Settings.MaximumEvaluations = 1;
			TestFalse(TEXT("再準備も実照会の有限予算を守る"), Bounded.Prepare(Fixture->Network, 0, Settings, {}, &Rejected).has_value());
			TestTrue(TEXT("共有準備の照会不足を識別"), Rejected.Failure == PlanetCore::SpillBypassFailure::Budget);
			TestTrue(TEXT("共有準備の予算後に照会しない"), Rejected.Evaluations < 2);
		}
		TestTrue(TEXT("共有準備でも元網は不変"), BypassNetworkValues(Fixture->Network) == Original);
	}
	// 所有期間・実費用・全物理成分の確認結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetBypassPreferredContinuous,
	"PlanetGenerator.SpillNetwork.PreferredContinuous",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 連続候補を先行して実出口を確認し，未確認時の格子も共通の実照会予算内で検査する
 * @param Parameters 未使用の実行引数
 * @return 三半径で正規の両出口・格子への復帰・元場欠落／中断／予算拒否が成立する場合に真
 */
bool FPlanetBypassPreferredContinuous::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		FString Failure;
		const std::optional<BypassFixture> Fixture = BypassFixture::Create(Radius, Failure);
		// 元網の欠落への返戻
		if(!TestTrue(TEXT("候補順比較の独立な元網"), Fixture.has_value())) return false;
		const std::vector<std::uint64_t> Original = BypassNetworkValues(Fixture->Network);
		PlanetCore::SpillBypassSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -6;
		PlanetCore::SpillBypassDiagnostics ReferenceReport;
		const std::optional<PlanetCore::SpillNetwork> Reference =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Fixture->Source, Settings, {}, &ReferenceReport);
		// 比較基準を欠く場合の返戻
		if(!TestTrue(TEXT("従来の格子先行の正規準備"), Reference.has_value())) return false;
		Settings.PreferContinuousCandidates = true;
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			// 同じ元場の実照会だけを数える返戻
			return Fixture->Source(D);
		};
		PlanetCore::SpillBypassDiagnostics PreferredReport;
		const std::optional<PlanetCore::SpillNetwork> Preferred =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &PreferredReport);
		TestEqual(TEXT("先行と格子を合算した実照会数"), Calls, PreferredReport.Evaluations);
		// 完成していない候補を成功にしない返戻
		if(!TestTrue(TEXT("連続候補を先行する正規準備"), Preferred.has_value())) return false;
		TestEqual(TEXT("先行試行は一回"), PreferredReport.PreferredContinuousAttempts, 1U);
		TestTrue(TEXT("解析場では先行候補で完成"), PreferredReport.AcceptedPreferredContinuous);
		TestEqual(TEXT("完成時に格子を二重探索しない"), PreferredReport.PreferredGridFallbacks, 0U);
		TestTrue(TEXT("先行候補も実接続の両大洋出口を確認"), PreferredReport.HasLowerConnectionRoute);
		for(const double Escape : PreferredReport.EscapeMeters) {
			TestTrue(TEXT("各実出口が元峠より低い"), Escape < Fixture->Network.Connections[0].Pass.ElevationMeters - 1E-6);
		}
		TestTrue(TEXT("先行準備も共通の実照会予算内"), PreferredReport.Evaluations <= Settings.MaximumEvaluations);
		if(Radius >= 1E6) TestTrue(TEXT("通常半径で従来の格子より実照会を削減"), PreferredReport.Evaluations < ReferenceReport.Evaluations);
		Settings.ContinuousEvaluationLimit = 1;
		Calls = 0;
		PlanetCore::SpillBypassDiagnostics FallbackReport;
		const std::optional<PlanetCore::SpillNetwork> Fallback =
		PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &FallbackReport);
		TestTrue(TEXT("先行試行の不足後は格子の正規経路が通る"), Fallback.has_value());
		TestTrue(TEXT("格子復帰の全物理成分が旧経路と一致"), Fallback && BypassNetworkValues(*Fallback) == BypassNetworkValues(*Reference));
		TestTrue(TEXT("先行試行の実不足を保持"), FallbackReport.FirstPreferredFailure == PlanetCore::SpillBypassFailure::Budget);
		TestFalse(TEXT("先行部分網を完成として扱わない"), FallbackReport.AcceptedPreferredContinuous);
		TestEqual(TEXT("格子の再試行は一回"), FallbackReport.PreferredGridFallbacks, 1U);
		TestEqual(TEXT("先行試行の実照会も一回"), FallbackReport.PreferredContinuousEvaluations, UINT64_C(1));
		TestEqual(TEXT("復帰後も双方の照会を数える"), Calls, FallbackReport.Evaluations);
		TestTrue(TEXT("復帰時も合算した予算内"), FallbackReport.Evaluations <= Settings.MaximumEvaluations);
		Settings.MaximumEvaluations = 1;
		Calls = 0;
		PlanetCore::SpillBypassDiagnostics Rejected;
		TestFalse(
			TEXT("共通予算不足を格子の別予算で迂回しない"),
			PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &Rejected).has_value()
		);
		TestTrue(TEXT("共通予算不足の実拒否"), Rejected.Failure == PlanetCore::SpillBypassFailure::Budget);
		TestEqual(TEXT("予算後は格子を始めない"), Rejected.PreferredGridFallbacks, 0U);
		TestEqual(TEXT("共通予算を超えて元場を呼ばない"), Calls, UINT64_C(1));
		Settings.MaximumEvaluations = 32000001;
		Calls = 0;
		TestFalse(
			TEXT("先行予算への縮小で元の過大な上限を隠さない"),
			PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &Rejected).has_value()
		);
		TestTrue(TEXT("過大な共通予算の入力拒否"), Rejected.Failure == PlanetCore::SpillBypassFailure::InvalidInput);
		TestEqual(TEXT("過大な共通予算で元場を呼ばない"), Calls, UINT64_C(0));
		Settings.MaximumEvaluations = 4000000;
		Settings.ContinuousEvaluationLimit = 0;
		Calls = 0;
		TestFalse(
			TEXT("零の先行予算を探索前に拒否"),
			PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Source, Settings, {}, &Rejected).has_value()
		);
		TestTrue(TEXT("不正な先行予算の理由"), Rejected.Failure == PlanetCore::SpillBypassFailure::InvalidInput);
		TestEqual(TEXT("不正な先行予算で元場を呼ばない"), Calls, UINT64_C(0));
		Settings.ContinuousEvaluationLimit = 131072;
		const double Invalid = std::numeric_limits<double>::quiet_NaN();
		const auto Missing = [Invalid](const PlanetCore::UnitDirection) -> PlanetCore::TerrainSample {
			// 未定義の元場を正常な値へ置換えない返戻
			return { Invalid, { Invalid, Invalid, Invalid } };
		};
		TestFalse(
			TEXT("元場欠落を格子で打消さない"),
			PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, Missing, Settings, {}, &Rejected).has_value()
		);
		TestTrue(TEXT("元場欠落の理由を保持"), Rejected.Failure == PlanetCore::SpillBypassFailure::Source);
		TestEqual(TEXT("元場欠落後は格子を始めない"), Rejected.PreferredGridFallbacks, 0U);
		std::stop_source Stop;
		Calls = 0;
		const auto CancelSource = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			if(Calls == 2) Stop.request_stop();
			// 中断でも元場の値を変えない返戻
			return Fixture->Source(D);
		};
		TestFalse(
			TEXT("先行試行中の中断を両経路へ伝播"),
			PlanetCore::CompleteSpillNetworkBypass(Fixture->Network, 0, CancelSource, Settings, Stop.get_token(), &Rejected).has_value()
		);
		TestTrue(TEXT("先行中の中断を保持"), Rejected.Failure == PlanetCore::SpillBypassFailure::Cancelled);
		TestEqual(TEXT("中断後に格子を始めない"), Rejected.PreferredGridFallbacks, 0U);
		TestEqual(TEXT("中断後に元場を呼ばない"), Calls, UINT64_C(2));
		TestTrue(TEXT("両方式の全試行でも入力網は不変"), BypassNetworkValues(Fixture->Network) == Original);
		AddInfo(
			FString::Printf(
				TEXT("Preferred continuous R=%.0f gridCalls=%llu preferredCalls=%llu cappedCalls=%llu fallbackCalls=%llu"),
				Radius,
				ReferenceReport.Evaluations,
				PreferredReport.Evaluations,
				FallbackReport.PreferredContinuousEvaluations,
				FallbackReport.Evaluations
			)
		);
	}
	// 正規経路と共通予算及び入力不変の確認結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetBypassOceanAnchorInput,
	"PlanetGenerator.SpillNetwork.OceanAnchorInput",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ元場の大洋標本を持つ準備と，所属述語が欠けた入力を検査する
 * @param Parameters 未使用の実行引数
 * @return 三半径の正規補完・入力不変・不整合入力の無評価拒否を確認した場合に真
 */
bool FPlanetBypassOceanAnchorInput::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		FString Failure;
		const std::optional<BypassFixture> Fixture = BypassFixture::Create(Radius, Failure);
		// 同じ元場の正規の準備を欠く場合の返戻
		if(!TestTrue(TEXT("大洋標本検査の独立な元網"), Fixture && !Fixture->Network.OceanWitnesses.empty())) return false;
		const std::vector<std::uint64_t> Original = BypassNetworkValues(Fixture->Network);
		const std::array<PlanetCore::UnitDirection, 1> Anchors { Fixture->Network.OceanWitnesses.front().Anchor };
		const auto IsOcean = [&Anchors](const PlanetCore::UnitDirection Direction) -> bool {
			const PlanetCore::Vec3 A = Direction.Vector(), B = Anchors[0].Vector();
			// 同じ元場で所属を確認済の正確な標本だけの返戻
			return A.X == B.X && A.Y == B.Y && A.Z == B.Z;
		};
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			// 元場の値を変更せず実照会を数えた返戻
			return Fixture->Source(D);
		};
		PlanetCore::SpillBypassSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -6;
		PlanetCore::SpillBypassPreparation Valid(Source, Settings.MaximumCachedSamples, Anchors, IsOcean);
		PlanetCore::SpillBypassDiagnostics Report;
		const std::optional<PlanetCore::SpillNetwork> Prepared = Valid.Prepare(Fixture->Network, 0, Settings, {}, &Report);
		TestTrue(TEXT("大洋所属の正規入力で実補完が通る"), Prepared && Report.AddedConnections && Report.HasLowerConnectionRoute);
		TestEqual(TEXT("正規入力の実照会を漏らさない"), Calls, Report.Evaluations);
		Calls = 0;
		PlanetCore::SpillBypassPreparation MissingPredicate(
			Source,
			Settings.MaximumCachedSamples,
			Anchors,
			std::function<bool(PlanetCore::UnitDirection)>{}
		);
		TestFalse(TEXT("標本だけで大洋所属を認定しない"), MissingPredicate.Prepare(Fixture->Network, 0, Settings, {}, &Report).has_value());
		TestTrue(TEXT("欠けた所属述語は不正入力"), Report.Failure == PlanetCore::SpillBypassFailure::InvalidInput);
		TestEqual(TEXT("不整合入力で元場を照会しない"), Calls, UINT64_C(0));
		PlanetCore::SpillBypassPreparation MissingAnchors(
			Source,
			Settings.MaximumCachedSamples,
			std::span<const PlanetCore::UnitDirection>{},
			IsOcean
		);
		TestFalse(TEXT("所属述語だけの入力も拒否"), MissingAnchors.Prepare(Fixture->Network, 0, Settings, {}, &Report).has_value());
		TestEqual(TEXT("標本を欠く入力でも実照会は零"), Calls, UINT64_C(0));
		TestTrue(TEXT("全入力検査で元の網は不変"), BypassNetworkValues(Fixture->Network) == Original);
		AddInfo(FString::Printf(TEXT("Ocean anchor input R=%.0f accepted=%d"), Radius, static_cast<int>(Prepared.has_value())));
	}
	// 正規経路と不正入力の双方を検査した結果の返戻
	return !HasAnyErrors();
}

#endif
