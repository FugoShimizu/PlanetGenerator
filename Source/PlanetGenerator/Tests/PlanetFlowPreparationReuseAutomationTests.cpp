#include "Core/PlanetFlowNetworkField.h"
#include "Core/PlanetFlowNetworkPreparation.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	struct PreparationFixture {
		std::shared_ptr<std::array<std::uint64_t, 2>> Calls;
		std::function<PlanetCore::TerrainSample(PlanetCore::UnitDirection)> Source;
		std::function<double(PlanetCore::UnitDirection)> Height;
		PlanetCore::SpillNetwork Network;
	};

	/**
	 * 独立な解析場で複数の湖・実流出口・大洋への通水を準備する
	 * @param Radius 物理半径
	 * @param IncludesPools 淵の配置を検査する為，物理長と比高を増やした同じ解析式を使う場合に真
	 * @param Diagnostics 元の網を準備出来ない場合の実段階，省略可
	 * @return 同じ定義の有界な水系，元の準備に失敗すれば空
	 */
	std::optional<PreparationFixture> MakePreparationFixture(
		const double Radius,
		const bool IncludesPools=false,
		PlanetCore::SpillNetworkDiagnostics *const Diagnostics=nullptr
	) {
		const double HorizontalScale = IncludesPools ? std::min(3.0, Radius / 1000) : 1, VerticalScale = IncludesPools ? 4 : 1;
		PreparationFixture Result;
		Result.Calls = std::make_shared<std::array<std::uint64_t, 2>>();
		Result.Source = [Radius, HorizontalScale, VerticalScale, Calls = Result.Calls](const PlanetCore::UnitDirection Direction)
		-> PlanetCore::TerrainSample {
			++(*Calls)[0];
			const PlanetCore::Vec3 D = Direction.Vector();
			const double Scale = Radius / HorizontalScale, X = Scale * D.X / D.Z, Y = Scale * D.Y / D.Z;
			const double DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			// 同じ解析場の標高と全微分の返戻
			return {
				VerticalScale * (4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400),
				{
					VerticalScale * Scale * DX / D.Z,
					VerticalScale * Scale * DY / D.Z,
					-VerticalScale * Scale * (DX * D.X + DY * D.Y) / (D.Z * D.Z)
				}
			};
		};
		Result.Height =
		[Radius, HorizontalScale, VerticalScale, Calls = Result.Calls](const PlanetCore::UnitDirection Direction) -> double {
			++(*Calls)[1];
			const PlanetCore::Vec3 D = Direction.Vector();
			const double Scale = Radius / HorizontalScale, X = Scale * D.X / D.Z, Y = Scale * D.Y / D.Z;
			// 全微分の照会と同じ解析標高の返戻
			return VerticalScale * (4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400);
		};
		const PlanetCore::UnitDirection Initial =
		*PlanetCore::UnitDirection::FromVector({ 40 * HorizontalScale * (4 * std::acos(-1.0) + std::asin(0.2)) / Radius, 0, 1 });
		const std::array<PlanetCore::UnitDirection, 1> Anchors {
			*PlanetCore::UnitDirection::FromVector({ -140 * HorizontalScale / Radius, 0, 1 })
		};
		// 同じ解析場で水没を確認した大洋標本への所属だけを許可
		const auto IsOcean = [&](const PlanetCore::UnitDirection D) -> bool {
			const PlanetCore::Vec3 A = D.Vector(), B = Anchors[0].Vector();
			// 同じ元場の確認済大洋標本への所属の返戻
			return A.X == B.X && A.Y == B.Y && A.Z == B.Z;
		};
		PlanetCore::SpillNetworkSettings Settings;
		Settings.RadiusMeters = Radius;
		std::optional<PlanetCore::SpillNetwork> Network =
		PlanetCore::PrepareSpillNetwork(Initial, Result.Source, Anchors, IsOcean, Settings, {}, Diagnostics);
		// 元場の水系を準備出来ない場合の返戻
		if(!Network) return {};
		Result.Network = std::move(*Network);
		// 所有する元場と，独立に確認した水系の返戻
		return Result;
	}

	/**
	 * 岸の全成分と独立な河道地点の河床・水位・勾配を，型の詰物無で採集する
	 * @param Network 同じ定義の準備結果
	 * @param IncludeConnectionIndex 内部の接続番号も比較する場合に真，物理的な並換え比較では偽
	 * @return 数値成分の全ビット；地点評価の欠落は空
	 */
	std::optional<std::vector<std::uint64_t>> PreparationValues(
		const PlanetCore::PreparedFlowNetwork &Network,
		const bool IncludeConnectionIndex=true
	) {
		std::vector<std::uint64_t> Values;
		// 数値の値だけを採集
		const auto Number = [&](const double Value) -> void {
			Values.push_back(std::bit_cast<std::uint64_t>(Value));
			// 終了
			return;
		};
		const auto Vector = [&](const PlanetCore::Vec3 V) -> void {
			Number(V.X);
			Number(V.Y);
			Number(V.Z);
			// 終了
			return;
		};
		Values.push_back(Network.Channels.size());
		for(const PlanetCore::FlowBasinChannel &Basin : Network.Channels) {
			const PlanetCore::SpillNetworkOutlet &Outlet = Basin.Outlet;
			Values.push_back(Outlet.Basin);
			Values.push_back(Outlet.ReceivingBasin);
			if(IncludeConnectionIndex) Values.push_back(Outlet.Connection);
			Values.push_back(Outlet.IsPositiveDownstream);
			Number(Outlet.UpstreamWaterMeters);
			Number(Outlet.DownstreamWaterMeters);
			Number(Basin.Channel.PassDistanceMeters);
			Number(Basin.Channel.UpstreamWaterMeters);
			Values.push_back(Basin.Channel.PoolCount);
			const PlanetCore::FlowWaterGeometry Geometry = Basin.Channel.Field->GetWaterGeometry();
			Number(Geometry.ChannelHalfWidthMeters);
			Number(Geometry.MaximumChannelHalfWidthMeters);
			const PlanetCore::TerrainContour &Shore = Geometry.Upstream.GetContour();
			Values.push_back(Shore.Samples.size());
			for(const PlanetCore::TerrainContourSample &Point : Shore.Samples) {
				Vector(Point.Direction.Vector());
				Number(Point.ElevationMeters);
				Number(Point.DistanceMeters);
			}
			for(unsigned i = 0; i < 101; ++i) {
				const std::optional<PlanetCore::TerrainFlowCurveSample> Point =
				Geometry.Route.Sample(Geometry.Route.GetLengthMeters() * (i / 100.0));
				// 元の経路の地点を採集出来ない場合の返戻
				if(!Point) return {};
				const std::optional<PlanetCore::FlowWaterSample> Sample = Basin.Channel.Field->EvaluateWithSupport(Point->Direction);
				// 未定義の河床を比較対象から除外しない返戻
				if(!Sample) return {};
				Vector(Point->Direction.Vector());
				Number(Sample->Section.Terrain.ElevationMeters);
				Vector(Sample->Section.Terrain.TangentialGradientPerRadian);
				Values.push_back(Sample->Section.HasWater);
				if(Sample->Section.HasWater) {
					Number(Sample->Section.Water.ElevationMeters);
					Vector(Sample->Section.Water.TangentialGradientPerRadian);
				}
				Number(Sample->SupportMeters);
			}
		}
		// 実際に評価した岸・河道・水位の成分列の返戻
		return Values;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowPreparationReuse,
	"PlanetGenerator.FlowChannel.PreparationReuse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 不変な元場の河道を再利用し，現在の所属と大洋の証拠の再検査を省略しない
 * @param Parameters 未使用の実行引数
 * @return 成分一致・費用削減・不正な所属の拒否・中断後の正常利用が成立した場合に真
 */
bool FPlanetFlowPreparationReuse::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::optional<PreparationFixture> Fixture = MakePreparationFixture(Radius);
		// 同じ解析水系を準備出来ない場合の返戻
		if(!TestTrue(TEXT("複数の実流出口を独立に準備"), Fixture.has_value())) return false;
		PlanetCore::FlowNetworkSettings Settings;
		Settings.Channel.Outlet.RadiusMeters = Radius;
		PlanetCore::FlowNetworkPreparation Preparation(Fixture->Source, Fixture->Height, Settings);
		PlanetCore::FlowNetworkDiagnostics FirstReport;
		*Fixture->Calls = {};
		const std::optional<PlanetCore::PreparedFlowNetwork> First = Preparation.Prepare(Fixture->Network, {}, &FirstReport);
		const std::array<std::uint64_t, 2> FirstCalls = *Fixture->Calls;
		// 初回の全河道を準備出来ない場合の返戻
		if(!TestTrue(TEXT("再利用前の正規水系"), First.has_value())) return false;
		TestTrue(TEXT("複数の下流・上流河道"), First->Channels.size() > 1);
		TestEqual(TEXT("初回は全河道を新規準備"), FirstReport.RebuiltChannels, FirstReport.PreparedChannels);
		const std::optional<std::vector<std::uint64_t>> Expected = PreparationValues(*First);
		// 比較基準の地点評価が欠ける場合の返戻
		if(!TestTrue(TEXT("初回の岸・河床・水位を評価"), Expected.has_value())) return false;
		PlanetCore::FlowNetworkDiagnostics WarmReport;
		*Fixture->Calls = {};
		const std::optional<PlanetCore::PreparedFlowNetwork> Warm = Preparation.Prepare(Fixture->Network, {}, &WarmReport);
		const std::array<std::uint64_t, 2> WarmCalls = *Fixture->Calls;
		// 再利用時の正規水系を準備出来ない場合の返戻
		if(!TestTrue(TEXT("同じ定義の再準備"), Warm.has_value())) return false;
		TestEqual(TEXT("同じ定義の全河道を再利用"), WarmReport.ReusedChannels, WarmReport.PreparedChannels);
		TestEqual(TEXT("同じ定義で新規準備しない"), WarmReport.RebuiltChannels, 0U);
		TestTrue(TEXT("河道・岸・水位・勾配の成分がビット一致"), PreparationValues(*Warm) == Expected);
		TestTrue(TEXT("現在の通水再検査を残しても照会を削減"), WarmCalls[0] + WarmCalls[1] < FirstCalls[0] + FirstCalls[1]);
		for(std::size_t i = 0; i < Warm->Channels.size(); ++i) {
			TestTrue(TEXT("同じ不変な河道を共有"), Warm->Channels[i].Channel.Field == First->Channels[i].Channel.Field);
		}
		std::stop_source Cancel;
		Cancel.request_stop();
		PlanetCore::FlowNetworkDiagnostics Rejected;
		*Fixture->Calls = {};
		TestFalse(TEXT("中断後は保持済の河道も公開しない"), Preparation.Prepare(Fixture->Network, Cancel.get_token(), &Rejected).has_value());
		TestTrue(TEXT("中断を別の失敗にしない"), Rejected.Failure == PlanetCore::FlowNetworkFailure::Cancelled);
		TestEqual(TEXT("中断後の元場照会は零"), (*Fixture->Calls)[0] + (*Fixture->Calls)[1], UINT64_C(0));
		PlanetCore::SpillNetwork Invalid = Fixture->Network;
		const std::uint32_t Basin = First->Channels.front().Outlet.Basin;
		bool HasInvalidMember = false;
		for(std::uint32_t Site = 0; Site < Invalid.Graph.Minima.size(); ++Site) {
			if(Invalid.Graph.Hierarchy.SiteRoots[Site] != Basin || !Invalid.Graph.Minima[Site]) continue;
			Invalid.Graph.Minima[Site]->Direction = *PlanetCore::UnitDirection::FromVector({ 0, 1000 / Radius, 1 });
			HasInvalidMember = true;
			break;
		}
		TestTrue(TEXT("元場を変えずに不正な現在の所属を用意"), HasInvalidMember);
		TestFalse(TEXT("旧河道の再利用でも現在の所属を再検査"), Preparation.Prepare(Invalid, {}, &Rejected).has_value());
		TestTrue(
			TEXT("岸の外の極小を拒否"),
			Rejected.Failure == PlanetCore::FlowNetworkFailure::Membership &&
			Rejected.Membership == PlanetCore::FlowMembershipFailure::UpstreamMinimum
		);
		Invalid = Fixture->Network;
		for(PlanetCore::SpillOceanWitness &Witness : Invalid.OceanWitnesses) Witness.CheckedMaximumMeters = 1;
		TestFalse(TEXT("旧河道の再利用でも大洋の証拠を再検査"), Preparation.Prepare(Invalid, {}, &Rejected).has_value());
		TestTrue(TEXT("現在の大洋証拠の拒否を保持"), Rejected.Failure == PlanetCore::FlowNetworkFailure::OceanPassage);
		const std::optional<PlanetCore::PreparedFlowNetwork> Restored = Preparation.Prepare(Fixture->Network);
		TestTrue(TEXT("拒否後の正規利用が再び通る"), Restored && PreparationValues(*Restored) == Expected);
		PlanetCore::FlowNetworkPreparation Independent(Fixture->Source, Fixture->Height, Settings);
		PlanetCore::FlowNetworkDiagnostics IndependentReport;
		const std::optional<PlanetCore::PreparedFlowNetwork> Cold = Independent.Prepare(Fixture->Network, {}, &IndependentReport);
		TestTrue(TEXT("別の準備も同じ生成結果"), Cold && PreparationValues(*Cold) == Expected);
		TestEqual(TEXT("別の準備へ旧河道を持越さない"), IndependentReport.ReusedChannels, 0U);
		// 同じ有効な方向の零成分だけを反転し，近接値へ丸めない照合を確認
		PlanetCore::SpillNetwork ZeroA = Fixture->Network, ZeroB = Fixture->Network;
		const std::uint32_t Coastal = First->Channels.front().Outlet.Connection;
		PlanetCore::Vec3 PassVector = Fixture->Network.Connections[Coastal].Pass.Direction.Vector();
		TestEqual(TEXT("符号付き零検査の独立な零成分"), PassVector.Y, 0.0);
		PassVector.Y = 0.0;
		ZeroA.Connections[Coastal].Pass.Direction = *PlanetCore::UnitDirection::FromVector(PassVector);
		PassVector.Y = -0.0;
		ZeroB.Connections[Coastal].Pass.Direction = *PlanetCore::UnitDirection::FromVector(PassVector);
		const PlanetCore::Vec3 A = ZeroA.Connections[Coastal].Pass.Direction.Vector();
		const PlanetCore::Vec3 B = ZeroB.Connections[Coastal].Pass.Direction.Vector();
		TestTrue(TEXT("独立な二方向は零以外の成分が一致"), A.X == B.X && A.Z == B.Z);
		TestTrue(TEXT("符号付き零のビットは異なる"), std::bit_cast<std::uint64_t>(A.Y) != std::bit_cast<std::uint64_t>(B.Y));
		PlanetCore::FlowNetworkPreparation ZeroPreparation(Fixture->Source, Fixture->Height, Settings);
		TestTrue(TEXT("正の零方向の河道を準備"), ZeroPreparation.Prepare(ZeroA).has_value());
		PlanetCore::FlowNetworkDiagnostics ZeroReport;
		const std::optional<PlanetCore::PreparedFlowNetwork> NegativeZero = ZeroPreparation.Prepare(ZeroB, {}, &ZeroReport);
		const std::optional<PlanetCore::PreparedFlowNetwork> IndependentZero =
		PlanetCore::PrepareFlowNetwork(ZeroB, Fixture->Source, Fixture->Height, Settings);
		TestTrue(TEXT("異なる入力ビットの旧河道を使わない"), NegativeZero && ZeroReport.RebuiltChannels > 0);
		TestTrue(
			TEXT("同じ入力から独立に準備した結果と一致"),
			NegativeZero && IndependentZero && PreparationValues(*NegativeZero) == PreparationValues(*IndependentZero)
		);
		// 同じ全河道の実保持量から，上限の両側と広い有界枠を独立に検査
		// 不正な測定値から容量の減算を続けない返戻
		if(!TestTrue(TEXT("比較対象の形状標本は複数"), FirstReport.RetainedGeometrySamples > 1)) return false;
		for(const std::uint32_t Capacity : { FirstReport.RetainedGeometrySamples, 524288U }) {
			Settings.MaximumGeometrySamples = Capacity;
			PlanetCore::FlowNetworkPreparation Enough(Fixture->Source, Fixture->Height, Settings);
			PlanetCore::FlowNetworkDiagnostics EnoughReport;
			const std::optional<PlanetCore::PreparedFlowNetwork> Within = Enough.Prepare(Fixture->Network, {}, &EnoughReport);
			TestTrue(TEXT("実保持量と広い有界枠で全河道を準備"), Within && PreparationValues(*Within) == Expected);
			TestEqual(TEXT("広い枠でも実標本数を増やさない"), EnoughReport.RetainedGeometrySamples, FirstReport.RetainedGeometrySamples);
			TestTrue(TEXT("公開標本は設定した枠以内"), EnoughReport.RetainedGeometrySamples <= Capacity);
		}
		Settings.MaximumGeometrySamples = FirstReport.RetainedGeometrySamples - 1;
		PlanetCore::FlowNetworkPreparation OneShort(Fixture->Source, Fixture->Height, Settings);
		TestFalse(TEXT("上限の一標本超過でも部分水系を公開しない"), OneShort.Prepare(Fixture->Network, {}, &Rejected).has_value());
		TestTrue(TEXT("一標本超過の実保持予算拒否"), Rejected.Failure == PlanetCore::FlowNetworkFailure::GeometryBudget);
		TestTrue(TEXT("拒否時の累計も上限内"), Rejected.RetainedGeometrySamples <= Settings.MaximumGeometrySamples);
		Settings.MaximumGeometrySamples = 1;
		PlanetCore::FlowNetworkPreparation Bounded(Fixture->Source, Fixture->Height, Settings);
		TestFalse(TEXT("別の保持上限を再利用で迂回しない"), Bounded.Prepare(Fixture->Network, {}, &Rejected).has_value());
		TestTrue(TEXT("保持上限の正しい拒否"), Rejected.Failure == PlanetCore::FlowNetworkFailure::GeometryBudget);
		AddInfo(
			FString::Printf(
				TEXT("Preparation reuse R=%.0f channels=%u calls=(%llu,%llu)/(%llu,%llu) components=%llu"),
				Radius,
				WarmReport.PreparedChannels,
				WarmCalls[0],
				WarmCalls[1],
				FirstCalls[0],
				FirstCalls[1],
				static_cast<unsigned long long>(Expected->size())
			)
		);
	}
	// 再利用と現在の利用契約の検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowConnectionOrderContracts,
	"PlanetGenerator.FlowChannel.ConnectionOrder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ物理水系の接続列だけを並換え，河床と淵の形状が列挙順に依存しない事を検査する
 * @param Parameters 未使用の実行引数
 * @return 大小三半径で，全物理量・河床・水位と全微分がビット一致する場合に真
 */
bool FPlanetFlowConnectionOrderContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	std::uint32_t ExercisedPools = 0;
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		PlanetCore::SpillNetworkDiagnostics Discovery;
		std::optional<PreparationFixture> Fixture = MakePreparationFixture(Radius, true, &Discovery);
		AddInfo(
			FString::Printf(
				TEXT("Order fixture R=%.0f prepared=%u failure=%u source=%llu steps=%u discovery=%u"),
				Radius,
				static_cast<unsigned>(Fixture.has_value()),
				static_cast<unsigned>(Discovery.Failure),
				Discovery.Evaluations,
				Discovery.Steps,
				static_cast<unsigned>(Discovery.Discovery.Failure)
			)
		);
		// 同じ実水系を準備出来ない場合の返戻
		if(!TestTrue(TEXT("順序比較の同じ実水系"), Fixture.has_value())) return false;
		PlanetCore::FlowNetworkSettings Settings;
		Settings.Channel.Outlet.RadiusMeters = Radius;
		Settings.Channel.Seed = 42;
		const std::optional<PlanetCore::PreparedFlowNetwork> Original =
		PlanetCore::PrepareFlowNetwork(Fixture->Network, Fixture->Source, Fixture->Height, Settings);
		// 正規の物理水系を準備出来ない場合の返戻
		if(!TestTrue(TEXT("並換え前の全河道"), Original.has_value())) return false;
		std::uint32_t Pools = 0;
		for(const PlanetCore::FlowBasinChannel &Basin : Original->Channels) Pools += Basin.Channel.PoolCount;
		ExercisedPools += Pools;
		const std::optional<std::vector<std::uint64_t>> Expected = PreparationValues(*Original, false);
		// 独立な地点評価を欠く場合の返戻
		if(!TestTrue(TEXT("元の物理値と全勾配"), Expected.has_value())) return false;
		for(unsigned Permutation = 0; Permutation < 2; ++Permutation) {
			PlanetCore::SpillNetwork Reordered = Fixture->Network;
			if(Permutation) std::rotate(Reordered.Connections.begin(), Reordered.Connections.begin() + 1, Reordered.Connections.end());
			else std::reverse(Reordered.Connections.begin(), Reordered.Connections.end());
			const auto IsOcean = [&](const PlanetCore::UnitDirection Direction) -> bool {
				const PlanetCore::Vec3 D = Direction.Vector();
				for(const PlanetCore::SpillOceanWitness &Witness : Reordered.OceanWitnesses) {
					const PlanetCore::Vec3 W = Witness.Terminal.Vector();
					// 元の同じ大洋所属を確認した実終端の返戻
					if(D.X == W.X && D.Y == W.Y && D.Z == W.Z) return true;
				}
				// 新しい大洋所属を捏造しない返戻
				return false;
			};
			PlanetCore::ContinuousSpillSettings GraphSettings;
			GraphSettings.RadiusMeters = Radius;
			std::optional<PlanetCore::ContinuousSpillGraph> Graph =
			PlanetCore::BuildContinuousSpillGraph(Reordered.Connections, IsOcean, GraphSettings);
			// 元の物理網を並換えただけの正規経路の返戻
			if(!TestTrue(TEXT("同じ実鞍点から排水階層を再構築"), Graph.has_value())) return false;
			Reordered.Graph = std::move(*Graph);
			TestTrue(TEXT("接続列だけを変え，物理水域の番号は保持"), Reordered.Graph.Hierarchy.SiteRoots == Fixture->Network.Graph.Hierarchy.SiteRoots);
			const std::optional<PlanetCore::PreparedFlowNetwork> Prepared =
			PlanetCore::PrepareFlowNetwork(Reordered, Fixture->Source, Fixture->Height, Settings);
			// 同じ元場から正規の河道を準備出来ない場合の返戻
			if(!TestTrue(TEXT("並換え後も全河道を準備"), Prepared.has_value())) return false;
			TestTrue(TEXT("接続番号に依らず，全形状・水位・微分がビット一致"), PreparationValues(*Prepared, false) == Expected);
			AddInfo(
				FString::Printf(
					TEXT("Connection order R=%.0f permutation=%u channels=%u pools=%u components=%llu"),
					Radius,
					Permutation,
					static_cast<unsigned>(Prepared->Channels.size()),
					Pools,
					static_cast<unsigned long long>(Expected->size())
				)
			);
		}
	}
	TestTrue(TEXT("全比較には乱数で変わる淵を実際に含める"), ExercisedPools > 0);
	// 同じ物理水系の列挙順不変性の検査結果の返戻
	return !HasAnyErrors();
}

#endif
