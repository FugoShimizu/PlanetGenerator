#include "Core/PlanetFlowNetworkField.h"
#include "Core/PlanetLakeTopology.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetCompoundLakeFlowContracts,
	"PlanetGenerator.FlowChannel.CompoundLake",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同水位の二つの実極小を一つの湖と流出河川に纏め，同じ元場の所有と通水を検査する
 * @param Parameters 未使用の自動試験引数
 * @return 補正済地形・湖岸・流出口・水位・準備後の所有権が成立する場合に真
 */
bool FPlanetCompoundLakeFlowContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	std::uint32_t Cases = 0, WetSamples = 0;
	double MinimumDepth = std::numeric_limits<double>::infinity(), MaximumDifference = 0;
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto At = [Radius](const double X, const double Y) -> UnitDirection {
			// 解析湖の心射座標の返戻
			return *UnitDirection::FromVector({ X / Radius, Y / Radius, 1 });
		};
		std::function<TerrainSample(UnitDirection)> Source = [Radius](const UnitDirection Direction) -> TerrainSample {
			const Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, T = X / 80, A = T * (T - 1), B = 0.2 * T - 0.3375;
			const double Derivative = 100 * (2 * A * (2 * T - 1) * B + 0.2 * A * A) / 80;
			const Vec3 DX = (Vec3{ 1, 0, 0 } * D.Z - Vec3{ 0, 0, 1 } * D.X) * (Radius / (D.Z * D.Z));
			const Vec3 DY = (Vec3{ 0, 1, 0 } * D.Z - Vec3{ 0, 0, 1 } * D.Y) * (Radius / (D.Z * D.Z));
			// 鞍点０・８０の標高は共に１０，右端は閉じた岸，左だけが大洋へ下降
			return { 10 + 100 * A * A * B + Y * Y / 400, DX * Derivative + DY * (Y / 200) };
		};
		std::function<double(UnitDirection)> Height = [Source](const UnitDirection Direction) -> double {
			// 同じ解析地形の標高の返戻
			return Source(Direction).ElevationMeters;
		};
		PlanetCore::SpillNetwork Network;
		PlanetCore::SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.RadiusMeters = Radius;
		for(const double X : { 0.0, 80.0 }) {
			PlanetCore::SpillConnectionDiagnostics Report;
			const std::optional<PlanetCore::SpillConnection> Connection =
			PlanetCore::PrepareSpillConnection(At(X, 0), Source, ConnectionSettings, {}, &Report);
			AddInfo(
				FString::Printf(
					TEXT("Compound connection R=%.12g x=%.12g accepted=%u failure=%u branch=%u end=%u"),
					Radius,
					X,
					static_cast<unsigned>(Connection.has_value()),
					static_cast<unsigned>(Report.Failure),
					Report.Branch,
					static_cast<unsigned>(Report.TraceEnd)
				)
			);
			// 元場の実接続を得られない場合の返戻
			if(!TestTrue(TEXT("解析元場で実際の鞍点と下降枝を準備"), Connection.has_value())) return false;
			TestEqual(TEXT("同水位の鞍点を丸めず保持"), Connection->Pass.ElevationMeters, 10.0);
			Network.Connections.push_back(*Connection);
		}
		const UnitDirection Ocean = At(-100, 0);
		for(const PlanetCore::SpillConnection &Connection : Network.Connections) {
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
				if(Branch.Kind != PlanetCore::SpillTerminalKind::SeaLevel) continue;
				const UnitDirection End = Branch.Path.Samples.back().Direction;
				double Highest = -std::numeric_limits<double>::infinity();
				for(unsigned I = 0; I < 129; ++I) {
					const UnitDirection D = *UnitDirection::FromVector(End.Vector() * (1 - I / 128.0) + Ocean.Vector() * (I / 128.0));
					Highest = std::max(Highest, Height(D));
				}
				TestTrue(TEXT("解析大洋迄の独立な通路は水没"), Highest <= 0);
				Network.OceanWitnesses.push_back({ End, Ocean, Highest });
			}
		}
		PlanetCore::ContinuousSpillSettings GraphSettings;
		GraphSettings.RadiusMeters = Radius;
		const auto IsOcean = [&Network](const UnitDirection Direction) -> bool {
			for(const PlanetCore::SpillOceanWitness &Witness : Network.OceanWitnesses) {
				// 確認済の実終端だけを受理する返戻
				if(!(Direction.Vector() - Witness.Terminal.Vector()).Length()) return true;
			}
			// 未確認の別水域を大洋としない返戻
			return false;
		};
		const std::optional<PlanetCore::ContinuousSpillGraph> Graph =
		PlanetCore::BuildContinuousSpillGraph(Network.Connections, IsOcean, GraphSettings);
		// 元場の網が未確認の場合の返戻
		if(!TestTrue(TEXT("同水位の実接続を階層へ対応"), Graph.has_value())) return false;
		Network.Graph = *Graph;
		const std::optional<PlanetCore::LakeTopology> Topology = PlanetCore::BuildLakeTopology(Graph->Hierarchy);
		TestTrue(TEXT("大洋と一つの複合湖"), Topology && Topology->Complexes.size() == 2);
		PlanetCore::FlowNetworkSettings Settings;
		Settings.Channel.Outlet.RadiusMeters = Radius;
		PlanetCore::FlowNetworkDiagnostics Report;
		std::optional<PlanetCore::PreparedFlowNetwork> Prepared =
		PlanetCore::PrepareFlowNetwork(Network, Source, Height, Settings, {}, &Report);
		AddInfo(
			FString::Printf(
				TEXT("Compound lake R=%.12g accepted=%u failure=%u basin=%u compound=%u sills=%u channels=%u channelFailure=%u shore=%u"),
				Radius,
				static_cast<unsigned>(Prepared.has_value()),
				static_cast<unsigned>(Report.Failure),
				Report.Basin,
				Report.CompoundWaterBodies,
				Report.LoweredSills,
				Report.PreparedChannels,
				static_cast<unsigned>(Report.Channel.Failure),
				static_cast<unsigned>(Report.Channel.Shore.Failure)
			)
		);
		AddInfo(
			FString::Printf(
				TEXT("Compound outlet failure=%u upstream=%u downstream=%u vertices=%u/%u endH=%.17g endD=%.17g move=%.17g minimum=%u"),
				static_cast<unsigned>(Report.Channel.Outlet.Failure),
				static_cast<unsigned>(Report.Channel.Outlet.UpstreamEnd),
				static_cast<unsigned>(Report.Channel.Outlet.DownstreamEnd),
				Report.Channel.Outlet.UpstreamVertices,
				Report.Channel.Outlet.DownstreamVertices,
				Report.Channel.Outlet.UpstreamHeightMeters,
				Report.Channel.Outlet.UpstreamDistanceMeters,
				Report.Channel.Outlet.UpstreamMinimumMoveMeters,
				static_cast<unsigned>(Report.Channel.Outlet.HasUpstreamMinimum)
			)
		);
		// 部分的な湖を成功としない返戻
		if(!TestTrue(TEXT("同水位湖を一つの流出河川へ準備"), Prepared.has_value())) return false;
		TestEqual(TEXT("架空の同水位河川を作らない"), Prepared->Channels.size(), std::size_t{ 1 });
		TestEqual(TEXT("内部鞍点だけを開削"), Report.LoweredSills, std::uint32_t{ 1 });
		TestEqual(TEXT("大洋へ達する一つの流出口を保持"), Report.CoastalChannels, std::uint32_t{ 1 });
		const PlanetCore::FlowWaterGeometry Geometry = Prepared->Channels.front().Channel.Field->GetWaterGeometry();
		TestTrue(TEXT("同じ湖岸が両極小を含む"), Geometry.Upstream.Contains(At(36, 0)) && Geometry.Upstream.Contains(At(120, 0)));
		TestEqual(TEXT("外部鞍点の標高を保持"), Prepared->SourceHeight(At(0, 0)), Height(At(0, 0)));
		TestTrue(TEXT("元場を変更せず内部鞍点だけ２５センチ下げる"), std::abs(Prepared->SourceHeight(At(80, 0)) - 9.75) < 1E-10);
		std::optional<PlanetCore::FlowNetworkField> Field = PlanetCore::FlowNetworkField::Create(*Prepared);
		// 表示と衝突へ渡す共通場を失う場合の返戻
		if(!TestTrue(TEXT("補正を所有する合成場を準備"), Field.has_value())) return false;
		const std::shared_ptr<const PlanetCore::FlowChannelField> Channel = Prepared->Channels.front().Channel.Field;
		// 呼出側と準備用コンテナの寿命を終了し，公開場だけで照会
		Prepared.reset();
		Source = {};
		Height = {};
		Network = {};
		for(unsigned X = 0; X < 33; ++X) for(int Y = -2; Y < 3; ++Y) {
			const UnitDirection D = At(64 + X, Y * 0.25);
			const std::optional<PlanetCore::FlowWaterSample> Value = Field->EvaluateWithSupport(D);
			const std::optional<PlanetCore::FlowChannelSection> Single = Channel->Evaluate(D);
			// 所属・水位・河床の欠落を除外しない返戻
			if(!TestTrue(TEXT("全通水標本で地形と水域を評価"), Value && Single && Value->Section.HasWater && Single->HasWater)) return false;
			TestTrue(TEXT("通水部は同じ水域の内側"), Value->SupportMeters > 0);
			TestEqual(TEXT("内部通水部の湖水位は一定"), Value->Section.Water.ElevationMeters, 10.0);
			MinimumDepth = std::min(MinimumDepth, Value->Section.Water.ElevationMeters - Value->Section.Terrain.ElevationMeters);
			MaximumDifference =
			std::max(MaximumDifference, std::abs(Value->Section.Terrain.ElevationMeters - Single->Terrain.ElevationMeters));
			MaximumDifference = std::max(
				MaximumDifference,
				(Value->Section.Terrain.TangentialGradientPerRadian - Single->Terrain.TangentialGradientPerRadian).Length() / Radius
			);
			const std::optional<PlanetCore::FlowWaterSample> HeightOnly = Field->EvaluateWithSupport(D, false);
			TestTrue(
				TEXT("微分省略経路も同じ補正を保持"),
				HeightOnly && HeightOnly->Section.Terrain.ElevationMeters == Value->Section.Terrain.ElevationMeters &&
				HeightOnly->SupportMeters == Value->SupportMeters
			);
			++WetSamples;
		}
		++Cases;
	}
	TestTrue(TEXT("有限幅の湖間通路に正の水深"), MinimumDepth > 0.01);
	TestTrue(TEXT("単独河道と合成場の元地形が一致"), MaximumDifference < 1E-10);
	AddInfo(
		FString::Printf(
			TEXT("Compound flow contracts: cases=%u wetSamples=%u minDepth=%.12g difference=%.12g"),
			Cases,
			WetSamples,
			MinimumDepth,
			MaximumDifference
		)
	);
	// 地形所有と通水の検査結果の返戻
	return !HasAnyErrors();
}

#endif
