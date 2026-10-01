#include "Core/PlanetFlowWatershed.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillNetwork.h"
#include "Core/PlanetSpillSeed.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <stop_token>
#include <tuple>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillSeedContracts,
	"PlanetGenerator.ContinuousSpill.LocationSeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 峠でない地点から実盆地と出口を発見し，解析解及び実水系で照合する
 * @param Parameters 未使用の実行引数
 * @return 元場の保持・予算・実接続を確認した場合に真
 */
bool FPlanetSpillSeedContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 解析的な鞍点と極小を持つ傾いた周期場の返戻
			return { 4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400, Gradient };
		};
		const double MinimumX = 40 * (5 * std::acos(-1.0) - std::asin(0.2));
		const PlanetCore::UnitDirection Location = *PlanetCore::UnitDirection::FromVector({ (MinimumX + 12) / Radius, 17 / Radius, 1 });
		PlanetCore::SpillSeedSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -1;
		PlanetCore::SpillSeedDiagnostics Report;
		const PlanetCore::TerrainSample Baseline = Source(Location);
		Calls = 0;
		const std::optional<PlanetCore::SpillSeed> Seed = PlanetCore::FindSpillSeed(Location, Source, Settings, {}, &Report);
		AddInfo(
			FString::Printf(
				TEXT("Location seed R=%.0f accepted=%u failure=%u evaluations=%llu windows=%u"),
				Radius,
				static_cast<unsigned>(Seed.has_value()),
				static_cast<unsigned>(Report.Failure),
				static_cast<unsigned long long>(Report.Evaluations),
				Report.Discovery.Windows
			)
		);
		// 発見出来ない場合は解析解との比較を進めない返戻
		if(!TestTrue(TEXT("一般地点から同じ盆地の実出口を発見"), Seed.has_value())) return false;
		const PlanetCore::Vec3 Minimum = Seed->Minimum.Direction.Vector(), Pass = Seed->Connection.Pass.Direction.Vector();
		const double ExpectedPassX = 40 * std::asin(0.2) + 160 * std::acos(-1.0);
		TestTrue(TEXT("解析解の極小に到達"), std::abs(Radius * Minimum.X / Minimum.Z - MinimumX) < 0.001);
		TestTrue(TEXT("解析解の峠へ接続"), std::abs(Radius * Pass.X / Pass.Z - ExpectedPassX) < 0.001);
		TestEqual(TEXT("全準備の外側で評価数を照合"), Report.Evaluations, Calls);
		const PlanetCore::TerrainSample Repeated = Source(Location);
		TestEqual(TEXT("発見処理が元の標高を変更しない"), Baseline.ElevationMeters, Repeated.ElevationMeters);
		Settings.MaximumEvaluations = 100;
		Calls = 0;
		TestFalse(TEXT("途中の共通評価予算を守る"), PlanetCore::FindSpillSeed(Location, Source, Settings, {}, &Report).has_value());
		TestEqual(TEXT("全段階を合わせた呼出上限"), Calls, UINT64_C(100));
		TestTrue(TEXT("評価予算を別の失敗に置換えない"), Report.Failure == PlanetCore::SpillSeedFailure::BudgetLimit);
		std::stop_source Stop;
		Stop.request_stop();
		Calls = 0;
		TestFalse(TEXT("中断済では元場を呼出さない"), PlanetCore::FindSpillSeed(Location, Source, Settings, Stop.get_token(), &Report).has_value());
		TestEqual(TEXT("中断後の評価は零"), Calls, UINT64_C(0));
		TestTrue(TEXT("中断の理由を保持"), Report.Failure == PlanetCore::SpillSeedFailure::Cancelled);
		Settings.MaximumEvaluations = 0;
		TestFalse(TEXT("零の評価予算を拒否"), PlanetCore::FindSpillSeed(Location, Source, Settings, {}, &Report).has_value());
		TestEqual(TEXT("不正設定では元場を照会しない"), Calls, UINT64_C(0));
		TestTrue(TEXT("不正設定を識別"), Report.Failure == PlanetCore::SpillSeedFailure::InvalidInput);
		Settings.MaximumEvaluations = 4000000;
		const PlanetCore::UnitDirection Sea = *PlanetCore::UnitDirection::FromVector({ -120 / Radius, 0, 1 });
		TestFalse(TEXT("水没地点へ架空の峠を作らない"), PlanetCore::FindSpillSeed(Sea, Source, Settings, {}, &Report).has_value());
		TestTrue(TEXT("海面への到達を未確認の盆地と区別"), Report.Failure == PlanetCore::SpillSeedFailure::ReachesSeaLevel);
		const auto Invalid = [](const PlanetCore::UnitDirection) -> PlanetCore::TerrainSample {
			// 不正な元場の拒否を確認する返戻
			return { std::numeric_limits<double>::quiet_NaN(), {} };
		};
		TestFalse(TEXT("非有限な元場を拒否"), PlanetCore::FindSpillSeed(Location, Invalid, Settings, {}, &Report).has_value());
		TestTrue(TEXT("元場の欠落を識別"), Report.Failure == PlanetCore::SpillSeedFailure::InvalidSource);
	}
	// 実地形でも既知の鞍点を渡さず，通常の陸上地点から始める
	PlanetCore::GeologySettings Definition;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 必要な準備に失敗した場合の返戻
	if(!TestTrue(TEXT("実地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 必要な準備に失敗した場合の返戻
	if(!TestTrue(TEXT("実造山場を準備"), Ridges.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> Prepared = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 必要な準備に失敗した場合の返戻
	if(!TestTrue(TEXT("実大洋の接続場を準備"), Prepared.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*Prepared));
	const auto Source = [Ocean](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		// 診断面の補間を介さない元の連続地形の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Location =
	*PlanetCore::UnitDirection::FromVector({ 0.54178538420982891, 0.81195543427398209, -0.21724863684165494 });
	const PlanetCore::TerrainSample Before = Source(Location);
	// 旧停止条件の終端と，広い範囲で独立に精査した実極小の差を測る
	PlanetCore::TerrainDescentSettings PreviousDescent;
	PreviousDescent.PositionToleranceMeters = 1E-5;
	const std::optional<PlanetCore::TerrainDescentPath> PreviousPath =
	PlanetCore::TraceTerrainDescent(Location, Source, PreviousDescent);
	// 旧経路の再現に失敗した場合の返戻
	if(!TestTrue(TEXT("初回失敗の下降経路を再現"), PreviousPath.has_value())) return false;
	const PlanetCore::UnitDirection PreviousEnd = PreviousPath->Samples.back().Direction;
	const std::optional<PlanetCore::TerrainCriticalPoint> ReferenceMinimum =
	PlanetCore::RefineTerrainCriticalPoint(PreviousEnd, Source, {});
	// 比較する実極小を確認出来ない場合の返戻
	if(!TestTrue(TEXT("独立な広域精査で極小を確認"), ReferenceMinimum.has_value())) return false;
	const PlanetCore::Vec3 PreviousDirection = PreviousEnd.Vector(), MinimumDirection = ReferenceMinimum->Direction.Vector();
	const double PreviousDistance = Definition.RadiusMeters *
	std::atan2(PreviousDirection.Cross(MinimumDirection).Length(), PreviousDirection.Dot(MinimumDirection));
	AddInfo(
		FString::Printf(
			TEXT("Previous descent end gradient=%.12g minimumDistanceMeters=%.12g minimumKind=%u"),
			Source(PreviousEnd).TangentialGradientPerRadian.Length() / Definition.RadiusMeters,
			PreviousDistance,
			static_cast<unsigned>(ReferenceMinimum->Kind)
		)
	);
	const double Started = FPlatformTime::Seconds();
	PlanetCore::SpillSeedDiagnostics Report;
	const std::optional<PlanetCore::SpillSeed> Seed = PlanetCore::FindSpillSeed(Location, Source, {}, {}, &Report);
	AddInfo(
		FString::Printf(
			TEXT("Actual location seed accepted=%u failure=%u descent=%u evaluations=%llu windows=%u elapsedMs=%.9g"),
			static_cast<unsigned>(Seed.has_value()),
			static_cast<unsigned>(Report.Failure),
			static_cast<unsigned>(Report.DescentEnd),
			static_cast<unsigned long long>(Report.Evaluations),
			Report.Discovery.Windows,
			(FPlatformTime::Seconds() - Started) * 1000
		)
	);
	// 必要な準備に失敗した場合の返戻
	if(!TestTrue(TEXT("実陸上地点から流出口を発見"), Seed.has_value())) return false;
	TestTrue(TEXT("入力地点は停留点ではない"), Before.TangentialGradientPerRadian.Length() > 1);
	TestTrue(TEXT("旧停止条件は１ cm の確認範囲に到達していない"), PreviousDistance > 0.01);
	const PlanetCore::Vec3 Confirmed = Seed->Minimum.Direction.Vector();
	const double IndependentDistance =
	Definition.RadiusMeters * std::atan2(Confirmed.Cross(MinimumDirection).Length(), Confirmed.Dot(MinimumDirection));
	TestTrue(TEXT("新しい下降終端は独立な実極小と一致"), IndependentDistance < 0.001);
	TestEqual(TEXT("実地点の元標高を保持"), Source(Location).ElevationMeters, Before.ElevationMeters);
	const std::optional<PlanetCore::SphereMesh> Sphere = PlanetCore::BuildUniformSphere(6);
	// 必要な準備に失敗した場合の返戻
	if(!TestTrue(TEXT("大洋の標本を準備"), Sphere.has_value())) return false;
	std::vector<PlanetCore::UnitDirection> Anchors;
	std::set<std::tuple<double, double, double>> Keys;
	for(const PlanetCore::UnitDirection D : Sphere->Directions) {
		if(!(Ocean->EvaluateElevation(D) < -1)) continue;
		Anchors.push_back(D);
		const PlanetCore::Vec3 P = D.Vector();
		Keys.emplace(P.X, P.Y, P.Z);
	}
	const auto IsOcean = [&](const PlanetCore::UnitDirection D) -> bool {
		const PlanetCore::Vec3 P = D.Vector();
		// 別途接続を確認した大洋の標本だけの返戻
		return Ocean->Statistics().ComponentsAfter == 1 && Keys.contains({ P.X, P.Y, P.Z });
	};
	PlanetCore::SpillNetworkDiagnostics NetworkReport;
	const std::optional<PlanetCore::SpillNetwork> Network =
	PlanetCore::PrepareSpillNetwork(Seed->Connection.Pass.Direction, Source, Anchors, IsOcean, {}, {}, &NetworkReport);
	AddInfo(
		FString::Printf(
			TEXT("Location network accepted=%u failure=%u evaluations=%llu"),
			static_cast<unsigned>(Network.has_value()),
			static_cast<unsigned>(NetworkReport.Failure),
			static_cast<unsigned long long>(NetworkReport.Evaluations)
		)
	);
	TestTrue(TEXT("地点から発見した水系が確認済の大洋へ到達"), Network.has_value());
	// 同じ地点から，診断アクター無しで河床と水面の所有権迄を準備する
	PlanetCore::FlowWatershedDiagnostics WatershedReport;
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(
		TEXT("共有準備も開始前の中断を守る"),
		PlanetCore::PrepareFlowWatershed(Location, Ocean, Cancel.get_token(), &WatershedReport).has_value()
	);
	TestTrue(TEXT("共有準備の中断理由を保持"), WatershedReport.Failure == PlanetCore::FlowWatershedFailure::Cancelled);
	TestEqual(TEXT("中断時に初期盆地を照会しない"), WatershedReport.Seed.Evaluations, UINT64_C(0));
	TestFalse(TEXT("元の大洋場が無ければ準備しない"), PlanetCore::PrepareFlowWatershed(Location, {}, {}, &WatershedReport).has_value());
	TestTrue(TEXT("元場の欠落を識別"), WatershedReport.Failure == PlanetCore::FlowWatershedFailure::InvalidOcean);
	const std::uint64_t OriginalHash = Ocean->DefinitionHash();
	const double PrepareAt = FPlatformTime::Seconds();
	std::optional<PlanetCore::PreparedFlowWatershed> Watershed =
	PlanetCore::PrepareFlowWatershed(Location, Ocean, {}, &WatershedReport);
	AddInfo(
		FString::Printf(
			TEXT("Location watershed accepted=%u failure=%u channels=%u coastal=%u anchors=%u elapsedMs=%.9g"),
			static_cast<unsigned>(Watershed.has_value()),
			static_cast<unsigned>(WatershedReport.Failure),
			WatershedReport.Channels.PreparedChannels,
			WatershedReport.Channels.CoastalChannels,
			WatershedReport.OceanAnchors,
			(FPlatformTime::Seconds() - PrepareAt) * 1000
		)
	);
	// 完成していない水系を表示用の場として検査しない返戻
	if(!TestTrue(TEXT("地点から共有河床と水面を準備"), Watershed.has_value())) return false;
	TestEqual(TEXT("元の大洋定義を変更しない"), Watershed->Ocean->DefinitionHash(), OriginalHash);
	const std::shared_ptr<const PlanetCore::FlowNetworkField> OwnedField = Watershed->Field;
	const std::optional<PlanetCore::FlowWaterSample> BeforeRelease = OwnedField->EvaluateWithSupport(Location);
	// 公開照会の欠落への返戻
	if(!TestTrue(TEXT("所有する場の公開照会が有効"), BeforeRelease.has_value())) return false;
	Watershed.reset();
	const std::optional<PlanetCore::FlowWaterSample> AfterRelease = OwnedField->EvaluateWithSupport(Location);
	// 結果構造体の寿命と共有場の寿命を混同しない返戻
	if(!TestTrue(TEXT("準備結果の解放後も表示用の場が有効"), AfterRelease.has_value())) return false;
	TestEqual(
		TEXT("準備結果の解放で標高が変わらない"),
		BeforeRelease->Section.Terrain.ElevationMeters,
		AfterRelease->Section.Terrain.ElevationMeters
	);
	std::uint32_t WaterSamples = 0;
	for(const PlanetCore::FlowWaterGeometry &Geometry : OwnedField->GetWaterGeometries()) for(unsigned I = 0; I < 33; ++I) {
		const std::optional<PlanetCore::TerrainFlowCurveSample> Point =
		Geometry.Route.Sample(Geometry.Route.GetLengthMeters() * I / 32);
		// 曲線の欠落を水面検査から除外しない返戻
		if(!TestTrue(TEXT("準備した曲線を公開後に照会"), Point.has_value())) return false;
		const std::optional<PlanetCore::FlowWaterSample> Value = OwnedField->EvaluateWithSupport(Point->Direction);
		// 一つでも欠けた河道が有れば返戻
		if(!TestTrue(TEXT("準備した全河道の照会が有効"), Value.has_value())) return false;
		TestTrue(TEXT("河床の標高が有限"), std::isfinite(Value->Section.Terrain.ElevationMeters));
		if(I && I < 32) {
			TestTrue(TEXT("河道内部に実水位が存在"), Value->Section.HasWater);
			TestTrue(TEXT("河道内部の水位が有限"), std::isfinite(Value->Section.Water.ElevationMeters));
			TestTrue(TEXT("河道内部の水位が河床より高い"), Value->Section.Water.ElevationMeters > Value->Section.Terrain.ElevationMeters);
		}
		++WaterSamples;
	}
	AddInfo(
		FString::Printf(
			TEXT("Location watershed retained samples=%u oceanAnchorBytes=%llu"),
			WaterSamples,
			static_cast<unsigned long long>(Ocean->ConnectedOceanAnchors().size_bytes())
		)
	);
	TestTrue(TEXT("空の水系を完成としない"), WaterSamples > 0);
	// 有限な一水系の確認を全球の全水系と区別した返戻
	return !HasAnyErrors();
}

#endif
