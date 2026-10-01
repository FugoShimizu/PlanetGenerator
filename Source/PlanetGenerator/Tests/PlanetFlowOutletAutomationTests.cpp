#include "Core/PlanetFlowChannel.h"
#include "Core/PlanetFlowOutlet.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetFlowOutletContracts, "PlanetGenerator.FlowChannel.Outlet",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 解析鞍点で湖・下降水位・端の開削を検査し，実地形でも同じ経路を確認する
 * @param Parameters 未使用の UE 引数
 * @return 独立な微分・水深・不正入力の検査を満たした場合に真
 */
bool FPlanetFlowOutletContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::FlowChannelShape;
	using PlanetCore::FlowChannelWaterLevel;
	using PlanetCore::FlowChannelWaterSettings;
	using PlanetCore::FlowOutletRoute;
	using PlanetCore::FlowOutletSettings;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	FlowChannelWaterSettings WaterSettings;
	WaterSettings.DownstreamWaterMeters = 92;
	double MaximumWaterDerivativeError = 0, MaximumBedDerivativeError = 0, MaximumPassPositionError = 0;
	std::uint32_t Samples = 0;
	// 深さ１㎝の解析窪地へ流入する経路；旧固定１０㎝終端では到達不能
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto ShallowSource = [Radius](const UnitDirection Direction) -> TerrainSample {
			const Vec3 D = Direction.Vector();
			const double X = Radius * D.Y, Y = Radius * D.Z, Squared = X * X;
			const double Height = 100 - 0.00005 * Squared + 6.25E-8 * Squared * Squared + 0.0001 * Y * Y;
			const Vec3 Gradient { 0, Radius * X * (-0.0001 + 2.5E-7 * Squared), Radius * 0.0002 * Y };
			// 極小９９．９９ｍの解析場と球面接線微分の返戻
			return { Height, Gradient - D * Gradient.Dot(D) };
		};
		FlowOutletSettings ShallowSettings;
		ShallowSettings.RadiusMeters = Radius;
		ShallowSettings.UpstreamLengthMeters = 10;
		ShallowSettings.MaximumDownstreamLengthMeters = 50;
		ShallowSettings.DownstreamWaterMeters = 99.995;
		PlanetCore::FlowOutletDiagnostics ShallowReport;
		const std::optional<FlowOutletRoute> Shallow =
		PlanetCore::PrepareFlowOutletRoute(*UnitDirection::FromVector({ 1, 0, 0 }), ShallowSource, ShallowSettings, {}, &ShallowReport);
		// 浅い正規の受入水域を拒否する場合の返戻
		if(!TestTrue(TEXT("三半径で浅い受入湖へ到達"), Shallow.has_value())) return false;
		const TerrainSample End = ShallowSource(Shallow->Route.Sample(Shallow->Route.GetLengthMeters())->Direction);
		TestTrue(TEXT("終端は実際に受入水面下であり湖底以上"), End.ElevationMeters < 99.995 && End.ElevationMeters >= 99.99 - 1E-12);
		TestTrue(TEXT("勾配停止を成功へ読替えない"), ShallowReport.DownstreamEnd == PlanetCore::TerrainDescentEnd::HeightLimit);
	}
	// 独立な放物線を入力し，遷移境界を含む非増加性を確認する
	double PreviousWater = 100;
	for(int i = -400; i < 1201; ++i) {
		const double S = i * 0.1;
		const auto WaterAt = [&](const double Distance) -> std::optional<FlowChannelWaterLevel> {
			// 既知の縦断を入力した水位候補の返戻
			return PlanetCore::SampleFlowOutletWaterLevel(
				{ 100 - 0.001 * Distance * Distance, -0.002 * Distance },
				Distance,
				100,
				WaterSettings
			);
		};
		const std::optional<FlowChannelWaterLevel> Water = WaterAt(S);
		// 有効な鞍点縦断を評価出来ない場合の返戻
		if(!TestTrue(TEXT("湖から下流まで同じ水位式を評価出来る"), Water.has_value())) return false;
		TestTrue(TEXT("湖は一定，河川は下流へ非増加"), Water->ElevationMeters <= PreviousWater && !(Water->DerivativePerMeter > 0));
		if(!(S > 0)) TestEqual(TEXT("湖の水面を元標高へ下げない"), Water->ElevationMeters, 100.0);
		if(S > 95) TestEqual(TEXT("下流静水面へ到達"), Water->ElevationMeters, 92.0);
		constexpr double Step = 1E-4;
		const double Difference = (WaterAt(S + Step)->ElevationMeters - WaterAt(S - Step)->ElevationMeters) / (2 * Step);
		MaximumWaterDerivativeError = std::max(MaximumWaterDerivativeError, std::abs(Difference - Water->DerivativePerMeter));
		PreviousWater = Water->ElevationMeters;
		++Samples;
	}
	TestTrue(TEXT("湖の接続を含む水位微分が独立差分と一致"), MaximumWaterDerivativeError < 1E-6);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto Source = [&](const UnitDirection Direction) -> TerrainSample {
			const Vec3 D = Direction.Vector();
			const double X = Radius * D.Y, Y = Radius * D.Z;
			const Vec3 G { 0, -0.002 * Radius * X, 0.0014 * Radius * Y };
			// 球面上で解析解を持つ鞍点の返戻
			return { 100 - 0.001 * X * X + 0.0007 * Y * Y, G - D * G.Dot(D) };
		};
		const UnitDirection Pass = *UnitDirection::FromVector({ 1, 0, 0 });
		FlowOutletSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.UpstreamLengthMeters = 40;
		Settings.MaximumDownstreamLengthMeters = 200;
		Settings.DownstreamWaterMeters = 92;
		const std::optional<FlowOutletRoute> Outlet = PlanetCore::PrepareFlowOutletRoute(Pass, Source, Settings);
		// 正常な両側の経路を準備出来ない場合の返戻
		if(!TestTrue(TEXT("三半径で鞍点を跨ぐ経路を準備"), Outlet.has_value())) return false;
		const PlanetCore::TerrainFlowRoute &Route = Outlet->Route;
		const Vec3 AtPass = Route.Sample(Outlet->PassDistanceMeters)->Direction.Vector();
		const double PassError = (AtPass - Pass.Vector()).Length() * Radius;
		MaximumPassPositionError = std::max(MaximumPassPositionError, PassError);
		TestTrue(TEXT("鞍点の前後に経路が存在"), Outlet->PassDistanceMeters > 40 && Outlet->PassDistanceMeters < Route.GetLengthMeters() - 80);
		TestTrue(TEXT("累積公称距離で鞍点の位置を保持"), PassError < 1E-5);
		FlowChannelShape Shape;
		Shape.RadiusMeters = Radius;
		const double Length = Route.GetLengthMeters();
		const auto Evaluate = [&](const UnitDirection Direction) -> std::optional<PlanetCore::FlowChannelSection> {
			const std::optional<PlanetCore::FlowRouteProjection> Projection = Route.FindFootpoint(Direction, 20, nullptr, true);
			// 領域外を端の断面へ丸めない返戻
			if(!Projection) return {};
			const PlanetCore::TerrainFlowFootpoint &Foot = Projection->Footpoint;
			const TerrainSample Center = Source(Foot.Curve.Direction);
			const std::optional<FlowChannelWaterLevel> Water = PlanetCore::SampleFlowOutletWaterLevel(
				{ Center.ElevationMeters, Center.TangentialGradientPerRadian.Dot(Foot.Curve.DirectionDerivativePerMeter) },
				Projection->DistanceMeters - Outlet->PassDistanceMeters,
				100,
				WaterSettings
			);
			const std::optional<FlowChannelShape> Faded = PlanetCore::FadeFlowChannelEnds(Shape, Projection->DistanceMeters, Length, 20);
			// 正規の河床又は水位を評価出来ない場合の返戻
			if(!Water || !Faded) return {};
			// 水位と河床の同時評価の返戻
			return PlanetCore::SampleFlowChannel(Foot, Source(Direction), *Water, *Faded);
		};
		for(const double Distance : { 0.0, Length }) {
			const std::optional<FlowChannelShape> Faded = PlanetCore::FadeFlowChannelEnds(Shape, Distance, Length, 20);
			TestTrue(TEXT("両端で開削の値と微分が零"), Faded && !Faded->CutWeight && !Faded->CutWeightDerivativePerMeter);
		}
		for(unsigned i = 1; i < 200; ++i) {
			const double S = Length * i / 200;
			const PlanetCore::TerrainFlowCurveSample Point = *Route.Sample(S);
			const Vec3 D = Point.Direction.Vector();
			const Vec3 Along = UnitDirection::FromVector(Point.DirectionDerivativePerMeter)->Vector(), Across = D.Cross(Along);
			for(const double Width : { -7.0, -3.0, 0.0, 3.0, 7.0 }) {
				const UnitDirection Query = *UnitDirection::FromVector(D * std::cos(Width / Radius) + Across * std::sin(Width / Radius));
				const std::optional<PlanetCore::FlowChannelSection> Here = Evaluate(Query);
				// 正規の経路内部を評価出来ない場合の返戻
				if(!TestTrue(TEXT("鞍点前後と端の遷移を評価出来る"), Here.has_value())) return false;
				if(!Width) TestTrue(TEXT("流出口と両側の中心水深が正"), Here->Water.ElevationMeters > Here->Terrain.ElevationMeters);
				for(const Vec3 Axis : { Along, Query.Vector().Cross(Along) }) {
					constexpr double Step = 0.001;
					const UnitDirection Low = *UnitDirection::FromVector(Query.Vector() * std::cos(Step / Radius) - Axis * std::sin(Step / Radius));
					const UnitDirection High =
					*UnitDirection::FromVector(Query.Vector() * std::cos(Step / Radius) + Axis * std::sin(Step / Radius));
					const std::optional<PlanetCore::FlowChannelSection> Lower = Evaluate(Low), Upper = Evaluate(High);
					// 差分の片側が欠落した結果を合格としない返戻
					if(!TestTrue(TEXT("独立差分の両端を評価出来る"), Lower && Upper)) return false;
					const double Difference = (Upper->Terrain.ElevationMeters - Lower->Terrain.ElevationMeters) / (2 * Step);
					const double Error = std::abs(Difference - Here->Terrain.TangentialGradientPerRadian.Dot(Axis) / Radius);
					MaximumBedDerivativeError = std::max(MaximumBedDerivativeError, Error);
					TestTrue(TEXT("端の強度を含む河床の二接線微分"), Error < 0.0005);
					++Samples;
				}
			}
		}
		for(unsigned Side = 0; Side < 2; ++Side) {
			const double Distance = Side ? Length : 0.0, Sign = Side ? 1.0 : -1.0;
			const PlanetCore::TerrainFlowCurveSample Point = *Route.Sample(Distance);
			const Vec3 Along = UnitDirection::FromVector(Point.DirectionDerivativePerMeter)->Vector();
			const UnitDirection Query =
			*UnitDirection::FromVector(Point.Direction.Vector() * std::cos(10 / Radius) + Along * (Sign * std::sin(10 / Radius)));
			TestFalse(TEXT("既定の直交照会は端の外を拒否"), Route.FindFootpoint(Query, 20).has_value());
			const std::optional<PlanetCore::FlowRouteProjection> Projection = Route.FindFootpoint(Query, 20, nullptr, true);
			// 明示した端の延長を評価出来ない場合の返戻
			if(!TestTrue(TEXT("端の最近点を明示して照会出来る"), Projection.has_value())) return false;
			TestEqual(TEXT("端の外では公称距離を固定"), Projection->DistanceMeters, Distance);
			TestEqual(TEXT("固定した公称距離の微分が零"), Projection->Footpoint.ParameterGradientMetersPerRadian.Length(), 0.0);
			const std::optional<PlanetCore::FlowChannelSection> Section = Evaluate(Query);
			// 両側の一定水面の評価に失敗した場合の返戻
			if(!TestTrue(TEXT("端の先でも水位を評価出来る"), Section && Section->HasWater)) return false;
			TestEqual(TEXT("端の外では元標高を正確に保持"), Section->Terrain.ElevationMeters, Source(Query).ElevationMeters);
			TestEqual(
				TEXT("端の外では元勾配を正確に保持"),
				(Section->Terrain.TangentialGradientPerRadian - Source(Query).TangentialGradientPerRadian).Length(),
				0.0
			);
			TestEqual(TEXT("端の外の水位は対応する静水面"), Section->Water.ElevationMeters, Side ? 92.0 : 100.0);
			TestEqual(TEXT("端の外の水面勾配は零"), Section->Water.TangentialGradientPerRadian.Length(), 0.0);
			TestFalse(TEXT("端の候補も検索半径の外なら拒否"), Route.FindFootpoint(Query, 5, nullptr, true).has_value());
			for(const Vec3 Axis : { Along, Query.Vector().Cross(Along) }) {
				constexpr double Step = 0.001;
				const UnitDirection Low = *UnitDirection::FromVector(Query.Vector() * std::cos(Step / Radius) - Axis * std::sin(Step / Radius));
				const UnitDirection High =
				*UnitDirection::FromVector(Query.Vector() * std::cos(Step / Radius) + Axis * std::sin(Step / Radius));
				const std::optional<PlanetCore::FlowRouteProjection> Lower = Route.FindFootpoint(Low, 20, nullptr, true);
				const std::optional<PlanetCore::FlowRouteProjection> Upper = Route.FindFootpoint(High, 20, nullptr, true);
				// 端の独立差分の評価失敗への返戻
				if(!TestTrue(TEXT("端の距離の差分標本が存在"), Lower && Upper)) return false;
				const double Difference =
				(Upper->Footpoint.SquaredChordDistance - Lower->Footpoint.SquaredChordDistance) * Radius * Radius / (2 * Step);
				TestTrue(
					TEXT("端の弦長二乗も二接線の差分と一致"),
					std::abs(Difference - Projection->Footpoint.SquaredChordGradientPerRadian.Dot(Axis) * Radius) < 0.001
				);
			}
		}
		FlowChannelShape Invalid = Shape;
		Invalid.CutWeight = -0.1;
		TestFalse(TEXT("負の開削強度を拒否"), Invalid.IsValid());
		Invalid = Shape;
		Invalid.CutWeightDerivativePerMeter = 0.1;
		TestFalse(TEXT("一定の基準断面へ架空の強度微分を渡せない"), PlanetCore::FlowChannelProfile::Create(Invalid, Length, {}).has_value());
		Invalid = Shape;
		Invalid.CutWeight = 1.1;
		TestFalse(TEXT("過大な開削強度を拒否"), Invalid.IsValid());
		TestFalse(TEXT("経路外の端接続を拒否"), PlanetCore::FadeFlowChannelEnds(Shape, -1, Length, 20).has_value());
		TestFalse(TEXT("重なる端接続を拒否"), PlanetCore::FadeFlowChannelEnds(Shape, 1, Length, Length).has_value());
		std::stop_source Cancellation;
		Cancellation.request_stop();
		TestFalse(
			TEXT("中断時に経路を公開しない"),
			PlanetCore::PrepareFlowOutletRoute(Pass, Source, Settings, Cancellation.get_token()).has_value()
		);
		Settings.DownstreamWaterMeters = 101;
		TestFalse(TEXT("鞍点より高い下流を拒否"), PlanetCore::PrepareFlowOutletRoute(Pass, Source, Settings).has_value());
	}
	TestFalse(TEXT("下流の上り返しを拒否"), PlanetCore::SampleFlowOutletWaterLevel({ 99, 0.1 }, 10, 100, WaterSettings).has_value());
	TestFalse(TEXT("鞍点より高い河川源場を拒否"), PlanetCore::SampleFlowOutletWaterLevel({ 101, -1 }, 10, 100, WaterSettings).has_value());
	FlowChannelWaterSettings InvalidWater = WaterSettings;
	InvalidWater.RampLengthMeters = 0;
	TestFalse(TEXT("湖側でも無効な設定を拒否"), PlanetCore::SampleFlowOutletWaterLevel({ 99, 1 }, -1, 100, InvalidWater).has_value());
	TestFalse(
		TEXT("非有限な源場を拒否"),
		PlanetCore::SampleFlowOutletWaterLevel({ std::numeric_limits<double>::quiet_NaN(), 0 }, -1, 100, WaterSettings).has_value()
	);
	// 実地形の一つの流出口で，同じ連続場と両側の準備を確認する
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 実地形の依存場が欠けた場合の返戻
	if(!TestTrue(TEXT("実流出口の地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山の準備失敗への返戻
	if(!TestTrue(TEXT("実流出口の造山を準備"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 元の連続場の準備失敗への返戻
	if(!TestTrue(TEXT("実流出口の元場を準備"), Ocean.has_value())) return false;
	const auto Field = [&](const UnitDirection Direction) -> TerrainSample {
		// 元場を変えない返戻
		return Ocean->Evaluate(Direction);
	};
	const UnitDirection KnownPass = *UnitDirection::FromVector({ 0.54180127453542282, 0.81192128360203597, -0.21733662403243018 });
	FlowOutletSettings Settings;
	Settings.DownstreamWaterMeters = 681.98759;
	const std::optional<FlowOutletRoute> Outlet = PlanetCore::PrepareFlowOutletRoute(KnownPass, Field, Settings);
	// 実際の両側経路が作れない場合の返戻
	if(!TestTrue(TEXT("実湖の鞍点から下流の水位へ届く"), Outlet.has_value())) return false;
	WaterSettings.DownstreamWaterMeters = Settings.DownstreamWaterMeters;
	// 広い湖表示の独立格子で失敗した地点を，描画と切離して再現する
	const UnitDirection FailedQuery = *UnitDirection::FromVector({ 0.5419872858408131, 0.8117835049625941, -0.21738749517291672 });
	const std::optional<PlanetCore::FlowRouteProjection> FailedProjection =
	Outlet->Route.FindFootpoint(FailedQuery, 500, nullptr, true);
	// 河道の近傍で投影が欠落した場合の返戻
	if(!TestTrue(TEXT("湖の独立格子の投影が存在"), FailedProjection.has_value())) return false;
	const PlanetCore::TerrainFlowFootpoint &Foot = FailedProjection->Footpoint;
	const TerrainSample FailedCenter = Field(Foot.Curve.Direction);
	const double FailedSlope = FailedCenter.TangentialGradientPerRadian.Dot(Foot.Curve.DirectionDerivativePerMeter);
	const std::optional<FlowChannelWaterLevel> FailedWater = PlanetCore::SampleFlowOutletWaterLevel(
		{ FailedCenter.ElevationMeters, FailedSlope },
		FailedProjection->DistanceMeters - Outlet->PassDistanceMeters,
		Outlet->Pass.ElevationMeters,
		WaterSettings
	);
	AddInfo(
		FString::Printf(
			TEXT(
				"Outlet wide query: segment=%u fraction=%.17g signedDistance=%.17g sourceHeight=%.17g passHeight=%.17g slope=%.17g water=%d"
			),
			FailedProjection->SegmentIndex,
			Foot.Fraction,
			FailedProjection->DistanceMeters - Outlet->PassDistanceMeters,
			FailedCenter.ElevationMeters,
			Outlet->Pass.ElevationMeters,
			FailedSlope,
			static_cast<int>(FailedWater.has_value())
		)
	);
	TestTrue(TEXT("湖の独立格子でも水位が定義される"), FailedWater.has_value());
	const std::optional<FlowChannelShape> FailedFade =
	PlanetCore::FadeFlowChannelEnds({}, FailedProjection->DistanceMeters, Outlet->Route.GetLengthMeters(), 25);
	AddInfo(FString::Printf(TEXT("Outlet wide fade: valid=%d"), static_cast<int>(FailedFade.has_value())));
	TestTrue(TEXT("湖の独立格子でも河床強度が定義される"), FailedFade.has_value());
	std::uint32_t InvalidFades = 0;
	for(unsigned i = 1; i < 10001; ++i) {
		const double Distance = 25 * (1 - i * 1E-9);
		if(!PlanetCore::FadeFlowChannelEnds({}, Distance, 1000, 25)) ++InvalidFades;
	}
	AddInfo(FString::Printf(TEXT("Outlet end rounding: undefined=%u samples=10000"), InvalidFades));
	TestEqual(TEXT("端接続に近い正規の距離を丸め誤差で拒否しない"), InvalidFades, 0U);
	double Previous = Outlet->Pass.ElevationMeters, MinimumDepth = std::numeric_limits<double>::infinity();
	std::uint32_t ActualSamples = 0;
	for(unsigned i = 0; i < 4001; ++i) {
		const double S = Outlet->Route.GetLengthMeters() * i / 4000;
		const PlanetCore::TerrainFlowCurveSample Point = *Outlet->Route.Sample(S);
		const TerrainSample Center = Field(Point.Direction);
		const std::optional<FlowChannelWaterLevel> Water = PlanetCore::SampleFlowOutletWaterLevel(
			{ Center.ElevationMeters, Center.TangentialGradientPerRadian.Dot(Point.DirectionDerivativePerMeter) },
			S - Outlet->PassDistanceMeters,
			Outlet->Pass.ElevationMeters,
			WaterSettings
		);
		// 実縦断の不整合を上書きして隠さない返戻
		if(!TestTrue(TEXT("実流出口の水位を評価出来る"), Water.has_value())) return false;
		TestTrue(TEXT("実流出口の水位も下流へ非増加"), Water->ElevationMeters <= Previous + 1E-9);
		const FlowChannelShape Faded = *PlanetCore::FadeFlowChannelEnds({}, S, Outlet->Route.GetLengthMeters(), 25);
		MinimumDepth = std::min(MinimumDepth, Water->ElevationMeters - Center.ElevationMeters + Faded.DepthMeters * Faded.CutWeight);
		Previous = Water->ElevationMeters;
		++ActualSamples;
	}
	TestTrue(TEXT("実湖から下流まで中心の水深が正"), MinimumDepth > 0);
	AddInfo(
		FString::Printf(
			TEXT("Flow outlet contracts: samples=%u waterDerivativeError=%.12g bedDerivativeError=%.12g ")
			TEXT("passPositionError=%.12g actualSamples=%u minimumDepth=%.12g passDistance=%.12g routeLength=%.12g"),
			Samples,
			MaximumWaterDerivativeError,
			MaximumBedDerivativeError,
			MaximumPassPositionError,
			ActualSamples,
			MinimumDepth,
			Outlet->PassDistanceMeters,
			Outlet->Route.GetLengthMeters()
		)
	);
	// 解析条件と実際の流出口に対する検査結果の返戻
	return !HasAnyErrors();
}

#endif
