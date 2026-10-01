#include "Core/PlanetErosionPreparation.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetAeolianContracts, "PlanetGenerator.Aeolian.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 固定砂丘の有限な高さ，解析勾配，海岸保護，再現性と河川準備との共有を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，個別の不一致は UE に記録
 */
bool FPlanetAeolianContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Start = FPlatformTime::Seconds();
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
	if(!TestTrue(TEXT("地質を準備"), Geology.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
	if(!TestTrue(TEXT("海域を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
		std::make_shared<const PlanetCore::OceanConnectivityField>(*PreparedOcean);
	const std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	if(!TestTrue(TEXT("大洋までの排水を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
		std::make_shared<const PlanetCore::DrainageNetwork>(*PreparedDrainage);
	const std::optional<PlanetCore::PlanetErosionPreparation> Prepared =
		PlanetCore::PlanetErosionPreparation::Create(Ocean, Drainage);
	if(!TestTrue(TEXT("砂丘と河川縦断を準備"), Prepared.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetAeolianField> &Dunes = Prepared->GetAeolianField();
	const std::shared_ptr<const PlanetCore::PlanetErodedTerrain> &Base = Prepared->GetTerrain();
	if(!TestTrue(TEXT("同じ準備から砂丘と侵食基盤を参照"), Dunes && Base && Prepared->GetDrainage())) return false;
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(TEXT("中断済みの砂丘準備を拒否"),
		PlanetCore::PlanetAeolianField::Create(Base, *Prepared->GetClimate(), Settings, Cancelled.get_token())
			.has_value());

	constexpr unsigned Count = 8192;
	unsigned Changed = 0, Protected = 0, GradientChecks = 0;
	double MaximumAddition = 0, MaximumGradientError = 0;
	for(unsigned Index = 0; Index < Count; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / Count;
		const double Angle = Index * 2.39996322972865332;
		const double Horizontal = std::sqrt(std::max(0.0, 1 - Z * Z));
		const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(
			{Horizontal * std::cos(Angle), Horizontal * std::sin(Angle), Z});
		const double Before = Base->EvaluateElevation(Direction);
		const PlanetCore::TerrainSample After = Dunes->Evaluate(Direction);
		const double ValueOnly = Dunes->EvaluateElevation(Direction);
		const double Addition = After.ElevationMeters - Before;
		if(!TestTrue(TEXT("砂丘値と解析勾配は有限"),
			std::isfinite(After.ElevationMeters) && std::isfinite(After.TangentialGradientPerRadian.Length())))
			return false;
		if(!TestEqual(TEXT("勾配経路と値だけの経路が一致"), After.ElevationMeters, ValueOnly)) return false;
		if(!TestTrue(TEXT("砂丘は標高を下げず上限を守る"),
			Addition >= -1E-9 && Addition <= PlanetCore::PlanetAeolianField::MaximumAddedHeightMeters + 1E-9))
			return false;
		if(Before <= Settings.SeaLevelMeters) {
			if(!TestEqual(TEXT("水没地形の標高を保持"), Addition, 0.0)) return false;
			++Protected;
		}
		MaximumAddition = std::max(MaximumAddition, Addition);
		if(Addition < 0.05) continue;
		++Changed;
		if(GradientChecks >= 64) continue;
		const PlanetCore::Vec3 Point = Direction.Vector();
		PlanetCore::Vec3 East = Point.Cross({0, 0, 1});
		if(East.Length() < 0.1) East = Point.Cross({1, 0, 0});
		East = East * (1 / East.Length());
		const PlanetCore::Vec3 North = Point.Cross(East);
		for(const PlanetCore::Vec3 Tangent : {East, North}) {
			constexpr double Step = 2E-8;
			const PlanetCore::UnitDirection Plus =
				*PlanetCore::UnitDirection::FromVector(Point + Tangent * Step);
			const PlanetCore::UnitDirection Minus =
				*PlanetCore::UnitDirection::FromVector(Point - Tangent * Step);
			const double Numeric = (Dunes->EvaluateElevation(Plus) - Dunes->EvaluateElevation(Minus)) / (2 * Step);
			const double Analytic = After.TangentialGradientPerRadian.Dot(Tangent);
			const double Error = std::abs(Numeric - Analytic);
			MaximumGradientError = std::max(MaximumGradientError, Error);
			if(!TestTrue(TEXT("砂丘の解析接線勾配と独立差分が一致"),
				Error <= std::max(0.2, 0.0002 * std::abs(Numeric))))
				return false;
		}
		++GradientChecks;
	}
	TestTrue(TEXT("砂丘のある陸上標本を検出"), Changed >= 8);
	TestTrue(TEXT("海岸と海底の保護標本を検出"), Protected >= 100);
	TestTrue(TEXT("二方向の解析勾配を検査"), GradientChecks >= 8);
	const std::span<const PlanetCore::DrainageNode> Nodes = Prepared->GetDrainage()->Nodes();
	unsigned DownstreamChecks = 0;
	for(const PlanetCore::DrainageNode &Node : Nodes) {
		if(Node.Ocean || Node.Downstream >= Nodes.size()) continue;
		if(!TestTrue(TEXT("砂丘後も河川水位が下流へ下がる"),
			Node.WaterMeters > Nodes[Node.Downstream].WaterMeters))
			return false;
		++DownstreamChecks;
	}
	const std::array<PlanetCore::Vec3, 4> CoastSites{{
		{-0.869131894668578, 0.4941695104425493, -0.020155510878650775},
		{-0.8654848510298172, 0.48942596215981726, 0.10676235386048931},
		{0.21240510455752087, 0.17405494705921232, 0.9615554830388746},
		{0.2040227725099067, 0.6233410273244182, 0.7548646712832108}
	}};
	unsigned CoastalBandSamples = 0, CoastalChanged = 0, CoastalGradientChecks = 0;
	double CoastalMaximumAddition = 0, CoastalGradientError = 0;
	for(const PlanetCore::Vec3 CoastPoint : CoastSites) {
		const PlanetCore::UnitDirection Coast = *PlanetCore::UnitDirection::FromVector(CoastPoint);
		PlanetCore::Vec3 East = Coast.Vector().Cross({0, 0, 1});
		if(East.Length() < 0.1) East = Coast.Vector().Cross({1, 0, 0});
		East = East * (1 / East.Length());
		const PlanetCore::Vec3 North = Coast.Vector().Cross(East);
		for(const double Distance : {50.0, 100.0, 200.0, 300.0}) {
			for(unsigned Index = 0; Index < 24; ++Index) {
				const double Angle = Index * 0.2617993877991494;
				const PlanetCore::Vec3 Tangent = East * std::cos(Angle) + North * std::sin(Angle);
				const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(
					Coast.Vector() + Tangent * (Distance / Settings.RadiusMeters));
				const double Before = Base->EvaluateElevation(Direction);
				if(Before <= Settings.SeaLevelMeters + 0.1 || Before >= Settings.SeaLevelMeters + 8) continue;
				++CoastalBandSamples;
				const double Addition = Dunes->EvaluateElevation(Direction) - Before;
				if(!TestTrue(TEXT("前砂丘は海岸標高を有界に上げる"), Addition >= -1E-9 && Addition <= 4 + 1E-9)) return false;
				if(Addition > 0.02) ++CoastalChanged;
				CoastalMaximumAddition = std::max(CoastalMaximumAddition, Addition);
				if(Addition <= 0.05 || CoastalGradientChecks >= 16) continue;
				const PlanetCore::TerrainSample Sample = Dunes->Evaluate(Direction);
				const PlanetCore::Vec3 Point = Direction.Vector();
				PlanetCore::Vec3 EastAt = Point.Cross({0, 0, 1});
				if(EastAt.Length() < 0.1) EastAt = Point.Cross({1, 0, 0});
				EastAt = EastAt * (1 / EastAt.Length());
				const PlanetCore::Vec3 NorthAt = Point.Cross(EastAt);
				for(const PlanetCore::Vec3 AxisAt : {EastAt, NorthAt}) {
					constexpr double Step = 1E-7;
					const PlanetCore::UnitDirection Plus =
						*PlanetCore::UnitDirection::FromVector(Point + AxisAt * Step);
					const PlanetCore::UnitDirection Minus =
						*PlanetCore::UnitDirection::FromVector(Point - AxisAt * Step);
					const double Numeric =
						(Dunes->EvaluateElevation(Plus) - Dunes->EvaluateElevation(Minus)) / (2 * Step);
					const double Analytic = Sample.TangentialGradientPerRadian.Dot(AxisAt);
					const double Error = std::abs(Numeric - Analytic);
					CoastalGradientError = std::max(CoastalGradientError, Error);
					if(!TestTrue(TEXT("前砂丘の解析勾配と独立差分が一致"),
						Error <= std::max(0.2, 0.0002 * std::abs(Numeric)))) return false;
				}
				++CoastalGradientChecks;
			}
		}
	}
	TestTrue(TEXT("砂浜近傍の標本を取得"), CoastalBandSamples >= 8);
	TestTrue(TEXT("砂漠以外の海岸にも前砂丘を生成"), CoastalChanged >= 32);
	TestTrue(TEXT("前砂丘の解析勾配を検査"), CoastalGradientChecks >= 8);
	TestTrue(TEXT("前砂丘に可視の起伏を生成"), CoastalMaximumAddition >= 1);
	UE_LOG(LogTemp, Display, TEXT("Aeolian coastal samples %u changed %u gradients %u ")
		TEXT("max uplift %.6f m gradient residual %.6f m/rad"),
		CoastalBandSamples, CoastalChanged, CoastalGradientChecks, CoastalMaximumAddition, CoastalGradientError);
	PlanetCore::GeologySettings SmallSettings;
	SmallSettings.Seed = Settings.Seed;
	SmallSettings.RadiusMeters = 1000;
	const std::optional<PlanetCore::GeologyField> SmallGeology = PlanetCore::GeologyField::Create(SmallSettings);
	if(!TestTrue(TEXT("小半径の地質を準備"), SmallGeology.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> SmallOcean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*SmallGeology));
	if(!TestTrue(TEXT("小半径の海域を準備"), SmallOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> SmallOceanShared =
		std::make_shared<const PlanetCore::OceanConnectivityField>(*SmallOcean);
	const std::optional<PlanetCore::DrainageNetwork> SmallDrainage =
		PlanetCore::DrainageNetwork::Create(*SmallOceanShared);
	if(!TestTrue(TEXT("小半径の排水を準備"), SmallDrainage.has_value())) return false;
	const std::optional<PlanetCore::PlanetErosionPreparation> SmallPrepared =
		PlanetCore::PlanetErosionPreparation::Create(SmallOceanShared,
			std::make_shared<const PlanetCore::DrainageNetwork>(*SmallDrainage));
	if(!TestTrue(TEXT("小半径の侵食と砂丘を準備"), SmallPrepared.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetAeolianField> &SmallDunes = SmallPrepared->GetAeolianField();
	const std::shared_ptr<const PlanetCore::PlanetErodedTerrain> &SmallBase = SmallPrepared->GetTerrain();
	unsigned SmallChanged = 0, SmallProtected = 0, SmallGradientChecks = 0;
	double SmallMaximumAddition = 0, SmallMaximumAddedGrade = 0, SmallMaximumGradientError = 0;
	constexpr unsigned SmallCount = 8192;
	for(unsigned Index = 0; Index < SmallCount; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / SmallCount;
		const double Angle = Index * 2.39996322972865332;
		const double Horizontal = std::sqrt(std::max(0.0, 1 - Z * Z));
		const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(
			{Horizontal * std::cos(Angle), Horizontal * std::sin(Angle), Z});
		const double Before = SmallBase->EvaluateElevation(Direction);
		const double Addition = SmallDunes->EvaluateElevation(Direction) - Before;
		if(!TestTrue(TEXT("小半径の砂丘高が有界"), Addition >= -1E-9 && Addition <= 5.0 + 1E-9)) return false;
		if(Before <= SmallSettings.SeaLevelMeters + 1) {
			if(!TestEqual(TEXT("小半径の海面と水際を保護"), Addition, 0.0)) return false;
			++SmallProtected;
		}
		if(Addition > 0.05) {
			++SmallChanged;
			const PlanetCore::TerrainSample BeforeSample = SmallBase->Evaluate(Direction);
			const PlanetCore::TerrainSample AfterSample = SmallDunes->Evaluate(Direction);
			const PlanetCore::Vec3 AddedGradient =
				AfterSample.TangentialGradientPerRadian - BeforeSample.TangentialGradientPerRadian;
			SmallMaximumAddedGrade = std::max(SmallMaximumAddedGrade, AddedGradient.Length() / SmallSettings.RadiusMeters);
			if(SmallGradientChecks < 32) {
				const PlanetCore::Vec3 Point = Direction.Vector();
				PlanetCore::Vec3 East = Point.Cross({0, 0, 1});
				if(East.Length() < 0.1) East = Point.Cross({1, 0, 0});
				East = East * (1 / East.Length());
				const PlanetCore::Vec3 North = Point.Cross(East);
				for(const PlanetCore::Vec3 Tangent : {East, North}) {
					constexpr double Step = 2E-7;
					const PlanetCore::UnitDirection Plus =
						*PlanetCore::UnitDirection::FromVector(Point + Tangent * Step);
					const PlanetCore::UnitDirection Minus =
						*PlanetCore::UnitDirection::FromVector(Point - Tangent * Step);
					const double Numeric = (SmallDunes->EvaluateElevation(Plus) -
						SmallDunes->EvaluateElevation(Minus)) / (2 * Step);
					const double Analytic = AfterSample.TangentialGradientPerRadian.Dot(Tangent);
					const double Error = std::abs(Numeric - Analytic);
					SmallMaximumGradientError = std::max(SmallMaximumGradientError, Error);
					if(!TestTrue(TEXT("小半径の砂丘勾配が独立差分と一致"),
						Error <= std::max(0.2, 0.0002 * std::abs(Numeric)))) return false;
				}
				++SmallGradientChecks;
			}
		}
		SmallMaximumAddition = std::max(SmallMaximumAddition, Addition);
	}
	TestTrue(TEXT("小半径の乾燥低地に砂丘を生成"), SmallChanged >= 8);
	TestTrue(TEXT("小半径の海域を保護"), SmallProtected >= 100);
	TestTrue(TEXT("小半径の解析勾配を検査"), SmallGradientChecks >= 8);
	UE_LOG(LogTemp, Display, TEXT("Aeolian small planet: samples %u changed %u protected %u max uplift %.6f m ")
		TEXT("max added grade %.6f gradient residual %.6f m/rad"),
		SmallCount, SmallChanged, SmallProtected, SmallMaximumAddition, SmallMaximumAddedGrade,
		SmallMaximumGradientError);
	UE_LOG(LogTemp, Display,
		TEXT("Aeolian contracts: samples %u changed %u protected %u gradients %u downstream %u max uplift %.6f m ")
			TEXT("max gradient residual %.6f m/rad preparation %.3f ms total %.3f s"),
		Count, Changed, Protected, GradientChecks, DownstreamChecks, MaximumAddition, MaximumGradientError,
		Prepared->GetStatistics().AeolianMilliseconds, FPlatformTime::Seconds() - Start);
	// 個別の失敗を UE に残した検査結果の返戻
	return !HasAnyErrors();
}
#endif
