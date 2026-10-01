#include "Core/PlanetErodedTerrain.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetErodedTerrainContracts, "PlanetGenerator.ErodedTerrain.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 独立な地点で海陸，索引，解析勾配を照合し，連続化の体積差を計測する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，個別の不一致は UE に記録
 */
bool FPlanetErodedTerrainContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Start = FPlatformTime::Seconds();
	std::array<PlanetCore::DrainageSite, 4> Sites;
	for(std::uint32_t i = 0; i < Sites.size(); ++i) {
		Sites[i] = {{std::cos(0.01 * i), std::sin(0.01 * i), 0}, i == 0 ? -1.0 : 100.0 * i, 1E6, i == 0};
	}
	const std::array<std::array<std::uint32_t, 2>, 3> Edges{{{0, 1}, {1, 2}, {2, 3}}};
	PlanetCore::DrainageSettings AnalyticSettings;
	AnalyticSettings.RadiusMeters = 1000;
	AnalyticSettings.PreferBreaching = true;
	const std::optional<PlanetCore::DrainageNetwork> AnalyticNetwork =
		PlanetCore::DrainageNetwork::Solve(Sites, Edges, AnalyticSettings);
	// 解析用の縦断排水が失敗した場合の返戻
	if(!TestTrue(TEXT("侵食前の高い河道を準備"), AnalyticNetwork.has_value())) return false;
	/**
	 * 全ての陸上節点を低くした解析用の地形を与える
	 * @param Direction 球面上の評価方向
	 * @return 角距離に比例する低い地形
	 */
	const auto LowTerrain = [](const PlanetCore::UnitDirection Direction) {
		const PlanetCore::Vec3 Point = Direction.Vector();
		// 既知の低い標高の返戻
		return 1 + 100 * std::atan2(Point.Y, Point.X);
	};
	const std::optional<PlanetCore::DrainageNetwork> AnalyticUpdated = AnalyticNetwork->WithTerrainProfiles(LowTerrain);
	// 低下した地形の縦断準備が失敗した場合の返戻
	if(!TestTrue(TEXT("低下した始点と合流点を受理"), AnalyticUpdated.has_value())) return false;
	for(std::uint32_t i = 1; i < Sites.size(); ++i) {
		const PlanetCore::DrainageNode &Node = AnalyticUpdated->Nodes()[i];
		const double Height = LowTerrain(*PlanetCore::UnitDirection::FromVector(Node.Direction));
		TestTrue(TEXT("始点と合流点の河床が現在の地面より下"), Node.WaterMeters - Node.DepthMeters <= Height - 0.02 + 1E-10);
		TestTrue(TEXT("解析地形でも下流水位が下降"), Node.WaterMeters > AnalyticUpdated->Nodes()[Node.Downstream].WaterMeters);
	}
	TestFalse(TEXT("始点だけが非有限な地形も拒否"),
		AnalyticNetwork
			->WithTerrainProfiles(
				/**
				 * 中間標本には現れない水源の非有限標高を与える
				 * @param Direction 評価方向
				 * @return 水源近傍だけが非有限な地形
				 */
				[&LowTerrain](const PlanetCore::UnitDirection Direction) {
					const PlanetCore::Vec3 Point = Direction.Vector();
					// 始点の入力検査を必要とする地形の返戻
					return std::atan2(Point.Y, Point.X) > 0.029999 ? std::numeric_limits<double>::quiet_NaN()
																   : LowTerrain(Direction);
				})
			.has_value());
	std::uint32_t Samples = 0, Coasts = 0;
	double MaximumIndexError = 0, MaximumGradientAngle = 0, MaximumLowering = 0;
	for(const double Radius : {1000.0, 1E6, 1E9}) {
		PlanetCore::GeologySettings Definition;
		Definition.RadiusMeters = Radius;
		const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Definition);
		// 地質準備が失敗した場合の返戻
		if(!TestTrue(TEXT("侵食前の地質を準備"), Geology.has_value())) return false;
		const std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
			PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
		// 海陸準備が失敗した場合の返戻
		if(!TestTrue(TEXT("侵食前の海陸を準備"), PreparedOcean.has_value())) return false;
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
			std::make_shared<const PlanetCore::OceanConnectivityField>(*PreparedOcean);
		const std::optional<PlanetCore::PlanetClimate> Climate = PlanetCore::PlanetClimate::Create(*Ocean);
		const std::optional<PlanetCore::DrainageNetwork> Network = PlanetCore::DrainageNetwork::Create(*Ocean);
		// 気候と排水の準備が失敗した場合の返戻
		if(!TestTrue(TEXT("同じ定義の気候と排水を準備"), Climate && Network)) return false;
		const std::optional<PlanetCore::PlanetRunoff> Flow = PlanetCore::PlanetRunoff::Create(*Network, *Climate);
		// 流量準備が失敗した場合の返戻
		if(!TestTrue(TEXT("気候の流量を準備"), Flow.has_value())) return false;
		const std::optional<PlanetCore::PlanetFluvialErosion> Erosion =
			PlanetCore::PlanetFluvialErosion::Create(*Network, *Flow);
		// 離散的な侵食が失敗した場合の返戻
		if(!TestTrue(TEXT("固定網の侵食を準備"), Erosion.has_value())) return false;
		const double FieldStart = FPlatformTime::Seconds();
		const std::optional<PlanetCore::PlanetErodedTerrain> Field =
			PlanetCore::PlanetErodedTerrain::Create(Ocean, *Network, *Erosion);
		const double FieldSeconds = FPlatformTime::Seconds() - FieldStart;
		// 連続化した場の準備が失敗した場合の返戻
		if(!TestTrue(TEXT("連続な侵食場を準備"), Field.has_value())) return false;
		PlanetCore::FluvialErosionSettings Disabled;
		Disabled.Years = 0;
		const std::optional<PlanetCore::PlanetFluvialErosion> IdentityErosion =
			PlanetCore::PlanetFluvialErosion::Create(*Network, *Flow, Disabled);
		// 零時間の準備が失敗した場合の返戻
		if(!TestTrue(TEXT("零時間の離散場を準備"), IdentityErosion.has_value())) return false;
		const std::optional<PlanetCore::PlanetErodedTerrain> Identity =
			PlanetCore::PlanetErodedTerrain::Create(Ocean, *Network, *IdentityErosion);
		// 零時間の連続化が失敗した場合の返戻
		if(!TestTrue(TEXT("零時間の連続場を準備"), Identity.has_value())) return false;
		double EstimatedVolume = 0;
		constexpr std::uint32_t SampleCount = 8192;
		const double SampleArea = 4 * std::numbers::pi * Radius * Radius / SampleCount;
		const double EvaluateStart = FPlatformTime::Seconds();
		for(std::uint32_t i = 0; i < SampleCount; ++i) {
			const double Z = 1 - 2 * (i + 0.5) / SampleCount, Angle = i * 2.39996322972865332;
			const double Horizontal = std::sqrt(std::max(0.0, 1 - Z * Z));
			const PlanetCore::UnitDirection Direction =
				*PlanetCore::UnitDirection::FromVector({Horizontal * std::cos(Angle), Horizontal * std::sin(Angle), Z});
			const PlanetCore::TerrainSample Before = Ocean->Evaluate(Direction), After = Field->Evaluate(Direction);
			const double Lowering = Before.ElevationMeters - After.ElevationMeters;
			TestTrue(TEXT("全球の独立標本で有限な地形"),
				std::isfinite(After.ElevationMeters) && std::isfinite(After.TangentialGradientPerRadian.Length()));
			TestTrue(TEXT("侵食で標高を上げない"), Lowering >= 0);
			TestEqual(TEXT("侵食で海陸を反転させない"), After.ElevationMeters <= 0, Before.ElevationMeters <= 0);
			if(Before.ElevationMeters <= 0) {
				TestEqual(TEXT("海底の標高を保持"), After.ElevationMeters, Before.ElevationMeters);
				TestEqual(TEXT("海底の勾配を保持"),
					(After.TangentialGradientPerRadian - Before.TangentialGradientPerRadian).Length(), 0.0);
			}
			TestEqual(TEXT("勾配演算の省略で標高を変えない"), Field->EvaluateElevation(Direction), After.ElevationMeters);
			TestEqual(TEXT("零時間の連続化で地形を変えない"), Identity->EvaluateElevation(Direction), Before.ElevationMeters);
			MaximumLowering = std::max(MaximumLowering, Lowering);
			const double OldRadius = 1 + Before.ElevationMeters / Radius, NewRadius = 1 + After.ElevationMeters / Radius;
			EstimatedVolume +=
				SampleArea * Lowering * (OldRadius * OldRadius + OldRadius * NewRadius + NewRadius * NewRadius) / 3;
			if(i % 32 == 0) {
				const PlanetCore::TerrainSample Reference = Field->EvaluateReference(Direction);
				MaximumIndexError = std::max({MaximumIndexError, std::abs(Reference.ElevationMeters - After.ElevationMeters),
					(Reference.TangentialGradientPerRadian - After.TangentialGradientPerRadian).Length()});
				const PlanetCore::Vec3 Point = Direction.Vector();
				const PlanetCore::Vec3 Axis = std::abs(Point.Z) < 0.9 ? PlanetCore::Vec3{0, 0, 1} : PlanetCore::Vec3{1, 0, 0};
				const PlanetCore::Vec3 East = PlanetCore::UnitDirection::FromVector(Axis.Cross(Point))->Vector(),
									   North = Point.Cross(East);
				PlanetCore::Vec3 Numerical;
				for(const PlanetCore::Vec3 Tangent : {East, North}) {
					double Difference[2]{};
					for(std::uint32_t j = 0; j < 2; ++j) {
						const double Step = j == 0 ? 1E-6 : 5E-7;
						const PlanetCore::UnitDirection Plus =
							*PlanetCore::UnitDirection::FromVector(Point * std::cos(Step) + Tangent * std::sin(Step));
						const PlanetCore::UnitDirection Minus =
							*PlanetCore::UnitDirection::FromVector(Point * std::cos(Step) - Tangent * std::sin(Step));
						Difference[j] = (Field->EvaluateElevation(Plus) - Field->EvaluateElevation(Minus)) / (2 * Step);
					}
					Numerical = Numerical + Tangent * ((4 * Difference[1] - Difference[0]) / 3);
				}
				const PlanetCore::Vec3 ExpectedNormal =
					PlanetCore::UnitDirection::FromVector(Point * (Radius + After.ElevationMeters) - Numerical)->Vector();
				const PlanetCore::Vec3 Normal = PlanetCore::UnitDirection::FromVector(
					Point * (Radius + After.ElevationMeters) - After.TangentialGradientPerRadian)
													->Vector();
				MaximumGradientAngle = std::max(
					MaximumGradientAngle, std::atan2(Normal.Cross(ExpectedNormal).Length(), Normal.Dot(ExpectedNormal)));
			}
			++Samples;
		}
		const double EvaluationSeconds = FPlatformTime::Seconds() - EvaluateStart;
		for(const PlanetCore::RiverMouth &Mouth : Network->Mouths()) {
			const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Mouth.Shore.Direction);
			const PlanetCore::TerrainSample Before = Ocean->Evaluate(Direction), After = Field->Evaluate(Direction);
			TestTrue(TEXT("海岸標本の標高を保持"), std::abs(After.ElevationMeters - Before.ElevationMeters) < 1E-8);
			const double GradientDifference = (After.TangentialGradientPerRadian - Before.TangentialGradientPerRadian).Length();
			TestTrue(TEXT("海岸標本の勾配を保持"),
				GradientDifference / std::max(1.0, Before.TangentialGradientPerRadian.Length()) < 1E-8);
			++Coasts;
		}
		const std::optional<PlanetCore::DrainageNetwork> Updated = Network->WithTerrainProfiles(
			/**
			 * 再構成した侵食地形を縦断処理へ渡す
			 * @param Direction 評価方向
			 * @return 同じ不変な侵食場の高さ
			 */
			[&Field](const PlanetCore::UnitDirection Direction) {
				// 侵食後の高さの返戻
				return Field->EvaluateElevation(Direction);
			});
		// 侵食後の水位を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("侵食後の縦断形状を準備"), Updated.has_value())) return false;
		double MaximumEndpointGap = 0;
		for(std::size_t i = 0; i < Updated->Nodes().size(); ++i) {
			const PlanetCore::DrainageNode &Node = Updated->Nodes()[i];
			if(Node.Ocean) continue;
			const double CurrentHeight = Field->EvaluateElevation(*PlanetCore::UnitDirection::FromVector(Node.Direction));
			TestEqual(TEXT("始点と合流点の標高を更新"), Node.OriginalElevationMeters, CurrentHeight);
			TestTrue(TEXT("更新後も大洋へ向かう水位が下降"), Node.WaterMeters > Updated->Nodes()[Node.Downstream].WaterMeters);
			TestEqual(TEXT("侵食後も固定した下流を保持"), Node.Downstream, Network->Nodes()[i].Downstream);
			MaximumEndpointGap = std::max(MaximumEndpointGap, Node.WaterMeters - Node.DepthMeters - CurrentHeight);
		}
		AddInfo(FString::Printf(TEXT("Eroded drainage R=%.0f: endpointGap=%.9g profileKnotGap=%.9g"), Radius,
			MaximumEndpointGap, Updated->Statistics().MaximumUnresolvedKnotBedGapMeters));
		std::stop_source Stop;
		Stop.request_stop();
		TestFalse(TEXT("中断された連続場を公開しない"),
			PlanetCore::PlanetErodedTerrain::Create(Ocean, *Network, *Erosion, Stop.get_token()).has_value());
		AddInfo(FString::Printf(
			TEXT("Eroded terrain R=%.0f: preparation=%.6fs evaluation=%.6fs quadratureRemoved=%.9g discreteRemoved=%.9g"),
			Radius, FieldSeconds, EvaluationSeconds, EstimatedVolume, Erosion->GetStatistics().TotalErodedCubicMeters));
	}
	TestTrue(TEXT("空間索引が全点探索と一致"), MaximumIndexError < 1E-10);
	TestTrue(TEXT("解析勾配が独立差分と一致"), MaximumGradientAngle < 1E-5);
	TestTrue(TEXT("連続な侵食が作用"), MaximumLowering > 0);
	AddInfo(FString::Printf(
		TEXT("Eroded terrain: samples=%u coasts=%u indexError=%.9g normalAngle=%.9g maxLowering=%.9g total=%.6f"), Samples,
		Coasts, MaximumIndexError, MaximumGradientAngle, MaximumLowering, FPlatformTime::Seconds() - Start));
	// 検査の実行結果の返戻
	return true;
}
#endif
