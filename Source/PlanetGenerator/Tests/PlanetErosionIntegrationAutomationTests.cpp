#include "Core/PlanetErosionPreparation.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetRiverTerrain.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetErosionIntegrationContracts, "PlanetGenerator.ErosionIntegration.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 本番と同じ生成入口で，侵食・河川縦断・解析法線・足場の整合を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，個別の不一致は UE に記録
 */
bool FPlanetErosionIntegrationContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	PlanetCore::ErosionPreparationStatistics Invalid;
	TestFalse(TEXT("欠落した侵食入力を拒否"), PlanetCore::PlanetErosionPreparation::Create({}, {}, {}, &Invalid).has_value());
	TestTrue(TEXT("入力不正の段階を記録"), Invalid.Failure == PlanetCore::ErosionPreparationFailure::InvalidInput);
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(TEXT("中断した準備を公開しない"),
		PlanetCore::PlanetErosionPreparation::Create({}, {}, Stop.get_token(), &Invalid).has_value());
	TestTrue(TEXT("中断を入力不正と区別"), Invalid.Failure == PlanetCore::ErosionPreparationFailure::Cancelled);
	for(const double Radius : {1000.0, 1E6, 1E9}) {
		const double Start = FPlatformTime::Seconds();
		PlanetCore::GeologySettings Settings;
		Settings.RadiusMeters = Radius;
		std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
		// 地質を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("統合前の地質を準備"), Geology.has_value())) return false;
		std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
			PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
		// 海陸を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("統合前の海陸を準備"), PreparedOcean.has_value())) return false;
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
			std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
		std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
		// 排水を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("統合前の排水を準備"), PreparedDrainage.has_value())) return false;
		const std::shared_ptr<const PlanetCore::DrainageNetwork> OriginalDrainage =
			std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
		const std::optional<PlanetCore::RiverTerrainField> Field =
			PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, OriginalDrainage);
		// 本番入口の準備が失敗した場合の返戻
		if(!TestTrue(TEXT("侵食を含む本番の地点評価場を準備"), Field.has_value())) return false;
		const std::shared_ptr<const PlanetCore::PlanetErosionPreparation> &Stage = Field->GetErosionPreparation();
		// 共有する基盤が欠落した場合の返戻
		if(!TestTrue(TEXT("気候・流量・侵食を同じ場へ保持"),
			   Stage && Stage->GetClimate() && Stage->GetRunoff() && Stage->GetTerrain() && Stage->GetAeolianField()))
			return false;
		TestTrue(TEXT("公開河道が更新済み縦断そのものを共有"), &Field->Drainage() == Stage->GetDrainage().get());
		TestTrue(TEXT("侵食前の共有排水を変更しない"), &Field->Drainage() != OriginalDrainage.get());
		TestTrue(TEXT("全準備段階が成功"), Stage->GetStatistics().Failure == PlanetCore::ErosionPreparationFailure::None);
		TestEqual(TEXT("流量と更新済み排水の生成定義が一致"), Stage->GetRunoff()->GetSourceDefinitionHash(),
			Field->Drainage().SourceDefinitionHash());
		double MaximumIndexError = 0, MaximumExtraDepth = 0, MaximumBedGap = 0, MaximumNormalAngle = 0;
		std::uint32_t Samples = 0;
		for(std::size_t i = 0; i < Field->Drainage().Reaches().size(); ++i) {
			for(const double Fraction : {0.0, 0.371, 1.0}) {
				const std::optional<PlanetCore::RiverProfileSample> Sample = Field->Drainage().SampleReach(i, Fraction);
				// 有効な河道上で評価出来なかった場合の返戻
				if(!TestTrue(TEXT("公開河道を評価"), Sample.has_value())) return false;
				const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Sample->Direction);
				const PlanetCore::TerrainSample Terrain = Field->Evaluate(Direction);
				TestTrue(TEXT("河道と合流点の標高・勾配が有限"),
					std::isfinite(Terrain.ElevationMeters) && std::isfinite(Terrain.TangentialGradientPerRadian.Length()));
				TestEqual(
					TEXT("標高専用評価が表示・法線用評価と一致"), Field->EvaluateElevation(Direction), Terrain.ElevationMeters);
				MaximumExtraDepth = std::max(MaximumExtraDepth, Sample->BedMeters - Terrain.ElevationMeters);
				MaximumBedGap = std::max(MaximumBedGap, Terrain.ElevationMeters - Sample->BedMeters);
				++Samples;
			}
			if(i % 31 != 0) continue;
			const PlanetCore::RiverTerrainChannel &Channel = Field->Channels()[i];
			const PlanetCore::Vec3 Across = PlanetCore::UnitDirection::FromVector(Channel.Start.Cross(Channel.End))->Vector();
			const PlanetCore::Vec3 Center = Field->Drainage().SampleReach(i, 0.371)->Direction;
			const double Angle = (Channel.BedHalfWidthMeters + Channel.InnerHalfWidthMeters) / (2 * Radius);
			const PlanetCore::UnitDirection Bank =
				*PlanetCore::UnitDirection::FromVector(Center * std::cos(Angle) + Across * std::sin(Angle));
			const PlanetCore::TerrainSample Terrain = Field->Evaluate(Bank), Reference = Field->EvaluateReference(Bank);
			MaximumIndexError = std::max({MaximumIndexError, std::abs(Terrain.ElevationMeters - Reference.ElevationMeters),
				(Terrain.TangentialGradientPerRadian - Reference.TangentialGradientPerRadian).Length()});
			const PlanetCore::Vec3 Point = Bank.Vector();
			const PlanetCore::Vec3 East = PlanetCore::UnitDirection::FromVector(Across - Point * Across.Dot(Point))->Vector();
			const PlanetCore::Vec3 North = Point.Cross(East);
			PlanetCore::Vec3 Numerical;
			for(const PlanetCore::Vec3 Tangent : {East, North}) {
				double Difference[2]{};
				for(std::uint32_t j = 0; j < 2; ++j) {
					const double Step = (j == 0 ? 0.02 : 0.01) / Radius;
					const PlanetCore::UnitDirection Plus =
						*PlanetCore::UnitDirection::FromVector(Point * std::cos(Step) + Tangent * std::sin(Step));
					const PlanetCore::UnitDirection Minus =
						*PlanetCore::UnitDirection::FromVector(Point * std::cos(Step) - Tangent * std::sin(Step));
					Difference[j] = (Field->EvaluateElevation(Plus) - Field->EvaluateElevation(Minus)) / (2 * Step);
				}
				Numerical = Numerical + Tangent * ((4 * Difference[1] - Difference[0]) / 3);
			}
			const PlanetCore::Vec3 ExpectedNormal =
				PlanetCore::UnitDirection::FromVector(Point * (Radius + Terrain.ElevationMeters) - Numerical)->Vector();
			const PlanetCore::Vec3 Normal = PlanetCore::UnitDirection::FromVector(
				Point * (Radius + Terrain.ElevationMeters) - Terrain.TangentialGradientPerRadian)
												->Vector();
			MaximumNormalAngle =
				std::max(MaximumNormalAngle, std::atan2(Normal.Cross(ExpectedNormal).Length(), Normal.Dot(ExpectedNormal)));
		}
		TestTrue(TEXT("統合後も河道索引が全探索と一致"), MaximumIndexError < 1E-8);
		TestTrue(TEXT("侵食と河岸の解析法線が独立差分と一致"), MaximumNormalAngle < 1E-4);
		TestTrue(TEXT("河道中心で予定河床より上に地面を残さない"), MaximumBedGap < 0.02);
		const PlanetCore::UnitDirection Anchor =
			*PlanetCore::UnitDirection::FromVector(Field->Drainage().SampleReach(Radius == 1000 ? 842 : 915, 0.5)->Direction);
		const std::optional<PlanetCore::GroundPatch> Ground = PlanetCore::BuildGroundPatch(Anchor, Radius, {},
			/**
			 * 身体の足場へ，描画と同じ侵食地形を渡す
			 * @param Direction 評価方向
			 * @return 同じ地形の標高と勾配
			 */
			[&Field](const PlanetCore::UnitDirection Direction) {
				// 共通の地点評価結果の返戻
				return Field->Evaluate(Direction);
			});
		TestTrue(TEXT("本番の侵食河岸に２ cm の足場を構築"), Ground.has_value());
		if(Ground) {
			TestTrue(TEXT("足場の誤差基準を緩和しない"), Ground->MaximumProbeErrorMeters <= 0.02);
			AddInfo(FString::Printf(TEXT("Erosion ground R=%.0f vertices=%llu triangles=%llu error=%.9g"), Radius,
				static_cast<unsigned long long>(Ground->Positions.size()),
				static_cast<unsigned long long>(Ground->Triangles.size()), Ground->MaximumProbeErrorMeters));
		}
		AddInfo(FString::Printf(
			TEXT(
				"Erosion integration R=%.0f samples=%u indexError=%.9g normalAngle=%.9g "
				"bedGap=%.9g extraDepth=%.9g preparation=%.3fms total=%.6fs"),
			Radius, Samples, MaximumIndexError, MaximumNormalAngle, MaximumBedGap, MaximumExtraDepth,
			Stage->GetStatistics().TotalMilliseconds, FPlatformTime::Seconds() - Start));
	}
	// 本番生成経路の検査結果の返戻
	return true;
}
#endif
