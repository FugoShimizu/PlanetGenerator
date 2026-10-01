#include "Misc/AutomationTest.h"
#include "Core/PlanetSpillLake.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include <cmath>
#include <memory>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillLakeContracts,
	"PlanetGenerator.SpillLake.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 面積と所属が既知の二つの窪地，及び実際の鞍点で湖岸の帰還を検査する
 * @param Parameters 未使用の実行引数
 * @return 指定側だけの単純な岸と元場の保持を確認した場合に真
 */
bool FPlanetSpillLakeContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Origin = *PlanetCore::UnitDirection::FromVector({ 0, 0, 1 });
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto DirectionAt = [&](const double X, const double Y) -> PlanetCore::UnitDirection {
			// 同じ物理寸法の接平面位置の返戻
			return *PlanetCore::UnitDirection::FromVector({ X / Radius, Y / Radius, 1 });
		};
		const auto Field = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, U = X / 40, V = Y / 20, DX = (4 * U * U * U - 4 * U) / 40;
			const double DY = 2 * V / 20;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 解析的な二重窪地と接線勾配の返戻
			return { U * U * U * U - 2 * U * U + V * V + 1, Gradient };
		};
		PlanetCore::TerrainCriticalPoint Pass { Origin };
		Pass.ElevationMeters = 1;
		Pass.MinimumCurvaturePerMeter = -4.0 / 1600;
		Pass.MaximumCurvaturePerMeter = 2.0 / 400;
		Pass.MinimumCurvatureDirection = { 1, 0, 0 };
		PlanetCore::TerrainContourSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.ChordToleranceMeters = 0.01;
		Settings.MaximumLengthMeters = 1000;
		for(const bool Positive : { false, true }) {
			const std::optional<PlanetCore::SpillLakeShore> Shore = PlanetCore::PrepareSpillLakeShore(Pass, Positive, Field, Settings);
			// 鞍点へ戻れない部分経路を受理しない返戻
			if(!TestTrue(TEXT("解析解の片側を鞍点で閉鎖"), Shore.has_value())) return false;
			const double Side = Positive ? 1.0 : -1.0;
			TestTrue(TEXT("選んだ窪地を含む"), Shore->Contains(DirectionAt(Side * 40, 0)));
			TestFalse(TEXT("同じ水位の隣の窪地を含めない"), Shore->Contains(DirectionAt(-Side * 40, 0)));
			TestFalse(TEXT("岸の外を含めない"), Shore->Contains(DirectionAt(Side * 40, 25)));
			TestFalse(TEXT("反対の半球を折返さない"), Shore->Contains(*PlanetCore::UnitDirection::FromVector({ 0, 0, -1 })));
			double TwiceArea = 0;
			for(std::size_t i = 0; i < Shore->GetPlanarVertices().size(); ++i) {
				const std::array<double, 2> &A = Shore->GetPlanarVertices()[i];
				const std::array<double, 2> &B = Shore->GetPlanarVertices()[(i + 1) % Shore->GetPlanarVertices().size()];
				TwiceArea += A[0] * B[1] - A[1] * B[0];
			}
			const double ExactArea = 4 * std::sqrt(2.0) / 3 * 40 * 20;
			TestTrue(TEXT("独立な解析面積に岸の精度で一致"), std::abs(TwiceArea * 0.5 - ExactArea) < 2);
			for(const PlanetCore::TerrainContourSample &Sample : Shore->GetContour().Samples) {
				TestTrue(TEXT("岸は元の水位に在る"), std::abs(Field(Sample.Direction).ElevationMeters - 1) <= Settings.HeightToleranceMeters);
			}
			AddInfo(
				FString::Printf(
					TEXT("Spill lake R=%.0f positive=%d points=%llu area=%.12g exact=%.12g"),
					Radius,
					static_cast<int>(Positive),
					static_cast<unsigned long long>(Shore->GetContour().Samples.size()),
					TwiceArea * 0.5,
					ExactArea
				)
			);
		}
		std::stop_source Cancel;
		Cancel.request_stop();
		TestFalse(TEXT("中断を湖岸の完成としない"), PlanetCore::PrepareSpillLakeShore(Pass, true, Field, Settings, Cancel.get_token()).has_value());
		Pass.Kind = PlanetCore::TerrainCriticalKind::Minimum;
		TestFalse(TEXT("極小を流出口として受理しない"), PlanetCore::PrepareSpillLakeShore(Pass, true, Field, Settings).has_value());
	}
	PlanetCore::GeologySettings Definition;
	Definition.RadiusMeters = 1E6;
	Definition.Seed = 42;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元地形を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の造山を準備"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 海域を含む元場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("変更しない元場を準備"), Ocean.has_value())) return false;
	const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
		// 同じ元場の返戻
		return Ocean->Evaluate(Direction);
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ 0.54180127453542282, 0.81192128360203597, -0.21733662403243018 });
	const std::optional<PlanetCore::TerrainCriticalPoint> Pass = PlanetCore::RefineTerrainCriticalPoint(Candidate, Source, {});
	// 同じ源場の鞍点が無い場合の返戻
	if(!TestTrue(TEXT("実際の流出口を再探索"), Pass.has_value())) return false;
	const std::optional<PlanetCore::SpillLakeShore> Shore = PlanetCore::PrepareSpillLakeShore(*Pass, false, Source, {});
	// 水位を引下げて閉鎖を偽装しない返戻
	if(!TestTrue(TEXT("実際の流出口と同じ水位で湖岸を閉鎖"), Shore.has_value())) return false;
	const PlanetCore::UnitDirection Lake =
	*PlanetCore::UnitDirection::FromVector({ 0.5418587314678923, 0.8119043299375797, -0.2172567010718343 });
	TestTrue(TEXT("実湖の内部を含む"), Shore->Contains(Lake));
	AddInfo(
		FString::Printf(
			TEXT("Actual spill lake points=%llu area=%.12g heightResidual=%.12g evaluations=%u"),
			static_cast<unsigned long long>(Shore->GetContour().Samples.size()),
			*PlanetCore::MeasureTerrainContourArea(Shore->GetContour(), Definition.RadiusMeters),
			Shore->GetContour().MaximumHeightResidualMeters,
			Shore->GetContour().Evaluations
		)
	);
	// 有限精度の湖岸と所属の検査結果の返戻
	return !HasAnyErrors();
}

#endif
