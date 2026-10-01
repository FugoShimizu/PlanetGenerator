#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Misc/AutomationTest.h"
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetCriticalPositionContracts,
	"PlanetGenerator.CriticalPoint.Position",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 小さな勾配でも位置誤差を残す緩い谷と，実際に分離された同じ谷底を検査する
 * @param Parameters 未使用の実行引数
 * @return 解析位置と実地形の独立な始点が，同一極小の照合精度内に収束した場合に真
 */
bool FPlanetCriticalPositionContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Pole = *PlanetCore::UnitDirection::FromVector({ 0.37, -0.21, 0.903 });
	const PlanetCore::Vec3 Up = Pole.Vector();
	const PlanetCore::Vec3 East = PlanetCore::UnitDirection::FromVector(PlanetCore::Vec3{ 0, 1, 0 }.Cross(Up))->Vector();
	const PlanetCore::Vec3 North = Up.Cross(East);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.Dot(East), Y = Radius * D.Dot(North);
			const PlanetCore::Vec3 Gradient = (East * (1E-6 * X) + North * (2E-6 * Y)) * Radius;
			// 停留位置は高さの丸めと独立に分かる解析二次式の返戻
			return { 1000 + 0.5E-6 * X * X + 1E-6 * Y * Y, Gradient - D * Gradient.Dot(D) };
		};
		const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector(Up + East * (0.003 / Radius));
		PlanetCore::TerrainCriticalSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.MaximumDisplacementMeters = 200;
		const std::optional<PlanetCore::TerrainCriticalPoint> Result = PlanetCore::RefineTerrainCriticalPoint(Start, Source, Settings);
		TestTrue(TEXT("緩い解析極小を解決"), Result.has_value());
		if(!Result) continue;
		const PlanetCore::Vec3 D = Result->Direction.Vector();
		const double Error = Radius * std::atan2(D.Cross(Up).Length(), D.Dot(Up));
		AddInfo(
			FString::Printf(TEXT("Shallow critical position: radius=%.0f error=%.17g gradient=%.17g"), Radius, Error, Result->GradientNorm)
		);
		TestTrue(TEXT("解析極小との位置誤差を一万分の一メートル未満にする"), Error < 0.0001);
		TestTrue(TEXT("元の勾配精度も維持"), Result->GradientNorm <= Settings.GradientTolerance);
		Settings.PositionToleranceMeters = 0;
		TestFalse(TEXT("零の位置精度を拒否"), PlanetCore::RefineTerrainCriticalPoint(Start, Source, Settings).has_value());
		Settings.PositionToleranceMeters = 1E-5;
		TestTrue(TEXT("同じ場を正規の位置精度で解決"), PlanetCore::RefineTerrainCriticalPoint(Start, Source, Settings).has_value());
	}
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 7;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 必須の地質の欠落への返戻
	if(!TestTrue(TEXT("再現用の地質"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 必須の造山場の欠落への返戻
	if(!TestTrue(TEXT("再現用の造山場"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 必須の元場の欠落への返戻
	if(!TestTrue(TEXT("再現用の大洋場"), Ocean.has_value())) return false;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		// 補助通路が拒否された時と同じ元場の返戻
		return Ocean->Evaluate(D);
	};
	const std::array<PlanetCore::Vec3, 2> Starts {
		PlanetCore::Vec3{ -0.2696126436390735, 0.5371851558613524, 0.79921281941180389 },
		PlanetCore::Vec3{ -0.26961264337046714, 0.53718515366696606, 0.79921282097735857 }
	};
	std::array<std::optional<PlanetCore::TerrainCriticalPoint>, 2> Results;
	PlanetCore::TerrainCriticalSettings Settings;
	Settings.DifferenceStepMeters = 0.05;
	Settings.MaximumStepMeters = 0.1;
	Settings.MaximumDisplacementMeters = 0.1;
	for(unsigned I = 0; I < Starts.size(); ++I) {
		Results[I] = PlanetCore::RefineTerrainCriticalPoint(*PlanetCore::UnitDirection::FromVector(Starts[I]), Source, Settings);
		// 実際の候補を解決出来ない場合の返戻
		if(!TestTrue(TEXT("実地形の極小を再精査"), Results[I].has_value())) return false;
		AddInfo(
			FString::Printf(
				TEXT("Actual critical: index=%u height=%.17g gradient=%.17g curvature=%.17g/%.17g"),
				I,
				Results[I]->ElevationMeters,
				Results[I]->GradientNorm,
				Results[I]->MinimumCurvaturePerMeter,
				Results[I]->MaximumCurvaturePerMeter
			)
		);
		TestTrue(TEXT("両候補が極小"), Results[I]->Kind == PlanetCore::TerrainCriticalKind::Minimum);
	}
	const PlanetCore::Vec3 A = Results[0]->Direction.Vector(), B = Results[1]->Direction.Vector();
	const double Distance = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
	AddInfo(FString::Printf(TEXT("Same basin distance: meters=%.17g"), Distance));
	TestTrue(TEXT("既存の一ミリメートルの照合精度内へ収束"), Distance < 0.001);
	TestTrue(TEXT("既存の標高照合精度も維持"), std::abs(Results[0]->ElevationMeters - Results[1]->ElevationMeters) <= 1E-6);
	// 元場の解析解と実再現条件の結果の返戻
	return !HasAnyErrors();
}

#endif
