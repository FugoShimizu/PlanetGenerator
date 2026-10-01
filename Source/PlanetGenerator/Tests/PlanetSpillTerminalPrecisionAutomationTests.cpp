#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillTerminalPrecision,
	"PlanetGenerator.SpillConnection.TerminalPrecision",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実地形の極小直前で拒否された枝について，標高と解析微分を独立に記録する
 * @param Parameters 未使用の実行引数
 * @return 元場を変えずに両極小への接続が成立する場合に真
 */
bool FPlanetSpillTerminalPrecision::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
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
		// 下降終端が拒否された時と同じ元場の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ -0.27116973940830658, 0.53927144250951209, 0.79727867381671591 });
	PlanetCore::SpillConnectionDiagnostics Report;
	const std::optional<PlanetCore::SpillConnection> Connection =
	PlanetCore::PrepareSpillConnection(Candidate, Source, {}, {}, &Report);
	TestTrue(TEXT("既知の峠の下降枝を解決"), Connection.has_value());
	AddInfo(
		FString::Printf(
			TEXT("Terminal connection: accepted=%d failure=%u branch=%u end=%u move=%.17g"),
			static_cast<int>(Connection.has_value()),
			static_cast<unsigned>(Report.Failure),
			Report.Branch,
			static_cast<unsigned>(Report.TraceEnd),
			Report.TerminalMoveMeters
		)
	);
	const std::optional<PlanetCore::TerrainCriticalPoint> Pass = PlanetCore::RefineTerrainCriticalPoint(Candidate, Source, {});
	// 鞍点を確認出来ない場合の返戻
	if(!TestTrue(TEXT("再現候補の鞍点"), Pass && Pass->Kind == PlanetCore::TerrainCriticalKind::Saddle)) return false;
	const double Angle = 5 / Definition.RadiusMeters;
	const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector(
		Pass->Direction.Vector() * std::cos(Angle) + Pass->MinimumCurvatureDirection * std::sin(Angle)
	);
	PlanetCore::TerrainDescentSettings Flow;
	Flow.MaximumLengthMeters = 19995;
	Flow.MaximumVertices = 8192;
	Flow.PositionToleranceMeters = 1E-5;
	const std::optional<PlanetCore::TerrainDescentPath> Path = PlanetCore::TraceTerrainDescent(Start, Source, Flow);
	// 未完成の経路を短区間比較に使わない返戻
	if(!TestTrue(TEXT("再現枝の下降経路"), Path && !Path->Samples.empty())) return false;
	PlanetCore::TerrainCriticalSettings EndSettings;
	EndSettings.DifferenceStepMeters = 0.05;
	EndSettings.MaximumStepMeters = 0.1;
	EndSettings.MaximumDisplacementMeters = 0.1;
	const PlanetCore::TerrainDescentSample &Last = Path->Samples.back();
	const std::optional<PlanetCore::TerrainCriticalPoint> End =
	PlanetCore::RefineTerrainCriticalPoint(Last.Direction, Source, EndSettings);
	// 停留点を欠く場合の返戻
	if(!TestTrue(TEXT("再現枝の極小"), End && End->Kind == PlanetCore::TerrainCriticalKind::Minimum)) return false;
	const PlanetCore::Vec3 A = Last.Direction.Vector(), B = End->Direction.Vector(), Delta = B - A;
	const double Move = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
	AddInfo(
		FString::Printf(
			TEXT("Terminal endpoints: end=%u move=%.17g last=%.17g minimum=%.17g minimumGradient=%.17g"),
			static_cast<unsigned>(Path->End),
			Move,
			Last.ElevationMeters,
			End->ElevationMeters,
			End->GradientNorm
		)
	);
	double Previous = Last.ElevationMeters, MaximumRise = 0;
	for(unsigned I = 0; I < 17; ++I) {
		const double Fraction = I / 16.0;
		const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
		const PlanetCore::TerrainSample Value = Source(D);
		const PlanetCore::Vec3 Unit = D.Vector(), Tangent = Delta - Unit * Unit.Dot(Delta);
		const double Rise = Value.ElevationMeters - Previous;
		const double Ulp = std::nextafter(Value.ElevationMeters, std::numeric_limits<double>::infinity()) - Value.ElevationMeters;
		MaximumRise = std::max(MaximumRise, Rise);
		AddInfo(
			FString::Printf(
				TEXT("Terminal probe: fraction=%.4g height=%.17g rise=%.17g ulp=%.17g derivative=%.17g"),
				Fraction,
				Value.ElevationMeters,
				Rise,
				Ulp,
				Value.TangentialGradientPerRadian.Dot(Tangent)
			)
		);
		Previous = Value.ElevationMeters;
	}
	AddInfo(FString::Printf(TEXT("Terminal maximum rise: meters=%.17g"), MaximumRise));
	// 元の失敗を数値記録後も不合格として保持する返戻
	return !HasAnyErrors();
}

#endif
