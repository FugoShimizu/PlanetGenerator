#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillLake.h"
#include "Core/PlanetSpillPassage.h"
#include "Core/PlanetTerrainContour.h"
#include "Misc/AutomationTest.h"
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillLakeSeedTopology,
	"PlanetGenerator.SpillLake.Seed90210Topology",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 負の向きで帰還した岸の実流出口を，同じ元場の両下降枝と低い迂回通路で切分ける
 * @param Parameters 未使用の実行引数
 * @return 非分離の峠を低い実測通路で確認出来た場合に真；仮説の未確認は失敗として残す
 */
bool FPlanetSpillLakeSeedTopology::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 90210;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元の失敗と同じ地質を作れない場合の返戻
	if(!TestTrue(TEXT("元の Seed の地質"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 元の造山場の欠落への返戻
	if(!TestTrue(TEXT("元の造山場"), Ridges.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> Prepared = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 元の大洋場を作れない場合の返戻
	if(!TestTrue(TEXT("元の大洋場"), Prepared.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*Prepared));
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
		++Calls;
		// 他の水系の補正を加えない同じ元場の返戻
		return Ocean->Evaluate(Direction);
	};
	const PlanetCore::UnitDirection PassDirection =
	*PlanetCore::UnitDirection::FromVector({ -0.025301198926034649, 0.11307646361785058, 0.99326409514720004 });
	PlanetCore::SpillConnectionSettings ConnectionSettings;
	ConnectionSettings.RadiusMeters = Definition.RadiusMeters;
	const std::optional<PlanetCore::SpillConnection> Connection =
	PlanetCore::PrepareSpillConnection(PassDirection, Source, ConnectionSettings);
	// 記録した実鞍点を再確認出来ない場合の返戻
	if(!TestTrue(TEXT("負の湖岸の実鞍点と両下降枝"), Connection.has_value())) return false;
	const PlanetCore::TerrainCriticalPoint &Pass = Connection->Pass;
	for(unsigned Side = 0; Side < 2; ++Side) {
		const PlanetCore::SpillConnectionBranch &Branch = Connection->Branches[Side];
		const PlanetCore::Vec3 End =
		Branch.Minimum ? Branch.Minimum->Direction.Vector() : Branch.Path.Samples.back().Direction.Vector();
		AddInfo(
			FString::Printf(
				TEXT("Lake topology branch=%u kind=%u minimum=%u H=%.17g D=(%.17g,%.17g,%.17g)"),
				Side,
				static_cast<unsigned>(Branch.Kind),
				static_cast<unsigned>(Branch.Minimum.has_value()),
				Branch.Minimum ? Branch.Minimum->ElevationMeters : Branch.Path.Samples.back().ElevationMeters,
				End.X,
				End.Y,
				End.Z
			)
		);
		PlanetCore::TerrainContourSettings ShoreSettings;
		ShoreSettings.RadiusMeters = Definition.RadiusMeters;
		ShoreSettings.MaximumLengthMeters = 1E6;
		PlanetCore::SpillLakeDiagnostics ShoreReport;
		std::optional<PlanetCore::TerrainContour> Shore =
		PlanetCore::TraceSpillLakeArc(Pass, Side != 0, Source, ShoreSettings, {}, &ShoreReport);
		double Area = 0;
		if(Shore) {
			Shore->End = PlanetCore::TerrainContourEnd::Closed;
			Area = PlanetCore::MeasureTerrainContourArea(*Shore, Definition.RadiusMeters).value_or(0);
		}
		AddInfo(
			FString::Printf(
				TEXT("Lake topology shore=%u accepted=%u failure=%u vertices=%u length=%.17g area=%.17g residual=%.17g"),
				Side,
				static_cast<unsigned>(Shore.has_value()),
				static_cast<unsigned>(ShoreReport.Failure),
				ShoreReport.Vertices,
				ShoreReport.LengthMeters,
				Area,
				ShoreReport.HeightResidualMeters
			)
		);
	}
	// 両枝の実極小が無ければ，極小間の迂回路の仮説を確認出来ない返戻
	if(!TestTrue(TEXT("同じ峠の両枝の実極小"), Connection->Branches[0].Minimum.has_value() && Connection->Branches[1].Minimum.has_value())) {
		// 実極小の欠落を別の端点で置換えない返戻
		return false;
	}
	const PlanetCore::UnitDirection From = Connection->Branches[0].Minimum->Direction;
	const PlanetCore::UnitDirection To = Connection->Branches[1].Minimum->Direction;
	bool HasLowerPassage = false;
	for(const double HalfWidth : { 256.0, 512.0, 1024.0, 2048.0 }) {
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Definition.RadiusMeters;
		Settings.SpacingMeters = 16;
		Settings.MaximumSampleStepMeters = 4;
		Settings.HalfWidthMeters = HalfWidth;
		const std::optional<PlanetCore::SpillPassage> Bypass = PlanetCore::FindSpillPassage(From, To, Source, Settings);
		AddInfo(
			FString::Printf(
				TEXT("Lake topology bypass width=%.0f accepted=%u crest=%.17g pass=%.17g excess=%.17g calls=%llu candidates=%u"),
				HalfWidth,
				static_cast<unsigned>(Bypass.has_value()),
				Bypass ? Bypass->CrestMeters : 0,
				Pass.ElevationMeters,
				Bypass ? Bypass->MaximumIndependentExcessMeters : 0,
				Calls,
				static_cast<unsigned>(Bypass ? Bypass->PassCandidates.size() : 0)
			)
		);
		if(!Bypass || !(Bypass->CrestMeters + Bypass->MaximumIndependentExcessMeters < Pass.ElevationMeters - 1E-5)) continue;
		HasLowerPassage = true;
		unsigned Checked = 0;
		for(const PlanetCore::UnitDirection Candidate : Bypass->PassCandidates) {
			if(Checked++ > 31) break;
			const std::optional<PlanetCore::SpillConnection> Lower =
			PlanetCore::PrepareSpillConnection(Candidate, Source, ConnectionSettings);
			const PlanetCore::Vec3 D = Lower ? Lower->Pass.Direction.Vector() : Candidate.Vector();
			AddInfo(
				FString::Printf(
					TEXT("Lake topology lower pass=%u H=%.17g D=(%.17g,%.17g,%.17g)"),
					static_cast<unsigned>(Lower.has_value()),
					Lower ? Lower->Pass.ElevationMeters : 0,
					D.X,
					D.Y,
					D.Z
				)
			);
			if(Lower) for(unsigned Side = 0; Side < 2; ++Side) {
				const PlanetCore::SpillConnectionBranch &Branch = Lower->Branches[Side];
				const PlanetCore::Vec3 End =
				Branch.Minimum ? Branch.Minimum->Direction.Vector() : Branch.Path.Samples.back().Direction.Vector();
				AddInfo(
					FString::Printf(
						TEXT("Lake topology lower branch=%u minimum=%u H=%.17g D=(%.17g,%.17g,%.17g)"),
						Side,
						static_cast<unsigned>(Branch.Minimum.has_value()),
						Branch.Minimum ? Branch.Minimum->ElevationMeters : Branch.Path.Samples.back().ElevationMeters,
						End.X,
						End.Y,
						End.Z
					)
				);
			}
		}
		break;
	}
	TestTrue(TEXT("両下降極小を元の峠より低い通路で結ぶ"), HasLowerPassage);
	AddInfo(FString::Printf(TEXT("Lake topology totalSourceCalls=%llu"), Calls));
	// 峠の非分離性を切分けた独立な結果の返戻
	return !HasAnyErrors();
}

#endif
