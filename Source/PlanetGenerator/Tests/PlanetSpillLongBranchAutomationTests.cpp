#include "Core/PlanetFlowOutlet.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillConnection.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillLongBranch,
	"PlanetGenerator.ContinuousSpill.LongBranch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 拒否した同じ実峠の下降を，距離だけの打切りと完了した実終端へ分けて検査する
 * @param Parameters 未使用の実行引数
 * @return 同じ元場・位置・追跡精度で短い上限を拒否し，長い実枝の終端を確認した場合に真
 */
bool FPlanetSpillLongBranch::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 7;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元の拒否条件の欠落への返戻
	if(!TestTrue(TEXT("長い枝の元地質"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 同じ造山場の欠落への返戻
	if(!TestTrue(TEXT("長い枝の元造山場"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 別の元場で長さを検査しない返戻
	if(!TestTrue(TEXT("長い枝の元大洋場"), Ocean.has_value())) return false;
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		++Calls;
		// 失敗記録と同じ不変な標高と全解析勾配の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ -0.17907333843240206, 0.48626118345090263, 0.85526767794159153 });
	PlanetCore::SpillConnectionSettings Settings;
	Settings.MaximumBranchLengthMeters = 20000;
	PlanetCore::SpillConnectionDiagnostics ShortReport;
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetCore::SpillConnection> Short =
	PlanetCore::PrepareSpillConnection(Candidate, Source, Settings, {}, &ShortReport);
	const std::uint64_t ShortCalls = Calls;
	TestFalse(TEXT("同じ枝の旧２０㎞上限を再現"), Short.has_value());
	TestTrue(
		TEXT("元場の欠落又は誤った極小との混同を避けた距離の打切り"),
		ShortReport.Failure == PlanetCore::SpillConnectionFailure::UnresolvedTerminal &&
		ShortReport.TraceEnd == PlanetCore::TerrainDescentEnd::LengthLimit
	);
	Calls = 0;
	Settings.MaximumBranchLengthMeters = 100000;
	PlanetCore::SpillConnectionDiagnostics LongReport;
	const std::optional<PlanetCore::SpillConnection> Long =
	PlanetCore::PrepareSpillConnection(Candidate, Source, Settings, {}, &LongReport);
	AddInfo(
		FString::Printf(
			TEXT("Actual long branch: accepted=%d shortFailure=%u shortEnd=%u longFailure=%u longEnd=%u ")
			TEXT("shortCalls=%llu longCalls=%llu elapsedMs=%.9g"),
			static_cast<int>(Long.has_value()),
			static_cast<unsigned>(ShortReport.Failure),
			static_cast<unsigned>(ShortReport.TraceEnd),
			static_cast<unsigned>(LongReport.Failure),
			static_cast<unsigned>(LongReport.TraceEnd),
			ShortCalls,
			Calls,
			(FPlatformTime::Seconds() - Started) * 1000
		)
	);
	// 範囲だけを広げても未確認の枝は検査を通さない返戻
	if(!TestTrue(TEXT("同じ実峠の長い下降終端"), Long.has_value())) return false;
	TestEqual(TEXT("保持した実照会数"), Long->Evaluations, Calls);
	double MaximumLength = 0;
	for(const PlanetCore::SpillConnectionBranch &Branch : Long->Branches) {
		TestTrue(TEXT("末尾を含む頂点予算"), Branch.Path.Samples.size() <= Settings.MaximumBranchVertices);
		TestTrue(
			TEXT("海面又は元場の実極小を終端とする"),
			Branch.Kind == PlanetCore::SpillTerminalKind::SeaLevel && !Branch.Minimum ||
			Branch.Kind == PlanetCore::SpillTerminalKind::Minimum && Branch.Minimum.has_value()
		);
		double Previous = Long->Pass.ElevationMeters;
		for(const PlanetCore::TerrainDescentSample &Sample : Branch.Path.Samples) {
			TestTrue(TEXT("追跡中も同じ元場の下降を保持"), Sample.ElevationMeters <= Previous + 1E-6);
			Previous = Sample.ElevationMeters;
		}
		MaximumLength = std::max(MaximumLength, Branch.Path.Samples.back().DistanceMeters);
	}
	AddInfo(
		FString::Printf(
			TEXT("Actual long branch maximumLengthMeters=%.17g passMeters=%.17g"),
			MaximumLength,
			Long->Pass.ElevationMeters
		)
	);
	TestTrue(TEXT("旧距離を実際に越えて新しい有限距離に収まる"), MaximumLength > 20000 && MaximumLength <= 100000);
	// 通常の水系準備で拒否した河道も，同じ元場・受入水位・下降向きで確認する
	const PlanetCore::UnitDirection OutletPass =
	*PlanetCore::UnitDirection::FromVector({ -0.17863754394745004, 0.48666782568010686, 0.8551275082350116 });
	PlanetCore::FlowOutletSettings OutletSettings;
	OutletSettings.DownstreamWaterMeters = 342.83649737944614;
	OutletSettings.IsPositiveBranch = false;
	OutletSettings.MaximumDownstreamLengthMeters = 20000;
	PlanetCore::FlowOutletDiagnostics ShortOutletReport;
	const std::optional<PlanetCore::FlowOutletRoute> ShortOutlet =
	PlanetCore::PrepareFlowOutletRoute(OutletPass, Source, OutletSettings, {}, &ShortOutletReport);
	TestFalse(TEXT("通常河道の同じ２０㎞の打切り"), ShortOutlet.has_value());
	TestTrue(
		TEXT("河道の短い距離の拒否理由"),
		ShortOutletReport.Failure == PlanetCore::FlowOutletFailure::Downstream &&
		ShortOutletReport.DownstreamEnd == PlanetCore::TerrainDescentEnd::LengthLimit
	);
	OutletSettings.MaximumDownstreamLengthMeters = 100000;
	PlanetCore::FlowOutletDiagnostics LongOutletReport;
	const std::optional<PlanetCore::FlowOutletRoute> LongOutlet =
	PlanetCore::PrepareFlowOutletRoute(OutletPass, Source, OutletSettings, {}, &LongOutletReport);
	AddInfo(
		FString::Printf(
			TEXT("Actual long outlet: accepted=%d shortFailure=%u shortEnd=%u longFailure=%u longEnd=%u vertices=%u"),
			static_cast<int>(LongOutlet.has_value()),
			static_cast<unsigned>(ShortOutletReport.Failure),
			static_cast<unsigned>(ShortOutletReport.DownstreamEnd),
			static_cast<unsigned>(LongOutletReport.Failure),
			static_cast<unsigned>(LongOutletReport.DownstreamEnd),
			LongOutletReport.DownstreamVertices
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Actual long outlet upstream: H=%.17g length=%.17g move=%.17g minimum=%d criticalFailure=%u ")
			TEXT("criticalStep=%.17g initialStep=%.17g gradient=%.17g minimumH=%.17g displacement=%.17g"),
			LongOutletReport.UpstreamHeightMeters,
			LongOutletReport.UpstreamDistanceMeters,
			LongOutletReport.UpstreamMinimumMoveMeters,
			static_cast<int>(LongOutletReport.HasUpstreamMinimum),
			static_cast<unsigned>(LongOutletReport.UpstreamCritical.Failure),
			LongOutletReport.UpstreamCritical.NewtonStepMeters,
			LongOutletReport.UpstreamCritical.InitialNewtonStepMeters,
			LongOutletReport.UpstreamCritical.GradientNorm,
			LongOutletReport.UpstreamCritical.ElevationMeters,
			LongOutletReport.UpstreamCritical.DisplacementMeters
		)
	);
	TestTrue(TEXT("実際の受入水位迄を繋いだ河道"), LongOutlet.has_value());
	TestTrue(TEXT("長さだけの停止を水位到達に偽造しない"), LongOutletReport.DownstreamEnd == PlanetCore::TerrainDescentEnd::HeightLimit);
	// 候補の都合の良い差替をしない回帰検査の結果の返戻
	return !HasAnyErrors();
}

#endif
