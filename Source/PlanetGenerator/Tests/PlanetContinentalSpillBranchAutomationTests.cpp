#include "Core/PlanetFlowOutlet.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillLake.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetContinentalSpillBranch, "PlanetGenerator.ContinuousSpill.ContinentalBranch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 広域探索で確認した同じ鞍点の１００ｋｍ打切りと，元場の実下降終端を切分ける
 * @param Parameters 未使用の実行引数
 * @return 元の精度・頂点上限で長い枝の下降と終端を確認し，旧打切りも再現した場合に真
 */
bool FPlanetContinentalSpillBranch::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 0;
	Definition.RadiusMeters = 1E6;
	const std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 再現条件の地質欠落への返戻
	if(!TestTrue(TEXT("同じ Seed０の地質"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology = std::make_shared<const PlanetCore::GeologyField>(*Base);
	const std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山の欠落への返戻
	if(!TestTrue(TEXT("同じ Seed０の造山"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(*Ridges),
		{},
		PlanetCore::PredrainageReliefMode::MesoOnly,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 尺度比較の元場の欠落への返戻
	if(!TestTrue(TEXT("拒否を記録した同じ元場"), Ocean.has_value())) return false;
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		constexpr std::uint64_t Limit = 1000000;
		if(Calls >= Limit) {
			const double Invalid = std::numeric_limits<double>::quiet_NaN();
			// 試験の評価予算を超えた場を受理しない返戻
			return { Invalid, { Invalid, Invalid, Invalid } };
		}
		++Calls;
		// 原失敗と同じ元標高と解析勾配の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ 0.24050518948779484, 0.058463262659215559, 0.96888559734814905 });
	PlanetCore::SpillConnectionSettings ShortSettings;
	ShortSettings.RadiusMeters = Definition.RadiusMeters;
	ShortSettings.MaximumBranchLengthMeters = 100000;
	PlanetCore::SpillConnectionDiagnostics ShortReport;
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetCore::SpillConnection> Short =
	PlanetCore::PrepareSpillConnection(Candidate, Source, ShortSettings, {}, &ShortReport);
	const std::uint64_t ShortCalls = Calls;
	TestFalse(TEXT("同じ１００ｋｍの距離打切りを再現"), Short.has_value());
	TestTrue(
		TEXT("元場や極小の失敗と距離を区別"),
		ShortReport.Failure == PlanetCore::SpillConnectionFailure::UnresolvedTerminal &&
		ShortReport.TraceEnd == PlanetCore::TerrainDescentEnd::LengthLimit && ShortReport.Branch == 1
	);
	PlanetCore::SpillConnectionSettings LongSettings;
	LongSettings.RadiusMeters = Definition.RadiusMeters;
	PlanetCore::SpillConnectionDiagnostics LongReport;
	Calls = 0;
	const std::optional<PlanetCore::SpillConnection> Long =
	PlanetCore::PrepareSpillConnection(Candidate, Source, LongSettings, {}, &LongReport);
	const std::uint64_t LongCalls = Calls;
	AddInfo(
		FString::Printf(
			TEXT("Continental branch accepted=%d oldFailure=%u oldEnd=%u oldSide=%u newFailure=%u newEnd=%u ")
			TEXT("oldSource=%llu newSource=%llu milliseconds=%.9g"),
			static_cast<int>(Long.has_value()),
			static_cast<unsigned>(ShortReport.Failure),
			static_cast<unsigned>(ShortReport.TraceEnd),
			ShortReport.Branch,
			static_cast<unsigned>(LongReport.Failure),
			static_cast<unsigned>(LongReport.TraceEnd),
			static_cast<unsigned long long>(ShortCalls),
			static_cast<unsigned long long>(LongCalls),
			(FPlatformTime::Seconds() - Started) * 1000
		)
	);
	// 距離の拡大だけで未確認の枝を成功にしない返戻
	if(!TestTrue(TEXT("同じ実鞍点の両側の実終端"), Long.has_value())) return false;
	TestEqual(TEXT("長い追跡の全元場照会を集計"), Long->Evaluations, LongCalls);
	TestEqual(TEXT("元地形と実追跡の物理半径は同一"), Ocean->Base().Settings().RadiusMeters, LongSettings.RadiusMeters);
	AddInfo(
		FString::Printf(
			TEXT("Continental actual source radius=%.17g traceRadius=%.17g passH=%.17g"),
			Ocean->Base().Settings().RadiusMeters,
			LongSettings.RadiusMeters,
			Long->Pass.ElevationMeters
		)
	);
	double MaximumLength = 0;
	for(unsigned Side = 0; Side < Long->Branches.size(); ++Side) {
		const PlanetCore::SpillConnectionBranch &Branch = Long->Branches[Side];
		const double Length = Branch.Path.Samples.back().DistanceMeters;
		MaximumLength = std::max(MaximumLength, Length);
		TestTrue(
			TEXT("元の頂点と物理長の上限を保持"),
			Branch.Path.Samples.size() <= LongSettings.MaximumBranchVertices && Length <= LongSettings.MaximumBranchLengthMeters
		);
		TestTrue(
			TEXT("海面高度又は精査済の元場の極小に到達"),
			Branch.Kind == PlanetCore::SpillTerminalKind::SeaLevel && !Branch.Minimum ||
			Branch.Kind == PlanetCore::SpillTerminalKind::Minimum && Branch.Minimum &&
			Branch.Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum
		);
		double Previous = Long->Pass.ElevationMeters;
		for(const PlanetCore::TerrainDescentSample &Point : Branch.Path.Samples) {
			TestTrue(TEXT("全保持点は同じ元標高"), std::abs(Source(Point.Direction).ElevationMeters - Point.ElevationMeters) <= 1E-6);
			TestTrue(TEXT("長い追跡でも実標高は下降"), Point.ElevationMeters <= Previous + 1E-6);
			Previous = Point.ElevationMeters;
		}
		AddInfo(
			FString::Printf(
				TEXT("Continental side=%u length=%.17g samples=%llu end=%u"),
				Side,
				Length,
				static_cast<unsigned long long>(Branch.Path.Samples.size()),
				static_cast<unsigned>(Branch.Path.End)
			)
		);
	}
	TestTrue(TEXT("旧範囲より長い正規の枝を確認"), MaximumLength > ShortSettings.MaximumBranchLengthMeters);
	// 同じ広域水系で拒否された河道側も，実際の下流受入水位迄を追跡する
	const PlanetCore::UnitDirection OutletPass =
	*PlanetCore::UnitDirection::FromVector({ 0.30961880997458341, 0.039201462702869318, 0.95005233425947566 });
	PlanetCore::FlowOutletSettings ShortOutletSettings;
	ShortOutletSettings.RadiusMeters = Definition.RadiusMeters;
	ShortOutletSettings.MaximumDownstreamLengthMeters = 100000;
	ShortOutletSettings.DownstreamWaterMeters = 130.50565537313005;
	// 記録した方向を再構築しただけでは，元の固有ベクトルの符号を復元出来ない
	// 同じ元場の両終端を確認し，実際に受入水位より低い物理枝の基準軸を保持する
	Calls = 0;
	const std::optional<PlanetCore::SpillConnection> OutletConnection =
	PlanetCore::PrepareSpillConnection(OutletPass, Source, LongSettings, {}, &LongReport);
	AddInfo(
		FString::Printf(
			TEXT("Continental outlet connection accepted=%d failure=%u side=%u end=%u length=%.17g H=%.17g move=%.17g ")
			TEXT("criticalFailure=%u criticalH=%.17g heightDelta=%.17g criticalStep=%.17g initialStep=%.17g curvature=%.17g source=%llu"),
			static_cast<int>(OutletConnection.has_value()),
			static_cast<unsigned>(LongReport.Failure),
			LongReport.Branch,
			static_cast<unsigned>(LongReport.TraceEnd),
			LongReport.TraceLengthMeters,
			LongReport.TerminalHeightMeters,
			LongReport.TerminalMoveMeters,
			static_cast<unsigned>(LongReport.TerminalCritical.Failure),
			LongReport.TerminalCritical.ElevationMeters,
			LongReport.TerminalCritical.ElevationMeters - LongReport.TerminalHeightMeters,
			LongReport.TerminalCritical.NewtonStepMeters,
			LongReport.TerminalCritical.InitialNewtonStepMeters,
			LongReport.TerminalCritical.MinimumCurvaturePerMeter,
			static_cast<unsigned long long>(Calls)
		)
	);
	// 独立な湖岸検査を省かず，河道側の失敗も最終結果へ保持する
	if(TestTrue(TEXT("河道の同じ実鞍点と両終端"), OutletConnection.has_value())) {
		const unsigned DownstreamSide = OutletConnection->Branches[0].Path.Samples.back().ElevationMeters <
		OutletConnection->Branches[1].Path.Samples.back().ElevationMeters ? 0U : 1U;
		ShortOutletSettings.IsPositiveBranch = DownstreamSide == 1;
		ShortOutletSettings.ReferenceMinimumDirection = OutletConnection->Pass.MinimumCurvatureDirection;
		TestTrue(
			TEXT("実終端が同じ受入水位より低い"),
			OutletConnection->Branches[DownstreamSide].Path.Samples.back().ElevationMeters < ShortOutletSettings.DownstreamWaterMeters
		);
		AddInfo(
			FString::Printf(
				TEXT("Continental outlet physical side=%u terminalHeights=(%.17g,%.17g) axis=(%.17g,%.17g,%.17g)"),
				DownstreamSide,
				OutletConnection->Branches[0].Path.Samples.back().ElevationMeters,
				OutletConnection->Branches[1].Path.Samples.back().ElevationMeters,
				OutletConnection->Pass.MinimumCurvatureDirection.X,
				OutletConnection->Pass.MinimumCurvatureDirection.Y,
				OutletConnection->Pass.MinimumCurvatureDirection.Z
			)
		);
		PlanetCore::FlowOutletDiagnostics ShortOutletReport;
		Calls = 0;
		const std::optional<PlanetCore::FlowOutletRoute> ShortOutlet =
		PlanetCore::PrepareFlowOutletRoute(OutletPass, Source, ShortOutletSettings, {}, &ShortOutletReport);
		const std::uint64_t ShortOutletCalls = Calls;
		TestFalse(TEXT("同じ河道側の１００ｋｍ打切り"), ShortOutlet.has_value());
		TestTrue(
			TEXT("下流の距離制限を元場の拒否と区別"),
			ShortOutletReport.Failure == PlanetCore::FlowOutletFailure::Downstream &&
			ShortOutletReport.DownstreamEnd == PlanetCore::TerrainDescentEnd::LengthLimit
		);
		PlanetCore::FlowOutletSettings LongOutletSettings = ShortOutletSettings;
		LongOutletSettings.MaximumDownstreamLengthMeters = PlanetCore::FlowOutletSettings{}.MaximumDownstreamLengthMeters;
		PlanetCore::FlowOutletDiagnostics LongOutletReport;
		Calls = 0;
		const std::optional<PlanetCore::FlowOutletRoute> LongOutlet =
		PlanetCore::PrepareFlowOutletRoute(OutletPass, Source, LongOutletSettings, {}, &LongOutletReport);
		const std::uint64_t LongOutletCalls = Calls;
		AddInfo(
			FString::Printf(
				TEXT("Continental outlet accepted=%d oldFailure=%u oldEnd=%u newFailure=%u newEnd=%u oldSource=%llu newSource=%llu"),
				static_cast<int>(LongOutlet.has_value()),
				static_cast<unsigned>(ShortOutletReport.Failure),
				static_cast<unsigned>(ShortOutletReport.DownstreamEnd),
				static_cast<unsigned>(LongOutletReport.Failure),
				static_cast<unsigned>(LongOutletReport.DownstreamEnd),
				static_cast<unsigned long long>(ShortOutletCalls),
				static_cast<unsigned long long>(LongOutletCalls)
			)
		);
		TestTrue(TEXT("同じ受入水位へ実際に到達した長い河道"), LongOutlet.has_value());
		if(LongOutlet) {
			TestTrue(TEXT("下流水位に実到達"), LongOutletReport.DownstreamEnd == PlanetCore::TerrainDescentEnd::HeightLimit);
			TestTrue(TEXT("有限な頂点予算を維持"), LongOutletReport.DownstreamVertices < 8193);
			TestEqual(TEXT("河道の全元場照会を集計"), static_cast<std::uint64_t>(LongOutlet->Evaluations), LongOutletCalls);
			const double Length = LongOutlet->Route.GetLengthMeters();
			TestTrue(TEXT("短い既定距離を超える正規の河道"), Length > 100000 && Length <= 300000 + 102);
			const std::optional<PlanetCore::TerrainFlowCurveSample> End = LongOutlet->Route.Sample(Length);
			TestTrue(TEXT("曲線末尾も元場の受入水位以下"), End && Source(End->Direction).ElevationMeters <= LongOutletSettings.DownstreamWaterMeters);
			AddInfo(
				FString::Printf(TEXT("Continental outlet lengthMeters=%.17g vertices=%u"), Length, LongOutletReport.DownstreamVertices)
			);
		}
	}
	// 同じ実峠の両等高枝を検査し，１０００ｋｍの打切りを帰還とは扱わない
	unsigned LongShoreCases = 0;
	for(unsigned Side = 0; Side < 2; ++Side) {
		PlanetCore::TerrainContourSettings ShortShoreSettings;
		ShortShoreSettings.RadiusMeters = Definition.RadiusMeters;
		ShortShoreSettings.MaximumVertices = 65536;
		ShortShoreSettings.MaximumLengthMeters = 1000000;
		PlanetCore::SpillLakeDiagnostics ShortShoreReport;
		Calls = 0;
		const std::optional<PlanetCore::SpillLakeShore> ShortShore =
		PlanetCore::PrepareSpillLakeShore(Long->Pass, Side == 1, Source, ShortShoreSettings, {}, &ShortShoreReport);
		const std::uint64_t ShortShoreCalls = Calls;
		AddInfo(
			FString::Printf(
				TEXT("Continental shore old side=%u accepted=%d failure=%u end=%u length=%.17g vertices=%u source=%llu"),
				Side,
				static_cast<int>(ShortShore.has_value()),
				static_cast<unsigned>(ShortShoreReport.Failure),
				static_cast<unsigned>(ShortShoreReport.End),
				ShortShoreReport.LengthMeters,
				ShortShoreReport.Vertices,
				static_cast<unsigned long long>(ShortShoreCalls)
			)
		);
		if(ShortShoreReport.End != PlanetCore::TerrainContourEnd::LengthLimit) continue;
		++LongShoreCases;
		TestFalse(TEXT("打切りの等高枝は湖岸として受理しない"), ShortShore.has_value());
		PlanetCore::TerrainContourSettings LongShoreSettings = ShortShoreSettings;
		LongShoreSettings.MaximumLengthMeters = 3000000;
		PlanetCore::SpillLakeDiagnostics LongShoreReport;
		Calls = 0;
		const std::optional<PlanetCore::SpillLakeShore> LongShore =
		PlanetCore::PrepareSpillLakeShore(Long->Pass, Side == 1, Source, LongShoreSettings, {}, &LongShoreReport);
		AddInfo(
			FString::Printf(
				TEXT("Continental shore new side=%u accepted=%d failure=%u end=%u length=%.17g vertices=%u source=%llu"),
				Side,
				static_cast<int>(LongShore.has_value()),
				static_cast<unsigned>(LongShoreReport.Failure),
				static_cast<unsigned>(LongShoreReport.End),
				LongShoreReport.LengthMeters,
				LongShoreReport.Vertices,
				static_cast<unsigned long long>(Calls)
			)
		);
		TestTrue(TEXT("同じ実等高枝の帰還を確認"), LongShore.has_value());
		if(!LongShore) continue;
		TestTrue(
			TEXT("帰還と正の領域面積を確認"),
			LongShoreReport.End == PlanetCore::TerrainContourEnd::Terminal && LongShoreReport.AreaMetersSquared > 0
		);
		TestTrue(
			TEXT("長い岸も元の頂点と精度を維持"),
			LongShoreReport.Vertices < 65537 && LongShoreReport.HeightResidualMeters <= 1E-6 && LongShoreReport.LengthMeters > 1000000 &&
			LongShoreReport.LengthMeters <= 3000000.0
		);
		TestTrue(TEXT("実際に囲んだ側の極小を保持"), Long->Branches[Side].Minimum && LongShore->Contains(Long->Branches[Side].Minimum->Direction));
	}
	TestTrue(TEXT("旧１０００ｋｍ条件の実打切りを再現"), LongShoreCases > 0);
	std::stop_source Stop;
	Stop.request_stop();
	Calls = 0;
	TestFalse(
		TEXT("長い上限でも中断後に準備しない"),
		PlanetCore::PrepareSpillConnection(Candidate, Source, LongSettings, Stop.get_token(), &LongReport).has_value()
	);
	TestEqual(TEXT("中断後の元場照会は零"), Calls, UINT64_C(0));
	// 原失敗と実下降終端の回帰結果の返戻
	return !HasAnyErrors();
}

#endif
