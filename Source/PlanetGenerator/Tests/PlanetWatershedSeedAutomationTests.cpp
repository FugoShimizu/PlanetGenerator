#include "Core/PlanetFlowWatershed.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_COMPLEX_AUTOMATION_TEST(
	FPlanetWatershedSeedCoverage,
	"PlanetGenerator.ContinuousSpill.WatershedSeedCoverage",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 三条件と陸上細部を除く尺度比較を，独立した検査項目へ展開する
 * @param Names 表示名
 * @param Commands 各実行に渡す Seed
 */
void FPlanetWatershedSeedCoverage::GetTests(TArray<FString> &Names, TArray<FString> &Commands) const {
	for(const FString &Seed : { FString(TEXT("0")), FString(TEXT("7")), FString(TEXT("90210")) }) {
		Names.Add(TEXT("Seed") + Seed);
		Commands.Add(Seed);
		Names.Add(TEXT("MacroSeed") + Seed);
		Commands.Add(TEXT("Macro") + Seed);
	}
	// 登録の終了
	return;
}

/**
 * 既知の峠・極小を指定せず，異なる生成定義の最初の陸上標本から水系を準備する
 * @param Parameters 登録済の生成 Seed
 * @return 指定条件で準備出来た場合に真；未対応の地点も失敗として記録
 */
bool FPlanetWatershedSeedCoverage::RunTest(const FString &Parameters) {
	const bool MacroOnly = Parameters.StartsWith(TEXT("Macro"));
	const FString SeedParameter = MacroOnly ? Parameters.RightChop(5) : Parameters;
	const bool RegisteredSeed = SeedParameter == TEXT("0") || SeedParameter == TEXT("7") || SeedParameter == TEXT("90210");
	// 登録した尺度比較と三つの Seed 以外を別条件で実行しない
	// 無効な検査指定への返戻
	if(!TestTrue(TEXT("登録済の尺度と Seed"), RegisteredSeed)) return false;
	const std::uint64_t Seed = SeedParameter == TEXT("0") ? 0 : SeedParameter == TEXT("7") ? 7 : 90210;
	// 通常の Seed と独立な球面候補の走査
	PlanetCore::GeologySettings Definition;
	Definition.Seed = Seed;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 必須の元場の欠落への返戻
	if(!TestTrue(TEXT("別 Seed の地質"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 必須の造山場の欠落への返戻
	if(!TestTrue(TEXT("別 Seed の造山場"), Ridges.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> Prepared = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		MacroOnly ? PlanetCore::PredrainageReliefMode::MesoOnly : PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 大洋を確認出来ない条件の返戻
	if(!TestTrue(TEXT("別 Seed の大洋"), Prepared.has_value())) return false;
	AddInfo(FString::Printf(TEXT("Watershed scale comparison: macroOnly=%d; landscape acceptance is not tested"), MacroOnly));
	AddInfo(FString::Printf(TEXT("Watershed implementation version=%u"), PlanetCore::GetSpillNetworkVersion()));
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*Prepared));
	std::optional<PlanetCore::UnitDirection> Location;
	unsigned Candidate = 0;
	for(; Candidate < 256; ++Candidate) {
		const double Z = 1 - 2 * (Candidate + 0.5) / 256, R = std::sqrt(1 - Z * Z), Angle = Candidate * 2.39996322972865332; // 黄金角
		const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector({ R * std::cos(Angle), R * std::sin(Angle), Z });
		if(Ocean->EvaluateElevation(D) > 20) {
			Location = D;
			break;
		}
	}
	// 都合の良い峠を選び直さず，陸上の欠落を失敗する返戻
	if(!TestTrue(TEXT("等面積方向列の陸上地点"), Location.has_value())) return false;
	const double Started = FPlatformTime::Seconds();
	PlanetCore::FlowWatershedDiagnostics Report;
	const std::optional<PlanetCore::PreparedFlowWatershed> Watershed =
	PlanetCore::PrepareFlowWatershed(*Location, Ocean, {}, &Report);
	const PlanetCore::Vec3 D = Location->Vector();
	const std::array<const PlanetCore::SpillDiscoveryDiagnostics *, 2> DiscoveryStages {
		&Report.Seed.Discovery,
		&Report.Network.Discovery
	};
	for(unsigned Stage = 0; Stage < DiscoveryStages.size(); ++Stage) {
		const PlanetCore::SpillDiscoveryDiagnostics &StageReport = *DiscoveryStages[Stage];
		const std::array<std::uint32_t, 10> &Failures = StageReport.CriticalFailures;
		const std::array<std::uint32_t, 3> &Kinds = StageReport.CriticalKinds;
		AddInfo(
			FString::Printf(
				TEXT("Discovery critical range stage=%u maxStep=%.17g maxSearchRadius=%.17g"),
				Stage,
				StageReport.MaximumCriticalStepMeters,
				StageReport.MaximumCriticalSearchRadiusMeters
			)
		);
		AddInfo(
			FString::Printf(
				TEXT("Discovery critical stage=%u failure=%u evaluations=%llu windows=%u width=%.17g ")
				TEXT("reasons=(%u,%u,%u,%u,%u,%u,%u,%u,%u,%u) kinds=(%u,%u,%u)"),
				Stage,
				static_cast<unsigned>(StageReport.Failure),
				StageReport.Evaluations,
				StageReport.Windows,
				StageReport.LastHalfWidthMeters,
				Failures[0],
				Failures[1],
				Failures[2],
				Failures[3],
				Failures[4],
				Failures[5],
				Failures[6],
				Failures[7],
				Failures[8],
				Failures[9],
				Kinds[0],
				Kinds[1],
				Kinds[2]
			)
		);
		const PlanetCore::TerrainCriticalDiagnostics &Failed = StageReport.FirstFailedCritical;
		const PlanetCore::Vec3 CandidateDirection =
		StageReport.FirstFailedCriticalCandidate ? StageReport.FirstFailedCriticalCandidate->Vector() : PlanetCore::Vec3{};
		AddInfo(
			FString::Printf(
				TEXT("Discovery first critical stage=%u candidate=(%.17g,%.17g,%.17g) failure=%u H=%.17g ")
				TEXT("gradient=%.17g step=%.17g curvature=(%.17g,%.17g) difference=%.17g displacement=%.17g iterations=%u"),
				Stage,
				CandidateDirection.X,
				CandidateDirection.Y,
				CandidateDirection.Z,
				static_cast<unsigned>(Failed.Failure),
				Failed.ElevationMeters,
				Failed.GradientNorm,
				Failed.NewtonStepMeters,
				Failed.MinimumCurvaturePerMeter,
				Failed.MaximumCurvaturePerMeter,
				Failed.CurvatureDifferencePerMeter,
				Failed.DisplacementMeters,
				Failed.Iterations
			)
		);
	}
	AddInfo(
		FString::Printf(
			TEXT("Seed continuation: attempts=%u initialFailure=%u initialStep=%.17g"),
			Report.Seed.Continuations,
			static_cast<unsigned>(Report.Seed.InitialCritical.Failure),
			Report.Seed.InitialCritical.InitialNewtonStepMeters
		)
	);
	const PlanetCore::TerrainCriticalDiagnostics &Critical = Report.Seed.Critical;
	const PlanetCore::Vec3 EndDirection =
	Report.Seed.DescentEndpoint ? Report.Seed.DescentEndpoint->Direction.Vector() : PlanetCore::Vec3{};
	AddInfo(
		FString::Printf(
			TEXT("Seed descent endpoint D=(%.17g,%.17g,%.17g) H=%.17g length=%.17g"),
			EndDirection.X,
			EndDirection.Y,
			EndDirection.Z,
			Report.Seed.DescentEndpoint ? Report.Seed.DescentEndpoint->ElevationMeters : 0,
			Report.Seed.DescentEndpoint ? Report.Seed.DescentEndpoint->DistanceMeters : 0
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Seed critical: failure=%u D=(%.17g,%.17g,%.17g) H=%.17g gradient=%.17g initialStep=%.17g ")
			TEXT("step=%.17g curvature=(%.17g,%.17g) difference=%.17g displacement=%.17g evaluations=%u iterations=%u kind=%d"),
			static_cast<unsigned>(Critical.Failure),
			Critical.Direction.X,
			Critical.Direction.Y,
			Critical.Direction.Z,
			Critical.ElevationMeters,
			Critical.GradientNorm,
			Critical.InitialNewtonStepMeters,
			Critical.NewtonStepMeters,
			Critical.MinimumCurvaturePerMeter,
			Critical.MaximumCurvaturePerMeter,
			Critical.CurvatureDifferencePerMeter,
			Critical.DisplacementMeters,
			Critical.Evaluations,
			Critical.Iterations,
			Critical.AcceptedKind ? static_cast<int>(*Critical.AcceptedKind) : -1
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Watershed seed coverage: seed=%llu candidate=%u D=(%.17g,%.17g,%.17g) height=%.12g ms=%.9g ")
			TEXT("failure=%u seedFailure=%u end=%u seedEvaluations=%llu networkFailure=%u networkEvaluations=%llu ")
			TEXT("discovery=%u windows=%u width=%.9g channelsFailure=%u channelFailure=%u channels=%u"),
			Seed,
			Candidate,
			D.X,
			D.Y,
			D.Z,
			Ocean->EvaluateElevation(*Location),
			(FPlatformTime::Seconds() - Started) * 1000,
			static_cast<unsigned>(Report.Failure),
			static_cast<unsigned>(Report.Seed.Failure),
			static_cast<unsigned>(Report.Seed.DescentEnd),
			Report.Seed.Evaluations,
			static_cast<unsigned>(Report.Network.Failure),
			Report.Network.Evaluations,
			static_cast<unsigned>(Report.Network.Discovery.Failure),
			Report.Network.Discovery.Windows,
			Report.Network.Discovery.LastHalfWidthMeters,
			static_cast<unsigned>(Report.Channels.Failure),
			static_cast<unsigned>(Report.Channels.Channel.Failure),
			Report.Channels.PreparedChannels
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Watershed discovery detail: seed=%llu steps=%u repeated=%u internal=%u reused=%u unrelated=%u unresolved=%u ")
			TEXT("bridges=%u rejectedBridges=%u relaxed=%u relaxedBridges=%u relaxationFailures=%u"),
			Seed,
			Report.Network.Steps,
			Report.Network.Discovery.RepeatedConnections,
			Report.Network.Discovery.NewInternalConnections,
			Report.Network.Discovery.ReusedConnections,
			Report.Network.Discovery.UnrelatedConnections,
			Report.Network.Discovery.UnresolvedPasses,
			Report.Network.Discovery.BridgeCorridors,
			Report.Network.Discovery.RejectedBridges,
			Report.Network.Discovery.RelaxedWindows,
			Report.Network.Discovery.RelaxedBridges,
			Report.Network.Discovery.RelaxationFailures
		)
	);
	AddInfo(
		FString::Printf(TEXT("Bypass exact samples: hits=%llu retained=%u"), Report.Bypass.CacheHits, Report.Bypass.CachedSamples)
	);
	AddInfo(
		FString::Printf(
			TEXT("Preferred continuous: attempts=%u accepted=%u evaluations=%llu gridFallbacks=%u lastFailure=%u"),
			Report.PreferredContinuousAttempts,
			Report.AcceptedPreferredContinuous,
			Report.PreferredContinuousEvaluations,
			Report.PreferredGridFallbacks,
			static_cast<unsigned>(Report.Bypass.FirstPreferredFailure)
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Bypass accumulated calls=%llu hits=%llu preparedBranches=%llu"),
			Report.BypassEvaluations,
			Report.BypassCacheHits,
			Report.BypassReusedCandidates
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Bypass connection relaxation: attempts=%llu evaluations=%llu iterations=%llu checks=%llu accepted=%llu"),
			Report.BypassConnectionRelaxations,
			Report.BypassConnectionRelaxationEvaluations,
			Report.BypassConnectionIterations,
			Report.BypassConnectionCheckpointChecks,
			Report.BypassAcceptedConnectionCheckpoints
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Continuous connection fallback attempts=%u accepted=%u old=%.17g escape=(%.17g,%.17g) pass=(%.17g,%.17g,%.17g)"),
			Report.ContinuousConnectionFallbacks,
			Report.AcceptedContinuousConnectionFallbacks,
			Report.FirstContinuousFallbackPassMeters,
			Report.FirstContinuousFallbackEscapeMeters[0],
			Report.FirstContinuousFallbackEscapeMeters[1],
			Report.FirstContinuousFallbackPass.X,
			Report.FirstContinuousFallbackPass.Y,
			Report.FirstContinuousFallbackPass.Z
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Watershed stage ms channels=%.9g bypass=%.9g reused=%llu rebuilt=%llu"),
			Report.ChannelPreparationMilliseconds,
			Report.BypassPreparationMilliseconds,
			Report.ReusedChannels,
			Report.RebuiltChannels
		)
	);
	const PlanetCore::SpillDiscoveryDiagnostics &Discovery = Report.Network.Discovery;
	AddInfo(
		FString::Printf(
			TEXT("Watershed bypass: completions=%u failure=%u evaluations=%llu windows=%u added=%u width=%.17g ")
			TEXT("oldPass=%.17g measured=%.17g graphFailure=%u rejected=%u"),
			Report.TopologyCompletions,
			static_cast<unsigned>(Report.Bypass.Failure),
			Report.Bypass.Evaluations,
			Report.Bypass.Windows,
			Report.Bypass.AddedConnections,
			Report.Bypass.LastHalfWidthMeters,
			Report.Bypass.OriginalPassMeters,
			Report.Bypass.MeasuredBypassMeters,
			static_cast<unsigned>(Report.Bypass.Graph.Failure),
			Report.Bypass.RejectedCandidates
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Watershed continuous bypass: attempts=%u evaluations=%llu lower=%u crest=%.17g"),
			Report.Bypass.RelaxationAttempts,
			Report.Bypass.RelaxationEvaluations,
			static_cast<unsigned>(Report.Bypass.HasLowerRelaxedPath),
			Report.Bypass.RelaxedCrestMeters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Watershed water profile: failure=%u knots=%u passes=%u evaluations=%u Herror=%.17g Gerror=%.17g"),
			static_cast<unsigned>(Report.Channels.Channel.Water.Failure),
			Report.Channels.Channel.Water.Knots,
			Report.Channels.Channel.Water.Passes,
			Report.Channels.Channel.Water.Evaluations,
			Report.Channels.Channel.Water.HeightErrorMeters,
			Report.Channels.Channel.Water.SlopeError
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Watershed bypass climbing: attempts=%u localWindows=%u escape0=%.17g escape1=%.17g"),
			Report.Bypass.ClimbingAttempts,
			Report.Bypass.LocalWindows,
			Report.Bypass.EscapeMeters[0],
			Report.Bypass.EscapeMeters[1]
		)
	);
	AddInfo(
		FString::Printf(
			TEXT(
				"Watershed local window: failure=%u evaluations=%llu windows=%u width=%.17g halfCells=%u critical=(%u,%u,%u) cellCandidates=%u"
			),
			static_cast<unsigned>(Report.Bypass.LastWindow.Failure),
			Report.Bypass.LastWindow.Evaluations,
			Report.Bypass.LastWindow.Windows,
			Report.Bypass.LastWindow.LastHalfWidthMeters,
			Report.Bypass.LastWindow.LastHalfCells,
			Report.Bypass.LastWindow.CriticalKinds[0],
			Report.Bypass.LastWindow.CriticalKinds[1],
			Report.Bypass.LastWindow.CriticalKinds[2],
			Report.Bypass.LastWindow.CellCriticalCandidates
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Watershed bypass ocean: attempts=%u calls=%llu distance=%.17g measured=%d nodes=%u spacing=%.17g crest=%.17g"),
			Report.Bypass.OceanVerificationAttempts,
			Report.Bypass.OceanVerificationEvaluations,
			Report.Bypass.LastOceanDistanceMeters,
			static_cast<int>(Report.Bypass.HasLastOceanPassage),
			Report.Bypass.LastOceanNodes,
			Report.Bypass.LastOceanSpacingMeters,
			Report.Bypass.LastOceanCrestMeters
		)
	);
	if(Report.Bypass.LastOceanTerminal && Report.Bypass.LastOceanAnchor) {
		const PlanetCore::Vec3 OceanTerminal = Report.Bypass.LastOceanTerminal->Vector();
		const PlanetCore::Vec3 OceanAnchor = Report.Bypass.LastOceanAnchor->Vector();
		const PlanetCore::Vec3 OceanPeak = Report.Bypass.LastOceanPeak ? Report.Bypass.LastOceanPeak->Vector() : PlanetCore::Vec3{};
		AddInfo(
			FString::Printf(
				TEXT("Watershed ocean geometry: width=%.17g terminal=(%.17g,%.17g,%.17g) anchor=(%.17g,%.17g,%.17g) peak=(%.17g,%.17g,%.17g)"),
				Report.Bypass.LastOceanHalfWidthMeters,
				OceanTerminal.X,
				OceanTerminal.Y,
				OceanTerminal.Z,
				OceanAnchor.X,
				OceanAnchor.Y,
				OceanAnchor.Z,
				OceanPeak.X,
				OceanPeak.Y,
				OceanPeak.Z
			)
		);
	}
	if(Report.Bypass.LastWindow.CenterDirection) {
		const PlanetCore::Vec3 WindowCenter = Report.Bypass.LastWindow.CenterDirection->Vector();
		AddInfo(
			FString::Printf(
				TEXT("Watershed bypass actual center: D=(%.17g,%.17g,%.17g) unrelated=%u internal=%u reused=%u unresolved=%u highExcluded=%u"),
				WindowCenter.X,
				WindowCenter.Y,
				WindowCenter.Z,
				Report.Bypass.LastWindow.UnrelatedConnections,
				Report.Bypass.LastWindow.NewInternalConnections,
				Report.Bypass.LastWindow.ReusedConnections,
				Report.Bypass.LastWindow.UnresolvedConnections,
				Report.Bypass.LastWindow.ExcludedHighPasses
			)
		);
	}
	if(Report.Bypass.FirstRejectedCandidate) {
		const PlanetCore::Vec3 P = Report.Bypass.FirstRejectedCandidate->Vector();
		AddInfo(
			FString::Printf(
				TEXT("Watershed bypass unresolved: D=(%.17g,%.17g,%.17g) failure=%u branch=%u end=%u move=%.17g"),
				P.X,
				P.Y,
				P.Z,
				static_cast<unsigned>(Report.Bypass.FirstRejectedConnection.Failure),
				Report.Bypass.FirstRejectedConnection.Branch,
				static_cast<unsigned>(Report.Bypass.FirstRejectedConnection.TraceEnd),
				Report.Bypass.FirstRejectedConnection.TerminalMoveMeters
			)
		);
	}
	AddInfo(
		FString::Printf(
			TEXT("Network retained: connections=%u minima=%u branchSamples=%u initialOceanMeters=%.17g ")
			TEXT("closestFrontierMeters=%.17g nearestOceanMeters=%.17g"),
			Report.Network.PreparedConnections,
			Report.Network.PreparedMinima,
			Report.Network.RetainedBranchSamples,
			Report.Network.InitialOceanDistanceMeters,
			Report.Network.ClosestFrontierDistanceMeters,
			Report.Network.ClosestOceanDistanceMeters
		)
	);
	const PlanetCore::FlowNetworkDiagnostics &Channels = Report.Channels;
	const PlanetCore::SpillLakeDiagnostics &Shore = Channels.Channel.Shore;
	AddInfo(FString::Printf(TEXT("Channel retained geometrySamples=%u"), Channels.RetainedGeometrySamples));
	AddInfo(
		FString::Printf(TEXT("Bypass actual windows=%u spacing=%.17g"), Report.Bypass.MeasuredWindows, Report.Bypass.LastSpacingMeters)
	);
	AddInfo(
		FString::Printf(
			TEXT("Bypass passage candidates calls=%llu stageBudget=%d"),
			static_cast<unsigned long long>(Report.Bypass.PassageCandidateEvaluations),
			static_cast<int>(Report.Bypass.PassageCandidateBudgetExhausted)
		)
	);
	const PlanetCore::Vec3 Membership = Channels.MembershipDirection;
	AddInfo(
		FString::Printf(
			TEXT("Channel membership: kind=%u site=%u H=%.17g distance=%.17g D=(%.17g,%.17g,%.17g)"),
			static_cast<unsigned>(Channels.Membership),
			Channels.MembershipSite,
			Channels.MembershipHeightMeters,
			Channels.MembershipSignedDistanceMeters,
			Membership.X,
			Membership.Y,
			Membership.Z
		)
	);
	const PlanetCore::FlowRouteDiagnostics &Curve = Channels.Channel.Outlet.Curve;
	AddInfo(
		FString::Printf(
			TEXT("Channel curve: failure=%u index=%u distance=%.17g chord=%.17g forward=(%.17g,%.17g) curvature=(%.17g,%.17g)"),
			static_cast<unsigned>(Curve.Failure),
			Curve.Index,
			Curve.DistanceDifferenceMeters,
			Curve.ChordMeters,
			Curve.ForwardProducts[0],
			Curve.ForwardProducts[1],
			Curve.CurvatureProducts[0],
			Curve.CurvatureProducts[1]
		)
	);
	const PlanetCore::Vec3 Pass = Channels.OutletPassDirection;
	AddInfo(
		FString::Printf(
			TEXT("Channel detail: basin=%u required=%u coastal=%u complexes=%u compound=%u equal=%u lowered=%u ")
			TEXT("pass=(%.17g,%.17g,%.17g) positive=%d water=%.17g receiver=%.17g"),
			Channels.Basin,
			Channels.RequiredChannels,
			Channels.CoastalChannels,
			Channels.LakeComplexes,
			Channels.CompoundWaterBodies,
			Channels.EqualWaterConnections,
			Channels.LoweredSills,
			Pass.X,
			Pass.Y,
			Pass.Z,
			static_cast<int>(Channels.IsPositiveDownstream),
			Channels.UpstreamWaterMeters,
			Channels.DownstreamWaterMeters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Channel outlet: failure=%u upstreamEnd=%u downstreamEnd=%u upstreamVertices=%u downstreamVertices=%u"),
			static_cast<unsigned>(Report.Channels.Channel.Outlet.Failure),
			static_cast<unsigned>(Report.Channels.Channel.Outlet.UpstreamEnd),
			static_cast<unsigned>(Report.Channels.Channel.Outlet.DownstreamEnd),
			Report.Channels.Channel.Outlet.UpstreamVertices,
			Report.Channels.Channel.Outlet.DownstreamVertices
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Channel shore: failure=%u end=%u vertices=%u length=%.17g residual=%.17g gap=%.17g area=%.17g ")
			TEXT("last=(%.17g,%.17g,%.17g)"),
			static_cast<unsigned>(Shore.Failure),
			static_cast<unsigned>(Shore.End),
			Shore.Vertices,
			Shore.LengthMeters,
			Shore.HeightResidualMeters,
			Shore.FirstGapMeters,
			Shore.AreaMetersSquared,
			Shore.LastDirection.X,
			Shore.LastDirection.Y,
			Shore.LastDirection.Z
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Network frontier: centerQueries=%u exhaustedInitialWindows=%u exhaustedClimbing=%u exhaustedWindows=%u"),
			Report.Network.CenterQueries,
			Report.Network.ExhaustedInitialWindowCenters,
			Report.Network.ExhaustedClimbingCenters,
			Report.Network.ExhaustedWindowCenters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Network climbing: attempts=%u candidates=%u failures=%u end=%u normal=%.17g tipGradient=%.17g"),
			Discovery.ClimbingAttempts,
			Discovery.ClimbingCandidates,
			Discovery.ClimbingFailures,
			Discovery.LastClimbEnd,
			Discovery.LastClimbNormalSlope,
			Discovery.LastClimbTipGradientNorm
		)
	);
	AddInfo(
		FString::Printf(TEXT("Network source reuse: hits=%llu retained=%u"), Report.Network.CacheHits, Report.Network.CachedSamples)
	);
	AddInfo(
		FString::Printf(
			TEXT("Last string: normalSlope=%.17g motion=%.17g discoveryEvaluations=%llu halfCells=%u"),
			Discovery.LastRelaxedNormalSlope,
			Discovery.LastRelaxedMotionMeters,
			Discovery.Evaluations,
			Discovery.LastHalfCells
		)
	);
	for(const std::array<PlanetCore::UnitDirection, 2> &Endpoints : Discovery.BridgeEndpoints) {
		const PlanetCore::Vec3 From = Endpoints[0].Vector(), To = Endpoints[1].Vector();
		AddInfo(
			FString::Printf(
				TEXT("Attempted bridge: from=(%.17g,%.17g,%.17g) to=(%.17g,%.17g,%.17g)"),
				From.X,
				From.Y,
				From.Z,
				To.X,
				To.Y,
				To.Z
			)
		);
	}
	AddInfo(
		FString::Printf(
			TEXT("Unresolved candidates: critical=%u connection=%u"),
			Discovery.UnresolvedCriticalPoints,
			Discovery.UnresolvedConnections
		)
	);
	if(Discovery.CenterDirection) {
		const PlanetCore::Vec3 Center = Discovery.CenterDirection->Vector();
		AddInfo(FString::Printf(TEXT("Discovery center: D=(%.17g,%.17g,%.17g)"), Center.X, Center.Y, Center.Z));
	}
	if(Discovery.FirstFailedConnectionCandidate) {
		const PlanetCore::Vec3 Failed = Discovery.FirstFailedConnectionCandidate->Vector();
		const PlanetCore::SpillConnectionDiagnostics &Connection = Discovery.FirstFailedConnection;
		AddInfo(
			FString::Printf(
				TEXT("First unresolved connection: D=(%.17g,%.17g,%.17g) failure=%u branch=%u end=%u move=%.17g"),
				Failed.X,
				Failed.Y,
				Failed.Z,
				static_cast<unsigned>(Connection.Failure),
				Connection.Branch,
				static_cast<unsigned>(Connection.TraceEnd),
				Connection.TerminalMoveMeters
			)
		);
	}
	if(Discovery.FirstRejectedBridgeStart && Discovery.FirstRejectedBridgeEnd) {
		const PlanetCore::Vec3 From = Discovery.FirstRejectedBridgeStart->Vector(), To = Discovery.FirstRejectedBridgeEnd->Vector();
		AddInfo(
			FString::Printf(
				TEXT("Rejected bridge: meters=%.17g from=(%.17g,%.17g,%.17g) to=(%.17g,%.17g,%.17g) discoveryEvaluations=%llu"),
				Discovery.FirstRejectedBridgeLengthMeters,
				From.X,
				From.Y,
				From.Z,
				To.X,
				To.Y,
				To.Z,
				Discovery.Evaluations
			)
		);
	}
	TestTrue(*FString::Printf(TEXT("Seed %llu の最初の陸上地点を準備"), Seed), Watershed.has_value());
	// 準備出来なかった条件を後続の照会で隠さない返戻
	if(!Watershed) return false;
	const std::optional<PlanetCore::FlowChannelSection> Sample = Watershed->Field->Evaluate(*Location);
	TestTrue(TEXT("開始地点の有限な標高"), Sample && std::isfinite(Sample->Terrain.ElevationMeters));
	TestTrue(TEXT("少なくとも一つの大洋への河道"), Report.Channels.CoastalChannels > 0);
	// 全ての生成条件の結果の返戻
	return !HasAnyErrors();
}

#endif
