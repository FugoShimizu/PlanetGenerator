#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetDescentPath.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetOceanShoreTerminalSurvey,
	"PlanetGenerator.Ocean.ShoreTerminalSurvey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実行 V52 の海面終点を同じ元場で再現し，深い下降先と既知大洋の対応を測る
 * 此の診断の成功を，未確認の大洋接続又は水系の成功とは扱わない
 * @param Parameters 未使用の実行引数
 * @return 有界な診断が同じ元場の実数値を取得出来た場合に真
 */
bool FPlanetOceanShoreTerminalSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 0;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Definition);
	// 元の地質を再現出来ない場合の返戻
	if(!TestTrue(TEXT("同じ地質を再現"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*PreparedGeology));
	std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 同じ造山場の欠落への返戻
	if(!TestTrue(TEXT("同じ造山場を再現"), PreparedRidges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*PreparedRidges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoOnly,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 未再現の元場を別の場へ置換えない返戻
	if(!TestTrue(TEXT("同じ大洋の元場を再現"), Ocean.has_value())) return false;
	constexpr std::uint64_t MaximumCalls = 4000000;
	std::uint64_t Calls = 0;
	bool BudgetExhausted = false;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		// 診断全体の元場照会予算を越えた場合の非有限値の返戻
		if(!(Calls < MaximumCalls)) {
			BudgetExhausted = true;
			PlanetCore::TerrainSample Invalid;
			Invalid.ElevationMeters = std::numeric_limits<double>::quiet_NaN();
			// 予算切れを有限な標高に見せない返戻
			return Invalid;
		}
		++Calls;
		// 同じ不変な元場の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Terminal =
	*PlanetCore::UnitDirection::FromVector({ -0.28934040550228901, 0.10658862306316474, 0.95127335460280271 });
	const PlanetCore::UnitDirection Anchor =
	*PlanetCore::UnitDirection::FromVector({ -0.31583105142581563, 0.11254306437113291, 0.94211719314383535 });
	const PlanetCore::UnitDirection Peak =
	*PlanetCore::UnitDirection::FromVector({ -0.3038319825058784, 0.099197817489389384, 0.94754731776934986 });
	for(const PlanetCore::UnitDirection D : { Terminal, Anchor, Peak }) {
		const PlanetCore::TerrainSample Value = Source(D);
		const double BaseHeight = Geology->EvaluateElevation(D);
		TestTrue(TEXT("再現点の元場が有限"), std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length()));
		const PlanetCore::Vec3 P = D.Vector();
		double NearestCorridor = std::numeric_limits<double>::infinity();
		std::optional<PlanetCore::UnitDirection> CorridorPoint;
		for(const PlanetCore::OceanCorridor &Corridor : Ocean->Corridors()) {
			const double AlongAngle = std::clamp(std::atan2(P.Dot(Corridor.Tangent), P.Dot(Corridor.Start)), 0.0, Corridor.ArcAngle);
			const PlanetCore::UnitDirection Q =
			*PlanetCore::UnitDirection::FromVector(Corridor.Start * std::cos(AlongAngle) + Corridor.Tangent * std::sin(AlongAngle));
			const double Distance = Definition.RadiusMeters * std::atan2(P.Cross(Q.Vector()).Length(), P.Dot(Q.Vector()));
			if(Distance < NearestCorridor) {
				NearestCorridor = Distance;
				CorridorPoint = Q;
			}
		}
		const PlanetCore::Vec3 C = CorridorPoint ? CorridorPoint->Vector() : PlanetCore::Vec3{};
		AddInfo(
			FString::Printf(
				TEXT("Shore terminal source D=(%.17g,%.17g,%.17g) H=%.17g base=%.17g gradient=%.17g ")
				TEXT("corridorMeters=%.17g center=(%.17g,%.17g,%.17g) centerH=%.17g"),
				P.X,
				P.Y,
				P.Z,
				Value.ElevationMeters,
				BaseHeight,
				Value.TangentialGradientPerRadian.Length() / Definition.RadiusMeters,
				NearestCorridor,
				C.X,
				C.Y,
				C.Z,
				CorridorPoint ? Source(*CorridorPoint).ElevationMeters : 0
			)
		);
	}
	PlanetCore::TerrainDescentSettings DescentSettings;
	DescentSettings.RadiusMeters = Definition.RadiusMeters;
	DescentSettings.StopElevationMeters = -100000;
	DescentSettings.MaximumLengthMeters = 200000;
	DescentSettings.MaximumVertices = 8192;
	DescentSettings.GradientTolerance = 1E-8;
	const std::optional<PlanetCore::TerrainDescentPath> Descent =
	PlanetCore::TraceTerrainDescent(Terminal, Source, DescentSettings);
	// 同じ水没域からの有界な下降の欠落への返戻
	if(!TestTrue(TEXT("同じ元場で海面以下の下降を測定"), Descent && !Descent->Samples.empty())) return false;
	const PlanetCore::TerrainDescentSample &Deep = Descent->Samples.back();
	PlanetCore::TerrainCriticalDiagnostics CriticalReport;
	const std::optional<PlanetCore::TerrainCriticalPoint> Critical =
	PlanetCore::RefineTerrainCriticalPoint(Deep.Direction, Source, {}, {}, &CriticalReport);
	PlanetCore::UnitDirection NearestAnchor = Anchor;
	double NearestDistance = std::numeric_limits<double>::infinity();
	for(const PlanetCore::UnitDirection Candidate : Ocean->ConnectedOceanAnchors()) {
		const PlanetCore::Vec3 A = Deep.Direction.Vector(), B = Candidate.Vector();
		const double Distance = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
		if(Distance < NearestDistance) {
			NearestDistance = Distance;
			NearestAnchor = Candidate;
		}
	}
	const PlanetCore::Vec3 DeepPoint = Deep.Direction.Vector(), Nearest = NearestAnchor.Vector();
	AddInfo(
		FString::Printf(
			TEXT("Shore terminal descent end=%u length=%.17g H=%.17g D=(%.17g,%.17g,%.17g) samples=%u ")
			TEXT("critical=%d failure=%u curvature=(%.17g,%.17g) nearest=%.17g anchor=(%.17g,%.17g,%.17g)"),
			static_cast<unsigned>(Descent->End),
			Deep.DistanceMeters,
			Deep.ElevationMeters,
			DeepPoint.X,
			DeepPoint.Y,
			DeepPoint.Z,
			static_cast<unsigned>(Descent->Samples.size()),
			Critical ? static_cast<int>(Critical->Kind) : -1,
			static_cast<unsigned>(CriticalReport.Failure),
			CriticalReport.MinimumCurvaturePerMeter,
			CriticalReport.MaximumCurvaturePerMeter,
			NearestDistance,
			Nearest.X,
			Nearest.Y,
			Nearest.Z
		)
	);
	for(const double Width : { 1000.0, 4000.0, 16000.0 }) {
		PlanetCore::SpillPassageSettings Settings;
		Settings.HalfWidthMeters = Width;
		Settings.AdaptCandidateSpacingToBudget = true;
		Settings.UseEndpointNeighborRing = true;
		const std::optional<PlanetCore::SpillPassage> Passage =
		PlanetCore::FindSpillPassage(Deep.Direction, NearestAnchor, Source, Settings);
		AddInfo(
			FString::Printf(
				TEXT("Shore terminal deep passage width=%.17g measured=%d crest=%.17g calls=%llu"),
				Width,
				static_cast<int>(Passage.has_value()),
				Passage ? Passage->CrestMeters + Passage->MaximumIndependentExcessMeters : 0,
				Calls
			)
		);
		if(Passage && !(Passage->CrestMeters + Passage->MaximumIndependentExcessMeters > 0)) break;
	}
	TestFalse(TEXT("診断の全体予算を越えない"), BudgetExhausted);
	// 元場と有界な計測の全結果の返戻；大洋への到達は此の返戻条件に含めない
	return !HasAnyErrors();
}

#endif
