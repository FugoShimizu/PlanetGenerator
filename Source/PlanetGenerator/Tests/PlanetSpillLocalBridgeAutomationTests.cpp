#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillPassage.h"
#include "Core/PlanetSpillString.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillLocalBridge,
	"PlanetGenerator.SpillString.LocalBridge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 長い水系探索が繰返した短区間を，同じ元場と固定極小から独立に再現する
 * @param Parameters 未使用の実行引数
 * @return 準備した実鞍点の両下降枝だけで入力の二つの極小を繋いだ場合に真
 */
bool FPlanetSpillLocalBridge::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 元の失敗と同じ不変な地質・起伏・大洋場の準備
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 7;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元の生成条件の欠落への返戻
	if(!TestTrue(TEXT("局所通路の元地質"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山基盤の欠落への返戻
	if(!TestTrue(TEXT("局所通路の造山場"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 同じ元場を再現出来ない場合の返戻
	if(!TestTrue(TEXT("局所通路の大洋場"), Ocean.has_value())) return false;
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		++Calls;
		// 元の標高と解析勾配の返戻
		return Ocean->Evaluate(D);
	};
	const std::array<PlanetCore::UnitDirection, 2> Endpoints {
		*PlanetCore::UnitDirection::FromVector({ -0.26516096830143476, 0.53667877432912447, 0.80104029491283046 }),
		*PlanetCore::UnitDirection::FromVector({ -0.26517059137068527, 0.5371332845254313, 0.80073240981431681 })
	};
	const double Radius = Definition.RadiusMeters;
	std::vector<PlanetCore::TerrainCriticalPoint> Minima;
	for(const PlanetCore::UnitDirection D : Endpoints) {
		const std::optional<PlanetCore::TerrainCriticalPoint> Minimum = PlanetCore::RefineTerrainCriticalPoint(D, Source, {});
		// 診断に記録した端点が実極小でない場合の返戻
		if(!TestTrue(TEXT("記録された両端は実極小"), Minimum && Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum)) return false;
		Minima.push_back(*Minimum);
	}
	const PlanetCore::Vec3 A = Endpoints[0].Vector(), B = Endpoints[1].Vector();
	const double Separation = Radius * std::atan2(A.Cross(B).Length(), A.Dot(B));
	PlanetCore::SpillPassageSettings PassageSettings;
	PassageSettings.SpacingMeters = std::min(25.0, Separation / 64);
	PassageSettings.HalfWidthMeters = std::min({ 512.0, Radius * 0.25, std::max(16.0, Separation * 0.5) });
	const std::optional<PlanetCore::SpillPassage> Passage =
	PlanetCore::FindSpillPassage(Endpoints[0], Endpoints[1], Source, PassageSettings);
	// 短い通路を準備出来ない場合の返戻
	if(!TestTrue(TEXT("未接続の同じ短区間を再現"), Passage.has_value())) return false;
	const auto Same = [&](const PlanetCore::TerrainCriticalPoint &P, const PlanetCore::TerrainCriticalPoint &Q) -> bool {
		const PlanetCore::Vec3 X = P.Direction.Vector(), Y = Q.Direction.Vector();
		// 本番と同じ物理距離・標高で独立に端点を照合した結果の返戻
		return P.Kind == Q.Kind && Radius * std::atan2(X.Cross(Y).Length(), X.Dot(Y)) <= 0.001 &&
		std::abs(P.ElevationMeters - Q.ElevationMeters) <= 1E-6;
	};
	const auto Connects = [&](const std::span<const PlanetCore::UnitDirection> Candidates, const TCHAR *Stage) -> bool {
		std::vector<PlanetCore::SpillConnection> Connections;
		std::array<std::uint32_t, 4> Kinds {};
		for(const PlanetCore::UnitDirection Candidate : Candidates) {
			const std::optional<PlanetCore::TerrainCriticalPoint> Critical = PlanetCore::RefineTerrainCriticalPoint(Candidate, Source, {});
			++Kinds[Critical ? 1 + static_cast<unsigned>(Critical->Kind) : 0];
			if(!Critical || Critical->Kind != PlanetCore::TerrainCriticalKind::Saddle) continue;
			if(std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(Candidate, Source, {})) {
				Connections.push_back(std::move(*Connection));
			}
		}
		// 形だけの経路でなく，元場の実下降終端を辺とする独立な接続判定
		std::vector<PlanetCore::TerrainCriticalPoint> Reached { Minima[0] };
		bool Changed = true;
		while(Changed) {
			Changed = false;
			for(const PlanetCore::SpillConnection &Connection : Connections) {
				if(!Connection.Branches[0].Minimum || !Connection.Branches[1].Minimum) continue;
				for(unsigned Side = 0; Side < 2; ++Side) {
					const PlanetCore::TerrainCriticalPoint &P = *Connection.Branches[Side].Minimum;
					const PlanetCore::TerrainCriticalPoint &Q = *Connection.Branches[1 - Side].Minimum;
					const bool First = std::any_of(
						Reached.begin(),
						Reached.end(),
						[&](const PlanetCore::TerrainCriticalPoint &R) -> bool {
							// 一方の実終端への到達の返戻
							return Same(R, P);
						}
					);
					const bool Second = std::any_of(
						Reached.begin(),
						Reached.end(),
						[&](const PlanetCore::TerrainCriticalPoint &R) -> bool {
							// 他方の実終端への到達の返戻
							return Same(R, Q);
						}
					);
					if(First && !Second) {
						Reached.push_back(Q);
						Changed = true;
					}
				}
			}
		}
		const bool Connected = std::any_of(
			Reached.begin(),
			Reached.end(),
			[&](const PlanetCore::TerrainCriticalPoint &P) -> bool {
				// 元の終点の実極小への到達の返戻
				return Same(P, Minima[1]);
			}
		);
		AddInfo(
			FString::Printf(
				TEXT(
					"Local bridge %s: candidates=%llu unresolved=%u minima=%u saddles=%u maxima=%u connections=%llu reached=%llu connected=%d"
				),
				Stage,
				static_cast<unsigned long long>(Candidates.size()),
				Kinds[0],
				Kinds[1],
				Kinds[2],
				Kinds[3],
				static_cast<unsigned long long>(Connections.size()),
				static_cast<unsigned long long>(Reached.size()),
				static_cast<int>(Connected)
			)
		);
		// 実接続群に依る到達結果の返戻
		return Connected;
	};
	const bool RawConnected = Connects(Passage->PassCandidates, TEXT("grid"));
	// 同じ固定端点で，有限な局所反復と下降刻みの影響を比較
	bool RelaxedConnected = false;
	for(const double TimeStep : { 500.0, 5000.0 }) {
		PlanetCore::SpillStringSettings StringSettings;
		StringSettings.MaximumIterations = 512;
		StringSettings.TimeStepMeters = TimeStep;
		const double Started = FPlatformTime::Seconds();
		const std::optional<PlanetCore::SpillString> Relaxed = PlanetCore::RelaxSpillString(Passage->Path, Source, StringSettings);
		// 元場と有限予算で緩和を準備出来ない場合の返戻
		if(!TestTrue(TEXT("局所通路の候補経路を緩和"), Relaxed.has_value())) return false;
		AddInfo(
			FString::Printf(
				TEXT("Local bridge relaxation: dt=%.0f separation=%.17g path=%.17g nodes=%llu iterations=%u evaluations=%llu ")
				TEXT("normalSlope=%.17g motion=%.17g ms=%.9g"),
				TimeStep,
				Separation,
				Passage->PathLengthMeters,
				static_cast<unsigned long long>(Relaxed->Path.size()),
				Relaxed->Iterations,
				Relaxed->Evaluations,
				Relaxed->MaximumNormalSlope,
				Relaxed->MaximumMotionMeters,
				(FPlatformTime::Seconds() - Started) * 1000
			)
		);
		const bool Connected = Connects(Relaxed->PassCandidates, TEXT("relaxed"));
		TestTrue(TEXT("各局所条件で同じ二盆地を接続"), Connected);
		if(TimeStep == 5000) {
			TestTrue(TEXT("採用条件では指定した変位の停止条件も満足"), Relaxed->MaximumMotionMeters <= StringSettings.MotionToleranceMeters);
		}
		RelaxedConnected = Connected || RelaxedConnected;
	}
	TestTrue(TEXT("同じ二盆地を実鞍点と下降終端で接続"), RawConnected || RelaxedConnected);
	AddInfo(FString::Printf(TEXT("Local bridge total source evaluations=%llu"), Calls));
	// 元の二つの実極小の接続結果の返戻
	return !HasAnyErrors();
}

#endif
