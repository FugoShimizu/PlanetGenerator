#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillWetDetourContracts,
	"PlanetGenerator.SpillPassage.WetDetour",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 細長い島の両側を結ぶ同じ元場で，狭い窓の正当な拒否と広い水没迂回を検証する
 * @param Parameters 未使用の実行引数
 * @return 独立な障害高度と水没迂回・保持上限・元場不変を確認した場合に真
 */
bool FPlanetSpillWetDetourContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const double Scale = std::min(1.0, Radius / 250000), ShortAxis = 2000 * Scale, LongAxis = 10000 * Scale;
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z;
			const double Signal = 50 * std::exp(-X * X / (ShortAxis * ShortAxis) - Y * Y / (LongAxis * LongAxis));
			const double DX = -2 * X * Signal / (ShortAxis * ShortAxis), DY = -2 * Y * Signal / (LongAxis * LongAxis);
			const PlanetCore::Vec3 G { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 独立に零等高線を求められる楕円形の滑らかな島の返戻
			return { Signal - 1, G };
		};
		const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector({ -6000 * Scale / Radius, 0, 1 });
		const PlanetCore::UnitDirection End = *PlanetCore::UnitDirection::FromVector({ 6000 * Scale / Radius, 0, 1 });
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.HalfWidthMeters = 4000 * Scale;
		Settings.SpacingMeters = 50 * Scale;
		Settings.MaximumSampleStepMeters = std::max(0.01, 20 * Scale);
		Settings.AdaptCandidateSpacingToBudget = true;
		Settings.UseEndpointNeighborRing = true;
		const std::optional<PlanetCore::SpillPassage> Narrow = PlanetCore::FindSpillPassage(Start, End, Source, Settings);
		// 狭い窓の実計測の欠落への返戻
		if(!TestTrue(TEXT("狭い窓の障害を実測"), Narrow.has_value())) return false;
		const double NecessaryHeight = 50 * std::exp(-0.16) - 1;
		TestTrue(TEXT("狭い矩形の全通路は島の独立な高度以上"), Narrow->CrestMeters + 1E-6 >= NecessaryHeight);
		TestTrue(TEXT("狭い窓の海面接続は拒否"), Narrow->CrestMeters > 0);
		Settings.HalfWidthMeters = 32000 * Scale;
		Calls = 0;
		const std::optional<PlanetCore::SpillPassage> Wide = PlanetCore::FindSpillPassage(Start, End, Source, Settings);
		// 同じ元場の広い候補を測れなかった場合の返戻
		if(!TestTrue(TEXT("島を回る水没通路を準備"), Wide.has_value())) return false;
		const double Maximum = Wide->CrestMeters + Wide->MaximumIndependentExcessMeters;
		TestTrue(TEXT("海面を上げず全実通路が水没"), std::isfinite(Maximum) && !(Maximum > 0));
		TestTrue(TEXT("広い窓も同じ格子の保持上限以内"), Wide->Nodes <= Settings.MaximumNodes);
		TestEqual(TEXT("広い候補の費用は実照会と一致"), Wide->Evaluations, Calls);
		for(unsigned I = 0; I < 257; ++I) {
			const double X = (static_cast<double>(I) / 256 - 0.5) * 12000 * Scale, Y = 25000 * Scale;
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector({ X / Radius, Y / Radius, 1 });
			const double Height = Source(D).ElevationMeters;
			TestTrue(TEXT("独立な楕円外側の迂回も水没"), std::isfinite(Height) && !(Height > 0));
		}
		AddInfo(
			FString::Printf(
				TEXT("Wet detour R=%.0f narrowCrest=%.17g necessary=%.17g wideCrest=%.17g nodes=%u spacing=%.17g calls=%llu"),
				Radius,
				Narrow->CrestMeters,
				NecessaryHeight,
				Wide->CrestMeters,
				Wide->Nodes,
				Wide->CandidateSpacingMeters,
				Wide->Evaluations
			)
		);
	}
	// 狭い窓の正当な拒否と，同じ元場の水没迂回の全結果の返戻
	return !HasAnyErrors();
}

#endif
