#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLongSpillCandidateBudget,
	"PlanetGenerator.SpillPassage.LongCandidateBudget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 格子点数で拒否される長い通路を，同じ保持量と辺実測精度で検査する
 * @param Parameters 未使用の実行引数
 * @return 解析的な通過高度・実端点・評価上限・中断を保持した場合に真
 */
bool FPlanetLongSpillCandidateBudget::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const double Length = Radius == 1000 ? 200 : 100000, HalfWidth = Radius == 1000 ? 8 : 4096;
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, U = X / Length, V = Y / HalfWidth, DX = -2 * U / Length;
			const double DY = 2 * V / HalfWidth;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 任意の連続経路がＸ＝０で高さ５０１以上を通る解析場の返戻
			return { 501 - U * U + V * V, Gradient };
		};
		const PlanetCore::UnitDirection From = *PlanetCore::UnitDirection::FromVector({ -Length / Radius, 0, 1 });
		const PlanetCore::UnitDirection To = *PlanetCore::UnitDirection::FromVector({ Length / Radius, 0, 1 });
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SpacingMeters = Radius == 1000 ? 1 : 16;
		Settings.HalfWidthMeters = HalfWidth;
		Settings.MaximumNodes = Radius == 1000 ? 1024 : 65536;
		Settings.MaximumSampleStepMeters = 4;
		Settings.UsePathEdgeSelection = true;
		TestFalse(TEXT("固定格子の保持量超過を同じ入力で再現"), PlanetCore::FindSpillPassage(From, To, Source, Settings).has_value());
		TestEqual(TEXT("固定格子の拒否は元場照会前"), Calls, UINT64_C(0));
		Settings.AdaptCandidateSpacingToBudget = true;
		const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(From, To, Source, Settings);
		// 有限な保持量へ収めても実測不能な場合の返戻
		if(!TestTrue(TEXT("同じ長い通路を元場で実測"), Passage.has_value())) return false;
		TestTrue(TEXT("解析的な通過高度５０１に一致"), std::abs(Passage->CrestMeters - 501) < 1E-8);
		TestTrue(TEXT("保持量の上限を拡げない"), Passage->Nodes <= Settings.MaximumNodes);
		TestTrue(TEXT("候補の間隔だけを保持量に合わせる"), Passage->CandidateSpacingMeters > Settings.SpacingMeters);
		TestEqual(TEXT("始点は元の実方向"), (Passage->Path.front().Vector() - From.Vector()).Length(), 0.0);
		TestEqual(TEXT("終点は元の実方向"), (Passage->Path.back().Vector() - To.Vector()).Length(), 0.0);
		TestEqual(TEXT("実際の全元場照会を集計"), Passage->Evaluations, Calls);
		TestTrue(TEXT("実測も同じ有限評価予算"), Calls <= Settings.MaximumEvaluations);
		// 格子と測定位置を共有しない独立な曲線上の試料
		for(std::size_t I = 1; I < Passage->Path.size(); ++I) for(unsigned Probe = 0; Probe < 5; ++Probe) {
			const double Fraction = (Probe + 0.5) / 5;
			const PlanetCore::UnitDirection D =
			*PlanetCore::UnitDirection::FromVector(Passage->Path[I - 1].Vector() * (1 - Fraction) + Passage->Path[I].Vector() * Fraction);
			TestTrue(TEXT("候補辺の内部も解析上限以下"), Source(D).ElevationMeters <= 501 + 1E-8);
		}
		AddInfo(
			FString::Printf(
				TEXT("Long candidate R=%.0f spacing=%.17g nodes=%u length=%.17g crest=%.17g evaluations=%llu"),
				Radius,
				Passage->CandidateSpacingMeters,
				Passage->Nodes,
				Passage->PathLengthMeters,
				Passage->CrestMeters,
				static_cast<unsigned long long>(Passage->Evaluations)
			)
		);
		Calls = 0;
		Settings.MaximumEvaluations = 100;
		TestFalse(TEXT("適応格子でも評価予算後は公開しない"), PlanetCore::FindSpillPassage(From, To, Source, Settings).has_value());
		TestEqual(TEXT("元場への呼出は指定した上限迄"), Calls, UINT64_C(100));
		std::stop_source Stop;
		Stop.request_stop();
		Calls = 0;
		TestFalse(TEXT("中断済では適応準備しない"), PlanetCore::FindSpillPassage(From, To, Source, Settings, Stop.get_token()).has_value());
		TestEqual(TEXT("中断後の照会は零"), Calls, UINT64_C(0));
		const auto Invalid = [](const PlanetCore::UnitDirection) -> PlanetCore::TerrainSample {
			const double NaN = std::numeric_limits<double>::quiet_NaN();
			// 適応候補でも欠落した元場を捨てない返戻
			return { NaN, {} };
		};
		Settings.MaximumEvaluations = 4000000;
		TestFalse(TEXT("非有限な元場の通路を受理しない"), PlanetCore::FindSpillPassage(From, To, Invalid, Settings).has_value());
	}
	// 同じ保持量・元場・実測精度の検査結果の返戻
	return !HasAnyErrors();
}

#endif
