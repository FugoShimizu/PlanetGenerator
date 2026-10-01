#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillEndpointAttachmentContracts,
	"PlanetGenerator.SpillPassage.EndpointAttachment",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 格子線上の海岸端点を，水没している反対側の格子へ実辺で接続する
 * 独立な解析曲線の岸を使い，旧四隅の拒否と実水没通路を同時に保持する
 * @param Parameters 未使用の実行引数
 * @return 三半径で実通路・旧失敗・評価予算・中断を確認した場合に真
 */
bool FPlanetSpillEndpointAttachmentContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, Q = X * X - 7225, DX = 4E-8 * X * Q;
			const PlanetCore::Vec3 G { Radius * DX / D.Z, Radius / D.Z, -Radius * (DX * D.X + D.Y) / (D.Z * D.Z) };
			// 端点 X ＝±８５・Y ＝０は水没し，格子線の北側四隅だけは陸になる元場の返戻
			return { 1E-8 * Q * Q + Y - 0.001, G };
		};
		const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector({ -85 / Radius, 0, 1 });
		const PlanetCore::UnitDirection End = *PlanetCore::UnitDirection::FromVector({ 85 / Radius, 0, 1 });
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.HalfWidthMeters = 200;
		Settings.SpacingMeters = 50;
		Settings.MaximumSampleStepMeters = 2;
		const std::optional<PlanetCore::SpillPassage> FourCorners = PlanetCore::FindSpillPassage(Start, End, Source, Settings);
		// 旧候補を測れなかった場合の返戻
		if(!TestTrue(TEXT("旧四隅の実通過高度を保持"), FourCorners.has_value())) return false;
		TestTrue(TEXT("旧四隅だけでは同じ海岸を陸と判定"), FourCorners->CrestMeters + FourCorners->MaximumIndependentExcessMeters > 0);
		Settings.UseEndpointNeighborRing = true;
		Calls = 0;
		const std::optional<PlanetCore::SpillPassage> Ring = PlanetCore::FindSpillPassage(Start, End, Source, Settings);
		// 水位や端点を変えて拒否を隠さない返戻
		if(!TestTrue(TEXT("周囲セルを含む実通路を準備"), Ring.has_value())) return false;
		const double Maximum = Ring->CrestMeters + Ring->MaximumIndependentExcessMeters;
		TestTrue(TEXT("実辺と独立な再標本も海面以下"), std::isfinite(Maximum) && !(Maximum > 0));
		TestEqual(TEXT("全候補格子の保持量は同じ"), Ring->Nodes, FourCorners->Nodes);
		TestEqual(TEXT("実照会数を過小計上しない"), Ring->Evaluations, Calls);
		TestTrue(TEXT("端点辺も実際に測定"), Ring->EvaluatedEdges > 0);
		std::uint32_t IndependentSamples = 0;
		for(std::size_t I = 1; I < Ring->Path.size(); ++I) {
			const PlanetCore::Vec3 A = Ring->Path[I - 1].Vector(), B = Ring->Path[I].Vector();
			const double Length = Radius * std::atan2(A.Cross(B).Length(), A.Dot(B));
			const unsigned Steps = std::max(1U, static_cast<unsigned>(std::ceil(Length)));
			for(unsigned J = 0; J < Steps + 1; ++J) {
				const double Fraction = static_cast<double>(J) / Steps;
				const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
				const double Height = Source(D).ElevationMeters;
				TestTrue(TEXT("独立な１ｍ以下の通路標本も水没"), std::isfinite(Height) && !(Height > 0));
				++IndependentSamples;
			}
		}
		AddInfo(
			FString::Printf(
				TEXT("Endpoint attachment R=%.0f fourCrest=%.17g ringCrest=%.17g nodes=%u calls=%llu independent=%u"),
				Radius,
				FourCorners->CrestMeters,
				Ring->CrestMeters,
				Ring->Nodes,
				Ring->Evaluations,
				IndependentSamples
			)
		);
		Settings.MaximumEvaluations = 32;
		Calls = 0;
		TestFalse(TEXT("未計測の端点接続を予算後に公開しない"), PlanetCore::FindSpillPassage(Start, End, Source, Settings).has_value());
		TestEqual(TEXT("端点の追加辺も同じ実照会予算"), Calls, UINT64_C(32));
		std::stop_source Stop;
		Stop.request_stop();
		Calls = 0;
		TestFalse(TEXT("中断した端点探索を拒否"), PlanetCore::FindSpillPassage(Start, End, Source, Settings, Stop.get_token()).has_value());
		TestEqual(TEXT("中断後の元場照会は零"), Calls, UINT64_C(0));
	}
	// 旧拒否・正規水没通路・独立標本・予算・中断の全結果の返戻
	return !HasAnyErrors();
}

#endif
