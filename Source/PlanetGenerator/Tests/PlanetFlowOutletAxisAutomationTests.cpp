#include "Core/PlanetFlowOutlet.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowOutletAxis, "PlanetGenerator.FlowChannel.AxisProvenance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 元の鞍点の主軸を反転しても，同じ物理枝を引継ぐ事を解析場で検査する
 * @param Parameters 未使用の実行引数
 * @return 三半径の方向・経路・終端が一致し，違う軸と中断を拒否した場合に真
 */
bool FPlanetFlowOutletAxis::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Origin = *PlanetCore::UnitDirection::FromVector({ 0, 0, 1 });
	unsigned Cases = 0;
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, DX = -0.002 * X, DY = 0.0014 * Y;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 東西下降・南北上昇の解析鞍点と，独立な接線微分の返戻
			return { 100 - 0.001 * X * X + 0.0007 * Y * Y, Gradient };
		};
		PlanetCore::FlowOutletSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.UpstreamLengthMeters = 40;
		Settings.MaximumDownstreamLengthMeters = 200;
		Settings.DownstreamWaterMeters = 92;
		Settings.ReferenceMinimumDirection = PlanetCore::Vec3{ 1, 0, 0 };
		Settings.IsPositiveBranch = true;
		PlanetCore::FlowOutletDiagnostics FirstReport;
		const std::optional<PlanetCore::FlowOutletRoute> First =
		PlanetCore::PrepareFlowOutletRoute(Origin, Source, Settings, {}, &FirstReport);
		// 正規の物理枝が作れない場合の返戻
		if(!TestTrue(TEXT("基準の東向き下降を準備"), First.has_value())) return false;
		TestEqual(TEXT("基準軸を保持した照会数"), static_cast<std::uint64_t>(First->Evaluations), Calls);
		Calls = 0;
		Settings.ReferenceMinimumDirection = PlanetCore::Vec3{ -1, 0, 0 };
		Settings.IsPositiveBranch = false;
		PlanetCore::FlowOutletDiagnostics InvertedReport;
		const std::optional<PlanetCore::FlowOutletRoute> Inverted =
		PlanetCore::PrepareFlowOutletRoute(Origin, Source, Settings, {}, &InvertedReport);
		// 符号だけを反転した同じ物理枝が作れない場合の返戻
		if(!TestTrue(TEXT("主軸と枝の符号の同時反転"), Inverted.has_value())) return false;
		TestEqual(TEXT("反転した基準軸でも照会数を保持"), static_cast<std::uint64_t>(Inverted->Evaluations), Calls);
		TestTrue(
			TEXT("受入水位へ到達"),
			FirstReport.DownstreamEnd == PlanetCore::TerrainDescentEnd::HeightLimit &&
			InvertedReport.DownstreamEnd == PlanetCore::TerrainDescentEnd::HeightLimit
		);
		TestTrue(TEXT("物理的な下降向きが東を向く"), FirstReport.DownstreamAxis.X > 0.99 && InvertedReport.DownstreamAxis.X > 0.99);
		const double Length = First->Route.GetLengthMeters();
		TestTrue(TEXT("符号反転でも経路長は不変"), std::abs(Length - Inverted->Route.GetLengthMeters()) < 1E-8);
		double MaximumDifference = 0;
		for(unsigned I = 0; I < 17; ++I) {
			const double Distance = Length * I / 16;
			const std::optional<PlanetCore::TerrainFlowCurveSample> A = First->Route.Sample(Distance), B = Inverted->Route.Sample(Distance);
			// 正規の内部距離が評価出来ない場合の返戻
			if(!TestTrue(TEXT("両経路の独立な内部距離を評価"), A && B)) return false;
			MaximumDifference = std::max(MaximumDifference, Radius * (A->Direction.Vector() - B->Direction.Vector()).Length());
			TestTrue(TEXT("解析流路の南北変位は零"), std::abs(Radius * A->Direction.Vector().Y) < 1E-7);
		}
		TestTrue(TEXT("全試料で同じ物理曲線を保持"), MaximumDifference < 1E-8);
		const PlanetCore::TerrainFlowCurveSample End = *First->Route.Sample(Length);
		TestTrue(
			TEXT("解析的な下流側に実到達"),
			End.Direction.Vector().X > 0 && Source(End.Direction).ElevationMeters < Settings.DownstreamWaterMeters
		);
		for(
			const PlanetCore::Vec3 Invalid :
			{
				PlanetCore::Vec3{ 0, 0, 0 },
				PlanetCore::Vec3{ 0, 1, 0 },
				PlanetCore::Vec3{ 0, 0, 1 },
				PlanetCore::Vec3{ std::numeric_limits<double>::quiet_NaN(), 0, 0 }
			}
		) {
			Settings.ReferenceMinimumDirection = Invalid;
			TestFalse(
				TEXT("主軸でない又は非有限な基準を拒否"),
				PlanetCore::PrepareFlowOutletRoute(Origin, Source, Settings, {}, &InvertedReport).has_value()
			);
			TestTrue(TEXT("別の物理枝へ読替えない拒否理由"), InvertedReport.Failure == PlanetCore::FlowOutletFailure::InvalidInput);
		}
		std::stop_source Stop;
		Stop.request_stop();
		Calls = 0;
		TestFalse(TEXT("軸検査でも中断済の準備を拒否"), PlanetCore::PrepareFlowOutletRoute(Origin, Source, Settings, Stop.get_token()).has_value());
		TestEqual(TEXT("中断済の元場照会は零"), Calls, UINT64_C(0));
		AddInfo(FString::Printf(TEXT("Outlet axis R=%.0f length=%.17g positionDifference=%.17g"), Radius, Length, MaximumDifference));
		++Cases;
	}
	TestEqual(TEXT("三半径を検査"), Cases, 3U);
	// 主軸の任意な符号に依らない物理枝の検査結果の返戻
	return !HasAnyErrors();
}

#endif
