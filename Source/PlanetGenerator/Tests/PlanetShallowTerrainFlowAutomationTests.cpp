#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetDescentPath.h"
#include "Core/PlanetFlowCurve.h"
#include "Core/PlanetFlowOutlet.h"
#include "Core/PlanetSpillSeed.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetShallowLocationSeed, "PlanetGenerator.ContinuousSpill.ShallowLocationSeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 小さい勾配でも極小まで５㎝残る独立な場で，範囲を緩和せず再追跡する
 * @param Parameters 未使用の実行引数
 * @return 三半径で実極小・実出口と同じ予算内の下降継続を確認した場合に真
 */
bool FPlanetShallowLocationSeed::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z;
			constexpr double Scale = 0.000025;
			const double DX = Scale * (-0.1 * std::sin(X / 40) + 0.02), DY = Scale * Y / 200;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 既知の位置を持つ極小・鞍点と，其の同じ元場の解析勾配の返戻
			return { Scale * (4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400), Gradient };
		};
		const double MinimumX = 40 * (5 * std::acos(-1.0) - std::asin(0.2));
		const PlanetCore::Vec3 MinimumDirection = PlanetCore::UnitDirection::FromVector({ MinimumX / Radius, 0, 1 })->Vector();
		const PlanetCore::Vec3 East = PlanetCore::UnitDirection::FromVector({ 1, 0, -MinimumX / Radius })->Vector();
		const PlanetCore::UnitDirection Start =
		*PlanetCore::UnitDirection::FromVector(MinimumDirection * std::cos(0.05 / Radius) + East * std::sin(0.05 / Radius));
		PlanetCore::TerrainDescentSettings Previous;
		Previous.RadiusMeters = Radius;
		Previous.StopElevationMeters = -1;
		Previous.GradientTolerance = 1E-8;
		Previous.MaximumLengthMeters = std::min(64000.0, Radius * 0.5);
		const std::optional<PlanetCore::TerrainDescentPath> OldPath = PlanetCore::TraceTerrainDescent(Start, Source, Previous);
		AddInfo(
			FString::Printf(
				TEXT("Shallow seed old path: radius=%.0f settings=%d gradient=%.17g path=%d"),
				Radius,
				static_cast<int>(Previous.IsValid()),
				Source(Start).TangentialGradientPerRadian.Length() / Radius,
				static_cast<int>(OldPath.has_value())
			)
		);
		const bool StoppedEarly =
		OldPath && OldPath->Samples.size() == 1 && OldPath->End == PlanetCore::TerrainDescentEnd::GradientLimit;
		// 独立に旧失敗を再現出来ない場合の返戻
		if(!TestTrue(TEXT("同じ閾値で５㎝手前に停止する旧条件"), StoppedEarly)) return false;
		PlanetCore::TerrainCriticalSettings Local;
		Local.RadiusMeters = Radius;
		Local.DifferenceStepMeters = 0.005;
		Local.MaximumStepMeters = 0.01;
		Local.MaximumDisplacementMeters = 0.01;
		PlanetCore::TerrainCriticalDiagnostics OldReport;
		TestFalse(
			TEXT("１㎝の精査だけでは５㎝先の極小を受理しない"),
			PlanetCore::RefineTerrainCriticalPoint(Start, Source, Local, {}, &OldReport).has_value()
		);
		TestTrue(
			TEXT("旧失敗は正の曲率を持つ範囲不足"),
			OldReport.Failure == PlanetCore::TerrainCriticalFailure::Range && OldReport.MinimumCurvaturePerMeter > 0
		);
		PlanetCore::SpillSeedSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -1;
		PlanetCore::SpillSeedDiagnostics Report;
		Calls = 0;
		const std::optional<PlanetCore::SpillSeed> Seed = PlanetCore::FindSpillSeed(Start, Source, Settings, {}, &Report);
		AddInfo(
			FString::Printf(
				TEXT("Shallow seed radius=%.0f accepted=%d failure=%u continuations=%u calls=%llu"),
				Radius,
				static_cast<int>(Seed.has_value()),
				static_cast<unsigned>(Report.Failure),
				Report.Continuations,
				Calls
			)
		);
		// 元場の実流出口を得られない場合の返戻
		if(!TestTrue(TEXT("許容範囲を維持して下降し実出口を発見"), Seed.has_value())) return false;
		TestEqual(TEXT("再追跡も共通の実呼出回数に含める"), Report.Evaluations, Calls);
		TestTrue(
			TEXT("旧拒否を保持したまま正規に再追跡"),
			Report.Continuations > 0 && Report.InitialCritical.Failure == PlanetCore::TerrainCriticalFailure::Range &&
			Report.Critical.Failure == PlanetCore::TerrainCriticalFailure::None
		);
		const PlanetCore::Vec3 D = Seed->Minimum.Direction.Vector();
		TestTrue(TEXT("解析的な極小位置へ収束"), std::abs(Radius * D.X / D.Z - MinimumX) < Local.PositionToleranceMeters);
		TestTrue(TEXT("元の精査範囲を拡大しない"), Report.Critical.DisplacementMeters <= Local.MaximumDisplacementMeters);
		Settings.MaximumEvaluations = 25;
		Calls = 0;
		TestFalse(TEXT("再追跡でも共通予算不足を成功にしない"), PlanetCore::FindSpillSeed(Start, Source, Settings, {}, &Report).has_value());
		TestTrue(
			TEXT("再追跡に別の評価枠を追加しない"),
			Report.Failure == PlanetCore::SpillSeedFailure::BudgetLimit && Calls == 25 && Report.Evaluations == Calls
		);
	}
	// 三半径の実接続と正規な予算拒否の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetShallowFlowDeparture, "PlanetGenerator.FlowChannel.ShallowDeparture",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 鞍点直後の小さい勾配を停止点と混同せず，受入水位まで辿る
 * @param Parameters 未使用の実行引数
 * @return 三半径・二つの曲率で正規な下降と到達不能の拒否を両方確認した場合に真
 */
bool FPlanetShallowFlowDeparture::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Pass = *PlanetCore::UnitDirection::FromVector({ 1, 0, 0 });
	for(const double Radius : { 1000.0, 1E6, 1E9 }) for(const double Scale : { 0.001, 0.00001 }) {
		const auto Source = [Radius, Scale](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.Y, Y = Radius * D.Z, Squared = X * X;
			const double Height = 100 + Scale * (-0.00005 * Squared + 6.25E-8 * Squared * Squared + 0.0001 * Y * Y);
			const PlanetCore::Vec3 Gradient { 0, Scale * Radius * X * (-0.0001 + 2.5E-7 * Squared), Scale * Radius * 0.0002 * Y };
			// 鞍点１００ｍと，独立に分かる極小１００－０．０１ Scale の返戻
			return { Height, Gradient - D * Gradient.Dot(D) };
		};
		const PlanetCore::UnitDirection Departure = *PlanetCore::UnitDirection::FromVector({ 1, -1 / Radius, 0 });
		PlanetCore::TerrainDescentSettings Previous;
		Previous.RadiusMeters = Radius;
		Previous.StopElevationMeters = -std::numeric_limits<double>::max();
		Previous.MaximumLengthMeters = 10;
		Previous.MaximumStepMeters = 10;
		const std::optional<PlanetCore::TerrainDescentPath> OldPath = PlanetCore::TraceTerrainDescent(Departure, Source, Previous);
		TestTrue(
			TEXT("鞍点直後を停止する旧条件を独立に再現"),
			OldPath && OldPath->Samples.size() == 1 && OldPath->End == PlanetCore::TerrainDescentEnd::GradientLimit
		);
		PlanetCore::FlowOutletSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.UpstreamLengthMeters = 10;
		Settings.MaximumDownstreamLengthMeters = 50;
		Settings.DownstreamWaterMeters = 100 - 0.005 * Scale;
		PlanetCore::FlowOutletDiagnostics Report;
		const std::optional<PlanetCore::FlowOutletRoute> Route =
		PlanetCore::PrepareFlowOutletRoute(Pass, Source, Settings, {}, &Report);
		AddInfo(
			FString::Printf(
				TEXT("Shallow departure radius=%.0f scale=%.9g accepted=%d failure=%u vertices=(%u,%u)"),
				Radius,
				Scale,
				static_cast<int>(Route.has_value()),
				static_cast<unsigned>(Report.Failure),
				Report.UpstreamVertices,
				Report.DownstreamVertices
			)
		);
		// 正規な下流水位を確認出来ない場合の返戻
		if(!TestTrue(TEXT("小さい勾配でも正規の受入水位へ到達"), Route.has_value())) return false;
		TestTrue(
			TEXT("開始直後を終端とせず実際に両枝を辿る"),
			Report.UpstreamVertices > 1 && Report.DownstreamVertices > 1 &&
			Report.UpstreamEnd == PlanetCore::TerrainDescentEnd::LengthLimit &&
			Report.DownstreamEnd == PlanetCore::TerrainDescentEnd::HeightLimit
		);
		const std::optional<PlanetCore::TerrainFlowCurveSample> End = Route->Route.Sample(Route->Route.GetLengthMeters());
		// 公開曲線の末尾欠落への返戻
		if(!TestTrue(TEXT("公開する曲線の末尾"), End.has_value())) return false;
		TestTrue(TEXT("実際の元地形が受入水面以下"), Source(End->Direction).ElevationMeters <= Settings.DownstreamWaterMeters);
		Settings.DownstreamWaterMeters = 100 - 0.02 * Scale;
		TestFalse(TEXT("存在しない深い水位には届いたとしない"), PlanetCore::PrepareFlowOutletRoute(Pass, Source, Settings, {}, &Report).has_value());
		TestTrue(
			TEXT("未到達の下降終端を成功にしない"),
			Report.Failure == PlanetCore::FlowOutletFailure::Downstream &&
			Report.DownstreamEnd != PlanetCore::TerrainDescentEnd::HeightLimit
		);
	}
	// 実到達と未到達を区別した結果の返戻
	return !HasAnyErrors();
}

#endif
