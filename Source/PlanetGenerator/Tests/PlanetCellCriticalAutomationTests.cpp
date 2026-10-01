#include "Core/PlanetBilinearCritical.h"
#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetCellCriticalCandidates, "PlanetGenerator.ContinuousSpill.CellCriticalCandidates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じセルで指数が相殺する二零点，境界，退化と球面窓の鎖則を検査する
 * @param Parameters 未使用の実行引数
 * @return 独立な解析位置と元場精査，三半径の予算・中断を確認した場合に真
 */
bool FPlanetCellCriticalCandidates::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	std::uint32_t Cases = 0;
	for(const double Scale : { 1E-250, 1.0, 1E250 }) for(const double Turn : { 0.0, 0.37, 1.23 }) {
		const double Cosine = std::cos(Turn), Sine = std::sin(Turn);
		std::array<std::array<double, 2>, 4> Corners {};
		for(unsigned I = 0; I < 4; ++I) {
			const double U = I % 2, V = I / 2, F = U + V - 1, G = U * V - 0.16;
			Corners[I] = { Scale * (Cosine * F - Sine * G), Scale * (Sine * F + Cosine * G) };
		}
		const PlanetCore::BilinearCriticalCandidates Roots = PlanetCore::FindBilinearCriticalCandidates(Corners);
		TestEqual(TEXT("指数の和が零でも二つの孤立零点を保持"), Roots.Count, UINT32_C(2));
		if(Roots.Count != 2) continue;
		unsigned Saddles = 0;
		for(std::uint32_t I = 0; I < Roots.Count; ++I) {
			const std::array<double, 2> &P = Roots.Points[I].Coordinates;
			const double Error = std::min(std::hypot(P[0] - 0.2, P[1] - 0.8), std::hypot(P[0] - 0.8, P[1] - 0.2));
			TestTrue(TEXT("独立な二次式の零点へ一致"), Error <= 1E-12);
			TestTrue(TEXT("独立な代入残差"), std::abs(P[0] + P[1] - 1) <= 1E-12 && std::abs(P[0] * P[1] - 0.16) <= 1E-12);
			if(Roots.Points[I].JacobianDeterminant < 0) ++Saddles;
		}
		TestEqual(TEXT("相殺する指数から鞍点一つを識別"), Saddles, 1U);
		++Cases;
	}
	const std::array<std::array<double, 2>, 4> Boundary { { { 0, -0.5 }, { 1, -0.5 }, { 0, 0.5 }, { 1, 0.5 } } };
	const PlanetCore::BilinearCriticalCandidates BoundaryRoot = PlanetCore::FindBilinearCriticalCandidates(Boundary);
	TestEqual(TEXT("セル境界上の孤立零点を保持"), BoundaryRoot.Count, UINT32_C(1));
	if(BoundaryRoot.Count) TestTrue(TEXT("境界位置を独立に確認"), BoundaryRoot.Points[0].Coordinates == std::array<double, 2>{ 0, 0.5 });
	std::array<std::array<double, 2>, 4> NoZero = Boundary;
	for(std::array<double, 2> &P : NoZero) P[0] += 2;
	TestEqual(TEXT("セル外の零点は候補にしない"), PlanetCore::FindBilinearCriticalCandidates(NoZero).Count, UINT32_C(0));
	std::array<std::array<double, 2>, 4> Degenerate {};
	for(unsigned I = 0; I < 4; ++I) Degenerate[I] = { static_cast<double>(I % 2) - 0.5, 2 * (static_cast<double>(I % 2) - 0.5) };
	const PlanetCore::BilinearCriticalCandidates DegenerateRoots = PlanetCore::FindBilinearCriticalCandidates(Degenerate);
	TestTrue(TEXT("零点曲線を孤立点へ置換えない"), !DegenerateRoots.Count && DegenerateRoots.HasDegenerateElimination);
	NoZero[0][0] = std::numeric_limits<double>::quiet_NaN();
	TestEqual(TEXT("非有限入力から零点を捏造しない"), PlanetCore::FindBilinearCriticalCandidates(NoZero).Count, UINT32_C(0));
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto At = [Radius](const double X, const double Y) -> PlanetCore::UnitDirection {
			// 独立な平面座標から正規化する球面方向の返戻
			return *PlanetCore::UnitDirection::FromVector({ X / Radius, Y / Radius, 1 });
		};
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z - 27, Y = Radius * D.Y / D.Z + 16, DX = 0.0002 * X, DY = -0.0002 * Y;
			const PlanetCore::Vec3 G { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 格子頂点に一致しない解析鞍点の標高と微分の返戻
			return { 1000 + 0.0001 * (X * X - Y * Y), G };
		};
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.HalfWidthMeters = 150;
		Settings.SpacingMeters = 50;
		Settings.UsePathEdgeSelection = true;
		const std::optional<PlanetCore::SpillWindowExit> OldWindow = PlanetCore::FindSpillWindowExit(At(0, 0), Source, Settings);
		// 同じ実辺を測る比較用窓の欠落への返戻
		if(!TestTrue(TEXT("元の標高候補窓を準備"), OldWindow.has_value())) return false;
		Calls = 0;
		Settings.UseCellCriticalCandidates = true;
		const std::optional<PlanetCore::SpillWindowExit> Window = PlanetCore::FindSpillWindowExit(At(0, 0), Source, Settings);
		// 元場で未確認の補間根を完成した窓として返さない返戻
		if(!TestTrue(TEXT("解析勾配の窓候補を保持"), Window && Window->CellCriticalCandidates > 0)) return false;
		TestEqual(TEXT("候補の消去計算は追加の元場を呼出さない"), Window->Evaluations, OldWindow->Evaluations);
		TestEqual(TEXT("実辺と窓標本の費用を漏らさない"), Calls, Window->Evaluations);
		const PlanetCore::UnitDirection Expected = At(27, -16);
		double Best = std::numeric_limits<double>::infinity();
		PlanetCore::UnitDirection Closest = Window->PassCandidates.front();
		for(const PlanetCore::UnitDirection Candidate : Window->PassCandidates) {
			const PlanetCore::Vec3 A = Candidate.Vector(), B = Expected.Vector();
			const double Error = Radius * std::atan2(A.Cross(B).Length(), A.Dot(B));
			if(Error < Best) {
				Best = Error;
				Closest = Candidate;
			}
		}
		TestTrue(TEXT("球面窓の鎖則も独立な鞍点位置へ一致"), Best <= 1E-8);
		PlanetCore::TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Radius;
		Critical.MaximumDisplacementMeters = std::min(Critical.MaximumDisplacementMeters, Radius * 0.125);
		const std::optional<PlanetCore::TerrainCriticalPoint> Pass = PlanetCore::RefineTerrainCriticalPoint(Closest, Source, Critical);
		TestTrue(
			TEXT("補間根を元の解析鞍点で再確認"),
			Pass && Pass->Kind == PlanetCore::TerrainCriticalKind::Saddle && std::abs(Pass->ElevationMeters - 1000) <= 1E-6 &&
			Pass->GradientNorm <= Critical.GradientTolerance
		);
		AddInfo(
			FString::Printf(
				TEXT("Cell candidates R=%.0f roots=%u error=%.17g unchangedSource=%llu"),
				Radius,
				Window->CellCriticalCandidates,
				Best,
				static_cast<unsigned long long>(Window->Evaluations)
			)
		);
		Settings.MaximumEvaluations = 10;
		Calls = 0;
		TestFalse(TEXT("同じ窓の共通予算を超えて公開しない"), PlanetCore::FindSpillWindowExit(At(0, 0), Source, Settings).has_value());
		TestEqual(TEXT("元場呼出は指定予算で停止"), Calls, UINT64_C(10));
		Settings.MaximumEvaluations = 4000000;
		std::stop_source Stop;
		Stop.request_stop();
		Calls = 0;
		TestFalse(TEXT("中断したセル候補を公開しない"), PlanetCore::FindSpillWindowExit(At(0, 0), Source, Settings, Stop.get_token()).has_value());
		TestEqual(TEXT("中断後の元場呼出は零"), Calls, UINT64_C(0));
		++Cases;
	}
	AddInfo(FString::Printf(TEXT("Cell critical candidate cases=%u"), Cases));
	// 独立解析根と正規の元場経路を検査した結果の返戻
	return !HasAnyErrors();
}

#endif
