#include "Core/PlanetCriticalPoint.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetCriticalPointContracts, "PlanetGenerator.CriticalPoint.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 球面上の既知の鞍点・極大・極小を３半径で求め，平坦面と不正入力を拒否する
 * @param Parameters 未使用の UE 引数
 * @return 独立な解析位置，曲率，主方向と一致する場合に真
 */
bool FPlanetCriticalPointContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::TerrainCriticalDiagnostics;
	using PlanetCore::TerrainCriticalFailure;
	using PlanetCore::TerrainCriticalKind;
	using PlanetCore::TerrainCriticalPoint;
	using PlanetCore::TerrainCriticalSettings;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	// 実装の基底とは異なる解析二次式の向き
	const UnitDirection Pole = *UnitDirection::FromVector({ 0.37, -0.21, 0.903 });
	const Vec3 Up = Pole.Vector(), East = UnitDirection::FromVector(Vec3{ 0, 1, 0 }.Cross(Up))->Vector(), North = Up.Cross(East);
	const std::array<std::array<double, 2>, 3> Coefficients { { { -0.002, 0.004 }, { 0.002, 0.004 }, { -0.004, -0.002 } } };
	double MaximumPositionError = 0, MaximumCurvatureError = 0, MaximumGradient = 0;
	std::uint32_t Cases = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) for(const std::array<double, 2> &Values : Coefficients) {
		// 元場の閉形式と独立に決まる停留点
		const auto Field = [&](const UnitDirection Direction) -> TerrainSample {
			const Vec3 Point = Direction.Vector();
			const double X = Radius * Point.Dot(East), Y = Radius * Point.Dot(North);
			const Vec3 Ambient = (East * (2 * Values[0] * X) + North * (2 * Values[1] * Y)) * Radius;
			// 球面上の解析二次式と接線勾配の返戻
			return { 1000 + Values[0] * X * X + Values[1] * Y * Y, Ambient - Point * Ambient.Dot(Point) };
		};
		TerrainCriticalSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.MaximumDisplacementMeters = 200;
		const UnitDirection Initial = *UnitDirection::FromVector(Up + (East * 27 - North * 19) * (1 / Radius));
		TerrainCriticalDiagnostics Report;
		const std::optional<TerrainCriticalPoint> Result =
		PlanetCore::RefineTerrainCriticalPoint(Initial, Field, Settings, {}, &Report);
		const std::optional<TerrainCriticalPoint> WithoutReport = PlanetCore::RefineTerrainCriticalPoint(Initial, Field, Settings);
		// 有効な局所二次式を解けない場合の返戻
		if(!TestTrue(TEXT("既知の局所停留点を発見"), Result.has_value())) return false;
		const bool HasUnchangedResult =
		WithoutReport.has_value() && WithoutReport->Kind == Result->Kind && WithoutReport->Evaluations == Result->Evaluations &&
		WithoutReport->Iterations == Result->Iterations;
		TestTrue(TEXT("記録の有無で停留点の分類・費用を変えない"), HasUnchangedResult);
		if(WithoutReport) {
			const std::array<double, 10> Recorded {
				Result->Direction.Vector().X,
				Result->Direction.Vector().Y,
				Result->Direction.Vector().Z,
				Result->ElevationMeters,
				Result->GradientNorm,
				Result->MinimumCurvaturePerMeter,
				Result->MaximumCurvaturePerMeter,
				Result->MinimumCurvatureDirection.X,
				Result->MinimumCurvatureDirection.Y,
				Result->MinimumCurvatureDirection.Z
			};
			const std::array<double, 10> Unrecorded {
				WithoutReport->Direction.Vector().X,
				WithoutReport->Direction.Vector().Y,
				WithoutReport->Direction.Vector().Z,
				WithoutReport->ElevationMeters,
				WithoutReport->GradientNorm,
				WithoutReport->MinimumCurvaturePerMeter,
				WithoutReport->MaximumCurvaturePerMeter,
				WithoutReport->MinimumCurvatureDirection.X,
				WithoutReport->MinimumCurvatureDirection.Y,
				WithoutReport->MinimumCurvatureDirection.Z
			};
			for(unsigned I = 0; I < Recorded.size(); ++I) {
				TestTrue(TEXT("記録の有無で全実数成分がビット一致"), std::bit_cast<std::uint64_t>(Recorded[I]) == std::bit_cast<std::uint64_t>(Unrecorded[I]));
			}
		}
		TestTrue(
			TEXT("成功時の元場照会と収束条件を記録"),
			Report.Failure == TerrainCriticalFailure::None && Report.Evaluations == Result->Evaluations &&
			Report.Iterations == Result->Iterations && Report.AcceptedKind == Result->Kind &&
			Report.GradientNorm <= Settings.GradientTolerance && Report.NewtonStepMeters <= Settings.PositionToleranceMeters
		);
		const double PositionError =
		Radius * std::atan2(Up.Cross(Result->Direction.Vector()).Length(), Up.Dot(Result->Direction.Vector()));
		const double CurvatureError = std::max(
			std::abs(Result->MinimumCurvaturePerMeter - 2 * Values[0]),
			std::abs(Result->MaximumCurvaturePerMeter - 2 * Values[1])
		);
		const TerrainCriticalKind Expected =
		Values[0] > 0 ? TerrainCriticalKind::Minimum : Values[1] < 0 ? TerrainCriticalKind::Maximum : TerrainCriticalKind::Saddle;
		TestTrue(TEXT("解析的な停留位置へ収束"), PositionError < 1E-4);
		TestTrue(TEXT("物理単位の曲率を再現"), CurvatureError < 1E-7);
		TestTrue(TEXT("停留点の種類を正しく分類"), Result->Kind == Expected);
		TestTrue(TEXT("固有方向を元の座標軸に依存せず復元"), std::abs(Result->MinimumCurvatureDirection.Dot(East)) > 1 - 1E-8);
		TestTrue(
			TEXT("独立な元式の勾配も指定精度内"),
			Field(Result->Direction).TangentialGradientPerRadian.Length() / Radius <= Settings.GradientTolerance
		);
		TestTrue(TEXT("元場の高さを変更しない"), std::abs(Result->ElevationMeters - 1000) < 1E-6);
		MaximumPositionError = std::max(MaximumPositionError, PositionError);
		MaximumCurvatureError = std::max(MaximumCurvatureError, CurvatureError);
		MaximumGradient = std::max(MaximumGradient, Result->GradientNorm);
		++Cases;
	}
	// 退化，非有限，範囲外，中断の拒否と正規経路の区別
	const auto Flat = [](const UnitDirection Direction) -> TerrainSample {
		static_cast<void>(Direction);
		// 一定地形の返戻
		return { 42, {} };
	};
	TerrainCriticalSettings Settings;
	TerrainCriticalDiagnostics Rejected;
	TestFalse(TEXT("停留点でない平坦領域を分類しない"), PlanetCore::RefineTerrainCriticalPoint(Pole, Flat, Settings, {}, &Rejected).has_value());
	TestTrue(
		TEXT("平坦面を曲率退化として区別"),
		Rejected.Failure == TerrainCriticalFailure::Degenerate && Rejected.Evaluations == 5 && !Rejected.AcceptedKind
	);
	TestFalse(TEXT("場が無い探索を拒否"), PlanetCore::RefineTerrainCriticalPoint(Pole, {}, Settings).has_value());
	const auto Invalid = [](const UnitDirection Direction) -> TerrainSample {
		static_cast<void>(Direction);
		// 意図した非有限値の返戻
		return { std::numeric_limits<double>::quiet_NaN(), {} };
	};
	TestFalse(TEXT("非有限な元場を拒否"), PlanetCore::RefineTerrainCriticalPoint(Pole, Invalid, Settings, {}, &Rejected).has_value());
	TestTrue(
		TEXT("非有限な照会と直前結果の初期化"),
		Rejected.Failure == TerrainCriticalFailure::InvalidSource && Rejected.Evaluations == 1 && !Rejected.Iterations &&
		!Rejected.AcceptedKind
	);
	Settings.DifferenceStepMeters = 0;
	TestFalse(TEXT("零の差分間隔を拒否"), PlanetCore::RefineTerrainCriticalPoint(Pole, Flat, Settings).has_value());
	Settings = TerrainCriticalSettings{};
	Settings.MaximumIterations = 0;
	TestFalse(TEXT("零の反復予算を拒否"), PlanetCore::RefineTerrainCriticalPoint(Pole, Flat, Settings).has_value());
	Settings = TerrainCriticalSettings{};
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断済の探索を拒否"),
		PlanetCore::RefineTerrainCriticalPoint(Pole, Flat, Settings, Cancelled.get_token(), &Rejected).has_value()
	);
	TestTrue(
		TEXT("中断後は追加照会なく取消を記録"),
		Rejected.Failure == TerrainCriticalFailure::Cancelled && !Rejected.Evaluations && !Rejected.AcceptedKind
	);
	// 有効な根の最後の曲率照会中に発生した中断の伝播
	std::stop_source MidEvaluation;
	std::uint32_t CancelCalls = 0;
	const auto CancelDuringSample = [&](const UnitDirection Direction) -> TerrainSample {
		// 極小で停止条件を満たす既知の二次式
		const Vec3 Point = Direction.Vector();
		const double X = Settings.RadiusMeters * Point.Dot(East), Y = Settings.RadiusMeters * Point.Dot(North);
		const Vec3 Ambient = (East * (0.002 * X) + North * (0.004 * Y)) * Settings.RadiusMeters;
		if(++CancelCalls == 9) MidEvaluation.request_stop();
		// 有限値でも中断済の候補は採用しない返戻
		return { 42 + 0.001 * X * X + 0.002 * Y * Y, Ambient - Point * Ambient.Dot(Point) };
	};
	TestFalse(
		TEXT("最後の曲率照会中の中断を拒否"),
		PlanetCore::RefineTerrainCriticalPoint(Pole, CancelDuringSample, Settings, MidEvaluation.get_token()).has_value()
	);
	TestEqual(TEXT("有効な停留点の最後の照会まで到達"), CancelCalls, std::uint32_t{ 9 });
	// 範囲の外にのみ停留点が有る正規な場
	const auto Distant = [&](const UnitDirection Direction) -> TerrainSample {
		const Vec3 Point = Direction.Vector();
		const double X = Settings.RadiusMeters * Point.Dot(East) - 1000, Y = Settings.RadiusMeters * Point.Dot(North);
		const Vec3 Ambient = (East * (0.002 * X) + North * (0.004 * Y)) * Settings.RadiusMeters;
		// 探索範囲より遠い解析極小の返戻
		return { 0.001 * X * X + 0.002 * Y * Y, Ambient - Point * Ambient.Dot(Point) };
	};
	Settings.MaximumDisplacementMeters = 200;
	TestFalse(TEXT("局所範囲から外れる零点を採用しない"), PlanetCore::RefineTerrainCriticalPoint(Pole, Distant, Settings).has_value());
	Settings.MaximumDisplacementMeters = 2000;
	TestTrue(TEXT("同じ正規場を十分な探索範囲で受理"), PlanetCore::RefineTerrainCriticalPoint(Pole, Distant, Settings).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Critical point contracts: cases=%u positionError=%.12g curvatureError=%.12g gradient=%.12g"),
			Cases,
			MaximumPositionError,
			MaximumCurvatureError,
			MaximumGradient
		)
	);
	// 解析解との比較結果の返戻
	return !HasAnyErrors();
}

#endif
