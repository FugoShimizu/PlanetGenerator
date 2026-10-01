#include "Core/PlanetLakeBoundary.h"
#include "Core/PlanetLakeSill.h"
#include "Core/PlanetSpillLake.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLakeSillContracts,
	"PlanetGenerator.LakeTopology.FiniteSill",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 有限幅の通水，補正外の不変性，解析勾配と境界の二階連続性を検査する
 * @param Parameters 未使用の自動試験引数
 * @return 解析湖の連結・岸・補正範囲・微分・正規入力と拒否条件が成立した場合に真
 */
bool FPlanetLakeSillContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::LakeSillPatch;
	using PlanetCore::LakeSillSettings;
	using PlanetCore::TerrainContour;
	using PlanetCore::TerrainCriticalPoint;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	constexpr double Length = 80, Amplitude = 20;
	std::uint32_t Cases = 0, Derivatives = 0, WetSamples = 0;
	double MaximumGradientError = 0, MaximumSlope = 0, MinimumDepth = std::numeric_limits<double>::infinity();
	for(const double Radius : { 1000.0, 1E6, 1E9 }) for(const bool Rotated : { false, true }) {
		const Vec3 Up = Rotated ? UnitDirection::FromVector({ -0.2, 0.3, 0.8 })->Vector() : Vec3{ 0, 0, 1 };
		const Vec3 East = UnitDirection::FromVector(Vec3{ 0, 1, 0 }.Cross(Up))->Vector(), North = Up.Cross(East);
		const auto At = [&](const double X, const double Y) -> UnitDirection {
			// 共通の解析座標を球面へ写す返戻
			return *UnitDirection::FromVector(Up + East * (X / Radius) + North * (Y / Radius));
		};
		const auto Source = [&](const UnitDirection Direction) -> TerrainSample {
			const Vec3 D = Direction.Vector();
			const double Z = D.Dot(Up), U = D.Dot(East), V = D.Dot(North), X = Radius * U / Z, Y = Radius * V / Z;
			const double Angle = std::numbers::pi * X / Length, Sine = std::sin(Angle), Cosine = std::cos(Angle);
			const Vec3 DX = (East * Z - Up * U) * (Radius / (Z * Z)), DY = (North * Z - Up * V) * (Radius / (Z * Z));
			// 同水位で点接触する二つの解析湖の返戻
			return {
				Y * Y / (Amplitude * Amplitude) - Sine * Sine,
				DX * (-2 * Sine * Cosine * std::numbers::pi / Length) + DY * (2 * Y / (Amplitude * Amplitude))
			};
		};
		std::vector<TerrainCriticalPoint> Passes;
		for(unsigned Index = 0; Index < 3; ++Index) {
			const double X = Length * Index, Scale = std::sqrt(1 + X * X / (Radius * Radius));
			TerrainCriticalPoint Pass { At(X, 0) };
			Pass.MinimumCurvatureDirection = East - Pass.Direction.Vector() * East.Dot(Pass.Direction.Vector());
			Pass.MinimumCurvaturePerMeter = -2 * std::numbers::pi * std::numbers::pi / (Length * Length) * std::pow(Scale, 4);
			Pass.MaximumCurvaturePerMeter = 2 / (Amplitude * Amplitude) * Scale * Scale;
			Passes.push_back(Pass);
		}
		LakeSillSettings Definition;
		Definition.RadiusMeters = Radius;
		const std::optional<LakeSillPatch> Patch = LakeSillPatch::Create(Passes[1], Definition);
		// 小さな正規の補正を準備出来ない場合の返戻
		if(!TestTrue(TEXT("鞍点の局所的な開削を準備"), Patch.has_value())) return false;
		const Vec3 Origin = Passes[1].Direction.Vector();
		const Vec3 Along = UnitDirection::FromVector(Passes[1].MinimumCurvatureDirection)->Vector();
		const Vec3 Across = Origin.Cross(Along);
		const auto Local = [&](const double X, const double Y) -> UnitDirection {
			// 補正自身の局所射影で指定した位置の返戻
			return *UnitDirection::FromVector(Origin + Along * (X / Radius) + Across * (Y / Radius));
		};
		const auto Combined = [&](const UnitDirection Direction) -> TerrainSample {
			const TerrainSample Base = Source(Direction), Delta = Patch->EvaluateCorrection(Direction);
			// 不変な元場と，静的な局所開削を合成した値の返戻
			return { Base.ElevationMeters + Delta.ElevationMeters, Base.TangentialGradientPerRadian + Delta.TangentialGradientPerRadian };
		};
		TestTrue(TEXT("中心の開削量は指定した２５センチ"), std::abs(Patch->EvaluateCorrection(Passes[1].Direction).ElevationMeters + 0.25) < 1E-12);
		TestTrue(TEXT("元の鞍点の水深は零"), std::abs(Source(Passes[1].Direction).ElevationMeters) < 1E-12);
		for(int X = -15; X < 16; ++X) for(int Y = -15; Y < 16; ++Y) {
			const UnitDirection Point = Local(X, Y);
			const TerrainSample Delta = Patch->EvaluateCorrection(Point);
			TestTrue(TEXT("持上げ又は指定値を越える開削をしない"), !(Delta.ElevationMeters > 0) && Delta.ElevationMeters >= -Definition.DepthMeters);
			const double Slope = Delta.TangentialGradientPerRadian.Length() / Radius;
			MaximumSlope = std::max(MaximumSlope, Slope);
			TestTrue(TEXT("勾配は接平面に在る"), std::abs(Delta.TangentialGradientPerRadian.Dot(Point.Vector())) / Radius < 1E-12);
			if(X * X + Y * Y > 145) {
				TestEqual(TEXT("支持範囲外は標高を厳密に変更しない"), Delta.ElevationMeters, 0.0);
				TestEqual(TEXT("支持範囲外は法線を厳密に変更しない"), Slope, 0.0);
			}
			for(const Vec3 Axis : { Along, Across }) {
				const Vec3 Tangent = UnitDirection::FromVector(Axis - Point.Vector() * Axis.Dot(Point.Vector()))->Vector();
				const auto Difference = [&](const double Step) -> double {
					const UnitDirection A = *UnitDirection::FromVector(Point.Vector() + Tangent * (Step / Radius));
					const UnitDirection B = *UnitDirection::FromVector(Point.Vector() - Tangent * (Step / Radius));
					// 解析勾配を使わない中央差分の返戻
					return (Patch->EvaluateCorrection(A).ElevationMeters - Patch->EvaluateCorrection(B).ElevationMeters) / (2 * Step);
				};
				const double Independent = (4 * Difference(0.025) - Difference(0.05)) / 3;
				const double Expected = Delta.TangentialGradientPerRadian.Dot(Tangent) / Radius;
				MaximumGradientError = std::max(MaximumGradientError, std::abs(Expected - Independent));
				++Derivatives;
			}
		}
		const TerrainSample Opposite = Patch->EvaluateCorrection(*UnitDirection::FromVector(Origin * -1));
		TestTrue(TEXT("反対側の半球へ補正を複製しない"), !Opposite.ElevationMeters && !Opposite.TangentialGradientPerRadian.Length());
		double PreviousCurvature = std::numeric_limits<double>::infinity();
		for(const double Step : { 0.08, 0.04, 0.02, 0.01 }) {
			const TerrainSample Inner = Patch->EvaluateCorrection(Local(Definition.HalfLengthMeters - Step, 0));
			const double CurvatureBound = Inner.TangentialGradientPerRadian.Length() / Radius / Step;
			TestTrue(TEXT("境界へ近付く二階微分が零へ収束"), CurvatureBound < PreviousCurvature * 0.6);
			PreviousCurvature = CurvatureBound;
		}
		for(unsigned X = 0; X < 65; ++X) for(unsigned Y = 0; Y < 9; ++Y) {
			const UnitDirection Point = At(64 + X * 0.5, -0.5 + Y * 0.125);
			MinimumDepth = std::min(MinimumDepth, -Combined(Point).ElevationMeters);
			++WetSamples;
		}
		// 外側の流出口を保持し，内部の鞍点を岸の終点から除いて一周する
		const std::array<TerrainCriticalPoint, 2> Outlets { Passes[0], Passes[2] };
		PlanetCore::TerrainContourSettings ShoreSettings;
		ShoreSettings.RadiusMeters = Radius;
		ShoreSettings.ChordToleranceMeters = ShoreSettings.TerminalRadiusMeters = 0.02;
		ShoreSettings.MaximumStepMeters = 20;
		ShoreSettings.MaximumLengthMeters = 1000;
		ShoreSettings.Terminals = { Outlets[0].Direction, Outlets[1].Direction };
		std::vector<TerrainContour> Arcs;
		for(unsigned Index = 0; Index < 2; ++Index) {
			std::optional<TerrainContour> Arc = PlanetCore::TraceSpillLakeArc(Outlets[Index], !Index, Combined, ShoreSettings);
			// 開削後の岸を追跡出来ない部分結果への返戻
			if(!TestTrue(TEXT("開削後の連結湖の外岸を準備"), Arc.has_value())) return false;
			TestTrue(TEXT("別の外部流出口まで岸が連続"), Arc->TerminalIndex == 1 - Index);
			Arcs.push_back(std::move(*Arc));
			TestEqual(TEXT("外部流出口の水位を変更しない"), Patch->EvaluateCorrection(Outlets[Index].Direction).ElevationMeters, 0.0);
		}
		const std::optional<std::vector<PlanetCore::LakeBoundaryLoop>> Boundary =
		PlanetCore::AssembleLakeBoundary(Outlets, Arcs, ShoreSettings);
		TestTrue(TEXT("二つの零水深接触の輪郭が一つの外岸に成る"), Boundary && Boundary->size() == 1);
		if(Boundary) {
			TestTrue(TEXT("連結した湖の外岸は正の向き"), Boundary->front().SignedAreaMetersSquared > 0);
			AddInfo(
				FString::Printf(
					TEXT("Finite sill R=%.0f rotated=%u points=%u area=%.12g"),
					Radius,
					static_cast<unsigned>(Rotated),
					static_cast<unsigned>(Boundary->front().Contour.Samples.size()),
					Boundary->front().SignedAreaMetersSquared
				)
			);
		}
		LakeSillSettings Invalid = Definition;
		Invalid.DepthMeters = 2.1;
		TestFalse(TEXT("開削深さの上限を超える補正を拒否"), LakeSillPatch::Create(Passes[1], Invalid).has_value());
		Invalid = Definition;
		Invalid.HalfWidthMeters = 0.2;
		TestFalse(TEXT("急過ぎる狭い開削を拒否"), LakeSillPatch::Create(Passes[1], Invalid).has_value());
		Invalid = Definition;
		Invalid.RadiusMeters = 999;
		TestFalse(TEXT("半径の下限未満を拒否"), LakeSillPatch::Create(Passes[1], Invalid).has_value());
		Invalid = Definition;
		Invalid.DepthMeters = std::numeric_limits<double>::quiet_NaN();
		TestFalse(TEXT("非有限な開削量を拒否"), LakeSillPatch::Create(Passes[1], Invalid).has_value());
		++Cases;
	}
	TestTrue(TEXT("独立な勾配差は１０の負５乗未満"), MaximumGradientError < 1E-5);
	TestTrue(TEXT("追加の物理勾配は０．０４未満"), MaximumSlope < 0.04);
	TestTrue(TEXT("幅１メートルの通水部の有限標本は水深１０センチ以上"), MinimumDepth > 0.1);
	AddInfo(
		FString::Printf(
			TEXT("Finite sill contracts: cases=%u derivatives=%u wetSamples=%u gradientError=%.12g slope=%.12g minDepth=%.12g"),
			Cases,
			Derivatives,
			WetSamples,
			MaximumGradientError,
			MaximumSlope,
			MinimumDepth
		)
	);
	// 有限の解析例での通水と補正品質の検査結果の返戻
	return !HasAnyErrors();
}

#endif
