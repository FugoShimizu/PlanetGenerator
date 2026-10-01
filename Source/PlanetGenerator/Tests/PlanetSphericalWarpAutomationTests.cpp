#include "Core/PlanetSphericalWarp.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalWarpContracts, "PlanetGenerator.SphericalWarp.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 球面写像の逆・面積・解析微分を独立な差分と照合する
 * @param Parameters 未使用の UE 引数
 * @return 制御入力と有限精度の検査が成立した場合に真，景観の受入れとは別
 */
bool FPlanetSphericalWarpContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetSphericalWarp;
	using PlanetCore::SphericalWarpMapping;
	using PlanetCore::SphericalWarpSettings;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	// 独立差分と解析比較の準備
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const auto Height = [](const UnitDirection Direction) -> double {
		const Vec3 P = Direction.Vector();
		// 解析勾配を別に記述出来る制御地形の返戻
		return 17 * P.X + 29 * P.Y - 31 * P.Z + 5 * P.X * P.Y;
	};
	double MaximumInverse = 0, MaximumArea = 0, MaximumNumericArea = 0, MaximumDerivative = 0, MaximumGradient = 0;
	double MaximumMappedDifference = 0, MaximumRadial = 0, MaximumStretch = 0, MaximumContinentalDisplacement = 0;
	double MaximumInverseDerivative = 0, MaximumInverseMappedDifference = 0, MaximumJacobianRoundtrip = 0;
	double MaximumContinentalJacobianRoundtrip = 0, MaximumScaledJacobianRoundtrip = 0;
	bool IsWorstContinental = false;
	unsigned WorstIndex = 0, WorstAxis = 0;
	double WorstRadius = 0, WorstFrequency = 0, WorstPositionRoundtrip = 0;
	Vec3 WorstDirection;
	std::uint64_t Samples = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		for(const bool ShouldUseContinentalWavelengths : { false, true }) {
			SphericalWarpSettings Settings;
			Settings.RadiusMeters = Radius;
			Settings.ShouldUseContinentalWavelengths = ShouldUseContinentalWavelengths;
			if(ShouldUseContinentalWavelengths) {
				Settings.LargestWavelengthMeters = 6.283185307179586 * Radius;
				Settings.DerivativeBudget = 8;
			}
			const std::optional<PlanetSphericalWarp> Warp = PlanetSphericalWarp::Create(Settings);
			// 正規入力の準備に失敗した場合の返戻
			if(!TestTrue(TEXT("対応する半径で準備"), Warp.has_value())) return false;
			const double Scale = ShouldUseContinentalWavelengths ? std::max(1.0, Settings.DerivativeBudget * Settings.DerivativeBudget) : 1;
			const double Step = std::min(1E-5, 0.002 / (Warp->GetMaximumFrequency() * Scale));
			double CaseJacobianRoundtrip = 0;
			// 球面方向の ulp は位相へ最大周波数倍で伝わる為，短波長の比較には其の尺度も保持する
			const double PhaseSensitivity =
			std::max(1.0, Warp->GetMaximumFrequency()) * (1 + Settings.DerivativeBudget) * (1 + Settings.DerivativeBudget);
			for(unsigned Index = 0; Index < 4102; ++Index) {
				Vec3 P;
				if(Index < 6) {
					constexpr std::array<Vec3, 6> Axes {
						Vec3{ 1, 0, 0 },
						Vec3{ -1, 0, 0 },
						Vec3{ 0, 1, 0 },
						Vec3{ 0, -1, 0 },
						Vec3{ 0, 0, 1 },
						Vec3{ 0, 0, -1 }
					};
					P = Axes[Index];
				} else {
					const double Z = 1 - 2 * (Index - 6 + 0.5) / 4096, Angle = (Index - 6) * 2.399963229728653, Ring = std::sqrt(1 - Z * Z);
					P = { Ring * std::cos(Angle), Ring * std::sin(Angle), Z };
				}
				const UnitDirection Direction = *UnitDirection::FromVector(P);
				P = Direction.Vector();
				const SphericalWarpMapping Mapped = Warp->MapWithDerivatives(Direction);
				const Vec3 Q = Mapped.Direction.Vector();
				if(ShouldUseContinentalWavelengths) MaximumContinentalDisplacement = std::max(MaximumContinentalDisplacement, (Q - P).Length());
				MaximumInverse = std::max(MaximumInverse, (Warp->ToWorld(Mapped.Direction).Vector() - P).Length());
				MaximumInverse = std::max(MaximumInverse, (Warp->ToSource(Warp->ToWorld(Direction)).Vector() - P).Length());
				MaximumMappedDifference = std::max(MaximumMappedDifference, (Warp->ToSource(Direction).Vector() - Q).Length());
				const Vec3 First = UnitDirection::FromVector((std::abs(P.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 }).Cross(P))->Vector();
				const Vec3 Second = P.Cross(First);
				const SphericalWarpMapping Inverse = Warp->MapToWorldWithDerivatives(Direction);
				const SphericalWarpMapping ForwardAgain = Warp->MapWithDerivatives(Inverse.Direction);
				MaximumInverseMappedDifference =
				std::max(MaximumInverseMappedDifference, (Inverse.Direction.Vector() - Warp->ToWorld(Direction).Vector()).Length());
				std::array<Vec3, 2> Actual, Numeric;
				for(unsigned Axis = 0; Axis < 2; ++Axis) {
					const Vec3 Tangent = Axis ? Second : First;
					const Vec3 InverseActual =
					Inverse.Derivatives[0] * Tangent.X + Inverse.Derivatives[1] * Tangent.Y + Inverse.Derivatives[2] * Tangent.Z;
					const auto InverseDifferenceAt = [&](const double Offset) -> Vec3 {
						const UnitDirection Before = *UnitDirection::FromVector(P * std::cos(Offset) - Tangent * std::sin(Offset));
						const UnitDirection After = *UnitDirection::FromVector(P * std::cos(Offset) + Tangent * std::sin(Offset));
						// 逆方向の値だけを使う独立な大円差分の返戻
						return (Warp->ToWorld(After).Vector() - Warp->ToWorld(Before).Vector()) * (0.5 / Offset);
					};
					const Vec3 InverseNumeric = (InverseDifferenceAt(Step * 0.5) * 4 - InverseDifferenceAt(Step)) * (1.0 / 3);
					MaximumInverseDerivative = std::max(MaximumInverseDerivative, (InverseActual - InverseNumeric).Length());
					const Vec3 Roundtrip = ForwardAgain.Derivatives[0] * InverseActual.X + ForwardAgain.Derivatives[1] * InverseActual.Y +
					ForwardAgain.Derivatives[2] * InverseActual.Z;
					const double RoundtripError = (Roundtrip - Tangent).Length();
					CaseJacobianRoundtrip = std::max(CaseJacobianRoundtrip, RoundtripError);
					MaximumScaledJacobianRoundtrip = std::max(MaximumScaledJacobianRoundtrip, RoundtripError / PhaseSensitivity);
					if(ShouldUseContinentalWavelengths) {
						MaximumContinentalJacobianRoundtrip = std::max(MaximumContinentalJacobianRoundtrip, RoundtripError);
					}
					if(RoundtripError > MaximumJacobianRoundtrip) {
						MaximumJacobianRoundtrip = RoundtripError;
						WorstRadius = Radius;
						WorstFrequency = Warp->GetMaximumFrequency();
						WorstIndex = Index;
						WorstAxis = Axis;
						IsWorstContinental = ShouldUseContinentalWavelengths;
						WorstPositionRoundtrip = (ForwardAgain.Direction.Vector() - P).Length();
						WorstDirection = P;
					}
					Actual[Axis] = Mapped.Derivatives[0] * Tangent.X + Mapped.Derivatives[1] * Tangent.Y + Mapped.Derivatives[2] * Tangent.Z;
					const auto DifferenceAt = [&](const double Offset) -> Vec3 {
						const UnitDirection Before = *UnitDirection::FromVector(P * std::cos(Offset) - Tangent * std::sin(Offset));
						const UnitDirection After = *UnitDirection::FromVector(P * std::cos(Offset) + Tangent * std::sin(Offset));
						// 解析微分を使わない球面大円上の独立差分の返戻
						return (Warp->ToSource(After).Vector() - Warp->ToSource(Before).Vector()) * (0.5 / Offset);
					};
					Numeric[Axis] = (DifferenceAt(Step * 0.5) * 4 - DifferenceAt(Step)) * (1.0 / 3);
					MaximumDerivative = std::max(MaximumDerivative, (Actual[Axis] - Numeric[Axis]).Length());
					MaximumStretch = std::max(MaximumStretch, Actual[Axis].Length());
					const auto HeightDifferenceAt = [&](const double Offset) -> double {
						const UnitDirection Before = *UnitDirection::FromVector(P * std::cos(Offset) - Tangent * std::sin(Offset));
						const UnitDirection After = *UnitDirection::FromVector(P * std::cos(Offset) + Tangent * std::sin(Offset));
						// 座標変形した高さだけから得た独立差分の返戻
						return (Height(Warp->ToSource(After)) - Height(Warp->ToSource(Before))) / (2 * Offset);
					};
					const double NumericGradient = (4 * HeightDifferenceAt(Step * 0.5) - HeightDifferenceAt(Step)) / 3;
					const Vec3 SourceGradient { 17 + 5 * Q.Y, 29 + 5 * Q.X, -31 };
					const Vec3 Gradient = Mapped.PullbackGradient(SourceGradient);
					MaximumGradient = std::max(MaximumGradient, std::abs(Gradient.Dot(Tangent) - NumericGradient));
					MaximumRadial = std::max(MaximumRadial, std::abs(P.Dot(Gradient)));
				}
				MaximumArea = std::max(MaximumArea, std::abs(Q.Dot(Actual[0].Cross(Actual[1])) - 1));
				MaximumNumericArea = std::max(MaximumNumericArea, std::abs(Q.Dot(Numeric[0].Cross(Numeric[1])) - 1));
				++Samples;
			}
			AddInfo(
				FString::Printf(
					TEXT("Inverse Jacobian scale: radius=%.17g continental=%u frequency=%.17g roundtrip=%.17g phaseScaled=%.17g."),
					Radius,
					static_cast<unsigned>(ShouldUseContinentalWavelengths),
					Warp->GetMaximumFrequency(),
					CaseJacobianRoundtrip,
					CaseJacobianRoundtrip / PhaseSensitivity
				)
			);
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Worst inverse Jacobian: radius=%.17g continental=%u frequency=%.17g index=%u axis=%u position=%.17g ")
			TEXT("direction=(%.17g,%.17g,%.17g) jacobian=%.17g."),
			WorstRadius,
			static_cast<unsigned>(IsWorstContinental),
			WorstFrequency,
			WorstIndex,
			WorstAxis,
			WorstPositionRoundtrip,
			WorstDirection.X,
			WorstDirection.Y,
			WorstDirection.Z,
			MaximumJacobianRoundtrip
		)
	);
	TestTrue(TEXT("逆微分の有無でも同じ実惑星方向を返す"), MaximumInverseMappedDifference < 1E-14);
	TestTrue(TEXT("逆写像の解析微分が独立差分と一致"), MaximumInverseDerivative < 1E-5);
	// 比較用の大陸では元の絶対基準を保持し，従来のメートル波長では位相の丸め尺度も検査する
	TestTrue(TEXT("全半径の大陸写像の接線往復は元の絶対基準内"), MaximumContinentalJacobianRoundtrip < 1E-10);
	TestTrue(TEXT("従来の短波長を含む接線往復は位相丸めの尺度内"), MaximumScaledJacobianRoundtrip < 64 * std::numeric_limits<double>::epsilon());
	TestTrue(TEXT("大陸規模の変形が実際に適用される"), MaximumContinentalDisplacement > 0.05);
	TestTrue(TEXT("明示逆写像で両方向の往復が一致"), MaximumInverse < 1E-12);
	TestTrue(TEXT("微分の有無で評価方向を変えない"), MaximumMappedDifference < 1E-14);
	TestTrue(TEXT("球面の面積倍率が正の１"), MaximumArea < 1E-10);
	TestTrue(TEXT("独立な差分でも面積倍率が１"), MaximumNumericArea < 1E-5);
	TestTrue(TEXT("解析的な接線写像が独立差分と一致"), MaximumDerivative < 1E-5);
	TestTrue(TEXT("合成した高さの勾配が独立差分と一致"), MaximumGradient < 0.0005);
	TestTrue(TEXT("引戻した勾配は球面接線"), MaximumRadial < 1E-10);
	SphericalWarpSettings Settings;
	Settings.DerivativeBudget = 0;
	const std::optional<PlanetSphericalWarp> Identity = PlanetSphericalWarp::Create(Settings);
	// 無変形も正規の比較入力として受理出来ない場合の返戻
	if(!TestTrue(TEXT("変形量０を受理"), Identity.has_value())) return false;
	const UnitDirection Point = *UnitDirection::FromVector({ 0.2, -0.3, 0.5 });
	TestTrue(TEXT("変形量０は恒等写像"), (Identity->ToSource(Point).Vector() - Point.Vector()).Length() < 1E-15);
	Settings.DerivativeBudget = 4;
	TestFalse(TEXT("従来モードは従来の予算上限を保持"), PlanetSphericalWarp::Create(Settings).has_value());
	Settings.ShouldUseContinentalWavelengths = true;
	TestTrue(TEXT("大陸モードだけが拡大予算を受理"), PlanetSphericalWarp::Create(Settings).has_value());
	Settings.DerivativeBudget = 12.01;
	TestFalse(TEXT("大陸モードも有限な予算上限を保持"), PlanetSphericalWarp::Create(Settings).has_value());
	Settings.DerivativeBudget = 0;
	Settings.ShouldUseContinentalWavelengths = false;
	Settings.RadiusMeters = 999;
	TestFalse(TEXT("最小半径未満を拒否"), PlanetSphericalWarp::Create(Settings).has_value());
	Settings.RadiusMeters = 1000;
	Settings.LargestWavelengthMeters = 0;
	TestFalse(TEXT("波長０を拒否"), PlanetSphericalWarp::Create(Settings).has_value());
	Settings.LargestWavelengthMeters = std::numeric_limits<double>::min();
	TestFalse(TEXT("非有限又は過大な周波数を拒否"), PlanetSphericalWarp::Create(Settings).has_value());
	Settings.LargestWavelengthMeters = 40000;
	Settings.DerivativeBudget = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("非有限な変形量を拒否"), PlanetSphericalWarp::Create(Settings).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Inverse spherical warp: derivative=%.12g direction=%.12g jacobianRoundtrip=%.12g."),
			MaximumInverseDerivative,
			MaximumInverseMappedDifference,
			MaximumJacobianRoundtrip
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spherical warp: samples=%llu inverse=%.12g direction=%.12g area=%.12g numericArea=%.12g derivative=%.12g ")
			TEXT("gradient=%.12g radial=%.12g maximumStretch=%.12g continentalDisplacement=%.12g milliseconds=%.6f"),
			static_cast<unsigned long long>(Samples),
			MaximumInverse,
			MaximumMappedDifference,
			MaximumArea,
			MaximumNumericArea,
			MaximumDerivative,
			MaximumGradient,
			MaximumRadial,
			MaximumStretch,
			MaximumContinentalDisplacement,
			std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count()
		)
	);
	// 逆写像・面積・独立差分・入力条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
