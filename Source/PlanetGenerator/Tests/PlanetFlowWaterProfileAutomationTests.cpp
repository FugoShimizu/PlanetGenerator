#include "Core/PlanetFlowWaterProfile.h"
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
	FPlanetFlowWaterProfileContracts,
	"PlanetGenerator.FlowWaterProfile.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 既知の単調関数と独立な内部標本で，誤差・解析微分・連続性・元場からの独立を検査する
 * @param Parameters 未使用の自動試験引数
 * @return 単調性・精度・有限予算と正規／拒否経路が成立した場合に真
 */
bool FPlanetFlowWaterProfileContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::FlowChannelWaterLevel;
	using PlanetCore::FlowWaterProfile;
	PlanetCore::FlowWaterProfileSettings Settings;
	PlanetCore::FlowWaterProfileDiagnostics Report;
	std::uint32_t Calls = 0, Samples = 0;
	double MaximumHeightError = 0, MaximumSlopeError = 0, MaximumDerivativeError = 0;
	const auto Exact = [](const double Distance) -> FlowChannelWaterLevel {
		// 両端で二階迄一定面へ繋がる解析水位の返戻
		const double T = std::clamp((Distance - 10) / 80, 0.0, 1.0), U = 1 - T;
		const double Height = T > 0.5 ? 10 * U * U * U * (10 + U * (-15 + 6 * U)) : 10 * (1 - T * T * T * (10 + T * (-15 + 6 * T)));
		// 下降端でも桁落ちを避けた独立な解析値の返戻
		return { Height, -3.75 * T * T * U * U };
	};
	const auto Source = [&](const double Distance) -> std::optional<FlowChannelWaterLevel> {
		++Calls;
		// 照会数を持つ準備時の元場の返戻
		return Exact(Distance);
	};
	const std::array<double, 4> Distances { 0, 10, 90, 100 };
	std::optional<FlowWaterProfile> Profile = FlowWaterProfile::Create(Source, Distances, Settings, {}, &Report);
	AddInfo(
		FString::Printf(
			TEXT("Water profile prepared=%u failure=%u knots=%u passes=%u evaluations=%u H=%.12g slope=%.12g"),
			static_cast<unsigned>(Profile.has_value()),
			static_cast<unsigned>(Report.Failure),
			Report.Knots,
			Report.Passes,
			Report.Evaluations,
			Report.HeightErrorMeters,
			Report.SlopeError
		)
	);
	// 精度未達を空の成功として扱わない返戻
	if(!TestTrue(TEXT("下降する解析縦断を準備"), Profile.has_value())) return false;
	const std::uint32_t PreparedCalls = Calls;
	double Previous = Profile->Sample(0)->ElevationMeters;
	for(unsigned I = 0; I < 8193; ++I) {
		const double Distance = 100 * (I + 0.371) / 8193;
		const FlowChannelWaterLevel Actual = Exact(Distance);
		const std::optional<FlowChannelWaterLevel> Value = Profile->Sample(Distance);
		// 水位の欠落を試料から除外しない返戻
		if(!TestTrue(TEXT("独立な節点外の標本を評価"), Value.has_value())) return false;
		MaximumHeightError = std::max(MaximumHeightError, std::abs(Value->ElevationMeters - Actual.ElevationMeters));
		MaximumSlopeError = std::max(MaximumSlopeError, std::abs(Value->DerivativePerMeter - Actual.DerivativePerMeter));
		TestTrue(TEXT("標高も解析勾配も下流へ非増加"), Value->ElevationMeters <= Previous && !(Value->DerivativePerMeter > 0));
		Previous = Value->ElevationMeters;
		constexpr double Step = 1E-4;
		const double Difference =
		(Profile->Sample(Distance + Step)->ElevationMeters - Profile->Sample(Distance - Step)->ElevationMeters) / (2 * Step);
		MaximumDerivativeError = std::max(MaximumDerivativeError, std::abs(Difference - Value->DerivativePerMeter));
		++Samples;
	}
	TestTrue(TEXT("節点外の高さ誤差を保持"), MaximumHeightError < Settings.HeightToleranceMeters);
	TestTrue(TEXT("節点外の微分誤差を保持"), MaximumSlopeError < Settings.SlopeTolerance);
	TestTrue(TEXT("公開微分が同じ補間曲線の独立差分と一致"), MaximumDerivativeError < 1E-7);
	TestEqual(TEXT("照会時に元の場を再評価しない"), Calls, PreparedCalls);
	TestEqual(TEXT("上流湖の水位を正確に保持"), Profile->Sample(0)->ElevationMeters, 10.0);
	TestEqual(TEXT("下流湖の水位を正確に保持"), Profile->Sample(100)->ElevationMeters, 0.0);
	TestEqual(TEXT("上流湖の水平接線を保持"), Profile->Sample(10)->DerivativePerMeter, 0.0);
	TestEqual(TEXT("下流湖の水平接線を保持"), Profile->Sample(90)->DerivativePerMeter, 0.0);
	TestFalse(TEXT("範囲外を黙って丸めない"), Profile->Sample(-1).has_value() || Profile->Sample(101).has_value());
	TestFalse(TEXT("非有限な照会を拒否"), Profile->Sample(std::numeric_limits<double>::quiet_NaN()).has_value());
	const std::array<double, 2> Ends { 0, 100 };
	const auto Linear = [](const double Distance) -> std::optional<FlowChannelWaterLevel> {
		// 既知の下降直線の返戻
		return FlowChannelWaterLevel{ 10 - Distance * 0.1, -0.1 };
	};
	const std::optional<FlowWaterProfile> Line = FlowWaterProfile::Create(Linear, Ends, Settings);
	TestTrue(TEXT("二節点で下降直線を保持"), Line && Line->KnotCount() == 2 && std::abs(Line->Sample(37)->ElevationMeters - 6.3) < 1E-14);
	const auto Invalid = [](const double) -> std::optional<FlowChannelWaterLevel> {
		// 欠落した元水位の返戻
		return {};
	};
	TestFalse(TEXT("欠落した元水位を拒否"), FlowWaterProfile::Create(Invalid, Ends, Settings, {}, &Report).has_value());
	TestTrue(TEXT("元場の失敗を識別"), Report.Failure == PlanetCore::FlowWaterProfileFailure::Source);
	const auto Rising = [](const double Distance) -> std::optional<FlowChannelWaterLevel> {
		// 高さも微分も上昇する不正水位の返戻
		return FlowChannelWaterLevel{ Distance, 1 };
	};
	TestFalse(TEXT("逆勾配を補間で隠さない"), FlowWaterProfile::Create(Rising, Ends, Settings, {}, &Report).has_value());
	TestTrue(TEXT("逆勾配の失敗を識別"), Report.Failure == PlanetCore::FlowWaterProfileFailure::Rising);
	Settings.MaximumKnots = 4;
	TestFalse(TEXT("節点不足で精度未達を公開しない"), FlowWaterProfile::Create(Source, Distances, Settings, {}, &Report).has_value());
	TestTrue(TEXT("節点予算の失敗を識別"), Report.Failure == PlanetCore::FlowWaterProfileFailure::Budget);
	Settings.MaximumKnots = 16384;
	std::stop_source Cancel;
	Cancel.request_stop();
	const std::uint32_t BeforeCancel = Calls;
	TestFalse(TEXT("中断済の近似を準備しない"), FlowWaterProfile::Create(Source, Distances, Settings, Cancel.get_token(), &Report).has_value());
	TestTrue(TEXT("中断理由を保持"), Report.Failure == PlanetCore::FlowWaterProfileFailure::Cancelled);
	TestEqual(TEXT("中断後に元場を照会しない"), Calls, BeforeCancel);
	const std::array<double, 3> Duplicated { 0, 50, 50 };
	TestFalse(TEXT("重複する公称距離を拒否"), FlowWaterProfile::Create(Source, Duplicated, Settings).has_value());
	Settings.SlopeTolerance = 0;
	TestFalse(TEXT("零の誤差予算を拒否"), FlowWaterProfile::Create(Source, Distances, Settings).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Water profile contracts: samples=%u H=%.12g slope=%.12g derivative=%.12g sourceCalls=%u"),
			Samples,
			MaximumHeightError,
			MaximumSlopeError,
			MaximumDerivativeError,
			PreparedCalls
		)
	);
	// 独立な精度と拒否条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
