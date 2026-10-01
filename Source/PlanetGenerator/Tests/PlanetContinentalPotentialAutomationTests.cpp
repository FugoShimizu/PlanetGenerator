#include "Core/PlanetContinentalPotential.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetContinentalPotentialContracts,
	"PlanetGenerator.ContinentalPotential.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 有限源の高さと微分を独立な閉形式・差分で検査する
 * @param Parameters 未使用の Automation 引数
 * @return 全条件が合格した場合に真
 */
bool FPlanetContinentalPotentialContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::ContinentalPotential;
	using PlanetCore::ContinentalPotentialSample;
	using PlanetCore::ContinentalPotentialSettings;
	ContinentalPotentialSettings SingleSettings;
	SingleSettings.BackgroundMeters = 10;
	SingleSettings.Sources.push_back({ 0, 0, 0.2, 120 });
	const std::optional<ContinentalPotential> Single = ContinentalPotential::Create(SingleSettings);
	// 正規入力を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("正の有限源を準備"), Single.has_value())) return false;
	const std::optional<ContinentalPotentialSample> Peak = Single->Evaluate(0, 0), Outside = Single->Evaluate(0.8, 0);
	// 解析解付きの点が欠落した場合の返戻
	if(!TestTrue(TEXT("山頂と調和的な周辺部の値を取得"), Peak && Outside)) return false;
	const double KnownPeak = 10 + 120 * (std::log(5.0) + 11.0 / 12), KnownOutside = 10 * (1 - 0.8 * 0.8) - 120 * std::log(0.8);
	const bool IsKnownSolution = TestTrue(
		TEXT("中心源の独立な閉形式と高さ・傾斜・曲率が一致"),
		std::abs(Peak->HeightMeters - KnownPeak) < 1E-12 && Peak->GradientPerUnit[0] == 0 && Peak->GradientPerUnit[1] == 0 &&
		std::abs(Peak->HessianPerUnitSquared[0] + 9020) < 1E-10 && std::abs(Outside->HeightMeters - KnownOutside) < 1E-12 &&
		std::abs(Outside->GradientPerUnit[0] + 166) < 1E-12 && std::abs(Outside->HessianPerUnitSquared[0] - 167.5) < 1E-10 &&
		std::abs(Outside->HessianPerUnitSquared[2] + 207.5) < 1E-10
	);
	// 閉形式との不一致を保持する返戻
	if(!IsKnownSolution) return false;
	const auto SourceHessian = [&](const double X) -> double {
		return Single->Evaluate(X, 0)->HessianPerUnitSquared[0];
	};
	constexpr double Step = 1E-5;
	const double BoundaryHessian = SourceHessian(0.2);
	const double LeftThird = (3 * BoundaryHessian - 4 * SourceHessian(0.2 - Step) + SourceHessian(0.2 - 2 * Step)) / (2 * Step);
	const double RightThird = (-3 * BoundaryHessian + 4 * SourceHessian(0.2 + Step) - SourceHessian(0.2 + 2 * Step)) / (2 * Step);
	const bool HasThirdDerivativeJoin =
	TestTrue(TEXT("有限源の外側と３階迄の微分が接続"), std::abs(LeftThird + 30000) < 0.01 && std::abs(RightThird + 30000) < 0.01);
	// 源の境界で３階の接続が不合格の場合の返戻
	if(!HasThirdDerivativeJoin) return false;
	ContinentalPotentialSettings Settings;
	Settings.BackgroundMeters = 2;
	Settings.Sources = { { -0.35, 0.12, 0.16, 210 }, { 0.42, -0.2, 0.08, 130 }, { 0.1, 0.3, 0.22, 90 } };
	const std::optional<ContinentalPotential> Field = ContinentalPotential::Create(Settings);
	const std::optional<ContinentalPotential> Repeated = ContinentalPotential::Create(Settings);
	// 同じ設定の不変場を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("複数の独立な源を準備"), Field && Repeated)) return false;
	std::size_t Samples = 0;
	double MaximumGradientError = 0, MaximumHessianError = 0, MaximumTraceError = 0;
	for(unsigned i = 0; i < 257; ++i) {
		const double Angle = i * 2.399963229728653, Radius = 0.92 * std::sqrt((i + 0.5) / 257.0), X = Radius * std::cos(Angle);
		const double Y = Radius * std::sin(Angle);
		const std::optional<ContinentalPotentialSample> Point = Field->Evaluate(X, Y), Again = Repeated->Evaluate(X, Y);
		const std::optional<double> Height = Field->EvaluateHeight(X, Y);
		const bool HasSamples = TestTrue(TEXT("照会結果の存在"), Point && Again && Height);
		// 独立検査に必要な値を得られなかった場合の返戻
		if(!HasSamples) return false;
		const double Laplacian = Point->LaplacianPerUnitSquared;
		const bool IsValid = TestTrue(
			TEXT("正の内部高さ・厳密に負の Laplacian・高さのみの照会が一致"),
			Point && Again && Height && Point->HeightMeters > 0 && Point->HeightMeters <= Field->GetAbsoluteHeightBoundMeters() &&
			Point->HeightMeters == *Height && Point->HeightMeters == Again->HeightMeters &&
			Point->GradientPerUnit == Again->GradientPerUnit && Point->HessianPerUnitSquared == Again->HessianPerUnitSquared &&
			Laplacian <= -8 && -Point->LaplacianPerUnitSquared <= Field->GetLaplacianMagnitudeBound()
		);
		// 値の拘束又は同一設定の再準備との不一致の返戻
		if(!IsValid) return false;
		const auto Value = [&](const double A, const double B) -> double {
			return *Field->EvaluateHeight(A, B);
		};
		// 主検査と異なる位置の値だけを使う差分で，勾配と Hessian の全成分を確認する
		constexpr double H = 2E-5;
		for(unsigned axis = 0; axis < 2; ++axis) {
			const double DX = !axis ? H : 0, DY = axis == 1 ? H : 0, Coarse = (Value(X + DX, Y + DY) - Value(X - DX, Y - DY)) / (2 * H);
			const double Fine = (Value(X + DX * 0.5, Y + DY * 0.5) - Value(X - DX * 0.5, Y - DY * 0.5)) / H;
			const double IndependentGradient = (4 * Fine - Coarse) / 3;
			MaximumGradientError = std::max(
				MaximumGradientError,
				std::abs(IndependentGradient - Point->GradientPerUnit[axis]) / std::max(1.0, std::abs(Point->GradientPerUnit[axis]))
			);
		}
		constexpr double K = 1E-4;
		const auto ValueHessian = [&](const double StepSize) -> std::array<double, 3> {
			const double XX = (Value(X + StepSize, Y) - 2 * *Height + Value(X - StepSize, Y)) / (StepSize * StepSize);
			const double YY = (Value(X, Y + StepSize) - 2 * *Height + Value(X, Y - StepSize)) / (StepSize * StepSize);
			const double XY = (
				Value(X + StepSize, Y + StepSize) - Value(X + StepSize, Y - StepSize) - Value(X - StepSize, Y + StepSize) +
				Value(X - StepSize, Y - StepSize)
			) / (4 * StepSize * StepSize);
			// 解析的な Hessian を参照しない値の二階差分の返戻
			return { XX, XY, YY };
		};
		const std::array<double, 3> CoarseHessian = ValueHessian(K), FineHessian = ValueHessian(K * 0.5);
		std::array<double, 3> IndependentHessian;
		for(unsigned component = 0; component < 3; ++component) {
			IndependentHessian[component] = (4 * FineHessian[component] - CoarseHessian[component]) / 3;
		}
		for(unsigned component = 0; component < 3; ++component) {
			MaximumHessianError = std::max(
				MaximumHessianError,
				std::abs(IndependentHessian[component] - Point->HessianPerUnitSquared[component]) /
				std::max(1.0, std::abs(Point->HessianPerUnitSquared[component]))
			);
		}
		MaximumTraceError = std::max(
			MaximumTraceError,
			std::abs(Point->HessianPerUnitSquared[0] + Point->HessianPerUnitSquared[2] - Point->LaplacianPerUnitSquared)
		);
		++Samples;
	}
	const bool HasIndependentDerivatives = TestTrue(
		TEXT("値だけの独立差分と解析的な Laplacian が一致"),
		MaximumGradientError < 2E-7 && MaximumHessianError < 2E-4 && MaximumTraceError < 1E-9
	);
	// 値の差分に依る独立微分の不合格を保持する返戻
	if(!HasIndependentDerivatives) return false;
	for(const std::array<double, 2> Coast : std::array<std::array<double, 2>, 4>{ { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } } }) {
		const std::optional<ContinentalPotentialSample> Point = Field->Evaluate(Coast[0], Coast[1]);
		const bool HasCoast = TestTrue(
			TEXT("海岸が厳密に零で外向きに下降"),
			Point && !Point->HeightMeters && Point->GradientPerUnit[0] * Coast[0] + Point->GradientPerUnit[1] * Coast[1] < 0
		);
		// 解析的な海岸条件の不合格を保持する返戻
		if(!HasCoast) return false;
	}
	TestEqual(TEXT("同一入力の識別子が一致"), Field->GetDefinitionHash(), Repeated->GetDefinitionHash());
	TestEqual(TEXT("全有限源を保持"), Field->GetSourceCount(), std::size_t{ 3 });
	// 離れた山塊の同時最大を仮定せず，最寄り領域の上界を実際の源・内部・海岸で照合
	ContinentalPotentialSettings SeparatedSettings;
	SeparatedSettings.BackgroundMeters = 1;
	SeparatedSettings.Sources = { { -0.45, 0, 0.004, 100 }, { 0.45, 0, 0.004, 100 } };
	const std::optional<ContinentalPotential> Separated = ContinentalPotential::Create(SeparatedSettings);
	// 正規な離れた有限源を拒否した場合の返戻
	if(!TestTrue(TEXT("離れた山塊の正規準備"), Separated.has_value())) return false;
	const double OriginalSeparatedBound = 1 + 200 * (std::log(1.45 / 0.004) + 11.0 / 12);
	const double SeparatedBound = Separated->GetAbsoluteHeightBoundMeters();
	TestTrue(TEXT("別地点のピークを重ねた古い上界より狭い"), SeparatedBound < 0.65 * OriginalSeparatedBound);
	std::uint64_t BoundSamples = 0;
	double MaximumBoundRatio = 0;
	for(unsigned row = 0; row < 129; ++row) for(unsigned column = 0; column < 129; ++column) {
		const double X = column / 64.0 - 1, Y = row / 64.0 - 1;
		if(X * X + Y * Y > 1) continue;
		const std::optional<double> Height = Separated->EvaluateHeight(X, Y);
		// 上界の独立な内部標本を解像出来ない場合の返戻
		if(!TestTrue(TEXT("上界検査の独立な円板内照会"), Height.has_value())) return false;
		TestTrue(TEXT("源の領域境界と全標本は構成上界以下"), *Height >= 0 && *Height <= SeparatedBound);
		MaximumBoundRatio = std::max(MaximumBoundRatio, *Height / SeparatedBound);
		++BoundSamples;
	}
	for(const double X : { -0.45, 0.45, -0.454, -0.446, 0.446, 0.454, 0.0 }) {
		const std::optional<double> Height = Separated->EvaluateHeight(X, 0);
		TestTrue(TEXT("源中心と有限支持の境界も全域上界以下"), Height && *Height >= 0 && *Height <= SeparatedBound);
	}
	ContinentalPotentialSettings CoincidentSettings = SeparatedSettings;
	CoincidentSettings.Sources[1].X = -0.45;
	const std::optional<ContinentalPotential> Coincident = ContinentalPotential::Create(CoincidentSettings);
	const std::optional<double> CoincidentPeak = Coincident ? Coincident->EvaluateHeight(-0.45, 0) : std::optional<double>{};
	TestTrue(
		TEXT("同じ地点の源では誤った離隔を仮造しない"),
		Coincident && CoincidentPeak && *CoincidentPeak <= Coincident->GetAbsoluteHeightBoundMeters()
	);
	AddInfo(
		FString::Printf(
			TEXT("Separated uplift bound: samples=%llu original=%.17g tightened=%.17g ratio=%.17g. ")
			TEXT("Nearest-source distance construction; finite samples complement the analytic bound."),
			static_cast<unsigned long long>(BoundSamples),
			OriginalSeparatedBound,
			SeparatedBound,
			MaximumBoundRatio
		)
	);
	ContinentalPotentialSettings Changed = Settings;
	++Changed.Sources[0].WeightMeters;
	const std::optional<ContinentalPotential> Different = ContinentalPotential::Create(Changed);
	TestTrue(TEXT("全入力が識別子へ参加"), Different && Different->GetDefinitionHash() != Field->GetDefinitionHash());
	TestFalse(TEXT("円板外を拒否"), Field->Evaluate(1.001, 0).has_value());
	TestFalse(TEXT("非有限な位置を拒否"), Field->EvaluateHeight(std::numeric_limits<double>::quiet_NaN(), 0).has_value());
	Changed = Settings;
	Changed.Sources[0].Radius = 0.9;
	TestFalse(TEXT("海岸を跨ぐ源を拒否"), ContinentalPotential::Create(Changed).has_value());
	Changed = Settings;
	Changed.Sources[0].Radius = std::numeric_limits<double>::denorm_min();
	TestFalse(TEXT("表現出来ない源の微分を拒否"), ContinentalPotential::Create(Changed).has_value());
	Changed = Settings;
	Changed.BackgroundMeters = 0;
	TestFalse(TEXT("厳密な背景隆起が必要"), ContinentalPotential::Create(Changed).has_value());
	Changed = Settings;
	Changed.Sources.resize(ContinentalPotential::MaximumSources + 1);
	TestFalse(TEXT("源数の予算超過を拒否"), ContinentalPotential::Create(Changed).has_value());
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(TEXT("中断された準備を公開しない"), ContinentalPotential::Create(Settings, Stop.get_token()).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Continental potential: samples=%llu gradient=%.12g Hessian=%.12g trace=%.12g leftThird=%.12g rightThird=%.12g."),
			static_cast<unsigned long long>(Samples),
			MaximumGradientError,
			MaximumHessianError,
			MaximumTraceError,
			LeftThird,
			RightThird
		)
	);
	// 有限個の条件の合格であり，球面・侵食・全河川・自然な景観の完成証明ではない返戻
	return true;
}

#endif
