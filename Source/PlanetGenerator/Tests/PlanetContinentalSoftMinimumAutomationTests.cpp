#include "Core/PlanetContinentalDescent.h"
#include "Core/PlanetContinentalPotential.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetContinentalSoftMinimumContracts,
    "PlanetGenerator.ContinentalSoftMinimum.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 複数隆起の滑らかな最小を海岸・独立微分・負の Laplacian・下降区間で検査する
 * @param Parameters 未使用の UE 引数
 * @return 最終合成場と其の下降経路が一致する場合に真
 */
bool FPlanetContinentalSoftMinimumContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::ContinentalDescentError;
	using PlanetCore::ContinentalDescentPath;
	using PlanetCore::ContinentalDescentSettings;
	using PlanetCore::ContinentalPotential;
	using PlanetCore::ContinentalPotentialSample;
	using PlanetCore::ContinentalPotentialSettings;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	std::vector<ContinentalPotentialSettings> Layers(2);
	Layers[0].BackgroundMeters = 80;
	Layers[0].Sources = { { -0.42, 0.15, 0.08, 140 }, { 0.25, 0.35, 0.11, 70 } };
	Layers[1].BackgroundMeters = 95;
	Layers[1].Sources = { { 0.38, -0.15, 0.07, 150 }, { -0.20, -0.42, 0.10, 90 } };
	const std::optional<ContinentalPotential> First = ContinentalPotential::Create(Layers[0]);
	const std::optional<ContinentalPotential> Second = ContinentalPotential::Create(Layers[1]);
	const std::optional<ContinentalPotential> Blend = ContinentalPotential::CreateBlend(Layers, 120);
	// 正規な単層と合成の準備失敗の返戻
	if(!TestTrue(TEXT("正の二層の滑らかな最小を準備"), First && Second && Blend)) return false;
	TestEqual(TEXT("合成の源を各層から計上"), Blend->GetSourceCount(), std::size_t{ 4 });
	double MinimumLaplacianMagnitude = std::numeric_limits<double>::infinity(), MaximumGradientError = 0, MaximumHessianError = 0;
	double MaximumUpperViolation = 0;
	std::uint64_t Samples = 0, Differences = 0;
	for(unsigned Row = 0; Row < 65; ++Row) for(unsigned Column = 0; Column < 65; ++Column) {
		const double X = Column / 32.0 - 1, Y = Row / 32.0 - 1;
		if(X * X + Y * Y >= 0.98) continue;
		const std::optional<ContinentalPotentialSample> Sample = Blend->Evaluate(X, Y);
		const std::optional<double> A = First->EvaluateHeight(X, Y), B = Second->EvaluateHeight(X, Y);
		// 内部の正値と独立した単層の境界を照合出来ない場合の返戻
		if(!TestTrue(TEXT("合成と二層の内部を照会"), Sample && A && B)) return false;
		TestTrue(TEXT("最終地形の内部に閉鎖極小を作らない"), Sample->HeightMeters > 0 && Sample->LaplacianPerUnitSquared < 0);
		TestTrue(
			TEXT("滑らかな最小は二層の高さの範囲内"),
			Sample->HeightMeters >= std::min(*A, *B) - 1E-11 && Sample->HeightMeters <= std::max(*A, *B) + 1E-11
		);
		const std::optional<double> Height = Blend->EvaluateHeight(X, Y);
		TestTrue(TEXT("高さと微分付き照会は同一"), Height && *Height == Sample->HeightMeters);
		MinimumLaplacianMagnitude = std::min(MinimumLaplacianMagnitude, -Sample->LaplacianPerUnitSquared);
		MaximumUpperViolation = std::max(MaximumUpperViolation, Sample->HeightMeters - Blend->GetAbsoluteHeightBoundMeters());
		if(X * X + Y * Y < 0.64 && !(Row % 8) && !(Column % 8)) {
			constexpr double H = 1E-4;
			const double XP = *Blend->EvaluateHeight(X + H, Y), XM = *Blend->EvaluateHeight(X - H, Y);
			const double YP = *Blend->EvaluateHeight(X, Y + H), YM = *Blend->EvaluateHeight(X, Y - H), NumericalGX = (XP - XM) / (2 * H);
			const double NumericalGY = (YP - YM) / (2 * H), NumericalXX = (XP - 2 * Sample->HeightMeters + XM) / (H * H);
			const double NumericalYY = (YP - 2 * Sample->HeightMeters + YM) / (H * H);
			MaximumGradientError =
			std::max(MaximumGradientError, std::hypot(NumericalGX - Sample->GradientPerUnit[0], NumericalGY - Sample->GradientPerUnit[1]));
			MaximumHessianError = std::max(
				MaximumHessianError,
				std::max(std::abs(NumericalXX - Sample->HessianPerUnitSquared[0]), std::abs(NumericalYY - Sample->HessianPerUnitSquared[2]))
			);
			++Differences;
		}
		++Samples;
	}
	for(const std::array<double, 2> Coast : { std::array<double, 2>{ 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } }) {
		const std::optional<ContinentalPotentialSample> Sample = Blend->Evaluate(Coast[0], Coast[1]);
		TestTrue(TEXT("同じ解析海岸の高さは正確に零"), Sample && !Sample->HeightMeters);
	}
	TestTrue(TEXT("独立差分と解析勾配が一致"), MaximumGradientError < 1E-4);
	TestTrue(TEXT("独立差分と解析曲率が一致"), MaximumHessianError < 0.1);
	TestTrue(TEXT("二層の保守的な標高上界を超えない"), MaximumUpperViolation <= 0);
	const std::optional<PlanetCore::ContinentalPotentialDerivativeBounds> Bounds = Blend->GetDerivativeBounds(0.1, 0.1, 0.05);
	TestTrue(TEXT("合成の有限な区間微分上界"), Bounds && Bounds->HessianNormPerUnitSquared > 0 && Bounds->ThirdDerivativeNormPerUnitCubed > 0);
	ContinentalDescentSettings Controls;
	std::uint64_t Paths = 0, Segments = 0;
	for(const std::array<double, 2> Start : { std::array<double, 2>{ 0.1, 0.1 }, { -0.4, 0.15 }, { 0.38, -0.15 }, { 0.0, 0.0 } }) {
		ContinentalDescentError Error = ContinentalDescentError::None;
		const std::optional<ContinentalDescentPath> Path = ContinentalDescentPath::Create(*Blend, Start, Controls, Error);
		const bool HasPath = TestTrue(
			FString::Printf(TEXT("最終合成場の下降経路が海岸へ到達 X=%.6g Y=%.6g reason=%u"), Start[0], Start[1], static_cast<unsigned>(Error)),
			Path.has_value()
		);
		// 元の単層経路で合成地形の不達を隠さない返戻
		if(!HasPath) return false;
		Segments += Path->GetSegments().size();
		++Paths;
	}
	TestTrue(TEXT("厳密に負の内部 Laplacian"), MinimumLaplacianMagnitude > 0);
	const std::optional<ContinentalPotential> Again = ContinentalPotential::CreateBlend(Layers, 120);
	TestTrue(TEXT("同じ合成入力は再現"), Again && Again->GetDefinitionHash() == Blend->GetDefinitionHash());
	const std::optional<ContinentalPotential> Different = ContinentalPotential::CreateBlend(Layers, 130);
	TestTrue(TEXT("温度を変えると識別子が変わる"), Different && Different->GetDefinitionHash() != Blend->GetDefinitionHash());
	TestFalse(TEXT("零温度を拒否"), ContinentalPotential::CreateBlend(Layers, 0).has_value());
	Layers.resize(1);
	TestFalse(TEXT("単層の合成を拒否"), ContinentalPotential::CreateBlend(Layers, 120).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Continental soft minimum: samples=%llu differences=%llu paths=%llu segments=%llu minimumNegativeLaplacian=%.12g ")
			TEXT("gradientError=%.12g HessianError=%.12g upperViolation=%.12g seconds=%.7f. Landscape and full drainage remain pending."),
			static_cast<unsigned long long>(Samples),
			static_cast<unsigned long long>(Differences),
			static_cast<unsigned long long>(Paths),
			static_cast<unsigned long long>(Segments),
			MinimumLaplacianMagnitude,
			MaximumGradientError,
			MaximumHessianError,
			MaximumUpperViolation,
			std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count()
		)
	);
	// 同じ最終場の境界・微分・区間下降を検査した結果の返戻
	return !HasAnyErrors();
}

#endif
