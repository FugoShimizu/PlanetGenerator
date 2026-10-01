#include "Core/PlanetSphericalGradient.h"
#include "Core/PlanetSphericalSpline.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalGradientContracts, "PlanetGenerator.SphericalGradient.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 独立な接平面の二次式，回転不変性，低周波場の収束，不正入力を検査する
 * @param Parameters 未使用の UE 引数
 * @return 近傍標高から得た接線が解析的な条件を満たす場合に真
 */
bool FPlanetSphericalGradientContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::SphericalGradientEstimate;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(3);
	// 検査網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("推定勾配の検査網を準備"), Mesh.has_value())) return false;
	// 推定処理とは異なる基底で指定した二次式
	const Vec3 Up = Mesh->Directions[27].Vector(), Axis { 0.21, -0.58, 0.13 };
	const Vec3 East = UnitDirection::FromVector(Axis - Up * Up.Dot(Axis))->Vector(), North = Up.Cross(East);
	std::vector<double> Heights;
	for(const UnitDirection Direction : Mesh->Directions) {
		const double X = Direction.Vector().Dot(East), Y = Direction.Vector().Dot(North);
		Heights.push_back(7 + 3 * X - 5 * Y + 0.3 * X * X + 0.7 * X * Y - 0.2 * Y * Y);
	}
	const std::optional<SphericalGradientEstimate> Result =
	PlanetCore::EstimateSphericalGradients(Mesh->Directions, Mesh->Triangles, Heights);
	// 有効な二次データを解けない場合の返戻
	if(!TestTrue(TEXT("中心値を保持する二次近似を準備"), Result.has_value())) return false;
	const double CenterError = (Result->Gradients[27] - East * 3 + North * 5).Length();
	TestTrue(TEXT("独立に指定した中心の二次接線を再現"), CenterError < 1E-11);
	// 同じ値を剛体回転した網へ載せ，座標軸への依存を検査する
	const Vec3 RotationAxis = UnitDirection::FromVector({ -0.37, 0.49, 0.78 })->Vector();
	const auto Rotate = [RotationAxis](const Vec3 Value) -> Vec3 {
		const double Cosine = std::cos(0.67), Sine = std::sin(0.67);
		// Rodrigues の独立な回転式の返戻
		return Value * Cosine + RotationAxis.Cross(Value) * Sine + RotationAxis * (RotationAxis.Dot(Value) * (1 - Cosine));
	};
	std::vector<UnitDirection> Rotated;
	for(const UnitDirection Direction : Mesh->Directions) Rotated.push_back(*UnitDirection::FromVector(Rotate(Direction.Vector())));
	const std::optional<SphericalGradientEstimate> RotatedResult =
	PlanetCore::EstimateSphericalGradients(Rotated, Mesh->Triangles, Heights);
	// 回転した正規入力を拒否しない返戻
	if(!TestTrue(TEXT("剛体回転後も同じ標本を受理"), RotatedResult.has_value())) return false;
	double MaximumRotationError = 0, MaximumNormalComponent = 0;
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		MaximumRotationError =
		std::max(MaximumRotationError, (RotatedResult->Gradients[Index] - Rotate(Result->Gradients[Index])).Length());
		MaximumNormalComponent =
		std::max(MaximumNormalComponent, std::abs(Result->Gradients[Index].Dot(Mesh->Directions[Index].Vector())));
	}
	TestTrue(TEXT("推定接線が剛体回転へ追従"), MaximumRotationError < 1E-10);
	TestTrue(TEXT("球面に接する微分だけを返す"), MaximumNormalComponent < 1E-12);
	TestTrue(TEXT("正規網を二次近似で処理"), Result->QuadraticVertices == Mesh->Directions.size() && Result->LinearVertices == 0);
	// 値の移動や雑音を含まない低周波解析場の収束
	double PreviousError = std::numeric_limits<double>::infinity();
	for(const std::uint8_t Level : { std::uint8_t{ 2 }, std::uint8_t{ 3 }, std::uint8_t{ 4 } }) {
		const std::optional<PlanetCore::SphereMesh> Refined = PlanetCore::BuildUniformSphere(Level);
		// 正規の細分化網が無い場合の返戻
		if(!TestTrue(TEXT("収束比較の網を準備"), Refined.has_value())) return false;
		std::vector<double> Values;
		for(const UnitDirection Direction : Refined->Directions) {
			const Vec3 P = Direction.Vector();
			Values.push_back(std::sin(1.7 * P.X + 2.3 * P.Y) + 0.8 * std::sin(2 * P.Z - P.X));
		}
		const std::optional<SphericalGradientEstimate> Estimate =
		PlanetCore::EstimateSphericalGradients(Refined->Directions, Refined->Triangles, Values);
		// 正規データの推定が成立しない場合の返戻
		if(!TestTrue(TEXT("低周波場の接線を推定"), Estimate.has_value())) return false;
		double Sum = 0, Maximum = 0;
		for(std::size_t Index = 0; Index < Values.size(); ++Index) {
			const Vec3 P = Refined->Directions[Index].Vector();
			const double A = 1.7 * P.X + 2.3 * P.Y, B = 2 * P.Z - P.X;
			const Vec3 Ambient { 1.7 * std::cos(A) - 0.8 * std::cos(B), 2.3 * std::cos(A), 1.6 * std::cos(B) };
			const double Error = (Estimate->Gradients[Index] - Ambient + P * P.Dot(Ambient)).Length();
			Sum += Error * Error;
			Maximum = std::max(Maximum, Error);
		}
		const double Rms = std::sqrt(Sum / Values.size());
		TestTrue(TEXT("細分化に伴って解析接線へ収束"), Rms < PreviousError * 0.4);
		PreviousError = Rms;
		AddInfo(
			FString::Printf(
				TEXT("Gradient convergence: level=%u nodes=%llu rms=%.12g maximum=%.12g linear=%u"),
				Level,
				static_cast<unsigned long long>(Values.size()),
				Rms,
				Maximum,
				Estimate->LinearVertices
			)
		);
	}
	// 正規経路と同じ公開入口に於ける入力拒否
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断済の推定を拒否"),
		PlanetCore::EstimateSphericalGradients(Mesh->Directions, Mesh->Triangles, Heights, Cancelled.get_token()).has_value()
	);
	TestFalse(TEXT("異なる標高列の寸法を拒否"), PlanetCore::EstimateSphericalGradients(Mesh->Directions, Mesh->Triangles, {}).has_value());
	std::vector<std::array<std::uint32_t, 3>> Broken = Mesh->Triangles;
	Broken.pop_back();
	TestFalse(TEXT("開いた球面を拒否"), PlanetCore::EstimateSphericalGradients(Mesh->Directions, Broken, Heights).has_value());
	Heights[12] = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("非有限な標高を拒否"), PlanetCore::EstimateSphericalGradients(Mesh->Directions, Mesh->Triangles, Heights).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Gradient contracts: centerError=%.12g rotationError=%.12g normal=%.12g"),
			CenterError,
			MaximumRotationError,
			MaximumNormalComponent
		)
	);
	// 全契約の検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalGradientAliasing, "PlanetGenerator.SphericalGradient.Aliasing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 節点間隔より短い波の瞬間勾配が，元振幅より大きい補間起伏を作る条件を比較する
 * @param Parameters 未使用の UE 引数
 * @return 推定接線で増幅を減らし，元節点値も保持する場合に真
 */
bool FPlanetSphericalGradientAliasing::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::SphericalSplineField;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(3);
	// 検査網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("短波長の比較網を準備"), Mesh.has_value())) return false;
	std::vector<double> Heights;
	std::vector<Vec3> ExactGradients;
	for(const UnitDirection Direction : Mesh->Directions) {
		const Vec3 P = Direction.Vector();
		const double Phase = 1000 * P.Z + 0.37;
		Heights.push_back(std::sin(Phase));
		const Vec3 Ambient { 0, 0, 1000 * std::cos(Phase) };
		ExactGradients.push_back(Ambient - P * P.Dot(Ambient));
	}
	const std::optional<PlanetCore::SphericalGradientEstimate> Estimated =
	PlanetCore::EstimateSphericalGradients(Mesh->Directions, Mesh->Triangles, Heights);
	// 推定を準備出来ない場合の返戻
	if(!TestTrue(TEXT("標本尺度の勾配を準備"), Estimated.has_value())) return false;
	const std::optional<SphericalSplineField> Analytic =
	SphericalSplineField::CreateOnMeshWithGradients(3, Mesh->Directions, Mesh->Triangles, Heights, ExactGradients);
	const std::optional<SphericalSplineField> Fitted =
	SphericalSplineField::CreateOnMeshWithGradients(3, Mesh->Directions, Mesh->Triangles, Heights, Estimated->Gradients);
	// 両方の比較面が無い場合の返戻
	if(!TestTrue(TEXT("同じ節点値を二種類の接線で補間"), Analytic.has_value() && Fitted.has_value())) return false;
	double ExactPeak = 0, FittedPeak = 0, ExactSquaredError = 0, FittedSquaredError = 0, NodalError = 0;
	for(unsigned Index = 0; Index < 8192; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / 8192, Angle = Index * 2.399963229728653, Ring = std::sqrt(1 - Z * Z);
		const UnitDirection Direction = *UnitDirection::FromVector({ Ring * std::cos(Angle), Ring * std::sin(Angle), Z });
		const double Original = std::sin(1000 * Direction.Vector().Z + 0.37), A = Analytic->Evaluate(Direction).ElevationMeters;
		const double F = Fitted->Evaluate(Direction).ElevationMeters;
		ExactPeak = std::max(ExactPeak, std::abs(A));
		FittedPeak = std::max(FittedPeak, std::abs(F));
		ExactSquaredError += (A - Original) * (A - Original);
		FittedSquaredError += (F - Original) * (F - Original);
	}
	for(std::size_t Index = 0; Index < Heights.size(); ++Index) {
		NodalError = std::max(NodalError, std::abs(Fitted->Evaluate(Mesh->Directions[Index]).ElevationMeters - Heights[Index]));
	}
	TestTrue(TEXT("正確な瞬間勾配を疎な網へ渡す増幅を再現"), ExactPeak > 5);
	TestTrue(TEXT("標本尺度の推定で偽の大起伏を減らす"), FittedPeak < 3 && FittedPeak < ExactPeak * 0.25);
	TestTrue(TEXT("節点外の元場との差も改善"), FittedSquaredError < ExactSquaredError * 0.25);
	TestTrue(TEXT("元の節点標高を保持"), NodalError < 1E-10);
	AddInfo(
		FString::Printf(
			TEXT("Gradient aliasing: amplitude=1 analyticPeak=%.12g fittedPeak=%.12g analyticRms=%.12g fittedRms=%.12g nodal=%.12g"),
			ExactPeak,
			FittedPeak,
			std::sqrt(ExactSquaredError / 8192),
			std::sqrt(FittedSquaredError / 8192),
			NodalError
		)
	);
	// 元の短波長自体の再現とは区別した，増幅条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
