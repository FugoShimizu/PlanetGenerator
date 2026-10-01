#include "Core/PlanetBasinReconstruction.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBasinReconstructionContracts, "PlanetGenerator.BasinReconstruction.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 面状の地形補正を解析解，標本残差，独立微分，索引無しの参照で検査する
 * @param Parameters 未使用の UE 検査引数
 * @return 連続補間と正規・拒否入力の全検査が成功する場合に真
 */
bool FPlanetBasinReconstructionContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetBasinReconstruction;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const UnitDirection Center = *UnitDirection::FromVector({ 1, 0, 0 });
	const std::array<UnitDirection, 1> One { Center };
	const std::array<double, 1> Change { 160 };
	constexpr double Width = 0.2;
	const std::optional<PlanetBasinReconstruction> Single = PlanetBasinReconstruction::Create(One, Change, Width);
	// 一地点の有限支持補正を準備の失敗時の返戻
	if(!TestTrue(TEXT("一地点の有限支持補正を準備"), Single.has_value())) return false;
	TestEqual(TEXT("支持中心で指定標高に一致"), Single->EvaluateElevation(Center), 160.0);
	const double HalfAngle = 2 * std::asin(Width * 0.25);
	const UnitDirection Half = *UnitDirection::FromVector({ std::cos(HalfAngle), std::sin(HalfAngle), 0 });
	TestTrue(TEXT("半支持幅で独立な厳密値30 mに一致"), std::abs(Single->EvaluateElevation(Half) - 30) < 1E-11);
	const double BoundaryAngle = 2 * std::asin(Width * 0.5);
	const UnitDirection Outside =
	*UnitDirection::FromVector({ std::cos(BoundaryAngle * 1.001), std::sin(BoundaryAngle * 1.001), 0 });
	const TerrainSample Beyond = Single->Evaluate(Outside);
	TestTrue(TEXT("支持外の値と勾配を厳密に０にする"), !Beyond.ElevationMeters && !Beyond.TangentialGradientPerRadian.Length());
	// 支持端へ近付けた際，値は四次，勾配は三次で零へ収束する
	const auto NearBoundary = [&](const double RelativeGap) {
		const double Angle = 2 * std::asin(Width * (1 - RelativeGap) * 0.5);
		// 境界の内側で独立に作った球面方向の返戻
		return Single->Evaluate(*UnitDirection::FromVector({ std::cos(Angle), std::sin(Angle), 0 }));
	};
	const TerrainSample Near = NearBoundary(0.01), Nearer = NearBoundary(0.005);
	TestTrue(
		TEXT("支持端の値が四次の率で接続"),
		Near.ElevationMeters / Nearer.ElevationMeters > 15.8 && Near.ElevationMeters / Nearer.ElevationMeters < 16.1
	);
	TestTrue(
		TEXT("支持端の勾配が三次の率で接続"),
		Near.TangentialGradientPerRadian.Length() / Nearer.TangentialGradientPerRadian.Length() > 7.8 &&
		Near.TangentialGradientPerRadian.Length() / Nearer.TangentialGradientPerRadian.Length() < 8.1
	);
	TestFalse(TEXT("空の標本を拒否"), PlanetBasinReconstruction::Create({}, {}, Width).has_value());
	TestFalse(TEXT("標本数の不一致を拒否"), PlanetBasinReconstruction::Create(One, {}, Width).has_value());
	TestFalse(TEXT("幅０を拒否"), PlanetBasinReconstruction::Create(One, Change, 0).has_value());
	TestFalse(TEXT("二乗が表現不能な微小幅を拒否"), PlanetBasinReconstruction::Create(One, Change, 1E-200).has_value());
	const std::array<UnitDirection, 2> Duplicates { Center, Center };
	const std::array<double, 2> Conflicting { 1, 2 };
	TestFalse(TEXT("同じ方向の重複拘束を拒否"), PlanetBasinReconstruction::Create(Duplicates, Conflicting, Width).has_value());
	const std::array<double, 1> Nonfinite { std::numeric_limits<double>::quiet_NaN() };
	TestFalse(TEXT("非有限な標高を拒否"), PlanetBasinReconstruction::Create(One, Nonfinite, Width).has_value());
	const std::array<double, 1> Huge { std::numeric_limits<double>::max() };
	TestFalse(TEXT("費用が桁溢れする入力を拒否"), PlanetBasinReconstruction::Create(One, Huge, Width).has_value());
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("中断済みの補間準備を拒否"), PlanetBasinReconstruction::Create(One, Change, Width, Cancel.get_token()).has_value());
	const std::array<double, 1> Zero { 0 }, Negative { -160 };
	const std::optional<PlanetBasinReconstruction> Flat = PlanetBasinReconstruction::Create(One, Zero, Width);
	const std::optional<PlanetBasinReconstruction> Cut = PlanetBasinReconstruction::Create(One, Negative, Width);
	TestTrue(TEXT("変更無しの正規入力は反復せず受理"), Flat && !Flat->Statistics().Iterations && !Flat->EvaluateElevation(Half));
	TestTrue(TEXT("開削側の符号も保持"), Cut && std::abs(Cut->EvaluateElevation(Half) + 30) < 1E-11);
	for(const unsigned Level : { 3U, 6U }) {
		const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(Level);
		// 閉じた球面上の標本を準備の失敗時の返戻
		if(!TestTrue(TEXT("閉じた球面上の標本を準備"), Mesh.has_value())) return false;
		double Support = 0;
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::uint32_t Corner = 0; Corner < 3; ++Corner) {
			const Vec3 A = Mesh->Directions[Triangle[Corner]].Vector(), B = Mesh->Directions[Triangle[(Corner + 1) % 3]].Vector();
			Support = std::max(Support, (A - B).Length() * 1.5);
		}
		std::vector<double> Changes;
		Changes.reserve(Mesh->Directions.size());
		const Vec3 BasinCenter = UnitDirection::FromVector({ 0.3, 0.8, 0.5 })->Vector();
		for(const UnitDirection Direction : Mesh->Directions) {
			const Vec3 P = Direction.Vector();
			Changes.push_back(180 * std::exp(-80 * (1 - P.Dot(BasinCenter))) - 60 * std::exp(-40 * (1 - P.X)));
		}
		const double Started = FPlatformTime::Seconds();
		const std::optional<PlanetBasinReconstruction> Surface = PlanetBasinReconstruction::CreateGlobal(Level, Changes);
		const double Milliseconds = (FPlatformTime::Seconds() - Started) * 1000;
		// 全球標本を有限支持の連続面へ再構成の失敗時の返戻
		if(!TestTrue(TEXT("全球標本を有限支持の連続面へ再構成"), Surface.has_value())) return false;
		const std::vector<double> ConstantValues(Mesh->Directions.size(), 100);
		const std::optional<PlanetBasinReconstruction> Constant = PlanetBasinReconstruction::CreateGlobal(Level, ConstantValues);
		const std::optional<PlanetBasinReconstruction> RawConstant =
		PlanetBasinReconstruction::Create(Mesh->Directions, ConstantValues, Support);
		// 一様な盛土の正規化比較を準備の失敗時の返戻
		if(!TestTrue(TEXT("一様な盛土の正規化比較を準備"), Constant.has_value() && RawConstant.has_value())) return false;
		std::vector<UnitDirection> Constraints;
		for(std::size_t Index = 0; Index < 3; ++Index) {
			const std::array<std::uint32_t, 3> Triangle = Mesh->Triangles[Index];
			Constraints.push_back(
				*UnitDirection::FromVector(
					Mesh->Directions[Triangle[0]].Vector() + Mesh->Directions[Triangle[1]].Vector() + Mesh->Directions[Triangle[2]].Vector()
				)
			);
		}
		const std::array<double, 3> Minimum { 180, 160, 140 };
		const std::optional<PlanetBasinReconstruction> Raised = Constant->WithLowerBounds(Constraints, Minimum);
		// 重なった追加拘束の準備に失敗した場合の返戻
		if(!TestTrue(TEXT("有限幅の同じ面へ重なった下限を追加"), Raised.has_value())) return false;
		for(std::size_t Index = 0; Index < Constraints.size(); ++Index) {
			TestTrue(TEXT("後の下限処理が前の下限を壊さない"), Raised->EvaluateElevation(Constraints[Index]) >= Minimum[Index] - 1E-6);
			TestTrue(TEXT("不変な元の場を変更しない"), std::abs(Constant->EvaluateElevation(Constraints[Index]) - 100) <= 2E-6);
		}
		const std::optional<PlanetBasinReconstruction> Repeated = Raised->WithLowerBounds(Constraints, Minimum);
		// 達成済みの下限を再適用する正規経路の失敗時の返戻
		if(!TestTrue(TEXT("達成済みの下限を受理"), Repeated.has_value())) return false;
		for(const UnitDirection Direction : Mesh->Directions) {
			TestTrue(TEXT("支持内外の全固定標本で元の面を下げない"), Raised->EvaluateElevation(Direction) >= Constant->EvaluateElevation(Direction) - 1E-10);
			TestEqual(TEXT("同じ下限の再適用は形状を変更しない"), Repeated->EvaluateElevation(Direction), Raised->EvaluateElevation(Direction));
		}
		TestFalse(TEXT("正規化されて居ない部分面への下限追加を拒否"), RawConstant->WithLowerBounds(Constraints, Minimum).has_value());
		TestFalse(TEXT("下限の個数不一致を拒否"), Constant->WithLowerBounds(Constraints, {}).has_value());
		TestFalse(TEXT("下限追加の中断を拒否"), Constant->WithLowerBounds(Constraints, Minimum, Cancel.get_token()).has_value());
		TestFalse(TEXT("下限追加の非有限値を拒否"), Constant->WithLowerBounds(One, Nonfinite).has_value());
		for(const UnitDirection Direction : Constraints) {
			const Vec3 P = Direction.Vector(), Axis = std::abs(P.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
			const Vec3 Tangent = UnitDirection::FromVector(P.Cross(Axis))->Vector();
			for(const Vec3 Along : { Tangent, P.Cross(Tangent) }) {
				const auto Difference = [&](const double Step) {
					const UnitDirection Low = *UnitDirection::FromVector(P * std::cos(Step) - Along * std::sin(Step));
					const UnitDirection High = *UnitDirection::FromVector(P * std::cos(Step) + Along * std::sin(Step));
					// 補正後の面を両側で評価した独立差分の返戻
					return (Raised->EvaluateElevation(High) - Raised->EvaluateElevation(Low)) / (2 * Step);
				};
				const double Independent = (4 * Difference(5E-6) - Difference(1E-5)) / 3;
				const double Analytical = Raised->Evaluate(Direction).TangentialGradientPerRadian.Dot(Along);
				TestTrue(TEXT("下限追加後も同じ解析勾配と独立差分が一致"), std::abs(Independent - Analytical) <= 1E-5 * std::max(1.0, std::abs(Analytical)));
			}
		}
		double MaximumConstantError = 0, MaximumRawConstantError = 0;
		for(std::size_t Index = 0; Index < Mesh->Triangles.size(); Index += 137) {
			const std::array<std::uint32_t, 3> Triangle = Mesh->Triangles[Index];
			const UnitDirection At = *UnitDirection::FromVector(
				Mesh->Directions[Triangle[0]].Vector() + Mesh->Directions[Triangle[1]].Vector() + Mesh->Directions[Triangle[2]].Vector()
			);
			MaximumConstantError = std::max(MaximumConstantError, std::abs(Constant->EvaluateElevation(At) - 100));
			MaximumRawConstantError = std::max(MaximumRawConstantError, std::abs(RawConstant->EvaluateElevation(At) - 100));
		}
		TestTrue(TEXT("一様な盛土を標本間でも一定に保持"), MaximumConstantError <= 2E-6);
		TestTrue(TEXT("正規化前の標本間の規則的な起伏を実際に検出"), MaximumRawConstantError > 0.01);
		AddInfo(
			FString::Printf(
				TEXT("Basin constant reproduction: level=%u rawError=%.9fm normalizedError=%.12gm normalizedBound=%.9fm."),
				Level,
				MaximumRawConstantError,
				MaximumConstantError,
				Constant->Statistics().AbsoluteBoundMeters
			)
		);
		double MaximumResidual = 0, MaximumGradientError = 0;
		for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			MaximumResidual = std::max(MaximumResidual, std::abs(Surface->EvaluateElevation(Mesh->Directions[Index]) - Changes[Index]));
		}
		TestTrue(TEXT("実地点評価で全標本の絶対誤差1E−6 m以下"), MaximumResidual <= 1E-6);
		for(std::uint32_t Index = 0; Index < 192; ++Index) {
			const double Z = -1 + 2 * (Index + 0.37) / 192, Angle = Index * 2.399963229728653, Radial = std::sqrt(1 - Z * Z);
			const UnitDirection Direction = *UnitDirection::FromVector({ Radial * std::cos(Angle), Radial * std::sin(Angle), Z });
			const Vec3 P = Direction.Vector();
			const TerrainSample H = Surface->Evaluate(Direction), Reference = Surface->EvaluateReference(Direction);
			TestEqual(TEXT("勾配省略経路でも同じ値を維持"), Surface->EvaluateElevation(Direction), H.ElevationMeters);
			TestTrue(
				TEXT("索引と全点探索の値・勾配が一致"),
				std::abs(H.ElevationMeters - Reference.ElevationMeters) <= 1E-10 * std::max(1.0, std::abs(Reference.ElevationMeters)) &&
				(H.TangentialGradientPerRadian - Reference.TangentialGradientPerRadian).Length() <=
				1E-10 * std::max(1.0, Reference.TangentialGradientPerRadian.Length())
			);
			const Vec3 Axis = std::abs(P.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
			const Vec3 Tangent = UnitDirection::FromVector(P.Cross(Axis))->Vector();
			for(const Vec3 Along : { Tangent, P.Cross(Tangent) }) {
				const auto Difference = [&](const double Step) {
					const UnitDirection Before = *UnitDirection::FromVector(P * std::cos(Step) - Along * std::sin(Step));
					const UnitDirection After = *UnitDirection::FromVector(P * std::cos(Step) + Along * std::sin(Step));
					// 球面上の独立な中央差分の返戻
					return (Surface->EvaluateElevation(After) - Surface->EvaluateElevation(Before)) / (2 * Step);
				};
				const double Fine = Difference(5E-6), Coarse = Difference(1E-5), Independent = (4 * Fine - Coarse) / 3;
				const double Analytical = H.TangentialGradientPerRadian.Dot(Along);
				const double Error = std::abs(Analytical - Independent) / std::max(1.0, std::abs(Analytical));
				MaximumGradientError = std::max(MaximumGradientError, Error);
				TestTrue(TEXT("二つの独立な接線差分で解析勾配を検証"), Error <= 1E-5);
			}
			TestTrue(TEXT("解析勾配が接線面上に在る"), std::abs(H.TangentialGradientPerRadian.Dot(P)) <= 1E-8);
		}
		AddInfo(
			FString::Printf(
				TEXT("Basin reconstruction: level=%u sites=%u couplings=%llu iterations=%u ")
				TEXT("prepare=%.6fms nodalResidual=%.12gm gradientRelative=%.12g bound=%.6fm."),
				Level,
				Surface->Statistics().Sites,
				static_cast<unsigned long long>(Surface->Statistics().Couplings),
				Surface->Statistics().Iterations,
				Milliseconds,
				MaximumResidual,
				MaximumGradientError,
				Surface->Statistics().AbsoluteBoundMeters
			)
		);
	}
	// 補間・解析勾配・入力条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
