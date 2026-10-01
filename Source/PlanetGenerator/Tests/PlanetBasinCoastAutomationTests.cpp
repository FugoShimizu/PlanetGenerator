#include "Core/PlanetBasinFill.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBasinCoastContracts, "PlanetGenerator.BasinReconstruction.CoastalLimit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 海岸の勾配増幅の回帰と，容量付き盛土の下限・微分・拒否条件を確認する
 * @param Parameters 未使用の UE 検査引数
 * @return 全ての拘束と独立差分が成立する場合に真
 */
bool FPlanetBasinCoastContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetBasinFill;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(2);
	// 検査する補間場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("海岸の比較網を準備"), Mesh.has_value())) return false;
	const std::vector<double> High(Mesh->Directions.size(), 1000), Floor(Mesh->Directions.size(), 900);
	const std::optional<PlanetBasinFill> Legacy = PlanetBasinFill::Create(2, High, Floor, 0, 1);
	const std::optional<PlanetBasinFill> Bounded = PlanetBasinFill::CreateWithCoastalLimit(2, High, Floor, 0, 1, 4);
	// 同じ補間面に対する比較を準備出来ない場合の返戻
	if(!TestTrue(TEXT("固定帯と容量付きの同じ補間面を準備"), Legacy.has_value() && Bounded.has_value())) return false;
	const UnitDirection Point = *UnitDirection::FromVector({ 1, 0, 0 });
	constexpr double Radius = 1000000, OriginalHeight = 0.22873290088, OriginalSlope = 0.0606352104086;
	const TerrainSample Original { OriginalHeight, { 0, OriginalSlope * Radius, 0 } };
	const TerrainSample Before = Legacy->Apply(Point, Original), After = Bounded->Apply(Point, Original);
	TestTrue(TEXT("固定帯に依る旧急勾配を隠さず再現"), Before.TangentialGradientPerRadian.Length() / Radius > 45);
	TestTrue(TEXT("元勾配の容量導関数に依る上限を保持"), After.TangentialGradientPerRadian.Length() / Radius <= 5.5 * OriginalSlope + 1E-8);
	const double Capacity = PlanetBasinFill::CoastalRaiseCapacity(OriginalHeight, 1, 4);
	TestTrue(TEXT("海岸の盛土が実際の容量以下"), After.ElevationMeters <= OriginalHeight + Capacity + 1E-12);
	double MaximumDerivativeError = 0;
	for(const double Height : { -1.0, 0.0, 0.01, 0.1, 0.22873290088, 1.0, 10.0, 100.0, 200.0, 500.0, 899.0, 901.0 }) {
		const TerrainSample Source { Height, { 0, OriginalSlope * Radius, 0 } };
		const TerrainSample Actual = Bounded->Apply(Point, Source);
		TestEqual(TEXT("値だけの経路と同じ容量付き標高"), Bounded->ApplyElevation(Point, Height), Actual.ElevationMeters);
		TestTrue(TEXT("全ての照会で非負の盛土"), Actual.ElevationMeters >= Height);
		if(Height <= 0.0) {
			TestEqual(TEXT("海面以下の標高を厳密に保持"), Actual.ElevationMeters, Height);
			TestEqual(TEXT("海面以下の勾配を厳密に保持"), Actual.TangentialGradientPerRadian.Y, Source.TangentialGradientPerRadian.Y);
		}
		// 角度と元標高を共に変える独立差分で，補間面だけを固定した微分との混同を避ける
		const auto Difference = [&](const double StepMeters) -> double {
			const double Angle = StepMeters / Radius;
			const UnitDirection A = *UnitDirection::FromVector({ std::cos(Angle), -std::sin(Angle), 0 });
			const UnitDirection B = *UnitDirection::FromVector({ std::cos(Angle), std::sin(Angle), 0 });
			const double Low = Bounded->ApplyElevation(A, Height - OriginalSlope * StepMeters);
			const double Upper = Bounded->ApplyElevation(B, Height + OriginalSlope * StepMeters);
			// 二つの独立した地点評価から求めた傾斜の返戻
			return (Upper - Low) / (2 * StepMeters);
		};
		const double Independent = (4 * Difference(0.005) - Difference(0.01)) / 3;
		const double Error = std::abs(Independent - Actual.TangentialGradientPerRadian.Y / Radius);
		MaximumDerivativeError = std::max(MaximumDerivativeError, Error);
		TestTrue(TEXT("海岸・線形部・飽和部の解析勾配が独立差分と一致"), Error <= 1E-6);
	}
	// 補間面が平坦な場合，元斜面の向きを接続だけで反転させない
	const std::vector<double> BlendSources(Mesh->Directions.size(), 100), BlendFloor(Mesh->Directions.size(), 50);
	const std::optional<PlanetBasinFill> Smooth = PlanetBasinFill::CreateWithCoastalLimit(2, BlendSources, BlendFloor, 0, 1, 4);
	// 接続幅の独立検査を準備出来ない場合の返戻
	if(!TestTrue(TEXT("平坦な補間面の接続検査を準備"), Smooth.has_value())) return false;
	constexpr double OldFraction = 0.6;
	const double OldWeight = OldFraction * OldFraction * (18 - 32 * OldFraction + 15 * OldFraction * OldFraction);
	TestTrue(TEXT("旧１µm接続式の逆勾配を再現"), 0.1 * (1 - OldWeight) < -0.05);
	double MaximumBlendError = 0, MaximumOvershoot = 0;
	for(const double Delta : { -1.5, -1.0, -0.999, -0.5, -0.0000006, 0.0, 0.0000006, 0.5, 0.999, 1.0, 1.5 }) {
		const double Height = 50 - Delta;
		const TerrainSample Source { Height, { 0, 0.1 * Radius, 0.07 * Radius } };
		const TerrainSample Actual = Smooth->Apply(Point, Source);
		const double Excess = Actual.ElevationMeters - std::max(Height, 50.0);
		MaximumOvershoot = std::max(MaximumOvershoot, Excess);
		TestTrue(TEXT("両面の高い方を下回らない接続"), Excess >= -2E-6);
		TestTrue(TEXT("平滑化の標高超過が公開上限以下"), Excess <= PlanetBasinFill::SmoothingOvershootBoundMeters() + 2E-6);
		TestEqual(TEXT("平滑化の値経路と勾配経路が一致"), Smooth->ApplyElevation(Point, Height), Actual.ElevationMeters);
		for(const Vec3 Along : { Vec3{ 0, 1, 0 }, Vec3{ 0, 0, 1 } }) {
			const double SourceSlope = Source.TangentialGradientPerRadian.Dot(Along) / Radius;
			const double Slope = Actual.TangentialGradientPerRadian.Dot(Along) / Radius;
			TestTrue(TEXT("平坦面との接続で斜面方向を反転させない"), Slope >= -1E-8 && Slope <= SourceSlope + 1E-8);
			const auto Difference = [&](const double StepMeters) -> double {
				const double Angle = StepMeters / Radius;
				const Vec3 Center = Point.Vector() * std::cos(Angle), Offset = Along * std::sin(Angle);
				const UnitDirection Low = *UnitDirection::FromVector(Center - Offset), High = *UnitDirection::FromVector(Center + Offset);
				// 補間と元地形を共に独立評価した二方向の差分の返戻
				return (
					Smooth->ApplyElevation(High, Height + SourceSlope * StepMeters) - Smooth->ApplyElevation(Low, Height - SourceSlope * StepMeters)
				) / (2 * StepMeters);
			};
			const double Independent = (4 * Difference(0.005) - Difference(0.01)) / 3, Error = std::abs(Independent - Slope);
			MaximumBlendError = std::max(MaximumBlendError, Error);
			TestTrue(TEXT("接続の両端と内部で独立差分に一致"), Error <= 1E-6);
		}
	}
	// 零から接続幅を越える盛土迄，初期拘束の逆変換を同じ許容値で確認する
	const std::vector<double> NearSources(Mesh->Directions.size(), 50);
	std::vector<double> NearMinima;
	for(std::size_t Index = 0; Index < NearSources.size(); ++Index) {
		const std::array<double, 8> Raises { -0.5, 0, 1E-5, 0.01, 0.1, 0.5, 1, 1.5 };
		NearMinima.push_back(50 + Raises[Index % Raises.size()]);
	}
	const std::optional<PlanetBasinFill> Near = PlanetBasinFill::CreateWithCoastalLimit(2, NearSources, NearMinima, 0, 1, 4);
	// 小さな盛土の正規入力を黙って拒否しない返戻
	if(!TestTrue(TEXT("零近傍の盛土を逆変換して準備"), Near.has_value())) return false;
	for(std::size_t Index = 0; Index < NearSources.size(); ++Index) {
		const double Expected = std::max(50.0, NearMinima[Index]);
		TestTrue(TEXT("平滑化後も初期標本の指定高さを保持"), std::abs(Near->ApplyElevation(Mesh->Directions[Index], 50) - Expected) <= 2E-6);
	}
	AddInfo(
		FString::Printf(
			TEXT("Basin smooth envelope: oldSlope=%.12g overshoot=%.12gm bound=%.12gm derivativeError=%.12g."),
			0.1 * (1 - OldWeight),
			MaximumOvershoot,
			PlanetBasinFill::SmoothingOvershootBoundMeters(),
			MaximumBlendError
		)
	);
	for(const double Height : { 0.25, 10.0, 100.0 }) {
		const double Available = PlanetBasinFill::CoastalRaiseCapacity(Height, 1, 4);
		const std::vector<double> Sources(Mesh->Directions.size(), Height);
		std::vector<double> Minima;
		for(std::size_t Index = 0; Index < Sources.size(); ++Index) {
			const std::array<double, 6> Fractions { 0.1, 0.3, 0.5, 0.7, 0.9, 1 };
			Minima.push_back(Height + Available * Fractions[Index % Fractions.size()]);
		}
		const std::optional<PlanetBasinFill> Prepared = PlanetBasinFill::CreateWithCoastalLimit(2, Sources, Minima, 0, 1, 4);
		// 非線形な逆変換を含む正規の入力を準備出来ない場合の返戻
		if(!TestTrue(TEXT("容量内の下限を初期補間で再現"), Prepared.has_value())) return false;
		for(std::size_t Index = 0; Index < Sources.size(); ++Index) {
			TestTrue(TEXT("初期標本の指定した下限と一致"), std::abs(Prepared->ApplyElevation(Mesh->Directions[Index], Height) - Minima[Index]) <= 2E-6);
		}
		Minima[0] = Height + Available + 0.01;
		TestFalse(TEXT("容量を越える下限を黙って切詰めず拒否"), PlanetBasinFill::CreateWithCoastalLimit(2, Sources, Minima, 0, 1, 4).has_value());
	}
	const std::vector<double> Ones(Mesh->Directions.size(), 1);
	std::optional<PlanetBasinFill> Updated = PlanetBasinFill::CreateWithCoastalLimit(2, Ones, Ones, 0, 1, 4);
	// 追加拘束を保持する初期場が無い場合の返戻
	if(!TestTrue(TEXT("追加拘束用の場を準備"), Updated.has_value())) return false;
	std::vector<UnitDirection> Retained;
	std::vector<double> Required;
	for(unsigned Index = 0; Index < 12; ++Index) {
		const UnitDirection Site = Mesh->Directions[Index * 7];
		const double Minimum = 1 + 2 * (0.6 + 0.02 * Index);
		const std::array<UnitDirection, 1> Sites { Site };
		const std::array<double, 1> Originals { 1 }, Minima { Minimum };
		Updated = Updated->WithMinimumElevations(Sites, Originals, Minima);
		// 容量内の追加拘束を適用出来ない場合の返戻
		if(!TestTrue(TEXT("容量内の追加下限を適用"), Updated.has_value())) return false;
		Retained.push_back(Site);
		Required.push_back(Minimum);
		for(std::size_t Previous = 0; Previous < Retained.size(); ++Previous) {
			TestTrue(TEXT("後の更新でも以前の下限を保持"), Updated->ApplyElevation(Retained[Previous], 1) >= Required[Previous] - 2E-6);
		}
	}
	const std::array<UnitDirection, 1> Sites { Point };
	const std::array<double, 1> Sources { 1 }, Impossible { 3.01 };
	TestFalse(TEXT("追加拘束でも容量超過を拒否"), Updated->WithMinimumElevations(Sites, Sources, Impossible).has_value());
	TestFalse(TEXT("相対容量０で比較用の旧式へ戻さない"), PlanetBasinFill::CreateWithCoastalLimit(2, High, Floor, 0, 1, 0).has_value());
	TestTrue(
		TEXT("非有限な容量定義を水位へ渡さない"),
		std::isnan(PlanetBasinFill::CoastalRaiseCapacity(1, 1, std::numeric_limits<double>::infinity()))
	);
	TestTrue(TEXT("容量の数値溢れも拒否"), std::isnan(PlanetBasinFill::CoastalRaiseCapacity(1E308, 1, 4)));
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(
		TEXT("容量付き準備の中断を拒否"),
		PlanetBasinFill::CreateWithCoastalLimit(2, High, Floor, 0, 1, 4, Cancel.get_token()).has_value()
	);
	AddInfo(
		FString::Printf(
			TEXT("Basin coastal operator: oldSlope=%.12g boundedSlope=%.12g oldHeight=%.12gm ")
			TEXT("boundedHeight=%.12gm capacity=%.12gm derivativeError=%.12g."),
			Before.TangentialGradientPerRadian.Length() / Radius,
			After.TangentialGradientPerRadian.Length() / Radius,
			Before.ElevationMeters,
			After.ElevationMeters,
			Capacity,
			MaximumDerivativeError
		)
	);
	// 修正した式の限定検査であり，実惑星の全排水を合格としない返戻
	return !HasAnyErrors();
}

#endif
