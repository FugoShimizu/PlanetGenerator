#include "Core/PlanetContinentalDescent.h"
#include "Core/PlanetSphericalContinentalDescent.h"
#include "Core/PlanetSphericalContinentalField.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSphericalContinentalDescentContracts,
	"PlanetGenerator.SphericalContinentalDescent.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ球面の高さ・逆写像・微分と，海岸迄下降する比較用縦断を照合する
 * @param Parameters 未使用の UE 引数
 * @return 有限区間の水位・河口・独立差分・入力条件が成立した場合に真
 */
bool FPlanetSphericalContinentalDescentContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::ContinentalDescentError;
	using PlanetCore::ContinentalDescentSettings;
	using PlanetCore::SphericalContinentalDescent;
	using PlanetCore::SphericalContinentalDescentError;
	using PlanetCore::SphericalContinentalDescentReason;
	using PlanetCore::SphericalContinentalDescentSample;
	using PlanetCore::SphericalContinentalField;
	using PlanetCore::SphericalContinentalFieldError;
	using PlanetCore::SphericalContinentalFieldSettings;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	std::uint64_t Paths = 0, Segments = 0, Samples = 0, Differences = 0, CurvatureEscapes = 0;
	double MaximumHeightDifference = 0, MaximumDirectionDifference = 0, MaximumDerivativeDifference = 0;
	double MaximumSpatialDerivativeDifference = 0, MaximumPositiveDerivative = 0, MaximumTangentRadial = 0, MaximumOutletGround = 0;
	double MaximumStartDirectionDifference = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		for(const std::uint64_t Seed : { UINT64_C(0), UINT64_C(7), UINT64_C(42), UINT64_MAX }) {
			SphericalContinentalFieldError FieldError = SphericalContinentalFieldError::None;
			std::optional<SphericalContinentalField> Prepared = SphericalContinentalField::CreateSeeded(Seed, Radius, FieldError);
			// 正規な球面レシピを準備出来ない場合の返戻
			if(!TestTrue(TEXT("下降用の正規な球面地形を受理"), Prepared.has_value())) return false;
			const std::shared_ptr<const SphericalContinentalField> Field =
			std::make_shared<const SphericalContinentalField>(std::move(*Prepared));
			const double Scale = std::min(1.0, Radius / 1000000), HeightLimit = 1E-6 * Scale;
			unsigned Accepted = 0;
			for(unsigned i = 0; i < 512 && Accepted < 8; ++i) {
				// 全緯度へ散った順序で，陸上だけの開始点を選ぶ
				const unsigned Index = (i * 53 + static_cast<unsigned>(Seed % 512)) % 512;
				const double Z = 1 - 2 * (Index + 0.5) / 512, Angle = Index * 2.399963229728653, Ring = std::sqrt(1 - Z * Z);
				const UnitDirection Start = *UnitDirection::FromVector({ Ring * std::cos(Angle), Ring * std::sin(Angle), Z });
				const std::optional<double> InitialHeight = Field->EvaluateElevation(Start);
				// 正常な球面方向を照会出来ない場合の返戻
				if(!TestTrue(TEXT("開始点の正常な標高照会"), InitialHeight.has_value())) return false;
				if(*InitialHeight < 0.001 * Scale) continue;
				ContinentalDescentSettings Settings;
				SphericalContinentalDescentError Error;
				const std::optional<SphericalContinentalDescent> Path = SphericalContinentalDescent::Create(Field, Start, Settings, Error);
				const bool HasPath = TestTrue(
					FString::Printf(
						TEXT("球面経路が海岸へ到達 Seed=%llu Radius=%.12g head=%u reason=%u disk=%u"),
						static_cast<unsigned long long>(Seed),
						Radius,
						Index,
						static_cast<unsigned>(Error.Reason),
						static_cast<unsigned>(Error.DiskReason)
					),
					Path.has_value()
				);
				// 元地形と失敗段階を確認した検査結果の返戻
				if(!HasPath) return false;
				const std::optional<SphericalContinentalDescentSample> First = Path->Evaluate(0, 0), Outlet = Path->EvaluateOutlet();
				// 陸の経路の始点又は河口を解像出来ない場合の返戻
				if(!TestTrue(TEXT("始点と同じ地形の河口を評価"), First && Outlet)) return false;
				MaximumStartDirectionDifference =
				std::max(MaximumStartDirectionDifference, (First->Direction.Vector() - Start.Vector()).Length());
				TestTrue(TEXT("開始点の高さを同じ場から保持"), std::abs(First->ElevationMeters - *InitialHeight) <= HeightLimit);
				TestTrue(TEXT("河口の平均水位は球状海面の正確な零"), !Outlet->ElevationMeters && !Outlet->ElevationDerivativePerFraction);
				const std::optional<double> OutletGround = Field->EvaluateElevation(Outlet->Direction);
				// 別の水面へ接続した候補を，海岸接続として扱わない返戻
				if(!TestTrue(TEXT("河口も同じ球面標高で照会"), OutletGround.has_value())) return false;
				MaximumOutletGround = std::max(MaximumOutletGround, std::abs(*OutletGround));
				TestTrue(TEXT("同じ球面地形の河岸へ閉じる"), std::abs(*OutletGround) <= HeightLimit);
				double PreviousHeight = *InitialHeight;
				for(std::size_t j = 0; j < Path->GetSegmentCount(); ++j) {
					for(unsigned k = 0; k < 9; ++k) {
						const double Fraction = k / 8.0;
						const std::optional<SphericalContinentalDescentSample> Sample = Path->Evaluate(j, Fraction);
						// 区間内の照会不足を端点の成功で隠さない返戻
						if(!TestTrue(TEXT("球面の区間内で有限な比較水位"), Sample.has_value())) return false;
						const std::optional<TerrainSample> Ground = Field->Evaluate(Sample->Direction);
						// 地形と比較縦断の不一致を計測出来ない場合の返戻
						if(!TestTrue(TEXT("同じ球面地形の区間内照会"), Ground.has_value())) return false;
						const double HeightDifference = std::abs(Ground->ElevationMeters - Sample->ElevationMeters);
						MaximumHeightDifference = std::max(MaximumHeightDifference, HeightDifference);
						MaximumPositiveDerivative = std::max(MaximumPositiveDerivative, Sample->ElevationDerivativePerFraction);
						MaximumTangentRadial =
						std::max(MaximumTangentRadial, std::abs(Sample->Direction.Vector().Dot(Sample->DirectionDerivativePerFraction)));
						const double Projected = Ground->TangentialGradientPerRadian.Dot(Sample->DirectionDerivativePerFraction);
						MaximumSpatialDerivativeDifference = std::max(
							MaximumSpatialDerivativeDifference,
							std::abs(Projected - Sample->ElevationDerivativePerFraction) / std::max(1.0, std::abs(Sample->ElevationDerivativePerFraction))
						);
						TestTrue(TEXT("区間内でも同じ球面標高を使う"), HeightDifference <= HeightLimit);
						TestTrue(TEXT("次の水位が上流より高くならない"), Sample->ElevationMeters <= PreviousHeight + HeightLimit);
						PreviousHeight = Sample->ElevationMeters;
						++Samples;
					}
					for(const double Fraction : { 0.25, 0.5, 0.75 }) {
						const std::optional<SphericalContinentalDescentSample> Center = Path->Evaluate(j, Fraction);
						// 独立差分の中心を解像出来ない場合の返戻
						if(!TestTrue(TEXT("差分の正規な区間中心"), Center.has_value())) return false;
						const auto Difference = [&](const double Delta) -> std::optional<std::pair<Vec3, double>> {
							const std::optional<SphericalContinentalDescentSample> Before = Path->Evaluate(j, Fraction - Delta);
							const std::optional<SphericalContinentalDescentSample> After = Path->Evaluate(j, Fraction + Delta);
							// 差分のどちらかを解像出来ない場合の返戻
							if(!Before || !After) return {};
							// 方向と高さの値だけを使った独立差分の返戻
							return std::pair<Vec3, double>{
								(After->Direction.Vector() - Before->Direction.Vector()) * (0.5 / Delta),
								(After->ElevationMeters - Before->ElevationMeters) / (2 * Delta)
							};
						};
						const std::optional<std::pair<Vec3, double>> Wide = Difference(1E-4), Narrow = Difference(5E-5);
						// 差分不足を相対誤差の NaN で隠さない返戻
						if(!TestTrue(TEXT("区間中心の両刻みの値差分"), Wide && Narrow)) return false;
						const Vec3 DirectionDerivative = (Narrow->first * 4 - Wide->first) * (1.0 / 3);
						const double HeightDerivative = (4 * Narrow->second - Wide->second) / 3;
						MaximumDirectionDifference =
						std::max(MaximumDirectionDifference, (DirectionDerivative - Center->DirectionDerivativePerFraction).Length());
						MaximumDerivativeDifference = std::max(
							MaximumDerivativeDifference,
							std::abs(HeightDerivative - Center->ElevationDerivativePerFraction) /
							std::max(1.0, std::abs(Center->ElevationDerivativePerFraction))
						);
						++Differences;
					}
					++Segments;
				}
				const std::optional<SphericalContinentalDescent> Again = SphericalContinentalDescent::Create(Field, Start, Settings, Error);
				// 同じ不変な場で正規経路を再準備出来ない場合の返戻
				if(!TestTrue(TEXT("同じ球面経路を再準備"), Again.has_value())) return false;
				TestTrue(
					TEXT("識別子と区間数が再現"),
					Again->GetDefinitionHash() == Path->GetDefinitionHash() && Again->GetSegmentCount() == Path->GetSegmentCount()
				);
				for(std::size_t j = 0; j < Path->GetSegmentCount(); ++j) {
					const std::optional<SphericalContinentalDescentSample> Before = Path->Evaluate(j, 0.375), After = Again->Evaluate(j, 0.375);
					TestTrue(
						TEXT("再準備の方向と水位が一致"),
						Before && After && !(Before->Direction.Vector() - After->Direction.Vector()).Length() &&
						Before->ElevationMeters == After->ElevationMeters &&
						Before->ElevationDerivativePerFraction == After->ElevationDerivativePerFraction
					);
				}
				CurvatureEscapes += Path->GetDiskStatistics().CurvatureEscapes;
				++Accepted;
				++Paths;
			}
			TestEqual(TEXT("各惑星で離れた８開始点を実際に検査"), Accepted, 8U);
		}
	}
	TestTrue(TEXT("元の開始方向と同じ球面位置を保持"), MaximumStartDirectionDifference < 1E-12);
	TestTrue(TEXT("区間全体の比較水位の微分が非正"), MaximumPositiveDerivative < 1E-8);
	TestTrue(TEXT("方向の微分は球面接線"), MaximumTangentRadial < 1E-12);
	TestTrue(TEXT("曲線の解析微分が独立方向差分と一致"), MaximumDirectionDifference < 1E-7);
	TestTrue(TEXT("比較水位の解析微分が独立高さ差分と一致"), MaximumDerivativeDifference < 1E-5);
	TestTrue(TEXT("球面勾配も同じ曲線の縦断微分と一致"), MaximumSpatialDerivativeDifference < 1E-7);
	// 山頂の零勾配・数値的海岸・拒否条件を解析的な１大陸で照合
	SphericalContinentalFieldSettings Settings;
	Settings.Domain.Warp.RadiusMeters = 1000;
	Settings.Domain.Warp.DerivativeBudget = 0;
	const double CapRadius = std::acos(0.3);
	Settings.Domain.Continents.push_back({ *UnitDirection::FromVector({ 1, 0, 0 }), CapRadius });
	PlanetCore::ContinentalPotentialSettings Uplift;
	Uplift.BackgroundMeters = 100;
	Settings.Uplifts.push_back(Uplift);
	Settings.PotentialReferenceMeters = 100;
	Settings.OceanDepthMeters = 40;
	SphericalContinentalFieldError FieldError = SphericalContinentalFieldError::None;
	std::optional<SphericalContinentalField> Prepared = SphericalContinentalField::Create(Settings, FieldError);
	// 解析的な正規比較を準備出来ない場合の返戻
	if(!TestTrue(TEXT("解析的な１大陸を正規経路で受理"), Prepared.has_value())) return false;
	std::shared_ptr<const SphericalContinentalField> Field =
	std::make_shared<const SphericalContinentalField>(std::move(*Prepared));
	ContinentalDescentSettings Controls;
	SphericalContinentalDescentError Error;
	const UnitDirection Summit = *UnitDirection::FromVector({ 1, 0, 0 });
	const std::optional<SphericalContinentalDescent> Peak = SphericalContinentalDescent::Create(Field, Summit, Controls, Error);
	TestTrue(TEXT("球面の山頂からも負曲率で海岸へ到達"), Peak && Peak->GetDiskStatistics().CurvatureEscapes > 0);
	const double CoastAngle = CapRadius - 8 * std::numeric_limits<double>::epsilon();
	const UnitDirection CoastDirection = *UnitDirection::FromVector({ std::cos(CoastAngle), std::sin(CoastAngle), 0 });
	std::optional<SphericalContinentalDescent> Coast = SphericalContinentalDescent::Create(Field, CoastDirection, Controls, Error);
	TestTrue(TEXT("数値的な海岸の正規経路も受理"), Coast && !Coast->GetSegmentCount() && Coast->EvaluateOutlet() && !Coast->Evaluate(0, 0));
	TestFalse(
		TEXT("大洋からの陸上流路を拒否"),
		SphericalContinentalDescent::Create(Field, Field->GetOceanAnchor(), Controls, Error).has_value()
	);
	TestTrue(TEXT("海側の拒否理由を保持"), Error.Reason == SphericalContinentalDescentReason::StartInOcean);
	TestFalse(TEXT("空の共有場を拒否"), SphericalContinentalDescent::Create({}, Summit, Controls, Error).has_value());
	TestTrue(TEXT("空の場の拒否理由を保持"), Error.Reason == SphericalContinentalDescentReason::InvalidField);
	Controls.MaximumSegments = 1;
	TestFalse(TEXT("到達しない円板経路を球面で公開しない"), SphericalContinentalDescent::Create(Field, Summit, Controls, Error).has_value());
	TestTrue(
		TEXT("円板の失敗段階と上限理由を保持"),
		Error.Reason == SphericalContinentalDescentReason::DiskPathRejected &&
		Error.DiskReason == ContinentalDescentError::SegmentBudget
	);
	Controls.MaximumSegments = 4096;
	std::stop_source Cancellation;
	Cancellation.request_stop();
	TestFalse(
		TEXT("中断した球面経路を公開しない"),
		SphericalContinentalDescent::Create(Field, Summit, Controls, Error, Cancellation.get_token()).has_value()
	);
	TestTrue(TEXT("球面準備の中断理由を保持"), Error.Reason == SphericalContinentalDescentReason::Cancelled);
	if(Peak) {
		TestFalse(TEXT("不存在の区間を拒否"), Peak->Evaluate(Peak->GetSegmentCount(), 0.5).has_value());
		TestFalse(TEXT("区間外の割合を拒否"), Peak->Evaluate(0, 1.001).has_value());
		TestFalse(TEXT("非有限な割合を拒否"), Peak->Evaluate(0, std::numeric_limits<double>::quiet_NaN()).has_value());
		Coast.reset();
		Field.reset();
		TestTrue(TEXT("元の所有者を解放しても曲線の場は生存"), Peak->Evaluate(0, 0.5).has_value());
	}
	AddInfo(
		FString::Printf(
			TEXT("Spherical continental descent: paths=%llu segments=%llu samples=%llu differences=%llu escapes=%llu ")
			TEXT("height=%.12g directionDerivative=%.12g heightDerivative=%.12g spatialDerivative=%.12g positiveDerivative=%.12g ")
			TEXT("radial=%.12g outletGround=%.12g startDirection=%.12g seconds=%.7f. ")
			TEXT("Base profiles only; erosion, channels, joins and water rendering are pending."),
			static_cast<unsigned long long>(Paths),
			static_cast<unsigned long long>(Segments),
			static_cast<unsigned long long>(Samples),
			static_cast<unsigned long long>(Differences),
			static_cast<unsigned long long>(CurvatureEscapes),
			MaximumHeightDifference,
			MaximumDirectionDifference,
			MaximumDerivativeDifference,
			MaximumSpatialDerivativeDifference,
			MaximumPositiveDerivative,
			MaximumTangentRadial,
			MaximumOutletGround,
			MaximumStartDirectionDifference,
			std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count()
		)
	);
	// 同じ球面・水位・微分・河口・許可と拒否経路の検査結果の返戻
	return !HasAnyErrors();
}

#endif
