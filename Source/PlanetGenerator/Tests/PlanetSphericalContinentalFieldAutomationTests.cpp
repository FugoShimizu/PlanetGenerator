#include "Core/PlanetConnectedOceanDomain.h"
#include "Core/PlanetContinentalPotential.h"
#include "Core/PlanetCore.h"
#include "Core/PlanetSphericalContinentalField.h"
#include "Core/PlanetSphericalWarp.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSphericalContinentalFieldContracts,
	"PlanetGenerator.SphericalContinentalField.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

namespace {
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;

	/**
	 * 値だけの大円差分に使う独立な接線を準備する
	 * @param Point 単位球面の方向
	 * @return 正規直交する二つの球面接線
	 */
	std::array<Vec3, 2> Tangents(const Vec3 Point) {
		const Vec3 Axis = std::abs(Point.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 };
		const Vec3 East = UnitDirection::FromVector(Axis.Cross(Point))->Vector();
		// 同じ局所平面に在る二つの独立接線の返戻
		return { East, Point.Cross(East) };
	}

	/**
	 * 原方向から大円に沿って角度だけ移動する
	 * @param Direction 元の方向
	 * @param Tangent 元方向と直交する単位接線
	 * @param Angle 移動角
	 * @return 差分に使う正規方向
	 */
	UnitDirection Offset(const UnitDirection Direction, const Vec3 Tangent, const double Angle) {
		return *UnitDirection::FromVector(Direction.Vector() * std::cos(Angle) + Tangent * std::sin(Angle));
	}
}

/**
 * 海域の構成と高さの合成，連鎖微分，海岸の接続，正規入力と拒否を照合する
 * @param Parameters 未使用の UE 引数
 * @return 有限個の数学的・数値的な条件の合格，自然さ又は全河川の完成証明とは別
 */
bool FPlanetSphericalContinentalFieldContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::ConnectedOceanDomain;
	using PlanetCore::ConnectedOceanDomainError;
	using PlanetCore::ContinentalDomainCap;
	using PlanetCore::ContinentalPotentialSettings;
	using PlanetCore::PlanetSphericalWarp;
	using PlanetCore::SphereMesh;
	using PlanetCore::SphericalContinentalField;
	using PlanetCore::SphericalContinentalFieldError;
	using PlanetCore::SphericalContinentalFieldSettings;
	using PlanetCore::TerrainSample;
	// 独立な閉球面と非交差大陸の検査用条件
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const std::optional<SphereMesh> Mesh = PlanetCore::BuildUniformSphere(4);
	// 固定の球面検査資料を準備出来ない場合の返戻
	if(!TestTrue(TEXT("検査用の閉球面を準備"), Mesh.has_value())) return false;
	std::vector<std::vector<std::uint32_t>> Adjacency(Mesh->Directions.size());
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(unsigned i = 0; i < 3; ++i) {
		const std::uint32_t First = Triangle[i], Second = Triangle[(i + 1) % 3];
		Adjacency[First].push_back(Second);
		Adjacency[Second].push_back(First);
	}
	// 検査用の大陸と集計項目の準備
	const double CapRadius = std::acos(1 - 2 * 0.35 / 6);
	const std::array<Vec3, 6> Centers {
		Vec3{ 1, 0, 0 },
		Vec3{ -1, 0, 0 },
		Vec3{ 0, 1, 0 },
		Vec3{ 0, -1, 0 },
		Vec3{ 0, 0, 1 },
		Vec3{ 0, 0, -1 }
	};
	std::uint64_t HeightSamples = 0, GradientSamples = 0, CoastSamples = 0, DownhillSamples = 0;
	double MaximumGradientError = 0, MaximumRadialError = 0, MaximumCoastHeight = 0, MaximumCoastGradient = 0, MaximumAreaError = 0;
	double MaximumCoastHeightRatio = 0, MaximumCoastGradientRatio = 0;
	SphericalContinentalFieldSettings LastSettings;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		for(const std::uint64_t Seed : { UINT64_C(0), UINT64_C(7), UINT64_C(42), UINT64_MAX }) {
			SphericalContinentalFieldSettings Settings;
			Settings.Domain.Warp.Seed = Seed;
			Settings.Domain.Warp.RadiusMeters = Radius;
			Settings.Domain.Warp.LargestWavelengthMeters = 0.3 * Radius;
			Settings.Domain.Warp.ShouldUseContinentalWavelengths = Seed == 42 || Seed == UINT64_MAX;
			if(Settings.Domain.Warp.ShouldUseContinentalWavelengths) {
				Settings.Domain.Warp.LargestWavelengthMeters = 6.283185307179586 * Radius;
				Settings.Domain.Warp.DerivativeBudget = 8;
			}
			const double Scale = std::min(1.0, Radius / 1000000);
			Settings.PotentialReferenceMeters = 1000 * Scale;
			Settings.OceanDepthMeters = std::min(4000.0, 0.04 * Radius);
			for(const Vec3 Center : Centers) {
				Settings.Domain.Continents.push_back({ *UnitDirection::FromVector(Center), CapRadius });
				ContinentalPotentialSettings Uplift;
				Uplift.BackgroundMeters = 40 * Scale;
				Uplift.Sources = { { 0.35, 0.12, 0.16, 180 * Scale }, { -0.25, -0.28, 0.18, 300 * Scale }, { 0.05, 0.52, 0.12, 160 * Scale } };
				Settings.Uplifts.push_back(Uplift);
			}
			SphericalContinentalFieldError Error = SphericalContinentalFieldError::None;
			ConnectedOceanDomainError DomainError = ConnectedOceanDomainError::None;
			const std::optional<SphericalContinentalField> Field = SphericalContinentalField::Create(Settings, Error);
			const std::optional<SphericalContinentalField> Repeated = SphericalContinentalField::Create(Settings, Error);
			const std::optional<ConnectedOceanDomain> Domain = ConnectedOceanDomain::Create(Settings.Domain, DomainError);
			const std::optional<PlanetSphericalWarp> Warp = PlanetSphericalWarp::Create(Settings.Domain.Warp);
			// 正規入力が同じ準備経路を通らない場合の返戻
			if(!TestTrue(TEXT("正規の球面合成を受理"), Field && Repeated && Domain && Warp)) return false;
			TestTrue(TEXT("正常な準備理由"), Error == SphericalContinentalFieldError::None);
			TestEqual(TEXT("全大陸と全源を保持"), Field->GetStatistics().SourceCount, std::size_t{ 18 });
			TestTrue(TEXT("解析的な海域比が６５％"), std::abs(Field->GetStatistics().OceanFraction - 0.65) < 1E-14);
			TestEqual(TEXT("全準備入力の識別子が一致"), Field->GetDefinitionHash(), Repeated->GetDefinitionHash());
			TestEqual(TEXT("基準半径を維持"), Field->GetRadiusMeters(), Radius);
			TestTrue(TEXT("同じ大洋基準点は海面下"), Field->EvaluateElevation(Field->GetOceanAnchor()).value_or(0) < 0);
			// 高さの符号・面積・独立差分・下降方向の照合
			unsigned Wet = 0;
			for(unsigned i = 0; i < 4096; ++i) {
				const double Z = 1 - 2 * (i + 0.5) / 4096, Angle = i * 2.399963229728653, Ring = std::sqrt(1 - Z * Z);
				const UnitDirection Direction = *UnitDirection::FromVector({ Ring * std::cos(Angle), Ring * std::sin(Angle), Z });
				const std::optional<TerrainSample> Sample = Field->Evaluate(Direction), Again = Repeated->Evaluate(Direction);
				const std::optional<double> Height = Field->EvaluateElevation(Direction);
				// 有効な球面方向を照会出来ない場合の返戻
				if(!TestTrue(TEXT("全球の正常な高さ照会"), Sample && Again && Height)) return false;
				const bool IsWet = Domain->ContainsOcean(Direction);
				TestTrue(TEXT("独立な領域判定と標高の符号が一致"), Sample->ElevationMeters < 0 == IsWet);
				TestEqual(TEXT("高さだけの経路もビット一致"), std::bit_cast<std::uint64_t>(*Height), std::bit_cast<std::uint64_t>(Sample->ElevationMeters));
				TestEqual(TEXT("再準備後の高さがビット一致"), std::bit_cast<std::uint64_t>(*Height), std::bit_cast<std::uint64_t>(Again->ElevationMeters));
				TestTrue(TEXT("再準備後の勾配が一致"), !(Sample->TangentialGradientPerRadian - Again->TangentialGradientPerRadian).Length());
				TestTrue(TEXT("保守的な高さ上界を保持"), std::abs(*Height) <= Field->GetStatistics().AbsoluteReliefBoundMeters);
				const double GradientLength = Sample->TangentialGradientPerRadian.Length();
				MaximumRadialError = std::max(
					MaximumRadialError,
					std::abs(Sample->TangentialGradientPerRadian.Dot(Direction.Vector())) / std::max(1.0, GradientLength)
				);
				Wet += IsWet ? 1 : 0;
				++HeightSamples;
				if(i % 16) continue;
				const double DifferenceScale = Settings.Domain.Warp.ShouldUseContinentalWavelengths ? 64 : 1;
				const double Step = std::min(1E-5, 0.001 / (Warp->GetMaximumFrequency() * DifferenceScale));
				for(const Vec3 Tangent : Tangents(Direction.Vector())) {
					const auto Difference = [&](const double Delta) -> double {
						const double Before =
						Field->EvaluateElevation(Offset(Direction, Tangent, -Delta)).value_or(std::numeric_limits<double>::quiet_NaN());
						const double After =
						Field->EvaluateElevation(Offset(Direction, Tangent, Delta)).value_or(std::numeric_limits<double>::quiet_NaN());
						// 高さ照会だけで得た独立な接線微分の返戻
						return (After - Before) / (2 * Delta);
					};
					const double Numeric = (4 * Difference(Step * 0.5) - Difference(Step)) / 3;
					const double Actual = Sample->TangentialGradientPerRadian.Dot(Tangent);
					const double RelativeError = std::abs(Numeric - Actual) / std::max(1.0, GradientLength);
					// 差分失敗を最大値への NaN 混入で隠さない返戻
					if(!TestTrue(TEXT("高さだけの独立差分が有限"), std::isfinite(RelativeError))) return false;
					MaximumGradientError = std::max(MaximumGradientError, RelativeError);
					++GradientSamples;
				}
				if(IsWet || GradientLength < 1E-8 || Sample->ElevationMeters < 1E-10) continue;
				const Vec3 Downhill = Sample->TangentialGradientPerRadian * (-1 / GradientLength);
				const double DownStep = std::min(Step, 0.01 * Sample->ElevationMeters / GradientLength);
				const std::optional<double> Lower = Field->EvaluateElevation(Offset(Direction, Downhill, DownStep));
				TestTrue(TEXT("単調変換後も解析下降方向へ高さが下がる"), Lower && *Lower < Sample->ElevationMeters);
				++DownhillSamples;
			}
			MaximumAreaError = std::max(MaximumAreaError, std::abs(static_cast<double>(Wet) / 4096 - 0.65));
			// 有限球面メッシュ上での海域成分の照合，連続場の証明とは区別
			std::vector<bool> Visited(Mesh->Directions.size());
			std::vector<std::uint32_t> Queue;
			unsigned Components = 0;
			for(std::size_t i = 0; i < Mesh->Directions.size(); ++i) {
				if(Visited[i] || Field->EvaluateElevation(Mesh->Directions[i]).value_or(0) >= 0) continue;
				++Components;
				Queue.clear();
				Queue.push_back(static_cast<std::uint32_t>(i));
				Visited[i] = true;
				for(std::size_t j = 0; j < Queue.size(); ++j) for(const std::uint32_t Next : Adjacency[Queue[j]]) {
					if(Visited[Next] || Field->EvaluateElevation(Mesh->Directions[Next]).value_or(0) >= 0) continue;
					Visited[Next] = true;
					Queue.push_back(Next);
				}
			}
			TestEqual(TEXT("実標高の有限球面海域も１成分"), Components, 1U);
			// 全大陸の海岸で２階迄零へつながる尺度の照合
			const double Width = Domain->Statistics().MinimumCoastWidthRadians;
			for(const ContinentalDomainCap &Cap : Settings.Domain.Continents) {
				const Vec3 Center = Cap.Center.Vector();
				const std::array<Vec3, 2> Axes = Tangents(Center);
				for(unsigned i = 0; i < 8; ++i) {
					const double Azimuth = i * std::numbers::pi_v<double> * 0.25;
					const Vec3 Along = Axes[0] * std::cos(Azimuth) + Axes[1] * std::sin(Azimuth);
					const UnitDirection Original =
					*UnitDirection::FromVector(Center * std::cos(Cap.RadiusRadians) + Along * std::sin(Cap.RadiusRadians));
					const std::optional<TerrainSample> Coast = Field->Evaluate(Warp->ToWorld(Original));
					// 正規の海岸を評価出来ない場合の返戻
					if(!TestTrue(TEXT("海岸の正規入力を受理"), Coast.has_value())) return false;
					MaximumCoastHeight = std::max(MaximumCoastHeight, std::abs(Coast->ElevationMeters));
					MaximumCoastGradient = std::max(MaximumCoastGradient, Coast->TangentialGradientPerRadian.Length());
					for(const double Side : { -1.0, 1.0 }) {
						std::array<double, 2> Heights {}, Gradients {};
						for(unsigned j = 0; j < 2; ++j) {
							const double Delta = Side * Width * 1E-4 * (j ? 0.5 : 1);
							const UnitDirection NearSource =
							*UnitDirection::FromVector(Center * std::cos(Cap.RadiusRadians + Delta) + Along * std::sin(Cap.RadiusRadians + Delta));
							const std::optional<TerrainSample> Near = Field->Evaluate(Warp->ToWorld(NearSource));
							// 有効な海岸近傍を評価出来ない場合の返戻
							if(!TestTrue(TEXT("海岸両側の正常な照会"), Near.has_value())) return false;
							Heights[j] = std::abs(Near->ElevationMeters);
							Gradients[j] = Near->TangentialGradientPerRadian.Length();
						}
						MaximumCoastHeightRatio = std::max(MaximumCoastHeightRatio, Heights[1] / Heights[0]);
						MaximumCoastGradientRatio = std::max(MaximumCoastGradientRatio, Gradients[1] / Gradients[0]);
					}
					++CoastSamples;
				}
			}
			LastSettings = Settings;
		}
	}
	TestTrue(TEXT("独立な球面勾配と一致"), MaximumGradientError < 1E-5);
	TestTrue(TEXT("解析勾配は球面接線"), MaximumRadialError < 1E-12);
	TestTrue(TEXT("有限標本での面積が構成値と一致"), MaximumAreaError < 0.015);
	TestTrue(TEXT("海岸の高さは零へ接続"), MaximumCoastHeight < 1E-12);
	TestTrue(TEXT("海岸の勾配は零へ接続"), MaximumCoastGradient < 1E-7);
	TestTrue(TEXT("半分の距離で海岸高さは三乗の尺度"), MaximumCoastHeightRatio < 0.13);
	TestTrue(TEXT("半分の距離で海岸勾配は二乗の尺度"), MaximumCoastGradientRatio < 0.26);
	// 対称な背景だけを使う独立の球面閉形式
	SphericalContinentalFieldSettings AnalyticSettings = LastSettings;
	AnalyticSettings.Domain.Warp.DerivativeBudget = 0;
	for(ContinentalPotentialSettings &Uplift : AnalyticSettings.Uplifts) {
		Uplift.Sources.clear();
		Uplift.BackgroundMeters = 100;
	}
	SphericalContinentalFieldError Error = SphericalContinentalFieldError::None;
	const std::optional<SphericalContinentalField> Analytic = SphericalContinentalField::Create(AnalyticSettings, Error);
	// 独立の閉形式を持つ正規の設定が拒否された場合の返戻
	if(!TestTrue(TEXT("背景だけの正規経路を受理"), Analytic.has_value())) return false;
	for(unsigned i = 0; i < 33; ++i) {
		const double Angle = CapRadius * i / 32, Ratio = std::tan(0.5 * Angle) / std::tan(0.5 * CapRadius);
		const double Potential = 100 * (1 - Ratio * Ratio), Expected = Potential * Potential * Potential / 1000000;
		const UnitDirection Direction = *UnitDirection::FromVector({ std::cos(Angle), std::sin(Angle), 0 });
		const std::optional<double> Height = Analytic->EvaluateElevation(Direction);
		TestTrue(TEXT("独立の背景閉形式と標高が一致"), Height && std::abs(*Height - Expected) < 1E-12);
	}
	// Seed レシピを，対称な解析資料と別に検査
	unsigned SeededPlanets = 0;
	std::uint64_t LayeredGradientSamples = 0;
	double MinimumSeededPeak = std::numeric_limits<double>::infinity(), MaximumSeededPeak = 0, MaximumLayeredGradientError = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		for(const std::uint64_t Seed : { UINT64_C(0), UINT64_C(7), UINT64_C(42), UINT64_MAX }) {
			const std::optional<SphericalContinentalField> Seeded = SphericalContinentalField::CreateSeeded(Seed, Radius, Error);
			const std::optional<SphericalContinentalField> Again = SphericalContinentalField::CreateSeeded(Seed, Radius, Error);
			// 正規 Seed でレシピの前提を満たせない場合の返戻
			if(!TestTrue(TEXT("非対称レシピが正規経路を通る"), Seeded && Again)) return false;
			TestEqual(TEXT("レシピの準備が再現"), Seeded->GetDefinitionHash(), Again->GetDefinitionHash());
			TestEqual(TEXT("全隆起帯の源を保持"), Seeded->GetStatistics().SourceCount, std::size_t{ 504 });
			TestEqual(TEXT("全山塊の形状変形を保持"), Seeded->GetStatistics().ShapeTwistCount, std::size_t{ 36 });
			TestTrue(TEXT("レシピの解析的海域比を維持"), std::abs(Seeded->GetStatistics().OceanFraction - 0.65) < 1E-14);
			std::vector<bool> Visited(Mesh->Directions.size());
			std::vector<std::uint32_t> Queue;
			unsigned Components = 0;
			double Peak = 0;
			for(std::size_t i = 0; i < Mesh->Directions.size(); ++i) {
				const std::optional<double> Height = Seeded->EvaluateElevation(Mesh->Directions[i]);
				// レシピの全球照会を解決出来ない場合の返戻
				if(!TestTrue(TEXT("レシピの全球高さが有限"), Height && std::isfinite(*Height))) return false;
				if(!(i % 128) && *Height > 1E-6) {
					const std::optional<TerrainSample> Sample = Seeded->Evaluate(Mesh->Directions[i]);
					// 二層 Seed の正規な解析微分を得られない場合の返戻
					if(!TestTrue(TEXT("二層Seedの解析標本を取得"), Sample.has_value())) return false;
					constexpr double Step = 1E-7;
					for(const Vec3 Tangent : Tangents(Mesh->Directions[i].Vector())) {
						const auto Difference = [&](const double Delta) -> double {
							const std::optional<double> Before = Seeded->EvaluateElevation(Offset(Mesh->Directions[i], Tangent, -Delta));
							const std::optional<double> After = Seeded->EvaluateElevation(Offset(Mesh->Directions[i], Tangent, Delta));
							// 独立な高さ照会から中央差分を返戻
							return Before && After ? (*After - *Before) / (2 * Delta) : std::numeric_limits<double>::quiet_NaN();
						};
						const double Numeric = (4 * Difference(Step * 0.5) - Difference(Step)) / 3;
						const double Actual = Sample->TangentialGradientPerRadian.Dot(Tangent);
						const double RelativeError = std::abs(Numeric - Actual) / std::max(1.0, Sample->TangentialGradientPerRadian.Length());
						// 非有限な差分を最大値の更新で隠さない返戻
						if(!TestTrue(TEXT("二層Seedの独立差分が有限"), std::isfinite(RelativeError))) return false;
						MaximumLayeredGradientError = std::max(MaximumLayeredGradientError, RelativeError);
						++LayeredGradientSamples;
					}
				}
				Peak = std::max(Peak, *Height);
				if(Visited[i] || *Height >= 0) continue;
				++Components;
				Queue.clear();
				Queue.push_back(static_cast<std::uint32_t>(i));
				Visited[i] = true;
				for(std::size_t j = 0; j < Queue.size(); ++j) for(const std::uint32_t Next : Adjacency[Queue[j]]) {
					if(Visited[Next] || Seeded->EvaluateElevation(Mesh->Directions[Next]).value_or(0) >= 0) continue;
					Visited[Next] = true;
					Queue.push_back(Next);
				}
			}
			TestEqual(TEXT("レシピの実標高海域も有限網上で１成分"), Components, 1U);
			MinimumSeededPeak = std::min(MinimumSeededPeak, Peak / std::min(1.0, Radius / 1000000));
			MaximumSeededPeak = std::max(MaximumSeededPeak, Peak / std::min(1.0, Radius / 1000000));
			++SeededPlanets;
		}
	}
	TestTrue(TEXT("二層Seedの解析勾配と独立差分が一致"), LayeredGradientSamples > 0 && MaximumLayeredGradientError < 1E-4);
	AddInfo(
		FString::Printf(
			TEXT("Seeded continental recipe: planets=%u sourcesPerPlanet=504 gradientSamples=%llu ")
			TEXT("gradientError=%.12g scaledPeakMin=%.12g scaledPeakMax=%.12g."),
			SeededPlanets,
			static_cast<unsigned long long>(LayeredGradientSamples),
			MaximumLayeredGradientError,
			MinimumSeededPeak,
			MaximumSeededPeak
		)
	);
	// 同一の隆起層を二重化しても球面の最終高さを変えない
	SphericalContinentalFieldSettings DuplicateLayers = LastSettings;
	for(const ContinentalPotentialSettings &Uplift : LastSettings.Uplifts) {
		DuplicateLayers.UpliftLayers.push_back({ Uplift, Uplift });
	}
	DuplicateLayers.Uplifts.clear();
	DuplicateLayers.BlendTemperatureMeters = 25;
	const std::optional<SphericalContinentalField> BaseField = SphericalContinentalField::Create(LastSettings, Error);
	const std::optional<SphericalContinentalField> DuplicateField = SphericalContinentalField::Create(DuplicateLayers, Error);
	// 基準と二層の両方が生成出来ない場合の返戻
	if(!TestTrue(TEXT("同じ二層の球面統合を受理"), BaseField && DuplicateField)) return false;
	for(std::size_t i = 0; i < 128; ++i) {
		const std::optional<double> BaseHeight = BaseField->EvaluateElevation(Mesh->Directions[i]);
		const std::optional<double> DuplicateHeight = DuplicateField->EvaluateElevation(Mesh->Directions[i]);
		TestTrue(TEXT("同一層の合成は元の球面標高を保つ"), BaseHeight && DuplicateHeight && *BaseHeight == *DuplicateHeight);
	}
	SphericalContinentalFieldSettings MixedLayers = DuplicateLayers;
	MixedLayers.Uplifts = LastSettings.Uplifts;
	TestFalse(TEXT("単層と二層の曖昧な入力を拒否"), SphericalContinentalField::Create(MixedLayers, Error).has_value());
	TestTrue(TEXT("曖昧な隆起指定の拒否理由"), Error == SphericalContinentalFieldError::InvalidSettings);
	// 全ての許可経路を確認後，無効入力とキャンセルを照合
	SphericalContinentalFieldSettings Invalid = LastSettings;
	Invalid.Uplifts.pop_back();
	TestFalse(TEXT("大陸と源の数の不一致を拒否"), SphericalContinentalField::Create(Invalid, Error).has_value());
	TestTrue(TEXT("対応の拒否理由"), Error == SphericalContinentalFieldError::InvalidSettings);
	Invalid = LastSettings;
	Invalid.OceanDepthMeters = Invalid.Domain.Warp.RadiusMeters;
	TestFalse(TEXT("中心を侵す海底を拒否"), SphericalContinentalField::Create(Invalid, Error).has_value());
	TestTrue(TEXT("深度の拒否理由"), Error == SphericalContinentalFieldError::ReliefOutsideSphere);
	Invalid = LastSettings;
	Invalid.PotentialReferenceMeters = std::numeric_limits<double>::denorm_min();
	TestFalse(TEXT("表現出来ない高さ変換を拒否"), SphericalContinentalField::Create(Invalid, Error).has_value());
	Invalid = LastSettings;
	Invalid.PotentialReferenceMeters = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("非有限な変換尺度を拒否"), SphericalContinentalField::Create(Invalid, Error).has_value());
	Invalid = LastSettings;
	Invalid.Domain.Continents[1] = Invalid.Domain.Continents[0];
	TestFalse(TEXT("連結領域の準備拒否を伝播"), SphericalContinentalField::Create(Invalid, Error).has_value());
	TestTrue(TEXT("領域の拒否理由"), Error == SphericalContinentalFieldError::OceanDomainRejected);
	Invalid = LastSettings;
	Invalid.Uplifts[0].Sources[0].Radius = 1;
	TestFalse(TEXT("源の準備拒否を伝播"), SphericalContinentalField::Create(Invalid, Error).has_value());
	TestTrue(TEXT("源の拒否理由"), Error == SphericalContinentalFieldError::UpliftRejected);
	Invalid = LastSettings;
	Invalid.Uplifts[0].BackgroundMeters = std::numeric_limits<double>::denorm_min();
	Invalid.Uplifts[0].Sources.clear();
	TestFalse(TEXT("高さが全域で過小化する構成を拒否"), SphericalContinentalField::Create(Invalid, Error).has_value());
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(TEXT("中断した準備を拒否"), SphericalContinentalField::Create(LastSettings, Error, Stop.get_token()).has_value());
	TestTrue(TEXT("中断の理由を維持"), Error == SphericalContinentalFieldError::Cancelled);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Spherical continental field: planets=12 height=%llu gradient=%llu coast=%llu downhill=%llu ")
			TEXT("gradientError=%.12g radial=%.12g area=%.12g coastH=%.12g coastG=%.12g ")
			TEXT("coastHRatio=%.12g coastGRatio=%.12g seconds=%.9g."),
			static_cast<unsigned long long>(HeightSamples),
			static_cast<unsigned long long>(GradientSamples),
			static_cast<unsigned long long>(CoastSamples),
			static_cast<unsigned long long>(DownhillSamples),
			MaximumGradientError,
			MaximumRadialError,
			MaximumAreaError,
			MaximumCoastHeight,
			MaximumCoastGradient,
			MaximumCoastHeightRatio,
			MaximumCoastGradientRatio,
			Seconds
		)
	);
	// 合成の有限検査であり，実地形の景観・侵食・全河川・期限の完成としない返戻
	return true;
}

#endif
