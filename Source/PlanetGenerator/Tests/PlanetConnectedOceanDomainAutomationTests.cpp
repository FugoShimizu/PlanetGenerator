#include "Core/PlanetConnectedOceanDomain.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetConnectedOceanDomainContracts,
	"PlanetGenerator.Ocean.ConstructiveDomain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 領域の構成条件，面積，独立な所属判定，海岸の解析勾配を照合する
 * @param Parameters 未使用の UE 引数
 * @return 構成と入力拒否の検査が成立した場合に真，景観の完成とは別
 */
bool FPlanetConnectedOceanDomainContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::ConnectedOceanDomain;
	using PlanetCore::ConnectedOceanDomainError;
	using PlanetCore::ConnectedOceanDomainSample;
	using PlanetCore::ConnectedOceanDomainSettings;
	using PlanetCore::ContinentalDomainCap;
	using PlanetCore::PlanetSphericalWarp;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const double Radius = std::acos(1 - 2 * 0.35 / 6);
	const std::array<Vec3, 6> Centers {
		Vec3{ 1, 0, 0 },
		Vec3{ -1, 0, 0 },
		Vec3{ 0, 1, 0 },
		Vec3{ 0, -1, 0 },
		Vec3{ 0, 0, 1 },
		Vec3{ 0, 0, -1 }
	};
	ConnectedOceanDomainSettings Settings;
	for(const Vec3 Center : Centers) Settings.Continents.push_back({ *UnitDirection::FromVector(Center), Radius });
	ConnectedOceanDomainError Error = ConnectedOceanDomainError::None;
	std::uint64_t SignSamples = 0, CoastSamples = 0;
	double MaximumAreaError = 0, MaximumGradientError = 0, MaximumRadial = 0, MaximumSignDifference = 0;
	for(const double PlanetRadius : { 1000.0, 1000000.0, 1000000000.0 }) {
		Settings.Warp.RadiusMeters = PlanetRadius;
		Settings.Warp.LargestWavelengthMeters = 0.5 * PlanetRadius;
		const std::optional<ConnectedOceanDomain> Domain = ConnectedOceanDomain::Create(Settings, Error);
		const std::optional<PlanetSphericalWarp> Warp = PlanetSphericalWarp::Create(Settings.Warp);
		// 正規入力での領域又は既存写像の準備に失敗した場合の返戻
		if(!TestTrue(TEXT("正規入力を受理"), Domain.has_value() && Warp.has_value())) return false;
		TestTrue(TEXT("成功理由を明示"), Error == ConnectedOceanDomainError::None);
		TestTrue(TEXT("解析的な海域比が６５％"), std::abs(Domain->Statistics().OceanFraction - 0.65) < 1E-14);
		TestTrue(TEXT("全円板が厳密に離れる"), Domain->Statistics().MinimumSeparationRadians > 0);
		TestTrue(TEXT("大洋基準点が同じ海域に属す"), Domain->ContainsOcean(Domain->OceanAnchor()));
		TestTrue(TEXT("大洋基準点が負の符号を持つ"), Domain->Evaluate(Domain->OceanAnchor()).CoastSign < 0);
		const Vec3 Root = Warp->ToSource(Domain->OceanAnchor()).Vector();
		for(const ContinentalDomainCap &Cap : Settings.Continents) {
			TestTrue(TEXT("大洋基準点が全閉円板の外"), Root.Dot(Cap.Center.Vector()) < std::cos(Cap.RadiusRadians));
		}
		std::uint32_t Wet = 0;
		for(unsigned I = 0; I < 8192; ++I) {
			const double Z = 1 - 2 * (I + 0.5) / 8192, Angle = I * 2.399963229728653, Ring = std::sqrt(1 - Z * Z);
			const UnitDirection Direction = *UnitDirection::FromVector({ Ring * std::cos(Angle), Ring * std::sin(Angle), Z });
			const Vec3 Source = Warp->ToSource(Direction).Vector();
			bool IndependentWet = true;
			for(const ContinentalDomainCap &Cap : Settings.Continents) {
				if(std::atan2(Source.Cross(Cap.Center.Vector()).Length(), Source.Dot(Cap.Center.Vector())) <= Cap.RadiusRadians) {
					IndependentWet = false;
				}
			}
			const ConnectedOceanDomainSample Value = Domain->Evaluate(Direction);
			TestTrue(TEXT("独立な角距離の海域所属と符号が一致"), Value.CoastSign < 0 == IndependentWet);
			TestTrue(TEXT("直接の所属判定と符号が一致"), Domain->ContainsOcean(Direction) == Value.CoastSign < 0);
			TestTrue(TEXT("符号と勾配が有限"), std::isfinite(Value.CoastSign) && std::isfinite(Value.TangentialGradientPerRadian.Length()));
			TestTrue(TEXT("固定域へ接続した符号の値域"), std::abs(Value.CoastSign) < 1 || std::abs(Value.CoastSign) == 1);
			Wet += IndependentWet ? 1 : 0;
			++SignSamples;
		}
		MaximumAreaError = std::max(MaximumAreaError, std::abs(static_cast<double>(Wet) / 8192 - 0.65));
		for(const ContinentalDomainCap &Cap : Settings.Continents) {
			const Vec3 Center = Cap.Center.Vector();
			const Vec3 Along =
			UnitDirection::FromVector((std::abs(Center.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 }).Cross(Center))->Vector();
			const Vec3 Across = Center.Cross(Along);
			const double Width = std::min(
				{
					Settings.CoastTransitionMeters / PlanetRadius,
					0.25 * Cap.RadiusRadians,
					0.25 * Domain->Statistics().MinimumSeparationRadians
				}
			);
			const double Step = std::min(1E-5, 0.001 * Width);
			for(unsigned I = 0; I < 8; ++I) {
				const double Angle = I * std::numbers::pi_v<double> * 0.25;
				const Vec3 Tangent = Along * std::cos(Angle) + Across * std::sin(Angle);
				for(const double Fraction : { -1.2, -0.75, -0.25, 0.0, 0.25, 0.75, 1.2 }) {
					const double Arc = Cap.RadiusRadians + Fraction * Width;
					const UnitDirection Original = *UnitDirection::FromVector(Center * std::cos(Arc) + Tangent * std::sin(Arc));
					const UnitDirection Direction = Warp->ToWorld(Original);
					const Vec3 Point = Direction.Vector();
					const ConnectedOceanDomainSample Value = Domain->Evaluate(Direction);
					const Vec3 First =
					UnitDirection::FromVector((std::abs(Point.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 }).Cross(Point))->Vector();
					const std::array<Vec3, 2> Axes { First, Point.Cross(First) };
					for(const Vec3 Axis : Axes) {
						// 解析勾配を使わず，大円上の値だけから二段階の差分を得る
						const auto Difference = [&](const double Offset) {
							const UnitDirection Before = *UnitDirection::FromVector(Point * std::cos(Offset) - Axis * std::sin(Offset));
							const UnitDirection After = *UnitDirection::FromVector(Point * std::cos(Offset) + Axis * std::sin(Offset));
							// 海陸の値だけで得た独立な接線差分の返戻
							return (Domain->Evaluate(After).CoastSign - Domain->Evaluate(Before).CoastSign) / (2 * Offset);
						};
						const double Numeric = (4 * Difference(Step * 0.5) - Difference(Step)) / 3;
						const double Actual = Value.TangentialGradientPerRadian.Dot(Axis);
						MaximumGradientError =
						std::max(MaximumGradientError, std::abs(Numeric - Actual) / std::max(1.0, Value.TangentialGradientPerRadian.Length()));
					}
					MaximumRadial = std::max(
						MaximumRadial,
						std::abs(Value.TangentialGradientPerRadian.Dot(Point)) / std::max(1.0, Value.TangentialGradientPerRadian.Length())
					);
					const double OriginalArc = std::atan2(Original.Vector().Cross(Center).Length(), Original.Vector().Dot(Center));
					if(std::abs(OriginalArc - Cap.RadiusRadians) > 1E-12) {
						TestTrue(TEXT("海岸の両側で符号を保存"), Value.CoastSign < 0 == OriginalArc > Cap.RadiusRadians);
					}
					++CoastSamples;
				}
			}
		}
		const std::optional<ConnectedOceanDomain> Repeated = ConnectedOceanDomain::Create(Settings, Error);
		// 同じ準備入力を受理出来ない場合の返戻
		if(!TestTrue(TEXT("同じ入力を再準備"), Repeated.has_value())) return false;
		TestTrue(TEXT("定義識別値が再現"), Repeated->DefinitionHash() == Domain->DefinitionHash());
		MaximumSignDifference = std::max(
			MaximumSignDifference,
			std::abs(Repeated->Evaluate(Domain->OceanAnchor()).CoastSign - Domain->Evaluate(Domain->OceanAnchor()).CoastSign)
		);
	}
	TestTrue(TEXT("独立な面積標本と構成式が整合"), MaximumAreaError < 0.01);
	TestTrue(TEXT("海岸の解析接線勾配が独立差分と一致"), MaximumGradientError < 1E-5);
	TestTrue(TEXT("海岸勾配は球面接線"), MaximumRadial < 1E-12);
	TestTrue(TEXT("独立再準備の値が一致"), !MaximumSignDifference);
	ConnectedOceanDomainSettings Invalid = Settings;
	Invalid.Continents.clear();
	TestFalse(TEXT("空の陸域を拒否"), ConnectedOceanDomain::Create(Invalid, Error).has_value());
	TestTrue(TEXT("空の理由を識別"), Error == ConnectedOceanDomainError::InvalidSettings);
	Invalid = Settings;
	Invalid.Continents[1] = Invalid.Continents[0];
	TestFalse(TEXT("交差した円板を拒否"), ConnectedOceanDomain::Create(Invalid, Error).has_value());
	TestTrue(TEXT("交差の理由を識別"), Error == ConnectedOceanDomainError::TouchingContinents);
	Invalid = Settings;
	Invalid.Continents[0].RadiusRadians = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("非有限の帽を拒否"), ConnectedOceanDomain::Create(Invalid, Error).has_value());
	TestTrue(TEXT("帽の理由を識別"), Error == ConnectedOceanDomainError::InvalidCap);
	Invalid = Settings;
	Invalid.Continents.erase(Invalid.Continents.begin() + 1, Invalid.Continents.end());
	TestFalse(TEXT("海陸比の範囲外を拒否"), ConnectedOceanDomain::Create(Invalid, Error).has_value());
	TestTrue(TEXT("海陸比の理由を識別"), Error == ConnectedOceanDomainError::OceanAreaOutsideRange);
	Invalid = Settings;
	Invalid.CoastTransitionMeters = std::numeric_limits<double>::min();
	TestFalse(TEXT("解像出来ない海岸幅を拒否"), ConnectedOceanDomain::Create(Invalid, Error).has_value());
	TestTrue(TEXT("海岸幅の理由を識別"), Error == ConnectedOceanDomainError::UnresolvedCoastWidth);
	const double PairRadius = std::acos(0.65);
	ConnectedOceanDomainSettings Pair = Settings;
	Pair.Continents.clear();
	Pair.Continents.push_back({ *UnitDirection::FromVector({ 1, 0, 0 }), PairRadius });
	Pair.Continents.push_back(
		{ *UnitDirection::FromVector({ std::cos(2 * PairRadius), std::sin(2 * PairRadius), 0 }), PairRadius }
	);
	TestFalse(TEXT("一点で触れる二つの円板を拒否"), ConnectedOceanDomain::Create(Pair, Error).has_value());
	TestTrue(TEXT("接触は連結した通水の証拠にしない"), Error == ConnectedOceanDomainError::TouchingContinents);
	Pair.Continents[1].Center = *UnitDirection::FromVector({ std::cos(2 * PairRadius + 1E-6), std::sin(2 * PairRadius + 1E-6), 0 });
	const std::optional<ConnectedOceanDomain> SeparatedPair = ConnectedOceanDomain::Create(Pair, Error);
	TestTrue(TEXT("正の離隔を持つ正常経路は受理"), SeparatedPair.has_value());
	if(SeparatedPair) TestTrue(TEXT("狭い正規通路の基準点が大洋に属す"), SeparatedPair->ContainsOcean(SeparatedPair->OceanAnchor()));
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(TEXT("中断済みの入力を拒否"), ConnectedOceanDomain::Create(Settings, Error, Stop.get_token()).has_value());
	TestTrue(TEXT("中断の理由を識別"), Error == ConnectedOceanDomainError::Cancelled);
	AddInfo(
		FString::Printf(
			TEXT("Constructive ocean domain: signs=%llu coasts=%llu areaError=%.12g ")
			TEXT("gradientError=%.12g radial=%.12g repeat=%.12g ms=%.6f. ")
			TEXT("The disjoint disk complement is connected by construction; ")
			TEXT("final geology, river integration and appearance are not accepted."),
			static_cast<unsigned long long>(SignSamples),
			static_cast<unsigned long long>(CoastSamples),
			MaximumAreaError,
			MaximumGradientError,
			MaximumRadial,
			MaximumSignDifference,
			std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count()
		)
	);
	// 領域の構成・独立な符号と勾配・入力拒否の検査結果の返戻
	return !HasAnyErrors();
}

#endif
