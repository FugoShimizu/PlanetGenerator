#include "Core/PlanetFlowChannel.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetFlowMorphologyContracts, "PlanetGenerator.FlowChannel.Morphology",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 可変断面の高さ・解析微分と，淵の支持端・不正入力を独立に検査する
 * @param Parameters 未使用の UE 引数
 * @return 全ての検査が成立した場合に真
 */
bool FPlanetFlowMorphologyContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::FlowChannelProfile;
	using PlanetCore::FlowChannelShape;
	using PlanetCore::FlowChannelPool;
	using PlanetCore::TerrainFlowFootpoint;
	using PlanetCore::TerrainFlowSegment;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::array<FlowChannelPool, 2> Pools { FlowChannelPool{ 15, 6, 2, 0.75 }, FlowChannelPool{ 38, 10, 3, 1 } };
	double MaximumBedError = 0, MaximumProfileError = 0;
	std::uint32_t Samples = 0;
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		FlowChannelShape Base;
		Base.RadiusMeters = Radius;
		const std::optional<FlowChannelProfile> Profile = FlowChannelProfile::Create(Base, 60, Pools);
		// 正常な縦断を作れない場合の返戻
		if(!TestTrue(TEXT("有限な二つの淵を準備出来る"), Profile.has_value())) return false;
		TestEqual(TEXT("最大幅は全ての淵を覆う"), Profile->GetMaximumHalfWidthMeters(), 9.0);
		TestEqual(TEXT("淵の中心で幅が広がる"), Profile->Sample(15)->HalfWidthMeters, 8.0);
		TestEqual(TEXT("淵の中心で河床が深くなる"), Profile->Sample(38)->DepthMeters, 2.5);
		for(const double S : { 0.0, 9.0, 21.0, 28.0, 48.0, 60.0 }) {
			const FlowChannelShape Shape = *Profile->Sample(S);
			TestEqual(TEXT("支持端と支持外で基準幅を保持"), Shape.HalfWidthMeters, Base.HalfWidthMeters);
			TestEqual(TEXT("支持端と支持外で基準深さを保持"), Shape.DepthMeters, Base.DepthMeters);
			TestEqual(TEXT("支持端で幅の微分が零"), Shape.HalfWidthDerivativePerMeter, 0.0);
			TestEqual(TEXT("支持端で深さの微分が零"), Shape.DepthDerivativePerMeter, 0.0);
		}
		const UnitDirection First = *UnitDirection::FromVector({ 1, 0, 0 });
		const double Angle = 60 / Radius;
		const UnitDirection Last = *UnitDirection::FromVector({ std::cos(Angle), std::sin(Angle), 0 });
		const std::optional<TerrainFlowSegment> Curve =
		TerrainFlowSegment::Create(First, Last, { 0, 1, 0 }, { -std::sin(Angle), std::cos(Angle), 0 }, Radius);
		// 正常な曲線を作れない場合の返戻
		if(!TestTrue(TEXT("三半径で同じ物理長の河道を準備出来る"), Curve.has_value())) return false;
		const auto Source = [&](const UnitDirection Direction) -> PlanetCore::TerrainSample {
			const Vec3 D = Direction.Vector(), V { 0, -0.02 * Radius, 0 };
			// 断面補正から独立した傾斜場の返戻
			return { 100 + V.Dot(D), V - D * V.Dot(D) };
		};
		const auto Evaluate = [&](const UnitDirection Query, const bool Analytic) -> std::optional<PlanetCore::FlowChannelSection> {
			const std::optional<TerrainFlowFootpoint> Foot = Curve->FindInteriorFootpoint(Query);
			// 有効な直交点が無い場合の返戻
			if(!Foot) return {};
			const double S = Foot->Fraction * Curve->GetParameterLengthMeters();
			std::optional<FlowChannelShape> Shape = Profile->Sample(S);
			// 範囲外の縦断への返戻
			if(!Shape) return {};
			if(Analytic) {
				Shape->HalfWidthMeters = 6 + std::sin(S / 8);
				Shape->DepthMeters = 1.5 + 0.25 * std::cos(S / 5);
				Shape->HalfWidthDerivativePerMeter = std::cos(S / 8) / 8;
				Shape->DepthDerivativePerMeter = -0.05 * std::sin(S / 5);
			}
			// 同じ断面を値と微分の照合に使う返戻
			return PlanetCore::SampleFlowChannel(*Foot, Source(Query), { 99.7 - 0.02 * S, -0.02 }, *Shape);
		};
		for(unsigned i = 1; i < 60; ++i) {
			const double S = static_cast<double>(i);
			const FlowChannelShape Shape = *Profile->Sample(S);
			constexpr double ProfileStep = 1E-4;
			const FlowChannelShape L = *Profile->Sample(S - ProfileStep), H = *Profile->Sample(S + ProfileStep);
			const double WidthError =
			std::abs((H.HalfWidthMeters - L.HalfWidthMeters) / (2 * ProfileStep) - Shape.HalfWidthDerivativePerMeter);
			const double DepthError = std::abs((H.DepthMeters - L.DepthMeters) / (2 * ProfileStep) - Shape.DepthDerivativePerMeter);
			MaximumProfileError = std::max({ MaximumProfileError, WidthError, DepthError });
			TestTrue(TEXT("縦断寸法の微分が独立差分と一致"), WidthError < 1E-7 && DepthError < 1E-7);
			const Vec3 D { std::cos(S / Radius), std::sin(S / Radius), 0 };
			const Vec3 Along { -std::sin(S / Radius), std::cos(S / Radius), 0 };
			for(const double Offset : { -12.0, -7.0, -4.0, 0.0, 4.0, 7.0, 12.0 }) {
				const UnitDirection Query =
				*UnitDirection::FromVector(D * std::cos(Offset / Radius) + Vec3{ 0, 0, 1 } * std::sin(Offset / Radius));
				for(const bool Analytic : { false, true }) {
					const std::optional<PlanetCore::FlowChannelSection> Here = Evaluate(Query, Analytic);
					// 正規の縦断内部を評価出来ない場合の返戻
					if(!TestTrue(TEXT("可変断面の正常な地点評価"), Here.has_value())) return false;
					for(const Vec3 Axis : { Along, Query.Vector().Cross(Along) }) {
						constexpr double Step = 0.002;
						const UnitDirection Low = *UnitDirection::FromVector(Query.Vector() * std::cos(Step / Radius) - Axis * std::sin(Step / Radius));
						const UnitDirection High =
						*UnitDirection::FromVector(Query.Vector() * std::cos(Step / Radius) + Axis * std::sin(Step / Radius));
						const std::optional<PlanetCore::FlowChannelSection> Lower = Evaluate(Low, Analytic), Upper = Evaluate(High, Analytic);
						// 独立差分の両端を評価出来ない場合の返戻
						if(!TestTrue(TEXT("可変断面の差分標本を評価出来る"), Lower && Upper)) return false;
						const double Difference = (Upper->Terrain.ElevationMeters - Lower->Terrain.ElevationMeters) / (2 * Step);
						const double Error = std::abs(Difference - Here->Terrain.TangentialGradientPerRadian.Dot(Axis) / Radius);
						MaximumBedError = std::max(MaximumBedError, Error);
						TestTrue(TEXT("幅と深さが変わる河床の両接線微分が一致"), Error < 0.0005);
						++Samples;
					}
					if(std::abs(Offset) > Profile->GetMaximumHalfWidthMeters()) {
						TestEqual(TEXT("可変断面の支持外で元標高を保持"), Here->Terrain.ElevationMeters, Source(Query).ElevationMeters);
						TestEqual(
							TEXT("可変断面の支持外で元勾配を保持"),
							(Here->Terrain.TangentialGradientPerRadian - Source(Query).TangentialGradientPerRadian).Length(),
							0.0
						);
					}
				}
			}
		}
		TestFalse(TEXT("経路外の寸法照会を拒否"), Profile->Sample(-1).has_value() || Profile->Sample(61).has_value());
		TestFalse(TEXT("非有限な寸法照会を拒否"), Profile->Sample(std::numeric_limits<double>::quiet_NaN()).has_value());
		std::array<FlowChannelPool, 2> Invalid = Pools;
		Invalid[1].CenterDistanceMeters = 20;
		TestFalse(TEXT("重なる淵を拒否"), FlowChannelProfile::Create(Base, 60, Invalid).has_value());
		Invalid = Pools;
		Invalid[0].HalfLengthMeters = 0;
		TestFalse(TEXT("零長の淵を拒否"), FlowChannelProfile::Create(Base, 60, Invalid).has_value());
		Invalid = Pools;
		Invalid[1].AddedDepthMeters = 20;
		TestFalse(TEXT("不正な断面比を拒否"), FlowChannelProfile::Create(Base, 60, Invalid).has_value());
		Invalid = Pools;
		Invalid[1].CenterDistanceMeters = 58;
		TestFalse(TEXT("経路からはみ出す淵を拒否"), FlowChannelProfile::Create(Base, 60, Invalid).has_value());
		TestTrue(TEXT("淵が無い通常の寸法を準備出来る"), FlowChannelProfile::Create(Base, 60, {}).has_value());
	}
	AddInfo(
		FString::Printf(
			TEXT("Flow morphology contracts: samples=%u bedDerivativeError=%.12g profileDerivativeError=%.12g"),
			Samples,
			MaximumBedError,
			MaximumProfileError
		)
	);
	// 独立な縦断・横断・支持端の検査結果の返戻
	return !HasAnyErrors();
}

#endif
