#include "Core/PlanetTerrainContour.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetTerrainContourContracts, "PlanetGenerator.TerrainContour.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 既知の円・楕円の岸を独立な面積と一周角で比較し，不完全な追跡を閉水面にしない事を検査する
 * @param Parameters 未使用の UE 引数
 * @return 有界な等高線追跡の検査結果，連続場の全等高線を列挙する保証ではない
 */
bool FPlanetTerrainContourContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::TerrainContour;
	using PlanetCore::TerrainContourEnd;
	using PlanetCore::TerrainContourSettings;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	// 描画前に検証する平面輪郭の正規経路と不正経路
	const std::array<std::array<double, 2>, 5> Concave { { { 0, 0 }, { 4, 0 }, { 4, 4 }, { 2, 1 }, { 0, 4 } } };
	const std::array<std::array<double, 2>, 5> Collinear { { { 0, 0 }, { 2, 0 }, { 4, 0 }, { 4, 4 }, { 0, 4 } } };
	const std::array<std::array<double, 2>, 4> Crossing { { { 0, 0 }, { 4, 4 }, { 0, 4 }, { 4, 0 } } };
	const std::array<std::array<double, 2>, 5> Backtracking { { { 0, 0 }, { 4, 0 }, { 2, 0 }, { 4, 4 }, { 0, 4 } } };
	const std::array<std::array<double, 2>, 5> Touching { { { 0, 0 }, { 4, 0 }, { 4, 4 }, { 2, 0 }, { 0, 4 } } };
	TestTrue(TEXT("凹形の単純な岸を受理"), PlanetCore::IsSimplePlanarContour(Concave));
	TestTrue(TEXT("真直ぐな途中の頂点を受理"), PlanetCore::IsSimplePlanarContour(Collinear));
	TestFalse(TEXT("対辺が交差する岸を拒否"), PlanetCore::IsSimplePlanarContour(Crossing));
	TestFalse(TEXT("隣接辺の折返を拒否"), PlanetCore::IsSimplePlanarContour(Backtracking));
	TestFalse(TEXT("非隣接辺の接触を拒否"), PlanetCore::IsSimplePlanarContour(Touching));
	TestFalse(TEXT("空の岸を拒否"), PlanetCore::IsSimplePlanarContour({}));
	std::array<std::array<double, 2>, 5> InvalidPolygon = Concave;
	InvalidPolygon[1] = InvalidPolygon[0];
	TestFalse(TEXT("長さ零の辺を拒否"), PlanetCore::IsSimplePlanarContour(InvalidPolygon));
	InvalidPolygon = Concave;
	InvalidPolygon[1][0] = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("非有限な岸の座標を拒否"), PlanetCore::IsSimplePlanarContour(InvalidPolygon));
	// 小さな固定上限で拒否していた大きな正規輪郭と，同じ頂点列の交差
	for(const std::uint32_t Count : { 4096U, 32768U, 65536U, 65537U }) {
		std::vector<std::array<double, 2>> Circle;
		Circle.reserve(Count);
		for(std::uint32_t I = 0; I < Count; ++I) {
			const double Angle = 2 * std::numbers::pi * I / Count;
			Circle.push_back({ 1000 * std::cos(Angle), 1000 * std::sin(Angle) });
		}
		if(Count > 65536) {
			TestFalse(TEXT("有効形状でも保持予算を超える頂点数は拒否"), PlanetCore::IsSimplePlanarContour(Circle));
			continue;
		}
		TestTrue(TEXT("大きな単純円形の岸を受理"), PlanetCore::IsSimplePlanarContour(Circle));
		std::reverse(Circle.begin(), Circle.end());
		TestTrue(TEXT("逆向きの大きな岸も受理"), PlanetCore::IsSimplePlanarContour(Circle));
		std::swap(Circle[Count / 4], Circle[Count / 2]);
		TestFalse(TEXT("大きな輪郭の非局所的交差を拒否"), PlanetCore::IsSimplePlanarContour(Circle));
	}
	// 全球座標の軸と一致しない既知の射影基底
	const Vec3 Center = UnitDirection::FromVector({ -0.2, 0.3, 0.8 })->Vector();
	const Vec3 East = UnitDirection::FromVector(Vec3{ 0, 0, 1 }.Cross(Center))->Vector(), North = Center.Cross(East);
	std::uint32_t ClosedCases = 0, CheckedVertices = 0;
	double MaximumAreaRelativeError = 0, MaximumCircleLengthRelativeError = 0, MaximumHeightResidual = 0;
	double MaximumSphericalAreaRelativeError = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) for(const double MinorAxis : { 100.0, 60.0 }) {
		// 射影上の閉形式の湖岸と，其の解析勾配
		constexpr double MajorAxis = 100;
		const auto Source = [&](const UnitDirection Direction) -> TerrainSample {
			// 解析面を定義する射影座標と微分
			const Vec3 D = Direction.Vector();
			const double Denominator = D.Dot(Center), U = D.Dot(East), V = D.Dot(North), X = Radius * U / Denominator;
			const double Y = Radius * V / Denominator;
			const Vec3 DX = (East * Denominator - Center * U) * (Radius / (Denominator * Denominator));
			const Vec3 DY = (North * Denominator - Center * V) * (Radius / (Denominator * Denominator));
			// 既知の楕円を持つ不変な元場の返戻
			return {
				20 + X * X / (MajorAxis * MajorAxis) + Y * Y / (MinorAxis * MinorAxis),
				DX * (2 * X / (MajorAxis * MajorAxis)) + DY * (2 * Y / (MinorAxis * MinorAxis))
			};
		};
		// 同じ物理精度を用いる３半径の追跡設定
		TerrainContourSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.LevelMeters = 21;
		Settings.MaximumLengthMeters = 2000;
		Settings.ChordToleranceMeters = 0.02;
		// 解析的に指定した岸の始点
		const UnitDirection Initial = *UnitDirection::FromVector(Center + East * (MajorAxis / Radius));
		const std::optional<TerrainContour> Curve = PlanetCore::TraceTerrainContour(Initial, Source, Settings);
		// 正規の閉曲線を準備出来ない場合の返戻
		if(!TestTrue(TEXT("解析的な円又は楕円の岸を受理"), Curve.has_value())) return false;
		TestEqual(TEXT("正則な岸は一周して閉鎖"), Curve->End, TerrainContourEnd::Closed);
		AddInfo(
			FString::Printf(
				TEXT("Contour analytic: radius=%.0f minor=%.0f end=%u vertices=%llu evaluations=%u length=%.12g"),
				Radius,
				MinorAxis,
				static_cast<unsigned>(Curve->End),
				static_cast<unsigned long long>(Curve->Samples.size()),
				Curve->Evaluations,
				Curve->Samples.back().DistanceMeters
			)
		);
		// 位相が失敗した場合も計測値を保持する返戻
		if(Curve->End != TerrainContourEnd::Closed) return false;
		TestTrue(
			TEXT("閉鎖点は始点と同一"),
			(Curve->Samples.front().Direction.Vector() - Curve->Samples.back().Direction.Vector()).Length() == 0
		);
		// 実装の誤差推定を用いない面積と回転角の積算
		double TwiceArea = 0, TotalAngle = 0, PreviousX = 0, PreviousY = 0, PreviousAngle = 0;
		for(std::uint32_t i = 0; i < Curve->Samples.size(); ++i) {
			const Vec3 D = Curve->Samples[i].Direction.Vector();
			const double X = Radius * D.Dot(East) / D.Dot(Center), Y = Radius * D.Dot(North) / D.Dot(Center);
			const double Angle = std::atan2(Y / MinorAxis, X / MajorAxis);
			MaximumHeightResidual = std::max(MaximumHeightResidual, std::abs(Source(Curve->Samples[i].Direction).ElevationMeters - 21));
			if(i) {
				TwiceArea += PreviousX * Y - X * PreviousY;
				double Turn = Angle - PreviousAngle;
				if(Turn < 0) Turn += 2 * std::numbers::pi;
				TestTrue(TEXT("岸を同じ向きで細かく辿る"), Turn > 0 && Turn < 0.3);
				TotalAngle += Turn;
			}
			PreviousX = X;
			PreviousY = Y;
			PreviousAngle = Angle;
			++CheckedVertices;
		}
		// 閉形式の楕円面積
		const double ExpectedArea = std::numbers::pi * MajorAxis * MinorAxis;
		MaximumAreaRelativeError = std::max(MaximumAreaRelativeError, std::abs(TwiceArea * 0.5 / ExpectedArea - 1));
		TestTrue(TEXT("低い側を左に保ち，過不足無く一周"), std::abs(TotalAngle - 2 * std::numbers::pi) < 1E-8);
		if(MinorAxis == MajorAxis) {
			// 球面上の小円の厳密な周長
			const double ExpectedLength = 2 * std::numbers::pi * Radius * MajorAxis / std::hypot(Radius, MajorAxis);
			MaximumCircleLengthRelativeError =
			std::max(MaximumCircleLengthRelativeError, std::abs(Curve->Samples.back().DistanceMeters / ExpectedLength - 1));
		}
		// 立体角による面積の向きと，閉形式の球面小円との比較
		const std::optional<double> SphericalArea = PlanetCore::MeasureTerrainContourArea(*Curve, Radius);
		// 半球内の閉曲線を計測出来ない場合の返戻
		if(!TestTrue(TEXT("球面上の閉じた岸の面積を取得"), SphericalArea.has_value())) return false;
		TestTrue(TEXT("低地を囲む向きは正"), *SphericalArea > 0);
		TestFalse(TEXT("範囲外の半径を拒否"), PlanetCore::MeasureTerrainContourArea(*Curve, 999).has_value());
		TerrainContour Reversed = *Curve;
		std::reverse(Reversed.Samples.begin(), Reversed.Samples.end());
		const std::optional<double> ReversedArea = PlanetCore::MeasureTerrainContourArea(Reversed, Radius);
		// 正規の逆向き曲線を計測出来ない場合の返戻
		if(!TestTrue(TEXT("逆向きの閉曲線も計測"), ReversedArea.has_value())) return false;
		TestTrue(TEXT("逆向きは同じ面積で符号だけが反転"), std::abs(*SphericalArea + *ReversedArea) <= *SphericalArea * 1E-10);
		if(MinorAxis == MajorAxis) {
			// １と余弦の相殺を避けた厳密な球冠面積
			const double Hypotenuse = std::hypot(Radius, MajorAxis);
			const double CapArea = 2 * std::numbers::pi * Radius * Radius * MajorAxis * MajorAxis / (Hypotenuse * (Hypotenuse + Radius));
			MaximumSphericalAreaRelativeError = std::max(MaximumSphericalAreaRelativeError, std::abs(*SphericalArea / CapArea - 1));
		}
		++ClosedCases;
	}
	TestTrue(TEXT("独立な射影面積と一致"), MaximumAreaRelativeError < 0.001);
	TestTrue(TEXT("独立な球冠面積と一致"), MaximumSphericalAreaRelativeError < 0.001);
	TestTrue(TEXT("独立な球面の円周長と一致"), MaximumCircleLengthRelativeError < 0.001);
	TestTrue(TEXT("岸の元標高は水位に一致"), MaximumHeightResidual <= 1E-6);
	// 両端と中点が一直線でも，四分点で湾曲する岸の回帰
	const auto Ripple = [](const UnitDirection Direction) -> TerrainSample {
		// 心射図法上の波状の等高線と解析勾配
		constexpr double Radius = 1000000, Amplitude = 0.2, WaveNumber = 2 * std::numbers::pi / 50;
		const Vec3 D = Direction.Vector();
		const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, Phase = WaveNumber * X;
		const Vec3 DX = Vec3{ D.Z, 0, -D.X } * (Radius / (D.Z * D.Z));
		const Vec3 DY = Vec3{ 0, D.Z, -D.Y } * (Radius / (D.Z * D.Z));
		// 標高が零の波状曲線を持つ元場の返戻
		return { Y - Amplitude * std::sin(Phase) * std::sin(Phase), DY - DX * (Amplitude * WaveNumber * std::sin(2 * Phase)) };
	};
	TerrainContourSettings RippleSettings;
	RippleSettings.MaximumLengthMeters = 100;
	const std::optional<TerrainContour> RippleCurve =
	PlanetCore::TraceTerrainContour(*UnitDirection::FromVector({ 0, 0, 1 }), Ripple, RippleSettings);
	// 正規の波状曲線を追跡出来ない場合の返戻
	if(!TestTrue(TEXT("中点に現れない湾曲を持つ岸を受理"), RippleCurve.has_value())) return false;
	TestTrue(TEXT("波状の開曲線を閉じない"), RippleCurve->End != TerrainContourEnd::Closed);
	double RippleIndependentOffset = 0;
	for(std::uint32_t i = 1; i < RippleCurve->Samples.size(); ++i) {
		const Vec3 A = RippleCurve->Samples[i - 1].Direction.Vector(), B = RippleCurve->Samples[i].Direction.Vector();
		for(unsigned j = 1; j < 16; ++j) {
			const double Fraction = j / 16.0;
			const UnitDirection Probe = *UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
			const TerrainSample Value = Ripple(Probe);
			RippleIndependentOffset =
			std::max(RippleIndependentOffset, std::abs(Value.ElevationMeters) * 1000000 / Value.TangentialGradientPerRadian.Length());
		}
	}
	TestTrue(TEXT("中点以外の独立な標本でも岸の 5 cm 精度を維持"), RippleIndependentOffset <= RippleSettings.ChordToleranceMeters);
	AddInfo(
		FString::Printf(
			TEXT("Contour ripple: vertices=%llu rejected=%u independentOffset=%.12g"),
			static_cast<unsigned long long>(RippleCurve->Samples.size()),
			RippleCurve->RejectedSteps,
			RippleIndependentOffset
		)
	);
	// 鞍点を跨ぐ等高線を通常の閉曲線へ結ばない検査
	TerrainContourSettings Settings;
	Settings.MaximumLengthMeters = 2000;
	const auto Saddle = [](const UnitDirection Direction) -> TerrainSample {
		const Vec3 D = Direction.Vector();
		const double X = 1000000 * D.X, Y = 1000000 * D.Y;
		// 鞍点を持つ元場の返戻
		return { X * Y * 0.001, { Y * 1000, X * 1000, 0 } };
	};
	const UnitDirection Initial = *UnitDirection::FromVector({ 0.0001, 0, 1 });
	const std::optional<TerrainContour> Open = PlanetCore::TraceTerrainContour(Initial, Saddle, Settings);
	// 有界な部分経路を返せない場合の返戻
	if(!TestTrue(TEXT("鞍点へ向かう部分経路を取得"), Open.has_value())) return false;
	TestTrue(TEXT("鞍点の分岐を閉じた岸と偽らない"), Open->End != TerrainContourEnd::Closed);
	TestFalse(TEXT("開曲線へ閉領域の面積を付けない"), PlanetCore::MeasureTerrainContourArea(*Open, 1000000).has_value());
	TestTrue(TEXT("停留点の向こう側へ飛び移らない"), Open->Samples.back().Direction.Vector().X >= 0);
	// 平坦面は正則な湖岸として受理しない
	const auto Flat = [](const UnitDirection) -> TerrainSample {
		return { 0, {} };
	};
	const std::optional<TerrainContour> FlatCurve = PlanetCore::TraceTerrainContour(Initial, Flat, Settings);
	TestTrue(TEXT("平坦面の停留を区別"), FlatCurve.has_value() && FlatCurve->End == TerrainContourEnd::CriticalPoint);
	// 制限による部分経路と，場の不正値・中断
	Settings.MaximumVertices = 4;
	const std::optional<TerrainContour> Budget = PlanetCore::TraceTerrainContour(Initial, Saddle, Settings);
	TestTrue(
		TEXT("頂点予算の停止を区別"),
		Budget.has_value() && Budget->End == TerrainContourEnd::VertexLimit && Budget->Samples.size() == 4
	);
	Settings.MaximumVertices = 32768;
	Settings.MaximumLengthMeters = Settings.MaximumStepMeters;
	const std::optional<TerrainContour> Length = PlanetCore::TraceTerrainContour(Initial, Saddle, Settings);
	TestTrue(TEXT("距離予算で閉鎖を捏造しない"), Length.has_value() && Length->End == TerrainContourEnd::LengthLimit);
	Settings.MinimumStepMeters = 0;
	TestFalse(TEXT("零の最小刻みを拒否"), PlanetCore::TraceTerrainContour(Initial, Saddle, Settings).has_value());
	Settings.MinimumStepMeters = 0.001;
	TestFalse(TEXT("未指定の場を拒否"), PlanetCore::TraceTerrainContour(Initial, {}, Settings).has_value());
	const auto Invalid = [](const UnitDirection) -> TerrainSample {
		return { std::numeric_limits<double>::quiet_NaN(), {} };
	};
	TestFalse(TEXT("非有限な場を拒否"), PlanetCore::TraceTerrainContour(Initial, Invalid, Settings).has_value());
	std::stop_source Cancelled;
	const auto Interrupting = [&](const UnitDirection) -> TerrainSample {
		Cancelled.request_stop();
		// 中断と同時に返る有限値の返戻
		return { 0, { 1, 0, 0 } };
	};
	TestFalse(
		TEXT("場の照会中に届いた中断を守る"),
		PlanetCore::TraceTerrainContour(Initial, Interrupting, Settings, Cancelled.get_token()).has_value()
	);
	AddInfo(
		FString::Printf(
			TEXT(
				"Contour contracts: closed=%u vertices=%u maxAreaRelativeError=%.12g maxCircleLengthRelativeError=%.12g maxHeightResidual=%.12g"
			),
			ClosedCases,
			CheckedVertices,
			MaximumAreaRelativeError,
			MaximumCircleLengthRelativeError,
			MaximumHeightResidual
		)
	);
	AddInfo(FString::Printf(TEXT("Contour spherical area: maximumRelativeError=%.12g"), MaximumSphericalAreaRelativeError));
	// 等高線追跡の検査結果の返戻
	return !HasAnyErrors();
}

#endif
