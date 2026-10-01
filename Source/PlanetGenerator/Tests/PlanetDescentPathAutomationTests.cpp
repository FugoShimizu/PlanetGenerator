#include "Core/PlanetDescentPath.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetDescentPathContracts, "PlanetGenerator.DescentPath.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 解析的な球面の鉢で，経路，標高，停止理由，有限予算と中断を検査する
 * @param Parameters 未使用の UE 引数
 * @return 独立な解析経路と停止契約が成立する場合に真
 */
bool FPlanetDescentPathContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::TerrainDescentEnd;
	using PlanetCore::TerrainDescentPath;
	using PlanetCore::TerrainDescentSettings;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	// 座標軸から回転した既知の極小と独立な経路面
	const UnitDirection Pole = *UnitDirection::FromVector({ 0.37, -0.21, 0.903 });
	const Vec3 Up = Pole.Vector(), East = UnitDirection::FromVector(Vec3{ 0, 1, 0 }.Cross(Up))->Vector(), North = Up.Cross(East);
	double MaximumPlaneError = 0, MaximumEndpointError = 0, MaximumHeightError = 0;
	std::uint32_t Samples = 0, Evaluations = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		// 等方的な二次式は同じ大円上を極へ下降
		const auto Field = [&](const UnitDirection Direction) -> TerrainSample {
			// 球面方向の接線成分から相殺を避けて評価する式
			const Vec3 Point = Direction.Vector();
			const double X = Radius * Point.Dot(East), Y = Radius * Point.Dot(North);
			const Vec3 Ambient = (East * X + North * Y) * (0.002 * Radius);
			// 解析標高と其の接線勾配の返戻
			return { 1000 + 0.001 * (X * X + Y * Y), Ambient - Point * Ambient.Dot(Point) };
		};
		// 物理寸法を揃えた球面上の出発点
		TerrainDescentSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.StopElevationMeters = 999;
		Settings.MaximumLengthMeters = 300;
		Settings.MinimumStepMeters = 1E-5;
		const UnitDirection Initial = *UnitDirection::FromVector(Up + (East * 120 + North * 90) * (1 / Radius));
		const Vec3 PlaneNormal = North * 0.8 - East * 0.6;
		const std::optional<TerrainDescentPath> Path = PlanetCore::TraceTerrainDescent(Initial, Field, Settings);
		// 正規な場の追跡を開始出来ない場合の返戻
		if(!TestTrue(TEXT("解析球面を追跡出来る"), Path.has_value())) return false;
		TestTrue(TEXT("極小付近の勾配停止を区別"), Path->End == TerrainDescentEnd::GradientLimit);
		TestTrue(TEXT("経路は有界な頂点数"), Path->Samples.size() <= Settings.MaximumVertices);
		TestTrue(TEXT("刻み誤差推定は設定内"), Path->MaximumEstimatedErrorMeters <= Settings.PositionToleranceMeters);
		for(std::size_t i = 0; i < Path->Samples.size(); ++i) {
			// 実装とは独立な大円面，元式と距離
			const PlanetCore::TerrainDescentSample &Point = Path->Samples[i];
			const double PlaneError = std::abs(Point.Direction.Vector().Dot(PlaneNormal)) * Radius;
			const double HeightError = std::abs(Field(Point.Direction).ElevationMeters - Point.ElevationMeters);
			TestTrue(TEXT("既知の大円上を下降"), PlaneError < 1E-3);
			TestEqual(TEXT("元の標高を保存"), HeightError, 0.0);
			if(i) {
				TestTrue(TEXT("経路の標高は非増加"), Point.ElevationMeters <= Path->Samples[i - 1].ElevationMeters);
				TestTrue(TEXT("採用した経路の距離は厳密に増加"), Point.DistanceMeters > Path->Samples[i - 1].DistanceMeters);
			}
			MaximumPlaneError = std::max(MaximumPlaneError, PlaneError);
			MaximumHeightError = std::max(MaximumHeightError, HeightError);
			++Samples;
		}
		// 独立な解析終点との球面距離
		const Vec3 Last = Path->Samples.back().Direction.Vector();
		const double EndpointError = Radius * std::atan2(Up.Cross(Last).Length(), Up.Dot(Last));
		TestTrue(TEXT("解析極小の１ mm 内へ到達"), EndpointError < 0.001);
		MaximumEndpointError = std::max(MaximumEndpointError, EndpointError);
		Evaluations += Path->Evaluations;
		AddInfo(
			FString::Printf(
				TEXT("Descent analytic case: radius=%.12g points=%llu reason=%u endpointError=%.12g length=%.12g"),
				Radius,
				static_cast<unsigned long long>(Path->Samples.size()),
				static_cast<unsigned>(Path->End),
				EndpointError,
				Path->Samples.back().DistanceMeters
			)
		);
		// 指定水位，距離又は頂点予算で止まった経路を極小到達と混同しない
		Settings.StopElevationMeters = 1010;
		const std::optional<TerrainDescentPath> Water = PlanetCore::TraceTerrainDescent(Initial, Field, Settings);
		TestTrue(TEXT("指定標高以下への到達を区別"), Water && Water->End == TerrainDescentEnd::HeightLimit);
		Settings.StopElevationMeters = 999;
		Settings.MaximumLengthMeters = 60;
		const std::optional<TerrainDescentPath> Limited = PlanetCore::TraceTerrainDescent(Initial, Field, Settings);
		TestTrue(TEXT("距離予算による停止を区別"), Limited && Limited->End == TerrainDescentEnd::LengthLimit);
		if(Limited) TestTrue(TEXT("許可した距離を越えて追跡しない"), Limited->Samples.back().DistanceMeters <= 60 + 1E-6);
		Settings.MaximumLengthMeters = 300;
		Settings.MaximumVertices = 2;
		const std::optional<TerrainDescentPath> Budget = PlanetCore::TraceTerrainDescent(Initial, Field, Settings);
		TestTrue(TEXT("頂点予算による停止を区別"), Budget && Budget->End == TerrainDescentEnd::BudgetLimit && Budget->Samples.size() == 2);
		Settings.MaximumVertices = 4096;
		Settings.MaximumStepMeters = 25;
		const std::optional<TerrainDescentPath> Fine = PlanetCore::TraceTerrainDescent(Initial, Field, Settings);
		TestTrue(TEXT("同じ場を異なる最大刻みでも追跡"), Fine && Fine->End == TerrainDescentEnd::GradientLimit);
		if(Fine) {
			const Vec3 Refined = Fine->Samples.back().Direction.Vector();
			TestTrue(TEXT("刻み変更後も同じ解析極小へ到達"), Radius * std::atan2(Up.Cross(Refined).Length(), Up.Dot(Refined)) < 0.001);
		}
	}
	// 立体射影座標の異方的な二次式では，等角性により下降曲線は V = C U^2
	double MaximumCurveResidual = 0;
	std::uint32_t CurvedSamples = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		// 解析的な曲線と場の微分を独立に比較する座標
		const auto Coordinates = [&](const UnitDirection Direction) -> std::array<double, 2> {
			const Vec3 Point = Direction.Vector();
			const double Factor = 2 * Radius / (1 + Point.Dot(Up));
			// 立体射影の物理座標の返戻
			return { Factor * Point.Dot(East), Factor * Point.Dot(North) };
		};
		const auto CurvedField = [&](const UnitDirection Direction) -> TerrainSample {
			// 射影座標の解析 Jacobian
			const Vec3 Point = Direction.Vector();
			const double Denominator = 1 + Point.Dot(Up), X = Point.Dot(East), Y = Point.Dot(North);
			const std::array<double, 2> UV = Coordinates(Direction);
			const Vec3 DU = (East * Denominator - Up * X) * (2 * Radius / (Denominator * Denominator));
			const Vec3 DV = (North * Denominator - Up * Y) * (2 * Radius / (Denominator * Denominator));
			const Vec3 Gradient = DU * (0.002 * UV[0]) + DV * (0.004 * UV[1]);
			// 独立な二次式と解析勾配の返戻
			return { 1000 + 0.001 * UV[0] * UV[0] + 0.002 * UV[1] * UV[1], Gradient - Point * Gradient.Dot(Point) };
		};
		// 平面で U=120，V=90 の点を球面へ戻す逆射影
		const double Square = (120.0 * 120 + 90.0 * 90) / (4 * Radius * Radius);
		const UnitDirection Initial = *UnitDirection::FromVector(Up * (1 - Square) + (East * 120 + North * 90) * (1 / Radius));
		TerrainDescentSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.StopElevationMeters = 999;
		Settings.MaximumLengthMeters = 300;
		Settings.MinimumStepMeters = 1E-5;
		Settings.MaximumStepMeters = 20;
		Settings.PositionToleranceMeters = 1E-5;
		const std::optional<TerrainDescentPath> Curve = PlanetCore::TraceTerrainDescent(Initial, CurvedField, Settings);
		// 正規な曲線場を積分出来ない場合の返戻
		if(!TestTrue(TEXT("曲がる解析経路を追跡出来る"), Curve.has_value())) return false;
		TestTrue(TEXT("曲がる解析経路も極小へ到達"), Curve->End == TerrainDescentEnd::GradientLimit);
		for(const PlanetCore::TerrainDescentSample &Sample : Curve->Samples) {
			// 積分器の刻み誤差とは独立な閉形式との一致
			const std::array<double, 2> UV = Coordinates(Sample.Direction);
			const double Residual = std::abs(UV[1] - 0.00625 * UV[0] * UV[0]);
			TestTrue(TEXT("閉形式の曲線から逸れない"), Residual < 0.0005);
			MaximumCurveResidual = std::max(MaximumCurveResidual, Residual);
			++CurvedSamples;
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Descent curved contracts: radii=3 samples=%u maxCurveResidual=%.12g"),
			CurvedSamples,
			MaximumCurveResidual
		)
	);
	// 二分縮小が最小歩幅を飛越す場合でも，許可された最小歩幅その物を試す
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto Bowl = [Radius](const UnitDirection Direction) -> TerrainSample {
			const Vec3 D = Direction.Vector();
			const double X = Radius * D.X, Y = Radius * D.Y;
			const Vec3 Ambient { 2 * Radius * X, 2 * Radius * Y, 0 };
			// 零点が既知の鉢の標高と解析勾配の返戻
			return { X * X + Y * Y, Ambient - D * Ambient.Dot(D) };
		};
		TerrainDescentSettings MinimumSettings;
		MinimumSettings.RadiusMeters = Radius;
		MinimumSettings.StopElevationMeters = -1;
		MinimumSettings.GradientTolerance = 1E-10;
		MinimumSettings.MinimumStepMeters = 0.001;
		MinimumSettings.MaximumStepMeters = 0.0016;
		MinimumSettings.PositionToleranceMeters = 1E-8;
		MinimumSettings.MaximumLengthMeters = 0.01;
		const UnitDirection Start = *UnitDirection::FromVector({ 0.0013 / Radius, 0, 1 });
		const std::optional<TerrainDescentPath> Path = PlanetCore::TraceTerrainDescent(Start, Bowl, MinimumSettings);
		// 正規の最小歩幅で進める経路を取得出来ない場合の返戻
		if(!TestTrue(TEXT("最小歩幅境界の解析経路を準備"), Path && Path->Samples.size() > 1)) return false;
		const Vec3 End = Path->Samples.back().Direction.Vector();
		const double Residual = Radius * std::atan2(std::hypot(End.X, End.Y), End.Z);
		TestTrue(TEXT("元の最小歩幅内に在る極小迄到達"), Residual < MinimumSettings.MinimumStepMeters);
		TestTrue(TEXT("極小を越えず，停止理由を保持"), End.X > 0 && Path->End == TerrainDescentEnd::StepLimit);
		TestTrue(TEXT("許可した最小歩幅未満で進めない"), Path->Samples.back().DistanceMeters >= 0.001 - 1E-12);
		AddInfo(
			FString::Printf(
				TEXT("Descent minimum step: R=%.12g residual=%.12g points=%u reason=%u"),
				Radius,
				Residual,
				static_cast<unsigned>(Path->Samples.size()),
				static_cast<unsigned>(Path->End)
			)
		);
	}
	// 不正入力と途中の場消失，中断の拒否
	TerrainDescentSettings Settings;
	const auto Flat = [](const UnitDirection Direction) -> TerrainSample {
		static_cast<void>(Direction);
		// 有効な平坦場の返戻
		return { 100, {} };
	};
	const std::optional<TerrainDescentPath> FlatPath = PlanetCore::TraceTerrainDescent(Pole, Flat, Settings);
	TestTrue(TEXT("平坦場を大洋到達でなく勾配停止として返す"), FlatPath && FlatPath->End == TerrainDescentEnd::GradientLimit);
	TestFalse(TEXT("場が無い入力を拒否"), PlanetCore::TraceTerrainDescent(Pole, {}, Settings).has_value());
	Settings.MinimumStepMeters = 0;
	TestFalse(TEXT("零の最小刻みを拒否"), PlanetCore::TraceTerrainDescent(Pole, Flat, Settings).has_value());
	Settings = PlanetCore::TerrainDescentSettings{};
	// 最初の場照会中の中断でも候補を公開しない
	std::stop_source Cancelled;
	const auto Cancel = [&](const UnitDirection Direction) -> TerrainSample {
		static_cast<void>(Direction);
		Cancelled.request_stop();
		// 中断に伴う有限な戻値も成功扱いにしない返戻
		return { 100, {} };
	};
	TestFalse(TEXT("場照会中の中断を拒否"), PlanetCore::TraceTerrainDescent(Pole, Cancel, Settings, Cancelled.get_token()).has_value());
	std::uint32_t Calls = 0;
	const auto Invalid = [&](const UnitDirection Direction) -> TerrainSample {
		// 最初だけ有限な場を返し，積分中に非有限値へ移行
		++Calls;
		const double Height = Calls < 3 ? 100 : std::numeric_limits<double>::quiet_NaN();
		// 途中で壊れる意図した場の返戻
		return { Height, (East - Direction.Vector() * East.Dot(Direction.Vector())) * (Settings.RadiusMeters * 0.1) };
	};
	TestFalse(TEXT("積分途中の非有限値を拒否"), PlanetCore::TraceTerrainDescent(Pole, Invalid, Settings).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Descent path contracts: radii=3 samples=%u evaluations=%u maxPlaneError=%.12g maxEndpointError=%.12g sourceChange=%.12g"),
			Samples,
			Evaluations,
			MaximumPlaneError,
			MaximumEndpointError,
			MaximumHeightError
		)
	);
	// 独立な解析経路と停止契約の検査結果の返戻
	return !HasAnyErrors();
}

#endif
