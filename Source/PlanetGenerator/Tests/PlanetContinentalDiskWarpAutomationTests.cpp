#include "Core/PlanetContinentalDiskWarp.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetContinentalDiskWarpContracts,
	"PlanetGenerator.ContinentalDiskWarp.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 変形の逆，解析微分，面積，海岸の固定と拒否条件を独立に照合する
 * @param Parameters 未使用の UE 引数
 * @return 入力と１２，０００点を超える閉円板で可逆な比較が成立した場合に真
 */
bool FPlanetContinentalDiskWarpContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::ContinentalDiskWarp;
	using PlanetCore::ContinentalDiskWarpMapping;
	using PlanetCore::ContinentalDiskWarpSettings;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	ContinentalDiskWarpSettings Settings;
	Settings.Twists = { { -0.18, 0.08, 0.32, 1.8 }, { 0.26, -0.1, 0.29, -2.1 }, { 0.15, 0.35, 0.16, 0.9 } };
	const std::optional<ContinentalDiskWarp> Warp = ContinentalDiskWarp::Create(Settings);
	// 正規の内部支持を準備出来ない場合の返戻
	if(!TestTrue(TEXT("支持域を海岸内に収めた写像を準備"), Warp.has_value())) return false;
	double MaximumInverse = 0, MaximumDerivative = 0, MaximumArea = 0, MaximumRoundtrip = 0, MaximumCoast = 0;
	double MaximumDisplacement = 0;
	std::uint64_t Samples = 0, CoastSamples = 0;
	for(unsigned row = 0; row < 129; ++row) for(unsigned column = 0; column < 129; ++column) {
		const double X = column / 64.0 - 1, Y = row / 64.0 - 1;
		if(X * X + Y * Y > 1) continue;
		const std::optional<ContinentalDiskWarpMapping> Forward = Warp->MapToSource(X, Y);
		// 閉円板上の点を解像出来ない場合の返戻
		if(!TestTrue(TEXT("閉円板の解析的な変形"), Forward.has_value())) return false;
		const std::optional<ContinentalDiskWarpMapping> Inverse = Warp->MapToWorld(Forward->Coordinates[0], Forward->Coordinates[1]);
		// 逆向きの写像を解像出来ない場合の返戻
		if(!TestTrue(TEXT("変形後の点を逆へ対応付ける"), Inverse.has_value())) return false;
		MaximumInverse = std::max(MaximumInverse, std::hypot(Inverse->Coordinates[0] - X, Inverse->Coordinates[1] - Y));
		MaximumDisplacement = std::max(MaximumDisplacement, std::hypot(Forward->Coordinates[0] - X, Forward->Coordinates[1] - Y));
		const double Area = Forward->Derivatives[0] * Forward->Derivatives[3] - Forward->Derivatives[1] * Forward->Derivatives[2];
		MaximumArea = std::max(MaximumArea, std::abs(Area - 1));
		const std::array<double, 2> First = Inverse->Pushforward(Forward->Pushforward({ 1, 0 }));
		const std::array<double, 2> Second = Inverse->Pushforward(Forward->Pushforward({ 0, 1 }));
		MaximumRoundtrip = std::max(MaximumRoundtrip, std::hypot(First[0] - 1, First[1]));
		MaximumRoundtrip = std::max(MaximumRoundtrip, std::hypot(Second[0], Second[1] - 1));
		for(unsigned axis = 0; axis < 2; ++axis) {
			constexpr double H = 5E-6, Fine = H * 0.5;
			const auto Difference = [&](const double Step) -> std::optional<std::array<double, 2>> {
				const double BX = X - (axis ? 0 : Step), BY = Y - (axis ? Step : 0), AX = X + (axis ? 0 : Step), AY = Y + (axis ? Step : 0);
				const std::optional<ContinentalDiskWarpMapping> Before = Warp->MapToSource(BX, BY), After = Warp->MapToSource(AX, AY);
				// 円板外への差分を内部証拠として使わない返戻
				if(!Before || !After) return {};
				// 値だけを使った独立差分の返戻
				return std::array<double, 2>{
					(After->Coordinates[0] - Before->Coordinates[0]) / (2 * Step),
					(After->Coordinates[1] - Before->Coordinates[1]) / (2 * Step)
				};
			};
			const double DifferenceLimit = std::hypot(X, Y) + H;
			if(DifferenceLimit >= 1) continue;
			const std::optional<std::array<double, 2>> Wide = Difference(H), Narrow = Difference(Fine);
			// 境界を跨がない独立差分を解像出来ない場合の返戻
			if(!TestTrue(TEXT("二段階の円板内差分を評価"), Wide && Narrow)) return false;
			const std::array<double, 2> Analytic =
			Forward->Pushforward(axis ? std::array<double, 2>{ 0, 1 } : std::array<double, 2>{ 1, 0 });
			const double DX = (4 * (*Narrow)[0] - (*Wide)[0]) / 3, DY = (4 * (*Narrow)[1] - (*Wide)[1]) / 3;
			MaximumDerivative = std::max(MaximumDerivative, std::hypot(DX - Analytic[0], DY - Analytic[1]));
		}
		++Samples;
	}
	for(unsigned i = 0; i < 256; ++i) {
		const double Angle = i * (6.283185307179586 / 256), CoastRadius = std::nextafter(1.0, 0.0), X = CoastRadius * std::cos(Angle);
		const double Y = CoastRadius * std::sin(Angle);
		const std::optional<ContinentalDiskWarpMapping> Coast = Warp->MapToSource(X, Y);
		// 海岸その物を解像出来ない場合の返戻
		if(!TestTrue(TEXT("支持外の海岸を評価"), Coast.has_value())) return false;
		MaximumCoast = std::max(MaximumCoast, std::hypot(Coast->Coordinates[0] - X, Coast->Coordinates[1] - Y));
		MaximumCoast = std::max(MaximumCoast, std::abs(Coast->Derivatives[0] - 1));
		MaximumCoast = std::max(MaximumCoast, std::abs(Coast->Derivatives[3] - 1));
		++CoastSamples;
	}
	TestTrue(TEXT("源の位置は実際に変形する"), MaximumDisplacement > 0.04);
	TestTrue(TEXT("円板の解析微分と独立差分が一致"), MaximumDerivative < 1E-7);
	TestTrue(TEXT("両方向の解析微分を合成して戻す"), MaximumRoundtrip < 1E-11);
	TestTrue(TEXT("局所回転の面積倍率は正の１"), MaximumArea < 1E-11);
	TestTrue(TEXT("閉円板の往復と海岸を保つ"), MaximumInverse < 1E-12 && MaximumCoast < 1E-14);
	const std::optional<ContinentalDiskWarp> Again = ContinentalDiskWarp::Create(Settings);
	TestTrue(TEXT("同じ構成の識別子が一致"), Again && Warp->GetDefinitionHash() == Again->GetDefinitionHash());
	Settings.Twists[1].AngleRadians += 0.01;
	const std::optional<ContinentalDiskWarp> Different = ContinentalDiskWarp::Create(Settings);
	TestTrue(TEXT("角度の変更が識別子へ反映"), Different && Different->GetDefinitionHash() != Warp->GetDefinitionHash());
	Settings.Twists[0].Radius = 1;
	TestFalse(TEXT("海岸を跨ぐ支持を拒否"), ContinentalDiskWarp::Create(Settings).has_value());
	Settings.Twists[0].Radius = 0.32;
	Settings.Twists[0].AngleRadians = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("非有限の角度を拒否"), ContinentalDiskWarp::Create(Settings).has_value());
	TestFalse(TEXT("閉円板の外を拒否"), Warp->MapToSource(1.01, 0).has_value());
	Settings.Twists.clear();
	const std::optional<ContinentalDiskWarp> Identity = ContinentalDiskWarp::Create(Settings);
	TestTrue(TEXT("無変形も正規で，海岸の外挿をしない"), Identity && Identity->GetTwistCount() == 0);
	AddInfo(
		FString::Printf(
			TEXT("Continental disk warp: points=%llu coast=%llu inverse=%.12g derivative=%.12g area=%.12g ")
			TEXT("JacobianRoundtrip=%.12g displacement=%.12g seconds=%.7f. Geometry only; erosion and scenery remain pending."),
			static_cast<unsigned long long>(Samples),
			static_cast<unsigned long long>(CoastSamples),
			MaximumInverse,
			MaximumDerivative,
			MaximumArea,
			MaximumRoundtrip,
			MaximumDisplacement,
			std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count()
		)
	);
	// 閉円板での値・微分・海岸・拒否条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
