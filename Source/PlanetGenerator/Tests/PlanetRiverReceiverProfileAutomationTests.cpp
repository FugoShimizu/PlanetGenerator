#include "Core/PlanetRiverReceiverProfile.h"
#include "Misc/AutomationTest.h"
#include <cmath>
#include <limits>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetRiverReceiverProfileContracts,
    "PlanetGenerator.RiverReceiverProfile.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 解析的な短い谷と拒否条件で，受入水位から下降する細分化縦断を検査する
 * @param Parameters 未使用の実行引数
 * @return 水深・下降・予算・中断の契約が成立した場合に真
 */
bool FPlanetRiverReceiverProfileContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	constexpr double Radius = 1000000;
	const auto DirectionAt = [](const double X) {
		// 平面上の短区間を厳密な球面方向へ変換した返戻
		return *PlanetCore::UnitDirection::FromVector({ X / Radius, 0, 1 });
	};
	const std::vector<PlanetCore::UnitDirection> Path { DirectionAt(0), DirectionAt(100), DirectionAt(200) };
	const auto Ground = [](const PlanetCore::UnitDirection Direction) {
		const PlanetCore::Vec3 D = Direction.Vector();
		const double X = Radius * D.X / D.Z, Offset = (X - 100) / 15;
		// 中間に狭い上り地形を持つ解析的な標高の返戻
		return 10 - 0.005 * X + 4 * std::exp(-Offset * Offset);
	};
	PlanetCore::RiverReceiverProfileSettings Settings;
	Settings.RadiusMeters = Radius;
	Settings.MaximumSampleStepMeters = 10;
	Settings.MinimumDepthMeters = 0.3;
	Settings.MinimumDownstreamSlope = 1E-6;
	PlanetCore::RiverReceiverProfileError Error = PlanetCore::RiverReceiverProfileError::None;
	const std::optional<PlanetCore::RiverReceiverProfile> Profile =
	PlanetCore::PrepareRiverReceiverProfile(Path, Ground, 10, Settings, Error);
	// 正規の縦断を得られない場合の返戻
	if(!TestTrue(TEXT("解析谷の下降水位を準備"), Profile.has_value())) return false;
	TestTrue(TEXT("解析谷の細分化点数"), Profile->Samples.size() > Path.size());
	TestTrue(TEXT("解析谷の経路長"), std::abs(Profile->Statistics.LengthMeters - 200) < 0.001);
	TestTrue(TEXT("解析谷の上流が実峠より高い"), Profile->Samples.front().WaterMeters > 13);
	TestTrue(TEXT("解析谷の受入水位"), std::abs(Profile->Samples.back().WaterMeters - 10) < 1E-12);
	TestTrue(TEXT("解析谷の最大水深"), Profile->Statistics.MaximumWaterDepthMeters > 0.3);
	for(std::size_t i = 1; i < Profile->Samples.size(); ++i) {
		const PlanetCore::RiverReceiverProfileSample &Up = Profile->Samples[i - 1], &Down = Profile->Samples[i];
		const double Length = Down.DistanceMeters - Up.DistanceMeters;
		TestTrue(TEXT("全標本で正の弧長"), Length > 0);
		TestTrue(TEXT("全標本で下流へ水位が下降"), Up.WaterMeters > Down.WaterMeters);
		TestTrue(TEXT("全標本で必要な河床水深"), Up.WaterMeters + 1E-10 >= Up.GroundMeters + Settings.MinimumDepthMeters);
	}
	TestTrue(
		TEXT("受入河川の必要な水深"),
		Profile->Samples.back().WaterMeters >= Profile->Samples.back().GroundMeters + Settings.MinimumDepthMeters
	);
	const std::optional<PlanetCore::RiverReceiverProfile> Exposed =
	PlanetCore::PrepareRiverReceiverProfile(Path, Ground, Ground(Path.back()) + 0.1, Settings, Error);
	TestFalse(TEXT("受入点に水深が無い場合を拒否"), Exposed.has_value());
	TestTrue(TEXT("受入点露出の理由"), Error == PlanetCore::RiverReceiverProfileError::ReceiverExposed);
	Settings.MaximumSamples = 3;
	const std::optional<PlanetCore::RiverReceiverProfile> Limited =
	PlanetCore::PrepareRiverReceiverProfile(Path, Ground, 10, Settings, Error);
	TestFalse(TEXT("細分化標本予算を超えた縦断を公開しない"), Limited.has_value());
	TestTrue(TEXT("細分化予算の理由"), Error == PlanetCore::RiverReceiverProfileError::SampleBudget);
	Settings.MaximumSamples = 65536;
	const std::vector<PlanetCore::UnitDirection> Duplicated { Path[0], Path[0] };
	TestFalse(TEXT("零長の球面辺を拒否"), PlanetCore::PrepareRiverReceiverProfile(Duplicated, Ground, 10, Settings, Error).has_value());
	TestTrue(TEXT("零長経路の理由"), Error == PlanetCore::RiverReceiverProfileError::InvalidPath);
	const auto MissingGround = [](const PlanetCore::UnitDirection) {
		// 欠落した地形値の返戻
		return std::numeric_limits<double>::quiet_NaN();
	};
	TestFalse(
		TEXT("欠落した地形で縦断を公開しない"),
		PlanetCore::PrepareRiverReceiverProfile(Path, MissingGround, 10, Settings, Error).has_value()
	);
	TestTrue(TEXT("欠落地形の理由"), Error == PlanetCore::RiverReceiverProfileError::InvalidTerrain);
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(
		TEXT("中断後の縦断を公開しない"),
		PlanetCore::PrepareRiverReceiverProfile(Path, Ground, 10, Settings, Error, Cancel.get_token()).has_value()
	);
	TestTrue(TEXT("中断理由"), Error == PlanetCore::RiverReceiverProfileError::Cancelled);
	const std::vector<PlanetCore::RiverReceiverProfileSample> ReceiverSamples {
		{ Path[0], 0, 10, 0 },
		{ Path[1], 100, 14, 0 },
		{ Path[2], 200, 9, 10 }
	};
	PlanetCore::RiverReceiverConditioningSettings ConditioningSettings;
	ConditioningSettings.MinimumDepthMeters = 0.3;
	ConditioningSettings.MaximumDepthMeters = 3;
	ConditioningSettings.MinimumDownstreamSlope = 0.001;
	ConditioningSettings.MaximumCutMeters = 1.3;
	PlanetCore::RiverReceiverConditioningError ConditioningError = PlanetCore::RiverReceiverConditioningError::None;
	PlanetCore::RiverReceiverConditioningDiagnostics Bounds;
	TestFalse(
		TEXT("開削上限未満の経路を拒否"),
		PlanetCore::ConditionRiverReceiverProfile(ReceiverSamples, ConditioningSettings, ConditioningError, &Bounds).has_value()
	);
	TestTrue(TEXT("開削上限の理由"), ConditioningError == PlanetCore::RiverReceiverConditioningError::CutLimit);
	TestTrue(TEXT("必要な最大開削の厳密な下界"), std::abs(Bounds.MinimumRequiredMaximumCutMeters - 1.4) < 1E-12);
	ConditioningSettings.MaximumCutMeters = 1.4;
	const std::optional<PlanetCore::RiverReceiverConditioning> Conditioned =
	PlanetCore::ConditionRiverReceiverProfile(ReceiverSamples, ConditioningSettings, ConditioningError, &Bounds);
	// 下界と一致する条件付けが得られない場合の返戻
	if(!TestTrue(TEXT("下界ちょうどの最大開削で成立"), Conditioned.has_value())) return false;
	TestTrue(TEXT("必要な最大開削量を実際に達成"), std::abs(Conditioned->MaximumActualCutMeters - 1.4) < 1E-12);
	TestTrue(TEXT("最大水深を厳守"), std::abs(Conditioned->MaximumWaterDepthMeters - 3) < 1E-12);
	TestTrue(
		TEXT("受入水位と河床を変更しない"),
		Conditioned->Samples.back().WaterMeters == 10 && Conditioned->Samples.back().ConditionedGroundMeters == 9
	);
	TestTrue(TEXT("正の開削面積"), Conditioned->CutAreaPerUnitWidthSquareMeters > 0);
	for(std::size_t i = 0; i < Conditioned->Samples.size(); ++i) {
		const PlanetCore::RiverReceiverConditioningSample &Point = Conditioned->Samples[i];
		TestTrue(TEXT("全標本で開削のみ"), Point.ConditionedGroundMeters <= Point.OriginalGroundMeters + 1E-12);
		TestTrue(TEXT("全標本で最小水深"), Point.WaterMeters - Point.ConditionedGroundMeters >= 0.3 - 1E-12);
		TestTrue(TEXT("全標本で最大水深"), Point.WaterMeters - Point.ConditionedGroundMeters <= 3 + 1E-12);
		if(i > 0) {
			const PlanetCore::RiverReceiverConditioningSample &Up = Conditioned->Samples[i - 1];
			TestTrue(TEXT("全辺で必要な下降"), Up.WaterMeters - Point.WaterMeters >= 0.001 * (Point.DistanceMeters - Up.DistanceMeters) - 1E-12);
		}
	}
	PlanetCore::RiverReceiverBankSettings BankSettings;
	BankSettings.RadiusMeters = Radius;
	PlanetCore::RiverReceiverBankError BankError = PlanetCore::RiverReceiverBankError::None;
	const auto LowBanks = [](const PlanetCore::UnitDirection) {
		// 解析的に一定の低い両岸標高の返戻
		return 11.0;
	};
	const std::optional<PlanetCore::RiverReceiverBankStatistics> Banks =
	PlanetCore::SurveyRiverReceiverBanks(Conditioned->Samples, LowBanks, BankSettings, BankError);
	// 横断面が作れない場合に未定義の統計値へ進まない返戻
	if(!TestTrue(TEXT("低い両岸を有限幅で測る"), Banks.has_value())) return false;
	TestTrue(TEXT("解析的な横断地点数"), Banks->Stations == 1);
	for(std::size_t Width = 0; Width < BankSettings.HalfWidthsMeters.size(); ++Width) {
		TestTrue(TEXT("全幅の両岸で余裕高が不足"), Banks->BelowFreeboardSides[Width] == 2);
		TestTrue(TEXT("全幅の両岸が水面下"), Banks->SubmergedSides[Width] == 2);
		TestTrue(TEXT("解析的な余裕高不足量"), std::abs(Banks->MaximumFreeboardDeficitMeters[Width] - 2.2) < 1E-12);
	}
	TestFalse(
		TEXT("欠落した河岸の地形値を拒否"),
		PlanetCore::SurveyRiverReceiverBanks(Conditioned->Samples, MissingGround, BankSettings, BankError).has_value()
	);
	TestTrue(TEXT("欠落した河岸の理由"), BankError == PlanetCore::RiverReceiverBankError::InvalidTerrain);
	std::vector<PlanetCore::RiverReceiverConditioningSample> BrokenBanks = Conditioned->Samples;
	BrokenBanks[1].DistanceMeters = 0;
	TestFalse(
		TEXT("重複する縦断地点を河岸測定へ渡さない"),
		PlanetCore::SurveyRiverReceiverBanks(BrokenBanks, LowBanks, BankSettings, BankError).has_value()
	);
	TestTrue(TEXT("不正な横断経路の理由"), BankError == PlanetCore::RiverReceiverBankError::InvalidPath);
	std::vector<PlanetCore::RiverReceiverProfileSample> DeepValley = ReceiverSamples;
	DeepValley[1].GroundMeters = 0;
	ConditioningSettings.MaximumDepthMeters = 5;
	ConditioningSettings.MaximumCutMeters = 100;
	TestFalse(
		TEXT("受入水位に起因する深い谷を開削量に関係なく拒否"),
		PlanetCore::ConditionRiverReceiverProfile(DeepValley, ConditioningSettings, ConditioningError, &Bounds).has_value()
	);
	TestTrue(TEXT("不可避の水深の理由"), ConditioningError == PlanetCore::RiverReceiverConditioningError::IrreducibleDepth);
	TestTrue(TEXT("不可避の水深の下界"), Bounds.MinimumIrreducibleDepthMeters > 10);
	TestFalse(
		TEXT("中断後に条件付けた河床を公開しない"),
		PlanetCore::ConditionRiverReceiverProfile(ReceiverSamples, ConditioningSettings, ConditioningError, &Bounds, Cancel.get_token())
		.has_value()
	);
	TestTrue(TEXT("河床条件付けの中断理由"), ConditioningError == PlanetCore::RiverReceiverConditioningError::Cancelled);
	TestFalse(
		TEXT("河岸測定の中断は部分結果を公開しない"),
		PlanetCore::SurveyRiverReceiverBanks(Conditioned->Samples, LowBanks, BankSettings, BankError, Cancel.get_token()).has_value()
	);
	TestTrue(TEXT("河岸測定の中断理由"), BankError == PlanetCore::RiverReceiverBankError::Cancelled);
	// 有限な解析谷と失敗経路を区別した結果の返戻
	return !HasAnyErrors();
}

#endif
