#include "Core/PlanetClimate.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>

#if WITH_DEV_AUTOMATION_TESTS
namespace {
	/**
	 * 全球を等面積に近い分布で独立に標本化する
	 * @param Index 標本番号
	 * @param Count 標本数
	 * @return 単位球上の方向
	 */
	PlanetCore::UnitDirection ClimateProbeDirection(const std::uint32_t Index, const std::uint32_t Count) {
		const double Height = 1 - 2 * (Index + 0.5) / Count;
		const double Azimuth = Index * 2.399963229728653;
		const double Width = std::sqrt(1 - Height * Height);
		// 独立な評価方向の返戻
		return *PlanetCore::UnitDirection::FromVector({ Width * std::cos(Azimuth), Width * std::sin(Azimuth), Height });
	}

	/**
	 * 二つの気候の最大成分差を測る
	 * @param First 第１の気候
	 * @param Second 第２の気候
	 * @return 温度・降水・蒸発散の最大絶対差，成分毎の単位の儘
	 */
	double ClimateDifference(const PlanetCore::ClimateCycle &First, const PlanetCore::ClimateCycle &Second) {
		double Difference = 0;
		for(std::size_t i = 0; i < PlanetCore::ClimatePeriodCount; ++i) {
			Difference = std::max(Difference, std::abs(First.Periods[i].TemperatureC - Second.Periods[i].TemperatureC));
			Difference = std::max(Difference, std::abs(First.Periods[i].PrecipitationMm - Second.Periods[i].PrecipitationMm));
			Difference = std::max(Difference,
				std::abs(First.Periods[i].PotentialEvapotranspirationMm - Second.Periods[i].PotentialEvapotranspirationMm));
		}
		// 最大差の返戻
		return Difference;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetClimateContracts, "PlanetGenerator.Climate.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 日射幾何と実地形の全球気候・補間・バイオーム接続を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，個別の不一致は UE に記録
 */
bool FPlanetClimateContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Begin = FPlatformTime::Seconds();
	double MaximumSolarError = 0;

	// 日平均式と，一日の正の日射だけを足す独立な数値積分
	for(const double Declination : { -0.3978, 0.0, 0.3978 }) {
		for(std::uint32_t i = 0; i < 23; ++i) {
			const double Latitude = -1 + i / 11.0;
			const std::optional<double> Projection = PlanetCore::PlanetClimate::DailyMeanSolarProjection(Latitude, Declination);
			// 正規な日射入力が拒否された場合の返戻
			if(!TestTrue(TEXT("日射の正規入力を受理"), Projection.has_value())) return false;
			double Integrated = 0;
			for(std::uint32_t j = 0; j < 4096; ++j) {
				const double Angle = 2 * std::numbers::pi * (j + 0.5) / 4096;
				const double Incidence = Latitude * Declination +
					std::sqrt((1 - Latitude * Latitude) * (1 - Declination * Declination)) * std::cos(Angle);
				Integrated += std::max(0.0, Incidence) / 4096;
			}
			MaximumSolarError = std::max(MaximumSolarError, std::abs(Integrated - *Projection));
			const std::optional<double> Opposite = PlanetCore::PlanetClimate::DailyMeanSolarProjection(-Latitude, -Declination);
			TestTrue(TEXT("南北と季節の同時反転"), Opposite && std::abs(*Projection - *Opposite) < 1.0E-14);
		}
		double GlobalMean = 0;
		for(std::uint32_t i = 0; i < 20000; ++i) {
			GlobalMean += *PlanetCore::PlanetClimate::DailyMeanSolarProjection(1 - 2 * (i + 0.5) / 20000, Declination) / 20000;
		}
		TestTrue(TEXT("球全体の日射投影平均は四分の一"), std::abs(GlobalMean - 0.25) < 1.0E-6);
	}
	TestTrue(TEXT("日平均式と独立積分の一致"), MaximumSolarError < 1.0E-6);
	TestFalse(TEXT("範囲外の日射入力を拒否"), PlanetCore::PlanetClimate::DailyMeanSolarProjection(1.001, 0).has_value());
	TestFalse(TEXT("非有限の日射入力を拒否"),
		PlanetCore::PlanetClimate::DailyMeanSolarProjection(0, std::numeric_limits<double>::quiet_NaN()).has_value());

	// 最小半径を含む実地形，検査用の緯度色分けに置き換えない
	const std::array<double, 3> Radii{ 1000, 1000000, 6371000 };
	std::uint32_t TotalSamples = 0;
	double MaximumIndexError = 0, MaximumBoundaryChange = 0, QuerySeconds = 0;
	for(std::uint32_t Case = 0; Case < Radii.size(); ++Case) {
		PlanetCore::GeologySettings GeologySettings;
		GeologySettings.RadiusMeters = Radii[Case];
		GeologySettings.Seed = Case == 2 ? 7 : 42;
		const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(GeologySettings);
		// 地質の準備失敗の返戻
		if(!TestTrue(TEXT("気候検査の実地質を準備"), Geology.has_value())) return false;
		const std::optional<PlanetCore::OceanConnectivityField> Ocean =
			PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
		// 海陸の準備失敗の返戻
		if(!TestTrue(TEXT("気候検査の海陸を準備"), Ocean.has_value())) return false;
		PlanetCore::ClimatePreparationStatistics Diagnostics;
		const double PreparationBegin = FPlatformTime::Seconds();
		const std::optional<PlanetCore::PlanetClimate> Climate = PlanetCore::PlanetClimate::Create(*Ocean, {}, {}, &Diagnostics);
		const double PreparationSeconds = FPlatformTime::Seconds() - PreparationBegin;
		if(!TestTrue(TEXT("実地形の全季節気候が収束"), Climate.has_value())) {
			AddError(FString::Printf(TEXT("Climate failure: case=%u stage=%u transport=%u periods=%u sweeps=%u residual=%.9g budget=%.9g"),
				Case, static_cast<unsigned>(Diagnostics.Failure), static_cast<unsigned>(Diagnostics.TransportFailure),
				Diagnostics.CompletedPeriods, Diagnostics.MaximumSweeps, Diagnostics.MaximumResidualMillimetersPerDay,
				Diagnostics.MaximumRelativeBudgetError));
			// 公開出来ない気候の返戻
			return false;
		}
		const PlanetCore::ClimatePreparationStatistics &Statistics = Climate->PreparationStatistics();
		TestEqual(TEXT("１２期全てを準備"), Statistics.CompletedPeriods, 12U);
		TestEqual(TEXT("視点に依存しない２，５６２地点"), Statistics.Nodes, 2562U);
		TestEqual(TEXT("閉じた球の７，６８０隣接対"), Statistics.Links, 7680U);
		TestTrue(TEXT("輸送領域が球全体を覆う"), std::abs(Statistics.SurfaceAreaSteradians - 4 * std::numbers::pi) < 1.0E-10);
		TestTrue(TEXT("輸送の局所残差"), Statistics.MaximumResidualMillimetersPerDay <= 1.0E-9);
		TestTrue(TEXT("輸送の全球収支"), Statistics.MaximumRelativeBudgetError <= 1.0E-8);
		TestTrue(TEXT("全球降水が有限且つ正"), std::isfinite(Statistics.GlobalAnnualPrecipitationMillimeters) &&
			Statistics.GlobalAnnualPrecipitationMillimeters > 0);
		std::uint32_t BiomeMask = 0, LandSamples = 0;
		double MinimumLandRain = std::numeric_limits<double>::max(), MaximumLandRain = 0;
		for(std::uint32_t i = 0; i < 384; ++i) {
			const PlanetCore::UnitDirection Direction = ClimateProbeDirection(i, 384);
			const double QueryBegin = FPlatformTime::Seconds();
			const std::optional<PlanetCore::ClimateCycle> Sample = Climate->Sample(Direction);
			QuerySeconds += FPlatformTime::Seconds() - QueryBegin;
			// 支持範囲や入力が不正な場合の返戻
			if(!TestTrue(TEXT("全球の任意地点で気候を評価"), Sample.has_value())) return false;
			++TotalSamples;
			if(!(i % 8)) {
				const std::optional<PlanetCore::ClimateCycle> Reference = Climate->SampleReference(Direction);
				// 独立探索に失敗した場合の返戻
				if(!TestTrue(TEXT("全地点探索で気候を評価"), Reference.has_value())) return false;
				MaximumIndexError = std::max(MaximumIndexError, ClimateDifference(*Sample, *Reference));
			}
			const std::optional<PlanetCore::BiomeProfile> Biome = Climate->SampleBiome(Direction);
			// 気候からバイオームへの接続失敗の返戻
			if(!TestTrue(TEXT("生成した実気候からバイオーム応答を準備"), Biome.has_value())) return false;
			TestTrue(TEXT("バイオームの周期水収支"), std::abs(Biome->Statistics().WaterBudgetResidualMm) < 1.0E-7);
			if(Ocean->EvaluateElevation(Direction) > GeologySettings.SeaLevelMeters) {
				const std::optional<PlanetCore::BiomeHabitat> Habitat = Biome->Habitat({});
				// 陸上の生育応答が欠けた場合の返戻
				if(!TestTrue(TEXT("陸上のバイオームが存在"), Habitat && Habitat->Dominant.has_value())) return false;
				BiomeMask |= 1U << static_cast<unsigned>(*Habitat->Dominant);
				MinimumLandRain = std::min(MinimumLandRain, Biome->Statistics().AnnualPrecipitationMm);
				MaximumLandRain = std::max(MaximumLandRain, Biome->Statistics().AnnualPrecipitationMm);
				++LandSamples;
			}
		}
		TestTrue(TEXT("陸上の降水が一様な定数ではない"), LandSamples && MaximumLandRain > MinimumLandRain + 1);

		// 索引セルの境界を跨ぐ微小移動で，地理的な切れ目を作らない
		for(std::uint32_t i = 1; i < 16; ++i) {
			const double Position = -1 + i / 8.0;
			const double Width = std::sqrt(1 - Position * Position);
			const PlanetCore::Vec3 Center{ Position, Width * 0.6, Width * 0.8 };
			const PlanetCore::UnitDirection Left = *PlanetCore::UnitDirection::FromVector(Center + PlanetCore::Vec3{ -1.0E-8, 0, 0 });
			const PlanetCore::UnitDirection Right = *PlanetCore::UnitDirection::FromVector(Center + PlanetCore::Vec3{ 1.0E-8, 0, 0 });
			const std::optional<PlanetCore::ClimateCycle> First = Climate->Sample(Left), Second = Climate->Sample(Right);
			// 索引境界の支持が欠けた場合の返戻
			if(!TestTrue(TEXT("索引境界の両側で評価"), First && Second)) return false;
			MaximumBoundaryChange = std::max(MaximumBoundaryChange, ClimateDifference(*First, *Second));
		}
		AddInfo(FString::Printf(
			TEXT("Climate: seed=%llu radius=%.0f nodes=%u periods=%u sweeps=%u budget=%.9g annualRain=%.6g ")
			TEXT("landRain=%.6g..%.6g biomes=%u seconds=%.6f"),
			static_cast<unsigned long long>(GeologySettings.Seed), GeologySettings.RadiusMeters, Statistics.Nodes,
			Statistics.CompletedPeriods, Statistics.MaximumSweeps, Statistics.MaximumRelativeBudgetError,
			Statistics.GlobalAnnualPrecipitationMillimeters, MinimumLandRain, MaximumLandRain, BiomeMask, PreparationSeconds
		));

		if(!Case) {
			// 準備し直しても同一実行環境の値を再現し，気候設定を識別する
			const std::optional<PlanetCore::PlanetClimate> Again = PlanetCore::PlanetClimate::Create(*Ocean);
			// 再準備が失敗した場合の返戻
			if(!TestTrue(TEXT("独立再準備が収束"), Again.has_value())) return false;
			TestEqual(TEXT("再準備した定義の一致"), Again->IdentityHash(), Climate->IdentityHash());
			for(std::uint32_t i = 0; i < 16; ++i) {
				const PlanetCore::UnitDirection Direction = ClimateProbeDirection(i, 16);
				const std::optional<PlanetCore::ClimateCycle> First = Climate->Sample(Direction), Second = Again->Sample(Direction);
				// 再準備後の評価失敗の返戻
				if(!TestTrue(TEXT("再準備した場の評価"), First && Second)) return false;
				for(std::size_t j = 0; j < PlanetCore::ClimatePeriodCount; ++j) {
					TestTrue(TEXT("再準備した降水のビット一致"),
						std::bit_cast<std::uint64_t>(First->Periods[j].PrecipitationMm) ==
						std::bit_cast<std::uint64_t>(Second->Periods[j].PrecipitationMm));
				}
			}
			PlanetCore::ClimateSettings NoTilt;
			NoTilt.AxialTiltRadians = 0;
			const std::optional<PlanetCore::PlanetClimate> Constant = PlanetCore::PlanetClimate::Create(*Ocean, NoTilt);
			// 季節変化の無い正規設定が拒否された場合の返戻
			if(!TestTrue(TEXT("軸傾斜ゼロの正規入力"), Constant.has_value())) return false;
			TestTrue(TEXT("気候設定の変更で定義を分離"), Constant->IdentityHash() != Climate->IdentityHash());
			const std::optional<PlanetCore::ClimateCycle> ConstantSample = Constant->Sample(ClimateProbeDirection(0, 1));
			// 軸傾斜ゼロの評価失敗の返戻
			if(!TestTrue(TEXT("軸傾斜ゼロで評価"), ConstantSample.has_value())) return false;
			for(const PlanetCore::ClimatePeriod &Period : ConstantSample->Periods) {
				TestTrue(TEXT("固定日射の周期温度"), std::abs(Period.TemperatureC - ConstantSample->Periods[0].TemperatureC) < 1.0E-10);
				TestTrue(TEXT("固定日射の周期降水"), std::abs(Period.PrecipitationMm - ConstantSample->Periods[0].PrecipitationMm) < 1.0E-8);
			}
			PlanetCore::ClimateSettings Invalid;
			Invalid.NorthAxis = { 0, 0, 2 };
			TestFalse(TEXT("不正な北極軸を拒否"), PlanetCore::PlanetClimate::Create(*Ocean, Invalid, {}, &Diagnostics).has_value());
			TestTrue(TEXT("以前の成功証跡を消去"),
				Diagnostics.Failure == PlanetCore::ClimatePreparationFailure::InvalidSettings && !Diagnostics.CompletedPeriods);
			std::stop_source Stop;
			Stop.request_stop();
			TestFalse(TEXT("中断した場を公開しない"),
				PlanetCore::PlanetClimate::Create(*Ocean, {}, Stop.get_token(), &Diagnostics).has_value());
			TestTrue(TEXT("中断理由を記録"), Diagnostics.Failure == PlanetCore::ClimatePreparationFailure::Cancelled);
		}
	}
	TestTrue(TEXT("索引と全点探索の一致"), MaximumIndexError < 1.0E-8);
	TestTrue(TEXT("索引境界の微小移動で気候が連続"), MaximumBoundaryChange < 0.01);
	AddInfo(FString::Printf(
		TEXT("Climate contracts: samples=%u solarError=%.9g indexError=%.9g ")
		TEXT("boundaryChange=%.9g querySeconds=%.6f totalSeconds=%.6f"),
		TotalSamples, MaximumSolarError, MaximumIndexError, MaximumBoundaryChange, QuerySeconds, FPlatformTime::Seconds() - Begin));
	// 検査実行結果の返戻
	return true;
}
#endif
