#include "Core/PlanetFluvialErosion.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetFluvialErosionContracts, "PlanetGenerator.FluvialErosion.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * ２地点の閉形式，放射状土柱の体積，縮尺，実流域の土砂収支を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，個別の不一致は UE に記録
 */
bool FPlanetFluvialErosionContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Start = FPlatformTime::Seconds();
	/**
	 * 年間 1.2mの雨が全量流出する局所気候を与える
	 * @param Direction 一様な気候のため未使用の方向
	 * @return 蒸発散の無い周期的な気候
	 */
	const auto WetClimate = [](const PlanetCore::UnitDirection Direction) {
		(void)Direction;
		PlanetCore::ClimateCycle Cycle;
		for(PlanetCore::ClimatePeriod &Period : Cycle.Periods) Period = {20, 100, 0};
		// 雨が流出へ対応する局所応答の返戻
		return PlanetCore::BiomeProfile::Create(Cycle);
	};
	PlanetCore::FluvialErosionSettings Erosion;
	Erosion.Years = 10;
	Erosion.ReferenceRadiusMeters = 1000;
	Erosion.ReferenceDischargeCubicMetersPerSecond = 12 / PlanetCore::PlanetRunoff::ReferenceYearSeconds;
	double MaximumHeightError = 0, MaximumVolumeError = 0;
	for(const double Radius : {1000.0, 1E6, 1E9})
		for(const double Sea : {0.0, 20.0}) {
			const double Area = 10 * (Radius / 1000) * (Radius / 1000);
			const std::array<PlanetCore::DrainageSite, 2> Sites{PlanetCore::DrainageSite{{1, 0, 0}, Sea - 1, Area, true},
				PlanetCore::DrainageSite{{std::cos(0.01), std::sin(0.01), 0}, Sea + 100, Area, false}};
			const std::array<std::array<std::uint32_t, 2>, 1> Edges{{{0, 1}}};
			PlanetCore::DrainageSettings Settings;
			Settings.RadiusMeters = Radius;
			Settings.SeaLevelMeters = Sea;
			const std::optional<PlanetCore::DrainageNetwork> Network =
				PlanetCore::DrainageNetwork::Solve(Sites, Edges, Settings);
			// 解析用の排水を準備出来なかった場合の返戻
			if(!TestTrue(TEXT("解析用の２地点を準備"), Network.has_value())) return false;
			const std::optional<PlanetCore::PlanetRunoff> Flow = PlanetCore::PlanetRunoff::FromProfiles(*Network, WetClimate);
			// 解析用の流量を準備出来なかった場合の返戻
			if(!TestTrue(TEXT("解析用の流量を準備"), Flow.has_value())) return false;
			TestEqual(TEXT("局所面積を直接共有"), Flow->GetNodeAreas()[1], Area);
			const std::optional<PlanetCore::PlanetFluvialErosion> Result =
				PlanetCore::PlanetFluvialErosion::Create(*Network, *Flow, Erosion);
			// 解析用の侵食を準備出来なかった場合の返戻
			if(!TestTrue(TEXT("解析用の侵食を準備"), Result.has_value())) return false;
			// 参照長１０m・移流速度１m/年・１０年の陰解法は標高差を半分にする
			MaximumHeightError = std::max(MaximumHeightError, std::abs(Result->GetNodes()[1].ElevationMeters - Sea - 50));
			const double Before = 1 + (Sea + 100) / Radius, After = 1 + (Sea + 50) / Radius;
			const double Volume = Area * 50 * (Before * Before + Before * After + After * After) / 3;
			MaximumVolumeError =
				std::max(MaximumVolumeError, std::abs(Result->GetNodes()[1].ErodedCubicMeters - Volume) / Volume);
			TestEqual(TEXT("海底の地形を勝手に変えない"), Result->GetNodes()[0].ElevationMeters, Sea - 1);
			TestEqual(TEXT("削った全体積が海へ到達"), Result->GetNodes()[0].OutgoingSedimentCubicMeters,
				Result->GetNodes()[1].ErodedCubicMeters);
			for(std::uint32_t Case = 0; Case < 3; ++Case) {
				PlanetCore::FluvialErosionSettings Disabled = Erosion;
				if(Case == 0)
					Disabled.Years = 0;
				else if(Case == 1)
					Disabled.ReferenceCelerityMetersPerYear = 0;
				else
					Disabled.MaximumFractionalLowering = 0;
				const std::optional<PlanetCore::PlanetFluvialErosion> Identity =
					PlanetCore::PlanetFluvialErosion::Create(*Network, *Flow, Disabled);
				// 無効化した侵食が失敗した場合の返戻
				if(!TestTrue(TEXT("零時間・零速度・零許容量を受理"), Identity.has_value())) return false;
				TestEqual(TEXT("無効化した侵食は標高を変えない"), Identity->GetNodes()[1].ElevationMeters, Sea + 100);
				TestEqual(TEXT("無効化した侵食は土砂を作らない"), Identity->GetStatistics().TotalErodedCubicMeters, 0.0);
			}
			const std::optional<PlanetCore::PlanetRunoff> DryFlow = PlanetCore::PlanetRunoff::FromProfiles(*Network,
				/**
				 * 無降水で流量を生じない気候を用意する
				 * @param Direction 一様な気候のため未使用
				 * @return 乾燥した周期応答
				 */
				[](const PlanetCore::UnitDirection Direction) {
					(void)Direction;
					PlanetCore::ClimateCycle Cycle;
					for(PlanetCore::ClimatePeriod &Period : Cycle.Periods) Period = {20, 0, 100};
					// 流量の無い気候の返戻
					return PlanetCore::BiomeProfile::Create(Cycle);
				});
			// 無降水の流量を用意出来なかった場合の返戻
			if(!TestTrue(TEXT("無降水の流量を準備"), DryFlow.has_value())) return false;
			const std::optional<PlanetCore::PlanetFluvialErosion> DryResult =
				PlanetCore::PlanetFluvialErosion::Create(*Network, *DryFlow, Erosion);
			// 無降水の侵食計算を用意出来なかった場合の返戻
			if(!TestTrue(TEXT("無降水の侵食計算を準備"), DryResult.has_value())) return false;
			TestEqual(TEXT("流量の無い地形を削らない"), DryResult->GetNodes()[1].ElevationMeters, Sea + 100);
			TestEqual(TEXT("流量の無い地形から土砂を作らない"), DryResult->GetStatistics().TotalErodedCubicMeters, 0.0);
			PlanetCore::FluvialErosionSettings Limited = Erosion;
			Limited.Years = 1000;
			const std::optional<PlanetCore::PlanetFluvialErosion> Bound =
				PlanetCore::PlanetFluvialErosion::Create(*Network, *Flow, Limited);
			// 侵食上限を持つ準備が失敗した場合の返戻
			if(!TestTrue(TEXT("侵食量を制限した結果を準備"), Bound.has_value())) return false;
			TestTrue(TEXT("海面以上の地形を反転させない"), Bound->GetNodes()[1].ElevationMeters > Sea);
			TestTrue(TEXT("指定した相対起伏を残す"), std::abs(Bound->GetNodes()[1].ElevationMeters - Sea - 20) < 1E-10);
			TestEqual(TEXT("制限が発動した地点を記録"), Bound->GetStatistics().LoweringLimitedNodes, 1U);
			std::stop_source Stop;
			Stop.request_stop();
			TestFalse(TEXT("中断された侵食を公開しない"),
				PlanetCore::PlanetFluvialErosion::Create(*Network, *Flow, Erosion, Stop.get_token()).has_value());
			Limited.Years = -1;
			TestFalse(
				TEXT("負の侵食時間を拒否"), PlanetCore::PlanetFluvialErosion::Create(*Network, *Flow, Limited).has_value());
		}
	TestTrue(TEXT("３半径・２海面の閉形式に一致"), MaximumHeightError < 1E-10);
	TestTrue(TEXT("土柱体積が縮尺に応じて一致"), MaximumVolumeError < 1E-12);
	PlanetCore::GeologySettings Definition;
	const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Definition);
	// 実地質を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("実地質を準備"), Geology.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
	// 実海域を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("実海域を準備"), Ocean.has_value())) return false;
	const std::optional<PlanetCore::PlanetClimate> Climate = PlanetCore::PlanetClimate::Create(*Ocean);
	const std::optional<PlanetCore::DrainageNetwork> Network = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 実気候と排水を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("実気候と排水を準備"), Climate && Network)) return false;
	const std::optional<PlanetCore::PlanetRunoff> Flow = PlanetCore::PlanetRunoff::Create(*Network, *Climate);
	// 実流量を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("実流量を準備"), Flow.has_value())) return false;
	const double ErosionStart = FPlatformTime::Seconds();
	const std::optional<PlanetCore::PlanetFluvialErosion> Result = PlanetCore::PlanetFluvialErosion::Create(*Network, *Flow);
	const double ErosionSeconds = FPlatformTime::Seconds() - ErosionStart;
	// 実侵食を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("実流域の侵食を準備"), Result.has_value())) return false;
	for(std::uint32_t i = 0; i < Network->Nodes().size(); ++i) {
		const PlanetCore::DrainageNode &Node = Network->Nodes()[i];
		const PlanetCore::FluvialErosionNode &Value = Result->GetNodes()[i];
		TestTrue(TEXT("元の地形より標高を上げない"), Value.ElevationMeters <= Node.OriginalElevationMeters);
		if(Node.Ocean)
			TestEqual(TEXT("全海底の標高を保持"), Value.ElevationMeters, Node.OriginalElevationMeters);
		else
			TestTrue(TEXT("全陸上地点が海面上"), Value.ElevationMeters > Definition.SeaLevelMeters);
		TestTrue(TEXT("非負且つ有限な土砂"),
			std::isfinite(Value.OutgoingSedimentCubicMeters) && Value.OutgoingSedimentCubicMeters >= 0);
	}
	const PlanetCore::FluvialErosionStatistics &Stats = Result->GetStatistics();
	TestTrue(TEXT("侵食と海への輸送が生じる"), Stats.ErodedNodes > 0 && Stats.OceanDeliveredCubicMeters > 0);
	TestTrue(TEXT("削った土砂を消失させない"), Stats.SedimentRelativeResidual < 1E-10);
	AddInfo(
		FString::Printf(TEXT("Fluvial erosion: heightError=%.9g volumeRelativeError=%.9g nodes=%u limited=%u maxLowering=%.9g ")
							TEXT("removed=%.9g delivered=%.9g residual=%.9g erosionSeconds=%.6f total=%.6f"),
			MaximumHeightError, MaximumVolumeError, Stats.ErodedNodes, Stats.LoweringLimitedNodes, Stats.MaximumLoweringMeters,
			Stats.TotalErodedCubicMeters, Stats.OceanDeliveredCubicMeters, Stats.SedimentRelativeResidual, ErosionSeconds,
			FPlatformTime::Seconds() - Start));
	// 検査実行結果の返戻
	return true;
}
#endif
