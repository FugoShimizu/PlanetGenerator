#include "Core/PlanetRunoff.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetRunoffContracts, "PlanetGenerator.Runoff.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 独立に積算出来る合流，乾燥・凍結，実惑星の海への水収支を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，個別の不一致は UE へ記録
 */
bool FPlanetRunoffContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Start = FPlatformTime::Seconds();
	const std::array<PlanetCore::DrainageSite, 4> Sites{PlanetCore::DrainageSite{{1, 0, 0}, -1, 1, true},
		PlanetCore::DrainageSite{{1, 0.01, 0}, 1, 10, false}, PlanetCore::DrainageSite{{1, 0.02, 0.01}, 2, 20, false},
		PlanetCore::DrainageSite{{1, 0.02, -0.01}, 2, 30, false}};
	const std::array<std::array<std::uint32_t, 2>, 3> Edges{{{0, 1}, {1, 2}, {1, 3}}};
	PlanetCore::DrainageSettings Settings;
	Settings.RadiusMeters = 1000;
	Settings.ChannelAreaFraction = 1E-8;
	const std::optional<PlanetCore::DrainageNetwork> Network = PlanetCore::DrainageNetwork::Solve(Sites, Edges, Settings);
	// 解析用の合流を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("解析用の排水を準備"), Network.has_value())) return false;
	std::uint32_t ProviderCalls = 0;
	/**
	 * 蒸発散の無い暖かい合流を構成する
	 * @param Direction 両支流を識別する方向
	 * @return 降水が全量流出する周期解
	 */
	const auto Provider = [&ProviderCalls](const PlanetCore::UnitDirection Direction) {
		++ProviderCalls;
		const double Scale = Direction.Vector().Z > 0 ? 2 : Direction.Vector().Z < 0 ? 0.5 : 1;
		PlanetCore::ClimateCycle Cycle;
		for(std::uint32_t i = 0; i < PlanetCore::ClimatePeriodCount; ++i) Cycle.Periods[i] = {20, Scale * (100 + i * 10), 0};
		// 年間で貯留が増えない局所気候の返戻
		return PlanetCore::BiomeProfile::Create(Cycle);
	};
	const std::optional<PlanetCore::PlanetRunoff> Flow = PlanetCore::PlanetRunoff::FromProfiles(*Network, Provider);
	// 流量を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("解析解付きの流量を準備"), Flow.has_value())) return false;
	TestEqual(TEXT("海面の降水を陸域へ二重計上しない"), ProviderCalls, 3U);
	constexpr double PeriodSeconds = PlanetCore::PlanetRunoff::ReferenceYearSeconds / PlanetCore::ClimatePeriodCount;
	double MaximumAnalyticError = 0, MaximumJunctionError = 0;
	for(std::uint32_t i = 0; i < PlanetCore::ClimatePeriodCount; ++i) {
		const double Rain = (100 + i * 10) * 0.001;
		const std::array<double, 4> Expected{Rain * 65, Rain * 65, Rain * 40, Rain * 15};
		for(std::uint32_t j = 0; j < 4; ++j) {
			MaximumAnalyticError =
				std::max(MaximumAnalyticError, std::abs(Flow->GetPeriods()[j][i] * PeriodSeconds - Expected[j]));
		}
	}
	TestTrue(TEXT("各支流と合流・海の量が独立な降水積算と一致"), MaximumAnalyticError < 1E-10);
	TestTrue(TEXT("既知の年降水量が海へ到達"), std::abs(Flow->GetStatistics().AnnualOceanRunoffCubicMeters - 120.9) < 1E-9);
	for(std::uint32_t i = 0; i < 1201; ++i) {
		const double Phase = i / 1200.0;
		const double Incoming = *Flow->Sample(2, Phase) + *Flow->Sample(3, Phase);
		const double Junction = *Flow->Sample(1, Phase);
		MaximumJunctionError = std::max(MaximumJunctionError, std::abs(Junction - Incoming * (65.0 / 55)));
		TestTrue(TEXT("位相補間でも下流流量が支流の和以上"), Junction >= Incoming);
		TestEqual(TEXT("海へ出る同じ量を保持"), *Flow->Sample(0, Phase), Junction);
	}
	TestTrue(TEXT("合流の局所寄与を補間でも保持"), MaximumJunctionError < 1E-14);
	TestEqual(TEXT("年境界が周期的"), *Flow->Sample(1, -0.25), *Flow->Sample(1, 0.75));
	TestFalse(TEXT("範囲外の地点を拒否"), Flow->Sample(4, 0).has_value());
	TestFalse(TEXT("非有限な位相を拒否"), Flow->Sample(0, std::numeric_limits<double>::quiet_NaN()).has_value());
	const std::optional<PlanetCore::PlanetRunoff> Again = PlanetCore::PlanetRunoff::FromProfiles(*Network, Provider);
	// 再準備出来なかった場合の返戻
	if(!TestTrue(TEXT("再準備を受理"), Again.has_value())) return false;
	TestTrue(
		TEXT("再準備が同一値"), std::equal(Flow->GetPeriods().begin(), Flow->GetPeriods().end(), Again->GetPeriods().begin()));

	for(const bool IsFrozen : {false, true}) {
		/**
		 * 無降水又は恒常的な降雪を与える
		 * @param Direction この一様な検査では未使用の方向
		 * @return 一様な局所応答
		 */
		const auto Extreme = [IsFrozen](const PlanetCore::UnitDirection Direction) {
			(void)Direction;
			PlanetCore::ClimateCycle Cycle;
			for(PlanetCore::ClimatePeriod &Period : Cycle.Periods)
				Period = IsFrozen ? PlanetCore::ClimatePeriod{-20, 100, 0} : PlanetCore::ClimatePeriod{20, 0, 100};
			// 乾燥又は凍結の局所応答の返戻
			return PlanetCore::BiomeProfile::Create(Cycle);
		};
		const std::optional<PlanetCore::PlanetRunoff> Result = PlanetCore::PlanetRunoff::FromProfiles(*Network, Extreme);
		// 極端な気候を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("乾燥・凍結を有効な収支として扱う"), Result.has_value())) return false;
		TestTrue(TEXT("無い河川水を捏造しない"), Result->GetStatistics().AnnualOceanRunoffCubicMeters < 1E-9);
		TestTrue(TEXT("氷河輸送を別の収支に残す"),
			std::abs(Result->GetStatistics().AnnualFrozenExportCubicMeters - (IsFrozen ? 72 : 0)) < 1E-8);
	}
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(TEXT("中断された流量を公開しない"),
		PlanetCore::PlanetRunoff::FromProfiles(*Network, Provider, Stop.get_token()).has_value());
	TestFalse(TEXT("空の気候供給を拒否"), PlanetCore::PlanetRunoff::FromProfiles(*Network, {}).has_value());

	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Definition);
	// 実地質を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("流量検査の実地質を準備"), Geology.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
	// 実海域を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("流量検査の海域を準備"), Ocean.has_value())) return false;
	const std::optional<PlanetCore::PlanetClimate> Climate = PlanetCore::PlanetClimate::Create(*Ocean);
	const std::optional<PlanetCore::DrainageNetwork> Drainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 実気候と排水を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("同じ海陸から気候と排水を準備"), Climate && Drainage)) return false;
	const double FlowStart = FPlatformTime::Seconds();
	const std::optional<PlanetCore::PlanetRunoff> Planet = PlanetCore::PlanetRunoff::Create(*Drainage, *Climate);
	const double FlowSeconds = FPlatformTime::Seconds() - FlowStart;
	// 実惑星の流量を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("実惑星の水収支を満たす"), Planet.has_value())) return false;
	const PlanetCore::RunoffStatistics &Stats = Planet->GetStatistics();
	TestTrue(TEXT("実惑星の陸上から海へ水が到達"), Stats.ReceivingOceanNodes > 0 && Stats.AnnualOceanRunoffCubicMeters > 0);
	TestTrue(TEXT("全球の経路積算が局所流出と一致"), Stats.RoutingRelativeResidual < 1E-10);
	TestTrue(TEXT("全球の年降水を蒸発散・河川・氷河へ配分"), Stats.WaterBudgetRelativeResidual < 1E-8);
	std::uint32_t NonDecreasing = 0;
	for(std::uint32_t i = 0; i < Drainage->Nodes().size(); ++i) {
		const PlanetCore::DrainageNode &Node = Drainage->Nodes()[i];
		if(Node.Ocean) continue;
		for(std::uint32_t j = 0; j < PlanetCore::ClimatePeriodCount; ++j) {
			TestTrue(TEXT("実河川で各期の下流流量が減少しない"),
				Planet->GetPeriods()[Node.Downstream][j] + 1E-10 >= Planet->GetPeriods()[i][j]);
			++NonDecreasing;
		}
	}
	AddInfo(FString::Printf(
		TEXT("Runoff: analyticError=%.9g junctionError=%.9g land=%u outlets=%u downstream=%u rain=%.9g ocean=%.9g ")
			TEXT("frozen=%.9g routingResidual=%.9g waterResidual=%.9g areaResidual=%.9g flowSeconds=%.6f total=%.6f"),
		MaximumAnalyticError, MaximumJunctionError, Stats.LandNodes, Stats.ReceivingOceanNodes, NonDecreasing,
		Stats.AnnualPrecipitationCubicMeters, Stats.AnnualOceanRunoffCubicMeters, Stats.AnnualFrozenExportCubicMeters,
		Stats.RoutingRelativeResidual, Stats.WaterBudgetRelativeResidual, Stats.AreaRelativeResidual, FlowSeconds,
		FPlatformTime::Seconds() - Start));
	// 検査実行結果の返戻
	return true;
}
#endif
