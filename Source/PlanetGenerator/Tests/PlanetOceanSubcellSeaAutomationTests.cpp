#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetOceanSubcellSeaConnectivity,
	"PlanetGenerator.Ocean.SubcellSeaConnectivity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 旧実行で見落とした海面以下の窪地から，同じ既知大洋へ実水没通路を検査する
 * @param Parameters 未使用の引数
 * @return 元の失敗地点を海域準備に含め，独立な元場標本で大洋へ接続した場合に真
 */
bool FPlanetOceanSubcellSeaConnectivity::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 0;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Prepared = PlanetCore::GeologyField::Create(Definition);
	// 元の惑星定義を準備出来ない場合の返戻
	if(!TestTrue(TEXT("元の地質の準備"), Prepared.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Prepared));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 不正な造山場の返戻
	if(!TestTrue(TEXT("同じ造山場の準備"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoOnly,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 海域準備が失敗した場合の返戻
	if(!TestTrue(TEXT("内部水没点を含む海域の準備"), Ocean.has_value())) return false;
	const PlanetCore::OceanConnectivityStatistics &Stats = Ocean->Statistics();
	AddInfo(
		FString::Printf(
			TEXT("Subcell ocean input=%u graph=%u added=%u candidates=%u source=%llu before=%u after=%u channels=%u area=%.17g"),
			Stats.InputGraphNodes,
			Stats.GraphNodes,
			Stats.SubcellOceanNodes,
			Stats.SubcellCriticalCandidates,
			static_cast<unsigned long long>(Stats.SubcellEvaluations),
			Stats.ComponentsBefore,
			Stats.ComponentsAfter,
			Stats.CorridorCount,
			Stats.IndependentOceanFractionEstimate
		)
	);
	TestEqual(TEXT("元の入力網の地点数を維持"), Stats.InputGraphNodes, 40962U);
	TestTrue(TEXT("実水没点を元の網の内部へ追加"), Stats.SubcellOceanNodes > 0 && Stats.GraphNodes > Stats.InputGraphNodes);
	TestEqual(TEXT("内部水没点を含む実接続"), Stats.ComponentsAfter, 1U);
	TestTrue(TEXT("海域面積が６割から７割"), Stats.IndependentOceanFractionEstimate >= 0.60 && Stats.IndependentOceanFractionEstimate <= 0.70);
	const PlanetCore::UnitDirection Terminal =
	*PlanetCore::UnitDirection::FromVector({ -0.28934040550228901, 0.10658862306316474, 0.95127335460280271 });
	const PlanetCore::UnitDirection MainOcean =
	*PlanetCore::UnitDirection::FromVector({ -0.31583105142581563, 0.11254306437113291, 0.94211719314383535 });
	const double Original = Geology->EvaluateElevation(Terminal), Corrected = Ocean->EvaluateElevation(Terminal);
	TestTrue(TEXT("元の見落とした終点は海面以下"), Original < 0);
	TestTrue(TEXT("同じ失敗地点も接続後に海面以下"), Corrected < 0);
	std::uint64_t Calls = 0;
	bool Exhausted = false;
	const auto Source = [&](const PlanetCore::UnitDirection D) {
		// 同じ実通路の全照会上限を越えた場合の非有限値の返戻
		if(!(Calls < 4000000)) {
			Exhausted = true;
			// 予算不足を水没地形に変換しない返戻
			return PlanetCore::TerrainSample{ std::numeric_limits<double>::quiet_NaN(), {} };
		}
		++Calls;
		// 同じ接続済みの元場の返戻
		return Ocean->Evaluate(D);
	};
	PlanetCore::SpillPassageSettings Settings;
	Settings.HalfWidthMeters = 16000;
	Settings.MaximumSampleStepMeters = 10;
	Settings.AdaptCandidateSpacingToBudget = true;
	Settings.UseEndpointNeighborRing = true;
	const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(Terminal, MainOcean, Source, Settings);
	// 距離又は単なる負の終点を接続証拠として代用しない返戻
	if(!TestTrue(TEXT("元の失敗地点と同じ大洋間の実通路"), Passage.has_value())) return false;
	const double Crest = Passage->CrestMeters + Passage->MaximumIndependentExcessMeters;
	AddInfo(
		FString::Printf(
			TEXT("Subcell source oldH=%.17g newH=%.17g measuredCrest=%.17g nodes=%u calls=%llu"),
			Original,
			Corrected,
			Crest,
			Passage->Nodes,
			static_cast<unsigned long long>(Calls)
		)
	);
	TestTrue(TEXT("同じ元場の実通路が一定海面以下"), std::isfinite(Crest) && !(Crest > 0));
	std::uint64_t IndependentSamples = 0;
	for(std::size_t I = 1; I < Passage->Path.size(); ++I) {
		const PlanetCore::Vec3 A = Passage->Path[I - 1].Vector(), B = Passage->Path[I].Vector();
		const double Distance = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
		const unsigned Steps = std::max(1U, static_cast<unsigned>(std::ceil(Distance / 7)));
		for(unsigned J = 0; J < Steps + 1; ++J) {
			const double T = static_cast<double>(J) / Steps;
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(A * (1 - T) + B * T);
			const double Height = Ocean->EvaluateElevation(D);
			// ７ m 以下の独立位置で陸越えを確認した場合の返戻
			if(!TestTrue(TEXT("独立な水没通路の標高"), std::isfinite(Height) && !(Height > 0))) return false;
			++IndependentSamples;
		}
	}
	AddInfo(
		FString::Printf(TEXT("Subcell independent ocean passage samples=%llu"), static_cast<unsigned long long>(IndependentSamples))
	);
	TestFalse(TEXT("通路の全体予算内"), Exhausted);
	// 記録した失敗地点の実水没接続の返戻；全 Seed の連続保証は別の受入条件
	return !HasAnyErrors();
}

#endif
