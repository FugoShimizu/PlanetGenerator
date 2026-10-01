#include "Core/PlanetRiverTerrain.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSubmergedRiverRoutes, "PlanetGenerator.RiverWater.SubmergedCenterRoutes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 新しく海面下に入った河道中心から大洋迄，下流側の中心線が水没し続ける事を標本で検査する
 * @param Parameters 未使用の検査引数
 * @return 標本化した全経路で乾いた中断と下流欠落が無い場合に真
 */
bool FPlanetSubmergedRiverRoutes::RunTest(const FString &Parameters) {
	(void)Parameters;
	const auto VerifyCase = [this](std::uint64_t Seed, double RadiusMeters) -> bool {
		PlanetCore::GeologySettings Settings;
		Settings.Seed = Seed;
		Settings.RadiusMeters = RadiusMeters;
		std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
		if(!TestTrue(TEXT("河道接続用の地質を準備"), Geology.has_value())) {
			// 前提条件を満たさないため検査を中断する．
			return false;
		}
		std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
			PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
		if(!TestTrue(TEXT("河道接続用の大洋を準備"), PreparedOcean.has_value())) {
			// 前提条件を満たさないため検査を中断する．
			return false;
		}
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
			std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
		std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
		if(!TestTrue(TEXT("河道接続用の排水を準備"), PreparedDrainage.has_value())) {
			// 前提条件を満たさないため検査を中断する．
			return false;
		}
		const std::optional<PlanetCore::RiverTerrainField> Terrain = PlanetCore::RiverTerrainField::CreateWithErosion(
			Ocean, std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage)));
		if(!TestTrue(TEXT("最終河道地形を準備"), Terrain.has_value())) {
			// 前提条件を満たさないため検査を中断する．
			return false;
		}
		const PlanetCore::DrainageNetwork &Network = Terrain->Drainage();
		const std::span<const PlanetCore::RiverReach> Reaches = Network.Reaches();
		const std::span<const PlanetCore::DrainageNode> Nodes = Network.Nodes();
		constexpr std::uint32_t Segments = PlanetCore::RiverLongitudinalProfile::Segments;
		constexpr std::uint32_t Missing = std::numeric_limits<std::uint32_t>::max();
		const std::uint32_t Count = static_cast<std::uint32_t>(Reaches.size());
		if(!TestTrue(TEXT("選択河道が存在"), Count > 0)) {
			// 前提条件を満たさないため検査を中断する．
			return false;
		}
		std::vector<std::uint32_t> ReachFromNode(Nodes.size(), Missing);
		std::vector<std::array<double, Segments + 1>> Heights(Count);
		std::vector<std::uint32_t> FirstNewWet(Count, Segments + 1);
		std::uint64_t NewWetCenters = 0;
		for(std::uint32_t i = 0; i < Count; ++i) {
			const PlanetCore::RiverReach &Reach = Reaches[i];
			if(Reach.Upstream >= ReachFromNode.size() || Reach.Downstream >= Nodes.size() ||
			   ReachFromNode[Reach.Upstream] != Missing) {
				AddError(TEXT("選択河道の節点対応が不正"));
				// 検査を続行出来ないため中断する．
				return false;
			}
			ReachFromNode[Reach.Upstream] = i;
			for(std::uint32_t k = 0; k <= Segments; ++k) {
				const std::optional<PlanetCore::RiverProfileSample> Sample = Network.SampleReach(i, k / static_cast<double>(Segments));
				const std::optional<PlanetCore::UnitDirection> Direction =
					Sample ? PlanetCore::UnitDirection::FromVector(Sample->Direction) : std::nullopt;
				if(!Direction) {
					AddError(TEXT("河道中心の方向を評価出来ない"));
					// 検査を続行出来ないため中断する．
					return false;
				}
				const double Original = Ocean->EvaluateElevation(*Direction);
				const double Final = Terrain->EvaluateElevation(*Direction);
				Heights[i][k] = Final;
				if(Original >= Settings.SeaLevelMeters && Final < Settings.SeaLevelMeters) {
					FirstNewWet[i] = std::min(FirstNewWet[i], k);
					++NewWetCenters;
				}
			}
		}
		std::uint64_t CheckedCenters = 0;
		std::uint32_t SubmergedRoutes = 0, DryInterruptions = 0, MissingDownstream = 0;
		for(std::uint32_t i = 0; i < Count; ++i) {
			if(FirstNewWet[i] > Segments) continue;
			++SubmergedRoutes;
			std::uint32_t Current = i, Start = FirstNewWet[i];
			bool ReachedOcean = false;
			for(std::uint32_t Hop = 0; Hop < Count; ++Hop) {
				for(std::uint32_t k = Start; k <= Segments; ++k) {
					++CheckedCenters;
					if(Heights[Current][k] <= Settings.SeaLevelMeters + 0.01) continue;
					if(DryInterruptions == 0)
						AddInfo(FString::Printf(TEXT("Dry downstream center: start=%u reach=%u sample=%u H=%.9g"),
							i, Current, k, Heights[Current][k]));
					++DryInterruptions;
				}
				const std::uint32_t Downstream = Reaches[Current].Downstream;
				if(Nodes[Downstream].Ocean) {
					ReachedOcean = true;
					break;
				}
				if(ReachFromNode[Downstream] == Missing) break;
				Current = ReachFromNode[Downstream];
				Start = 0;
			}
			if(!ReachedOcean) ++MissingDownstream;
		}
		AddInfo(FString::Printf(TEXT("Submerged center routes: seed=%llu radius=%.0f reaches=%u newWetCenters=%llu routes=%u ")
			TEXT("checkedCenters=%llu dryInterruptions=%u missingDownstream=%u"),
			static_cast<unsigned long long>(Seed), RadiusMeters, Count,
			static_cast<unsigned long long>(NewWetCenters), SubmergedRoutes,
			static_cast<unsigned long long>(CheckedCenters), DryInterruptions, MissingDownstream));
		TestTrue(TEXT("新たに水没した河道中心が存在"), NewWetCenters > 0);
		TestEqual(TEXT("標本化した下流中心線に乾いた中断が無い"), DryInterruptions, 0u);
		TestEqual(TEXT("水没した河道中心は大洋へ到達"), MissingDownstream, 0u);
		// 有限個の中心試料の結果の返戻
		return !HasAnyErrors();
	};
	for(std::uint64_t Seed : {7ull, 42ull, 2026ull}) {
		for(double RadiusMeters : {1000.0, 1000000.0, 6371000.0}) {
			if(!VerifyCase(Seed, RadiusMeters)) {
				// 検査に失敗した条件で中断する．
				return false;
			}
		}
	}

	// 有限個の中心試料の結果の返戻
	return !HasAnyErrors();
}

#endif
