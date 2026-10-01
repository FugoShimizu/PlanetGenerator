#include "Core/PlanetAdaptiveDrainageTopology.h"
#include "Core/PlanetDrainage.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetMinimalDrainageMinimax.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetSphericalDrainageBaseline.h"
#include "Core/PlanetSphericalMesh.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetAdaptiveDrainageContracts, "PlanetGenerator.AdaptiveDrainage.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 閉球面の乾いた窪地，二つの球半径，拒否と決定論性を検査する
 * @param Parameters 未使用の UE 自動検査引数
 * @return 全契約を満たす場合に真
 */
bool FPlanetAdaptiveDrainageContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(3);
	// 閉球面網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較用の閉球面を準備"), Mesh.has_value())) return false;
	std::vector<double> Original;
	Original.reserve(Mesh->Directions.size());
	std::uint32_t Pit = 0;
	double HighestZ = -2;
	for(std::uint32_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		const PlanetCore::Vec3 Point = Mesh->Directions[Index].Vector();
		Original.push_back(Point.Z < -0.25 ? -30 + 10 * Point.Z : 100 + 20 * Point.X + 11 * Point.Y + 7 * Point.Z);
		if(Point.Z <= HighestZ) continue;
		HighestZ = Point.Z;
		Pit = Index;
	}
	Original[Pit] = 1;
	for(const double Radius : { 1E3, 1E6 }) {
		PlanetCore::MinimalDrainageFillSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.MinimumLandSlope = 1E-5;
		PlanetCore::MinimalDrainageFillError FillError = PlanetCore::MinimalDrainageFillError::None;
		const std::optional<PlanetCore::MinimalDrainageFill> Fill =
		PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, FillError);
		// 同条件の基準盛土を準備出来ない場合の返戻
		if(!TestTrue(TEXT("人工球面の基準盛土を準備"), Fill.has_value())) return false;
		PlanetCore::AdaptiveDrainageError Error = PlanetCore::AdaptiveDrainageError::None;
		const std::optional<PlanetCore::AdaptiveDrainageTopology> Adaptive =
		PlanetCore::AdaptiveDrainageTopology::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, Error);
		if(!TestTrue(TEXT("人工球面の流路木を選び直す"), Adaptive.has_value())) {
			AddError(FString::Printf(TEXT("Adaptive synthetic radius=%.9gm reason=%u."), Radius, static_cast<unsigned>(Error)));
			// 結果の無い条件を受け入れない返戻
			return false;
		}
		TestTrue(
			TEXT("変更幅は同じ球面の基準盛土以下"),
			Adaptive->GetStatistics().MaximumAbsoluteChangeMeters <= Fill->GetStatistics().MaximumRaiseMeters + 1E-8
		);
		TestTrue(
			TEXT("二分探索の幅と実際の変更幅が一致"),
			std::abs(Adaptive->GetStatistics().MinimumFeasibleBoundMeters - Adaptive->GetStatistics().MaximumAbsoluteChangeMeters) < 1E-6
		);
		std::vector<std::array<std::uint32_t, 2>> Edges;
		for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			Edges.push_back({ std::min(A, B), std::max(A, B) });
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		for(std::uint32_t Index = 0; Index < Original.size(); ++Index) {
			if(Original[Index] < Settings.SeaLevelMeters) {
				TestEqual(TEXT("元の海底を保存"), Adaptive->GetHeightsMeters()[Index], Original[Index]);
				continue;
			}
			const std::uint32_t Parent = Adaptive->GetDownstreamVertices()[Index];
			// 各陸上節点の親が共有辺上に無い場合の返戻
			if(!TestTrue(TEXT("親辺は閉球面の共有辺"), Parent < Original.size() && Parent != Index)) return false;
			const std::array<std::uint32_t, 2> Edge { std::min(Index, Parent), std::max(Index, Parent) };
			TestTrue(TEXT("選択した親辺は球面三角網に存在"), std::binary_search(Edges.begin(), Edges.end(), Edge));
			const double ParentHeight =
			Original[Parent] < Settings.SeaLevelMeters ? Settings.SeaLevelMeters : Adaptive->GetHeightsMeters()[Parent];
			const PlanetCore::Vec3 Difference = Mesh->Directions[Index].Vector() - Mesh->Directions[Parent].Vector();
			const double Arc = 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
			TestTrue(
				TEXT("親辺の正勾配と厳密下降を保つ"),
				Adaptive->GetHeightsMeters()[Index] > ParentHeight &&
				Adaptive->GetHeightsMeters()[Index] - ParentHeight + 1E-9 >= Settings.MinimumLandSlope * Radius * Arc
			);
		}
		PlanetCore::AdaptiveDrainageError RepeatedError = PlanetCore::AdaptiveDrainageError::None;
		const std::optional<PlanetCore::AdaptiveDrainageTopology> Repeated =
		PlanetCore::AdaptiveDrainageTopology::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, RepeatedError);
		// 同一定義の独立再準備が失敗した場合の返戻
		if(!TestTrue(TEXT("同じ人工球面を再準備"), Repeated.has_value())) return false;
		for(std::uint32_t Index = 0; Index < Original.size(); ++Index) {
			TestTrue(
				TEXT("同一環境の節点標高と親がビット一致"),
				std::bit_cast<std::uint64_t>(Adaptive->GetHeightsMeters()[Index]) ==
				std::bit_cast<std::uint64_t>(Repeated->GetHeightsMeters()[Index]) &&
				Adaptive->GetDownstreamVertices()[Index] == Repeated->GetDownstreamVertices()[Index]
			);
		}
		AddInfo(
			FString::Printf(
				TEXT("Adaptive synthetic: radius=%.9gm vertices=%llu maxChange=%.9gm changedParent=%u runs=%u. "),
				Radius,
				static_cast<unsigned long long>(Original.size()),
				Adaptive->GetStatistics().MaximumAbsoluteChangeMeters,
				Adaptive->GetStatistics().ChangedParentVertices,
				Adaptive->GetStatistics().FeasibilityRuns
			)
		);
	}
	PlanetCore::MinimalDrainageFillSettings Settings;
	PlanetCore::AdaptiveDrainageError Error = PlanetCore::AdaptiveDrainageError::None;
	const std::vector<double> NoOcean(Mesh->Directions.size(), 100);
	TestFalse(
		TEXT("大洋の無い閉球面を拒否"),
		PlanetCore::AdaptiveDrainageTopology::Create(Mesh->Directions, Mesh->Triangles, NoOcean, Settings, Error).has_value()
	);
	TestTrue(TEXT("大洋の無い入力の理由"), Error == PlanetCore::AdaptiveDrainageError::FillRejected);
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(
		TEXT("中断済みの流路木準備を拒否"),
		PlanetCore::AdaptiveDrainageTopology::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, Error, Cancel.get_token())
		.has_value()
	);
	TestTrue(TEXT("中断理由を返す"), Error == PlanetCore::AdaptiveDrainageError::Cancelled);
	// 人工球面の各条件を返す
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetAdaptiveDrainageProductionSurvey, "PlanetGenerator.AdaptiveDrainage.ProductionSurvey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 侵食後の通常地形で固定木と親辺再選択の最大補正幅を比較する
 * @param Parameters 未使用の UE 自動検査引数
 * @return 両候補と独立な全球排水検査が成立する場合に真
 */
bool FPlanetAdaptiveDrainageProductionSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 全球の共有網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の閉球面を準備"), Mesh.has_value())) return false;
	PlanetCore::GeologySettings GeologySettings;
	GeologySettings.Seed = 7;
	GeologySettings.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(GeologySettings);
	// 地質場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の地質を準備"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 大洋場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の大洋を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 通常の排水網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の排水網を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
	const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	// 通常の侵食後 H を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の侵食後標高を準備"), Field.has_value())) return false;
	std::vector<double> Original;
	Original.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) Original.push_back(Field->EvaluateElevation(Direction));
	PlanetCore::MinimalDrainageFillSettings Settings;
	Settings.RadiusMeters = GeologySettings.RadiusMeters;
	Settings.SeaLevelMeters = Field->Drainage().Settings().SeaLevelMeters;
	Settings.MinimumLandSlope = 1E-6;
	PlanetCore::MinimalDrainageMinimaxError FixedError = PlanetCore::MinimalDrainageMinimaxError::None;
	const std::optional<PlanetCore::MinimalDrainageMinimax> Fixed =
	PlanetCore::MinimalDrainageMinimax::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, FixedError);
	if(!TestTrue(TEXT("固定排水木の最小最大補正を準備"), Fixed.has_value())) {
		AddError(FString::Printf(TEXT("Fixed drainage minimax reason=%u."), static_cast<unsigned>(FixedError)));
		// 比較対象が無い条件を成功としない返戻
		return false;
	}
	PlanetCore::AdaptiveDrainageError AdaptiveError = PlanetCore::AdaptiveDrainageError::None;
	const std::optional<PlanetCore::AdaptiveDrainageTopology> Adaptive =
	PlanetCore::AdaptiveDrainageTopology::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, AdaptiveError);
	if(!TestTrue(TEXT("実地形の流路木と標高を共に選ぶ"), Adaptive.has_value())) {
		AddError(FString::Printf(TEXT("Adaptive production reason=%u."), static_cast<unsigned>(AdaptiveError)));
		// 全球解が無い条件を成功としない返戻
		return false;
	}
	TestTrue(
		TEXT("親辺再選択の最適幅は固定木の最適幅以下"),
		Adaptive->GetStatistics().MaximumAbsoluteChangeMeters <= Fixed->GetStatistics().MaximumAbsoluteChangeMeters + 1E-6
	);
	TestTrue(
		TEXT("探索幅と実際の最大補正幅が一致"),
		std::abs(Adaptive->GetStatistics().MinimumFeasibleBoundMeters - Adaptive->GetStatistics().MaximumAbsoluteChangeMeters) < 1E-6
	);
	PlanetCore::SphericalDrainageBaselineError BaselineError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		Adaptive->GetHeightsMeters(),
		Settings.SeaLevelMeters,
		BaselineError
	);
	TestTrue(TEXT("流路木再選択後の全陸節点が大洋へ到達"), Baseline.has_value());
	AddInfo(
		FString::Printf(
			TEXT("Adaptive production: seed=7 radius=1000000m vertices=%llu fixedMax=%.9gm ")
			TEXT("adaptiveBound=%.9gm adaptiveMax=%.9gm maxRaise=%.9gm maxCut=%.9gm ")
			TEXT("sumAbsolute=%.9gm changed=%u changedParent=%u runs=%u baseline=%u reason=%u ")
			TEXT("seconds=%.6f. Final continuous landscape unaccepted."),
			static_cast<unsigned long long>(Original.size()),
			Fixed->GetStatistics().MaximumAbsoluteChangeMeters,
			Adaptive->GetStatistics().MinimumFeasibleBoundMeters,
			Adaptive->GetStatistics().MaximumAbsoluteChangeMeters,
			Adaptive->GetStatistics().MaximumRaiseMeters,
			Adaptive->GetStatistics().MaximumCutMeters,
			Adaptive->GetStatistics().SumAbsoluteChangeMeters,
			Adaptive->GetStatistics().ChangedLandVertices,
			Adaptive->GetStatistics().ChangedParentVertices,
			Adaptive->GetStatistics().FeasibilityRuns,
			static_cast<unsigned>(Baseline.has_value()),
			static_cast<unsigned>(BaselineError),
			FPlatformTime::Seconds() - Started
		)
	);
	// 実地形での固定木と流路再選択の比較結果を返す
	return !HasAnyErrors();
}

#endif
