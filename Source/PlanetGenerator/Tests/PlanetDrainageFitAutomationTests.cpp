#include "Core/PlanetDrainage.h"
#include "Core/PlanetDrainageFit.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetMinimalDrainageFill.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetSphericalDrainageBaseline.h"
#include "Core/PlanetSphericalMesh.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <numeric>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 木の全ての等値辺集合を列挙し，独立な最小二乗費用を求める
	 * 計算量：O(2ⁿ n²)，n ≤５の検査専用
	 * @param Sites 下流番号が自分より小さい木
	 * @param Lower 全地点に共通の下限
	 * @return 全結合分割の中で実行可能な最小費用
	 */
	double ExhaustiveTreeCost(const std::vector<PlanetCore::DrainageFitSite> &Sites, const double Lower) {
		const std::uint32_t Count = static_cast<std::uint32_t>(Sites.size());
		double Best = std::numeric_limits<double>::infinity();
		for(std::uint32_t Mask = 0; Mask < (1U << (Count - 1)); ++Mask) {
			std::array<std::uint32_t, 5> Groups;
			std::iota(Groups.begin(), Groups.end(), 0U);
			const auto Root = [&Groups](std::uint32_t Index) {
				while(Groups[Index] != Index) Index = Groups[Index];
				// 列挙した等値制約成分の代表番号の返戻
				return Index;
			};
			for(std::uint32_t Index = 1; Index < Count; ++Index) {
				if(Mask & (1U << (Index - 1))) Groups[Root(Index)] = Root(Sites[Index].Downstream);
			}
			std::array<double, 5> Moments {}, Weights {}, Values {};
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				const std::uint32_t Group = Root(Index);
				Moments[Group] += Sites[Index].TargetMeters * Sites[Index].Weight;
				Weights[Group] += Sites[Index].Weight;
			}
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				const std::uint32_t Group = Root(Index);
				Values[Index] = std::max(Lower, Moments[Group] / Weights[Group]);
			}
			bool Feasible = true;
			double Cost = 0;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				if(Index) Feasible &= Values[Index] >= Values[Sites[Index].Downstream];
				const double Change = Values[Index] - Sites[Index].TargetMeters;
				Cost += Sites[Index].Weight * Change * Change;
			}
			if(Feasible) Best = std::min(Best, Cost);
		}
		// 各辺の結合有無を全列挙した最小費用の返戻
		return Best;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetDrainageFitContracts, "PlanetGenerator.DrainageFit.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 排水木の共有水位を全分割列挙，拒否条件，大規模な木で検査する
 * @param Parameters 未使用の UE 自動検査引数
 * @return 全条件を満たす場合に真
 */
bool FPlanetDrainageFitContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::DrainageFitSite;
	using PlanetCore::FitDrainageTree;
	constexpr std::uint32_t Sea = PlanetCore::NoDrainageFitDownstream;
	TestFalse(TEXT("空の木を拒否"), FitDrainageTree({}, 0).has_value());
	const std::array<DrainageFitSite, 2> Cycle { { { 1, 1, 1 }, { 0, 2, 1 } } };
	TestFalse(TEXT("大洋根を持たない循環を拒否"), FitDrainageTree(Cycle, 0).has_value());
	const std::array<DrainageFitSite, 1> InvalidWeight { { { Sea, 1, 0 } } };
	TestFalse(TEXT("非正の費用重みを拒否"), FitDrainageTree(InvalidWeight, 0).has_value());
	const std::array<DrainageFitSite, 1> InvalidParent { { { 5, 1, 1 } } };
	TestFalse(TEXT("範囲外の下流参照を拒否"), FitDrainageTree(InvalidParent, 0).has_value());
	const std::array<DrainageFitSite, 3> Branch { { { Sea, 10, 1 }, { 0, 0, 1 }, { 0, 100, 1 } } };
	const std::optional<PlanetCore::DrainageFitResult> Branched = FitDrainageTree(Branch, 0);
	// 基本の合流解を得られない場合の返戻
	if(!TestTrue(TEXT("合流の共有水位を準備"), Branched.has_value())) return false;
	TestTrue(TEXT("低い支流だけを統合し，高い支流を不必要に平坦化しない"), Branched->Levels == std::vector<double>{ 5, 5, 100 });
	const std::array<DrainageFitSite, 3> Permuted { { { 1, 100, 1 }, { Sea, 10, 1 }, { 1, 0, 1 } } };
	const std::optional<PlanetCore::DrainageFitResult> Reordered = FitDrainageTree(Permuted, 0);
	TestTrue(TEXT("下流番号が自分より大きい通常入力も受理"), Reordered && Reordered->Levels == std::vector<double>{ 100, 5, 5 });
	const std::array<PlanetCore::DrainageTerrainSite, 3> TerrainBranch {
		{ { Sea, 100, 10, 1 }, { 0, 200, 0, 1 }, { 0, 300, 100, 1 } }
	};
	const std::optional<PlanetCore::DrainageFitResult> Terrain = PlanetCore::FitDrainageTerrain(TerrainBranch, 0, 0.01);
	TestTrue(TEXT("実距離の正勾配を保つ合流標高を準備"), Terrain.has_value());
	if(Terrain) {
		TestTrue(
			TEXT("陸上根と支流が実長の落差で大洋へ下降"),
			std::abs(Terrain->Levels[0] - 4.5) < 1E-12 && std::abs(Terrain->Levels[1] - 5.5) < 1E-12 &&
			std::abs(Terrain->Levels[2] - 100) < 1E-12
		);
		TestTrue(TEXT("物理標高の重み付き変更量を報告"), std::abs(Terrain->WeightedSquaredChange - 60.5) < 1E-10);
		TestTrue(
			TEXT("同じ合流木で避けられない最大変更量と祖先・子の証拠"),
			std::abs(Terrain->MinimumTheoreticalMaxChangeMeters - 5.5) < 1E-12 && Terrain->CriticalAncestor == 0 &&
			Terrain->CriticalSite == 1
		);
	}
	const std::array<PlanetCore::DrainageTerrainSite, 1> BelowSea { { { Sea, 100, -10, 1 } } };
	const std::optional<PlanetCore::DrainageFitResult> CoastalBound = PlanetCore::FitDrainageTerrain(BelowSea, 0, 0.01);
	TestTrue(
		TEXT("大洋根では固定海面が最大変更量の下限になる"),
		CoastalBound && std::abs(CoastalBound->MinimumTheoreticalMaxChangeMeters - 11) < 1E-12 &&
		CoastalBound->CriticalAncestor == Sea && CoastalBound->CriticalSite == 0
	);
	std::array<PlanetCore::DrainageTerrainSite, 3> WrongDistances = TerrainBranch;
	WrongDistances[1].DistanceToOceanMeters = 100;
	TestFalse(TEXT("下流距離が増えない親辺を拒否"), PlanetCore::FitDrainageTerrain(WrongDistances, 0, 0.01).has_value());
	WrongDistances = TerrainBranch;
	WrongDistances[0].DistanceToOceanMeters = std::numeric_limits<double>::infinity();
	TestFalse(TEXT("非有限な大洋距離を拒否"), PlanetCore::FitDrainageTerrain(WrongDistances, 0, 0.01).has_value());
	TestFalse(TEXT("零勾配を厳密下降と誤認しない"), PlanetCore::FitDrainageTerrain(TerrainBranch, 0, 0).has_value());
	constexpr double RoundedBase = 75.65862465885904, RoundedSlope = 9.284763193061028E-5, RoundedRootDistance = 0.9834822706145525;
	constexpr double RoundedChildDistance = 1.095559090875323;
	const double RoundedRootHeight = RoundedBase + RoundedSlope * RoundedRootDistance;
	const double RoundedChildHeight = RoundedBase + RoundedSlope * RoundedChildDistance;
	const double RequiredRoundedChild = RoundedRootHeight + RoundedSlope * (RoundedChildDistance - RoundedRootDistance);
	TestTrue(TEXT("正の落差だけでは指定勾配を失う丸め反例"), RoundedChildHeight > RoundedRootHeight && RoundedChildHeight < RequiredRoundedChild);
	const std::array<PlanetCore::DrainageTerrainSite, 2> RoundedEdge {
		{ { Sea, RoundedRootDistance, RoundedRootHeight, 1 }, { 0, RoundedChildDistance, RoundedChildHeight, 1 } }
	};
	const std::optional<PlanetCore::DrainageFitResult> RoundedFit = PlanetCore::FitDrainageTerrain(RoundedEdge, 0, RoundedSlope);
	TestTrue(
		TEXT("最小勾配を一 ulp 下回る解だけを上側へ丸める"),
		RoundedFit && RoundedFit->Levels[1] >= RoundedFit->Levels[0] + RoundedSlope * (RoundedChildDistance - RoundedRootDistance) &&
		RoundedFit->Levels[1] > RoundedChildHeight
	);
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("中断済みの準備を拒否"), FitDrainageTree(Branch, 0, Cancel.get_token()).has_value());
	TestFalse(TEXT("中断済みの地形標高準備を拒否"), PlanetCore::FitDrainageTerrain(TerrainBranch, 0, 0.01, Cancel.get_token()).has_value());
	const std::optional<PlanetCore::SphereMesh> TerrainMesh = PlanetCore::BuildUniformSphere(3);
	// 独立な全球検査へ渡す閉球面を用意出来ない場合の返戻
	if(!TestTrue(TEXT("距離付き標高回帰の閉球面を準備"), TerrainMesh.has_value())) return false;
	std::vector<double> OriginalTerrain;
	OriginalTerrain.reserve(TerrainMesh->Directions.size());
	std::uint32_t BasinPit = 0;
	double HighestZ = -2;
	for(std::uint32_t Index = 0; Index < TerrainMesh->Directions.size(); ++Index) {
		const PlanetCore::Vec3 Point = TerrainMesh->Directions[Index].Vector();
		OriginalTerrain.push_back(Point.Z < -0.25 ? -30 + 10 * Point.Z : 100 + 20 * Point.X + 11 * Point.Y + 7 * Point.Z);
		if(Point.Z <= HighestZ) continue;
		HighestZ = Point.Z;
		BasinPit = Index;
	}
	OriginalTerrain[BasinPit] = 1;
	PlanetCore::MinimalDrainageFillSettings TerrainSettings;
	TerrainSettings.MinimumLandSlope = 1E-5;
	PlanetCore::MinimalDrainageFillError FillError = PlanetCore::MinimalDrainageFillError::None;
	const std::optional<PlanetCore::MinimalDrainageFill> FilledTerrain = PlanetCore::MinimalDrainageFill::Create(
		TerrainMesh->Directions,
		TerrainMesh->Triangles,
		OriginalTerrain,
		TerrainSettings,
		FillError
	);
	// 下流木を用意出来ない場合の返戻
	if(!TestTrue(TEXT("乾いた窪地を持つ閉球面の排水木を準備"), FilledTerrain.has_value())) return false;
	std::vector<std::uint32_t> LandOrder;
	for(std::uint32_t Index = 0; Index < OriginalTerrain.size(); ++Index) {
		if(OriginalTerrain[Index] >= TerrainSettings.SeaLevelMeters) LandOrder.push_back(Index);
	}
	std::sort(
		LandOrder.begin(),
		LandOrder.end(),
		[&](const std::uint32_t A, const std::uint32_t B) {
			const double HA = FilledTerrain->GetHeightsMeters()[A], HB = FilledTerrain->GetHeightsMeters()[B];
			return HA != HB ? HA < HB : A < B;
		}
	);
	std::vector<std::uint32_t> LandSite(OriginalTerrain.size(), Sea);
	for(std::uint32_t Index = 0; Index < LandOrder.size(); ++Index) LandSite[LandOrder[Index]] = Index;
	std::vector<double> OceanDistance(OriginalTerrain.size());
	std::vector<PlanetCore::DrainageTerrainSite> TerrainSites(LandOrder.size());
	for(const std::uint32_t Vertex : LandOrder) {
		const std::uint32_t Parent = FilledTerrain->GetDownstreamVertices()[Vertex];
		const PlanetCore::Vec3 A = TerrainMesh->Directions[Vertex].Vector();
		const PlanetCore::Vec3 B = TerrainMesh->Directions[Parent].Vector();
		const double EdgeMeters = std::atan2(A.Cross(B).Length(), A.Dot(B)) * TerrainSettings.RadiusMeters;
		OceanDistance[Vertex] = OceanDistance[Parent] + EdgeMeters;
		TerrainSites[LandSite[Vertex]] = { LandSite[Parent], OceanDistance[Vertex], OriginalTerrain[Vertex], 1 };
	}
	const std::optional<PlanetCore::DrainageFitResult> TerrainFit =
	PlanetCore::FitDrainageTerrain(TerrainSites, TerrainSettings.SeaLevelMeters, TerrainSettings.MinimumLandSlope);
	// 全球の節点標高を得られない場合の返戻
	if(!TestTrue(TEXT("実球面辺長を使う全陸節点の標高回帰"), TerrainFit.has_value())) return false;
	std::vector<double> FittedTerrain = OriginalTerrain;
	double LargestChange = 0;
	for(std::uint32_t Index = 0; Index < LandOrder.size(); ++Index) {
		const std::uint32_t Vertex = LandOrder[Index];
		FittedTerrain[Vertex] = TerrainFit->Levels[Index];
		LargestChange = std::max(LargestChange, std::abs(FittedTerrain[Vertex] - OriginalTerrain[Vertex]));
	}
	PlanetCore::SphericalDrainageBaselineError FittedError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> FittedBaseline = PlanetCore::SphericalDrainageBaseline::Create(
		TerrainMesh->Directions,
		TerrainMesh->Triangles,
		FittedTerrain,
		TerrainSettings.SeaLevelMeters,
		FittedError
	);
	TestTrue(TEXT("距離付き最小二乗の結果を独立な全球排水検査へ通す"), FittedBaseline.has_value());
	AddInfo(
		FString::Printf(
			TEXT("Distance fit sphere: land=%llu cost=%.9gm2 maximumChange=%.9gm baseline=%u reason=%u. Final landscape unaccepted."),
			static_cast<unsigned long long>(LandOrder.size()),
			TerrainFit->WeightedSquaredChange,
			LargestChange,
			static_cast<unsigned>(FittedBaseline.has_value()),
			static_cast<unsigned>(FittedError)
		)
	);
	std::uint32_t Cases = 0;
	for(std::uint32_t Count = 1; Count < 6; ++Count) {
		std::uint32_t Trees = 1, Targets = 1;
		for(std::uint32_t Index = 1; Index < Count; ++Index) Trees *= Index;
		for(std::uint32_t Index = 0; Index < Count; ++Index) Targets *= 3;
		for(std::uint32_t Tree = 0; Tree < Trees; ++Tree) for(std::uint32_t Code = 0; Code < Targets; ++Code) {
			std::vector<DrainageFitSite> Sites(Count);
			std::uint32_t ParentDigits = Tree, HeightDigits = Code;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				Sites[Index].TargetMeters = static_cast<double>(HeightDigits % 3) * 2 - 1;
				Sites[Index].Weight = Index % 3 + 1;
				HeightDigits /= 3;
				if(Index) {
					Sites[Index].Downstream = ParentDigits % Index;
					ParentDigits /= Index;
				}
			}
			const std::optional<PlanetCore::DrainageFitResult> Result = FitDrainageTree(Sites, 0.25);
			// 独立列挙と比較する解を得られない場合の返戻
			if(!TestTrue(TEXT("有限な正重み木を準備"), Result.has_value())) return false;
			TestTrue(TEXT("全等値辺集合の独立最適費用と一致"), std::abs(Result->WeightedSquaredChange - ExhaustiveTreeCost(Sites, 0.25)) <= 1E-10);
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				TestTrue(
					TEXT("全ての合流と海口で下降順序を保持"),
					Result->Levels[Index] >= 0.25 && (!Index || Result->Levels[Index] >= Result->Levels[Sites[Index].Downstream])
				);
			}
			++Cases;
		}
	}
	AddInfo(FString::Printf(TEXT("Drainage fit oracle: %u small weighted trees checked against all edge partitions."), Cases));
	for(const bool Binary : { false, true }) {
		std::vector<DrainageFitSite> Sites(100000);
		for(std::uint32_t Index = 0; Index < Sites.size(); ++Index) {
			if(Index) Sites[Index].Downstream = Binary ? (Index - 1) / 2 : Index - 1;
			Sites[Index].TargetMeters = std::sin(static_cast<double>(Index) * 0.013) * 2000;
			Sites[Index].Weight = Index % 3 + 1;
		}
		const double Started = FPlatformTime::Seconds();
		const std::optional<PlanetCore::DrainageFitResult> Result = FitDrainageTree(Sites, 0);
		const double Milliseconds = (FPlatformTime::Seconds() - Started) * 1000;
		// 大規模木の解を得られない場合の返戻
		if(!TestTrue(TEXT("十万地点の排水木を準備"), Result.has_value())) return false;
		for(std::uint32_t Index = 0; Index < Sites.size(); ++Index) {
			TestTrue(
				TEXT("大規模木の全下流辺で共有順序を保持"),
				Result->Levels[Index] >= 0 && (!Index || Result->Levels[Index] >= Result->Levels[Sites[Index].Downstream])
			);
		}
		AddInfo(
			FString::Printf(
				TEXT("Drainage fit scale: binary=%u nodes=%llu time=%.6fms moves=%llu pooled=%u."),
				Binary ? 1U : 0U,
				static_cast<unsigned long long>(Sites.size()),
				Milliseconds,
				static_cast<unsigned long long>(Result->HeapMoves),
				Result->PooledKnots
			)
		);
	}
	// 独立な最適性と入力拒否，大規模な共有順序の全検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetDrainageFitProductionSurvey, "PlanetGenerator.DrainageFit.ProductionSurvey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 侵食後の通常地形で同一排水木の最小二乗標高を比較し，大洋到達を独立に検査する
 * @param Parameters 未使用の UE 自動検査引数
 * @return 全球網と有限費用の契約を満たす場合に真
 */
bool FPlanetDrainageFitProductionSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	constexpr std::uint32_t Sea = PlanetCore::NoDrainageFitDownstream;
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 比較用の閉球面を構成出来ない場合の返戻
	if(!TestTrue(TEXT("比較用の閉球面を準備"), Mesh.has_value())) return false;
	PlanetCore::GeologySettings GeologySettings;
	GeologySettings.Seed = 7;
	GeologySettings.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(GeologySettings);
	// 通常の地質を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較条件の地質を準備"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 通常の大洋を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較条件の大洋を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 通常の排水網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較条件の排水網を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
	const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	// 通常の侵食後標高場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較条件の侵食後標高場を準備"), Field.has_value())) return false;
	std::vector<double> Original;
	Original.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) Original.push_back(Field->EvaluateElevation(Direction));
	PlanetCore::MinimalDrainageFillSettings Settings;
	Settings.RadiusMeters = GeologySettings.RadiusMeters;
	Settings.SeaLevelMeters = Field->Drainage().Settings().SeaLevelMeters;
	Settings.MinimumLandSlope = 1E-6;
	PlanetCore::MinimalDrainageFillError FillError = PlanetCore::MinimalDrainageFillError::None;
	const std::optional<PlanetCore::MinimalDrainageFill> Filled =
	PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, FillError);
	if(!TestTrue(TEXT("同じ排水木の基準盛土を準備"), Filled.has_value())) {
		AddError(FString::Printf(TEXT("Production drainage fill reason=%u."), static_cast<unsigned>(FillError)));
		// 基準の排水木が無い場合の返戻
		return false;
	}
	std::vector<std::uint32_t> LandOrder;
	for(std::uint32_t Index = 0; Index < Original.size(); ++Index) {
		if(Original[Index] >= Settings.SeaLevelMeters) LandOrder.push_back(Index);
	}
	std::sort(
		LandOrder.begin(),
		LandOrder.end(),
		[&](const std::uint32_t A, const std::uint32_t B) {
			const double HA = Filled->GetHeightsMeters()[A], HB = Filled->GetHeightsMeters()[B];
			return HA != HB ? HA < HB : A < B;
		}
	);
	std::vector<std::uint32_t> LandSite(Original.size(), Sea);
	for(std::uint32_t Index = 0; Index < LandOrder.size(); ++Index) LandSite[LandOrder[Index]] = Index;
	std::vector<double> OceanDistance(Original.size());
	std::vector<PlanetCore::DrainageTerrainSite> Sites(LandOrder.size());
	std::uint32_t UnorderedParents = 0, InvalidDistances = 0;
	for(const std::uint32_t Vertex : LandOrder) {
		const std::uint32_t Parent = Filled->GetDownstreamVertices()[Vertex];
		// 参照先が球面網から外れる場合の返戻
		if(!TestTrue(TEXT("親辺を球面網から参照"), Parent < Original.size() && Parent != Vertex)) return false;
		if(Original[Parent] >= Settings.SeaLevelMeters && !(LandSite[Parent] < LandSite[Vertex])) ++UnorderedParents;
		const PlanetCore::Vec3 A = Mesh->Directions[Vertex].Vector();
		const PlanetCore::Vec3 B = Mesh->Directions[Parent].Vector();
		const double EdgeMeters = std::atan2(A.Cross(B).Length(), A.Dot(B)) * Settings.RadiusMeters;
		OceanDistance[Vertex] = OceanDistance[Parent] + EdgeMeters;
		if(!std::isfinite(OceanDistance[Vertex]) || !(OceanDistance[Vertex] > OceanDistance[Parent])) ++InvalidDistances;
		Sites[LandSite[Vertex]] = { LandSite[Parent], OceanDistance[Vertex], Original[Vertex], 1 };
	}
	TestEqual(TEXT("基準木は下流から順に並ぶ"), UnorderedParents, 0U);
	TestEqual(TEXT("全球の親辺は海まで正の実長を持つ"), InvalidDistances, 0U);
	// 距離順の入力が成立しない場合の返戻
	if(HasAnyErrors()) return false;
	const std::optional<PlanetCore::DrainageFitResult> Fit =
	PlanetCore::FitDrainageTerrain(Sites, Settings.SeaLevelMeters, Settings.MinimumLandSlope);
	if(!TestTrue(TEXT("侵食後地形の全陸標高を回帰"), Fit.has_value())) {
		AddError(FString::Printf(TEXT("Production drainage fit land=%llu."), static_cast<unsigned long long>(LandOrder.size())));
		// 全球の陸標高を回帰出来ない場合の返戻
		return false;
	}
	std::vector<double> Fitted = Original;
	double FillCost = 0, MaximumRaise = 0, MaximumCut = 0, SumAbsolute = 0;
	std::uint32_t ChangedAboveCentimeter = 0;
	for(const std::uint32_t Vertex : LandOrder) {
		const double Change = Fit->Levels[LandSite[Vertex]] - Original[Vertex];
		const double FillChange = Filled->GetHeightsMeters()[Vertex] - Original[Vertex];
		Fitted[Vertex] = Fit->Levels[LandSite[Vertex]];
		FillCost += FillChange * FillChange;
		MaximumRaise = std::max(MaximumRaise, Change);
		MaximumCut = std::max(MaximumCut, -Change);
		SumAbsolute += std::abs(Change);
		if(std::abs(Change) > 0.01) ++ChangedAboveCentimeter;
	}
	TestTrue(TEXT("同じ木で基準盛土以下の最小二乗費用"), Fit->WeightedSquaredChange <= FillCost + 1E-6 * std::max(1.0, FillCost));
	TestTrue(TEXT("実地形の変更量は同じ排水木の理論的下限以上"), std::max(MaximumRaise, MaximumCut) + 1E-6 >= Fit->MinimumTheoreticalMaxChangeMeters);
	PlanetCore::SphericalDrainageBaselineError BaselineError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		Fitted,
		Settings.SeaLevelMeters,
		BaselineError
	);
	TestTrue(TEXT("侵食後の回帰標高から全陸が大洋へ到達"), Baseline.has_value());
	AddInfo(
		FString::Printf(
			TEXT("Production distance fit: seed=7 radius=1000000m vertices=%llu land=%llu changed=%u ")
			TEXT("L2=%.9gm2 fillL2=%.9gm2 maxRaise=%.9gm maxCut=%.9gm sumAbsolute=%.9gm ")
			TEXT("fillMaxRaise=%.9gm theoreticalBound=%.9gm criticalAncestor=%u criticalSite=%u ")
			TEXT("baseline=%u reason=%u preparation=%.6fs. Final landscape unaccepted."),
			static_cast<unsigned long long>(Original.size()),
			static_cast<unsigned long long>(LandOrder.size()),
			ChangedAboveCentimeter,
			Fit->WeightedSquaredChange,
			FillCost,
			MaximumRaise,
			MaximumCut,
			SumAbsolute,
			Filled->GetStatistics().MaximumRaiseMeters,
			Fit->MinimumTheoreticalMaxChangeMeters,
			static_cast<unsigned>(Fit->CriticalAncestor),
			static_cast<unsigned>(Fit->CriticalSite),
			static_cast<unsigned>(Baseline.has_value()),
			static_cast<unsigned>(BaselineError),
			FPlatformTime::Seconds() - Started
		)
	);
	// 全球の独立排水と比較費用の結果の返戻
	return !HasAnyErrors();
}

#endif
