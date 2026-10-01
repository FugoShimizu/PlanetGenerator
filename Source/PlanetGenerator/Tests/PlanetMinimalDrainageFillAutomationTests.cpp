#include "Core/PlanetBasinReconstruction.h"
#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetDescentPath.h"
#include "Core/PlanetDrainage.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetMinimalDrainageBlend.h"
#include "Core/PlanetMinimalDrainageFill.h"
#include "Core/PlanetMinimalDrainageMinimax.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetRiverReceiver.h"
#include "Core/PlanetRiverReceiverProfile.h"
#include "Core/PlanetRiverRouting.h"
#include "Core/PlanetRiverRoutingTerrain.h"
#include "Core/PlanetSphericalDrainageBaseline.h"
#include "Core/PlanetSphericalMesh.h"
#include "Core/PlanetSphericalSpline.h"
#include "Core/PlanetSpillDiscovery.h"
#include "Core/PlanetSpillNetwork.h"
#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <queue>
#include <span>
#include <stop_token>
#include <tuple>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetMinimalDrainageFillContracts,
    "PlanetGenerator.MinimalDrainageFill.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 解析的な閉球面に乾いた窪地を入れ，地質保存と全球の下流を検査する
 * @param Parameters 未使用の UE 引数
 * @return 有限網の最小盛土・拒否条件の結果
 */
bool FPlanetMinimalDrainageFillContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::MinimalDrainageFill;
	using PlanetCore::MinimalDrainageFillError;
	using PlanetCore::MinimalDrainageFillSettings;
	using PlanetCore::SphericalDrainageBaseline;
	using PlanetCore::SphericalDrainageBaselineError;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(3);
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("閉球面を準備"), Mesh.has_value())) return false;
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
	MinimalDrainageFillSettings Settings;
	Settings.MinimumLandSlope = 1E-5;
	MinimalDrainageFillError Error = MinimalDrainageFillError::None;
	const std::optional<MinimalDrainageFill> Filled =
	MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, Error);
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("乾いた窪地の条件付け"), Filled.has_value())) return false;
	TestTrue(TEXT("窪地を実際に変更"), Filled->GetHeightsMeters()[Pit] > Original[Pit]);
	PlanetCore::MinimalDrainageMinimaxError MinimaxError = PlanetCore::MinimalDrainageMinimaxError::None;
	const std::optional<PlanetCore::MinimalDrainageMinimax> Minimax =
	PlanetCore::MinimalDrainageMinimax::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, MinimaxError);
	if(!TestTrue(TEXT("人工窪地の最大補正最小化を準備"), Minimax.has_value())) {
		AddError(FString::Printf(TEXT("Minimax synthetic reason=%u."), static_cast<unsigned>(MinimaxError)));
		// 最小最大補正の失敗を無視しない
		return false;
	}
	TestTrue(
		TEXT("人工窪地の最大補正が全盛土より小さい"),
		Minimax->GetStatistics().MaximumAbsoluteChangeMeters < Filled->GetStatistics().MaximumRaiseMeters
	);
	for(std::uint32_t Index = 0; Index < Original.size(); ++Index) {
		if(Original[Index] < Settings.SeaLevelMeters) {
			TestEqual(TEXT("人工窪地で元の海底を保持"), Minimax->GetHeightsMeters()[Index], Original[Index]);
			continue;
		}
		TestTrue(TEXT("人工窪地で元の陸地を保持"), Minimax->GetHeightsMeters()[Index] >= Settings.SeaLevelMeters);
		TestTrue(
			TEXT("人工窪地で最大補正幅を保持"),
			std::abs(Minimax->GetHeightsMeters()[Index] - Original[Index]) <= Minimax->GetStatistics().MaximumAbsoluteChangeMeters + 1E-8
		);
	}
	AddInfo(
		FString::Printf(
			TEXT("Synthetic minimax: fillMax=%.9gm maxCorrection=%.9gm maxRaise=%.9gm maxCut=%.9gm."),
			Filled->GetStatistics().MaximumRaiseMeters,
			Minimax->GetStatistics().MaximumAbsoluteChangeMeters,
			Minimax->GetStatistics().MaximumRaiseMeters,
			Minimax->GetStatistics().MaximumCutMeters
		)
	);
	SphericalDrainageBaselineError BaselineError = SphericalDrainageBaselineError::None;
	const std::optional<SphericalDrainageBaseline> Baseline = SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		Filled->GetHeightsMeters(),
		Settings.SeaLevelMeters,
		BaselineError
	);
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("全陸から一つの大洋へ下る区分線形基準面"), Baseline.has_value())) return false;
	for(const double FillWeight : { 0.5, 1.0 }) {
		PlanetCore::MinimalDrainageBlendError BlendError = PlanetCore::MinimalDrainageBlendError::None;
		const std::optional<PlanetCore::MinimalDrainageBlend> Blend =
		PlanetCore::MinimalDrainageBlend::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, FillWeight, BlendError);
		if(!TestTrue(TEXT("同じ全球木の盛土・切下げ混合"), Blend.has_value())) {
			AddError(FString::Printf(TEXT("Synthetic blend weight=%.9g reason=%u."), FillWeight, static_cast<unsigned>(BlendError)));
			// 失敗した比率を無視しない返戻
			return false;
		}
		TestEqual(TEXT("同じ元大洋数"), Blend->GetStatistics().OceanVertices, Filled->GetStatistics().OceanVertices);
		TestTrue(TEXT("乾いた窪地を調整"), std::abs(Blend->GetHeightsMeters()[Pit] - Original[Pit]) > 0.01);
	}
	PlanetCore::MinimalDrainageBlendError CoastalBlendError = PlanetCore::MinimalDrainageBlendError::None;
	TestFalse(
		TEXT("全量切下げが新しい海域を作る人工窪地を拒否"),
		PlanetCore::MinimalDrainageBlend::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, 0, CoastalBlendError)
		.has_value()
	);
	TestTrue(TEXT("切下げだけの海域変更理由"), CoastalBlendError == PlanetCore::MinimalDrainageBlendError::NewOcean);
	PlanetCore::MinimalDrainageBlendError CoastalBalanceError = PlanetCore::MinimalDrainageBlendError::None;
	const std::optional<PlanetCore::MinimalDrainageBlend> CoastalBalance =
	PlanetCore::MinimalDrainageBlend::CreateWithCoastalBalance(
		Mesh->Directions,
		Mesh->Triangles,
		Original,
		Settings,
		0,
		CoastalBalanceError
	);
	if(!TestTrue(TEXT("人工窪地の海岸勾配を保つ比率"), CoastalBalance.has_value())) {
		AddError(FString::Printf(TEXT("Coastal balance reason=%u."), static_cast<unsigned>(CoastalBalanceError)));
		// 海岸比率の失敗を無視しない返戻
		return false;
	}
	TestTrue(TEXT("必要な盛土比を上げる"), CoastalBalance->GetStatistics().AppliedFillWeight > 0);
	PlanetCore::MinimalDrainageBlendError InvalidBlendError = PlanetCore::MinimalDrainageBlendError::None;
	TestFalse(
		TEXT("不正な混合比を拒否"),
		PlanetCore::MinimalDrainageBlend::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, -0.01, InvalidBlendError)
		.has_value()
	);
	TestTrue(TEXT("不正な混合比の理由"), InvalidBlendError == PlanetCore::MinimalDrainageBlendError::InvalidInput);
	const std::optional<MinimalDrainageFill> Repeated =
	MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, Error);
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("同一定義を再準備"), Repeated.has_value())) return false;
	std::uint32_t CheckedLand = 0;
	for(std::uint32_t Index = 0; Index < Original.size(); ++Index) {
		const double Height = Filled->GetHeightsMeters()[Index];
		TestTrue(
			TEXT("同じ標高と下流がビット一致"),
			std::bit_cast<std::uint64_t>(Height) == std::bit_cast<std::uint64_t>(Repeated->GetHeightsMeters()[Index]) &&
			Filled->GetDownstreamVertices()[Index] == Repeated->GetDownstreamVertices()[Index]
		);
		if(Original[Index] < Settings.SeaLevelMeters) {
			TestEqual(TEXT("元の海底を保存"), Height, Original[Index]);
			continue;
		}
		TestTrue(TEXT("元の陸の標高を下回らない"), Height >= Original[Index]);
		const std::uint32_t Parent = Filled->GetDownstreamVertices()[Index];
		// 必要な標高又は排水条件を満たさない場合の返戻
		if(!TestTrue(TEXT("有限な下流"), Parent < Original.size() && Parent != Index)) return false;
		const double ParentHeight =
		Original[Parent] < Settings.SeaLevelMeters ? Settings.SeaLevelMeters : Filled->GetHeightsMeters()[Parent];
		const PlanetCore::Vec3 Difference = Mesh->Directions[Index].Vector() - Mesh->Directions[Parent].Vector();
		const double Arc = 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
		TestTrue(
			TEXT("下流の標高と最小勾配を守る"),
			Height > ParentHeight && Height - ParentHeight + 1E-9 >= Settings.MinimumLandSlope * Settings.RadiusMeters * Arc
		);
		++CheckedLand;
	}
	std::vector<double> Invalid = Original;
	std::fill(Invalid.begin(), Invalid.end(), 1);
	TestFalse(
		TEXT("大洋の無い入力を拒否"),
		MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Invalid, Settings, Error).has_value()
	);
	TestTrue(TEXT("大洋の無い理由"), Error == MinimalDrainageFillError::NoOcean);
	std::fill(Invalid.begin(), Invalid.end(), -1);
	TestFalse(
		TEXT("陸の無い入力を拒否"),
		MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Invalid, Settings, Error).has_value()
	);
	TestTrue(TEXT("陸の無い理由"), Error == MinimalDrainageFillError::NoLand);
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断を拒否"),
		MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, Error, Cancelled.get_token()).has_value()
	);
	TestTrue(TEXT("中断理由"), Error == MinimalDrainageFillError::Cancelled);
	AddInfo(
		FString::Printf(
			TEXT("Minimal fill contracts: vertices=%u land=%u raised=%u maxRaise=%.9gm sumRaise=%.9gm."),
			static_cast<unsigned>(Original.size()),
			CheckedLand,
			Filled->GetStatistics().RaisedLandVertices,
			Filled->GetStatistics().MaximumRaiseMeters,
			Filled->GetStatistics().SumRaiseMeters
		)
	);
	// この網での契約と診断を終えた返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetMinimalDrainageFillProductionSurvey,
    "PlanetGenerator.MinimalDrainageFill.ProductionSurvey",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 通常の侵食・河谷・中規模起伏後の H を変え過ぎないか定量測定する
 * @param Parameters 未使用の UE 引数
 * @return 大量盛土の採否とは別の測定と有限網排水の検査
 */
bool FPlanetMinimalDrainageFillProductionSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	PlanetCore::GeologySettings GeologySettings;
	GeologySettings.Seed = 42;
	GeologySettings.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(GeologySettings);
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("通常地質を準備"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("通常大洋を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("通常排水網を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
	const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("侵食後の通常最終 H を準備"), Field.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 必要な標高又は排水条件を満たさない場合の返戻
	if(!TestTrue(TEXT("通常の全球共有網を準備"), Mesh.has_value())) return false;
	std::vector<double> Original;
	Original.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) {
		const double Height = Field->EvaluateElevation(Direction);
		// 必要な標高又は排水条件を満たさない場合の返戻
		if(!TestTrue(TEXT("最終 H の有限標高"), std::isfinite(Height))) return false;
		Original.push_back(Height);
	}
	PlanetCore::MinimalDrainageFillSettings Settings;
	Settings.RadiusMeters = GeologySettings.RadiusMeters;
	Settings.SeaLevelMeters = Field->Drainage().Settings().SeaLevelMeters;
	Settings.MinimumLandSlope = 1E-6;
	PlanetCore::MinimalDrainageFillError Error = PlanetCore::MinimalDrainageFillError::None;
	const std::optional<PlanetCore::MinimalDrainageFill> Filled =
	PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, Error);
	if(!TestTrue(TEXT("通常最終 H を地質保存で条件付け"), Filled.has_value())) {
		AddError(FString::Printf(TEXT("Minimal fill reason=%u."), static_cast<unsigned>(Error)));
		// 必要な標高又は排水条件を満たさない場合の返戻
		return false;
	}
	PlanetCore::SphericalDrainageBaselineError BaselineError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		Filled->GetHeightsMeters(),
		Settings.SeaLevelMeters,
		BaselineError
	);
	if(!TestTrue(TEXT("通常最終 H に対する全球区分線形排水"), Baseline.has_value())) {
		AddError(FString::Printf(TEXT("Baseline reason=%u."), static_cast<unsigned>(BaselineError)));
		// 必要な標高又は排水条件を満たさない場合の返戻
		return false;
	}
	std::uint32_t Changed = 0;
	double Maximum = 0;
	for(std::uint32_t Index = 0; Index < Original.size(); ++Index) {
		const double Difference = Filled->GetHeightsMeters()[Index] - Original[Index];
		if(Original[Index] < Settings.SeaLevelMeters) {
			TestEqual(TEXT("大洋海底を変更しない"), Difference, 0.0);
			continue;
		}
		TestTrue(TEXT("乾いた元標高を切り下げない"), Difference >= 0);
		if(Difference > 0.01) ++Changed;
		Maximum = std::max(Maximum, Difference);
	}
	const std::chrono::steady_clock::time_point SmoothStarted = std::chrono::steady_clock::now();
	std::vector<double> Corrections;
	std::vector<bool> MaterialRaise;
	Corrections.reserve(Original.size());
	MaterialRaise.reserve(Original.size());
	for(std::uint32_t Index = 0; Index < Original.size(); ++Index) {
		const double Raise = Filled->GetHeightsMeters()[Index] - Original[Index];
		Corrections.push_back(Raise);
		MaterialRaise.push_back(Raise > 0.01);
	}
	const std::optional<PlanetCore::PlanetBasinReconstruction> Smooth =
	PlanetCore::PlanetBasinReconstruction::CreateGlobal(6, Corrections);
	const std::optional<PlanetCore::SphericalSplineField> Shape =
	PlanetCore::SphericalSplineField::CreateOnMesh(6, Mesh->Directions, Mesh->Triangles, Filled->GetHeightsMeters());
	std::uint32_t DryDepartureVertices = 0, ParentDepartureRises = 0, NoNeighborDeparture = 0;
	double MaximumParentDepartureRise = 0;
	if(Shape) {
		std::vector<std::vector<std::uint32_t>> Neighbors(Mesh->Directions.size());
		for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			Neighbors[A].push_back(B);
			Neighbors[B].push_back(A);
		}
		for(std::uint32_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			if(Original[Index] < Settings.SeaLevelMeters) continue;
			++DryDepartureVertices;
			std::vector<std::uint32_t> &Adjacent = Neighbors[Index];
			std::sort(Adjacent.begin(), Adjacent.end());
			Adjacent.erase(std::unique(Adjacent.begin(), Adjacent.end()), Adjacent.end());
			bool HasLowerDeparture = false;
			for(const std::uint32_t Other : Adjacent) {
				const PlanetCore::Vec3 Near = Mesh->Directions[Index].Vector() * 0.999 + Mesh->Directions[Other].Vector() * 0.001;
				const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector(Near);
				if(!Direction) {
					AddError(TEXT("A departure direction is invalid."));
					// 不正な方向を排水済みと扱わない返戻
					return false;
				}
				const double NearHeight = Shape->Evaluate(*Direction).ElevationMeters;
				if(!std::isfinite(NearHeight)) {
					AddError(TEXT("A departure elevation is nonfinite."));
					// 非有限な標高を排水済みと扱わない返戻
					return false;
				}
				const double Departure = NearHeight - Filled->GetHeightsMeters()[Index];
				if(Departure < -1E-8) HasLowerDeparture = true;
				if(Other != Filled->GetDownstreamVertices()[Index] || Departure <= 1E-8) continue;
				++ParentDepartureRises;
				MaximumParentDepartureRise = std::max(MaximumParentDepartureRise, Departure);
			}
			if(!HasLowerDeparture) ++NoNeighborDeparture;
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Quadratic dry departure: vertices=%u parentRises=%u noLowerNeighbor=%u ")
			TEXT("maxParentRise=%.9gm at edgeFraction=0.001. Samples are not an analytic guarantee."),
			DryDepartureVertices,
			ParentDepartureRises,
			NoNeighborDeparture,
			MaximumParentDepartureRise
		)
	);
	std::uint32_t ShapeEdgeDips = 0;
	double MaximumShapeDip = 0, MaximumShapeNodeResidual = 0;
	std::uint32_t EdgeMidpointDips = 0, CheckedEdgeMidpoints = 0;
	std::uint32_t OriginalEdgeDips = 0, IntroducedEdgeDips = 0, WorsenedEdgeDips = 0;
	double MaximumEdgeDip = 0, MaximumNodeResidual = 0, MaximumAddedDip = 0;
	double MinimumMidpointCorrection = std::numeric_limits<double>::infinity();
	if(Smooth) {
		for(std::uint32_t Index = 0; Index < Original.size(); ++Index) {
			const double Reconstructed = Original[Index] + Smooth->EvaluateElevation(Mesh->Directions[Index]);
			MaximumNodeResidual = std::max(MaximumNodeResidual, std::abs(Reconstructed - Filled->GetHeightsMeters()[Index]));
			if(Shape) {
				const double ShapeNode = Shape->Evaluate(Mesh->Directions[Index]).ElevationMeters;
				MaximumShapeNodeResidual = std::max(MaximumShapeNodeResidual, std::abs(ShapeNode - Filled->GetHeightsMeters()[Index]));
			}
		}
		std::vector<std::array<std::uint32_t, 2>> Edges;
		Edges.reserve(Mesh->Triangles.size() * 3);
		for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			if(!MaterialRaise[A] && !MaterialRaise[B]) continue;
			Edges.push_back({ std::min(A, B), std::max(A, B) });
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			const std::uint32_t A = Edge[0], B = Edge[1];
			if(Filled->GetHeightsMeters()[A] < Settings.SeaLevelMeters || Filled->GetHeightsMeters()[B] < Settings.SeaLevelMeters) continue;
			const std::optional<PlanetCore::UnitDirection> Midpoint =
			PlanetCore::UnitDirection::FromVector(Mesh->Directions[A].Vector() + Mesh->Directions[B].Vector());
			if(!Midpoint) continue;
			const double OriginalMidpoint = Field->EvaluateElevation(*Midpoint), MidpointCorrection = Smooth->EvaluateElevation(*Midpoint);
			const double Height = OriginalMidpoint + MidpointCorrection;
			if(!std::isfinite(Height)) {
				AddError(TEXT("The smoothed drainage correction has a nonfinite midpoint."));
				// 非有限な補正を地形として受理しない返戻
				return false;
			}
			MinimumMidpointCorrection = std::min(MinimumMidpointCorrection, MidpointCorrection);
			const double OriginalDip = std::min(Original[A], Original[B]) - OriginalMidpoint;
			const double Dip = std::min(Filled->GetHeightsMeters()[A], Filled->GetHeightsMeters()[B]) - Height;
			if(OriginalDip > 0.01) ++OriginalEdgeDips;
			if(Dip > 0.01) {
				++EdgeMidpointDips;
				MaximumEdgeDip = std::max(MaximumEdgeDip, Dip);
				if(OriginalDip <= 0.01) ++IntroducedEdgeDips;
			}
			if(Dip > OriginalDip + 0.01) {
				++WorsenedEdgeDips;
				MaximumAddedDip = std::max(MaximumAddedDip, Dip - OriginalDip);
			}
			if(Shape) {
				const double ShapeMidpoint = Shape->Evaluate(*Midpoint).ElevationMeters;
				if(!std::isfinite(ShapeMidpoint)) {
					AddError(TEXT("The quadratic spherical interpolant has a nonfinite midpoint."));
					// 非有限な二次球面を排水面として扱わない返戻
					return false;
				}
				const double ShapeDip = std::min(Filled->GetHeightsMeters()[A], Filled->GetHeightsMeters()[B]) - ShapeMidpoint;
				if(ShapeDip > 0.01) {
					++ShapeEdgeDips;
					MaximumShapeDip = std::max(MaximumShapeDip, ShapeDip);
				}
			}
			++CheckedEdgeMidpoints;
		}
	}
	std::uint32_t AllDryEdges = 0, AllDryFaces = 0, SampledEdgeDips = 0, SampledFaceDips = 0;
	double MaximumSampledEdgeDip = 0, MaximumSampledFaceDip = 0;
	if(Shape) {
		std::vector<std::array<std::uint32_t, 2>> Edges;
		Edges.reserve(Mesh->Triangles.size() * 3);
		for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) {
			for(unsigned Side = 0; Side < 3; ++Side) {
				const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
				Edges.push_back({ std::min(A, B), std::max(A, B) });
			}
			const double First = Filled->GetHeightsMeters()[Face[0]], Second = Filled->GetHeightsMeters()[Face[1]];
			const double Third = Filled->GetHeightsMeters()[Face[2]];
			if(First < Settings.SeaLevelMeters || Second < Settings.SeaLevelMeters || Third < Settings.SeaLevelMeters) continue;
			const std::optional<PlanetCore::UnitDirection> Center = PlanetCore::UnitDirection::FromVector(
				Mesh->Directions[Face[0]].Vector() + Mesh->Directions[Face[1]].Vector() + Mesh->Directions[Face[2]].Vector()
			);
			if(!Center) continue;
			const double CenterHeight = Shape->Evaluate(*Center).ElevationMeters, Dip = std::min({ First, Second, Third }) - CenterHeight;
			if(!std::isfinite(CenterHeight)) {
				AddError(TEXT("The quadratic spherical face has a nonfinite center."));
				// 不正な面内試料を無視しない返戻
				return false;
			}
			if(Dip > 0.01) {
				++SampledFaceDips;
				MaximumSampledFaceDip = std::max(MaximumSampledFaceDip, Dip);
			}
			++AllDryFaces;
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			const std::uint32_t A = Edge[0], B = Edge[1];
			const double Low = Filled->GetHeightsMeters()[A], High = Filled->GetHeightsMeters()[B];
			if(Low < Settings.SeaLevelMeters || High < Settings.SeaLevelMeters) continue;
			bool HasDip = false;
			for(const double Fraction : { 0.25, 0.5, 0.75 }) {
				const std::optional<PlanetCore::UnitDirection> Along =
				PlanetCore::UnitDirection::FromVector(Mesh->Directions[A].Vector() * (1 - Fraction) + Mesh->Directions[B].Vector() * Fraction);
				if(!Along) continue;
				const double Height = Shape->Evaluate(*Along).ElevationMeters;
				if(!std::isfinite(Height)) {
					AddError(TEXT("The quadratic spherical edge has a nonfinite sample."));
					// 不正な共有辺試料を無視しない返戻
					return false;
				}
				const double Dip = std::min(Low, High) - Height;
				if(Dip <= 0.01) continue;
				HasDip = true;
				MaximumSampledEdgeDip = std::max(MaximumSampledEdgeDip, Dip);
			}
			if(HasDip) ++SampledEdgeDips;
			++AllDryEdges;
		}
	}
	std::uint64_t DryPieceSideIncidences = 0, InteriorMinimumIncidences = 0, BelowEndpointMinimumIncidences = 0;
	std::uint64_t MeaningfulInteriorMinima = 0, SmallFullGradientCandidates = 0;
	double MaximumAnalyticSideDip = 0, MinimumFullGradient = std::numeric_limits<double>::infinity();
	if(Shape) for(std::uint32_t Index = 0; Index < Shape->GetStatistics().Pieces; ++Index) {
		const std::optional<std::array<PlanetCore::Vec3, 3>> Domain = Shape->GetPieceDomain(Index);
		if(!Domain) {
			AddError(TEXT("A quadratic piece domain could not be recovered."));
			// 欠けた小区画を排水済みとして扱わない返戻
			return false;
		}
		std::array<double, 3> Corners {};
		for(unsigned Corner = 0; Corner < 3; ++Corner) {
			const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector((*Domain)[Corner]);
			if(!Direction) {
				AddError(TEXT("A quadratic piece corner is invalid."));
				// 無効な小区画角を除外して合格にしない返戻
				return false;
			}
			Corners[Corner] = Shape->EvaluatePiece(Index, *Direction)->ElevationMeters;
		}
		for(unsigned Side = 0; Side < 3; ++Side) {
			const unsigned Other = (Side + 1) % 3;
			const double First = Corners[Side], Last = Corners[Other];
			if(First < Settings.SeaLevelMeters || Last < Settings.SeaLevelMeters) continue;
			const PlanetCore::Vec3 A = (*Domain)[Side], B = (*Domain)[Other];
			const double ChordSquared = (A - B).Dot(A - B);
			const std::optional<PlanetCore::UnitDirection> Middle = PlanetCore::UnitDirection::FromVector(A + B);
			if(!Middle) {
				AddError(TEXT("A quadratic piece edge midpoint is invalid."));
				// 無効な共有辺を無視しない返戻
				return false;
			}
			const double Midpoint = Shape->EvaluatePiece(Index, *Middle)->ElevationMeters, Dmid = 1 - 0.25 * ChordSquared;
			const double M1 = Last - First, Mmid = (Midpoint - First) * Dmid, NumeratorA = 2 * (M1 - 2 * Mmid);
			const double NumeratorB = M1 - NumeratorA, PolynomialA = -ChordSquared * M1, PolynomialB = 2 * NumeratorA;
			const double PolynomialC = NumeratorB;
			const double Scale = std::max({ std::abs(PolynomialA), std::abs(PolynomialB), std::abs(PolynomialC) });
			std::array<double, 2> Roots {};
			unsigned RootCount = 0;
			if(Scale > 0 && std::abs(PolynomialA) <= Scale * 1E-14) {
				if(std::abs(PolynomialB) > Scale * 1E-14) Roots[RootCount++] = -PolynomialC / PolynomialB;
			} else if(Scale > 0) {
				const double Discriminant = PolynomialB * PolynomialB - 4 * PolynomialA * PolynomialC;
				if(Discriminant >= 0) {
					const double SqrtDiscriminant = std::sqrt(Discriminant);
					Roots[RootCount++] = (-PolynomialB - SqrtDiscriminant) / (2 * PolynomialA);
					Roots[RootCount++] = (-PolynomialB + SqrtDiscriminant) / (2 * PolynomialA);
				}
			}
			for(unsigned Root = 0; Root < RootCount; ++Root) {
				const double Fraction = Roots[Root];
				if(!(Fraction > 0 && Fraction < 1) || !(2 * PolynomialA * Fraction + PolynomialB > 0)) continue;
				const std::optional<PlanetCore::UnitDirection> Stationary =
				PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
				if(!Stationary) {
					AddError(TEXT("A quadratic piece stationary point is invalid."));
					// 不正な停留候補を排水済みとしない返戻
					return false;
				}
				const PlanetCore::TerrainSample Sample = *Shape->EvaluatePiece(Index, *Stationary);
				const double Height = Sample.ElevationMeters, Gradient = Sample.TangentialGradientPerRadian.Length();
				if(!std::isfinite(Height) || !std::isfinite(Gradient)) {
					AddError(TEXT("A quadratic piece stationary sample is nonfinite."));
					// 非有限な停留標本を受理しない返戻
					return false;
				}
				++InteriorMinimumIncidences;
				if(Fraction >= 1E-5 && Fraction <= 1 - 1E-5) {
					++MeaningfulInteriorMinima;
					MinimumFullGradient = std::min(MinimumFullGradient, Gradient);
					if(Gradient < 1E-3) ++SmallFullGradientCandidates;
				}
				const double Dip = std::min(First, Last) - Height;
				if(Dip <= 0.01) continue;
				++BelowEndpointMinimumIncidences;
				MaximumAnalyticSideDip = std::max(MaximumAnalyticSideDip, Dip);
			}
			++DryPieceSideIncidences;
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Quadratic analytic piece edges: drySideIncidences=%llu interiorMinima=%llu ")
			TEXT("meaningful=%llu smallGradient=%llu minGradient=%.9gmPerRadian ")
			TEXT("belowEndpoints=%llu maxDip=%.9gm. Side incidences include duplicates."),
			static_cast<unsigned long long>(DryPieceSideIncidences),
			static_cast<unsigned long long>(InteriorMinimumIncidences),
			static_cast<unsigned long long>(MeaningfulInteriorMinima),
			static_cast<unsigned long long>(SmallFullGradientCandidates),
			MinimumFullGradient,
			static_cast<unsigned long long>(BelowEndpointMinimumIncidences),
			MaximumAnalyticSideDip
		)
	);
	std::uint64_t PieceCriticalInteriors = 0, DryPieceInteriorMinima = 0, AmbiguousPieceEigenvalues = 0;
	std::uint64_t AmbiguousTouchesDry = 0, AmbiguousAllDry = 0;
	std::uint64_t UnresolvedPieceEigenvalues = 0;
	double MaximumCriticalGradient = 0, MaximumQuadraticResidual = 0;
	if(Shape) for(std::uint32_t Index = 0; Index < Shape->GetStatistics().Pieces; ++Index) {
		const std::array<PlanetCore::Vec3, 3> Domain = *Shape->GetPieceDomain(Index);
		const auto ExtendedHeight = [&](const PlanetCore::Vec3 Point) -> double {
			const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector(Point);
			// 二次式の区画外延長を同じ評価経路で復元し，無効方向は非有限値で拒否する返戻
			return Direction ? Shape->EvaluatePiece(Index, *Direction)->ElevationMeters : std::numeric_limits<double>::quiet_NaN();
		};
		const PlanetCore::Vec3 X { 1, 0, 0 }, Y { 0, 1, 0 }, Z { 0, 0, 1 };
		const double HX = ExtendedHeight(X), HY = ExtendedHeight(Y), HZ = ExtendedHeight(Z), HXY = ExtendedHeight(X + Y);
		const double HXZ = ExtendedHeight(X + Z), HYZ = ExtendedHeight(Y + Z);
		if(!std::isfinite(HX + HY + HZ + HXY + HXZ + HYZ)) {
			AddError(TEXT("A quadratic piece cannot be reconstructed from six unit directions."));
			// 非有限な二次片を排水済みとして扱わない返戻
			return false;
		}
		std::array<std::array<double, 3>, 3> Form {
			{
				{ { HX - HZ, HXY - 0.5 * (HX + HY), HXZ - 0.5 * (HX + HZ) } },
				{ { HXY - 0.5 * (HX + HY), HY - HZ, HYZ - 0.5 * (HY + HZ) } },
				{ { HXZ - 0.5 * (HX + HZ), HYZ - 0.5 * (HY + HZ), 0 } }
			}
		};
		const double Scale =
		std::max({ 1.0, std::abs(Form[0][0]), std::abs(Form[1][1]), std::abs(Form[0][1]), std::abs(Form[0][2]), std::abs(Form[1][2]) });
		if(!(Index % 4096)) {
			const PlanetCore::Vec3 Point = PlanetCore::UnitDirection::FromVector(Domain[0] + Domain[1] + Domain[2])->Vector();
			const double Predicted = HZ + Form[0][0] * Point.X * Point.X + Form[1][1] * Point.Y * Point.Y +
			2 * (Form[0][1] * Point.X * Point.Y + Form[0][2] * Point.X * Point.Z + Form[1][2] * Point.Y * Point.Z);
			MaximumQuadraticResidual = std::max(MaximumQuadraticResidual, std::abs(Predicted - ExtendedHeight(Point)));
		}
		std::array<std::array<double, 3>, 3> Modes { { { { 1, 0, 0 } }, { { 0, 1, 0 } }, { { 0, 0, 1 } } } };
		for(unsigned Iteration = 0; Iteration < 32; ++Iteration) {
			unsigned P = 0, Q = 1;
			for(const std::array<unsigned, 2> Pair : { std::array<unsigned, 2>{ 0, 2 }, { 1, 2 } }) {
				if(std::abs(Form[Pair[0]][Pair[1]]) > std::abs(Form[P][Q])) {
					P = Pair[0];
					Q = Pair[1];
				}
			}
			const double Off = Form[P][Q];
			if(std::abs(Off) <= Scale * 1E-13) break;
			const double Tau = (Form[Q][Q] - Form[P][P]) / (2 * Off);
			const double T = std::copysign(1.0, Tau) / (std::abs(Tau) + std::sqrt(1 + Tau * Tau)), C = 1 / std::sqrt(1 + T * T), S = T * C;
			const double OldP = Form[P][P], OldQ = Form[Q][Q];
			Form[P][P] = OldP - T * Off;
			Form[Q][Q] = OldQ + T * Off;
			Form[P][Q] = Form[Q][P] = 0;
			for(unsigned Row = 0; Row < 3; ++Row) {
				if(Row != P && Row != Q) {
					const double RP = Form[Row][P], RQ = Form[Row][Q];
					Form[Row][P] = Form[P][Row] = C * RP - S * RQ;
					Form[Row][Q] = Form[Q][Row] = S * RP + C * RQ;
				}
				const double VP = Modes[Row][P], VQ = Modes[Row][Q];
				Modes[Row][P] = C * VP - S * VQ;
				Modes[Row][Q] = S * VP + C * VQ;
			}
		}
		if(std::max({ std::abs(Form[0][1]), std::abs(Form[0][2]), std::abs(Form[1][2]) }) > Scale * 1E-10) {
			++UnresolvedPieceEigenvalues;
			continue;
		}
		const double Minimum = std::min({ Form[0][0], Form[1][1], Form[2][2] });
		unsigned MinimumMultiplicity = 0;
		for(unsigned Mode = 0; Mode < 3; ++Mode) if(Form[Mode][Mode] - Minimum <= Scale * 1E-9) ++MinimumMultiplicity;
		if(MinimumMultiplicity > 1) {
			++AmbiguousPieceEigenvalues;
			std::array<double, 3> CornerHeights {};
			for(unsigned Corner = 0; Corner < 3; ++Corner) CornerHeights[Corner] = ExtendedHeight(Domain[Corner]);
			if(*std::max_element(CornerHeights.begin(), CornerHeights.end()) > Settings.SeaLevelMeters + 0.01) ++AmbiguousTouchesDry;
			if(*std::min_element(CornerHeights.begin(), CornerHeights.end()) > Settings.SeaLevelMeters + 0.01) ++AmbiguousAllDry;
		}
		const double Determinant = Domain[0].Dot(Domain[1].Cross(Domain[2]));
		if(!(std::abs(Determinant) > 1E-18)) {
			AddError(TEXT("A quadratic piece has a degenerate cone."));
			// 退化した小区画を排水済みとして扱わない返戻
			return false;
		}
		for(unsigned Mode = 0; Mode < 3; ++Mode) for(const double Sign : { -1.0, 1.0 }) {
			const PlanetCore::Vec3 Eigenvector { Sign * Modes[0][Mode], Sign * Modes[1][Mode], Sign * Modes[2][Mode] };
			const double U = Eigenvector.Dot(Domain[1].Cross(Domain[2])) / Determinant;
			const double V = Eigenvector.Dot(Domain[2].Cross(Domain[0])) / Determinant;
			const double W = Eigenvector.Dot(Domain[0].Cross(Domain[1])) / Determinant;
			if(std::min({ U, V, W }) <= 1E-5) continue;
			const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector(Eigenvector);
			if(!Direction) continue;
			const PlanetCore::TerrainSample Critical = *Shape->EvaluatePiece(Index, *Direction);
			++PieceCriticalInteriors;
			MaximumCriticalGradient = std::max(MaximumCriticalGradient, Critical.TangentialGradientPerRadian.Length());
			if(Form[Mode][Mode] - Minimum <= Scale * 1E-9 && Critical.ElevationMeters > Settings.SeaLevelMeters + 0.01) {
				++DryPieceInteriorMinima;
			}
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Quadratic analytic piece interiors: critical=%llu dryMinima=%llu ambiguous=%llu ")
			TEXT("ambiguousTouchesDry=%llu ambiguousAllDry=%llu unresolved=%llu ")
			TEXT("maxCriticalGradient=%.9gmPerRadian maxFormResidual=%.9gm. ")
			TEXT("Dry minima require further topology classification."),
			static_cast<unsigned long long>(PieceCriticalInteriors),
			static_cast<unsigned long long>(DryPieceInteriorMinima),
			static_cast<unsigned long long>(AmbiguousPieceEigenvalues),
			static_cast<unsigned long long>(AmbiguousTouchesDry),
			static_cast<unsigned long long>(AmbiguousAllDry),
			static_cast<unsigned long long>(UnresolvedPieceEigenvalues),
			MaximumCriticalGradient,
			MaximumQuadraticResidual
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Quadratic global samples: dryEdges=%u edgeDips=%u maxEdgeDip=%.9gm ")
			TEXT("dryFaces=%u centerDips=%u maxCenterDip=%.9gm. Samples are not an analytic guarantee."),
			AllDryEdges,
			SampledEdgeDips,
			MaximumSampledEdgeDip,
			AllDryFaces,
			SampledFaceDips,
			MaximumSampledFaceDip
		)
	);
	const double SmoothSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - SmoothStarted).count();
	AddInfo(
		FString::Printf(
			TEXT("Minimal fill quadratic comparison: created=%u edgeDips=%u maxDip=%.9gm ")
			TEXT("maxNodalResidual=%.9gm sampledEdges=%u secondsIncludingC2=%.9g."),
			static_cast<unsigned>(Shape.has_value()),
			ShapeEdgeDips,
			MaximumShapeDip,
			MaximumShapeNodeResidual,
			CheckedEdgeMidpoints,
			SmoothSeconds
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Minimal fill C2 survey: created=%u sampledEdges=%u originalDips=%u correctedDips=%u ")
			TEXT("introduced=%u worsened=%u maxDip=%.9gm maxAddedDip=%.9gm minCorrection=%.9gm ")
			TEXT("maxNodalResidual=%.9gm seconds=%.9g. Midpoint dips do not prove closed basins."),
			static_cast<unsigned>(Smooth.has_value()),
			CheckedEdgeMidpoints,
			OriginalEdgeDips,
			EdgeMidpointDips,
			IntroducedEdgeDips,
			WorsenedEdgeDips,
			MaximumEdgeDip,
			MaximumAddedDip,
			MinimumMidpointCorrection,
			MaximumNodeResidual,
			SmoothSeconds
		)
	);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Production minimal fill: vertices=%u ocean=%u changedAbove1cm=%u maxRaise=%.9gm ")
			TEXT("sumRaise=%.9gm minimumDrop=%.9gm seconds=%.9g. PL reference, landscape unaccepted."),
			static_cast<unsigned>(Original.size()),
			Filled->GetStatistics().OceanVertices,
			Changed,
			Maximum,
			Filled->GetStatistics().SumRaiseMeters,
			Filled->GetStatistics().MinimumLandDropMeters,
			Seconds
		)
	);
	// この網での契約と診断を終えた返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetMinimalDrainageFillProductionRange,
    "PlanetGenerator.MinimalDrainageFill.ProductionRange",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 複数 Seed と半径で実際の侵食後 H に有限網の大洋排水条件を適用する
 * @param Parameters 未使用の UE 引数
 * @return 標高を変える量と区分線形排水の比較結果
 */
bool FPlanetMinimalDrainageFillProductionRange::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	struct Case {
		std::uint64_t Seed;
		double RadiusMeters;
	};
	const std::array<Case, 4> Cases { { { 7, 1E6 }, { 44, 1E6 }, { 42, 1E3 }, { 42, 1E9 } } };
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 全球の共有基準網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較用の全球網を準備"), Mesh.has_value())) return false;
	for(const Case Current : Cases) {
		const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
		PlanetCore::GeologySettings GeologySettings;
		GeologySettings.Seed = Current.Seed;
		GeologySettings.RadiusMeters = Current.RadiusMeters;
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
		// 通常の侵食後 H を準備出来ない場合の返戻
		if(!TestTrue(TEXT("比較条件の侵食後 H を準備"), Field.has_value())) return false;
		std::vector<double> Original;
		Original.reserve(Mesh->Directions.size());
		for(const PlanetCore::UnitDirection Direction : Mesh->Directions) Original.push_back(Field->EvaluateElevation(Direction));
		PlanetCore::MinimalDrainageFillSettings Settings;
		Settings.RadiusMeters = Current.RadiusMeters;
		Settings.SeaLevelMeters = Field->Drainage().Settings().SeaLevelMeters;
		Settings.MinimumLandSlope = 1E-6;
		PlanetCore::MinimalDrainageFillError Error = PlanetCore::MinimalDrainageFillError::None;
		const std::optional<PlanetCore::MinimalDrainageFill> Filled =
		PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, Error);
		if(!TestTrue(TEXT("比較条件の最小盛土を準備"), Filled.has_value())) {
			AddError(
				FString::Printf(
					TEXT("Seed=%llu radius=%.9gm reason=%u."),
					static_cast<unsigned long long>(Current.Seed),
					Current.RadiusMeters,
					static_cast<unsigned>(Error)
				)
			);
			// 失敗した条件を無視して比較完了としない返戻
			return false;
		}
		PlanetCore::SphericalDrainageBaselineError BaselineError = PlanetCore::SphericalDrainageBaselineError::None;
		const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
			Mesh->Directions,
			Mesh->Triangles,
			Filled->GetHeightsMeters(),
			Settings.SeaLevelMeters,
			BaselineError
		);
		if(!TestTrue(TEXT("比較条件の全陸から大洋への有限網排水"), Baseline.has_value())) {
			AddError(
				FString::Printf(
					TEXT("Seed=%llu radius=%.9gm baselineReason=%u."),
					static_cast<unsigned long long>(Current.Seed),
					Current.RadiusMeters,
					static_cast<unsigned>(BaselineError)
				)
			);
			// 失敗した条件を無視して比較完了としない返戻
			return false;
		}
		std::uint32_t ChangedAboveCentimeter = 0;
		for(std::size_t Index = 0; Index < Original.size(); ++Index) {
			const double Change = Filled->GetHeightsMeters()[Index] - Original[Index];
			if(Original[Index] < Settings.SeaLevelMeters) {
				// 大洋の海底を変更した場合の返戻
				if(!TestEqual(TEXT("元の海底を保存"), Change, 0.0)) return false;
				continue;
			}
			if(Change > 0.01) ++ChangedAboveCentimeter;
		}
		PlanetCore::MinimalDrainageBlendError BlendError = PlanetCore::MinimalDrainageBlendError::None;
		const std::optional<PlanetCore::MinimalDrainageBlend> Blend =
		PlanetCore::MinimalDrainageBlend::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, 0.5, BlendError);
		AddInfo(
			FString::Printf(
				TEXT("Production blend range: seed=%llu radius=%.9gm prepared=%u reason=%u ")
				TEXT("changed=%u maxRaise=%.9gm maxCut=%.9gm sumAbsolute=%.9gm. Landscape unaccepted."),
				static_cast<unsigned long long>(Current.Seed),
				Current.RadiusMeters,
				static_cast<unsigned>(Blend.has_value()),
				static_cast<unsigned>(BlendError),
				Blend ? Blend->GetStatistics().ChangedLandVertices : 0,
				Blend ? Blend->GetStatistics().MaximumRaiseMeters : 0,
				Blend ? Blend->GetStatistics().MaximumCutMeters : 0,
				Blend ? Blend->GetStatistics().SumAbsoluteChangeMeters : 0
			)
		);
		PlanetCore::MinimalDrainageBlendError CoastalError = PlanetCore::MinimalDrainageBlendError::None;
		const std::optional<PlanetCore::MinimalDrainageBlend> Coastal = PlanetCore::MinimalDrainageBlend::CreateWithCoastalBalance(
			Mesh->Directions,
			Mesh->Triangles,
			Original,
			Settings,
			0.5,
			CoastalError
		);
		AddInfo(
			FString::Printf(
				TEXT("Coastal blend range: seed=%llu radius=%.9gm prepared=%u reason=%u ")
				TEXT("fillWeight=%.9g maxRaise=%.9gm maxCut=%.9gm sumAbsolute=%.9gm. Landscape unaccepted."),
				static_cast<unsigned long long>(Current.Seed),
				Current.RadiusMeters,
				static_cast<unsigned>(Coastal.has_value()),
				static_cast<unsigned>(CoastalError),
				Coastal ? Coastal->GetStatistics().AppliedFillWeight : 0,
				Coastal ? Coastal->GetStatistics().MaximumRaiseMeters : 0,
				Coastal ? Coastal->GetStatistics().MaximumCutMeters : 0,
				Coastal ? Coastal->GetStatistics().SumAbsoluteChangeMeters : 0
			)
		);
		if(Current.RadiusMeters != 1E6) {
			PlanetCore::MinimalDrainageFillSettings AngularSettings = Settings;
			AngularSettings.MinimumLandSlope = 1E-6 * 1E6 / Current.RadiusMeters;
			PlanetCore::MinimalDrainageBlendError AngularError = PlanetCore::MinimalDrainageBlendError::None;
			const std::optional<PlanetCore::MinimalDrainageBlend> Angular = PlanetCore::MinimalDrainageBlend::CreateWithCoastalBalance(
				Mesh->Directions,
				Mesh->Triangles,
				Original,
				AngularSettings,
				0.5,
				AngularError
			);
			AddInfo(
				FString::Printf(
					TEXT("Angular descent comparison: seed=%llu radius=%.9gm numericalSlope=%.9g ")
					TEXT("prepared=%u reason=%u fillWeight=%.9g maxRaise=%.9gm maxCut=%.9gm ")
					TEXT("sumAbsolute=%.9gm minimumDrop=%.9gm. River-water slope is separate."),
					static_cast<unsigned long long>(Current.Seed),
					Current.RadiusMeters,
					AngularSettings.MinimumLandSlope,
					static_cast<unsigned>(Angular.has_value()),
					static_cast<unsigned>(AngularError),
					Angular ? Angular->GetStatistics().AppliedFillWeight : 0,
					Angular ? Angular->GetStatistics().MaximumRaiseMeters : 0,
					Angular ? Angular->GetStatistics().MaximumCutMeters : 0,
					Angular ? Angular->GetStatistics().SumAbsoluteChangeMeters : 0,
					Angular ? Angular->GetStatistics().MinimumLandDropMeters : 0
				)
			);
			if(Current.RadiusMeters == 1E9) {
				// 比較対象の混合補正が拒否された場合は参照しない
				if(!TestTrue(TEXT("巨大半径の角度基準混合補正を準備"), Angular.has_value())) return false;
				PlanetCore::MinimalDrainageMinimaxError AngularMinimaxError = PlanetCore::MinimalDrainageMinimaxError::None;
				const std::optional<PlanetCore::MinimalDrainageMinimax> AngularMinimax =
				PlanetCore::MinimalDrainageMinimax::Create(Mesh->Directions, Mesh->Triangles, Original, AngularSettings, AngularMinimaxError);
				AddInfo(
					FString::Printf(
						TEXT("Angular minimax: seed=%llu radius=%.9gm prepared=%u reason=%u ")
						TEXT("maxCorrection=%.9gm maxRaise=%.9gm maxCut=%.9gm sumAbsolute=%.9gm tie=%.9gm. ")
						TEXT("Landscape unaccepted."),
						static_cast<unsigned long long>(Current.Seed),
						Current.RadiusMeters,
						static_cast<unsigned>(AngularMinimax.has_value()),
						static_cast<unsigned>(AngularMinimaxError),
						AngularMinimax ? AngularMinimax->GetStatistics().MaximumAbsoluteChangeMeters : 0,
						AngularMinimax ? AngularMinimax->GetStatistics().MaximumRaiseMeters : 0,
						AngularMinimax ? AngularMinimax->GetStatistics().MaximumCutMeters : 0,
						AngularMinimax ? AngularMinimax->GetStatistics().SumAbsoluteChangeMeters : 0,
						AngularMinimax ? AngularMinimax->GetStatistics().MaximumTieBreakMeters : 0
					)
				);
				// 数値下降量を分離した比較も独立した排水検査を必須とする
				if(!TestTrue(TEXT("巨大半径の角度基準最小最大補正を準備"), AngularMinimax.has_value())) return false;
				TestTrue(
					TEXT("巨大半径の角度基準最小最大補正が混合補正以下"),
					AngularMinimax->GetStatistics().MaximumAbsoluteChangeMeters <=
					std::max(Angular->GetStatistics().MaximumRaiseMeters, Angular->GetStatistics().MaximumCutMeters) + 1E-6
				);
				PlanetCore::MinimalDrainageFillSettings NetworkSlopeSettings = Settings;
				NetworkSlopeSettings.MinimumLandSlope = 1E-7 * std::min(1.0, 1E6 / Current.RadiusMeters);
				PlanetCore::MinimalDrainageBlendError NetworkBlendError = PlanetCore::MinimalDrainageBlendError::None;
				const std::optional<PlanetCore::MinimalDrainageBlend> NetworkBlend = PlanetCore::MinimalDrainageBlend::CreateWithCoastalBalance(
					Mesh->Directions,
					Mesh->Triangles,
					Original,
					NetworkSlopeSettings,
					0.5,
					NetworkBlendError
				);
				PlanetCore::MinimalDrainageMinimaxError NetworkMinimaxError = PlanetCore::MinimalDrainageMinimaxError::None;
				const std::optional<PlanetCore::MinimalDrainageMinimax> NetworkMinimax = PlanetCore::MinimalDrainageMinimax::Create(
					Mesh->Directions,
					Mesh->Triangles,
					Original,
					NetworkSlopeSettings,
					NetworkMinimaxError
				);
				AddInfo(
					FString::Printf(
						TEXT("Network-slope postcondition comparison: seed=%llu radius=%.9gm slope=%.9g ")
						TEXT("blendReady=%u blendReason=%u blendMax=%.9gm blendSum=%.9gm ")
						TEXT("minimaxReady=%u minimaxReason=%u minimaxMax=%.9gm minimaxSum=%.9gm. ")
						TEXT("Postcondition survey only; landscape unaccepted."),
						static_cast<unsigned long long>(Current.Seed),
						Current.RadiusMeters,
						NetworkSlopeSettings.MinimumLandSlope,
						static_cast<unsigned>(NetworkBlend.has_value()),
						static_cast<unsigned>(NetworkBlendError),
						NetworkBlend ? std::max(NetworkBlend->GetStatistics().MaximumRaiseMeters, NetworkBlend->GetStatistics().MaximumCutMeters) : 0,
						NetworkBlend ? NetworkBlend->GetStatistics().SumAbsoluteChangeMeters : 0,
						static_cast<unsigned>(NetworkMinimax.has_value()),
						static_cast<unsigned>(NetworkMinimaxError),
						NetworkMinimax ? NetworkMinimax->GetStatistics().MaximumAbsoluteChangeMeters : 0,
						NetworkMinimax ? NetworkMinimax->GetStatistics().SumAbsoluteChangeMeters : 0
					)
				);
				// 実際の数値最低勾配でも連続面の保証とは区別して有限網を検証する
				if(!TestTrue(TEXT("排水網と同じ最低勾配の混合補正を準備"), NetworkBlend.has_value())) return false;
				// 補正候補が無い結果を成功として扱わない
				if(!TestTrue(TEXT("排水網と同じ最低勾配の最小最大補正を準備"), NetworkMinimax.has_value())) return false;
			}
		}
		if(Current.RadiusMeters == 1E9) {
			PlanetCore::MinimalDrainageMinimaxError MinimaxError = PlanetCore::MinimalDrainageMinimaxError::None;
			const std::optional<PlanetCore::MinimalDrainageMinimax> Minimax =
			PlanetCore::MinimalDrainageMinimax::Create(Mesh->Directions, Mesh->Triangles, Original, Settings, MinimaxError);
			AddInfo(
				FString::Printf(
					TEXT("Production minimax: seed=%llu radius=%.9gm prepared=%u reason=%u ")
					TEXT("maxCorrection=%.9gm maxRaise=%.9gm maxCut=%.9gm sumAbsolute=%.9gm changed=%u tie=%.9gm. ")
					TEXT("Landscape unaccepted."),
					static_cast<unsigned long long>(Current.Seed),
					Current.RadiusMeters,
					static_cast<unsigned>(Minimax.has_value()),
					static_cast<unsigned>(MinimaxError),
					Minimax ? Minimax->GetStatistics().MaximumAbsoluteChangeMeters : 0,
					Minimax ? Minimax->GetStatistics().MaximumRaiseMeters : 0,
					Minimax ? Minimax->GetStatistics().MaximumCutMeters : 0,
					Minimax ? Minimax->GetStatistics().SumAbsoluteChangeMeters : 0,
					Minimax ? Minimax->GetStatistics().ChangedLandVertices : 0,
					Minimax ? Minimax->GetStatistics().MaximumTieBreakMeters : 0
				)
			);
			// 有効な比較実装が失敗した場合は全体の試験を失敗とする
			if(!TestTrue(TEXT("巨大半径の最大補正最小化を準備"), Minimax.has_value())) return false;
			TestTrue(
				TEXT("巨大半径の最大補正が全盛土以下"),
				Minimax->GetStatistics().MaximumAbsoluteChangeMeters <= Filled->GetStatistics().MaximumRaiseMeters + 1E-8
			);
		}
		const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
		AddInfo(
			FString::Printf(
				TEXT("Production range: seed=%llu radius=%.9gm ocean=%u changed=%u maxRaise=%.9gm ")
				TEXT("sumRaise=%.9gm minimumDrop=%.9gm seconds=%.9g. Landscape unaccepted."),
				static_cast<unsigned long long>(Current.Seed),
				Current.RadiusMeters,
				Filled->GetStatistics().OceanVertices,
				ChangedAboveCentimeter,
				Filled->GetStatistics().MaximumRaiseMeters,
				Filled->GetStatistics().SumRaiseMeters,
				Filled->GetStatistics().MinimumLandDropMeters,
				Seconds
			)
		);
	}
	// 四条件の区分線形排水と盛土量を比較し終えた返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetMinimalDrainageFillPredrainageSurvey,
    "PlanetGenerator.MinimalDrainageFill.PredrainageSurvey",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 起伏を排水前へ移した既存経路が後付け地形の窪みを減らすか調べる
 * @param Parameters 未使用の UE 引数
 * @return 通常 H の有限網排水と盛土量の比較結果
 */
bool FPlanetMinimalDrainageFillPredrainageSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1E6;
	const std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Settings);
	// 通常の地質を準備出来ない場合の返戻
	if(!TestTrue(TEXT("排水前起伏の地質を準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*PreparedGeology));
	std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("排水前起伏の造山場を準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*PreparedRidges));
	for(
		const PlanetCore::PredrainageReliefMode Mode :
		{ PlanetCore::PredrainageReliefMode::MesoOnly, PlanetCore::PredrainageReliefMode::MesoAndLand }
	) {
		const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
		std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
		PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(Geology, Ridges, {}, Mode);
		// 起伏を含む大洋を準備出来ない場合の返戻
		if(!TestTrue(TEXT("排水前起伏の大洋を準備"), PreparedOcean.has_value())) return false;
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
		std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
		std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
		// 起伏を含む排水網を準備出来ない場合の返戻
		if(!TestTrue(TEXT("排水前起伏の排水網を準備"), PreparedDrainage.has_value())) return false;
		const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
		std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
		const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
		// 侵食後の H を準備出来ない場合の返戻
		if(!TestTrue(TEXT("排水前起伏の侵食後 H を準備"), Field.has_value())) return false;
		const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
		// 閉じた全球標本網を準備出来ない場合の返戻
		if(!TestTrue(TEXT("排水前起伏の全球網を準備"), Mesh.has_value())) return false;
		std::vector<double> Heights;
		Heights.reserve(Mesh->Directions.size());
		for(const PlanetCore::UnitDirection Direction : Mesh->Directions) Heights.push_back(Field->EvaluateElevation(Direction));
		PlanetCore::MinimalDrainageFillSettings FillSettings;
		FillSettings.RadiusMeters = Settings.RadiusMeters;
		FillSettings.SeaLevelMeters = Field->Drainage().Settings().SeaLevelMeters;
		FillSettings.MinimumLandSlope = 1E-6;
		PlanetCore::MinimalDrainageFillError Error = PlanetCore::MinimalDrainageFillError::None;
		const std::optional<PlanetCore::MinimalDrainageFill> Filled =
		PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Heights, FillSettings, Error);
		if(!TestTrue(TEXT("排水前起伏の最終 H を条件付け"), Filled.has_value())) {
			AddError(FString::Printf(TEXT("Predrainage fill reason=%u."), static_cast<unsigned>(Error)));
			// 有効な結果が無い条件を成功として扱わない返戻
			return false;
		}
		PlanetCore::SphericalDrainageBaselineError BaselineError = PlanetCore::SphericalDrainageBaselineError::None;
		const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
			Mesh->Directions,
			Mesh->Triangles,
			Filled->GetHeightsMeters(),
			FillSettings.SeaLevelMeters,
			BaselineError
		);
		if(!TestTrue(TEXT("排水前起伏の全陸から大洋への排水"), Baseline.has_value())) {
			AddError(FString::Printf(TEXT("Predrainage baseline reason=%u."), static_cast<unsigned>(BaselineError)));
			// 流出保証の無い比較面を成功として扱わない返戻
			return false;
		}
		if(Mode == PlanetCore::PredrainageReliefMode::MesoOnly) {
			std::vector<double> Cut = Heights;
			std::vector<std::uint32_t> Descending;
			Descending.reserve(Heights.size() - Filled->GetStatistics().OceanVertices);
			for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
				if(Heights[Index] >= FillSettings.SeaLevelMeters) Descending.push_back(Index);
			}
			std::sort(
				Descending.begin(),
				Descending.end(),
				[&](const std::uint32_t A, const std::uint32_t B) {
					const double First = Filled->GetHeightsMeters()[A], Second = Filled->GetHeightsMeters()[B];
					// 子を親より先に切下げるため，盛土木の標高の降順を返す
					return First == Second ? A > B : First > Second;
				}
			);
			std::uint32_t ShoreShortages = 0;
			for(const std::uint32_t Vertex : Descending) {
				const std::uint32_t Parent = Filled->GetDownstreamVertices()[Vertex];
				const PlanetCore::Vec3 Difference = Mesh->Directions[Vertex].Vector() - Mesh->Directions[Parent].Vector();
				const double Length = FillSettings.RadiusMeters * 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
				const double Drop = FillSettings.MinimumLandSlope * Length;
				if(Heights[Parent] < FillSettings.SeaLevelMeters) {
					if(Cut[Vertex] <= FillSettings.SeaLevelMeters + Drop) ++ShoreShortages;
					continue;
				}
				const double Upper = std::min(Cut[Vertex] - Drop, std::nextafter(Cut[Vertex], -std::numeric_limits<double>::infinity()));
				Cut[Parent] = std::min(Cut[Parent], Upper);
			}
			std::uint32_t CutNodes = 0, NewlyFlooded = 0;
			double MaximumCut = 0, SumCut = 0;
			std::vector<double> Halfway(Heights.size());
			for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
				const double Lowering = Heights[Index] - Cut[Index];
				if(Lowering > 0.01) ++CutNodes;
				MaximumCut = std::max(MaximumCut, Lowering);
				SumCut += Lowering;
				if(Heights[Index] >= FillSettings.SeaLevelMeters && Cut[Index] < FillSettings.SeaLevelMeters) ++NewlyFlooded;
				Halfway[Index] = 0.5 * (Filled->GetHeightsMeters()[Index] + Cut[Index]);
			}
			PlanetCore::SphericalDrainageBaselineError CutError = PlanetCore::SphericalDrainageBaselineError::None;
			const std::optional<PlanetCore::SphericalDrainageBaseline> CutBaseline =
			PlanetCore::SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Cut, FillSettings.SeaLevelMeters, CutError);
			PlanetCore::SphericalDrainageBaselineError MixedError = PlanetCore::SphericalDrainageBaselineError::None;
			const std::optional<PlanetCore::SphericalDrainageBaseline> MixedBaseline = PlanetCore::SphericalDrainageBaseline::Create(
				Mesh->Directions,
				Mesh->Triangles,
				Halfway,
				FillSettings.SeaLevelMeters,
				MixedError
			);
			PlanetCore::MinimalDrainageBlendError BlendError = PlanetCore::MinimalDrainageBlendError::None;
			const std::optional<PlanetCore::MinimalDrainageBlend> Blend =
			PlanetCore::MinimalDrainageBlend::Create(Mesh->Directions, Mesh->Triangles, Heights, FillSettings, 0.5, BlendError);
			if(!TestTrue(TEXT("前置起伏の同じ排水木で混合"), Blend.has_value())) {
				AddError(FString::Printf(TEXT("Production blend reason=%u."), static_cast<unsigned>(BlendError)));
				// 新しい比較場の拒否を見逃さない返戻
				return false;
			}
			double MaximumIndependentDifference = 0;
			for(std::size_t Index = 0; Index < Halfway.size(); ++Index) {
				MaximumIndependentDifference =
				std::max(MaximumIndependentDifference, std::abs(Halfway[Index] - Blend->GetHeightsMeters()[Index]));
			}
			TestTrue(TEXT("独立した同じ木の混合標高"), MaximumIndependentDifference <= 1E-8);
			PlanetCore::MinimalDrainageMinimaxError MinimaxError = PlanetCore::MinimalDrainageMinimaxError::None;
			const std::optional<PlanetCore::MinimalDrainageMinimax> Minimax =
			PlanetCore::MinimalDrainageMinimax::Create(Mesh->Directions, Mesh->Triangles, Heights, FillSettings, MinimaxError);
			AddInfo(
				FString::Printf(
					TEXT("Predrainage minimax: prepared=%u reason=%u maxCorrection=%.9gm ")
					TEXT("maxRaise=%.9gm maxCut=%.9gm sumAbsolute=%.9gm tie=%.9gm. Landscape unaccepted."),
					static_cast<unsigned>(Minimax.has_value()),
					static_cast<unsigned>(MinimaxError),
					Minimax ? Minimax->GetStatistics().MaximumAbsoluteChangeMeters : 0,
					Minimax ? Minimax->GetStatistics().MaximumRaiseMeters : 0,
					Minimax ? Minimax->GetStatistics().MaximumCutMeters : 0,
					Minimax ? Minimax->GetStatistics().SumAbsoluteChangeMeters : 0,
					Minimax ? Minimax->GetStatistics().MaximumTieBreakMeters : 0
				)
			);
			// 盛土だけの解より最大補正が悪化しない事を確認する
			if(!TestTrue(TEXT("前置起伏の最大補正最小化を準備"), Minimax.has_value())) return false;
			TestTrue(
				TEXT("前置起伏の最大補正が全盛土以下"),
				Minimax->GetStatistics().MaximumAbsoluteChangeMeters <= Filled->GetStatistics().MaximumRaiseMeters + 1E-6
			);
			AddInfo(
				FString::Printf(
					TEXT("Balanced fill-tree production: changed=%u maxRaise=%.9gm maxCut=%.9gm ")
					TEXT("sumAbsolute=%.9gm minDrop=%.9gm maxIndependentDifference=%.9gm. Landscape unaccepted."),
					Blend->GetStatistics().ChangedLandVertices,
					Blend->GetStatistics().MaximumRaiseMeters,
					Blend->GetStatistics().MaximumCutMeters,
					Blend->GetStatistics().SumAbsoluteChangeMeters,
					Blend->GetStatistics().MinimumLandDropMeters,
					MaximumIndependentDifference
				)
			);
			AddInfo(
				FString::Printf(
					TEXT("Fixed fill-tree cut survey: cutNodes=%u maxCut=%.9gm sumCut=%.9gm newOcean=%u ")
					TEXT("shoreShortages=%u cutBaseline=%u cutReason=%u halfBaseline=%u halfReason=%u. ")
					TEXT("Selected tree and piecewise-linear mesh only; landscape unaccepted."),
					CutNodes,
					MaximumCut,
					SumCut,
					NewlyFlooded,
					ShoreShortages,
					static_cast<unsigned>(CutBaseline.has_value()),
					static_cast<unsigned>(CutError),
					static_cast<unsigned>(MixedBaseline.has_value()),
					static_cast<unsigned>(MixedError)
				)
			);
		}
		std::uint32_t Changed = 0, AboveTenMeters = 0, AboveHundredMeters = 0, PeakIndex = 0;
		double PeakRaise = 0;
		for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
			const double Raise = Filled->GetHeightsMeters()[Index] - Heights[Index];
			if(Raise > 0.01) ++Changed;
			if(Raise > 10) ++AboveTenMeters;
			if(Raise > 100) ++AboveHundredMeters;
			if(Raise <= PeakRaise) continue;
			PeakRaise = Raise;
			PeakIndex = Index;
		}
		const PlanetCore::Vec3 Peak = Mesh->Directions[PeakIndex].Vector();
		std::uint32_t RouteVertices = 0;
		double RouteLengthMeters = 0, HighestOriginalAlongRoute = -std::numeric_limits<double>::infinity();
		for(std::uint32_t Vertex = PeakIndex; RouteVertices < Mesh->Directions.size(); ++RouteVertices) {
			HighestOriginalAlongRoute = std::max(HighestOriginalAlongRoute, Heights[Vertex]);
			const std::uint32_t Parent = Filled->GetDownstreamVertices()[Vertex];
			if(Heights[Vertex] < FillSettings.SeaLevelMeters || Parent == Vertex) break;
			const PlanetCore::Vec3 Difference = Mesh->Directions[Vertex].Vector() - Mesh->Directions[Parent].Vector();
			RouteLengthMeters += FillSettings.RadiusMeters * 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
			Vertex = Parent;
		}
		// 全球より長い経路を排水済みと扱わない返戻
		if(!TestTrue(TEXT("盛土頂点から大洋への有限経路"), RouteVertices < Mesh->Directions.size())) return false;
		const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
		AddInfo(
			FString::Printf(
				TEXT("Predrainage range: mode=%u seed=42 radius=1000000m ocean=%u changed=%u maxRaise=%.9gm ")
				TEXT("sumRaise=%.9gm minimumDrop=%.9gm above10m=%u above100m=%u peakIndex=%u ")
				TEXT("peakOriginal=%.9gm peakDirection=(%.9g,%.9g,%.9g) routeVertices=%u routeKm=%.9g ")
				TEXT("highestOriginalOnRoute=%.9gm seconds=%.9g. Landscape unaccepted."),
				static_cast<unsigned>(Mode),
				Filled->GetStatistics().OceanVertices,
				Changed,
				Filled->GetStatistics().MaximumRaiseMeters,
				Filled->GetStatistics().SumRaiseMeters,
				Filled->GetStatistics().MinimumLandDropMeters,
				AboveTenMeters,
				AboveHundredMeters,
				PeakIndex,
				Heights[PeakIndex],
				Peak.X,
				Peak.Y,
				Peak.Z,
				RouteVertices,
				RouteLengthMeters / 1000,
				HighestOriginalAlongRoute,
				Seconds
			)
		);
	}
	// 前置起伏の最終 H の数値比較を終えた返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetFinalHeightFeedbackSurvey,
    "PlanetGenerator.MinimalDrainageFill.FinalHeightFeedbackSurvey",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 最終 H を排水網へ戻した一回の反復が，閉鎖陸域の補正量を減らすか測る
 * @param Parameters 未使用の UE 引数
 * @return 両地形の準備と診断を実行できたか
 */
bool FPlanetFinalHeightFeedbackSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	PlanetCore::GeologySettings GeologySettings;
	GeologySettings.Seed = 42;
	GeologySettings.RadiusMeters = 1E6;
	const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(GeologySettings);
	// 一回目の地形に必要な地質を準備する
	if(!TestTrue(TEXT("反復比較の地質を準備"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 両地形で同じ接続済み海域を共有する
	if(!TestTrue(TEXT("反復比較の接続大洋を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedInitialDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 一回目の排水網を準備する
	if(!TestTrue(TEXT("反復前の排水網を準備"), PreparedInitialDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> InitialDrainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedInitialDrainage));
	const std::optional<PlanetCore::RiverTerrainField> InitialField =
	PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, InitialDrainage);
	// 侵食後の最終 H を同じ入口で評価する
	if(!TestTrue(TEXT("反復前の侵食地形を準備"), InitialField.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 同一網で前後を比較する
	if(!TestTrue(TEXT("反復比較の球面網を準備"), Mesh.has_value())) return false;
	std::vector<double> InitialHeights;
	InitialHeights.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) {
		InitialHeights.push_back(InitialField->EvaluateElevation(Direction));
	}
	PlanetCore::MinimalDrainageFillSettings FillSettings;
	FillSettings.RadiusMeters = GeologySettings.RadiusMeters;
	FillSettings.SeaLevelMeters = InitialDrainage->Settings().SeaLevelMeters;
	FillSettings.MinimumLandSlope = InitialDrainage->Settings().MinimumSlope;
	PlanetCore::MinimalDrainageFillError InitialFillError = PlanetCore::MinimalDrainageFillError::None;
	const std::optional<PlanetCore::MinimalDrainageFill> InitialFill =
	PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, InitialHeights, FillSettings, InitialFillError);
	// 最初の有限網で必要な事後補正を測る
	if(!TestTrue(TEXT("反復前の最小盛土を準備"), InitialFill.has_value())) return false;
	PlanetCore::SphericalDrainageBaselineError InitialBaselineError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> InitialBaseline = PlanetCore::SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		InitialHeights,
		FillSettings.SeaLevelMeters,
		InitialBaselineError
	);
	std::optional<PlanetCore::DrainageNetwork> PreparedFeedbackDrainage = PlanetCore::DrainageNetwork::CreateForHeight(
		*Ocean,
		[&InitialField](const PlanetCore::UnitDirection Direction) -> double {
			// 初回の最終 H を次の排水網へ入力する
			return InitialField->EvaluateElevation(Direction);
		}
	);
	// 海域定義を保った再経路計算を実行する
	if(!TestTrue(TEXT("最終 H から排水網を再準備"), PreparedFeedbackDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> FeedbackDrainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedFeedbackDrainage));
	const std::optional<PlanetCore::RiverTerrainField> FeedbackField =
	PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, FeedbackDrainage);
	// 再経路後も通常の侵食地形を作る
	if(!TestTrue(TEXT("反復後の侵食地形を準備"), FeedbackField.has_value())) return false;
	std::vector<double> FeedbackHeights;
	FeedbackHeights.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) {
		FeedbackHeights.push_back(FeedbackField->EvaluateElevation(Direction));
	}
	PlanetCore::MinimalDrainageFillError FeedbackFillError = PlanetCore::MinimalDrainageFillError::None;
	const std::optional<PlanetCore::MinimalDrainageFill> FeedbackFill =
	PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, FeedbackHeights, FillSettings, FeedbackFillError);
	// 一回の反復後の閉鎖陸域も同じ測定器で評価する
	if(!TestTrue(TEXT("反復後の最小盛土を準備"), FeedbackFill.has_value())) return false;
	PlanetCore::SphericalDrainageBaselineError FeedbackBaselineError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> FeedbackBaseline = PlanetCore::SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		FeedbackHeights,
		FillSettings.SeaLevelMeters,
		FeedbackBaselineError
	);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Final-height feedback survey: seed=42 radius=1000000m initialBaseline=%u initialReason=%u ")
			TEXT("initialMaxRaise=%.9gm initialSumRaise=%.9gm initialChanged=%u ")
			TEXT("feedbackBaseline=%u feedbackReason=%u feedbackMaxRaise=%.9gm ")
			TEXT("feedbackSumRaise=%.9gm feedbackChanged=%u seconds=%.9g. Landscape unaccepted."),
			static_cast<unsigned>(InitialBaseline.has_value()),
			static_cast<unsigned>(InitialBaselineError),
			InitialFill->GetStatistics().MaximumRaiseMeters,
			InitialFill->GetStatistics().SumRaiseMeters,
			InitialFill->GetStatistics().RaisedLandVertices,
			static_cast<unsigned>(FeedbackBaseline.has_value()),
			static_cast<unsigned>(FeedbackBaselineError),
			FeedbackFill->GetStatistics().MaximumRaiseMeters,
			FeedbackFill->GetStatistics().SumRaiseMeters,
			FeedbackFill->GetStatistics().RaisedLandVertices,
			Seconds
		)
	);
	// 反復の効果は測定値で判断し，改善を試験成功の条件にしない
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetFinalHeightStageDrainageSurvey,
    "PlanetGenerator.MinimalDrainageFill.FinalHeightStageSurvey",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実際の河川地形生成の各段階で閉鎖陸域がいつ増えるか測る
 * @param Parameters 未使用の UE 引数
 * @return 各段階を同一球面網で評価できたか
 */
bool FPlanetFinalHeightStageDrainageSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	PlanetCore::GeologySettings GeologySettings;
	GeologySettings.Seed = 42;
	GeologySettings.RadiusMeters = 1E6;
	const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(GeologySettings);
	// 実際の地形経路と同じ地質を準備する
	if(!TestTrue(TEXT("段階別比較の地質を準備"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 大陸と海域を固定して比較する
	if(!TestTrue(TEXT("段階別比較の接続大洋を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 本番と同じ排水設定を使用する
	if(!TestTrue(TEXT("段階別比較の排水網を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
	const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	// 谷と細部起伏を含む最終 H の生成器を準備する
	if(!TestTrue(TEXT("段階別比較の河川地形を準備"), Field.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 全段階で同一網を使用する
	if(!TestTrue(TEXT("段階別比較の球面網を準備"), Mesh.has_value())) return false;
	std::array<std::vector<double>, 5> Heights;
	for(std::vector<double> &Stage : Heights) Stage.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) {
		const PlanetCore::RiverElevationStages Values = Field->EvaluateElevationStages(Direction);
		Heights[0].push_back(Values.BeforeBasinMeters);
		Heights[1].push_back(Values.AfterBasinMeters);
		Heights[2].push_back(Values.AfterValleyMeters);
		Heights[3].push_back(Values.AfterFineReliefMeters);
		Heights[4].push_back(Values.FinalMeters);
	}
	const std::array<const TCHAR *, 5> Names {
		TEXT("BeforeBasin"),
		TEXT("AfterBasin"),
		TEXT("AfterValley"),
		TEXT("AfterFineRelief"),
		TEXT("Final")
	};
	PlanetCore::MinimalDrainageFillSettings FillSettings;
	FillSettings.RadiusMeters = GeologySettings.RadiusMeters;
	FillSettings.SeaLevelMeters = Drainage->Settings().SeaLevelMeters;
	FillSettings.MinimumLandSlope = Drainage->Settings().MinimumSlope;
	for(std::size_t Stage = 0; Stage < Heights.size(); ++Stage) {
		PlanetCore::SphericalDrainageBaselineError BaselineError = PlanetCore::SphericalDrainageBaselineError::None;
		const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
			Mesh->Directions,
			Mesh->Triangles,
			Heights[Stage],
			FillSettings.SeaLevelMeters,
			BaselineError
		);
		PlanetCore::MinimalDrainageFillError FillError = PlanetCore::MinimalDrainageFillError::None;
		const std::optional<PlanetCore::MinimalDrainageFill> Filled =
		PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Heights[Stage], FillSettings, FillError);
		AddInfo(
			FString::Printf(
				TEXT("Final-height stage drainage: stage=%s baseline=%u baselineReason=%u ")
				TEXT("fillReady=%u fillReason=%u changed=%u maxRaise=%.9gm sumRaise=%.9gm. ")
				TEXT("Finite-mesh diagnostic only."),
				Names[Stage],
				static_cast<unsigned>(Baseline.has_value()),
				static_cast<unsigned>(BaselineError),
				static_cast<unsigned>(Filled.has_value()),
				static_cast<unsigned>(FillError),
				Filled ? Filled->GetStatistics().RaisedLandVertices : 0,
				Filled ? Filled->GetStatistics().MaximumRaiseMeters : 0,
				Filled ? Filled->GetStatistics().SumRaiseMeters : 0
			)
		);
		// 比較する全段階で大洋根の補正解を用意する
		if(!TestTrue(TEXT("各標高段階の最小盛土を準備"), Filled.has_value())) return false;
	}
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(FString::Printf(TEXT("Final-height stage survey seconds=%.9g. Landscape unaccepted."), Seconds));
	// 各段階の原因特定は記録した差から判断する
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetSharedPredrainageSurvey,
    "PlanetGenerator.MinimalDrainageFill.SharedPredrainageSurvey",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 排水網と共通の球面網で中規模起伏を前置した場合の費用と残余窪地を測る
 * @param Parameters 未使用の UE 引数
 * @return 共通網の生成と全球排水診断を実行できたか
 */
bool FPlanetSharedPredrainageSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = 1E6;
	const std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Definition);
	// 同じ惑星定義の地質を準備する
	if(!TestTrue(TEXT("共有網比較の地質を準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*PreparedGeology));
	std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 前置場と同じ造山履歴を用意する
	if(!TestTrue(TEXT("共有網比較の造山を準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*PreparedRidges));
	const double PreOceanSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	const std::chrono::steady_clock::time_point OceanStarted = std::chrono::steady_clock::now();
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		Ridges,
		{},
		PlanetCore::PredrainageReliefMode::MesoOnly,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 共通網でも連結した大洋を準備する
	if(!TestTrue(TEXT("共有網の前置中規模起伏大洋を準備"), PreparedOcean.has_value())) return false;
	const double OceanSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - OceanStarted).count();
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	TestTrue(TEXT("共有網に前置中規模起伏を含む"), Ocean->HasPredrainageMesoRelief());
	TestEqual(TEXT("共有網の大洋は一成分"), Ocean->Statistics().ComponentsAfter, 1U);
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 前置起伏を読んだ排水網を準備する
	if(!TestTrue(TEXT("共有網の排水網を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
	const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	// 侵食後の最終標高で比較する
	if(!TestTrue(TEXT("共有網の侵食地形を準備"), Field.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 排水網と同じ全球網を測定する
	if(!TestTrue(TEXT("共有網の球面格子を準備"), Mesh.has_value())) return false;
	std::vector<double> Heights;
	Heights.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) Heights.push_back(Field->EvaluateElevation(Direction));
	PlanetCore::MinimalDrainageFillSettings FillSettings;
	FillSettings.RadiusMeters = Definition.RadiusMeters;
	FillSettings.SeaLevelMeters = Drainage->Settings().SeaLevelMeters;
	FillSettings.MinimumLandSlope = Drainage->Settings().MinimumSlope;
	PlanetCore::MinimalDrainageFillError FillError = PlanetCore::MinimalDrainageFillError::None;
	const std::optional<PlanetCore::MinimalDrainageFill> Filled =
	PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Heights, FillSettings, FillError);
	// 残る閉鎖陸域の補正量を同じ方法で測る
	if(!TestTrue(TEXT("共有網の最小盛土を準備"), Filled.has_value())) return false;
	PlanetCore::SphericalDrainageBaselineError BaselineError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		Heights,
		FillSettings.SeaLevelMeters,
		BaselineError
	);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Shared predrainage survey: seed=42 radius=1000000m graph=%u components=%u ")
			TEXT("preOceanSeconds=%.9g oceanSeconds=%.9g totalSeconds=%.9g ")
			TEXT("baseline=%u baselineReason=%u oceanVertices=%u changed=%u ")
			TEXT("maxRaise=%.9gm sumRaise=%.9gm. Landscape unaccepted."),
			Ocean->Statistics().GraphNodes,
			Ocean->Statistics().ComponentsAfter,
			PreOceanSeconds,
			OceanSeconds,
			Seconds,
			static_cast<unsigned>(Baseline.has_value()),
			static_cast<unsigned>(BaselineError),
			Filled->GetStatistics().OceanVertices,
			Filled->GetStatistics().RaisedLandVertices,
			Filled->GetStatistics().MaximumRaiseMeters,
			Filled->GetStatistics().SumRaiseMeters
		)
	);
	// 共通網の比較は数値診断であり，景観の合格ではない
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetSharedPredrainageRange,
    "PlanetGenerator.MinimalDrainageFill.SharedPredrainageRange",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 中規模起伏を前置した共通網を複数 Seed と半径で検査する
 * @param Parameters 未使用の UE 引数
 * @return 大洋連結，海陸比と有限網診断を実行できたか
 */
bool FPlanetSharedPredrainageRange::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	struct Case {
		std::uint64_t Seed;
		double RadiusMeters;
		PlanetCore::PredrainageReliefMode Mode;
	};
	using Mode = PlanetCore::PredrainageReliefMode;
	const std::array<Case, 7> Cases {
		{
			{ 7, 1E6, Mode::MesoOnly },
			{ 44, 1E6, Mode::MesoOnly },
			{ 42, 1E3, Mode::MesoOnly },
			{ 42, 1E9, Mode::MesoOnly },
			{ 7, 1E6, Mode::MesoAndLand },
			{ 44, 1E6, Mode::MesoAndLand },
			{ 42, 1E6, Mode::MesoAndLand }
		}
	};
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 全条件で同じ閉じた全球網を測定する
	if(!TestTrue(TEXT("共通網範囲比較の球面網を準備"), Mesh.has_value())) return false;
	for(const Case Current : Cases) {
		const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
		PlanetCore::GeologySettings Definition;
		Definition.Seed = Current.Seed;
		Definition.RadiusMeters = Current.RadiusMeters;
		const std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Definition);
		// 惑星定義ごとの地質を用意する
		if(!TestTrue(TEXT("共通網範囲比較の地質を準備"), PreparedGeology.has_value())) return false;
		const std::shared_ptr<const PlanetCore::GeologyField> Geology =
		std::make_shared<const PlanetCore::GeologyField>(std::move(*PreparedGeology));
		std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
		// 前置地形と同じ地質履歴の造山を用意する
		if(!TestTrue(TEXT("共通網範囲比較の造山を準備"), PreparedRidges.has_value())) return false;
		const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*PreparedRidges));
		std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
		PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
			Geology,
			Ridges,
			{},
			Current.Mode,
			PlanetCore::OceanSamplingMode::SharedDrainage
		);
		// 大洋連結に失敗した条件を隠さない
		if(!TestTrue(TEXT("共通網範囲比較の大洋を準備"), PreparedOcean.has_value())) return false;
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
		std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
		TestEqual(TEXT("共通網範囲比較の大洋一成分"), Ocean->Statistics().ComponentsAfter, 1U);
		const double AreaFraction = Ocean->Statistics().GraphOceanAreaFraction;
		const double IndependentFraction = Ocean->Statistics().IndependentOceanFractionEstimate;
		TestTrue(TEXT("共通網範囲比較の海域面積比"), AreaFraction >= 0.6 && AreaFraction <= 0.7);
		TestTrue(TEXT("共通網範囲比較の独立海域推定"), IndependentFraction >= 0.6 && IndependentFraction <= 0.7);
		std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
		// 前置地形の排水網を作る
		if(!TestTrue(TEXT("共通網範囲比較の排水網を準備"), PreparedDrainage.has_value())) return false;
		const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
		std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
		const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
		// 本番と同じ侵食後地形を測る
		if(!TestTrue(TEXT("共通網範囲比較の最終地形を準備"), Field.has_value())) return false;
		const std::span<const PlanetCore::DrainageNode> Nodes = Field->Drainage().Nodes();
		std::vector<std::uint8_t> Visit(Nodes.size());
		std::vector<std::uint32_t> Path;
		bool AllNodesReachOcean = true;
		for(std::uint32_t Start = 0; Start < Nodes.size(); ++Start) {
			if(Visit[Start]) continue;
			std::uint32_t CurrentNode = Start;
			Path.clear();
			while(CurrentNode < Nodes.size() && !Nodes[CurrentNode].Ocean && !Visit[CurrentNode]) {
				Visit[CurrentNode] = 1;
				Path.push_back(CurrentNode);
				const std::uint32_t Downstream = Nodes[CurrentNode].Downstream;
				if(Downstream >= Nodes.size() || Nodes[CurrentNode].WaterMeters <= Nodes[Downstream].WaterMeters) {
					AllNodesReachOcean = false;
					break;
				}
				CurrentNode = Downstream;
			}
			if(!AllNodesReachOcean || CurrentNode >= Nodes.size() || Visit[CurrentNode] == 1) {
				AllNodesReachOcean = false;
				break;
			}
			for(const std::uint32_t Node : Path) Visit[Node] = 2;
			if(Nodes[CurrentNode].Ocean) Visit[CurrentNode] = 2;
		}
		// 侵食後にも全節点の水位が下降し，経路が循環せず大洋へ着く事を照合する
		if(!TestTrue(TEXT("侵食後の全排水節点が大洋へ下降"), AllNodesReachOcean)) return false;
		std::vector<bool> Selected(Nodes.size());
		bool SelectedReachesConnected = true;
		for(const PlanetCore::RiverReach &Reach : Field->Drainage().Reaches()) {
			if(Reach.Upstream >= Nodes.size() || Reach.Downstream >= Nodes.size()) {
				SelectedReachesConnected = false;
				break;
			}
			Selected[Reach.Upstream] = true;
		}
		if(SelectedReachesConnected) for(const PlanetCore::RiverReach &Reach : Field->Drainage().Reaches()) {
			if(Nodes[Reach.Downstream].Ocean || Selected[Reach.Downstream]) continue;
			SelectedReachesConnected = false;
			break;
		}
		// 表示対象に選ばれた河道だけが内陸で途切れる場合も拒否する
		if(!TestTrue(TEXT("侵食後の選択河道が海又は次の河道へ接続"), SelectedReachesConnected)) return false;
		bool SharedIndices = Nodes.size() >= Mesh->Directions.size();
		for(std::size_t Index = 0; SharedIndices && Index < Mesh->Directions.size(); ++Index) {
			SharedIndices = (Nodes[Index].Direction - Mesh->Directions[Index].Vector()).Length() < 1E-12;
		}
		// 異なる地点の河道番号を同じ節点として集計しない返戻
		if(!TestTrue(TEXT("共有球面網の節点番号が侵食後の排水網と一致"), SharedIndices)) return false;
		std::vector<double> Heights;
		Heights.reserve(Mesh->Directions.size());
		for(const PlanetCore::UnitDirection Direction : Mesh->Directions) Heights.push_back(Field->EvaluateElevation(Direction));
		PlanetCore::MinimalDrainageFillSettings FillSettings;
		FillSettings.RadiusMeters = Current.RadiusMeters;
		FillSettings.SeaLevelMeters = Drainage->Settings().SeaLevelMeters;
		FillSettings.MinimumLandSlope = Drainage->Settings().MinimumSlope;
		PlanetCore::MinimalDrainageFillError FillError = PlanetCore::MinimalDrainageFillError::None;
		const std::optional<PlanetCore::MinimalDrainageFill> Filled =
		PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Heights, FillSettings, FillError);
		// 連結大洋を出口とする有限網の残余補正を測る
		if(!TestTrue(TEXT("共通網範囲比較の最小盛土を準備"), Filled.has_value())) return false;
		std::vector<std::vector<std::uint32_t>> Neighbors(Mesh->Directions.size());
		for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			Neighbors[A].push_back(B);
			Neighbors[B].push_back(A);
		}
		std::vector<bool> Raised(Mesh->Directions.size()), Seen(Mesh->Directions.size());
		for(std::size_t Index = 0; Index < Raised.size(); ++Index) {
			Raised[Index] = Filled->GetHeightsMeters()[Index] > Heights[Index] + 0.000001;
		}
		std::uint32_t RaisedComponents = 0, DeepestComponentVertices = 0, DeepestSelectedVertices = 0, DeepestNode = 0;
		double DeepestComponentRaise = 0;
		std::vector<std::uint32_t> Queue;
		for(std::uint32_t StartNode = 0; StartNode < Raised.size(); ++StartNode) {
			if(!Raised[StartNode] || Seen[StartNode]) continue;
			++RaisedComponents;
			Queue.clear();
			Queue.push_back(StartNode);
			Seen[StartNode] = true;
			std::uint32_t ComponentSelected = 0;
			double ComponentMaximum = 0;
			std::uint32_t ComponentMaximumNode = StartNode;
			for(std::size_t Position = 0; Position < Queue.size(); ++Position) {
				const std::uint32_t Node = Queue[Position];
				if(Selected[Node]) ++ComponentSelected;
				const double Raise = Filled->GetHeightsMeters()[Node] - Heights[Node];
				if(Raise > ComponentMaximum) {
					ComponentMaximum = Raise;
					ComponentMaximumNode = Node;
				}
				for(const std::uint32_t Next : Neighbors[Node]) {
					if(!Raised[Next] || Seen[Next]) continue;
					Seen[Next] = true;
					Queue.push_back(Next);
				}
			}
			if(ComponentMaximum <= DeepestComponentRaise) continue;
			DeepestComponentRaise = ComponentMaximum;
			DeepestComponentVertices = static_cast<std::uint32_t>(Queue.size());
			DeepestSelectedVertices = ComponentSelected;
			DeepestNode = ComponentMaximumNode;
		}
		std::uint32_t RouteSteps = 0, FirstReceiverKind = 2;
		double RouteDistanceMeters = 0, FirstReceiverDistanceMeters = -1, MaximumRouteBarrierMeters = 0;
		double BarrierBeforeReceiverMeters = 0, FirstReceiverWaterMeters = 0, FirstReceiverGroundMeters = 0;
		std::uint32_t RouteNode = DeepestNode;
		const double SourceHeightMeters = Heights[DeepestNode];
		const std::span<const std::uint32_t> Tree = Filled->GetDownstreamVertices();
		for(; RouteSteps <= Mesh->Directions.size(); ++RouteSteps) {
			MaximumRouteBarrierMeters = std::max(MaximumRouteBarrierMeters, Heights[RouteNode] - SourceHeightMeters);
			if(FirstReceiverDistanceMeters < 0) {
				BarrierBeforeReceiverMeters = std::max(BarrierBeforeReceiverMeters, Heights[RouteNode] - SourceHeightMeters);
			}
			if(FirstReceiverDistanceMeters < 0 && (Selected[RouteNode] || Heights[RouteNode] < FillSettings.SeaLevelMeters)) {
				FirstReceiverKind = Selected[RouteNode] ? 1 : 0;
				FirstReceiverDistanceMeters = RouteDistanceMeters;
				FirstReceiverGroundMeters = Heights[RouteNode];
				FirstReceiverWaterMeters = Nodes[RouteNode].Ocean ? FillSettings.SeaLevelMeters : Nodes[RouteNode].WaterMeters;
			}
			if(Heights[RouteNode] < FillSettings.SeaLevelMeters) break;
			const std::uint32_t Next = Tree[RouteNode];
			if(Next == RouteNode || Next >= Tree.size()) break;
			const PlanetCore::Vec3 A = Mesh->Directions[RouteNode].Vector(), B = Mesh->Directions[Next].Vector();
			RouteDistanceMeters += Current.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
			RouteNode = Next;
		}
		// 固定木の不正を検査失敗として残す
		TestTrue(
			TEXT("粗網の最大補正地点が有限な大洋根経路を持つ"),
			RouteSteps < Mesh->Directions.size() && Heights[RouteNode] < FillSettings.SeaLevelMeters
		);
		// 地質面を固定した場合に，別の選択河道へ繋ぐ低障壁の粗網経路を探す
		const double Infinity = std::numeric_limits<double>::infinity();
		std::vector<double> BestBarrier(Heights.size(), Infinity), BestLength(Heights.size(), Infinity);
		using FrontierItem = std::tuple<double, double, std::uint32_t>;
		std::priority_queue<FrontierItem, std::vector<FrontierItem>, std::greater<>> Frontier;
		BestBarrier[DeepestNode] = 0;
		BestLength[DeepestNode] = 0;
		Frontier.emplace(0, 0, DeepestNode);
		std::uint32_t AlternativeReceiver = static_cast<std::uint32_t>(Heights.size()), AlternativeSettled = 0;
		while(!Frontier.empty()) {
			const auto [Barrier, Length, Node] = Frontier.top();
			Frontier.pop();
			if(Barrier != BestBarrier[Node] || Length != BestLength[Node]) continue;
			++AlternativeSettled;
			const bool OceanNode = Heights[Node] < FillSettings.SeaLevelMeters;
			const double ReceiverWater = OceanNode ? FillSettings.SeaLevelMeters : Nodes[Node].WaterMeters;
			if(Node != DeepestNode && (OceanNode || Selected[Node]) && ReceiverWater < SourceHeightMeters) {
				AlternativeReceiver = Node;
				break;
			}
			for(const std::uint32_t Next : Neighbors[Node]) {
				const double NewBarrier = std::max(Barrier, Heights[Next] - SourceHeightMeters);
				const PlanetCore::Vec3 A = Mesh->Directions[Node].Vector(), B = Mesh->Directions[Next].Vector();
				const double NewLength = Length + Current.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
				if(std::tie(NewBarrier, NewLength) >= std::tie(BestBarrier[Next], BestLength[Next])) continue;
				BestBarrier[Next] = NewBarrier;
				BestLength[Next] = NewLength;
				Frontier.emplace(NewBarrier, NewLength, Next);
			}
		}
		// 大域費用の探索失敗は補助出口を採用出来ない検査結果として残す
		TestTrue(TEXT("許容水位の受入河川又は大洋への粗網経路を発見"), AlternativeReceiver < Heights.size());
		const double AlternativeBarrierMeters = AlternativeReceiver < Heights.size() ? BestBarrier[AlternativeReceiver] : -1;
		const double AlternativeLengthMeters = AlternativeReceiver < Heights.size() ? BestLength[AlternativeReceiver] : -1;
		const double AlternativeWaterMeters = AlternativeReceiver < Heights.size() ?
		Heights[AlternativeReceiver] < FillSettings.SeaLevelMeters ?
		FillSettings.SeaLevelMeters :
		Nodes[AlternativeReceiver].WaterMeters :
		0;
		unsigned OriginTraceEnd = 255, OriginTraceSamples = 0, OriginCriticalKind = 255, OriginCriticalFailure = 0;
		std::optional<PlanetCore::TerrainCriticalPoint> OriginMinimum;
		double OriginTraceLengthMeters = -1, OriginTraceHeightMeters = 0, OriginCriticalHeightMeters = 0;
		double OriginCriticalDistanceMeters = -1;
		bool OriginTraceDescending = true;
		if(Current.RadiusMeters == 1E6) {
			PlanetCore::TerrainDescentSettings TraceSettings;
			TraceSettings.RadiusMeters = Current.RadiusMeters;
			TraceSettings.StopElevationMeters = FillSettings.SeaLevelMeters;
			TraceSettings.GradientTolerance = 1E-8;
			TraceSettings.PositionToleranceMeters = 0.001;
			TraceSettings.MaximumStepMeters = 250;
			TraceSettings.MaximumLengthMeters = 50000;
			TraceSettings.MaximumVertices = 8192;
			const auto Evaluate = [&Field](const PlanetCore::UnitDirection Direction) {
				return Field->Evaluate(Direction);
			};
			const std::optional<PlanetCore::TerrainDescentPath> Trace =
			PlanetCore::TraceTerrainDescent(Mesh->Directions[DeepestNode], Evaluate, TraceSettings);
			if(Trace && !Trace->Samples.empty()) {
				OriginTraceEnd = static_cast<unsigned>(Trace->End);
				OriginTraceSamples = static_cast<unsigned>(Trace->Samples.size());
				OriginTraceLengthMeters = Trace->Samples.back().DistanceMeters;
				OriginTraceHeightMeters = Trace->Samples.back().ElevationMeters;
				for(std::size_t Index = 1; Index < Trace->Samples.size(); ++Index) {
					if(Trace->Samples[Index].ElevationMeters > Trace->Samples[Index - 1].ElevationMeters + 0.000001) OriginTraceDescending = false;
				}
				PlanetCore::TerrainCriticalSettings Settings;
				Settings.RadiusMeters = Current.RadiusMeters;
				Settings.MaximumStepMeters = 10000;
				Settings.MaximumDisplacementMeters = 100000;
				PlanetCore::TerrainCriticalDiagnostics Diagnostics;
				const std::optional<PlanetCore::TerrainCriticalPoint> Point =
				PlanetCore::RefineTerrainCriticalPoint(Trace->Samples.back().Direction, Evaluate, Settings, {}, &Diagnostics);
				OriginCriticalFailure = static_cast<unsigned>(Diagnostics.Failure);
				if(Point) {
					OriginCriticalKind = static_cast<unsigned>(Point->Kind);
					OriginCriticalHeightMeters = Point->ElevationMeters;
					if(Point->Kind == PlanetCore::TerrainCriticalKind::Minimum && Point->ElevationMeters <= SourceHeightMeters + 0.000001) {
						OriginMinimum = Point;
					}
					const PlanetCore::Vec3 A = Mesh->Directions[DeepestNode].Vector(), B = Point->Direction.Vector();
					OriginCriticalDistanceMeters = Current.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
				}
			}
			// 下降履歴が増加した場合は流域同一性を受理しない
			TestTrue(TEXT("元格子点からの解析下降経路が上り返さない"), OriginTraceDescending);
			TestTrue(TEXT("元格子点の下降終端がより低い実極小"), OriginMinimum.has_value());
		}
		int CriticalKind = -1;
		unsigned CriticalFailure = 0;
		double CriticalDisplacementMeters = 0;
		unsigned NearbyMinima = 0, NearbySaddles = 0, NearbyMaxima = 0, NearbyRejected = 0;
		std::optional<PlanetCore::TerrainCriticalPoint> NearestMinimum;
		double NearestMinimumDistanceMeters = -1, NearestMinimumHeightMeters = 0;
		if(Current.RadiusMeters == 1E6 && RaisedComponents) {
			PlanetCore::TerrainCriticalSettings CriticalSettings;
			CriticalSettings.RadiusMeters = Current.RadiusMeters;
			CriticalSettings.MaximumStepMeters = 10000;
			CriticalSettings.MaximumDisplacementMeters = 100000;
			const PlanetCore::UnitDirection Origin = Mesh->Directions[DeepestNode];
			const PlanetCore::Vec3 Center = Origin.Vector();
			const PlanetCore::Vec3 Reference = std::abs(Center.Z) < 0.9 ? PlanetCore::Vec3{ 0, 0, 1 } : PlanetCore::Vec3{ 1, 0, 0 };
			const PlanetCore::Vec3 East = PlanetCore::UnitDirection::FromVector(Reference.Cross(Center))->Vector();
			const PlanetCore::Vec3 North = Center.Cross(East);
			const auto Evaluate = [&Field](const PlanetCore::UnitDirection Direction) {
				// 極値の探索と最終地形の同じ解析場を評価する
				return Field->Evaluate(Direction);
			};
			PlanetCore::TerrainCriticalDiagnostics CriticalDiagnostics;
			const std::optional<PlanetCore::TerrainCriticalPoint> Critical =
			PlanetCore::RefineTerrainCriticalPoint(Origin, Evaluate, CriticalSettings, {}, &CriticalDiagnostics);
			CriticalKind = Critical ? static_cast<int>(Critical->Kind) : -1;
			CriticalFailure = static_cast<unsigned>(CriticalDiagnostics.Failure);
			CriticalDisplacementMeters = CriticalDiagnostics.DisplacementMeters;
			const auto InspectCritical = [&](const std::optional<PlanetCore::TerrainCriticalPoint> &Point) {
				if(!Point) ++NearbyRejected;
				else if(Point->Kind == PlanetCore::TerrainCriticalKind::Saddle) ++NearbySaddles;
				else if(Point->Kind == PlanetCore::TerrainCriticalKind::Maximum) ++NearbyMaxima;
				else if(Point->Kind == PlanetCore::TerrainCriticalKind::Minimum) {
					++NearbyMinima;
					const double Distance =
					Current.RadiusMeters * std::atan2(Center.Cross(Point->Direction.Vector()).Length(), Center.Dot(Point->Direction.Vector()));
					if(NearestMinimumDistanceMeters < 0 || Distance < NearestMinimumDistanceMeters) {
						NearestMinimumDistanceMeters = Distance;
						NearestMinimumHeightMeters = Point->ElevationMeters;
						NearestMinimum = Point;
					}
				}
			};
			InspectCritical(Critical);
			for(const double Distance : { 2000.0, 8000.0, 25000.0 }) for(unsigned Bearing = 0; Bearing < 8; ++Bearing) {
				const double Angle = 2 * std::numbers::pi * static_cast<double>(Bearing) / 8;
				const PlanetCore::Vec3 Tangent = East * std::cos(Angle) + North * std::sin(Angle);
				const PlanetCore::UnitDirection Start =
				*PlanetCore::UnitDirection::FromVector(Center + Tangent * (Distance / Current.RadiusMeters));
				InspectCritical(PlanetCore::RefineTerrainCriticalPoint(Start, Evaluate, CriticalSettings));
			}
		}
		unsigned OutletFailure = 0, OutletWindows = 0;
		std::uint64_t OutletEvaluations = 0;
		double OutletPassHeightMeters = -1;
		unsigned SecondOutletFailure = 0, SecondOutletWindows = 0;
		std::uint64_t SecondOutletEvaluations = 0;
		double SecondOutletPassHeightMeters = -1;
		unsigned SecondOutletBranchKinds[2] = { 255, 255 };
		double SecondOutletBranchHeightsMeters[2] = { 0, 0 }, SecondOutletBranchKnownDistancesMeters[2] = { -1, -1 };
		unsigned CorrectOriginNetworkFailure = 255, CorrectOriginNetworkSteps = 0, CorrectOriginNetworkMinima = 0;
		unsigned CorrectOriginNetworkConnections = 0, CorrectOriginNetworkCenterQueries = 0;
		std::uint64_t CorrectOriginNetworkEvaluations = 0;
		double CorrectOriginNetworkOceanDistanceMeters = -1;
		unsigned OutletBranchKinds[2] = { 255, 255 };
		double OutletBranchHeightsMeters[2] = { 0, 0 }, OutletBranchOriginDistancesMeters[2] = { -1, -1 };
		const bool InspectOriginOutlet = Current.Mode == Mode::MesoOnly && (Current.Seed == 7 || Current.Seed == 44) ||
		Current.Mode == Mode::MesoAndLand && Current.Seed == 42;
		if(Current.RadiusMeters == 1E6 && InspectOriginOutlet && OriginMinimum) {
			PlanetCore::RiverReceiverError ReceiverError = PlanetCore::RiverReceiverError::None;
			const std::optional<PlanetCore::RiverReceiverIndex> Receivers =
			PlanetCore::RiverReceiverIndex::Create(*Field, 16, ReceiverError);
			// 大洋へ下降しない網又は実水位の無い河道を受理しない返戻
			if(!TestTrue(TEXT("全節点が大洋へ下降する実水位河川の受入候補を準備"), Receivers.has_value())) return false;
			const std::optional<PlanetCore::RiverReceiverNearest> NearestRiver =
			Receivers->FindNearest(OriginMinimum->Direction, std::numeric_limits<double>::infinity());
			// 有効な受入河川の無い惑星を成功としない返戻
			if(!TestTrue(TEXT("元窪地からの実水位付き既存河川を発見"), NearestRiver.has_value())) return false;
			TestFalse(
				TEXT("海面未満の水位上限では陸上の受入河川を返さない"),
				Receivers->FindNearest(OriginMinimum->Direction, -std::numeric_limits<double>::infinity()).has_value()
			);
			TestFalse(
				TEXT("元窪地と重ならない距離零の受入候補を作らない"),
				Receivers->FindNearest(OriginMinimum->Direction, std::numeric_limits<double>::infinity(), 0).has_value()
			);
			if(Current.Seed == 7) {
				PlanetCore::RiverReceiverError RejectedError = PlanetCore::RiverReceiverError::None;
				TestFalse(TEXT("零区分の河道標本を拒否"), PlanetCore::RiverReceiverIndex::Create(*Field, 0, RejectedError).has_value());
				TestTrue(TEXT("無効な標本数の理由"), RejectedError == PlanetCore::RiverReceiverError::InvalidInput);
				std::stop_source CancelReceiver;
				CancelReceiver.request_stop();
				TestFalse(
					TEXT("受入候補の準備中断は部分結果を返さない"),
					PlanetCore::RiverReceiverIndex::Create(*Field, 16, RejectedError, CancelReceiver.get_token()).has_value()
				);
				TestTrue(TEXT("中断理由を区別"), RejectedError == PlanetCore::RiverReceiverError::Cancelled);
				TestFalse(
					TEXT("非支配受入候補の中断は部分集合を公開しない"),
					Receivers->FindDistanceWaterPareto(
						OriginMinimum->Direction,
						std::numeric_limits<double>::infinity(),
						std::numeric_limits<double>::infinity(),
						CancelReceiver.get_token()
					).has_value()
				);
				TestFalse(
					TEXT("非支配受入候補の無効な水位上限を拒否"),
					Receivers->FindDistanceWaterPareto(OriginMinimum->Direction, std::numeric_limits<double>::quiet_NaN()).has_value()
				);
				TestFalse(
					TEXT("受入候補の照会中断は候補を返さない"),
					Receivers->FindNearest(
						OriginMinimum->Direction,
						std::numeric_limits<double>::infinity(),
						std::numeric_limits<double>::infinity(),
						CancelReceiver.get_token()
					).has_value()
				);
			}
			AddInfo(
				FString::Printf(
					TEXT("Nearest connected river receiver: seed=%llu reach=%u sampled=%u supported=%u rejected=%u ")
					TEXT("distance=%.9gm water=%.9gm ground=%.9gm support=%.9g. Landscape unaccepted."),
					static_cast<unsigned long long>(Current.Seed),
					NearestRiver->Site.Reach,
					Receivers->GetStatistics().Sampled,
					Receivers->GetStatistics().Supported,
					Receivers->GetStatistics().Rejected,
					NearestRiver->DistanceMeters,
					NearestRiver->Site.WaterMeters,
					NearestRiver->Site.GroundMeters,
					NearestRiver->Site.WaterSupport
				)
			);
			PlanetCore::SpillDiscoverySettings OutletSettings;
			OutletSettings.RadiusMeters = Current.RadiusMeters;
			OutletSettings.SeaLevelMeters = FillSettings.SeaLevelMeters;
			OutletSettings.InitialHalfWidthMeters = 1024;
			OutletSettings.MaximumHalfWidthMeters = 16384;
			OutletSettings.HalfCells = 32;
			OutletSettings.CandidateSpacingMeters = 100;
			OutletSettings.MaximumEvaluations = 100000;
			OutletSettings.MaximumConnections = 32;
			OutletSettings.MaximumKnownMinima = 32;
			OutletSettings.MaximumBranchSamples = 16384;
			OutletSettings.MaximumBridgeCorridors = 2;
			PlanetCore::SpillDiscoveryDiagnostics OutletDiagnostics;
			const std::optional<PlanetCore::SpillConnection> Outlet = PlanetCore::FindSpillComponentExit(
				std::span<const PlanetCore::TerrainCriticalPoint>(&*OriginMinimum, 1),
				[&Field](const PlanetCore::UnitDirection Direction) {
					return Field->Evaluate(Direction);
				},
				OutletSettings,
				{},
				&OutletDiagnostics
			);
			OutletFailure = static_cast<unsigned>(OutletDiagnostics.Failure);
			OutletWindows = OutletDiagnostics.Windows;
			OutletEvaluations = OutletDiagnostics.Evaluations;
			if(Outlet) {
				OutletPassHeightMeters = Outlet->Pass.ElevationMeters;
				const std::optional<PlanetCore::RiverReceiverNearest> FeasibleReceiver =
				Receivers->FindNearest(OriginMinimum->Direction, OutletPassHeightMeters);
				TestTrue(TEXT("元窪地の初回峠以下の実水位受入河川を発見"), FeasibleReceiver.has_value());
				if(FeasibleReceiver && Current.Seed == 7 && Current.Mode == Mode::MesoOnly) {
					const std::optional<std::vector<PlanetCore::RiverReceiverNearest>> Pareto =
					Receivers->FindDistanceWaterPareto(OriginMinimum->Direction, OutletPassHeightMeters);
					if(TestTrue(TEXT("距離と水位の非支配受入候補を列挙"), Pareto.has_value() && !Pareto->empty())) {
						TestTrue(TEXT("非支配集合の最近点が独立な最近傍と一致"), std::abs(Pareto->front().DistanceMeters - FeasibleReceiver->DistanceMeters) < 1E-6);
						for(std::size_t i = 1; i < Pareto->size(); ++i) {
							TestTrue(TEXT("非支配候補の距離が増加"), (*Pareto)[i].DistanceMeters > (*Pareto)[i - 1].DistanceMeters);
							TestTrue(TEXT("非支配候補の水位が下降"), (*Pareto)[i].Site.WaterMeters < (*Pareto)[i - 1].Site.WaterMeters);
						}
						const PlanetCore::Vec3 A = OriginMinimum->Direction.Vector();
						std::size_t Undominated = 0;
						for(const PlanetCore::RiverReceiverSite &Site : Receivers->GetSites()) {
							if(Site.WaterMeters > OutletPassHeightMeters) continue;
							const PlanetCore::Vec3 Delta = Site.Direction.Vector() - A;
							const double CandidateChord = Delta.Dot(Delta);
							bool Covered = false;
							for(const PlanetCore::RiverReceiverNearest &Front : *Pareto) {
								const PlanetCore::Vec3 FrontDelta = Front.Site.Direction.Vector() - A;
								if(Front.Site.WaterMeters <= Site.WaterMeters && FrontDelta.Dot(FrontDelta) <= CandidateChord) {
									Covered = true;
									break;
								}
							}
							if(!Covered) ++Undominated;
						}
						TestTrue(TEXT("全水位適合候補が非支配集合に被覆される"), !Undominated);
						AddInfo(
							FString::Printf(
								TEXT("River receiver Pareto: seed=%llu candidates=%llu firstReach=%u firstWater=%.9gm ")
								TEXT("lastReach=%u lastWater=%.9gm uncovered=%llu. Corridor cost unmeasured."),
								static_cast<unsigned long long>(Current.Seed),
								static_cast<unsigned long long>(Pareto->size()),
								Pareto->front().Site.Reach,
								Pareto->front().Site.WaterMeters,
								Pareto->back().Site.Reach,
								Pareto->back().Site.WaterMeters,
								static_cast<unsigned long long>(Undominated)
							)
						);
					}
				}
				for(const double SourceDepthCap : std::array<double, 4>{ 1, 2, 3, 5 }) {
					const double WaterCeiling = OriginMinimum->ElevationMeters + SourceDepthCap;
					const std::optional<PlanetCore::RiverReceiverNearest> DepthReceiver =
					Receivers->FindNearest(OriginMinimum->Direction, WaterCeiling);
					AddInfo(
						FString::Printf(
							TEXT("Source-depth-limited river receiver: seed=%llu sourceDepthCap=%.9gm found=%u ")
							TEXT("reach=%u distance=%.9gm water=%.9gm ground=%.9gm. Necessary source bound only."),
							static_cast<unsigned long long>(Current.Seed),
							SourceDepthCap,
							static_cast<unsigned>(DepthReceiver.has_value()),
							DepthReceiver ? DepthReceiver->Site.Reach : UINT32_MAX,
							DepthReceiver ? DepthReceiver->DistanceMeters : -1,
							DepthReceiver ? DepthReceiver->Site.WaterMeters : -1,
							DepthReceiver ? DepthReceiver->Site.GroundMeters : -1
						)
					);
				}
				if(FeasibleReceiver) {
					const PlanetCore::Vec3 SourceDirection = OriginMinimum->Direction.Vector();
					double IndependentDistanceMeters = std::numeric_limits<double>::infinity();
					std::uint32_t IndependentReach = UINT32_MAX;
					for(const PlanetCore::RiverReceiverSite &Site : Receivers->GetSites()) {
						if(Site.WaterMeters > OutletPassHeightMeters) continue;
						const PlanetCore::Vec3 End = Site.Direction.Vector();
						const double Distance = Current.RadiusMeters * std::atan2(SourceDirection.Cross(End).Length(), SourceDirection.Dot(End));
						if(Distance >= IndependentDistanceMeters) continue;
						IndependentDistanceMeters = Distance;
						IndependentReach = Site.Reach;
					}
					TestEqual(TEXT("弦距離索引が独立な全角距離走査と同じ河道を選ぶ"), FeasibleReceiver->Site.Reach, IndependentReach);
					TestTrue(TEXT("弦距離索引の実球面距離が独立走査と一致"), std::abs(FeasibleReceiver->DistanceMeters - IndependentDistanceMeters) < 1E-6);
					TestFalse(
						TEXT("最近傍より短い距離上限で受入候補を作らない"),
						Receivers->FindNearest(OriginMinimum->Direction, OutletPassHeightMeters, FeasibleReceiver->DistanceMeters - 0.001).has_value()
					);
					TestTrue(TEXT("受入河川の水位が越流峠以下"), FeasibleReceiver->Site.WaterMeters <= OutletPassHeightMeters);
					TestTrue(TEXT("受入河川の河床が実水面より低い"), FeasibleReceiver->Site.GroundMeters <= FeasibleReceiver->Site.WaterMeters + 0.001);
					if(Current.Seed == 7 && Current.Mode == Mode::MesoOnly) {
						PlanetCore::SpillPassageSettings CorridorSettings;
						CorridorSettings.RadiusMeters = Current.RadiusMeters;
						CorridorSettings.SpacingMeters = 100;
						CorridorSettings.HalfWidthMeters = 2000;
						CorridorSettings.MaximumEvaluations = 250000;
						CorridorSettings.MaximumNodes = 65536;
						CorridorSettings.UseEndpointNeighborRing = true;
						const std::optional<PlanetCore::SpillPassage> Corridor = PlanetCore::FindSpillPassage(
							OriginMinimum->Direction,
							FeasibleReceiver->Site.Direction,
							[&Field](const PlanetCore::UnitDirection Direction) {
								return Field->Evaluate(Direction);
							},
							CorridorSettings
						);
						AddInfo(
							FString::Printf(
								TEXT("River receiver corridor: found=%u origin=%.9gm receiverWater=%.9gm ")
								TEXT("crest=%.9gm excess=%.9gm path=%.9gm nodes=%u evaluations=%llu. Landscape unaccepted."),
								static_cast<unsigned>(Corridor.has_value()),
								OriginMinimum->ElevationMeters,
								FeasibleReceiver->Site.WaterMeters,
								Corridor ? Corridor->CrestMeters : -1,
								Corridor ? Corridor->MaximumIndependentExcessMeters : -1,
								Corridor ? Corridor->PathLengthMeters : -1,
								Corridor ? Corridor->Nodes : 0,
								static_cast<unsigned long long>(Corridor ? Corridor->Evaluations : 0)
							)
						);
						constexpr double TargetGroundMeters = 370.0, CostSmoothingMeters = 0.01;
						const std::optional<PlanetCore::SpillPassage> BalancedCorridor = PlanetCore::FindSpillPassage(
							OriginMinimum->Direction,
							FeasibleReceiver->Site.Direction,
							[&Field](const PlanetCore::UnitDirection Direction) {
								const PlanetCore::TerrainSample Ground = Field->Evaluate(Direction);
								const double Difference = Ground.ElevationMeters - TargetGroundMeters, Cost = std::hypot(Difference, CostSmoothingMeters);
								// 標高からの両側偏差を滑らかな解析場として返す
								return PlanetCore::TerrainSample{ Cost, Ground.TangentialGradientPerRadian * (Difference / Cost) };
							},
							CorridorSettings
						);
						double BalancedMinimumGround = std::numeric_limits<double>::infinity();
						double BalancedMaximumGround = -std::numeric_limits<double>::infinity();
						if(BalancedCorridor) for(const PlanetCore::UnitDirection Direction : BalancedCorridor->Path) {
							const double Ground = Field->EvaluateElevation(Direction);
							BalancedMinimumGround = std::min(BalancedMinimumGround, Ground);
							BalancedMaximumGround = std::max(BalancedMaximumGround, Ground);
						}
						AddInfo(
							FString::Printf(
								TEXT("Balanced river corridor: found=%u targetGround=%.9gm costCrest=%.9gm ")
								TEXT("minimumGround=%.9gm maximumGround=%.9gm path=%.9gm nodes=%u evaluations=%llu. Landscape unaccepted."),
								static_cast<unsigned>(BalancedCorridor.has_value()),
								TargetGroundMeters,
								BalancedCorridor ? BalancedCorridor->CrestMeters : -1,
								BalancedMinimumGround,
								BalancedMaximumGround,
								BalancedCorridor ? BalancedCorridor->PathLengthMeters : -1,
								BalancedCorridor ? BalancedCorridor->Nodes : 0,
								static_cast<unsigned long long>(BalancedCorridor ? BalancedCorridor->Evaluations : 0)
							)
						);
						// 直線軸の両側を探索費用へ戻し，選択後は実経路の接線で独立に測る
						const PlanetCore::UnitDirection BankAxisDirection =
						*PlanetCore::UnitDirection::FromVector(OriginMinimum->Direction.Vector().Cross(FeasibleReceiver->Site.Direction.Vector()));
						const PlanetCore::Vec3 BankAxis = BankAxisDirection.Vector();
						constexpr double BankTargetGroundMeters = 373, BankSmoothingMeters = 0.01;
						const double BankFloorMeters = FeasibleReceiver->Site.WaterMeters + 0.3, BankRadiusMeters = Current.RadiusMeters;
						const auto BankAwareSource =
						[&Field, BankAxis, BankFloorMeters, BankRadiusMeters](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
							const PlanetCore::Vec3 P = Direction.Vector();
							const PlanetCore::TerrainSample Center = Field->Evaluate(Direction);
							const double Difference = Center.ElevationMeters - BankTargetGroundMeters;
							double Cost = std::hypot(Difference, BankSmoothingMeters);
							PlanetCore::Vec3 Gradient = Center.TangentialGradientPerRadian * (Difference / Cost);
							for(const double Width : { 20.0, 100.0 }) for(const double Side : { -1.0, 1.0 }) {
								const PlanetCore::Vec3 Tilt = P + BankAxis * (Side * std::tan(Width / BankRadiusMeters));
								const double Scale = Tilt.Length();
								const PlanetCore::UnitDirection Bank = *PlanetCore::UnitDirection::FromVector(Tilt);
								const PlanetCore::TerrainSample Sample = Field->Evaluate(Bank);
								const double Deficit = BankFloorMeters - Sample.ElevationMeters, Smooth = std::hypot(Deficit, BankSmoothingMeters);
								const double Weight = Width == 20 ? 2.0 : 1.0;
								Cost += Weight * 0.5 * (Deficit + Smooth);
								const PlanetCore::Vec3 Mapped = Sample.TangentialGradientPerRadian * (1 / Scale);
								Gradient = Gradient - (Mapped - P * Mapped.Dot(P)) * (Weight * 0.5 * (1 + Deficit / Smooth));
							}
							// 直線軸に対する横断標本の滑らかな費用と解析勾配の返戻
							return { Cost, Gradient };
						};
						const PlanetCore::Vec3 SourceAxis = OriginMinimum->Direction.Vector();
						const PlanetCore::TerrainSample SourceCost = BankAwareSource(OriginMinimum->Direction);
						for(const PlanetCore::Vec3 Tangent : { BankAxis, SourceAxis.Cross(BankAxis) }) {
							constexpr double StepRadians = 1E-6;
							const PlanetCore::UnitDirection Plus =
							*PlanetCore::UnitDirection::FromVector(SourceAxis * std::cos(StepRadians) + Tangent * std::sin(StepRadians));
							const PlanetCore::UnitDirection Minus =
							*PlanetCore::UnitDirection::FromVector(SourceAxis * std::cos(StepRadians) - Tangent * std::sin(StepRadians));
							const double Difference = (BankAwareSource(Plus).ElevationMeters - BankAwareSource(Minus).ElevationMeters) / (2 * StepRadians);
							const double Analytic = SourceCost.TangentialGradientPerRadian.Dot(Tangent);
							TestTrue(TEXT("河岸探索費用の解析勾配が独立差分と一致"), std::abs(Difference - Analytic) <= std::max(1.0, std::abs(Analytic)) * 1E-3);
						}
						const std::optional<PlanetCore::SpillPassage> BankAwareCorridor =
						PlanetCore::FindSpillPassage(OriginMinimum->Direction, FeasibleReceiver->Site.Direction, BankAwareSource, CorridorSettings);
						TestTrue(TEXT("河岸費用を考慮した有限候補通路を探索"), BankAwareCorridor.has_value());
						AddInfo(
							FString::Printf(
								TEXT("Bank-aware receiver corridor: found=%u costCrest=%.9g path=%.9gm evaluations=%llu. Landscape unaccepted."),
								static_cast<unsigned>(BankAwareCorridor.has_value()),
								BankAwareCorridor ? BankAwareCorridor->CrestMeters : -1,
								BankAwareCorridor ? BankAwareCorridor->PathLengthMeters : -1,
								static_cast<unsigned long long>(BankAwareCorridor ? BankAwareCorridor->Evaluations : 0)
							)
						);
						const double SourceCenterCap = OutletPassHeightMeters + 19.7;
						const auto CappedBankSource =
						[&BankAwareSource, &Field, SourceCenterCap](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
							PlanetCore::TerrainSample Cost = BankAwareSource(Direction);
							const PlanetCore::TerrainSample Ground = Field->Evaluate(Direction);
							const double Excess = Ground.ElevationMeters - SourceCenterCap, Smooth = std::hypot(Excess, 0.01);
							Cost.ElevationMeters += 50 * (Excess + Smooth);
							Cost.TangentialGradientPerRadian =
							Cost.TangentialGradientPerRadian + Ground.TangentialGradientPerRadian * (50 * (1 + Excess / Smooth));
							// 元地面の最大開削制約を探索費用へ加えた解析標本の返戻
							return Cost;
						};
						const std::optional<PlanetCore::SpillPassage> CappedBankCorridor =
						PlanetCore::FindSpillPassage(OriginMinimum->Direction, FeasibleReceiver->Site.Direction, CappedBankSource, CorridorSettings);
						double CappedMaximumGround = -std::numeric_limits<double>::infinity();
						std::uint32_t CappedOverLimit = 0;
						if(CappedBankCorridor) for(const PlanetCore::UnitDirection Direction : CappedBankCorridor->Path) {
							const double Ground = Field->EvaluateElevation(Direction);
							CappedMaximumGround = std::max(CappedMaximumGround, Ground);
							if(Ground > SourceCenterCap) ++CappedOverLimit;
						}
						AddInfo(
							FString::Printf(
								TEXT("Lake-level capped bank corridor: found=%u points=%llu length=%.9gm peak=%.9gm ")
								TEXT("centerAboveCap=%u evaluations=%llu. Banks untested; landscape unaccepted."),
								static_cast<unsigned>(CappedBankCorridor.has_value()),
								static_cast<unsigned long long>(CappedBankCorridor ? CappedBankCorridor->Path.size() : 0),
								CappedBankCorridor ? CappedBankCorridor->PathLengthMeters : -1.0,
								CappedMaximumGround,
								CappedOverLimit,
								static_cast<unsigned long long>(CappedBankCorridor ? CappedBankCorridor->Evaluations : 0)
							)
						);
						const std::optional<PlanetCore::SpillPassage> SaddleCorridor =
						PlanetCore::FindSpillPassage(Outlet->Pass.Direction, FeasibleReceiver->Site.Direction, CappedBankSource, CorridorSettings);
						double SaddleMaximumGround = -std::numeric_limits<double>::infinity();
						if(SaddleCorridor) for(const PlanetCore::UnitDirection Direction : SaddleCorridor->Path) {
							SaddleMaximumGround = std::max(SaddleMaximumGround, Field->EvaluateElevation(Direction));
						}
						const PlanetCore::Vec3 SaddleA = OriginMinimum->Direction.Vector(), SaddleB = Outlet->Pass.Direction.Vector();
						const double SaddleDistance = Current.RadiusMeters * std::atan2(SaddleA.Cross(SaddleB).Length(), SaddleA.Dot(SaddleB));
						AddInfo(
							FString::Printf(
								TEXT("Actual saddle source corridor: found=%u saddleDistance=%.9gm saddleH=%.9gm ")
								TEXT("points=%llu length=%.9gm peak=%.9gm evaluations=%llu. Banks untested; landscape unaccepted."),
								static_cast<unsigned>(SaddleCorridor.has_value()),
								SaddleDistance,
								Outlet->Pass.ElevationMeters,
								static_cast<unsigned long long>(SaddleCorridor ? SaddleCorridor->Path.size() : 0),
								SaddleCorridor ? SaddleCorridor->PathLengthMeters : -1.0,
								SaddleMaximumGround,
								static_cast<unsigned long long>(SaddleCorridor ? SaddleCorridor->Evaluations : 0)
							)
						);
						if(CappedBankCorridor && CappedBankCorridor->Path.size() > 1) {
							std::vector<std::array<std::uint32_t, 2>> CappedEdges;
							for(std::uint32_t i = 1; i < CappedBankCorridor->Path.size(); ++i) CappedEdges.push_back({ i - 1, i });
							PlanetCore::RiverRoutingTerrainSettings CappedSettings;
							CappedSettings.RadiusMeters = Current.RadiusMeters;
							CappedSettings.MaximumEvaluations = 200000;
							PlanetCore::RiverRoutingTerrainError CappedError = PlanetCore::RiverRoutingTerrainError::None;
							const std::array<std::uint32_t, 2> OwnedEndpoints { 0, static_cast<std::uint32_t>(CappedBankCorridor->Path.size() - 1) };
							std::optional<PlanetCore::RiverRoutingTerrainGraph> CappedGraph = PlanetCore::PrepareRiverRoutingTerrainGraph(
								CappedBankCorridor->Path,
								CappedEdges,
								[&Field](const PlanetCore::UnitDirection Direction) {
									return Field->EvaluateElevation(Direction);
								},
								CappedSettings,
								CappedError,
								{},
								OwnedEndpoints
							);
							TestTrue(TEXT("湖水位制限通路の実地形を岸付き水位網へ変換"), CappedGraph.has_value());
							if(CappedGraph) {
								CappedGraph->Sites.front() = { OutletPassHeightMeters, OutletPassHeightMeters };
								CappedGraph->Sites.back() = { FeasibleReceiver->Site.WaterMeters, FeasibleReceiver->Site.WaterMeters };
								PlanetCore::RiverRoutingError CappedRouteError = PlanetCore::RiverRoutingError::None;
								PlanetCore::RiverRoutingDiagnostics CappedReachability;
								const std::optional<PlanetCore::RiverRoutingResult> CappedRoute = PlanetCore::FindRiverRoutingPath(
									CappedGraph->Sites,
									CappedGraph->Edges,
									0,
									OwnedEndpoints.back(),
									FeasibleReceiver->Site.WaterMeters,
									CappedSettings.MinimumDownstreamSlope,
									CappedRouteError,
									{},
									&CappedReachability
								);
								std::size_t FirstReachable = CappedGraph->Sites.size();
								for(std::size_t i = 0; i < CappedReachability.MinimumRequiredWaterMeters.size(); ++i) {
									if(!std::isfinite(CappedReachability.MinimumRequiredWaterMeters[i])) continue;
									FirstReachable = i;
									break;
								}
								AddInfo(
									FString::Printf(
										TEXT("Lake-level capped bank route: sites=%llu edges=%llu rejected=%u evaluations=%llu ")
										TEXT("route=%u error=%u firstReachable=%llu sourceWater=%.9gm. Landscape unaccepted."),
										static_cast<unsigned long long>(CappedGraph->Sites.size()),
										static_cast<unsigned long long>(CappedGraph->Edges.size()),
										CappedGraph->RejectedUndirectedEdges,
										static_cast<unsigned long long>(CappedGraph->Evaluations),
										static_cast<unsigned>(CappedRoute.has_value()),
										static_cast<unsigned>(CappedRouteError),
										static_cast<unsigned long long>(FirstReachable),
										CappedRoute ? CappedRoute->WaterMeters.front() : -1.0
									)
								);
							}
						}
						if(BankAwareCorridor && BankAwareCorridor->Path.size() > 1) {
							std::vector<std::array<std::uint32_t, 2>> PathEdges;
							PathEdges.reserve(BankAwareCorridor->Path.size() - 1);
							for(std::uint32_t i = 1; i < BankAwareCorridor->Path.size(); ++i) PathEdges.push_back({ i - 1, i });
							PlanetCore::RiverRoutingTerrainSettings RoutingSettings;
							RoutingSettings.RadiusMeters = Current.RadiusMeters;
							RoutingSettings.MaximumEvaluations = 200000;
							PlanetCore::RiverRoutingTerrainError TerrainError = PlanetCore::RiverRoutingTerrainError::None;
							const std::optional<PlanetCore::RiverRoutingTerrainGraph> TerrainGraph = PlanetCore::PrepareRiverRoutingTerrainGraph(
								BankAwareCorridor->Path,
								PathEdges,
								[&Field](const PlanetCore::UnitDirection Direction) {
									return Field->EvaluateElevation(Direction);
								},
								RoutingSettings,
								TerrainError
							);
							TestTrue(TEXT("実地形の河道候補を両岸付き水位網へ変換"), TerrainGraph.has_value());
							if(TerrainGraph) {
								PlanetCore::RiverRoutingError RouteError = PlanetCore::RiverRoutingError::None;
								const std::optional<PlanetCore::RiverRoutingResult> Routed =
								TerrainGraph->Edges.empty() ? std::nullopt : PlanetCore::FindRiverRoutingPath(
									TerrainGraph->Sites,
									TerrainGraph->Edges,
									0,
									static_cast<std::uint32_t>(BankAwareCorridor->Path.size() - 1),
									FeasibleReceiver->Site.WaterMeters,
									RoutingSettings.MinimumDownstreamSlope,
									RouteError
								);
								AddInfo(
									FString::Printf(
										TEXT("Real-terrain river constraint graph: sites=%llu directedEdges=%llu rejected=%u ")
										TEXT("evaluations=%llu route=%u routeError=%u sourceWater=%.9gm. Landscape unaccepted."),
										static_cast<unsigned long long>(TerrainGraph->Sites.size()),
										static_cast<unsigned long long>(TerrainGraph->Edges.size()),
										TerrainGraph->RejectedUndirectedEdges,
										static_cast<unsigned long long>(TerrainGraph->Evaluations),
										static_cast<unsigned>(Routed.has_value()),
										static_cast<unsigned>(RouteError),
										Routed ? Routed->WaterMeters.front() : -1
									)
								);
							}
						}
						if(BankAwareCorridor && BankAwareCorridor->Path.size() > 1) {
							const std::size_t Stations = BankAwareCorridor->Path.size();
							const double LateralAngle = 100 / Current.RadiusMeters;
							std::vector<PlanetCore::UnitDirection> LaneDirections;
							std::vector<std::array<std::uint32_t, 2>> LaneEdges;
							LaneDirections.reserve(Stations * 3);
							LaneEdges.reserve((Stations - 1) * 3 + Stations * 2);
							for(std::size_t Station = 0; Station < Stations; ++Station) {
								const PlanetCore::Vec3 P = BankAwareCorridor->Path[Station].Vector();
								const PlanetCore::Vec3 Before = BankAwareCorridor->Path[Station ? Station - 1 : Station].Vector();
								const PlanetCore::Vec3 After = BankAwareCorridor->Path[Station + 1 < Stations ? Station + 1 : Station].Vector();
								const PlanetCore::Vec3 AlongRaw = After - Before;
								const PlanetCore::Vec3 Along = PlanetCore::UnitDirection::FromVector(AlongRaw - P * AlongRaw.Dot(P))->Vector();
								const PlanetCore::Vec3 Across = P.Cross(Along);
								for(const double Side : { -1.0, 0.0, 1.0 }) {
									const PlanetCore::Vec3 Offset = P * std::cos(LateralAngle * Side) + Across * std::sin(LateralAngle * Side);
									LaneDirections.push_back(*PlanetCore::UnitDirection::FromVector(Offset));
								}
								const std::uint32_t Base = static_cast<std::uint32_t>(Station * 3);
								LaneEdges.push_back({ Base, Base + 1 });
								LaneEdges.push_back({ Base + 1, Base + 2 });
								if(Station) for(std::uint32_t Lane = 0; Lane < 3; ++Lane) LaneEdges.push_back({ Base - 3 + Lane, Base + Lane });
							}
							PlanetCore::RiverRoutingTerrainSettings LaneSettings;
							LaneSettings.RadiusMeters = Current.RadiusMeters;
							LaneSettings.MaximumEvaluations = 500000;
							PlanetCore::RiverRoutingTerrainError LaneError = PlanetCore::RiverRoutingTerrainError::None;
							const std::optional<PlanetCore::RiverRoutingTerrainGraph> LaneGraph = PlanetCore::PrepareRiverRoutingTerrainGraph(
								LaneDirections,
								LaneEdges,
								[&Field](const PlanetCore::UnitDirection Direction) {
									return Field->EvaluateElevation(Direction);
								},
								LaneSettings,
								LaneError
							);
							TestTrue(TEXT("実地形に沿う三列の迂回候補を河岸付き制約網へ変換"), LaneGraph.has_value());
							if(LaneGraph) {
								PlanetCore::RiverRoutingError RouteError = PlanetCore::RiverRoutingError::None;
								PlanetCore::RiverRoutingDiagnostics Reachability;
								const std::optional<PlanetCore::RiverRoutingResult> LaneRoute =
								LaneGraph->Edges.empty() ? std::nullopt : PlanetCore::FindRiverRoutingPath(
									LaneGraph->Sites,
									LaneGraph->Edges,
									1,
									static_cast<std::uint32_t>((Stations - 1) * 3 + 1),
									FeasibleReceiver->Site.WaterMeters,
									LaneSettings.MinimumDownstreamSlope,
									RouteError,
									{},
									&Reachability
								);
								std::size_t FirstReachableStation = Stations;
								std::uint32_t ReachableSites = 0, ValidReceiverIncoming = 0;
								for(std::size_t Node = 0; Node < Reachability.MinimumRequiredWaterMeters.size(); ++Node) {
									if(!std::isfinite(Reachability.MinimumRequiredWaterMeters[Node])) continue;
									++ReachableSites;
									FirstReachableStation = std::min(FirstReachableStation, Node / 3);
								}
								const std::uint32_t ReceiverIndex = static_cast<std::uint32_t>((Stations - 1) * 3 + 1);
								for(const PlanetCore::RiverRoutingEdge &Edge : LaneGraph->Edges) if(
									Edge.Downstream == ReceiverIndex && FeasibleReceiver->Site.WaterMeters >= Edge.MinimumDownstreamWaterMeters &&
									FeasibleReceiver->Site.WaterMeters <= Edge.MaximumDownstreamWaterMeters
								) ++ValidReceiverIncoming;
								AddInfo(
									FString::Printf(
										TEXT("Three-lane river constraint graph: sites=%llu directedEdges=%llu rejected=%u ")
										TEXT("evaluations=%llu route=%u routeError=%u routeLength=%.9gm reachableSites=%u firstReachableStation=%llu ")
										TEXT("receiverIncoming=%u. Landscape unaccepted."),
										static_cast<unsigned long long>(LaneGraph->Sites.size()),
										static_cast<unsigned long long>(LaneGraph->Edges.size()),
										LaneGraph->RejectedUndirectedEdges,
										static_cast<unsigned long long>(LaneGraph->Evaluations),
										static_cast<unsigned>(LaneRoute.has_value()),
										static_cast<unsigned>(RouteError),
										LaneRoute ? LaneRoute->LengthMeters : -1,
										ReachableSites,
										static_cast<unsigned long long>(FirstReachableStation),
										ValidReceiverIncoming
									)
								);
							}
						}
						if(BankAwareCorridor && BankAwareCorridor->Path.size() > 1) {
							const PlanetCore::Vec3 P = BankAwareCorridor->Path.back().Vector();
							const PlanetCore::Vec3 Before = BankAwareCorridor->Path[BankAwareCorridor->Path.size() - 2].Vector();
							const PlanetCore::Vec3 AlongRaw = P - Before;
							const PlanetCore::Vec3 Along = PlanetCore::UnitDirection::FromVector(AlongRaw - P * AlongRaw.Dot(P))->Vector();
							const PlanetCore::Vec3 Across = P.Cross(Along);
							for(const double OffsetMeters : { -320.0, -160.0, -80.0, -40.0, -20.0, 0.0, 20.0, 40.0, 80.0, 160.0, 320.0 }) {
								const double Angle = OffsetMeters / Current.RadiusMeters;
								const PlanetCore::UnitDirection Direction =
								*PlanetCore::UnitDirection::FromVector(P * std::cos(Angle) + Across * std::sin(Angle));
								const PlanetCore::RiverWaterHeights Sample = Field->EvaluateWater(Direction);
								AddInfo(
									FString::Printf(
										TEXT("Receiving river cross-section: offset=%.9gm ground=%.9gm water=%.9gm support=%.9g wet=%u."),
										OffsetMeters,
										Sample.TerrainElevationMeters,
										Sample.WaterElevationMeters,
										Sample.WaterSupport,
										static_cast<unsigned>(Sample.WaterSupport > 0 && Sample.WaterElevationMeters > Sample.TerrainElevationMeters)
									)
								);
							}
						}
						if(SaddleCorridor && SaddleCorridor->Path.size() > 1) {
							const std::vector<PlanetCore::UnitDirection> &ReceiverPath = SaddleCorridor->Path;
							std::optional<std::size_t> ShoreSegment;
							for(std::size_t i = ReceiverPath.size() - 1; i > 0; --i) {
								const PlanetCore::RiverWaterHeights Dry = Field->EvaluateWater(ReceiverPath[i - 1]);
								const PlanetCore::RiverWaterHeights Wet = Field->EvaluateWater(ReceiverPath[i]);
								if(Wet.WaterSupport < 0.5 || Wet.WaterElevationMeters <= Wet.TerrainElevationMeters) break;
								if(Dry.WaterSupport > 0.5 && Dry.WaterElevationMeters > Dry.TerrainElevationMeters) continue;
								if(Dry.WaterSupport > 0.5) ShoreSegment = i;
								break;
							}
							TestTrue(TEXT("受入河川へ入る候補線に実水面の岸交差を検出"), ShoreSegment.has_value());
							if(ShoreSegment) {
								const std::optional<PlanetCore::ShoreCrossing> Shore = PlanetCore::FindShoreCrossing(
									ReceiverPath[*ShoreSegment - 1],
									ReceiverPath[*ShoreSegment],
									Current.RadiusMeters,
									0,
									[&Field](const PlanetCore::UnitDirection Direction) {
										const PlanetCore::RiverWaterHeights Sample = Field->EvaluateWater(Direction);
										// 同じ最終地形と受入水面の符号付き岸距離の返戻
										return Sample.TerrainElevationMeters - Sample.WaterElevationMeters;
									}
								);
								TestTrue(TEXT("受入河川の実水面境界を絞り込む"), Shore.has_value());
								if(Shore) {
									std::vector<PlanetCore::UnitDirection> ToShore(
										ReceiverPath.begin(),
										ReceiverPath.begin() + static_cast<std::ptrdiff_t>(*ShoreSegment)
									);
									const PlanetCore::UnitDirection ShoreDirection = *PlanetCore::UnitDirection::FromVector(Shore->Direction);
									ToShore.push_back(ShoreDirection);
									std::vector<std::array<std::uint32_t, 2>> ShoreEdges;
									ShoreEdges.reserve(ToShore.size() - 1);
									for(std::uint32_t i = 1; i < ToShore.size(); ++i) ShoreEdges.push_back({ i - 1, i });
									const std::array<std::uint32_t, 1> WaterOwned { static_cast<std::uint32_t>(ToShore.size() - 1) };
									PlanetCore::RiverRoutingTerrainSettings ShoreSettings;
									ShoreSettings.RadiusMeters = Current.RadiusMeters;
									ShoreSettings.MaximumEvaluations = 200000;
									PlanetCore::RiverRoutingTerrainError ShoreError = PlanetCore::RiverRoutingTerrainError::None;
									const std::optional<PlanetCore::RiverRoutingTerrainGraph> ShoreGraph = PlanetCore::PrepareRiverRoutingTerrainGraph(
										ToShore,
										ShoreEdges,
										[&Field](const PlanetCore::UnitDirection Direction) {
											return Field->EvaluateElevation(Direction);
										},
										ShoreSettings,
										ShoreError,
										{},
										WaterOwned
									);
									TestTrue(TEXT("実水域が所有する岸端へ乾いた河道候補を接続"), ShoreGraph.has_value());
									if(ShoreGraph) {
										const PlanetCore::RiverWaterHeights Boundary = Field->EvaluateWater(ShoreDirection);
										PlanetCore::RiverRoutingError RouteError = PlanetCore::RiverRoutingError::None;
										PlanetCore::RiverRoutingDiagnostics Reachability;
										const std::optional<PlanetCore::RiverRoutingResult> Route =
										ShoreGraph->Edges.empty() ? std::nullopt : PlanetCore::FindRiverRoutingPath(
											ShoreGraph->Sites,
											ShoreGraph->Edges,
											0,
											WaterOwned[0],
											Boundary.WaterElevationMeters,
											ShoreSettings.MinimumDownstreamSlope,
											RouteError,
											{},
											&Reachability
										);
										std::size_t FirstReachable = ToShore.size();
										for(std::size_t i = 0; i < Reachability.MinimumRequiredWaterMeters.size(); ++i) {
											if(std::isfinite(Reachability.MinimumRequiredWaterMeters[i])) FirstReachable = std::min(FirstReachable, i);
										}
										for(const PlanetCore::RiverRoutingEdge &Edge : ShoreGraph->Edges) {
											if(Edge.Upstream + 1 != WaterOwned[0] || Edge.Downstream != WaterOwned[0]) continue;
											AddInfo(
												FString::Printf(
													TEXT("Receiving shoreline final edge: length=%.9gm shoreWater=%.9gm ")
													TEXT("minimumUpstream=%.9gm maximumUpstream=%.9gm minimumDownstream=%.9gm ")
													TEXT("maximumDownstream=%.9gm upstreamSite=[%.9g,%.9g]."),
													Edge.LengthMeters,
													Boundary.WaterElevationMeters,
													Edge.MinimumUpstreamWaterMeters,
													Edge.MaximumUpstreamWaterMeters,
													Edge.MinimumDownstreamWaterMeters,
													Edge.MaximumDownstreamWaterMeters,
													ShoreGraph->Sites[Edge.Upstream].MinimumWaterMeters,
													ShoreGraph->Sites[Edge.Upstream].MaximumWaterMeters
												)
											);
											break;
										}
										AddInfo(
											FString::Printf(
												TEXT("Receiving river shoreline route: segment=%llu landFraction=%.9g wetFraction=%.9g ")
												TEXT("bracket=%.9gm residual=%.9gm shoreH=%.9gm shoreW=%.9gm sites=%llu edges=%llu rejected=%u ")
												TEXT("route=%u routeError=%u firstReachable=%llu. Landscape unaccepted."),
												static_cast<unsigned long long>(*ShoreSegment),
												Shore->LandFraction,
												Shore->WetFraction,
												Shore->BracketWidthMeters,
												Shore->TerrainResidualMeters,
												Boundary.TerrainElevationMeters,
												Boundary.WaterElevationMeters,
												static_cast<unsigned long long>(ShoreGraph->Sites.size()),
												static_cast<unsigned long long>(ShoreGraph->Edges.size()),
												ShoreGraph->RejectedUndirectedEdges,
												static_cast<unsigned>(Route.has_value()),
												static_cast<unsigned>(RouteError),
												static_cast<unsigned long long>(FirstReachable)
											)
										);
									}
									const std::optional<PlanetCore::RiverProfileSample> ReceivingCurve =
									Field->Drainage().SampleReach(FeasibleReceiver->Site.Reach, FeasibleReceiver->Site.Fraction);
									TestTrue(TEXT("受入河川の接線を河道定義から取得"), ReceivingCurve.has_value());
									if(ReceivingCurve) {
										const PlanetCore::Vec3 ReceiverP = FeasibleReceiver->Site.Direction.Vector();
										const PlanetCore::Vec3 RiverAlong = PlanetCore::UnitDirection::FromVector(
											ReceivingCurve->DownstreamTangent - ReceiverP * ReceivingCurve->DownstreamTangent.Dot(ReceiverP)
										)->Vector();
										std::vector<PlanetCore::UnitDirection> FanDirections(
											ReceiverPath.begin(),
											ReceiverPath.begin() + static_cast<std::ptrdiff_t>(*ShoreSegment)
										);
										std::vector<std::array<std::uint32_t, 2>> FanEdges;
										for(std::uint32_t i = 1; i < FanDirections.size(); ++i) FanEdges.push_back({ i - 1, i });
										const std::uint32_t DryIndex = static_cast<std::uint32_t>(FanDirections.size() - 1);
										std::vector<std::uint32_t> FanOwned;
										std::vector<PlanetCore::RiverRoutingReceiver> FanReceivers;
										std::vector<double> FanOffsets;
										for(const double Offset : { -60.0, -40.0, -20.0, -10.0, 0.0, 10.0, 20.0, 40.0, 60.0 }) {
											const double Angle = Offset / Current.RadiusMeters;
											const PlanetCore::UnitDirection Candidate =
											*PlanetCore::UnitDirection::FromVector(ReceiverP * std::cos(Angle) + RiverAlong * std::sin(Angle));
											const PlanetCore::RiverWaterHeights Wet = Field->EvaluateWater(Candidate);
											if(Wet.WaterSupport < 0.5 || Wet.WaterElevationMeters <= Wet.TerrainElevationMeters) continue;
											const std::optional<PlanetCore::ShoreCrossing> CandidateShore = PlanetCore::FindShoreCrossing(
												FanDirections[DryIndex],
												Candidate,
												Current.RadiusMeters,
												0,
												[&Field](const PlanetCore::UnitDirection Direction) {
													const PlanetCore::RiverWaterHeights Sample = Field->EvaluateWater(Direction);
													// 既存河川の局所水面での岸距離の返戻
													return Sample.TerrainElevationMeters - Sample.WaterElevationMeters;
												}
											);
											if(!CandidateShore) continue;
											const PlanetCore::UnitDirection Boundary = *PlanetCore::UnitDirection::FromVector(CandidateShore->Direction);
											const PlanetCore::RiverWaterHeights Water = Field->EvaluateWater(Boundary);
											if(Water.WaterSupport < 0.5 || std::abs(Water.TerrainElevationMeters - Water.WaterElevationMeters) > 0.01) continue;
											const std::uint32_t Node = static_cast<std::uint32_t>(FanDirections.size());
											FanDirections.push_back(Boundary);
											FanEdges.push_back({ DryIndex, Node });
											FanOwned.push_back(Node);
											FanReceivers.push_back({ Node, Water.WaterElevationMeters });
											FanOffsets.push_back(Offset);
										}
										std::optional<PlanetCore::ShoreCrossing> SourceShore;
										std::uint32_t SourceNode = 0, SourceLandStation = 0;
										bool SourceAtSaddle = false;
										const PlanetCore::Vec3 Origin = OriginMinimum->Direction.Vector();
										for(const PlanetCore::SpillConnectionBranch &Branch : Outlet->Branches) {
											if(Branch.Kind != PlanetCore::SpillTerminalKind::Minimum || !Branch.Minimum) continue;
											const PlanetCore::Vec3 Down = Branch.Minimum->Direction.Vector();
											const double Distance = Current.RadiusMeters * std::atan2(Origin.Cross(Down).Length(), Origin.Dot(Down));
											if(Distance < 0.001) SourceAtSaddle = true;
										}
										const PlanetCore::Vec3 SaddleStart = ReceiverPath.front().Vector(), SaddlePoint = Outlet->Pass.Direction.Vector();
										const double StartDistance =
										Current.RadiusMeters * std::atan2(SaddleStart.Cross(SaddlePoint).Length(), SaddleStart.Dot(SaddlePoint));
										SourceAtSaddle = SourceAtSaddle && StartDistance < 0.001 &&
										std::abs(Field->EvaluateElevation(ReceiverPath.front()) - OutletPassHeightMeters) < 0.001;
										TestTrue(TEXT("元窪地に接続した実鞍点を湖出口として検証"), SourceAtSaddle);
										if(SourceAtSaddle) FanOwned.push_back(0);
										if(!SourceAtSaddle) for(std::uint32_t Station = 1; Station <= std::min<std::uint32_t>(DryIndex, 32); ++Station) {
											const double Land = Field->EvaluateElevation(FanDirections[Station]);
											const double Wet = Field->EvaluateElevation(FanDirections[Station - 1]);
											if(!(Land > OutletPassHeightMeters && Wet <= OutletPassHeightMeters)) continue;
											SourceShore = PlanetCore::FindShoreCrossing(
												FanDirections[Station],
												FanDirections[Station - 1],
												Current.RadiusMeters,
												OutletPassHeightMeters,
												[&Field](const PlanetCore::UnitDirection Direction) {
													return Field->EvaluateElevation(Direction);
												}
											);
											if(!SourceShore) continue;
											SourceNode = static_cast<std::uint32_t>(FanDirections.size());
											SourceLandStation = Station;
											FanDirections.push_back(*PlanetCore::UnitDirection::FromVector(SourceShore->Direction));
											FanEdges.push_back({ SourceNode, Station });
											FanOwned.push_back(SourceNode);
											break;
										}
										AddInfo(
											FString::Printf(
												TEXT("Source lake shoreline: found=%u landStation=%u water=%.9gm ")
												TEXT("bracket=%.9gm residual=%.9gm. Lake topology unaccepted."),
												static_cast<unsigned>(SourceShore.has_value()),
												SourceLandStation,
												OutletPassHeightMeters,
												SourceShore ? SourceShore->BracketWidthMeters : -1.0,
												SourceShore ? SourceShore->TerrainResidualMeters : -1.0
											)
										);
										std::uint32_t HighStart = UINT32_MAX, HighEnd = 0;
										std::uint32_t HighestStation = 0;
										double HighestGround = -std::numeric_limits<double>::infinity();
										for(std::uint32_t Station = 0; Station <= DryIndex; ++Station) {
											const double Ground = Field->EvaluateElevation(FanDirections[Station]);
											if(Ground + 0.3 - OutletPassHeightMeters > 20) {
												HighStart = std::min(HighStart, Station);
												HighEnd = Station;
											}
											if(Ground > HighestGround) {
												HighestGround = Ground;
												HighestStation = Station;
											}
										}
										AddInfo(
											FString::Printf(
												TEXT("High passage center span: first=%u last=%u peak=%u peakH=%.9gm."),
												HighStart,
												HighEnd,
												HighestStation,
												HighestGround
											)
										);
										const PlanetCore::Vec3 Peak = FanDirections[HighestStation].Vector();
										const PlanetCore::Vec3 PeakBefore = FanDirections[HighestStation ? HighestStation - 1 : HighestStation].Vector();
										const PlanetCore::Vec3 PeakAfter = FanDirections[HighestStation < DryIndex ? HighestStation + 1 : HighestStation].Vector();
										const PlanetCore::Vec3 PeakRaw = PeakAfter - PeakBefore;
										const PlanetCore::Vec3 PeakAlong = PlanetCore::UnitDirection::FromVector(PeakRaw - Peak * PeakRaw.Dot(Peak))->Vector();
										const PlanetCore::Vec3 PeakAcross = Peak.Cross(PeakAlong);
										for(const double Offset : { -2000.0, -1000.0, -500.0, -200.0, 0.0, 200.0, 500.0, 1000.0, 2000.0 }) {
											const double Angle = Offset / Current.RadiusMeters;
											const PlanetCore::UnitDirection Probe =
											*PlanetCore::UnitDirection::FromVector(Peak * std::cos(Angle) + PeakAcross * std::sin(Angle));
											const double Ground = Field->EvaluateElevation(Probe);
											AddInfo(
												FString::Printf(
													TEXT("High passage cross-section: station=%u offset=%.9gm H=%.9gm ")
													TEXT("minimumCutAtLake=%.9gm lakeWater=%.9gm. Bank untested."),
													HighestStation,
													Offset,
													Ground,
													std::max(0.0, Ground + 0.3 - OutletPassHeightMeters),
													OutletPassHeightMeters
												)
											);
										}
										TestTrue(TEXT("受入河川上の複数の実水面岸候補を検出"), !FanReceivers.empty());
										if(!FanReceivers.empty()) {
											std::vector<std::uint32_t> FanStations(FanDirections.size(), static_cast<std::uint32_t>(DryIndex + 1));
											for(std::uint32_t Station = 0; Station <= DryIndex; ++Station) FanStations[Station] = Station;
											if(SourceShore) FanStations[SourceNode] = SourceLandStation - 1;
											std::vector<std::array<std::uint32_t, 5>> LaneNodes(DryIndex + 1);
											std::vector<std::uint8_t> LaneBuilt(DryIndex + 1, 0), LaneConnected(DryIndex + 1, 0);
											const auto AddLattice = [&](const std::size_t Target) {
												const std::size_t First = Target > 20 ? Target - 20 : 0;
												std::size_t Added = 0;
												for(std::size_t Station = First; Station <= Target; ++Station) {
													if(LaneBuilt[Station]) continue;
													const PlanetCore::Vec3 P = FanDirections[Station].Vector();
													const PlanetCore::Vec3 Before = FanDirections[Station ? Station - 1 : Station].Vector();
													const PlanetCore::Vec3 After = FanDirections[Station < DryIndex ? Station + 1 : Station].Vector();
													const PlanetCore::Vec3 AlongRaw = After - Before;
													const PlanetCore::Vec3 Along = PlanetCore::UnitDirection::FromVector(AlongRaw - P * AlongRaw.Dot(P))->Vector();
													const PlanetCore::Vec3 Across = P.Cross(Along);
													std::array<std::uint32_t, 5> &Lanes = LaneNodes[Station];
													Lanes[2] = static_cast<std::uint32_t>(Station);
													for(std::size_t Side = 0; Side < 4; ++Side) {
														const double Offset = std::array<double, 4>{ -200, -100, 100, 200 }[Side], Angle = Offset / Current.RadiusMeters;
														const PlanetCore::Vec3 Point = P * std::cos(Angle) + Across * std::sin(Angle);
														Lanes[Side < 2 ? Side : Side + 1] = static_cast<std::uint32_t>(FanDirections.size());
														FanDirections.push_back(*PlanetCore::UnitDirection::FromVector(Point));
														FanStations.push_back(static_cast<std::uint32_t>(Station));
													}
													LaneBuilt[Station] = 1;
													++Added;
												}
												for(std::size_t Station = std::max<std::size_t>(1, First); Station <= std::min<std::size_t>(DryIndex, Target + 1); ++Station) {
													if(!LaneBuilt[Station - 1] || !LaneBuilt[Station] || LaneConnected[Station]) continue;
													const std::array<std::uint32_t, 5> &Previous = LaneNodes[Station - 1], &Present = LaneNodes[Station];
													for(std::size_t Lane = 0; Lane < 5; ++Lane) if(Lane != 2) FanEdges.push_back({ Previous[Lane], Present[Lane] });
													for(std::size_t Lane = 1; Lane < 5; ++Lane) {
														FanEdges.push_back({ Previous[Lane - 1], Present[Lane] });
														FanEdges.push_back({ Previous[Lane], Present[Lane - 1] });
													}
													LaneConnected[Station] = 1;
												}
												// 新しく追加した局所候補の測点数を返す
												return Added;
											};
											AddLattice(DryIndex);
											AddLattice(HighestStation);
											AddLattice(std::min<std::size_t>(DryIndex, HighestStation + 20));
											if(SourceShore) {
												AddLattice(SourceLandStation);
												for(std::size_t Lane = 0; Lane < 5; ++Lane) {
													if(Lane == 2) continue;
													FanEdges.push_back({ SourceNode, LaneNodes[SourceLandStation][Lane] });
												}
											}
											PlanetCore::RiverRoutingTerrainSettings FanSettings;
											FanSettings.RadiusMeters = Current.RadiusMeters;
											FanSettings.MaximumSampleStepMeters = 12.5;
											FanSettings.MaximumEvaluations = 500000;
											for(std::size_t Pass = 0; Pass < 40; ++Pass) {
												std::vector<double> FanBankHalfWidths(FanEdges.size(), FanSettings.BankHalfWidthMeters);
												for(std::size_t EdgeIndex = 0; EdgeIndex < FanEdges.size(); ++EdgeIndex) {
													if(SourceShore && FanEdges[EdgeIndex][0] == SourceNode) FanBankHalfWidths[EdgeIndex] = 160;
													if(FanStations[FanEdges[EdgeIndex][1]] > DryIndex) FanBankHalfWidths[EdgeIndex] = 5;
												}
												PlanetCore::RiverRoutingTerrainError FanError = PlanetCore::RiverRoutingTerrainError::None;
												std::optional<PlanetCore::RiverRoutingTerrainGraph> FanGraph = PlanetCore::PrepareRiverRoutingTerrainGraph(
													FanDirections,
													FanEdges,
													[&Field](const PlanetCore::UnitDirection Direction) {
														return Field->EvaluateElevation(Direction);
													},
													FanSettings,
													FanError,
													{},
													FanOwned,
													FanBankHalfWidths
												);
												TestTrue(TEXT("実水面岸の適応候補を河岸付き制約網へ変換"), FanGraph.has_value());
												if(!FanGraph) break;
												if(SourceAtSaddle || SourceShore) FanGraph->Sites[SourceNode] = { OutletPassHeightMeters, OutletPassHeightMeters };
												std::vector<PlanetCore::RiverRoutingEdge> AcyclicEdges;
												for(const PlanetCore::RiverRoutingEdge &Edge : FanGraph->Edges) {
													if(FanStations[Edge.Upstream] < FanStations[Edge.Downstream]) AcyclicEdges.push_back(Edge);
												}
												if(!Pass) for(std::size_t ReceiverIndex = 0; ReceiverIndex < FanReceivers.size(); ++ReceiverIndex) {
													const PlanetCore::RiverRoutingReceiver &Receiver = FanReceivers[ReceiverIndex];
													std::uint32_t Incoming = 0;
													double MaximumDownstream = -std::numeric_limits<double>::infinity();
													for(const PlanetCore::RiverRoutingEdge &Edge : AcyclicEdges) {
														if(Edge.Downstream != Receiver.Node) continue;
														++Incoming;
														MaximumDownstream = std::max(MaximumDownstream, Edge.MaximumDownstreamWaterMeters);
													}
													AddInfo(
														FString::Printf(
															TEXT("Receiving fan water window: offset=%.9gm node=%u incoming=%u receiverWater=%.12gm ")
															TEXT("maximumDownstream=%.12gm deficit=%.12gm. Landscape unaccepted."),
															FanOffsets[ReceiverIndex],
															Receiver.Node,
															Incoming,
															Receiver.WaterMeters,
															MaximumDownstream,
															Receiver.WaterMeters - MaximumDownstream
														)
													);
												}
												PlanetCore::RiverRoutingError RouteError = PlanetCore::RiverRoutingError::None;
												PlanetCore::RiverRoutingDiagnostics Reachability;
												const std::optional<PlanetCore::RiverRoutingResult> Route = PlanetCore::FindRiverRoutingPath(
													FanGraph->Sites,
													AcyclicEdges,
													SourceNode,
													FanReceivers,
													FanSettings.MinimumDownstreamSlope,
													RouteError,
													{},
													&Reachability
												);
												std::size_t FirstReachable = FanDirections.size();
												for(std::size_t i = 0; i <= DryIndex && i < Reachability.MinimumRequiredWaterMeters.size(); ++i) {
													if(std::isfinite(Reachability.MinimumRequiredWaterMeters[i])) FirstReachable = std::min(FirstReachable, i);
												}
												std::size_t FirstStation = DryIndex + 1;
												for(std::size_t i = 0; i < FanStations.size() && i < Reachability.MinimumRequiredWaterMeters.size(); ++i) {
													if(std::isfinite(Reachability.MinimumRequiredWaterMeters[i])) {
														FirstStation = std::min(FirstStation, static_cast<std::size_t>(FanStations[i]));
													}
												}
												AddInfo(
													FString::Printf(
														TEXT("Adaptive shoreline corridor: pass=%llu sites=%llu edges=%llu evaluations=%u ")
														TEXT("route=%u error=%u firstReachableDry=%llu firstStation=%llu length=%.9gm. Landscape unaccepted."),
														static_cast<unsigned long long>(Pass),
														static_cast<unsigned long long>(FanGraph->Sites.size()),
														static_cast<unsigned long long>(AcyclicEdges.size()),
														FanGraph->Evaluations,
														static_cast<unsigned>(Route.has_value()),
														static_cast<unsigned>(RouteError),
														static_cast<unsigned long long>(FirstReachable),
														static_cast<unsigned long long>(FirstStation),
														Route ? Route->LengthMeters : -1.0
													)
												);
												if(Route) {
													double ReceiverWater = std::numeric_limits<double>::quiet_NaN();
													for(const PlanetCore::RiverRoutingReceiver &Receiver : FanReceivers) {
														if(Receiver.Node == Route->SelectedReceiver) ReceiverWater = Receiver.WaterMeters;
													}
													TestTrue(TEXT("選択河口が実水面所有の候補に一致"), std::isfinite(ReceiverWater));
													TestTrue(TEXT("元窪地の鞍点水位を保つ"), std::abs(Route->WaterMeters.front() - OutletPassHeightMeters) < 1E-6);
													TestTrue(TEXT("選択河口の実水位に終着"), std::abs(Route->WaterMeters.back() - ReceiverWater) < 1E-6);
													std::vector<PlanetCore::UnitDirection> RefinedDirections;
													std::vector<std::array<std::uint32_t, 2>> RefinedLinks;
													RefinedDirections.reserve(Route->Nodes.size());
													RefinedLinks.reserve(Route->Nodes.size() - 1);
													for(std::uint32_t k = 0; k < Route->Nodes.size(); ++k) {
														RefinedDirections.push_back(FanDirections[Route->Nodes[k]]);
														if(k) RefinedLinks.push_back({ k - 1, k });
													}
													const std::array<std::uint32_t, 2> RefinedOwned { 0, static_cast<std::uint32_t>(RefinedDirections.size() - 1) };
													std::vector<double> RefinedBankHalfWidths(RefinedLinks.size(), FanSettings.BankHalfWidthMeters);
													RefinedBankHalfWidths.back() = 5;
													PlanetCore::RiverRoutingTerrainSettings RefinedSettings = FanSettings;
													RefinedSettings.MaximumSampleStepMeters = 6.25;
													PlanetCore::RiverRoutingTerrainError RefinedError = PlanetCore::RiverRoutingTerrainError::None;
													std::optional<PlanetCore::RiverRoutingTerrainGraph> RefinedGraph = PlanetCore::PrepareRiverRoutingTerrainGraph(
														RefinedDirections,
														RefinedLinks,
														[&Field](const PlanetCore::UnitDirection Direction) {
															// 元の表示・衝突と同じ解析標高場を再標本化して返す
															return Field->EvaluateElevation(Direction);
														},
														RefinedSettings,
														RefinedError,
														{},
														RefinedOwned,
														RefinedBankHalfWidths
													);
													TestTrue(TEXT("選択河道を半刻みで独立に再標本化"), RefinedGraph.has_value());
													if(RefinedGraph) {
														RefinedGraph->Sites.front() = { OutletPassHeightMeters, OutletPassHeightMeters };
														RefinedGraph->Sites.back() = { ReceiverWater, ReceiverWater };
														std::vector<PlanetCore::RiverRoutingEdge> RefinedEdges;
														for(const PlanetCore::RiverRoutingEdge &Edge : RefinedGraph->Edges) {
															if(Edge.Upstream + 1 == Edge.Downstream) RefinedEdges.push_back(Edge);
														}
														PlanetCore::RiverRoutingError RefinedRouteError = PlanetCore::RiverRoutingError::None;
														PlanetCore::RiverRoutingDiagnostics RefinedReachability;
														const std::optional<PlanetCore::RiverRoutingResult> RefinedRoute = PlanetCore::FindRiverRoutingPath(
															RefinedGraph->Sites,
															RefinedEdges,
															0,
															RefinedOwned.back(),
															ReceiverWater,
															RefinedSettings.MinimumDownstreamSlope,
															RefinedRouteError,
															{},
															&RefinedReachability
														);
														if(!RefinedRoute) {
															std::size_t RefinedFirstReachable = RefinedDirections.size();
															for(std::size_t Node = 0; Node < RefinedReachability.MinimumRequiredWaterMeters.size(); ++Node) {
																if(!std::isfinite(RefinedReachability.MinimumRequiredWaterMeters[Node])) continue;
																RefinedFirstReachable = Node;
																break;
															}
															AddInfo(
																FString::Printf(
																	TEXT("Refined route blockage: first=%llu rejected=%u sourceRequired=%.12gm ")
																	TEXT("predecessorH=%.12gm currentH=%.12gm originalUpWater=%.12gm originalDownWater=%.12gm."),
																	static_cast<unsigned long long>(RefinedFirstReachable),
																	RefinedGraph->RejectedUndirectedEdges,
																	RefinedReachability.MinimumRequiredWaterMeters.front(),
																	RefinedFirstReachable && RefinedFirstReachable < RefinedDirections.size() ?
																	Field->EvaluateElevation(RefinedDirections[RefinedFirstReachable - 1]) :
																	-1.0,
																	RefinedFirstReachable < RefinedDirections.size() ? Field->EvaluateElevation(RefinedDirections[RefinedFirstReachable]) : -1.0,
																	RefinedFirstReachable && RefinedFirstReachable < Route->WaterMeters.size() ?
																	Route->WaterMeters[RefinedFirstReachable - 1] :
																	-1.0,
																	RefinedFirstReachable < Route->WaterMeters.size() ? Route->WaterMeters[RefinedFirstReachable] : -1.0
																)
															);
														}
														TestTrue(TEXT("半刻みの中心・両岸・水位制約でも同じ河道が成立"), RefinedRoute.has_value());
														AddInfo(
															FString::Printf(
																TEXT("Refined saddle river route: selected=%u nodes=%llu edges=%llu evaluations=%llu ")
																TEXT("route=%u error=%u sourceWater=%.9gm receiverWater=%.9gm length=%.9gm. Final landscape unaccepted."),
																Route->SelectedReceiver,
																static_cast<unsigned long long>(RefinedGraph->Sites.size()),
																static_cast<unsigned long long>(RefinedEdges.size()),
																static_cast<unsigned long long>(RefinedGraph->Evaluations),
																static_cast<unsigned>(RefinedRoute.has_value()),
																static_cast<unsigned>(RefinedRouteError),
																RefinedRoute ? RefinedRoute->WaterMeters.front() : -1.0,
																ReceiverWater,
																RefinedRoute ? RefinedRoute->LengthMeters : -1.0
															)
														);
													}
												}
												if(Route || RouteError != PlanetCore::RiverRoutingError::NoRoute || FirstStation > DryIndex) break;
												if(!FirstStation) {
													for(const std::uint32_t Node : { 0U, 1U, SourceNode }) {
														const double Required = Reachability.MinimumRequiredWaterMeters[Node];
														AddInfo(
															FString::Printf(
																TEXT("Capped source endpoint: node=%u H=%.12gm required=%.12gm site=[%.12g,%.12g]."),
																Node,
																Field->EvaluateElevation(FanDirections[Node]),
																Required,
																FanGraph->Sites[Node].MinimumWaterMeters,
																FanGraph->Sites[Node].MaximumWaterMeters
															)
														);
													}
													for(const PlanetCore::RiverRoutingEdge &Edge : AcyclicEdges) {
														if(Edge.Upstream != SourceNode) continue;
														AddInfo(
															FString::Printf(
																TEXT("Capped source edge: to=%u length=%.12gm up=[%.12g,%.12g] down=[%.12g,%.12g]."),
																Edge.Downstream,
																Edge.LengthMeters,
																Edge.MinimumUpstreamWaterMeters,
																Edge.MaximumUpstreamWaterMeters,
																Edge.MinimumDownstreamWaterMeters,
																Edge.MaximumDownstreamWaterMeters
															)
														);
													}
													const std::vector<PlanetCore::UnitDirection> MouthDirections { FanDirections[SourceNode], FanDirections[1] };
													const std::vector<std::array<std::uint32_t, 2>> MouthEdges { { 0, 1 } };
													const std::array<std::uint32_t, 1> MouthOwned { 0 };
													for(const double HalfWidth : { 20.0, 40.0, 80.0, 160.0, 320.0, 640.0 }) {
														PlanetCore::RiverRoutingTerrainSettings MouthSettings;
														MouthSettings.RadiusMeters = Current.RadiusMeters;
														MouthSettings.BankHalfWidthMeters = HalfWidth;
														PlanetCore::RiverRoutingTerrainError MouthError = PlanetCore::RiverRoutingTerrainError::None;
														const std::optional<PlanetCore::RiverRoutingTerrainGraph> MouthGraph = PlanetCore::PrepareRiverRoutingTerrainGraph(
															MouthDirections,
															MouthEdges,
															[&Field](const PlanetCore::UnitDirection Direction) {
																return Field->EvaluateElevation(Direction);
															},
															MouthSettings,
															MouthError,
															{},
															MouthOwned
														);
														const PlanetCore::RiverRoutingEdge *OutletEdge = nullptr;
														if(MouthGraph) {
															for(const PlanetCore::RiverRoutingEdge &Edge : MouthGraph->Edges) if(!Edge.Upstream && Edge.Downstream == 1) OutletEdge = &Edge;
														}
														AddInfo(
															FString::Printf(
																TEXT("Lake mouth width trial: halfWidth=%.9gm graph=%u error=%u downstreamCeiling=%.12gm lakeWater=%.12gm."),
																HalfWidth,
																static_cast<unsigned>(MouthGraph.has_value()),
																static_cast<unsigned>(MouthError),
																OutletEdge ? OutletEdge->MaximumDownstreamWaterMeters : -1.0,
																OutletPassHeightMeters
															)
														);
													}
													break;
												}
												if(!AddLattice(FirstStation)) {
													for(std::size_t Lane = 0; Lane < 5; ++Lane) {
														const std::uint32_t Node = LaneNodes[HighestStation][Lane];
														const double Water = Node < Reachability.MinimumRequiredWaterMeters.size() ?
														Reachability.MinimumRequiredWaterMeters[Node] :
														std::numeric_limits<double>::infinity();
														AddInfo(
															FString::Printf(
																TEXT("High passage lane: lane=%llu node=%u H=%.9gm required=%.9gm site=[%.9g,%.9g]."),
																static_cast<unsigned long long>(Lane),
																Node,
																Field->EvaluateElevation(FanDirections[Node]),
																Water,
																FanGraph->Sites[Node].MinimumWaterMeters,
																FanGraph->Sites[Node].MaximumWaterMeters
															)
														);
													}
													for(std::uint32_t Station = 0; Station <= std::min<std::uint32_t>(DryIndex, 8); ++Station) {
														const double Required = Station < Reachability.MinimumRequiredWaterMeters.size() ?
														Reachability.MinimumRequiredWaterMeters[Station] :
														std::numeric_limits<double>::infinity();
														AddInfo(
															FString::Printf(
																TEXT("Source shoreline station: index=%u H=%.12gm site=[%.12g,%.12g] required=%.12gm."),
																Station,
																Field->EvaluateElevation(FanDirections[Station]),
																FanGraph->Sites[Station].MinimumWaterMeters,
																FanGraph->Sites[Station].MaximumWaterMeters,
																Required
															)
														);
													}
													for(const PlanetCore::RiverRoutingEdge &Edge : AcyclicEdges) {
														if(!(Edge.Upstream < 9 && Edge.Downstream < 9 || Edge.Upstream == SourceNode)) continue;
														AddInfo(
															FString::Printf(
																TEXT("Source shoreline edge: from=%u to=%u length=%.12gm ") TEXT("up=[%.12g,%.12g] down=[%.12g,%.12g]."),
																static_cast<unsigned>(Edge.Upstream),
																static_cast<unsigned>(Edge.Downstream),
																Edge.LengthMeters,
																Edge.MinimumUpstreamWaterMeters,
																Edge.MaximumUpstreamWaterMeters,
																Edge.MinimumDownstreamWaterMeters,
																Edge.MaximumDownstreamWaterMeters
															)
														);
													}
													break;
												}
											}
										}
									}
								}
							}
						}
						const auto AnalyzeProfile =
						[&](const TCHAR *const Label, const std::optional<PlanetCore::SpillPassage> &Passage, const double ReceiverWaterMeters) {
							if(Passage) {
								PlanetCore::RiverReceiverProfileSettings ProfileSettings;
								ProfileSettings.RadiusMeters = Current.RadiusMeters;
								ProfileSettings.MaximumSampleStepMeters = 25;
								ProfileSettings.MinimumDepthMeters = 0.3;
								ProfileSettings.MinimumDownstreamSlope = 1E-7;
								PlanetCore::RiverReceiverProfileError ProfileError = PlanetCore::RiverReceiverProfileError::None;
								const std::optional<PlanetCore::RiverReceiverProfile> Profile = PlanetCore::PrepareRiverReceiverProfile(
									Passage->Path,
									[&Field](const PlanetCore::UnitDirection Direction) {
										return Field->EvaluateElevation(Direction);
									},
									ReceiverWaterMeters,
									ProfileSettings,
									ProfileError
								);
								TestTrue(TEXT("実地形上の受入水位から下降縦断を準備"), Profile.has_value());
								if(Profile) {
									const PlanetCore::RiverReceiverProfileStatistics &Stats = Profile->Statistics;
									AddInfo(
										FString::Printf(
											TEXT("River receiver profile: kind=%s samples=%llu length=%.9gm minGround=%.9gm maxGround=%.9gm ")
											TEXT("crestAt=%.9gm sourceWater=%.9gm receiverWater=%.9gm maxDepth=%.9gm maxSlope=%.9g. Landscape unaccepted."),
											Label,
											static_cast<unsigned long long>(Profile->Samples.size()),
											Stats.LengthMeters,
											Stats.MinimumGroundMeters,
											Stats.MaximumGroundMeters,
											Stats.CrestDistanceMeters,
											Profile->Samples.front().WaterMeters,
											Profile->Samples.back().WaterMeters,
											Stats.MaximumWaterDepthMeters,
											Stats.MaximumDownstreamSlope
										)
									);
									if(
										!FCString::Strcmp(Label, TEXT("balanced")) || !FCString::Strcmp(Label, TEXT("bankAware")) ||
										!FCString::Strcmp(Label, TEXT("cappedBank"))
									) {
										std::uint32_t EmptyWindows = 0, BelowReceiverFloor = 0, BlockedRuns = 0;
										double FirstFloorCompatibleMeters = -1, LastBlockedMeters = -1, RunStartMeters = -1, LongestBlockedMeters = 0;
										const double BankAngle = 20 / Current.RadiusMeters;
										for(std::size_t i = 0; i < Profile->Samples.size(); ++i) {
											const PlanetCore::RiverReceiverProfileSample &Point = Profile->Samples[i];
											const PlanetCore::Vec3 P = Point.Direction.Vector();
											const PlanetCore::Vec3 Before = Profile->Samples[i ? i - 1 : i].Direction.Vector();
											const PlanetCore::Vec3 After = Profile->Samples[i + 1 < Profile->Samples.size() ? i + 1 : i].Direction.Vector();
											const PlanetCore::Vec3 AlongRaw = After - Before;
											const PlanetCore::Vec3 Along = PlanetCore::UnitDirection::FromVector(AlongRaw - P * AlongRaw.Dot(P))->Vector();
											const PlanetCore::Vec3 Across = P.Cross(Along);
											const PlanetCore::UnitDirection Left =
											*PlanetCore::UnitDirection::FromVector(P * std::cos(BankAngle) + Across * std::sin(BankAngle));
											const PlanetCore::UnitDirection Right =
											*PlanetCore::UnitDirection::FromVector(P * std::cos(BankAngle) - Across * std::sin(BankAngle));
											const double MinimumBank = std::min(Field->EvaluateElevation(Left), Field->EvaluateElevation(Right));
											const std::optional<PlanetCore::RiverRoutingWindow> Window =
											PlanetCore::MakeRiverRoutingWindow(Point.GroundMeters, MinimumBank, 0.3, 10, 20, 0.3);
											const bool Blocked = !Window || Window->MaximumWaterMeters < ReceiverWaterMeters;
											if(!Window) ++EmptyWindows;
											else if(Blocked) ++BelowReceiverFloor;
											else if(FirstFloorCompatibleMeters < 0) FirstFloorCompatibleMeters = Point.DistanceMeters;
											if(Blocked) {
												LastBlockedMeters = Point.DistanceMeters;
												if(RunStartMeters < 0) {
													RunStartMeters = Point.DistanceMeters;
													++BlockedRuns;
												}
											} else if(RunStartMeters >= 0) {
												LongestBlockedMeters = std::max(LongestBlockedMeters, LastBlockedMeters - RunStartMeters);
												RunStartMeters = -1;
											}
										}
										if(RunStartMeters >= 0) {
											const double TailLengthMeters = LastBlockedMeters - RunStartMeters;
											LongestBlockedMeters = std::max(LongestBlockedMeters, TailLengthMeters);
										}
										AddInfo(
											FString::Printf(
												TEXT("Receiver physical windows: kind=%s samples=%llu halfWidth=20m depthCap=10m cutCap=20m ")
												TEXT("empty=%u belowReceiverFloor=%u blockedRuns=%u firstFloorCompatible=%.9gm ")
												TEXT("lastBlocked=%.9gm longestBlocked=%.9gm. Finite necessary conditions only."),
												Label,
												static_cast<unsigned long long>(Profile->Samples.size()),
												EmptyWindows,
												BelowReceiverFloor,
												BlockedRuns,
												FirstFloorCompatibleMeters,
												LastBlockedMeters,
												LongestBlockedMeters
											)
										);
									}
									for(const double DepthCap : std::array<double, 3>{ 5, 10, 15 }) {
										PlanetCore::RiverReceiverConditioningSettings ConditionSettings;
										ConditionSettings.MinimumDepthMeters = ProfileSettings.MinimumDepthMeters;
										ConditionSettings.MaximumDepthMeters = DepthCap;
										ConditionSettings.MinimumDownstreamSlope = ProfileSettings.MinimumDownstreamSlope;
										ConditionSettings.MaximumCutMeters = 20;
										PlanetCore::RiverReceiverConditioningError ConditionError = PlanetCore::RiverReceiverConditioningError::None;
										PlanetCore::RiverReceiverConditioningDiagnostics Bounds;
										const std::optional<PlanetCore::RiverReceiverConditioning> Conditioned =
										PlanetCore::ConditionRiverReceiverProfile(Profile->Samples, ConditionSettings, ConditionError, &Bounds);
										AddInfo(
											FString::Printf(
												TEXT("River receiver conditioning: kind=%s depthCap=%.9gm found=%u error=%u ")
												TEXT("irreducibleDepth=%.9gm requiredCut=%.9gm achievedDepth=%.9gm actualCut=%.9gm ")
												TEXT("maxSlope=%.9g cutAreaPerWidth=%.9gm2. Landscape unaccepted."),
												Label,
												DepthCap,
												static_cast<unsigned>(Conditioned.has_value()),
												static_cast<unsigned>(ConditionError),
												Bounds.MinimumIrreducibleDepthMeters,
												Bounds.MinimumRequiredMaximumCutMeters,
												Conditioned ? Conditioned->MaximumWaterDepthMeters : -1,
												Conditioned ? Conditioned->MaximumActualCutMeters : -1,
												Conditioned ? Conditioned->MaximumDownstreamSlope : -1,
												Conditioned ? Conditioned->CutAreaPerUnitWidthSquareMeters : -1
											)
										);
										if(Conditioned) {
											TestTrue(TEXT("条件付けの全経路標本数を維持"), Conditioned->Samples.size() == Profile->Samples.size());
											TestTrue(TEXT("条件付けの最大水深を守る"), Conditioned->MaximumWaterDepthMeters <= DepthCap + 1E-9);
											TestTrue(TEXT("条件付けの最大開削を守る"), Conditioned->MaximumActualCutMeters <= 20 + 1E-9);
											TestTrue(TEXT("条件付けの受入水位を守る"), Conditioned->Samples.back().WaterMeters == ReceiverWaterMeters);
											const bool SurveyBanks = DepthCap == 10 && (
												FCString::Strcmp(Label, TEXT("balanced")) == 0 || FCString::Strcmp(Label, TEXT("bankAware")) == 0 ||
												FCString::Strcmp(Label, TEXT("cappedBank")) == 0
											) || DepthCap == 5 && FCString::Strcmp(Label, TEXT("target373")) == 0;
											if(SurveyBanks) {
												PlanetCore::RiverReceiverBankSettings BankSettings;
												BankSettings.RadiusMeters = Current.RadiusMeters;
												PlanetCore::RiverReceiverBankError BankError = PlanetCore::RiverReceiverBankError::None;
												const std::optional<PlanetCore::RiverReceiverBankStatistics> Banks = PlanetCore::SurveyRiverReceiverBanks(
													Conditioned->Samples,
													[&Field](const PlanetCore::UnitDirection Direction) {
														// 表示・衝突と同じ最終地面を横断面に返す
														return Field->EvaluateElevation(Direction);
													},
													BankSettings,
													BankError
												);
												if(TestTrue(TEXT("実地形の河岸を有限幅で測定"), Banks.has_value())) {
													AddInfo(
														FString::Printf(
															TEXT("River receiver bank survey: kind=%s depthCap=%.9gm stations=%u ")
															TEXT("halfWidths=20,100,500m belowFreeboardSides=%u,%u,%u submergedSides=%u,%u,%u ")
															TEXT("maxBankDeficit=%.9g,%.9g,%.9gm. Landscape unaccepted."),
															Label,
															DepthCap,
															Banks->Stations,
															Banks->BelowFreeboardSides[0],
															Banks->BelowFreeboardSides[1],
															Banks->BelowFreeboardSides[2],
															Banks->SubmergedSides[0],
															Banks->SubmergedSides[1],
															Banks->SubmergedSides[2],
															Banks->MaximumFreeboardDeficitMeters[0],
															Banks->MaximumFreeboardDeficitMeters[1],
															Banks->MaximumFreeboardDeficitMeters[2]
														)
													);
												}
											}
										}
									}
								}
							}
						};
						AnalyzeProfile(TEXT("crest"), Corridor, FeasibleReceiver->Site.WaterMeters);
						AnalyzeProfile(TEXT("balanced"), BalancedCorridor, FeasibleReceiver->Site.WaterMeters);
						AnalyzeProfile(TEXT("bankAware"), BankAwareCorridor, FeasibleReceiver->Site.WaterMeters);
						AnalyzeProfile(TEXT("cappedBank"), CappedBankCorridor, FeasibleReceiver->Site.WaterMeters);
						for(const double TrialGroundMeters : std::array<double, 3>{ 371, 372, 373 }) {
							const std::optional<PlanetCore::SpillPassage> Trial = PlanetCore::FindSpillPassage(
								OriginMinimum->Direction,
								FeasibleReceiver->Site.Direction,
								[&Field, TrialGroundMeters](const PlanetCore::UnitDirection Direction) {
									const PlanetCore::TerrainSample Ground = Field->Evaluate(Direction);
									const double Difference = Ground.ElevationMeters - TrialGroundMeters, Cost = std::hypot(Difference, 0.01);
									// 試験用の標高帯からの滑らかな偏差を返す
									return PlanetCore::TerrainSample{ Cost, Ground.TangentialGradientPerRadian * (Difference / Cost) };
								},
								CorridorSettings
							);
							AddInfo(
								FString::Printf(
									TEXT("River receiver target trial: target=%.9gm found=%u path=%.9gm evaluations=%llu. Landscape unaccepted."),
									TrialGroundMeters,
									static_cast<unsigned>(Trial.has_value()),
									Trial ? Trial->PathLengthMeters : -1,
									static_cast<unsigned long long>(Trial ? Trial->Evaluations : 0)
								)
							);
							const FString TrialLabel = FString::Printf(TEXT("target%.0f"), TrialGroundMeters);
							AnalyzeProfile(*TrialLabel, Trial, FeasibleReceiver->Site.WaterMeters);
						}
						const std::optional<PlanetCore::RiverReceiverNearest> LowerReceiver =
						Receivers->FindNearest(OriginMinimum->Direction, OriginMinimum->ElevationMeters + 2);
						if(LowerReceiver) {
							constexpr double LowerTargetGroundMeters = 370;
							const std::optional<PlanetCore::SpillPassage> LowerPassage = PlanetCore::FindSpillPassage(
								OriginMinimum->Direction,
								LowerReceiver->Site.Direction,
								[&Field](const PlanetCore::UnitDirection Direction) {
									const PlanetCore::TerrainSample Ground = Field->Evaluate(Direction);
									const double Difference = Ground.ElevationMeters - LowerTargetGroundMeters, Cost = std::hypot(Difference, 0.01);
									// 水位の低い受入先に向かう標高帯費用の返戻
									return PlanetCore::TerrainSample{ Cost, Ground.TangentialGradientPerRadian * (Difference / Cost) };
								},
								CorridorSettings
							);
							AddInfo(
								FString::Printf(
									TEXT("Lower-stage river corridor: reach=%u receiverWater=%.9gm found=%u path=%.9gm ")
									TEXT("evaluations=%llu. Landscape unaccepted."),
									LowerReceiver->Site.Reach,
									LowerReceiver->Site.WaterMeters,
									static_cast<unsigned>(LowerPassage.has_value()),
									LowerPassage ? LowerPassage->PathLengthMeters : -1,
									static_cast<unsigned long long>(LowerPassage ? LowerPassage->Evaluations : 0)
								)
							);
							AnalyzeProfile(TEXT("lowerStage370"), LowerPassage, LowerReceiver->Site.WaterMeters);
						}
					}
				}
				AddInfo(
					FString::Printf(
						TEXT("Water-limited river receiver: seed=%llu pass=%.9gm found=%u reach=%u ")
						TEXT("distance=%.9gm water=%.9gm ground=%.9gm. Landscape unaccepted."),
						static_cast<unsigned long long>(Current.Seed),
						OutletPassHeightMeters,
						static_cast<unsigned>(FeasibleReceiver.has_value()),
						FeasibleReceiver ? FeasibleReceiver->Site.Reach : UINT32_MAX,
						FeasibleReceiver ? FeasibleReceiver->DistanceMeters : -1,
						FeasibleReceiver ? FeasibleReceiver->Site.WaterMeters : 0,
						FeasibleReceiver ? FeasibleReceiver->Site.GroundMeters : 0
					)
				);
				for(unsigned Branch = 0; Branch < 2; ++Branch) {
					const PlanetCore::SpillConnectionBranch &End = Outlet->Branches[Branch];
					OutletBranchKinds[Branch] = static_cast<unsigned>(End.Kind);
					if(!End.Minimum) {
						OutletBranchHeightsMeters[Branch] = FillSettings.SeaLevelMeters;
						continue;
					}
					OutletBranchHeightsMeters[Branch] = End.Minimum->ElevationMeters;
					const PlanetCore::Vec3 A = OriginMinimum->Direction.Vector(), B = End.Minimum->Direction.Vector();
					OutletBranchOriginDistancesMeters[Branch] = Current.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
				}
				TestTrue(
					TEXT("見つけた峠の片側が元格子点の実極小"),
					OutletBranchOriginDistancesMeters[0] >= 0 && OutletBranchOriginDistancesMeters[0] < 1 ||
					OutletBranchOriginDistancesMeters[1] >= 0 && OutletBranchOriginDistancesMeters[1] < 1
				);
				const unsigned OriginBranch = OutletBranchOriginDistancesMeters[0] >= 0 && OutletBranchOriginDistancesMeters[0] < 1 ? 0 : 1;
				const std::optional<PlanetCore::TerrainCriticalPoint> &ReceivingMinimum = Outlet->Branches[1 - OriginBranch].Minimum;
				if(ReceivingMinimum) {
					const std::array<PlanetCore::TerrainCriticalPoint, 2> Known { *OriginMinimum, *ReceivingMinimum };
					PlanetCore::SpillDiscoveryDiagnostics SecondDiagnostics;
					const std::optional<PlanetCore::SpillConnection> Second = PlanetCore::FindSpillComponentExit(
						Known,
						[&Field](const PlanetCore::UnitDirection Direction) {
							return Field->Evaluate(Direction);
						},
						OutletSettings,
						{},
						&SecondDiagnostics,
						std::span<const PlanetCore::SpillConnection>(&*Outlet, 1)
					);
					SecondOutletFailure = static_cast<unsigned>(SecondDiagnostics.Failure);
					SecondOutletWindows = SecondDiagnostics.Windows;
					SecondOutletEvaluations = SecondDiagnostics.Evaluations;
					if(Second) {
						SecondOutletPassHeightMeters = Second->Pass.ElevationMeters;
						for(unsigned Branch = 0; Branch < 2; ++Branch) {
							const PlanetCore::SpillConnectionBranch &End = Second->Branches[Branch];
							SecondOutletBranchKinds[Branch] = static_cast<unsigned>(End.Kind);
							if(!End.Minimum) {
								SecondOutletBranchHeightsMeters[Branch] = FillSettings.SeaLevelMeters;
								continue;
							}
							SecondOutletBranchHeightsMeters[Branch] = End.Minimum->ElevationMeters;
							for(const PlanetCore::TerrainCriticalPoint &Site : Known) {
								const PlanetCore::Vec3 A = Site.Direction.Vector(), B = End.Minimum->Direction.Vector();
								const double Distance = Current.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
								if(SecondOutletBranchKnownDistancesMeters[Branch] < 0 || Distance < SecondOutletBranchKnownDistancesMeters[Branch]) {
									SecondOutletBranchKnownDistancesMeters[Branch] = Distance;
								}
							}
						}
					}
				}
				if(Current.Seed == 7 && Current.Mode == Mode::MesoOnly) {
					std::vector<std::pair<double, std::uint32_t>> OceanCandidates;
					const PlanetCore::Vec3 Start = Outlet->Pass.Direction.Vector();
					for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
						if(Heights[Index] >= FillSettings.SeaLevelMeters - 10) continue;
						const PlanetCore::Vec3 Delta = Mesh->Directions[Index].Vector() - Start;
						OceanCandidates.emplace_back(Delta.Dot(Delta), Index);
					}
					std::sort(OceanCandidates.begin(), OceanCandidates.end());
					std::vector<PlanetCore::UnitDirection> OceanAnchors;
					for(const auto &[Distance, Index] : OceanCandidates) {
						static_cast<void>(Distance);
						const PlanetCore::UnitDirection Anchor = Mesh->Directions[Index];
						if(Ocean->EvaluateElevation(Anchor) >= FillSettings.SeaLevelMeters - 10) continue;
						OceanAnchors.push_back(Anchor);
						if(OceanAnchors.size() == 16) break;
					}
					const auto IsOceanAnchor = [&OceanAnchors, &Ocean](const PlanetCore::UnitDirection Query) {
						const PlanetCore::Vec3 A = Query.Vector();
						for(const PlanetCore::UnitDirection Anchor : OceanAnchors) {
							const PlanetCore::Vec3 B = Anchor.Vector();
							// 同じ海域標本の所属だけを受理する返戻
							if(A.X == B.X && A.Y == B.Y && A.Z == B.Z) return Ocean->Statistics().ComponentsAfter == 1;
						}
						// 未確認の海域標本への返戻
						return false;
					};
					PlanetCore::SpillNetworkSettings NetworkSettings;
					NetworkSettings.RadiusMeters = Current.RadiusMeters;
					NetworkSettings.SeaLevelMeters = FillSettings.SeaLevelMeters;
					NetworkSettings.MaximumEvaluations = 500000;
					NetworkSettings.MaximumSteps = 64;
					NetworkSettings.MaximumConnections = 128;
					NetworkSettings.MaximumBranchSamples = 65536;
					PlanetCore::SpillNetworkDiagnostics NetworkDiagnostics;
					const std::optional<PlanetCore::SpillNetwork> Network = PlanetCore::PrepareSpillNetwork(
						Outlet->Pass.Direction,
						[&Field](const PlanetCore::UnitDirection Direction) {
							return Field->Evaluate(Direction);
						},
						OceanAnchors,
						IsOceanAnchor,
						NetworkSettings,
						{},
						&NetworkDiagnostics
					);
					CorrectOriginNetworkFailure = static_cast<unsigned>(NetworkDiagnostics.Failure);
					CorrectOriginNetworkSteps = NetworkDiagnostics.Steps;
					CorrectOriginNetworkMinima = NetworkDiagnostics.PreparedMinima;
					CorrectOriginNetworkConnections = NetworkDiagnostics.PreparedConnections;
					CorrectOriginNetworkCenterQueries = NetworkDiagnostics.CenterQueries;
					CorrectOriginNetworkEvaluations = NetworkDiagnostics.Evaluations;
					CorrectOriginNetworkOceanDistanceMeters = NetworkDiagnostics.ClosestOceanDistanceMeters;
					AddInfo(
						FString::Printf(
							TEXT("Correct origin spill network: success=%u failure=%u steps=%u minima=%u connections=%u ")
							TEXT("centerQueries=%u evaluations=%llu closestOcean=%.9gm anchors=%llu. Landscape unaccepted."),
							static_cast<unsigned>(Network.has_value()),
							CorrectOriginNetworkFailure,
							CorrectOriginNetworkSteps,
							CorrectOriginNetworkMinima,
							CorrectOriginNetworkConnections,
							CorrectOriginNetworkCenterQueries,
							static_cast<unsigned long long>(CorrectOriginNetworkEvaluations),
							CorrectOriginNetworkOceanDistanceMeters,
							static_cast<unsigned long long>(OceanAnchors.size())
						)
					);
				}
			}
		}
		const double OceanShare =
		static_cast<double>(Filled->GetStatistics().OceanVertices) / static_cast<double>(Mesh->Directions.size());
		TestTrue(TEXT("共通網範囲比較の海陸比"), OceanShare >= 0.6 && OceanShare <= 0.7);
		PlanetCore::SphericalDrainageBaselineError BaselineError = PlanetCore::SphericalDrainageBaselineError::None;
		const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
			Mesh->Directions,
			Mesh->Triangles,
			Heights,
			FillSettings.SeaLevelMeters,
			BaselineError
		);
		double FlexibleMaximum = -1;
		const bool CompareFlexible = Current.Mode == Mode::MesoOnly && (Current.Seed == 7 || Current.Seed == 44) ||
		Current.Mode == Mode::MesoAndLand && Current.Seed == 42;
		if(CompareFlexible) {
			// 元の海陸区分を保持し，各許容幅ごとに大洋根の流路を選び直す
			std::vector<double> Lowered(Heights.size());
			double Low = 0, High = Filled->GetStatistics().MaximumRaiseMeters + 0.0001;
			for(unsigned Step = 0; Step < 28; ++Step) {
				const double Bound = std::midpoint(Low, High);
				for(std::size_t Index = 0; Index < Heights.size(); ++Index) {
					Lowered[Index] = Heights[Index] < FillSettings.SeaLevelMeters ?
					Heights[Index] :
					std::max(std::nextafter(FillSettings.SeaLevelMeters, std::numeric_limits<double>::infinity()), Heights[Index] - Bound);
				}
				PlanetCore::MinimalDrainageFillError FlexibleError = PlanetCore::MinimalDrainageFillError::None;
				const std::optional<PlanetCore::MinimalDrainageFill> Candidate =
				PlanetCore::MinimalDrainageFill::Create(Mesh->Directions, Mesh->Triangles, Lowered, FillSettings, FlexibleError);
				// 比較の一条件でも計算不能なら，実現可能と誤認しない返戻
				if(!TestTrue(TEXT("可変流路の下界に対する最小盛土を準備"), Candidate.has_value())) return false;
				bool WithinBound = true;
				for(std::size_t Index = 0; Index < Heights.size(); ++Index) {
					if(Candidate->GetHeightsMeters()[Index] <= Heights[Index] + Bound) continue;
					WithinBound = false;
					break;
				}
				if(WithinBound) High = Bound;
				else Low = Bound;
			}
			FlexibleMaximum = High;
		}
		std::uint64_t RiverSamples = 0, UnsupportedCenters = 0, ExposedCenters = 0, WaterRises = 0;
		double MaximumExposureMeters = 0, MaximumWaterRiseMeters = 0;
		bool RiverSamplesValid = true;
		for(std::size_t Reach = 0; Reach < Field->Drainage().Reaches().size(); ++Reach) {
			double PreviousWater = std::numeric_limits<double>::infinity();
			for(unsigned Knot = 0; Knot < 9; ++Knot) {
				const std::optional<PlanetCore::RiverProfileSample> Profile = Field->Drainage().SampleReach(Reach, Knot / 8.0);
				if(!Profile) {
					RiverSamplesValid = false;
					break;
				}
				const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector(Profile->Direction);
				if(!Direction) {
					RiverSamplesValid = false;
					break;
				}
				const PlanetCore::RiverWaterHeights Water = Field->EvaluateWater(*Direction);
				if(
					!std::isfinite(Water.WaterElevationMeters) || !std::isfinite(Water.TerrainElevationMeters) || !std::isfinite(Water.WaterSupport)
				) {
					RiverSamplesValid = false;
					break;
				}
				if(Water.WaterSupport < 0.99) ++UnsupportedCenters;
				const double Exposure = Water.TerrainElevationMeters - Water.WaterElevationMeters;
				if(Exposure > 0.01) ++ExposedCenters;
				MaximumExposureMeters = std::max(MaximumExposureMeters, Exposure);
				const double Rise = Water.WaterElevationMeters - PreviousWater;
				if(Rise > 0.000001) ++WaterRises;
				MaximumWaterRiseMeters = std::max(MaximumWaterRiseMeters, Rise);
				PreviousWater = Water.WaterElevationMeters;
				++RiverSamples;
			}
			if(!RiverSamplesValid) break;
		}
		// 一地点でも不正なら部分的な集計を成功として公開しない返戻
		if(!TestTrue(TEXT("全河川中心線の標本が有限"), RiverSamplesValid)) return false;
		const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
		AddInfo(
			FString::Printf(
				TEXT("Shared predrainage range: seed=%llu radius=%.9gm mode=%u graph=%u components=%u ")
				TEXT("oceanShare=%.9g areaShare=%.9g independentShare=%.9g graphOcean=%u baseline=%u ")
				TEXT("baselineReason=%u changed=%u basinComponents=%u deepestVertices=%u deepestSelected=%u ")
				TEXT("deepestNode=%u criticalKind=%d criticalFailure=%u criticalOffset=%.9gm ")
				TEXT("nearbyMinima=%u nearbySaddles=%u nearbyMaxima=%u nearbyRejected=%u nearestMinDistance=%.9gm nearestMinHeight=%.9gm ")
				TEXT("outletFailure=%u outletWindows=%u outletEvaluations=%llu outletPassHeight=%.9gm ")
				TEXT("outletBranch0Kind=%u outletBranch0Height=%.9gm outletBranch0OriginDistance=%.9gm ")
				TEXT("outletBranch1Kind=%u outletBranch1Height=%.9gm outletBranch1OriginDistance=%.9gm ")
				TEXT("secondOutletFailure=%u secondOutletWindows=%u secondOutletEvaluations=%llu secondOutletPass=%.9gm ")
				TEXT("secondBranch0Kind=%u secondBranch0Height=%.9gm secondBranch0KnownDistance=%.9gm ")
				TEXT("secondBranch1Kind=%u secondBranch1Height=%.9gm secondBranch1KnownDistance=%.9gm ")
				TEXT("routeSteps=%u routeLength=%.9gm receiverKind=%u receiverDistance=%.9gm receiverWater=%.9gm ")
				TEXT("sourceHeight=%.9gm receiverGround=%.9gm barrierBeforeReceiver=%.9gm routeBarrier=%.9gm ")
				TEXT("altSettled=%u altReceiver=%u altBarrier=%.9gm altLength=%.9gm altWater=%.9gm ")
				TEXT("originTraceEnd=%u originTraceSamples=%u originTraceLength=%.9gm originTraceHeight=%.9gm ")
				TEXT("originCriticalKind=%u originCriticalFailure=%u originCriticalHeight=%.9gm originCriticalDistance=%.9gm ")
				TEXT("maxRaise=%.9gm sumRaise=%.9gm flexibleBound=%.9gm riverSamples=%llu ")
				TEXT("unsupported=%llu exposed=%llu waterRises=%llu maxExposure=%.9gm maxRise=%.9gm ")
				TEXT("seconds=%.9g. Landscape unaccepted."),
				static_cast<unsigned long long>(Current.Seed),
				Current.RadiusMeters,
				static_cast<unsigned>(Current.Mode),
				Ocean->Statistics().GraphNodes,
				Ocean->Statistics().ComponentsAfter,
				OceanShare,
				AreaFraction,
				IndependentFraction,
				static_cast<unsigned>(AllNodesReachOcean && SelectedReachesConnected),
				static_cast<unsigned>(Baseline.has_value()),
				static_cast<unsigned>(BaselineError),
				Filled->GetStatistics().RaisedLandVertices,
				RaisedComponents,
				DeepestComponentVertices,
				DeepestSelectedVertices,
				DeepestNode,
				CriticalKind,
				CriticalFailure,
				CriticalDisplacementMeters,
				NearbyMinima,
				NearbySaddles,
				NearbyMaxima,
				NearbyRejected,
				NearestMinimumDistanceMeters,
				NearestMinimumHeightMeters,
				OutletFailure,
				OutletWindows,
				static_cast<unsigned long long>(OutletEvaluations),
				OutletPassHeightMeters,
				OutletBranchKinds[0],
				OutletBranchHeightsMeters[0],
				OutletBranchOriginDistancesMeters[0],
				OutletBranchKinds[1],
				OutletBranchHeightsMeters[1],
				OutletBranchOriginDistancesMeters[1],
				SecondOutletFailure,
				SecondOutletWindows,
				static_cast<unsigned long long>(SecondOutletEvaluations),
				SecondOutletPassHeightMeters,
				SecondOutletBranchKinds[0],
				SecondOutletBranchHeightsMeters[0],
				SecondOutletBranchKnownDistancesMeters[0],
				SecondOutletBranchKinds[1],
				SecondOutletBranchHeightsMeters[1],
				SecondOutletBranchKnownDistancesMeters[1],
				RouteSteps,
				RouteDistanceMeters,
				FirstReceiverKind,
				FirstReceiverDistanceMeters,
				FirstReceiverWaterMeters,
				SourceHeightMeters,
				FirstReceiverGroundMeters,
				BarrierBeforeReceiverMeters,
				MaximumRouteBarrierMeters,
				AlternativeSettled,
				AlternativeReceiver,
				AlternativeBarrierMeters,
				AlternativeLengthMeters,
				AlternativeWaterMeters,
				OriginTraceEnd,
				OriginTraceSamples,
				OriginTraceLengthMeters,
				OriginTraceHeightMeters,
				OriginCriticalKind,
				OriginCriticalFailure,
				OriginCriticalHeightMeters,
				OriginCriticalDistanceMeters,
				Filled->GetStatistics().MaximumRaiseMeters,
				Filled->GetStatistics().SumRaiseMeters,
				FlexibleMaximum,
				static_cast<unsigned long long>(RiverSamples),
				static_cast<unsigned long long>(UnsupportedCenters),
				static_cast<unsigned long long>(ExposedCenters),
				static_cast<unsigned long long>(WaterRises),
				MaximumExposureMeters,
				MaximumWaterRiseMeters,
				Seconds
			)
		);
	}
	// 有限網の比較を連続面の完成証明に置き換えない
	return true;
}

#endif
