#include "Core/PlanetOceanDistanceTerrain.h"
#include "Core/PlanetSphericalContinentalField.h"
#include "Core/PlanetSphericalDrainageBaseline.h"
#include "Core/PlanetSphericalMesh.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetOceanDistanceTerrainContracts,
	"PlanetGenerator.OceanDistanceTerrain.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 大洋根から先に決めた標高の全陸下降と海底保存を複数半径で検査する
 * @param Parameters 未使用の UE 引数
 * @return 有限網の厳密下降・拒否条件の結果，C1 又は景観の完成とは別
 */
bool FPlanetOceanDistanceTerrainContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::OceanDistanceTerrain;
	using PlanetCore::OceanDistanceTerrainError;
	using PlanetCore::OceanDistanceTerrainSettings;
	using PlanetCore::SphereMesh;
	using PlanetCore::SphericalContinentalField;
	using PlanetCore::SphericalContinentalFieldError;
	using PlanetCore::SphericalDrainageBaseline;
	using PlanetCore::SphericalDrainageBaselineError;
	using PlanetCore::UnitDirection;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const std::optional<SphereMesh> Mesh = PlanetCore::BuildUniformSphere(4);
	// 比較する閉球面網が無い場合の返戻
	if(!TestTrue(TEXT("大洋距離の全球網を準備"), Mesh.has_value())) return false;
	const std::array<std::array<double, 2>, 4> Cases {
		{ { { 7, 1000 } }, { { 42, 1E6 } }, { { 137, 6371000 } }, { { 2026, 1E9 } } }
	};
	std::uint64_t CheckedVertices = 0, CheckedLandRoutes = 0;
	double LowestOceanRatio = 1, HighestOceanRatio = 0, MaximumWeightedDistance = 0;
	double MinimumLandDrop = std::numeric_limits<double>::infinity();
	for(const std::array<double, 2> &Case : Cases) {
		const std::uint64_t Seed = static_cast<std::uint64_t>(Case[0]), Radius = static_cast<std::uint64_t>(Case[1]);
		SphericalContinentalFieldError FieldError = SphericalContinentalFieldError::None;
		const std::optional<SphericalContinentalField> Source = SphericalContinentalField::CreateSeeded(Seed, Radius, FieldError);
		// 連結大洋を持つ実地形の海陸分類が無い場合の返戻
		if(!TestTrue(TEXT("実Seedの海陸場を準備"), Source.has_value())) return false;
		std::vector<double> Original, Costs;
		Original.reserve(Mesh->Directions.size());
		Costs.reserve(Mesh->Directions.size());
		for(const UnitDirection Direction : Mesh->Directions) {
			const std::optional<double> Height = Source->EvaluateElevation(Direction);
			// 元の海底と海陸の値が欠ける場合の返戻
			if(!TestTrue(TEXT("元地形の有限標高"), Height && std::isfinite(*Height))) return false;
			Original.push_back(*Height);
			const PlanetCore::Vec3 Point = Direction.Vector();
			Costs.push_back(0.4 + std::exp(0.55 * std::sin(9 * Point.X + 6 * Point.Y) * std::cos(7 * Point.Z - 3 * Point.X)));
		}
		OceanDistanceTerrainSettings Settings;
		Settings.RadiusMeters = static_cast<double>(Radius);
		Settings.MaximumReliefMeters = std::min(5000.0, 0.1 * Settings.RadiusMeters);
		OceanDistanceTerrainError Error = OceanDistanceTerrainError::None;
		const std::optional<OceanDistanceTerrain> Terrain =
		OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Original, Costs, Settings, Error);
		if(!TestTrue(TEXT("大洋先行の地形を準備"), Terrain.has_value())) {
			AddError(
				FString::Printf(
					TEXT("Ocean distance terrain reason=%u seed=%llu."),
					static_cast<unsigned>(Error),
					static_cast<unsigned long long>(Seed)
				)
			);
			// 不完全な流路へ進まない返戻
			return false;
		}
		const std::optional<OceanDistanceTerrain> Repeated =
		OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Original, Costs, Settings, Error);
		// 同じ定義の再準備が無い場合の返戻
		if(!TestTrue(TEXT("同一定義の再準備"), Repeated.has_value())) return false;
		const std::span<const double> Heights = Terrain->GetHeightsMeters();
		const std::span<const std::uint32_t> Downstream = Terrain->GetDownstreamVertices();
		for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
			TestTrue(
				TEXT("同一入力の標高と下流がビット一致"),
				std::bit_cast<std::uint64_t>(Heights[Index]) == std::bit_cast<std::uint64_t>(Repeated->GetHeightsMeters()[Index]) &&
				Downstream[Index] == Repeated->GetDownstreamVertices()[Index]
			);
			if(Original[Index] < Settings.SeaLevelMeters) {
				TestEqual(TEXT("元の海底標高を保存"), Heights[Index], Original[Index]);
				continue;
			}
			const std::uint32_t Parent = Downstream[Index];
			// 全陸節点が範囲内の厳密に低い地点を持たない場合の返戻
			if(!TestTrue(TEXT("陸の下流が厳密下降"), Parent < Heights.size() && Heights[Parent] < Heights[Index])) return false;
			++CheckedLandRoutes;
		}
		SphericalDrainageBaselineError BaselineError = SphericalDrainageBaselineError::None;
		const std::optional<SphericalDrainageBaseline> Baseline =
		SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Heights, Settings.SeaLevelMeters, BaselineError);
		// 面内と全共有辺の連続な下降を確かめられない場合の返戻
		if(!TestTrue(TEXT("同じ最終Hの区分線形全球排水"), Baseline.has_value())) return false;
		const double OceanRatio = static_cast<double>(Terrain->GetStatistics().OceanVertices) / Mesh->Directions.size();
		// Earth 型の海陸比と大洋接続を外した場合の返戻
		if(!TestTrue(TEXT("海域が６～７割"), OceanRatio >= 0.6 && OceanRatio <= 0.7)) return false;
		LowestOceanRatio = std::min(LowestOceanRatio, OceanRatio);
		HighestOceanRatio = std::max(HighestOceanRatio, OceanRatio);
		MaximumWeightedDistance = std::max(MaximumWeightedDistance, Terrain->GetStatistics().MaximumWeightedDistanceMeters);
		MinimumLandDrop = std::min(MinimumLandDrop, Terrain->GetStatistics().MinimumLandDropMeters);
		CheckedVertices += Mesh->Directions.size();
	}
	OceanDistanceTerrainSettings Settings;
	std::vector<double> Heights(Mesh->Directions.size(), 1), Costs(Mesh->Directions.size(), 1);
	OceanDistanceTerrainError Error = OceanDistanceTerrainError::None;
	TestFalse(
		TEXT("大洋の無い入力を拒否"),
		OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Heights, Costs, Settings, Error).has_value()
	);
	TestTrue(TEXT("大洋の無い理由"), Error == OceanDistanceTerrainError::NoOcean);
	std::fill(Heights.begin(), Heights.end(), -1);
	TestFalse(
		TEXT("陸の無い入力を拒否"),
		OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Heights, Costs, Settings, Error).has_value()
	);
	TestTrue(TEXT("陸の無い理由"), Error == OceanDistanceTerrainError::NoLand);
	std::fill(Heights.begin(), Heights.end(), 1);
	const PlanetCore::Vec3 First = Mesh->Directions[0].Vector();
	std::uint32_t Opposite = 1;
	for(std::uint32_t Index = 2; Index < Mesh->Directions.size(); ++Index) {
		if(Mesh->Directions[Index].Vector().Dot(First) < Mesh->Directions[Opposite].Vector().Dot(First)) Opposite = Index;
	}
	Heights[0] = -1;
	Heights[Opposite] = -1;
	TestFalse(
		TEXT("孤立した二海域を拒否"),
		OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Heights, Costs, Settings, Error).has_value()
	);
	TestTrue(TEXT("孤立海域の理由"), Error == OceanDistanceTerrainError::DisconnectedOcean);
	Costs[0] = 0;
	TestFalse(
		TEXT("非正の費用を拒否"),
		OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Heights, Costs, Settings, Error).has_value()
	);
	TestTrue(TEXT("非正費用の理由"), Error == OceanDistanceTerrainError::InvalidInput);
	Costs[0] = 1;
	Settings.RadiusMeters = 999;
	TestFalse(
		TEXT("最小半径未満を拒否"),
		OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Heights, Costs, Settings, Error).has_value()
	);
	TestTrue(TEXT("半径の拒否理由"), Error == OceanDistanceTerrainError::InvalidInput);
	std::stop_source Cancelled;
	Cancelled.request_stop();
	Settings.RadiusMeters = 1E6;
	TestFalse(
		TEXT("中断を拒否"),
		OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Heights, Costs, Settings, Error, Cancelled.get_token())
		.has_value()
	);
	TestTrue(TEXT("中断の理由"), Error == OceanDistanceTerrainError::Cancelled);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Ocean distance terrain: planets=4 vertices=%llu landRoutes=%llu oceanFraction=%.6f..%.6f ")
			TEXT("maximumWeightedDistance=%.9gm minimumLandDrop=%.9gm seconds=%.9g. PL reference only."),
			static_cast<unsigned long long>(CheckedVertices),
			static_cast<unsigned long long>(CheckedLandRoutes),
			LowestOceanRatio,
			HighestOceanRatio,
			MaximumWeightedDistance,
			MinimumLandDrop,
			Seconds
		)
	);
	// 排水先行の有限網を検証し，完成した景観と区別する返戻
	return true;
}

#endif
