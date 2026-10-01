#include "Core/PlanetOutletPotential.h"
#include "Core/PlanetOrogenicRidges.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetOutletPotentialContracts, "PlanetGenerator.OutletPotential.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 大洋零境界と正の源から作る固定網に閉鎖窪地が無い事を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を完了した場合に真
 */
bool FPlanetOutletPotentialContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Started = FPlatformTime::Seconds();
	std::array<PlanetCore::DrainageSite, 4> Chain;
	Chain[0].Ocean = true;
	const std::array<std::array<std::uint32_t, 2>, 3> ChainEdges{{{0, 1}, {1, 2}, {2, 3}}};
	const std::array<double, 4> ChainSource{0, 1, 1, 1};
	const std::optional<PlanetCore::PlanetOutletPotential> ChainField =
		PlanetCore::PlanetOutletPotential::Create(Chain, ChainEdges, ChainSource, 6);
	if(!TestTrue(TEXT("解析解を持つ一本道を解く"), ChainField.has_value())) return false;
	if(!TestTrue(TEXT("解析解の四地点と一致"), ChainField->Elevations()[0] == 0 &&
		std::abs(ChainField->Elevations()[1] - 3) < 1E-9 &&
		std::abs(ChainField->Elevations()[2] - 5) < 1E-9 &&
		std::abs(ChainField->Elevations()[3] - 6) < 1E-9)) return false;
	std::array<double, 4> BadSource{0, 1, 0, 1};
	if(!TestFalse(TEXT("零隆起源の陸を拒否"),
		PlanetCore::PlanetOutletPotential::Create(Chain, ChainEdges, BadSource, 6).has_value())) return false;
	const std::array<std::array<std::uint32_t, 2>, 2> IsolatedEdges{{{0, 1}, {2, 3}}};
	if(!TestFalse(TEXT("大洋へ繋がらない陸を拒否"),
		PlanetCore::PlanetOutletPotential::Create(Chain, IsolatedEdges, ChainSource, 6).has_value())) return false;
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
	if(!TestTrue(TEXT("地質場を準備"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::Create(
		std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	if(!TestTrue(TEXT("接続済みの海陸を準備"), Ocean.has_value())) return false;
	const std::optional<PlanetCore::PlanetOrogenicRidges> Ridges =
		PlanetCore::PlanetOrogenicRidges::Create(Ocean->Base());
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	if(!TestTrue(TEXT("山稜と全球試験網を準備"), Ridges.has_value() && Mesh.has_value())) return false;
	std::vector<PlanetCore::DrainageSite> Sites(Mesh->Directions.size());
	std::vector<double> UniformSource(Sites.size(), 1), OrogenicSource(Sites.size(), 1);
	std::vector<std::vector<std::uint32_t>> Neighbors(Sites.size());
	std::vector<std::array<std::uint32_t, 2>> Edges;
	Edges.reserve(Mesh->Triangles.size() * 3);
	for(std::size_t Index = 0; Index < Sites.size(); ++Index) {
		const PlanetCore::UnitDirection Direction = Mesh->Directions[Index];
		Sites[Index].Direction = Direction.Vector();
		Sites[Index].Ocean = Ocean->EvaluateElevation(Direction) <= Settings.SeaLevelMeters;
		OrogenicSource[Index] += Ridges->EvaluateElevation(Direction) / 100;
	}
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
		for(std::uint32_t Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
			Edges.push_back({std::min(A, B), std::max(A, B)});
			Neighbors[A].push_back(B);
			Neighbors[B].push_back(A);
		}
	}
	std::sort(Edges.begin(), Edges.end());
	Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
	const std::optional<PlanetCore::PlanetOutletPotential> Uniform =
		PlanetCore::PlanetOutletPotential::Create(Sites, Edges, UniformSource, 4000);
	const std::optional<PlanetCore::PlanetOutletPotential> Orogenic =
		PlanetCore::PlanetOutletPotential::Create(Sites, Edges, OrogenicSource, 4000);
	if(!TestTrue(TEXT("全球網で均一源と造山源を解く"), Uniform.has_value() && Orogenic.has_value())) return false;
	std::uint32_t LandCount = 0, NoDescent = 0;
	double MaximumDifference = 0;
	for(std::size_t Index = 0; Index < Sites.size(); ++Index) {
		const double Height = Orogenic->Elevations()[Index];
		if(Sites[Index].Ocean) {
			if(!TestTrue(TEXT("大洋零境界を維持"), Height == 0)) return false;
			continue;
		}
		++LandCount;
		bool Descent = false;
		for(const std::uint32_t Neighbor : Neighbors[Index]) if(Orogenic->Elevations()[Neighbor] < Height) {
			Descent = true;
			break;
		}
		if(!Descent) ++NoDescent;
		MaximumDifference = std::max(MaximumDifference, std::abs(Height - Uniform->Elevations()[Index]));
	}
	double MaximumEdgeSlope = 0, MaximumEdgeRiseMeters = 0;
	for(const std::array<std::uint32_t, 2> &Edge : Edges) {
		const double Rise = std::abs(Orogenic->Elevations()[Edge[0]] - Orogenic->Elevations()[Edge[1]]);
		const double Angle = std::acos(std::clamp(Sites[Edge[0]].Direction.Dot(Sites[Edge[1]].Direction), -1.0, 1.0));
		if(!TestTrue(TEXT("固定網の辺長が有効"), Angle > 0)) return false;
		MaximumEdgeRiseMeters = std::max(MaximumEdgeRiseMeters, Rise);
		MaximumEdgeSlope = std::max(MaximumEdgeSlope, Rise / (Settings.RadiusMeters * Angle));
	}
	if(!TestTrue(TEXT("全陸地点で海方向の下り隣接を持つ"), NoDescent == 0 && LandCount > 0)) return false;
	if(!TestTrue(TEXT("地質源の変化が解へ反映"), MaximumDifference > 1E-6)) return false;
	AddInfo(FString::Printf(
		TEXT("Outlet potential prototype: nodes=%u land=%u iterations=%u minimumDownhill=%.9fm ")
		TEXT("maximumResidual=%.9g sourceDifference=%.6fm maximumEdgeRise=%.6fm ")
		TEXT("maximumEdgeSlope=%.9f elapsed=%.3fs. Graph-only; no C1 surface."),
		Orogenic->Statistics().Nodes, LandCount, Orogenic->Statistics().Iterations,
		Orogenic->Statistics().MinimumDownhillMeters, Orogenic->Statistics().MaximumResidual,
		MaximumDifference, MaximumEdgeRiseMeters, MaximumEdgeSlope, FPlatformTime::Seconds() - Started));
	const std::array<std::pair<std::uint64_t, double>, 2> RadiusCases{{{42, 1000}, {7, 1000000000}}};
	for(const auto &[Seed, Radius] : RadiusCases) {
		PlanetCore::GeologySettings OtherSettings;
		OtherSettings.Seed = Seed;
		OtherSettings.RadiusMeters = Radius;
		std::optional<PlanetCore::GeologyField> OtherGeology = PlanetCore::GeologyField::Create(OtherSettings);
		if(!TestTrue(TEXT("別半径の地質を準備"), OtherGeology.has_value())) return false;
		std::optional<PlanetCore::OceanConnectivityField> OtherOcean = PlanetCore::OceanConnectivityField::Create(
			std::make_shared<const PlanetCore::GeologyField>(std::move(*OtherGeology)));
		if(!TestTrue(TEXT("別半径の大洋を準備"), OtherOcean.has_value())) return false;
		const std::optional<PlanetCore::PlanetOrogenicRidges> OtherRidges =
			PlanetCore::PlanetOrogenicRidges::Create(OtherOcean->Base());
		if(!TestTrue(TEXT("別半径の隆起源を準備"), OtherRidges.has_value())) return false;
		for(std::size_t Index = 0; Index < Sites.size(); ++Index) {
			const PlanetCore::UnitDirection Direction = Mesh->Directions[Index];
			Sites[Index].Ocean = OtherOcean->EvaluateElevation(Direction) <= OtherSettings.SeaLevelMeters;
			OrogenicSource[Index] = 1 + OtherRidges->EvaluateElevation(Direction) / 100;
		}
		const double TargetHeight = std::min(4000.0, 0.04 * Radius);
		const std::optional<PlanetCore::PlanetOutletPotential> Other =
			PlanetCore::PlanetOutletPotential::Create(Sites, Edges, OrogenicSource, TargetHeight);
		if(!TestTrue(TEXT("別半径でも全陸地点に下りを得る"), Other.has_value())) return false;
		AddInfo(FString::Printf(TEXT("Outlet radius case: seed=%llu radius=%.0fm land=%u iterations=%u ")
			TEXT("minimumDownhill=%.9fm maximumResidual=%.9g."),
			static_cast<unsigned long long>(Seed), Radius, Other->Statistics().LandNodes, Other->Statistics().Iterations,
			Other->Statistics().MinimumDownhillMeters, Other->Statistics().MaximumResidual));
	}
	return true;
}
#endif
