#include "Core/PlanetLod.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <atomic>
#include <bit>
#include <thread>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetLodWorkerIsolation, "PlanetGenerator.Lod.WorkerIsolation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 地表と水面へ分割木を渡す際のキャッシュ所有を検査する
 * @param Parameters 未使用の検査引数
 * @return 両ワーカーの探索が独立し，出力が一致すれば真
 */
bool FPlanetLodWorkerIsolation::RunTest(const FString &Parameters) {
	(void)Parameters;
	using PlanetCore::AdaptiveSphereMesh;
	using PlanetCore::BuildAdaptiveSphere;
	using PlanetCore::LodEditStatus;
	using PlanetCore::LodNeighborCache;
	using PlanetCore::LodTree;
	using PlanetCore::TileId;
	const double Begin = FPlatformTime::Seconds();
	const std::shared_ptr<LodNeighborCache> LandNeighbors = std::make_shared<LodNeighborCache>(37);
	LodTree Land(LandNeighbors);
	LodTree Independent;
	const auto Prepare = [&](LodTree &Tree) {
		for(std::uint8_t Root = 0; Root < 20; ++Root)
			if(Tree.Refine(TileId{0, Root, 0}, 4096).Status != LodEditStatus::Applied) return false;
		for(std::uint8_t Root = 0; Root < 20; ++Root)
			if(Tree.Refine(TileId{0, Root, 1}, 4096).Status != LodEditStatus::Applied) return false;
		// 同じ葉集合の準備結果の返戻
		return true;
	};
	if(!TestTrue(TEXT("Two reference trees prepared"), Prepare(Land) && Prepare(Independent))) return false;
	LodTree Water = Land;
	Water.DetachNeighborCache();
	const std::vector<TileId> LandLeaves(Land.Leaves().begin(), Land.Leaves().end());
	const std::uint64_t HitsBefore = LandNeighbors->Hits();
	const std::size_t TilesBefore = LandNeighbors->TileCount();
	if(!TestTrue(TEXT("Independent water refinement applies"),
		   Water.Refine(TileId{0, 0, 2}, 4096).Status == LodEditStatus::Applied &&
			   Independent.Refine(TileId{0, 0, 2}, 4096).Status == LodEditStatus::Applied))
		return false;
	const std::optional<AdaptiveSphereMesh> WaterMesh = BuildAdaptiveSphere(Water);
	const std::optional<AdaptiveSphereMesh> ReferenceMesh = BuildAdaptiveSphere(Independent);
	if(!TestTrue(TEXT("Both private partitions triangulate"), WaterMesh.has_value() && ReferenceMesh.has_value())) return false;
	if(!TestTrue(TEXT("Detaching and editing water leaves does not mutate the land tree or its LRU"),
		   std::vector<TileId>(Land.Leaves().begin(), Land.Leaves().end()) == LandLeaves &&
			   LandNeighbors->Hits() == HitsBefore && LandNeighbors->TileCount() == TilesBefore))
		return false;
	if(!TestTrue(TEXT("Detached water topology and canonical vertices match the uncached reference"),
		   WaterMesh->Mesh.Keys == ReferenceMesh->Mesh.Keys && WaterMesh->Mesh.Triangles == ReferenceMesh->Mesh.Triangles &&
			   WaterMesh->TriangleOwners == ReferenceMesh->TriangleOwners &&
			   WaterMesh->StitchMaskCounts == ReferenceMesh->StitchMaskCounts &&
			   WaterMesh->Mesh.Directions.size() == ReferenceMesh->Mesh.Directions.size()))
		return false;
	for(std::size_t Index = 0; Index < WaterMesh->Mesh.Directions.size(); ++Index) {
		const PlanetCore::Vec3 A = WaterMesh->Mesh.Directions[Index].Vector();
		const PlanetCore::Vec3 B = ReferenceMesh->Mesh.Directions[Index].Vector();
		if(!TestTrue(TEXT("Water vertex directions remain bit identical"),
			   std::bit_cast<std::uint64_t>(A.X) == std::bit_cast<std::uint64_t>(B.X) &&
				   std::bit_cast<std::uint64_t>(A.Y) == std::bit_cast<std::uint64_t>(B.Y) &&
				   std::bit_cast<std::uint64_t>(A.Z) == std::bit_cast<std::uint64_t>(B.Z)))
			return false;
	}
	const TileId LandLeaf = *Land.Leaves().begin(), WaterLeaf = *Water.Leaves().begin();
	Land.HasFinerNeighbor(LandLeaf, 0);
	const std::uint64_t WarmLandHits = LandNeighbors->Hits();
	std::atomic<bool> Start{false};
	std::jthread LandTask([&] {
		while(!Start.load(std::memory_order_acquire)) std::this_thread::yield();
		for(unsigned Index = 0; Index < 256; ++Index) Land.HasFinerNeighbor(LandLeaf, 0);
	});
	std::jthread WaterTask([&] {
		while(!Start.load(std::memory_order_acquire)) std::this_thread::yield();
		for(unsigned Index = 0; Index < 256; ++Index) Water.HasFinerNeighbor(WaterLeaf, Index % 3);
	});
	Start.store(true, std::memory_order_release);
	LandTask.join();
	WaterTask.join();
	if(!TestTrue(TEXT("Concurrent water lookups never touch the land neighbor cache"),
		   LandNeighbors->Hits() - WarmLandHits == 256 && LandNeighbors->TileCount() <= 37))
		return false;
	AddInfo(FString::Printf(TEXT("Detached LOD worker: %llu leaves, %llu bitwise vertices, 512 concurrent lookups, %.3f s."),
		static_cast<unsigned long long>(Water.Leaves().size()),
		static_cast<unsigned long long>(WaterMesh->Mesh.Directions.size()), FPlatformTime::Seconds() - Begin));
	// 私有キャッシュ，木，頂点の契約を満たす返戻
	return true;
}
#endif
