#include "Core/PlanetLod.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <bit>
#include <iterator>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetLodIncremental, "PlanetGenerator.Lod.Incremental",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 差分組立を独立な全組立と比較し，接合・順序・倍精度座標の一致を検証する
 * @param Parameters 未使用の検査引数
 * @return 全条件の成否
 */
bool FPlanetLodIncremental::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Start = FPlatformTime::Seconds();
	std::uint64_t ComparedVertices = 0, ComparedMeshes = 0;
	const auto Check = [&](const PlanetCore::LodTree &Tree, PlanetCore::AdaptiveSphereMesh &Previous) -> bool {
		std::optional<PlanetCore::AdaptiveSphereMesh> Actual = PlanetCore::UpdateAdaptiveSphere(Tree, Previous);
		const std::optional<PlanetCore::AdaptiveSphereMesh> Expected = PlanetCore::BuildAdaptiveSphere(Tree);
		// 比較対象を構築出来ない場合の返戻
		if(!TestTrue(TEXT("Both independent assembly paths succeed"), Actual.has_value() && Expected.has_value())) return false;
		// 接合と頂点順序の不一致の返戻
		if(!TestTrue(TEXT("Keys, faces, owners and stitch counts exactly match full assembly"),
			   Actual->Mesh.Keys == Expected->Mesh.Keys && Actual->Mesh.Triangles == Expected->Mesh.Triangles &&
				   Actual->TriangleOwners == Expected->TriangleOwners &&
				   Actual->StitchMaskCounts == Expected->StitchMaskCounts &&
				   Actual->Mesh.Directions.size() == Expected->Mesh.Directions.size()))
			return false;
		for(std::size_t Index = 0; Index < Actual->Mesh.Directions.size(); ++Index) {
			const PlanetCore::Vec3 A = Actual->Mesh.Directions[Index].Vector(), B = Expected->Mesh.Directions[Index].Vector();
			// 倍精度の丸め迄含めた不一致の返戻
			if(!TestTrue(TEXT("Canonical direction components agree bit for bit"),
				   std::bit_cast<std::uint64_t>(A.X) == std::bit_cast<std::uint64_t>(B.X) &&
					   std::bit_cast<std::uint64_t>(A.Y) == std::bit_cast<std::uint64_t>(B.Y) &&
					   std::bit_cast<std::uint64_t>(A.Z) == std::bit_cast<std::uint64_t>(B.Z)))
				return false;
		}
		ComparedVertices += Actual->Mesh.Directions.size();
		++ComparedMeshes;
		Previous = std::move(*Actual);
		// 完全一致の返戻
		return true;
	};
	for(std::uint8_t Root = 0; Root < 20; ++Root)
		for(std::uint8_t Mask = 0; Mask < 8; ++Mask) {
			PlanetCore::LodTree Tree;
			PlanetCore::AdaptiveSphereMesh Previous = *PlanetCore::BuildAdaptiveSphere(Tree);
			const PlanetCore::TileId Tile{0, Root, 0};
			for(std::uint8_t Edge = 0; Edge < 3; ++Edge)
				if(Mask & (1 << Edge)) Tree.Refine(PlanetCore::GetNeighbor(Tile, Edge)->Tile, 4096);
			// 全根と全接合形状に於ける不一致の返戻
			if(!Check(Tree, Previous) || !Check(Tree, Previous)) return false;
			for(std::uint8_t Edge = 0; Edge < 3; ++Edge)
				if(Mask & (1 << Edge)) {
					Tree.Coarsen(PlanetCore::GetNeighbor(Tile, Edge)->Tile);
					// 縮約に依る接合変更の不一致の返戻
					if(!Check(Tree, Previous)) return false;
				}
		}
	{
		PlanetCore::LodTree Tree;
		PlanetCore::AdaptiveSphereMesh Previous = *PlanetCore::BuildAdaptiveSphere(Tree);
		PlanetCore::TileId Tile{0, 7, 0};
		for(std::uint8_t Level = 0; Level < PlanetCore::MaxTileLevel; ++Level) {
			const PlanetCore::LodEdit Edit = Tree.Refine(Tile, 16384);
			// 最大深度迄の平衡分割の失敗の返戻
			if(!TestTrue(TEXT("Deep balanced refinement applies"), Edit.Status == PlanetCore::LodEditStatus::Applied) ||
				!Check(Tree, Previous))
				return false;
			Tile = *Tile.Child(Level % 4);
		}
	}
	{
		PlanetCore::LodTree Tree;
		PlanetCore::AdaptiveSphereMesh Previous = *PlanetCore::BuildAdaptiveSphere(Tree);
		std::uint32_t State = 1879;
		std::uint32_t Coarsened = 0;
		for(std::uint32_t Step = 0; Step < 160; ++Step) {
			for(std::uint32_t Batch = 0; Batch < 3; ++Batch) {
				State = State * 1664525u + 1013904223u;
				std::set<PlanetCore::TileId, PlanetCore::TileOrder>::const_iterator Selected = Tree.Leaves().begin();
				std::advance(Selected, State % Tree.Leaves().size());
				Tree.Refine(*Selected, 16384);
			}
			if(Step % 2 == 0) {
				const std::vector<PlanetCore::TileId> Snapshot(Tree.Leaves().begin(), Tree.Leaves().end());
				for(const PlanetCore::TileId Leaf : Snapshot)
					if(const std::optional<PlanetCore::TileId> Parent = Leaf.Parent();
						Parent && Tree.Coarsen(*Parent) == PlanetCore::LodEditStatus::Applied) {
						++Coarsened;
						break;
					}
			}
			// 一括変更と平衡分割を含む混合操作の不一致の返戻
			if(!Check(Tree, Previous)) return false;
		}
		// 縮約経路が実際に通った事を確認した結果の返戻
		if(!TestTrue(TEXT("Mixed sequence exercises successful coarsening"), Coarsened > 0)) return false;
		std::stop_source Cancel;
		Cancel.request_stop();
		TestFalse(TEXT("Cancelled incremental assembly rejected"),
			PlanetCore::UpdateAdaptiveSphere(Tree, Previous, Cancel.get_token()).has_value());
		PlanetCore::AdaptiveSphereMesh Invalid = Previous;
		Invalid.TriangleOwners.pop_back();
		TestFalse(TEXT("Missing owner rejected"), PlanetCore::UpdateAdaptiveSphere(Tree, Invalid).has_value());
		Invalid = Previous;
		Invalid.Mesh.Triangles[0][0] = static_cast<std::uint32_t>(Invalid.Mesh.Keys.size());
		TestFalse(TEXT("Out of range vertex rejected"), PlanetCore::UpdateAdaptiveSphere(Tree, Invalid).has_value());
		Invalid = Previous;
		Invalid.Mesh.Keys.pop_back();
		TestFalse(TEXT("Missing canonical key rejected"), PlanetCore::UpdateAdaptiveSphere(Tree, Invalid).has_value());
	}
	AddInfo(FString::Printf(TEXT("Incremental assembly: %llu exact meshes, %llu bitwise vertices, 20 roots, 8 masks, "
								 "depth 32, 160 batched mixed updates; %.3f seconds."),
		static_cast<unsigned long long>(ComparedMeshes), static_cast<unsigned long long>(ComparedVertices),
		FPlatformTime::Seconds() - Start));
	// 蓄積した検査結果の返戻
	return !HasAnyErrors();
}

#endif
