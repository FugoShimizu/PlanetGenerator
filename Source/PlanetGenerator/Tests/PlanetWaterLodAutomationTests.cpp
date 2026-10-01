#include "Misc/AutomationTest.h"
#include "Core/PlanetWaterLod.h"
#include "Core/PlanetWaterCoverage.h"
#include "../../../Tests/Core/PlanetLodContractChecks.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetWaterLodContracts, "PlanetGenerator.WaterLod.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/** 遠景の縮約後も近景・固定骨格・全球接続が残る事を検査する */
bool FPlanetWaterLodContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	using PlanetCore::TileId;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	std::optional<PlanetCore::LodTree> Tree = PlanetCore::PrepareWaterSkeletonTree(PlanetCore::LodTree{}, 6);
	if(!TestTrue(TEXT("Fixed skeleton prepared"), Tree.has_value())) return false;
	TileId Tile{0, 0, 6};
	for(unsigned Level = 6; Level < 12; ++Level) {
		if(!TestTrue(TEXT("Local refinement applied"),
			   Tree->Refine(Tile, PlanetCore::LodTree::MaximumLeaves).Status == PlanetCore::LodEditStatus::Applied))
			return false;
		Tile = *Tile.Child(0);
	}
	Vec3 Sum;
	for(std::uint8_t Index = 0; Index < 3; ++Index)
		Sum = Sum + PlanetCore::GetDirection(*PlanetCore::GetVertexKey(Tile, Index))->Vector();
	const UnitDirection Near = *UnitDirection::FromVector(Sum);
	const UnitDirection Far = *UnitDirection::FromVector(Near.Vector() * -1);
	PlanetCore::LodView View;
	View.RadiusMeters = 1000;
	View.ReliefBoundMeters = 100;
	View.ObserverMeters = Far.Vector() * 1002;
	const std::optional<PlanetCore::WaterLodTrim> Trimmed = PlanetCore::TrimWaterLod(*Tree, View, Far, 36, 6);
	if(!TestTrue(TEXT("Distant old detail removed"),
		   Trimmed && Trimmed->CoarsenedParents > 0 && Trimmed->Tree.Leaves().size() < Tree->Leaves().size()))
		return false;
	for(const TileId Leaf : Trimmed->Tree.Leaves()) {
		if(!TestTrue(TEXT("Fixed hydrology level retained"), Leaf.Level >= 6)) return false;
	}
	PlanetTests::LodCheckReport Report;
	if(!PlanetTests::CheckPartition(Trimmed->Tree, Report) || !PlanetTests::CheckMesh(Trimmed->Tree, Report)) {
		AddError(UTF8_TO_TCHAR(Report.Failure.c_str()));
		// 接続不整合の返戻
		return false;
	}
	View.ObserverMeters = Near.Vector() * 1002;
	const std::optional<PlanetCore::WaterLodTrim> Kept = PlanetCore::TrimWaterLod(*Tree, View, Near, 36, 6);
	if(!TestTrue(TEXT("Near detail retained"), Kept && Kept->Tree.IsLeaf(Tile))) return false;
	std::uint32_t NearLeaves = 0;
	for(const TileId Leaf : Tree->Leaves()) {
		Vec3 Center;
		for(std::uint8_t Index = 0; Index < 3; ++Index)
			Center = Center + PlanetCore::GetDirection(*PlanetCore::GetVertexKey(Leaf, Index))->Vector();
		if((UnitDirection::FromVector(Center)->Vector() - Near.Vector()).Length() * View.RadiusMeters <= 36) {
			if(!TestTrue(TEXT("Original near leaf unchanged"), Kept->Tree.IsLeaf(Leaf))) return false;
			++NearLeaves;
		}
	}
	std::stop_source Cancelled;
	Cancelled.request_stop();
	if(!TestFalse(TEXT("Cancellation has no candidate"),
		   PlanetCore::TrimWaterLod(*Tree, View, Near, 36, 6, Cancelled.get_token()).has_value()))
		return false;
	if(!TestFalse(TEXT("Invalid radius rejected"), PlanetCore::TrimWaterLod(*Tree, View, Near, -1, 6).has_value()))
		return false;
	AddInfo(FString::Printf(
		TEXT("Water LOD: %u coarsened parents, %u protected near leaves, %llu topology checks; original leaves %llu -> %llu."),
		Trimmed->CoarsenedParents, NearLeaves, static_cast<unsigned long long>(Report.Checks),
		static_cast<unsigned long long>(Tree->Leaves().size()),
		static_cast<unsigned long long>(Trimmed->Tree.Leaves().size())));
	// 全検査の成功の返戻
	return true;
}
#endif
