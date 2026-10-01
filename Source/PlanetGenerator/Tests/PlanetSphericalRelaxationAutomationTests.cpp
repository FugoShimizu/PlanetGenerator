#include "Core/PlanetSphericalRefinement.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalReliefRelaxationContracts,
	"PlanetGenerator.SphericalRelief.RelaxationContracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 優先度付き再評価を独立な反復解と比較し，既存河道の保持と盛土の減少を検査する
 * @param Parameters 未使用の UE 引数
 * @return 全解析場で固定条件・下降・最小解が一致する場合に真
 */
bool FPlanetSphericalReliefRelaxationContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetSphericalRefinement;
	using PlanetCore::SphericalReliefSettings;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	using Triangle = std::array<std::uint32_t, 3>;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(2);
	// 元の閉球面を準備出来ない場合の返戻
	if(!TestTrue(TEXT("独立な補正検査の元網を準備"), Mesh.has_value())) return false;
	std::uint32_t ChangedCount = 0, ProtectedCount = 0;
	double MaximumReferenceError = 0, MaximumLowering = 0;
	for(const double Amplitude : { 0.0, 60.0, 120.0 }) {
		const auto Source = [Amplitude](const UnitDirection Direction) -> double {
			const Vec3 P = Direction.Vector();
			// 格子の分割と独立な解析的起伏の返戻
			return 600 + 180 * P.Z + Amplitude * (std::sin(12 * P.X + 3 * P.Z) + 0.5 * std::cos(9 * P.Y - 4 * P.X));
		};
		std::vector<double> Heights;
		std::vector<std::uint32_t> Routes;
		for(std::uint32_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			Heights.push_back(Source(Mesh->Directions[Index]));
			Routes.push_back(Index);
		}
		for(const Triangle &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			if(Heights[B] < Heights[Routes[A]]) Routes[A] = B;
		}
		SphericalReliefSettings Settings;
		Settings.RadiusMeters = 1000;
		Settings.MinimumSlope = 1E-4;
		Settings.TargetErrorMeters = 0.5;
		Settings.MaximumAddedVertices = 512;
		const std::optional<PlanetSphericalRefinement> Before =
		PlanetSphericalRefinement::CreateWithRefinedEdges(Mesh->Directions, Mesh->Triangles, Heights, Routes, Source, Settings);
		Settings.ShouldRelaxAddedHeights = true;
		const std::optional<PlanetSphericalRefinement> After =
		PlanetSphericalRefinement::CreateWithRefinedEdges(Mesh->Directions, Mesh->Triangles, Heights, Routes, Source, Settings);
		// 比較する網を準備出来ない場合の返戻
		if(!TestTrue(TEXT("補正前後の細分化を準備"), Before.has_value() && After.has_value())) return false;
		const std::size_t Count = Before->GetDirections().size(), OriginalCount = Mesh->Directions.size();
		TestEqual(TEXT("補正は節点を増減しない"), After->GetDirections().size(), Count);
		const bool HasSameTriangles = std::equal(
			Before->GetTriangles().begin(),
			Before->GetTriangles().end(),
			After->GetTriangles().begin(),
			After->GetTriangles().end()
		);
		TestTrue(TEXT("補正は共有網を変更しない"), HasSameTriangles);
		std::vector<double> Original;
		for(const UnitDirection Direction : Before->GetDirections()) Original.push_back(Source(Direction));
		std::vector<bool> Fixed(Count, false);
		// 元節点と開削点から全経路を辿り，保護する節点を独立に列挙する
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(Index >= OriginalCount && Before->GetElevations()[Index] >= Original[Index]) continue;
			std::uint32_t Current = Index;
			for(std::size_t Step = 0; Step < Count; ++Step) {
				Fixed[Current] = true;
				const std::uint32_t Parent = Before->GetDownstreamVertices()[Current];
				if(Parent == Current) break;
				Current = Parent;
			}
		}
		std::vector<double> Reference(Count, std::numeric_limits<double>::infinity());
		for(std::size_t Index = 0; Index < Count; ++Index) {
			if(Fixed[Index]) Reference[Index] = Before->GetElevations()[Index];
			TestTrue(TEXT("補正後も同じ球面方向"), (Before->GetDirections()[Index].Vector() - After->GetDirections()[Index].Vector()).Length() == 0);
		}
		// 優先度列を使わず，固定点だけから全辺を同期反復する別計算
		for(std::size_t Iteration = 0; Iteration < Count; ++Iteration) {
			std::vector<double> Next = Reference;
			for(const Triangle &Face : Before->GetTriangles()) for(unsigned Side = 0; Side < 3; ++Side) {
				const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
				if(Fixed[A] || !std::isfinite(Reference[B])) continue;
				const Vec3 P = Before->GetDirections()[A].Vector(), Q = Before->GetDirections()[B].Vector();
				const double Distance = std::atan2(P.Cross(Q).Length(), P.Dot(Q)) * Settings.RadiusMeters;
				const double Lower = std::nextafter(Reference[B] + Settings.MinimumSlope * Distance, std::numeric_limits<double>::infinity());
				Next[A] = std::min(Next[A], std::max(Original[A], Lower));
			}
			if(Next == Reference) break;
			Reference = std::move(Next);
		}
		for(std::size_t Index = 0; Index < Count; ++Index) {
			const double Height = After->GetElevations()[Index], Previous = Before->GetElevations()[Index];
			if(Fixed[Index]) {
				TestEqual(TEXT("元河道の標高を変更しない"), Height, Previous);
				TestEqual(TEXT("元河道の接続を変更しない"), After->GetDownstreamVertices()[Index], Before->GetDownstreamVertices()[Index]);
				if(Index >= OriginalCount) ++ProtectedCount;
			} else TestTrue(TEXT("不要な盛土だけを下げる"), Height >= Original[Index] && Height <= Previous);
			TestTrue(TEXT("独立な反復解が全点へ到達"), std::isfinite(Reference[Index]));
			MaximumReferenceError = std::max(MaximumReferenceError, std::abs(Height - Reference[Index]));
			const std::uint32_t Parent = After->GetDownstreamVertices()[Index];
			TestTrue(TEXT("補正後も下流へ厳密に下降"), Parent == Index || Height > After->GetElevations()[Parent]);
			if(Height < Previous) ++ChangedCount;
			MaximumLowering = std::max(MaximumLowering, Previous - Height);
		}
		TestTrue(
			TEXT("盛土の最大量を増やさない"),
			After->GetStatistics().MaximumConditioningRaiseMeters <= Before->GetStatistics().MaximumConditioningRaiseMeters
		);
		TestTrue(
			TEXT("既存の開削量を変更しない"),
			After->GetStatistics().MaximumConditioningCutMeters == Before->GetStatistics().MaximumConditioningCutMeters
		);
	}
	TestTrue(TEXT("不要な盛土を持つ解析条件を実際に検査"), ChangedCount > 0 && MaximumLowering > 0);
	TestTrue(TEXT("元河道の追加節点も実際に固定"), ProtectedCount > 0);
	TestTrue(TEXT("優先度付き補正と独立な反復解が一致"), MaximumReferenceError < 1E-8);
	AddInfo(
		FString::Printf(
			TEXT("Spherical relief relaxation reference: fields=3 changed=%u protected=%u lowering=%.12g error=%.12g"),
			ChangedCount,
			ProtectedCount,
			MaximumLowering,
			MaximumReferenceError
		)
	);
	// 独立解・既存河道・補正量の検査結果の返戻
	return !HasAnyErrors();
}

#endif
