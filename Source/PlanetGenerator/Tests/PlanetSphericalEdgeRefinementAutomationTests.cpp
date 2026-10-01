#include "Core/PlanetSphericalRefinement.h"
#include "Core/PlanetSphericalSpline.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalEdgeContracts, "PlanetGenerator.SphericalRelief.EdgeContracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 共有辺の途中の起伏，両側の接続，元下流辺の分割経路，独立な再現誤差を検査する
 * @param Parameters 未使用の UE 引数
 * @return 位相・下降・海岸・再現誤差の条件を満たす場合に真
 */
bool FPlanetSphericalEdgeContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetSphericalRefinement;
	using PlanetCore::SphericalReliefSettings;
	using PlanetCore::SphericalSplineField;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	using Triangle = std::array<std::uint32_t, 3>;
	using Edge = std::pair<std::uint32_t, std::uint32_t>;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(2);
	// 元網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("辺細分化用の球面を準備"), Mesh.has_value())) return false;
	std::optional<UnitDirection> Center;
	for(const Triangle &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3 && !Center; ++Side) {
		const Vec3 A = Mesh->Directions[Face[Side]].Vector(), B = Mesh->Directions[Face[(Side + 1) % 3]].Vector();
		if(A.Z > 0.4 && B.Z > 0.4 && std::abs(A.Z - B.Z) < 0.02) Center = UnitDirection::FromVector(A + B);
	}
	// 節点間の解析的な山頂を配置出来ない場合の返戻
	if(!TestTrue(TEXT("高緯度の辺中点を選択"), Center.has_value())) return false;
	const auto Source = [Center](const UnitDirection Point) -> TerrainSample {
		const Vec3 P = Point.Vector(), Difference = P - Center->Vector();
		const double Gaussian = 800 * std::exp(-Difference.Dot(Difference) / (2 * 0.035 * 0.035));
		const Vec3 Ambient = Vec3{ 0, 0, 200 } - Difference * (Gaussian / (0.035 * 0.035));
		// 節点間の山頂と海岸を含む解析場の返戻
		return { 80 + 200 * P.Z + Gaussian, Ambient - P * P.Dot(Ambient) };
	};
	std::vector<double> Heights;
	std::vector<std::uint32_t> OriginalRoutes;
	for(std::uint32_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		Heights.push_back(Source(Mesh->Directions[Index]).ElevationMeters);
		OriginalRoutes.push_back(Index);
	}
	for(const Triangle &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
		if(Heights[A] > 0 && Heights[B] < Heights[OriginalRoutes[A]]) OriginalRoutes[A] = B;
	}
	SphericalReliefSettings Settings;
	Settings.TargetErrorMeters = 0.5;
	Settings.MaximumAddedVertices = 512;
	const auto Height = [&Source](const UnitDirection Point) -> double {
		return Source(Point).ElevationMeters;
	};
	const std::optional<PlanetSphericalRefinement> Refined =
	PlanetSphericalRefinement::CreateWithRefinedEdges(Mesh->Directions, Mesh->Triangles, Heights, OriginalRoutes, Height, Settings);
	// 正規の共有辺分割に失敗した場合の返戻
	if(!TestTrue(TEXT("下降経路を保持して辺を細分化"), Refined.has_value())) return false;
	const std::size_t Count = Refined->GetDirections().size(), OriginalCount = Mesh->Directions.size();
	TestTrue(TEXT("予算内で辺中点を追加"), Count > OriginalCount && Count <= OriginalCount + Settings.MaximumAddedVertices);
	TestEqual(TEXT("閉球面の Euler 特性"), Refined->GetTriangles().size(), 2 * (Count - 2));
	std::map<Edge, std::pair<unsigned, int>> Edges;
	for(const Triangle &Face : Refined->GetTriangles()) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
		std::pair<unsigned, int> &Owners = Edges[{ std::min(A, B), std::max(A, B) }];
		++Owners.first;
		Owners.second += A < B ? 1 : -1;
	}
	for(const std::pair<const Edge, std::pair<unsigned, int>> &Entry : Edges) {
		TestTrue(TEXT("全共有辺の両側が閉じる"), Entry.second.first == 2 && !Entry.second.second);
	}
	const std::span<const std::uint32_t> Routes = Refined->GetDownstreamVertices();
	unsigned SplitRoutes = 0;
	double MaximumRoutePlaneError = 0;
	for(std::uint32_t Vertex = 0; Vertex < Count; ++Vertex) {
		const std::uint32_t Parent = Routes[Vertex];
		TestTrue(TEXT("下降先は有効な節点"), Parent < Count);
		if(Parent >= Count) continue;
		if(Parent != Vertex) {
			TestTrue(TEXT("下降先へ実辺が有る"), Edges.contains({ std::min(Vertex, Parent), std::max(Vertex, Parent) }));
			TestTrue(TEXT("全下降辺で標高が下がる"), Refined->GetElevations()[Vertex] > Refined->GetElevations()[Parent]);
		}
		if(Vertex >= OriginalCount) continue;
		TestEqual(TEXT("元節点の標高を保持"), Refined->GetElevations()[Vertex], Heights[Vertex]);
		if(OriginalRoutes[Vertex] == Vertex) {
			TestEqual(TEXT("元の終端を保持"), Parent, Vertex);
			continue;
		}
		const Vec3 A = Mesh->Directions[Vertex].Vector(), B = Mesh->Directions[OriginalRoutes[Vertex]].Vector();
		const Vec3 Plane = UnitDirection::FromVector(A.Cross(B - A))->Vector();
		std::uint32_t Current = Parent;
		unsigned Steps = 0;
		while(Current >= OriginalCount && Current < Count && Steps++ < Count) {
			MaximumRoutePlaneError = std::max(MaximumRoutePlaneError, std::abs(Plane.Dot(Refined->GetDirections()[Current].Vector())));
			Current = Routes[Current];
		}
		TestEqual(TEXT("元の下流端点へ分割経路が到達"), Current, OriginalRoutes[Vertex]);
		if(Parent >= OriginalCount) ++SplitRoutes;
	}
	TestTrue(TEXT("元流路の途中にも節点を追加して検査"), SplitRoutes > 0 && MaximumRoutePlaneError < 1E-10);
	std::vector<Vec3> Gradients;
	for(const UnitDirection Point : Refined->GetDirections()) Gradients.push_back(Source(Point).TangentialGradientPerRadian);
	const std::optional<std::vector<Vec3>> Limited = Refined->ConstrainDrainageGradients(Gradients, 0, Routes);
	// 分割後の下降と海岸を守る接線を準備出来ない場合の返戻
	if(!TestTrue(TEXT("分割後の下流と海陸に勾配を制限"), Limited.has_value())) return false;
	const std::optional<SphericalSplineField> After = SphericalSplineField::CreateOnMeshWithGradients(
		2,
		Refined->GetDirections(),
		Refined->GetTriangles(),
		Refined->GetElevations(),
		*Limited
	);
	const std::optional<SphericalSplineField> Before = SphericalSplineField::Create(2, Heights);
	// 独立な形状比較に必要な面を準備出来ない場合の返戻
	if(!TestTrue(TEXT("細分化前後の C1 面を準備"), After.has_value() && Before.has_value())) return false;
	double BeforeSquared = 0, AfterSquared = 0, MaximumUphill = 0;
	for(unsigned Index = 0; Index < 16384; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / 16384, Angle = Index * 2.399963229728653, Radius = std::sqrt(1 - Z * Z);
		const UnitDirection Point = *UnitDirection::FromVector({ Radius * std::cos(Angle), Radius * std::sin(Angle), Z });
		const double Original = Source(Point).ElevationMeters;
		BeforeSquared += std::pow(Before->Evaluate(Point).ElevationMeters - Original, 2);
		AfterSquared += std::pow(After->Evaluate(Point).ElevationMeters - Original, 2);
		TestEqual(TEXT("海岸の符号を保持"), After->Evaluate(Point).ElevationMeters < 0, Before->Evaluate(Point).ElevationMeters < 0);
	}
	for(std::uint32_t Vertex = 0; Vertex < Count; ++Vertex) {
		const std::uint32_t Parent = Routes[Vertex];
		if(Parent == Vertex) continue;
		const Vec3 A = Refined->GetDirections()[Vertex].Vector(), B = Refined->GetDirections()[Parent].Vector();
		double Last = After->Evaluate(*UnitDirection::FromVector(A)).ElevationMeters;
		for(unsigned Step = 1; Step < 33; ++Step) {
			const double Fraction = Step / 32.0;
			const UnitDirection Point = *UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
			const double Next = After->Evaluate(Point).ElevationMeters;
			MaximumUphill = std::max(MaximumUphill, Next - Last);
			Last = Next;
		}
	}
	TestTrue(TEXT("独立試料の元起伏への再現誤差を削減"), AfterSquared < 0.64 * BeforeSquared);
	TestTrue(TEXT("分割後の河道区間も下降"), MaximumUphill < 1E-8);
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断を受理"),
		PlanetSphericalRefinement::CreateWithRefinedEdges(
			Mesh->Directions,
			Mesh->Triangles,
			Heights,
			OriginalRoutes,
			Height,
			Settings,
			Cancelled.get_token()
		).has_value()
	);
	TestFalse(
		TEXT("下流の欠落を拒否"),
		PlanetSphericalRefinement::CreateWithRefinedEdges(Mesh->Directions, Mesh->Triangles, Heights, {}, Height, Settings).has_value()
	);
	std::vector<std::uint32_t> Invalid = OriginalRoutes;
	Invalid[0] = static_cast<std::uint32_t>(OriginalCount);
	TestFalse(
		TEXT("範囲外の下流を拒否"),
		PlanetSphericalRefinement::CreateWithRefinedEdges(Mesh->Directions, Mesh->Triangles, Heights, Invalid, Height, Settings)
		.has_value()
	);
	Settings.MaximumAddedVertices = 0;
	const std::optional<PlanetSphericalRefinement> Unchanged =
	PlanetSphericalRefinement::CreateWithRefinedEdges(Mesh->Directions, Mesh->Triangles, Heights, OriginalRoutes, Height, Settings);
	TestTrue(
		TEXT("追加予算０で元網と下降先を保持"),
		Unchanged.has_value() && !Unchanged->GetStatistics().AddedVertices && !Unchanged->GetStatistics().SourceEvaluations
	);
	if(Unchanged) {
		for(std::size_t Index = 0; Index < OriginalCount; ++Index) {
			TestEqual(TEXT("追加予算０でも下流を失わない"), Unchanged->GetDownstreamVertices()[Index], OriginalRoutes[Index]);
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Spherical edge refinement: original=%llu added=%u splitRoutes=%u samples=%llu before=%.12g after=%.12g uphill=%.12g ")
			TEXT("routePlane=%.12g raise=%.12g cut=%.12g quality=%.12g"),
			static_cast<unsigned long long>(OriginalCount),
			Refined->GetStatistics().AddedVertices,
			SplitRoutes,
			static_cast<unsigned long long>(Refined->GetStatistics().SourceEvaluations),
			std::sqrt(BeforeSquared / 16384),
			std::sqrt(AfterSquared / 16384),
			MaximumUphill,
			MaximumRoutePlaneError,
			Refined->GetStatistics().MaximumConditioningRaiseMeters,
			Refined->GetStatistics().MaximumConditioningCutMeters,
			Refined->GetStatistics().MinimumCreatedTriangleQuality
		)
	);
	// 共有網・元流路・独立な形状評価の検査結果の返戻
	return !HasAnyErrors();
}

#endif
