#include "Core/PlanetDrainageRefinement.h"
#include "Core/PlanetDrainage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetDrainageRefinementContracts, "PlanetGenerator.Drainage.Refinement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 八種類の分割の閉被覆・向き・面積・入力順の不変性と不正辺を検査する
 * @param Parameters 未使用の UE 検査引数
 * @return 全ての位相・面積・拒否経路を満たした場合に真
 */
bool FPlanetDrainageRefinementContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::BuildDrainageRefinement;
	using PlanetCore::DrainageRefinementMesh;
	const std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(6);
	// 正規の基準網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("局所分割の基準網を準備"), Base.has_value())) return false;
	double MaximumAreaError = 0;
	std::uint64_t CheckedTriangles = 0;
	for(unsigned Mask = 0; Mask < 8; ++Mask) {
		std::vector<std::array<std::uint32_t, 2>> Edges;
		const std::array<std::uint32_t, 3> Face = Base->Triangles[123];
		for(unsigned Edge = 0; Edge < 3; ++Edge) if(Mask & 1U << Edge) Edges.push_back({ Face[(Edge + 1) % 3], Face[(Edge + 2) % 3] });
		const std::optional<DrainageRefinementMesh> Mesh = BuildDrainageRefinement(Edges);
		// 指定した分割を作れない場合の返戻
		if(!TestTrue(TEXT("八種類の辺分割が成立"), Mesh.has_value())) return false;
		TestEqual(TEXT("追加頂点は指定した共有中点だけ"), Mesh->Directions.size(), Base->Directions.size() + Edges.size());
		TestEqual(TEXT("閉じた面数の増加"), Mesh->Triangles.size(), Base->Triangles.size() + 2 * Edges.size());
		for(std::size_t Index = 0; Index < Base->Directions.size(); ++Index) {
			TestEqual(TEXT("元頂点の位置と順序を保持"), (Mesh->Directions[Index].Vector() - Base->Directions[Index].Vector()).Length(), 0.0);
		}
		std::map<std::array<std::uint32_t, 2>, std::pair<unsigned, int>> Incidence;
		double Area = 0;
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
			const PlanetCore::Vec3 A = Mesh->Directions[Triangle[0]].Vector();
			const PlanetCore::Vec3 B = Mesh->Directions[Triangle[1]].Vector();
			const PlanetCore::Vec3 C = Mesh->Directions[Triangle[2]].Vector();
			const double Orientation = A.Dot((B - A).Cross(C - A));
			TestTrue(TEXT("全ての面が外向きで非退化"), Orientation > 0);
			Area += 2 * std::atan2(Orientation, 1 + A.Dot(B) + B.Dot(C) + C.Dot(A));
			for(unsigned Edge = 0; Edge < 3; ++Edge) {
				const std::uint32_t U = Triangle[Edge], V = Triangle[(Edge + 1) % 3];
				std::pair<unsigned, int> &Use = Incidence[{ std::min(U, V), std::max(U, V) }];
				++Use.first;
				Use.second += U < V ? 1 : -1;
			}
		}
		for(const auto &[Edge, Use] : Incidence) {
			static_cast<void>(Edge);
			TestEqual(TEXT("共有辺は二つの面だけに所属"), Use.first, 2U);
			TestEqual(TEXT("共有辺の向きが相殺"), Use.second, 0);
		}
		TestEqual(TEXT("閉じた球面の Euler 特性"), Mesh->Directions.size() + Mesh->Triangles.size(), Incidence.size() + 2);
		MaximumAreaError = std::max(MaximumAreaError, std::abs(Area - 4 * 3.14159265358979323846));
		TestTrue(TEXT("全球面積を保持"), std::abs(Area - 4 * 3.14159265358979323846) < 1E-10);
		CheckedTriangles += Mesh->Triangles.size();
		std::reverse(Edges.begin(), Edges.end());
		for(std::array<std::uint32_t, 2> &Edge : Edges) std::swap(Edge[0], Edge[1]);
		if(!Edges.empty()) Edges.push_back(Edges.front());
		const std::optional<DrainageRefinementMesh> Reordered = BuildDrainageRefinement(Edges);
		// 入力順を変更した正規経路を拒否した場合の返戻
		if(!TestTrue(TEXT("逆順・重複の辺も受理"), Reordered.has_value())) return false;
		TestTrue(TEXT("面の順序まで決定論的"), Reordered->Triangles == Mesh->Triangles);
		TestTrue(TEXT("中点番号まで決定論的"), Reordered->SplitEdges == Mesh->SplitEdges);
	}
	std::uint32_t Opposite = 1;
	for(std::uint32_t Index = 2; Index < Base->Directions.size(); ++Index) {
		const double CandidateDot = Base->Directions[Index].Vector().Dot(Base->Directions[0].Vector());
		const double CurrentDot = Base->Directions[Opposite].Vector().Dot(Base->Directions[0].Vector());
		if(CandidateDot < CurrentDot) Opposite = Index;
	}
	TestTrue(TEXT("非隣接の拒否試料は実際に反対側"), Base->Directions[Opposite].Vector().Dot(Base->Directions[0].Vector()) < -0.99);
	for(const std::array<std::uint32_t, 2> Edge : { std::array<std::uint32_t, 2>{ 0, 0 }, { 0, 40962 }, { 0, Opposite } }) {
		const std::array<std::array<std::uint32_t, 2>, 1> Invalid { Edge };
		TestFalse(TEXT("退化・範囲外・存在しない辺を拒否"), BuildDrainageRefinement(Invalid).has_value());
	}
	const std::array<std::uint32_t, 2> ValidEdge { Base->Triangles[0][0], Base->Triangles[0][1] };
	const std::array<std::array<std::uint32_t, 2>, 1> OneEdge { ValidEdge };
	const std::optional<DrainageRefinementMesh> OneSplit = BuildDrainageRefinement(OneEdge);
	// 正規な一辺分割を作れない場合の返戻
	if(!TestTrue(TEXT("正規の一辺を受理"), OneSplit.has_value())) return false;
	std::vector<PlanetCore::DrainageSite> OversizedGraph(OneSplit->Directions.size());
	for(std::size_t Index = 0; Index < OversizedGraph.size(); ++Index) {
		OversizedGraph[Index] = { OneSplit->Directions[Index].Vector(), -1, 1, true };
	}
	std::vector<std::array<std::uint32_t, 2>> GraphEdges;
	for(const std::array<std::uint32_t, 3> &Triangle : OneSplit->Triangles) {
		for(unsigned Edge = 0; Edge < 3; ++Edge) GraphEdges.push_back({ Triangle[Edge], Triangle[(Edge + 1) % 3] });
	}
	TestEqual(TEXT("上限試料は旧網より一地点だけ大きい"), OversizedGraph.size(), std::size_t{ 40963 });
	TestFalse(TEXT("任意グラフの既存規模上限を維持"), PlanetCore::DrainageNetwork::Solve(OversizedGraph, GraphEdges, {}).has_value());
	const std::vector<std::array<std::uint32_t, 2>> TooManyEdges(122881, ValidEdge);
	TestFalse(TEXT("局所分割にも有限な要求数上限"), BuildDrainageRefinement(TooManyEdges).has_value());
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("中断要求を拒否"), BuildDrainageRefinement({}, Cancel.get_token()).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Drainage refinement: patterns=8 triangles=%llu maximumAreaError=%.12g steradians."),
			static_cast<unsigned long long>(CheckedTriangles),
			MaximumAreaError
		)
	);
	// 位相と面積の独立検査の返戻
	return !HasAnyErrors();
}

#endif
