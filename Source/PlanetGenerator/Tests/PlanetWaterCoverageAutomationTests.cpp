#include "Core/PlanetWaterCoverage.h"
#include "Core/PlanetOceanSurface.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#if WITH_DEV_AUTOMATION_TESTS
namespace {
	/** 二重精度の表現が完全に一致するかを調べる */
	bool SameScalar(double A, double B) {
		// 符号付き零を含めた一致の返戻
		return std::bit_cast<std::uint64_t>(A) == std::bit_cast<std::uint64_t>(B);
	}

	/** 三成分の丸めを含めた一致を調べる */
	bool SameVector(PlanetCore::Vec3 A, PlanetCore::Vec3 B) {
		// 各成分の一致の返戻
		return SameScalar(A.X, B.X) && SameScalar(A.Y, B.Y) && SameScalar(A.Z, B.Z);
	}

	/** 検査再利用の有無で出力の位相・順序・座標・補間属性が一致するかを調べる */
	bool SameCoverage(const PlanetCore::AdaptiveWaterCoverage &A, const PlanetCore::AdaptiveWaterCoverage &B) {
		// 出力規模と接続が異なる場合の返戻
		if(A.Partition.Keys != B.Partition.Keys || A.Partition.Triangles != B.Partition.Triangles ||
			A.Wet.Triangles != B.Wet.Triangles || A.Wet.SourceTriangles != B.Wet.SourceTriangles ||
			A.Partition.Directions.size() != B.Partition.Directions.size() || A.Wet.Vertices.size() != B.Wet.Vertices.size() ||
			A.RefinementPasses != B.RefinementPasses || !SameScalar(A.MaximumFloatErrorMeters, B.MaximumFloatErrorMeters) ||
			A.Hierarchy.has_value() != B.Hierarchy.has_value())
			return false;
		if(A.Hierarchy) {
			// 参照元の平衡木と接合が異なる場合の返戻
			if(A.Hierarchy->Tree.Leaves() != B.Hierarchy->Tree.Leaves() ||
				A.Hierarchy->TriangleOwners != B.Hierarchy->TriangleOwners ||
				A.Hierarchy->StitchMaskCounts != B.Hierarchy->StitchMaskCounts)
				return false;
		}
		for(std::size_t i = 0; i < A.Partition.Directions.size(); ++i) {
			// 正規頂点の表現が異なる場合の返戻
			if(!SameVector(A.Partition.Directions[i].Vector(), B.Partition.Directions[i].Vector())) return false;
		}
		for(std::size_t i = 0; i < A.Wet.Vertices.size(); ++i) {
			const PlanetCore::WaterCoverageVertex &U = A.Wet.Vertices[i], &V = B.Wet.Vertices[i];
			// 水際の位置又は参照が異なる場合の返戻
			if(U.SourceA != V.SourceA || U.SourceB != V.SourceB || !SameVector(U.Position, V.Position) ||
				!SameScalar(U.Fraction, V.Fraction))
				return false;
		}
		// 出力全体の一致の返戻
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetWaterCoverageContracts, "PlanetGenerator.WaterCoverage.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 既知の半平面・交差水域で，面積，向き，共有水際と生成順の独立性を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行した場合に真，個別の不一致は UE に記録
 */
bool FPlanetWaterCoverageContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	PlanetCore::LodTree NearTree;
	PlanetCore::TileId NearTile;
	for(unsigned Level = 0; Level < 8; ++Level) {
		const PlanetCore::LodEdit Edit = NearTree.Refine(NearTile, 1024);
		if(!TestTrue(TEXT("深い近景の試験分割を準備"), Edit.Status == PlanetCore::LodEditStatus::Applied)) return false;
		NearTile = *NearTile.Child(0);
	}
	const std::size_t OriginalLeaves = NearTree.Leaves().size();
	const std::optional<PlanetCore::LodTree> Skeleton = PlanetCore::PrepareWaterSkeletonTree(NearTree, 6);
	if(!TestTrue(TEXT("近景を保持して全球の河道骨格を準備"), Skeleton.has_value())) return false;
	TestEqual(TEXT("入力の公開分割は変更しない"), NearTree.Leaves().size(), OriginalLeaves);
	TestTrue(TEXT("骨格より深い足元の葉を保持"), Skeleton->IsLeaf(NearTile));
	for(const PlanetCore::TileId Tile : Skeleton->Leaves()) {
		TestTrue(TEXT("出力は固定骨格以上に細かい"), Tile.Level >= 6);
		TestTrue(TEXT("全出力葉が元の分割の内部に有る"), NearTree.CoveringLeaf(Tile).has_value());
	}
	const std::optional<PlanetCore::AdaptiveSphereMesh> SkeletonMesh = PlanetCore::BuildAdaptiveSphere(*Skeleton);
	const std::optional<PlanetCore::SphereMesh> FixedGraph = PlanetCore::BuildUniformSphere(6);
	if(!TestTrue(TEXT("固定排水分割と水面分割の両方を構築"), SkeletonMesh && FixedGraph)) return false;
	PlanetCore::LodView SkeletonView;
	SkeletonView.RadiusMeters = 1000;
	SkeletonView.ReliefBoundMeters = 0;
	SkeletonView.ObserverMeters = {0, 0, 2000};
	TestTrue(TEXT("通常の視点上限を満たす骨格投影の入力"), SkeletonView.IsValid());
	TestFalse(TEXT("通常の地表分割経路では視点予算超過を引続き拒否"),
		PlanetCore::BuildOceanSurfaceFromTerrain(SkeletonView, 0, *Skeleton, *SkeletonMesh).has_value());
	const std::optional<PlanetCore::OceanSurfaceMesh> SkeletonOcean =
		PlanetCore::BuildOceanSurfaceFromSkeleton(SkeletonView, 0, *Skeleton, *SkeletonMesh);
	if(!TestTrue(TEXT("明示的な骨格経路では同じ正当な大分割を投影可能"), SkeletonOcean.has_value())) return false;
	TestTrue(TEXT("骨格投影も一画素の曲率誤差を満たす"), SkeletonOcean->MaximumProjectionErrorPixels <= 1);
	SkeletonView.LeafBudget = PlanetCore::LodView::MaximumViewLeaves + 1;
	TestFalse(TEXT("骨格経路でも不正な視点設定を通さない"),
		PlanetCore::BuildOceanSurfaceFromSkeleton(SkeletonView, 0, *Skeleton, *SkeletonMesh).has_value());
	std::set<std::array<double, 3>> Directions;
	for(const PlanetCore::UnitDirection D : SkeletonMesh->Mesh.Directions) {
		const PlanetCore::Vec3 V = D.Vector();
		Directions.insert({V.X, V.Y, V.Z});
	}
	for(const PlanetCore::UnitDirection D : FixedGraph->Directions) {
		const PlanetCore::Vec3 V = D.Vector();
		TestTrue(TEXT("全球の排水節点が水面初期分割と厳密に一致"), Directions.contains({V.X, V.Y, V.Z}));
	}
	TestFalse(TEXT("有界な固定骨格の上限を超える段階を拒否"), PlanetCore::PrepareWaterSkeletonTree(NearTree, 7).has_value());
	std::stop_source SkeletonStop;
	SkeletonStop.request_stop();
	TestFalse(TEXT("中断した骨格を公開しない"),
		PlanetCore::PrepareWaterSkeletonTree(NearTree, 6, SkeletonStop.get_token()).has_value());
	AddInfo(FString::Printf(TEXT("Water skeleton: %llu drainage vertices retained, %llu leaves, near level %u preserved."),
		static_cast<unsigned long long>(FixedGraph->Directions.size()),
		static_cast<unsigned long long>(Skeleton->Leaves().size()), NearTile.Level));
	const std::array<double, 5> Coordinates{-1, -0.25, 0, 0.25, 1};
	std::vector<PlanetCore::WaterCoverageSample> Samples;
	std::vector<std::array<std::uint32_t, 3>> Faces;
	for(const double Y : Coordinates)
		for(const double X : Coordinates) Samples.push_back({{X, Y, 0}, X - 0.2});
	for(std::uint32_t y = 0; y < 4; ++y)
		for(std::uint32_t x = 0; x < 4; ++x) {
			const std::uint32_t A = y * 5 + x;
			Faces.push_back({A, A + 1, A + 6});
			Faces.push_back({A, A + 6, A + 5});
		}
	std::uint32_t VerifiedFaces = 0, SharedEdges = 0;
	for(unsigned Case = 0; Case < 2; ++Case) {
		if(Case)
			for(PlanetCore::WaterCoverageSample &Sample : Samples)
				Sample.DepthMeters = std::max(0.25 - std::abs(Sample.Position.X), 0.25 - std::abs(Sample.Position.Y));
		const std::optional<PlanetCore::WaterCoverageMesh> Mesh = PlanetCore::BuildWaterCoverage(Samples, Faces);
		// 解析的な水域を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("解析的な水際を切出す"), Mesh.has_value())) return false;
		double Area = 0;
		std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>> Edges;
		std::set<std::array<std::uint32_t, 3>> UniqueFaces;
		for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) {
			const PlanetCore::Vec3 A = Mesh->Vertices[Face[0]].Position, B = Mesh->Vertices[Face[1]].Position;
			const PlanetCore::Vec3 C = Mesh->Vertices[Face[2]].Position;
			const double SignedArea = (B - A).Cross(C - A).Z * 0.5;
			TestTrue(TEXT("三角形の向きを保持し退化しない"), SignedArea > 0);
			Area += SignedArea;
			std::array<std::uint32_t, 3> Key = Face;
			std::sort(Key.begin(), Key.end());
			TestTrue(TEXT("同じ水面を重複して出力しない"), UniqueFaces.insert(Key).second);
			for(unsigned k = 0; k < 3; ++k) {
				const std::uint32_t U = Face[k], V = Face[(k + 1) % 3];
				std::pair<unsigned, int> &Edge = Edges[{std::min(U, V), std::max(U, V)}];
				++Edge.first;
				Edge.second += U < V ? 1 : -1;
			}
			++VerifiedFaces;
		}
		TestTrue(TEXT("半平面又は交差水域の和集合の面積を保持"), std::abs(Area - (Case ? 1.75 : 1.6)) < 1E-12);
		TestEqual(TEXT("各出力面が元の面を参照"), Mesh->Triangles.size(), Mesh->SourceTriangles.size());
		for(const std::pair<const std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>> &Entry : Edges) {
			TestTrue(TEXT("内部辺は反対向きの二面だけで共有"),
				Entry.second.first == 1 || (Entry.second.first == 2 && Entry.second.second == 0));
			SharedEdges += Entry.second.first == 2 ? 1 : 0;
		}
		TestEqual(TEXT("穴の無い単一の水域の Euler 特性"),
			static_cast<int64>(Mesh->Vertices.size()) - static_cast<int64>(Edges.size()) +
				static_cast<int64>(Mesh->Triangles.size()),
			int64{1});
		for(const PlanetCore::WaterCoverageVertex &Vertex : Mesh->Vertices) {
			TestTrue(TEXT("属性の補間率が同じ辺の範囲内"), Vertex.Fraction >= 0 && Vertex.Fraction <= 1);
			if(Vertex.SourceA == Vertex.SourceB) continue;
			const double Depth =
				std::lerp(Samples[Vertex.SourceA].DepthMeters, Samples[Vertex.SourceB].DepthMeters, Vertex.Fraction);
			TestTrue(TEXT("交点は区分線形な水深が零"), std::abs(Depth) < 1E-14);
		}
		std::vector<std::array<std::uint32_t, 3>> Reversed = Faces;
		std::reverse(Reversed.begin(), Reversed.end());
		const std::optional<PlanetCore::WaterCoverageMesh> Other = PlanetCore::BuildWaterCoverage(Samples, Reversed);
		// 生成順を変えた分割を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("面の処理順を逆転しても構築可能"), Other.has_value())) return false;
		TestEqual(TEXT("処理順に依らず頂点数が一致"), Mesh->Vertices.size(), Other->Vertices.size());
		for(std::size_t i = 0; i < Mesh->Vertices.size(); ++i) {
			const PlanetCore::WaterCoverageVertex &A = Mesh->Vertices[i], &B = Other->Vertices[i];
			TestTrue(TEXT("共有交点の位置と識別が生成順に依らず一致"),
				A.Position.X == B.Position.X && A.Position.Y == B.Position.Y && A.Position.Z == B.Position.Z &&
					A.SourceA == B.SourceA && A.SourceB == B.SourceB && A.Fraction == B.Fraction);
		}
		for(std::array<std::uint32_t, 3> Face : Other->Triangles) {
			std::sort(Face.begin(), Face.end());
			TestTrue(TEXT("処理順に依らず同じ水面を生成"), UniqueFaces.erase(Face) == 1);
		}
		TestTrue(TEXT("逆順の出力に欠けた面が無い"), UniqueFaces.empty());
	}
	const std::array<std::array<std::uint32_t, 3>, 1> OneFace{{{0, 1, 2}}};
	std::array<PlanetCore::WaterCoverageSample, 3> Triangle{{{{0, 0, 0}, 1}, {{1, 0, 0}, 1}, {{0, 1, 0}, 1}}};
	for(unsigned Mask = 0; Mask < 8; ++Mask) {
		for(unsigned k = 0; k < 3; ++k) Triangle[k].DepthMeters = Mask & (1U << k) ? 1E308 : -1E308;
		const std::optional<PlanetCore::WaterCoverageMesh> Mesh = PlanetCore::BuildWaterCoverage(Triangle, OneFace);
		// 有限な大水深で符号差の減算がオーバーフローした場合の返戻
		if(!TestTrue(TEXT("八つの濡れ配置と有限な極大水深を処理"), Mesh.has_value())) return false;
		TestTrue(TEXT("完全に乾いた面だけが空になる"), Mesh->Triangles.empty() == !Mask);
	}
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(TEXT("中断した候補を公開しない"), PlanetCore::BuildWaterCoverage(Samples, Faces, Stop.get_token()).has_value());
	Triangle[0].DepthMeters = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("非有限な水深を拒否"), PlanetCore::BuildWaterCoverage(Triangle, OneFace).has_value());
	Triangle[0].DepthMeters = 1;
	Triangle[1].Position = Triangle[0].Position;
	TestFalse(TEXT("退化した元の面を拒否"), PlanetCore::BuildWaterCoverage(Triangle, OneFace).has_value());
	const std::array<std::array<std::uint32_t, 3>, 1> InvalidFace{{{0, 1, 999}}};
	TestFalse(TEXT("範囲外の頂点番号を拒否"), PlanetCore::BuildWaterCoverage(Samples, InvalidFace).has_value());
	for(const double Radius : {1000.0, 1E6, 1E9}) {
		const std::function<PlanetCore::UnitDirection(double, double)> Direction = [=](const double X, const double Y) {
			// 独立な平面座標を球面方向へ写した返戻
			return *PlanetCore::UnitDirection::FromVector({X / Radius, Y / Radius, 1});
		};
		const std::array<PlanetCore::UnitDirection, 3> TriangleDirections{Direction(-1, -1), Direction(1, -1), Direction(0, 1)};
		TestTrue(TEXT("両端が外側でも三角形を横切る大円弧を検出"),
			PlanetCore::ArcIntersectsSphericalTriangle(Direction(-2, 0), Direction(2, 0), TriangleDirections));
		TestFalse(TEXT("包囲円に近くても三角形を通らない弧は除外"),
			PlanetCore::ArcIntersectsSphericalTriangle(Direction(-1, 0.6), Direction(-0.8, 0.8), TriangleDirections));
		TestFalse(TEXT("三角形の外側の平行な弧は除外"),
			PlanetCore::ArcIntersectsSphericalTriangle(Direction(-2, 2), Direction(2, 2), TriangleDirections));
		TestTrue(TEXT("共有辺に一致する弧を除外しない"),
			PlanetCore::ArcIntersectsSphericalTriangle(Direction(-1, -1), Direction(1, -1), TriangleDirections));
		TestTrue(TEXT("内側の一点に縮んだ弧も検出"),
			PlanetCore::ArcIntersectsSphericalTriangle(Direction(0, 0), Direction(0, 0), TriangleDirections));
		TestTrue(TEXT("既存の水面内を通る区間全体を被覆済みと判定"),
			PlanetCore::ArcCoveredByLinearWater(Direction(-2, 0), Direction(2, 0), TriangleDirections, {1, 1, 1}));
		TestFalse(TEXT("通過区間の片側だけ濡れる面を被覆済みとしない"),
			PlanetCore::ArcCoveredByLinearWater(Direction(-2, 0), Direction(2, 0), TriangleDirections, {-2, 1, 0.2}));
		TestFalse(TEXT("乾いた河道を被覆済みとしない"),
			PlanetCore::ArcCoveredByLinearWater(Direction(-2, 0), Direction(2, 0), TriangleDirections, {-1, -1, -1}));
		TestTrue(TEXT("流路を通さない面に発見用の分割を要求しない"),
			PlanetCore::ArcCoveredByLinearWater(Direction(-2, 2), Direction(2, 2), TriangleDirections, {-1, -1, -1}));
	}
	const PlanetCore::UnitDirection Focus = *PlanetCore::UnitDirection::FromVector({0, 0, 1});
	const PlanetCore::GroundFrame Frame = *PlanetCore::GroundFrame::Create(Focus, 1000, 0);
	PlanetCore::SphereMesh Curved;
	for(const double Y : std::array<double, 5>{-4, -2, 0, 2, 4})
		for(const double X : std::array<double, 5>{-4, -2, 0, 2, 4}) Curved.Directions.push_back(Frame.DirectionAt(X, Y));
	Curved.Triangles = Faces;
	const std::function<PlanetCore::WaterCoverageSample(PlanetCore::UnitDirection)> Evaluate =
		[&](const PlanetCore::UnitDirection D) {
			const double X = D.Vector().X * Frame.RadiusMeters;
			// 曲がった水面と解析的な水際を持つ場の返戻
			return PlanetCore::WaterCoverageSample{Frame.SurfaceToLocal(D, 0.04 * X * X), 2.3 - 0.4 * X * X};
		};
	PlanetCore::WaterRefinementDiagnostics Diagnostics;
	const std::optional<PlanetCore::AdaptiveWaterCoverage> Adaptive =
		PlanetCore::BuildAdaptiveWaterCoverage(Curved, Frame, Focus, 8, Evaluate, Diagnostics);
	// 曲面の細分化候補を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("曲がった水面と水際を局所細分化"), Adaptive.has_value())) return false;
	TestTrue(TEXT("精度不足を細分化して解消"), Adaptive->RefinementPasses > 0 && Adaptive->RefinementPasses <= 8);
	std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>> PartitionEdges;
	for(const std::array<std::uint32_t, 3> &T : Adaptive->Partition.Triangles)
		for(unsigned k = 0; k < 3; ++k) {
			const std::uint32_t A = T[k], B = T[(k + 1) % 3];
			std::pair<unsigned, int> &Edge = PartitionEdges[{std::min(A, B), std::max(A, B)}];
			++Edge.first;
			Edge.second += A < B ? 1 : -1;
		}
	for(const std::pair<const std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>> &Entry : PartitionEdges) {
		TestTrue(TEXT("細分化後も辺を二面以下で整合して共有"),
			Entry.second.first == 1 || (Entry.second.first == 2 && Entry.second.second == 0));
		if(Entry.second.first != 1) continue;
		for(const std::uint32_t Index : {Entry.first.first, Entry.first.second}) {
			const PlanetCore::Vec3 D = Adaptive->Partition.Directions[Index].Vector();
			const double X = D.X / D.Z * 1000, Y = D.Y / D.Z * 1000;
			TestTrue(TEXT("一面だけの辺は外周に限り内部の裂目を作らない"),
				std::abs(std::abs(X) - 4) < 1E-9 || std::abs(std::abs(Y) - 4) < 1E-9);
		}
	}
	TestEqual(TEXT("細分化した参照元の Euler 特性を保持"),
		static_cast<int64>(Adaptive->Partition.Directions.size()) - static_cast<int64>(PartitionEdges.size()) +
			static_cast<int64>(Adaptive->Partition.Triangles.size()),
		int64{1});
	double IndependentError = 0, AnalyticShoreError = 0;
	for(const std::array<std::uint32_t, 3> &T : Adaptive->Wet.Triangles)
		for(const std::array<double, 3> Weight :
			std::array<std::array<double, 3>, 2>{{{0.19, 0.37, 0.44}, {0.61, 0.28, 0.11}}}) {
			const PlanetCore::Vec3 P = Adaptive->Wet.Vertices[T[0]].Position * Weight[0] +
				Adaptive->Wet.Vertices[T[1]].Position * Weight[1] + Adaptive->Wet.Vertices[T[2]].Position * Weight[2];
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(Frame.LocalToPlanet(P));
			IndependentError = std::max(IndependentError, (P - Evaluate(D).Position).Length());
		}
	for(const PlanetCore::WaterCoverageVertex &Vertex : Adaptive->Wet.Vertices)
		if(Vertex.SourceA != Vertex.SourceB) {
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(Frame.LocalToPlanet(Vertex.Position));
			AnalyticShoreError = std::max(AnalyticShoreError, std::abs(Evaluate(D).DepthMeters));
		}
	TestTrue(TEXT("独立な水面内点も一センチメートル以内"), IndependentError <= 0.01);
	TestTrue(TEXT("解析的な水際との水深差は二センチメートル以内"), AnalyticShoreError <= 0.02);
	TestFalse(TEXT("細分化中断候補を公開しない"),
		PlanetCore::BuildAdaptiveWaterCoverage(Curved, Frame, Focus, 8, Evaluate, Diagnostics, Stop.get_token()).has_value());
	TestFalse(TEXT("不正な近景半径を拒否"),
		PlanetCore::BuildAdaptiveWaterCoverage(Curved, Frame, Focus, -1, Evaluate, Diagnostics).has_value());
	const PlanetCore::WaterCoverageView View{{0, 0, 1008}, 540, 8, 980};
	const std::function<PlanetCore::WaterCoverageSample(PlanetCore::UnitDirection)> HalfOcean =
		[&](const PlanetCore::UnitDirection D) {
			// 全球を横切る解析的な水際の返戻
			return PlanetCore::WaterCoverageSample{Frame.SurfaceToLocal(D, 0), D.Vector().X * 1000 - 0.123};
		};
	const std::optional<PlanetCore::AdaptiveWaterCoverage> Hierarchical = PlanetCore::BuildAdaptiveWaterCoverage(
		SkeletonOcean->Sphere, Frame, Focus, 8, HalfOcean, Diagnostics, {}, &View, {}, SkeletonOcean->Hierarchy);
	// 本番と同じ階層経路が候補を作れなかった場合の返戻
	if(!TestTrue(TEXT("接合面の親区画を細分化して水際を収束"), Hierarchical && Hierarchical->Hierarchy)) return false;
	TestTrue(TEXT("元区画の細分化が実際に発生"), Hierarchical->RefinementPasses > 0);
	const PlanetCore::WaterRefinementDiagnostics CachedDiagnostics = Diagnostics;
	PlanetCore::WaterRefinementDiagnostics FullDiagnostics;
	const std::optional<PlanetCore::AdaptiveWaterCoverage> FullHierarchy = PlanetCore::BuildAdaptiveWaterCoverage(
		SkeletonOcean->Sphere, Frame, Focus, 8, HalfOcean, FullDiagnostics, {}, &View, {}, SkeletonOcean->Hierarchy, false);
	// 検査を全てやり直した参照結果との不一致の返戻
	if(!TestTrue(
		   TEXT("検査再利用の有無で全球分割と水際がビット一致"), FullHierarchy && SameCoverage(*Hierarchical, *FullHierarchy)))
		return false;
	TestTrue(TEXT("変更の無い面の検査を実際に再利用"), CachedDiagnostics.SourceCheckHits > 0);
	TestTrue(TEXT("変更の無い水面頂点の検査を実際に再利用"), CachedDiagnostics.VertexCheckHits > 0);
	TestTrue(TEXT("全検査と再利用の面数が一致"),
		CachedDiagnostics.SourceCheckHits + CachedDiagnostics.SourceCheckEvaluations ==
				FullDiagnostics.SourceCheckEvaluations &&
			FullDiagnostics.SourceCheckHits == 0);
	TestTrue(TEXT("全検査と再利用の水面頂点数が一致"),
		CachedDiagnostics.VertexCheckHits + CachedDiagnostics.VertexCheckEvaluations ==
				FullDiagnostics.VertexCheckEvaluations &&
			FullDiagnostics.VertexCheckHits == 0);
	TestTrue(TEXT("最大誤差と水際の投影寸法も再利用の有無でビット一致"),
		SameScalar(CachedDiagnostics.WaterErrorMeters, FullDiagnostics.WaterErrorMeters) &&
			SameScalar(CachedDiagnostics.ShoreErrorMeters, FullDiagnostics.ShoreErrorMeters) &&
			SameScalar(CachedDiagnostics.FloatErrorMeters, FullDiagnostics.FloatErrorMeters) &&
			SameScalar(CachedDiagnostics.MaximumShoreEdgePixels, FullDiagnostics.MaximumShoreEdgePixels));
	AddInfo(
		FString::Printf(TEXT("Water source checks: hits=%llu evaluated=%llu full=%llu; topology and attributes bit-identical."),
			static_cast<unsigned long long>(CachedDiagnostics.SourceCheckHits),
			static_cast<unsigned long long>(CachedDiagnostics.SourceCheckEvaluations),
			static_cast<unsigned long long>(FullDiagnostics.SourceCheckEvaluations)));
	TestTrue(TEXT("階層経路でも水際を八画素以内に制限"), Diagnostics.MaximumShoreEdgePixels <= 8);
	TestEqual(
		TEXT("階層経路は共通頂点の鍵を保持"), Hierarchical->Partition.Keys.size(), Hierarchical->Partition.Directions.size());
	TestEqual(TEXT("全出力面の元区画を保持"), Hierarchical->Hierarchy->TriangleOwners.size(),
		Hierarchical->Partition.Triangles.size());
	std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>> HierarchicalEdges;
	for(std::size_t i = 0; i < Hierarchical->Partition.Triangles.size(); ++i) {
		TestTrue(TEXT("出力面は分割済の接合面でなく有効な葉に所属"),
			Hierarchical->Hierarchy->Tree.IsLeaf(Hierarchical->Hierarchy->TriangleOwners[i]));
		const std::array<std::uint32_t, 3> &T = Hierarchical->Partition.Triangles[i];
		for(unsigned k = 0; k < 3; ++k) {
			const std::uint32_t A = T[k], B = T[(k + 1) % 3];
			std::pair<unsigned, int> &Edge = HierarchicalEdges[{std::min(A, B), std::max(A, B)}];
			++Edge.first;
			Edge.second += A < B ? 1 : -1;
		}
	}
	for(const std::pair<const std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>> &Entry : HierarchicalEdges)
		TestTrue(TEXT("全球の全共有辺が反対向きの二面で閉じる"), Entry.second.first == 2 && !Entry.second.second);
	TestEqual(TEXT("階層細分化後も球面の Euler 特性を保持"),
		static_cast<int64>(Hierarchical->Partition.Directions.size()) - static_cast<int64>(HierarchicalEdges.size()) +
			static_cast<int64>(Hierarchical->Partition.Triangles.size()),
		int64{2});
	std::set<std::array<double, 3>> RetainedDirections;
	for(const PlanetCore::UnitDirection D : Hierarchical->Partition.Directions) {
		const PlanetCore::Vec3 V = D.Vector();
		RetainedDirections.insert({V.X, V.Y, V.Z});
	}
	for(const PlanetCore::UnitDirection D : FixedGraph->Directions) {
		const PlanetCore::Vec3 V = D.Vector();
		TestTrue(TEXT("細分化後も全排水節点を厳密に保持"), RetainedDirections.contains({V.X, V.Y, V.Z}));
	}
	PlanetCore::SphereRefinementHierarchy InvalidHierarchy = *SkeletonOcean->Hierarchy;
	InvalidHierarchy.TriangleOwners.pop_back();
	TestFalse(TEXT("面と親区画が対応しない入力を拒否"),
		PlanetCore::BuildAdaptiveWaterCoverage(
			SkeletonOcean->Sphere, Frame, Focus, 8, HalfOcean, Diagnostics, {}, &View, {}, std::move(InvalidHierarchy))
			.has_value());
	AddInfo(FString::Printf(TEXT("Hierarchical water: %u passes, %llu vertices, closed sphere and 40962 fixed nodes retained."),
		Hierarchical->RefinementPasses, static_cast<unsigned long long>(Hierarchical->Partition.Directions.size())));
	const std::optional<PlanetCore::AdaptiveWaterCoverage> Projected =
		PlanetCore::BuildAdaptiveWaterCoverage(Curved, Frame, Focus, 8, Evaluate, Diagnostics, {}, &View);
	// 投影寸法を制限した水際を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("水際の投影寸法でも細分化"), Projected.has_value())) return false;
	TestTrue(
		TEXT("画面上の水際に必要な頂点を追加"), Projected->Partition.Directions.size() > Adaptive->Partition.Directions.size());
	double ProjectedEdge = 0;
	std::map<std::pair<std::uint32_t, std::uint32_t>, unsigned> WetEdges;
	for(const std::array<std::uint32_t, 3> &T : Projected->Wet.Triangles)
		for(unsigned k = 0; k < 3; ++k) ++WetEdges[{std::min(T[k], T[(k + 1) % 3]), std::max(T[k], T[(k + 1) % 3])}];
	for(const std::pair<const std::pair<std::uint32_t, std::uint32_t>, unsigned> &Entry : WetEdges) {
		if(Entry.second != 1) continue;
		const PlanetCore::WaterCoverageVertex &A = Projected->Wet.Vertices[Entry.first.first];
		const PlanetCore::WaterCoverageVertex &B = Projected->Wet.Vertices[Entry.first.second];
		if(A.SourceA == A.SourceB || B.SourceA == B.SourceB) continue;
		const PlanetCore::Vec3 P = Frame.LocalToPlanet(A.Position), Q = Frame.LocalToPlanet(B.Position);
		// 端点迄の距離を使う独立な投影評価は，構築時の線分最短距離に依る上界以下になる
		ProjectedEdge = std::max(ProjectedEdge,
			(P - Q).Length() * 540 / std::min((P - View.ObserverMeters).Length(), (Q - View.ObserverMeters).Length()));
	}
	TestTrue(TEXT("水際の辺を独立な投影評価でも八画素以下に保持"), ProjectedEdge > 0 && ProjectedEdge <= 8);
	PlanetCore::WaterCoverageView HiddenView = View;
	HiddenView.ObserverMeters = {0, 0, -1008};
	const std::optional<PlanetCore::AdaptiveWaterCoverage> Hidden =
		PlanetCore::BuildAdaptiveWaterCoverage(Curved, Frame, Focus, 8, Evaluate, Diagnostics, {}, &HiddenView);
	TestTrue(TEXT("惑星内部球に確実に隠れる辺には投影細分化を要求しない"),
		Hidden && Hidden->Partition.Directions.size() == Adaptive->Partition.Directions.size() &&
			Hidden->Wet.Triangles == Adaptive->Wet.Triangles);
	PlanetCore::WaterCoverageView InvalidView = View;
	InvalidView.FocalLengthPixels = 0;
	TestFalse(TEXT("不正な投影条件を拒否"),
		PlanetCore::BuildAdaptiveWaterCoverage(Curved, Frame, Focus, 8, Evaluate, Diagnostics, {}, &InvalidView).has_value());
	AddInfo(FString::Printf(TEXT("Projected shoreline: %u passes, %llu vertices, maximum independent edge %.9g pixels."),
		Projected->RefinementPasses, static_cast<unsigned long long>(Projected->Partition.Directions.size()), ProjectedEdge));
	const std::function<PlanetCore::WaterCoverageSample(PlanetCore::UnitDirection)> ThinWater =
		[&](const PlanetCore::UnitDirection D) {
			// 初期頂点と通常の面内標本が外れる細い解析流路の返戻
			return PlanetCore::WaterCoverageSample{Frame.SurfaceToLocal(D, 0), 0.02 - std::abs(D.Vector().X * 1000 - 0.123)};
		};
	std::vector<PlanetCore::WaterCoverageSample> ThinSamples;
	for(const PlanetCore::UnitDirection D : Curved.Directions) ThinSamples.push_back(ThinWater(D));
	const std::optional<PlanetCore::WaterCoverageMesh> MissingThin =
		PlanetCore::BuildWaterCoverage(ThinSamples, Curved.Triangles);
	TestTrue(TEXT("頂点だけでは細い流路を見落す解析条件"), MissingThin && MissingThin->Triangles.empty());
	const PlanetCore::WaterDetailEvaluator ThinResolution = [](const PlanetCore::UnitDirection Center, const double Cap,
																const std::array<PlanetCore::UnitDirection, 3> &Face,
																const std::array<double, 3> &Depths) {
		const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector({0.123 / 1000, -0.004, 1});
		const PlanetCore::UnitDirection End = *PlanetCore::UnitDirection::FromVector({0.123 / 1000, 0.004, 1});
		// 既に覆う内部を分割せず，既知の流路が欠ける面だけを発見する返戻
		if(std::abs(Center.Vector().X * 1000 - 0.123) > Cap * 1000 + 0.02 ||
			PlanetCore::ArcCoveredByLinearWater(Start, End, Face, Depths))
			return std::numeric_limits<double>::infinity();
		// 未発見の流路へ必要な辺長の返戻
		return 0.02;
	};
	const std::optional<PlanetCore::AdaptiveWaterCoverage> Thin =
		PlanetCore::BuildAdaptiveWaterCoverage(Curved, Frame, Focus, 8, ThinWater, Diagnostics, {}, nullptr, ThinResolution);
	// 既知の細い流路を確保出来なかった場合の返戻
	if(!TestTrue(TEXT("全初期頂点が乾いていても既知の流路を確保"), Thin && !Thin->Wet.Triangles.empty())) return false;
	PlanetCore::WaterRefinementDiagnostics FullThinDiagnostics;
	const std::optional<PlanetCore::AdaptiveWaterCoverage> FullThin = PlanetCore::BuildAdaptiveWaterCoverage(
		Curved, Frame, Focus, 8, ThinWater, FullThinDiagnostics, {}, nullptr, ThinResolution, {}, false);
	// 頂点追加方式と細流条件でも，再利用が欠落や順序差を作らない事を確認した返戻
	if(!TestTrue(TEXT("非階層の細流でも全検査と再利用の出力がビット一致"), FullThin && SameCoverage(*Thin, *FullThin)))
		return false;
	std::vector<PlanetCore::UnitDirection> ThinDirections;
	for(const PlanetCore::WaterCoverageVertex &Vertex : Thin->Wet.Vertices)
		ThinDirections.push_back(*PlanetCore::UnitDirection::FromVector(Frame.LocalToPlanet(Vertex.Position)));
	for(unsigned k = 0; k < 17; ++k) {
		const double X = 0.123 / 1000, Y = (-3.8 + 7.6 * k / 16) / 1000;
		const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector({X, Y, std::sqrt(1 - X * X - Y * Y)});
		bool Covered = false;
		for(const std::array<std::uint32_t, 3> &T : Thin->Wet.Triangles)
			if(PlanetCore::SphericalTriangleChordDistance(
				   D, {ThinDirections[T[0]], ThinDirections[T[1]], ThinDirections[T[2]]}) < 1E-12) {
				Covered = true;
				break;
			}
		TestTrue(TEXT("独立な細流中心１７地点に描画の欠落が無い"), Covered);
	}
	AddInfo(FString::Printf(TEXT("Thin water: %u passes, %llu source vertices, 17 independent channel points."),
		Thin->RefinementPasses, static_cast<unsigned long long>(Thin->Partition.Directions.size())));
	AddInfo(FString::Printf(TEXT("Adaptive water: %u passes, %llu vertices, independent %.9g m, shore %.9g m."),
		Adaptive->RefinementPasses, static_cast<unsigned long long>(Adaptive->Partition.Directions.size()), IndependentError,
		AnalyticShoreError));
	AddInfo(FString::Printf(TEXT("Water coverage: %u analytic faces, %u shared edges; 1.6 and 1.75 square metre areas."),
		VerifiedFaces, SharedEdges));
	// 個別の結果を検査器へ記録した上での返戻
	return true;
}
#endif
