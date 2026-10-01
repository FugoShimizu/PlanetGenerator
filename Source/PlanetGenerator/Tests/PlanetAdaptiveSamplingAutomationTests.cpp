#include "Core/PlanetDrainageSampling.h"
#include "Core/PlanetSphericalMesh.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	using PlanetCore::DrainageSamplingMesh;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;

	using Triangle = std::array<std::uint32_t, 3>;

	/**
	 * 有向性に依存せず，網の全ての共有辺を集める
	 * @param Triangles 比較する三角形
	 * @return 両端の番号で識別した辺集合
	 */
	std::set<std::uint64_t> CollectSamplingEdges(const std::span<const Triangle> Triangles) {
		// 頂点番号に依る辺の識別
		std::set<std::uint64_t> Result;
		for(const Triangle &Face : Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			Result.insert(static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B));
		}
		// 重複を除いた全辺の返戻
		return Result;
	}

	/**
	 * 元の三角形へ追加する厳密な内部点を選ぶ
	 * @param Mesh 元の節点と面
	 * @param FaceIndex 元の面番号
	 * @return 元面の内部に在る単位方向
	 */
	UnitDirection SamplingInterior(const DrainageSamplingMesh &Mesh, const std::size_t FaceIndex) {
		const Triangle &Face = Mesh.Triangles[FaceIndex];
		// 正の重みに依る厳密な内部点の返戻
		return *UnitDirection::FromVector(
			Mesh.Directions[Face[0]].Vector() * 0.31 + Mesh.Directions[Face[1]].Vector() * 0.47 + Mesh.Directions[Face[2]].Vector() * 0.22
		);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetAdaptiveSamplingInsertion, "PlanetGenerator.Drainage.AdaptiveSampling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 適応済の辺を固定した河口挿入と，頂点番号・閉被覆・入力順の不変性を検査する
 * @param Parameters 未使用の UE 引数
 * @return 全ての位置と位相の条件を満たす場合に真
 */
bool FPlanetAdaptiveSamplingInsertion::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::BuildDrainageSamplingMesh;
	using PlanetCore::BuildDrainageSamplingMeshOnMesh;
	// 元の球面と，先に確定する適応位置
	const std::optional<DrainageSamplingMesh> Base = BuildDrainageSamplingMesh({});
	// 基準球面を準備出来ない場合の返戻
	if(!TestTrue(TEXT("適応挿入の基準球面を準備"), Base.has_value())) return false;
	std::vector<UnitDirection> Prior;
	for(unsigned Index = 0; Index < 12; ++Index) Prior.push_back(SamplingInterior(*Base, Index * 6799 % Base->Triangles.size()));
	const std::optional<DrainageSamplingMesh> Initial = BuildDrainageSamplingMesh(Prior);
	// 元の適応網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("先行する適応網を準備"), Initial.has_value())) return false;
	const std::optional<DrainageSamplingMesh> Empty = BuildDrainageSamplingMeshOnMesh(Initial->Directions, Initial->Triangles, {});
	// 空の追加でも元の適応網を拒否しない返戻
	if(!TestTrue(TEXT("適応網へ追加０点を受理"), Empty.has_value())) return false;
	TestTrue(TEXT("追加０点で先行する全ての面を保持"), Empty->Triangles == Initial->Triangles);
	// 適応点を含む面だけに追加し，元の共有辺の途中と面内部を別々に検査する
	std::vector<UnitDirection> Samples;
	std::vector<std::array<std::uint32_t, 2>> SplitEdges;
	std::set<std::uint64_t> ExpectedSplits;
	for(std::size_t Index = 0; Index < Initial->Triangles.size() && SplitEdges.size() < 12; ++Index) {
		const Triangle &Face = Initial->Triangles[Index];
		if(*std::max_element(Face.begin(), Face.end()) < Base->Directions.size()) continue;
		const std::uint32_t A = Face[0], B = Face[1];
		const std::uint64_t Key = static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B);
		if(ExpectedSplits.contains(Key)) continue;
		ExpectedSplits.insert(Key);
		SplitEdges.push_back({ A, B });
		Samples.push_back(*UnitDirection::FromVector(Initial->Directions[A].Vector() + Initial->Directions[B].Vector()));
		Samples.push_back(SamplingInterior(*Initial, Index));
	}
	TestTrue(TEXT("適応面の共有辺と内部を実際に検査"), SplitEdges.size() == 12);
	Samples.push_back(Initial->Directions.back());
	Samples.push_back(Samples.front());
	const std::optional<DrainageSamplingMesh> Result =
	BuildDrainageSamplingMeshOnMesh(Initial->Directions, Initial->Triangles, Samples);
	// 河口に相当する位置を追加出来ない場合の返戻
	if(!TestTrue(TEXT("先行する網を保持して追加"), Result.has_value())) return false;
	// 対応配列の欠落を添字参照前に検出する返戻
	if(!TestTrue(TEXT("全入力の対応番号を保持"), Result->SampleNodes.size() == Samples.size())) return false;
	const std::set<std::uint64_t> Before = CollectSamplingEdges(Initial->Triangles);
	const std::set<std::uint64_t> After = CollectSamplingEdges(Result->Triangles);
	std::uint32_t Preserved = 0;
	for(const std::uint64_t Edge : Before) {
		if(ExpectedSplits.contains(Edge)) continue;
		TestTrue(TEXT("分割しない既存辺を組替えない"), After.contains(Edge));
		++Preserved;
	}
	for(std::size_t Index = 0; Index < SplitEdges.size(); ++Index) {
		const std::uint32_t Mid = Result->SampleNodes[Index * 2];
		for(const std::uint32_t Endpoint : SplitEdges[Index]) {
			const std::uint64_t Key = static_cast<std::uint64_t>(std::min(Endpoint, Mid)) << 32 | std::max(Endpoint, Mid);
			TestTrue(TEXT("分割した元辺の両半分を保持"), After.contains(Key));
		}
	}
	for(std::size_t Index = 0; Index < Initial->Directions.size(); ++Index) {
		const double Difference = (Initial->Directions[Index].Vector() - Result->Directions[Index].Vector()).Length();
		TestTrue(TEXT("適応頂点の番号と位置を保持"), Difference == 0);
	}
	double MaximumPositionError = 0;
	for(std::size_t Index = 0; Index < Samples.size(); ++Index) {
		const Vec3 Difference = Result->Directions[Result->SampleNodes[Index]].Vector() - Samples[Index].Vector();
		MaximumPositionError = std::max(MaximumPositionError, Difference.Length());
	}
	TestTrue(TEXT("全ての追加位置を保持"), MaximumPositionError < 1.5E-14);
	TestTrue(TEXT("追加後も閉じた外向きの球面"), PlanetCore::IsClosedSphericalSamplingMesh(Result->Directions, Result->Triangles));
	TestTrue(TEXT("独立な辺数に依る Euler 特性"), Result->Directions.size() + Result->Triangles.size() == After.size() + 2);
	// 追加位置の順序だけを変え，先行する位相へ影響しない事を確認する
	std::reverse(Samples.begin(), Samples.end());
	const std::optional<DrainageSamplingMesh> Reordered =
	BuildDrainageSamplingMeshOnMesh(Initial->Directions, Initial->Triangles, Samples);
	// 逆順の正規経路が通らない場合の返戻
	if(!TestTrue(TEXT("逆順の追加位置を受理"), Reordered.has_value())) return false;
	TestTrue(TEXT("追加順変更で位相を変えない"), Reordered->Triangles == Result->Triangles);
	for(std::size_t Index = 0; Index < Samples.size(); ++Index) {
		TestTrue(TEXT("逆順に対応する頂点番号"), Reordered->SampleNodes[Index] == Result->SampleNodes[Samples.size() - 1 - Index]);
	}
	// 別の頂点番号と不正な位相を持つ入力の拒否
	std::vector<UnitDirection> Mismatched = Initial->Directions;
	std::swap(Mismatched[0], Mismatched[1]);
	TestFalse(TEXT("異なる元番号の拒否"), BuildDrainageSamplingMeshOnMesh(Mismatched, Initial->Triangles, {}).has_value());
	std::vector<Triangle> MissingFace = Initial->Triangles;
	MissingFace.pop_back();
	TestFalse(TEXT("欠落した元面の拒否"), BuildDrainageSamplingMeshOnMesh(Initial->Directions, MissingFace, {}).has_value());
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断した適応挿入の拒否"),
		BuildDrainageSamplingMeshOnMesh(Initial->Directions, Initial->Triangles, Samples, Cancelled.get_token()).has_value()
	);
	AddInfo(
		FString::Printf(
			TEXT("Adaptive sampling insertion: initial=%llu inserted=%llu preservedEdges=%u splitEdges=%llu positionError=%.12g"),
			static_cast<unsigned long long>(Initial->Directions.size()),
			static_cast<unsigned long long>(Result->Directions.size() - Initial->Directions.size()),
			Preserved,
			static_cast<unsigned long long>(SplitEdges.size()),
			MaximumPositionError
		)
	);
	// 既存辺と正規の位置を保持した挿入の検査結果の返戻
	return !HasAnyErrors();
}

#endif
