#include "Core/PlanetDrainageSampling.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetDrainageSamplingContracts, "PlanetGenerator.Drainage.Sampling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 任意位置・共有辺・既存頂点・重複の追加と，閉被覆・外向き・面積・入力順不変性を検査する
 * @param Parameters 未使用の UE 検査引数
 * @return 全ての位相と位置の条件を満たす場合に真
 */
bool FPlanetDrainageSamplingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::BuildDrainageSamplingMesh;
	using PlanetCore::DrainageSamplingMesh;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(6);
	const std::optional<DrainageSamplingMesh> Empty = BuildDrainageSamplingMesh({});
	// 有効な基準網を作れない場合の返戻
	if(!TestTrue(TEXT("基準網と追加なしの標本網を準備"), Base.has_value() && Empty.has_value())) return false;
	TestTrue(TEXT("追加なしの全ての面と順序が一致"), Empty->Triangles == Base->Triangles);
	TestEqual(TEXT("追加なしの全ての頂点数が一致"), Empty->Directions.size(), Base->Directions.size());
	std::vector<UnitDirection> Samples;
	for(unsigned Case = 0; Case < 48; ++Case) {
		const std::array<std::uint32_t, 3> Face = Base->Triangles[Case * 1699 % Base->Triangles.size()];
		const Vec3 A = Base->Directions[Face[0]].Vector(), B = Base->Directions[Face[1]].Vector();
		const Vec3 C = Base->Directions[Face[2]].Vector();
		Samples.push_back(Base->Directions[Face[0]]);
		Samples.push_back(*UnitDirection::FromVector(A + B));
		Samples.push_back(*UnitDirection::FromVector(A * 3 + B));
		Samples.push_back(*UnitDirection::FromVector(A + B * 3));
		Samples.push_back(*UnitDirection::FromVector(A + B + C));
		Samples.push_back(*UnitDirection::FromVector(A * 0.31 + B * 0.47 + C * 0.22));
		Samples.push_back(Samples.back());
	}
	const std::optional<DrainageSamplingMesh> Mesh = BuildDrainageSamplingMesh(Samples);
	// 正規の位置の追加を拒否した場合の返戻
	if(!TestTrue(TEXT("元辺に束縛されない全ての標本位置を受理"), Mesh.has_value())) return false;
	TestEqual(TEXT("全入力に対応する頂点番号を保持"), Mesh->SampleNodes.size(), Samples.size());
	for(std::size_t Index = 0; Index < Base->Directions.size(); ++Index) {
		TestEqual(TEXT("元頂点の位置と番号を維持"), (Mesh->Directions[Index].Vector() - Base->Directions[Index].Vector()).Length(), 0.0);
	}
	double MaximumPositionError = 0;
	for(std::size_t Index = 0; Index < Samples.size(); ++Index) {
		// 範囲外の対応番号を参照しない返戻
		if(!TestTrue(TEXT("入力から有効な頂点へ対応"), Mesh->SampleNodes[Index] < Mesh->Directions.size())) return false;
		const double Error = (Mesh->Directions[Mesh->SampleNodes[Index]].Vector() - Samples[Index].Vector()).Length();
		MaximumPositionError = std::max(MaximumPositionError, Error);
		TestTrue(TEXT("追加地点は正規化の数値精度で一致"), Error < 1.5E-14);
	}
	std::map<std::array<std::uint32_t, 2>, std::pair<unsigned, int>> Incidence;
	double Area = 0;
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
		const Vec3 A = Mesh->Directions[Triangle[0]].Vector(), B = Mesh->Directions[Triangle[1]].Vector();
		const Vec3 C = Mesh->Directions[Triangle[2]].Vector();
		const double Orientation = A.Dot((B - A).Cross(C - A));
		TestTrue(TEXT("局所追加した全ての面が外向きで非退化"), Orientation > 0);
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
		TestEqual(TEXT("追加後も各辺に二面だけ接続"), Use.first, 2U);
		TestEqual(TEXT("追加後も辺の向きが相殺"), Use.second, 0);
	}
	const std::size_t Inserted = Mesh->Directions.size() - Base->Directions.size();
	TestEqual(TEXT("面数の増加が局所頂点数に対応"), Mesh->Triangles.size(), Base->Triangles.size() + 2 * Inserted);
	TestEqual(TEXT("閉じた球面の Euler 特性を保持"), Mesh->Directions.size() + Mesh->Triangles.size(), Incidence.size() + 2);
	const double AreaError = std::abs(Area - 4 * 3.14159265358979323846);
	TestTrue(TEXT("全球の面積を保持"), AreaError < 1E-10);
	std::reverse(Samples.begin(), Samples.end());
	const std::optional<DrainageSamplingMesh> Reordered = BuildDrainageSamplingMesh(Samples);
	// 入力順だけを変えた正規経路を拒否した場合の返戻
	if(!TestTrue(TEXT("任意地点の逆順も受理"), Reordered.has_value())) return false;
	TestTrue(TEXT("入力順変更後も同じ面と頂点番号"), Reordered->Triangles == Mesh->Triangles);
	TestEqual(TEXT("入力順変更後も同じ頂点数"), Reordered->Directions.size(), Mesh->Directions.size());
	for(std::size_t Index = 0; Index < Samples.size(); ++Index) {
		TestEqual(TEXT("入力番号の逆順に合わせて対応を返す"), Reordered->SampleNodes[Index], Mesh->SampleNodes[Samples.size() - 1 - Index]);
	}
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("局所追加の中断を受理しない"), BuildDrainageSamplingMesh(Samples, Cancel.get_token()).has_value());
	const std::vector<UnitDirection> TooMany(16385, Samples.front());
	TestFalse(TEXT("重複に依らず準備作業量の上限を守る"), BuildDrainageSamplingMesh(TooMany).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Drainage sampling: inputs=%llu inserted=%llu triangles=%llu positionError=%.12g areaError=%.12g."),
			static_cast<unsigned long long>(Samples.size()),
			static_cast<unsigned long long>(Inserted),
			static_cast<unsigned long long>(Mesh->Triangles.size()),
			MaximumPositionError,
			AreaError
		)
	);
	// 位相・位置・面積・正規入力の再現性の返戻
	return !HasAnyErrors();
}

#endif
