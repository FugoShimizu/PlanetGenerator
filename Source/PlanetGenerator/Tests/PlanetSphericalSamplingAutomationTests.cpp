#include "Core/PlanetSphericalRefinement.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalSamplingContracts, "PlanetGenerator.SphericalRelief.PredrainageSampling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 粗い節点間の谷を盛土で消さず，排水前の標本として保持する
 * @param Parameters 未使用の UE 引数
 * @return 元標高・閉球面・予算・不正入力の条件を満たす場合に真
 */
bool FPlanetSphericalSamplingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetSphericalRefinement;
	using PlanetCore::SphericalReliefSamplingMesh;
	using PlanetCore::SphericalReliefSettings;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	using Triangle = std::array<std::uint32_t, 3>;
	using Edge = std::pair<std::uint32_t, std::uint32_t>;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(2);
	// 元網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("排水前の標本化用球面を準備"), Mesh.has_value())) return false;
	std::optional<UnitDirection> Center;
	for(const Triangle &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3 && !Center; ++Side) {
		const Vec3 A = Mesh->Directions[Face[Side]].Vector(), B = Mesh->Directions[Face[(Side + 1) % 3]].Vector();
		if(A.Z > 0.4 && B.Z > 0.4 && std::abs(A.Z - B.Z) < 0.02) Center = UnitDirection::FromVector(A + B);
	}
	// 節点間の解析的な谷を配置出来ない場合の返戻
	if(!TestTrue(TEXT("辺中点の谷を配置"), Center.has_value())) return false;
	const auto Source = [Center](const UnitDirection Direction) -> double {
		const Vec3 P = Direction.Vector(), D = P - Center->Vector();
		// 全て陸上に有る，元節点間の深い谷の返戻
		return 600 + 80 * P.Z - 200 * std::exp(-D.Dot(D) / (2 * 0.035 * 0.035));
	};
	std::vector<double> Heights;
	std::vector<std::uint32_t> Routes;
	for(std::uint32_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		Heights.push_back(Source(Mesh->Directions[Index]));
		Routes.push_back(Index);
	}
	SphericalReliefSettings Settings;
	Settings.RadiusMeters = 1000;
	Settings.MaximumAddedVertices = 512;
	Settings.TargetErrorMeters = 0.5;
	const std::optional<PlanetSphericalRefinement> Conditioned =
	PlanetSphericalRefinement::CreateWithRefinedEdges(Mesh->Directions, Mesh->Triangles, Heights, Routes, Source, Settings);
	const std::optional<SphericalReliefSamplingMesh> Sampled =
	PlanetSphericalRefinement::CreateSamplingMesh(Mesh->Directions, Mesh->Triangles, Heights, Source, Settings);
	// 比較する両方の網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("補正付と排水前の網を準備"), Conditioned.has_value() && Sampled.has_value())) return false;
	const std::size_t Count = Sampled->Directions.size();
	TestTrue(TEXT("予算内で元節点間を標本化"), Count > Heights.size() && Count <= Heights.size() + Settings.MaximumAddedVertices);
	TestEqual(TEXT("閉球面の Euler 特性を保持"), Sampled->Triangles.size(), 2 * (Count - 2));
	std::map<Edge, std::pair<unsigned, int>> Edges;
	for(const Triangle &Face : Sampled->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
		std::pair<unsigned, int> &Owners = Edges[{ std::min(A, B), std::max(A, B) }];
		++Owners.first;
		Owners.second += A < B ? 1 : -1;
	}
	for(const std::pair<const Edge, std::pair<unsigned, int>> &Entry : Edges) {
		TestTrue(TEXT("共有辺は逆向きの２所有面を持つ"), Entry.second.first == 2 && !Entry.second.second);
	}
	double MaximumSourceError = 0, CenterError = std::numeric_limits<double>::infinity();
	for(std::size_t Index = 0; Index < Count; ++Index) {
		MaximumSourceError =
		std::max(MaximumSourceError, std::abs(Sampled->ElevationsMeters[Index] - Source(Sampled->Directions[Index])));
		if((Sampled->Directions[Index].Vector() - Center->Vector()).Length() < 1E-14) {
			CenterError = std::abs(Sampled->ElevationsMeters[Index] - Source(*Center));
		}
		if(Index < Heights.size()) TestEqual(TEXT("元標高を変更しない"), Sampled->ElevationsMeters[Index], Heights[Index]);
	}
	TestTrue(TEXT("全節点で元地形の高さを保持"), MaximumSourceError == 0);
	TestTrue(TEXT("粗い節点間の谷底を実際に保持"), CenterError < 1E-10);
	TestTrue(TEXT("従来の補正が谷を埋める再現条件"), Conditioned->GetStatistics().MaximumConditioningRaiseMeters > 50);
	TestTrue(
		TEXT("排水前の網に開削や盛土を加えない"),
		Sampled->Statistics.MaximumConditioningRaiseMeters == 0 && Sampled->Statistics.MaximumConditioningCutMeters == 0
	);
	Settings.MaximumAddedVertices = 0;
	const std::optional<SphericalReliefSamplingMesh> Unchanged =
	PlanetSphericalRefinement::CreateSamplingMesh(Mesh->Directions, Mesh->Triangles, Heights, Source, Settings);
	TestTrue(
		TEXT("予算０では追加評価を行わない"),
		Unchanged.has_value() && Unchanged->Directions.size() == Heights.size() && Unchanged->Statistics.SourceEvaluations == 0
	);
	std::stop_source Cancellation;
	Cancellation.request_stop();
	TestFalse(
		TEXT("中断要求を受理"),
		PlanetSphericalRefinement::CreateSamplingMesh(
			Mesh->Directions,
			Mesh->Triangles,
			Heights,
			Source,
			Settings,
			Cancellation.get_token()
		).has_value()
	);
	Settings.MaximumAddedVertices = 512;
	const auto InvalidSource = [](const UnitDirection Direction) -> double {
		static_cast<void>(Direction);
		// 不正な評価器を再現する値の返戻
		return std::numeric_limits<double>::quiet_NaN();
	};
	TestFalse(
		TEXT("非有限な追加標高を拒否"),
		PlanetSphericalRefinement::CreateSamplingMesh(Mesh->Directions, Mesh->Triangles, Heights, InvalidSource, Settings).has_value()
	);
	AddInfo(
		FString::Printf(
			TEXT("Spherical predrainage sampling: added=%u sourceError=%.12g centerError=%.12g avoidedRaise=%.12g evaluations=%llu"),
			Sampled->Statistics.AddedVertices,
			MaximumSourceError,
			CenterError,
			Conditioned->GetStatistics().MaximumConditioningRaiseMeters,
			static_cast<unsigned long long>(Sampled->Statistics.SourceEvaluations)
		)
	);
	// 標本の高さと位相を検査した結果の返戻
	return !HasAnyErrors();
}

#endif
