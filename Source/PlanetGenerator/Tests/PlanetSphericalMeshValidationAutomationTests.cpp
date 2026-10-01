#include "Core/PlanetCore.h"
#include "Core/PlanetSphericalMesh.h"
#include "Misc/AutomationTest.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 六段の固定網から共有中点の一段細分化を作り，内部上限の検査入力を準備する
	 * 診断用 BuildUniformSphere の六段制限は変更しない
	 * @param Base 正規の基準球面
	 * @return 一段だけ細分化した検査用球面，退化した辺は空
	 */
	std::optional<PlanetCore::SphereMesh> FineValidationFixture(const PlanetCore::SphereMesh &Base) {
		PlanetCore::SphereMesh Result;
		Result.Directions = Base.Directions;
		Result.Directions.reserve(163842);
		Result.Triangles.reserve(Base.Triangles.size() * 4);
		std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> Midpoints;
		const auto Middle = [&Result, &Midpoints](std::uint32_t A, std::uint32_t B) -> std::optional<std::uint32_t> {
			if(A > B) std::swap(A, B);
			const std::pair<std::uint32_t, std::uint32_t> Key { A, B };
			const std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t>::const_iterator Found = Midpoints.find(Key);
			// 共有中点の返戻
			if(Found != Midpoints.end()) return Found->second;
			const std::optional<PlanetCore::UnitDirection> Direction =
			PlanetCore::UnitDirection::FromVector(Result.Directions[A].Vector() + Result.Directions[B].Vector());
			// 退化した検査辺の返戻
			if(!Direction) return {};
			const std::uint32_t Index = static_cast<std::uint32_t>(Result.Directions.size());
			Result.Directions.push_back(*Direction);
			Midpoints.emplace(Key, Index);
			// 新しい検査用中点の返戻
			return Index;
		};
		for(const std::array<std::uint32_t, 3> &Face : Base.Triangles) {
			const std::optional<std::uint32_t> AB = Middle(Face[0], Face[1]), BC = Middle(Face[1], Face[2]), CA = Middle(Face[2], Face[0]);
			// 退化した辺を含む入力の返戻
			if(!AB || !BC || !CA) return {};
			Result.Triangles.push_back({ Face[0], *AB, *CA });
			Result.Triangles.push_back({ *AB, Face[1], *BC });
			Result.Triangles.push_back({ *CA, *BC, Face[2] });
			Result.Triangles.push_back({ *AB, *BC, *CA });
		}
		// 六段診断の上限を変えずに用意した高密度球面の返戻
		return Result;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSphericalMeshValidationContracts,
	"PlanetGenerator.Ocean.MeshValidationBudgets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 外部適応網の上限と内部固定網の正常経路を区別し，向き・閉包の検査を維持する
 * @param Parameters 未使用の UE 引数
 * @return 正常な内部網と既定の外部上限・不正な面の拒否が成立する場合に真
 */
bool FPlanetSphericalMeshValidationContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(6);
	// 正規の基準球面を準備出来ない場合の返戻
	if(!TestTrue(TEXT("通常の固定網を準備"), Base.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Fine = FineValidationFixture(*Base);
	// 有界な一段細分化を準備出来ない場合の返戻
	if(!TestTrue(TEXT("内部高密度の検査網を準備"), Fine.has_value())) return false;
	TestEqual(TEXT("通常網の地点数"), Base->Directions.size(), static_cast<std::size_t>(40962));
	TestEqual(TEXT("内部高密度網の地点数"), Fine->Directions.size(), static_cast<std::size_t>(163842));
	TestTrue(TEXT("通常網は外部の既定上限で受理"), PlanetCore::IsClosedSphericalSamplingMesh(Base->Directions, Base->Triangles));
	TestFalse(TEXT("外部の既定上限を高密度網で越えない"), PlanetCore::IsClosedSphericalSamplingMesh(Fine->Directions, Fine->Triangles));
	TestTrue(TEXT("正規の内部高密度網は指定上限で受理"), PlanetCore::IsClosedSphericalSamplingMesh(Fine->Directions, Fine->Triangles, {}, 167938));
	std::vector<std::array<std::uint32_t, 3>> Reversed = Fine->Triangles;
	std::swap(Reversed[0][0], Reversed[0][1]);
	TestFalse(TEXT("大きな正規上限でも逆向き面を拒否"), PlanetCore::IsClosedSphericalSamplingMesh(Fine->Directions, Reversed, {}, 167938));
	std::vector<std::array<std::uint32_t, 3>> Missing = Fine->Triangles;
	Missing.pop_back();
	TestFalse(TEXT("大きな正規上限でも欠落面を拒否"), PlanetCore::IsClosedSphericalSamplingMesh(Fine->Directions, Missing, {}, 167938));
	TestFalse(TEXT("内部でも無制限の上限を拒否"), PlanetCore::IsClosedSphericalSamplingMesh(Fine->Directions, Fine->Triangles, {}, 167939));
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(
		TEXT("内部高密度網も中断を受理しない"),
		PlanetCore::IsClosedSphericalSamplingMesh(Fine->Directions, Fine->Triangles, Stop.get_token(), 167938)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spherical mesh validation: default vertices=%llu internal fine=%llu hard maximum=167938; ")
			TEXT("reversed faces, missing faces, excessive budgets and cancellation rejected."),
			static_cast<unsigned long long>(Base->Directions.size()),
			static_cast<unsigned long long>(Fine->Directions.size())
		)
	);
	// 正常な二経路と不正入力の拒否結果の返戻
	return !HasAnyErrors();
}

#endif
