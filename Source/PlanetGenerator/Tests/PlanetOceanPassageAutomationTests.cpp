#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetOceanPassageContracts,
	"PlanetGenerator.Ocean.PassageConnectivity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

namespace {
	/**
	 * 両端だけの水没を通路の接続とせず，途中の元標高でも全海域の連結を検査する
	 * @param Test 検査結果の受取先
	 * @param Seed 惑星の乱数種
	 * @param Radius 物理半径
	 * @param Adaptive 別の標本方向で追加接続する場合に真
	 * @return 別の通路標本に依る水没域が一成分の場合に真
	 */
	bool VerifyOceanPassages(FAutomationTestBase &Test, const std::uint64_t Seed, const double Radius, const bool Adaptive) {
		// 同じ定義からの地質と海域の準備
		PlanetCore::GeologySettings Definition;
		Definition.RadiusMeters = Radius;
		Definition.Seed = Seed;
		std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
		// 元地形を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("実地形の地質を準備"), Base.has_value())) return false;
		const std::shared_ptr<const PlanetCore::GeologyField> Geology =
		std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
		std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
		// 造山を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("実地形の造山を準備"), Ridges.has_value())) return false;
		std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
			Geology,
			std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
			{},
			PlanetCore::PredrainageReliefMode::MesoAndLand,
			PlanetCore::OceanSamplingMode::SharedDrainage
		);
		// 海域を含む元場を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("変更しない元場を準備"), Ocean.has_value())) return false;
		const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
		// 独立な検査網を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("海域検査の全球網を準備"), Mesh.has_value())) return false;
		std::vector<PlanetCore::UnitDirection> Directions = Mesh->Directions;
		// 既存の海峡を保持した追加網の正規経路
		if(Adaptive) {
			constexpr double Angle = 0.002;
			const double Cosine = std::cos(Angle), Sine = std::sin(Angle);
			for(PlanetCore::UnitDirection &D : Directions) {
				const PlanetCore::Vec3 V = D.Vector();
				D = *PlanetCore::UnitDirection::FromVector({ V.X * Cosine - V.Y * Sine, V.X * Sine + V.Y * Cosine, V.Z });
			}
			Ocean = Ocean->WithSamplingMesh(Directions, Mesh->Triangles);
			// 別網で接続出来ない元場を成功としない返戻
			if(!Test.TestTrue(TEXT("回転した追加網の海域を接続"), Ocean.has_value())) return false;
		}
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			// 同じ元場の返戻
			return Ocean->Evaluate(Direction);
		};
		// 独立な評価点と重複を除いた共有辺の列挙
		const std::size_t Count = Directions.size();
		std::vector<double> Heights;
		std::vector<std::array<std::uint32_t, 2>> Edges;
		for(const PlanetCore::UnitDirection D : Directions) {
			const double Height = Source(D).ElevationMeters, HeightOnly = Ocean->EvaluateElevation(D);
			// 同じ元場の微分省略で標高が変わる場合の拒否
			if(std::bit_cast<std::uint64_t>(Height) != std::bit_cast<std::uint64_t>(HeightOnly)) {
				Test.AddError(
					FString::Printf(
						TEXT("Height-only mismatch seed=%llu radius=%.0f full=%.17g height=%.17g"),
						static_cast<unsigned long long>(Seed),
						Radius,
						Height,
						HeightOnly
					)
				);
				// 標高の不一致を後続の接続検査へ混ぜない返戻
				return false;
			}
			Heights.push_back(Height);
		}
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(unsigned i = 0; i < 3; ++i) {
			const std::uint32_t A = Triangle[i], B = Triangle[(i + 1) % 3];
			Edges.push_back({ std::min(A, B), std::max(A, B) });
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		// 生成側より細かい辺内標本に依る通過判定
		std::vector<std::vector<std::uint32_t>> Neighbors(Count);
		std::uint32_t BlockedWetEdges = 0;
		std::uint64_t Probes = 0;
		for(const std::array<std::uint32_t, 2> &Pair : Edges) {
			if(Heights[Pair[0]] >= Definition.SeaLevelMeters || Heights[Pair[1]] >= Definition.SeaLevelMeters) continue;
			bool IsWet = true;
			const PlanetCore::Vec3 A = Directions[Pair[0]].Vector(), B = Directions[Pair[1]].Vector();
			for(unsigned i = 1; i < 16 && IsWet; ++i) {
				const double T = i / 16.0;
				const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(A * (1 - T) + B * T);
				IsWet = IsWet && Source(D).ElevationMeters < Definition.SeaLevelMeters;
				++Probes;
			}
			if(IsWet) {
				Neighbors[Pair[0]].push_back(Pair[1]);
				Neighbors[Pair[1]].push_back(Pair[0]);
			} else ++BlockedWetEdges;
		}
		// 独立な幅優先探索に依る接続成分の照合
		std::vector<bool> Seen(Count, false);
		std::vector<std::uint32_t> Pending;
		std::uint32_t Components = 0, Largest = 0, Wet = 0;
		for(std::uint32_t i = 0; i < Count; ++i) {
			if(!(Heights[i] < Definition.SeaLevelMeters)) continue;
			++Wet;
			if(Seen[i]) continue;
			++Components;
			Seen[i] = true;
			Pending.clear();
			Pending.push_back(i);
			for(std::size_t j = 0; j < Pending.size(); ++j) for(const std::uint32_t Adjacent : Neighbors[Pending[j]]) {
				if(Seen[Adjacent]) continue;
				Seen[Adjacent] = true;
				Pending.push_back(Adjacent);
			}
			Largest = std::max(Largest, static_cast<std::uint32_t>(Pending.size()));
		}
		Test.AddInfo(
			FString::Printf(
				TEXT(
					"Ocean passages seed=%llu radius=%.0f reported=%u components=%u wet=%u largest=%u blockedWetEdges=%u probes=%llu channels=%u"
				),
				static_cast<unsigned long long>(Definition.Seed),
				Definition.RadiusMeters,
				Ocean->Statistics().ComponentsAfter,
				Components,
				Wet,
				Largest,
				BlockedWetEdges,
				static_cast<unsigned long long>(Probes),
				Ocean->Statistics().CorridorCount
			)
		);
		Test.AddInfo(
			FString::Printf(
				TEXT("Ocean passage preparation before=%u after=%u evaluations=%llu area=%.12g version=%u adaptive=%u"),
				Ocean->Statistics().BlockedWetEdgesBefore,
				Ocean->Statistics().BlockedWetEdgesAfter,
				static_cast<unsigned long long>(Ocean->Statistics().PassageEvaluations),
				Ocean->Statistics().GraphOceanAreaFraction,
				PlanetCore::OceanConnectivityVersion,
				static_cast<unsigned>(Adaptive)
			)
		);
		Test.TestTrue(
			TEXT("海域の面積が要求範囲内"),
			Ocean->Statistics().IndependentOceanFractionEstimate >= 0.60 && Ocean->Statistics().IndependentOceanFractionEstimate <= 0.70
		);
		Test.TestEqual(TEXT("途中の地形を含めた全水没域の接続"), Components, 1U);
		Test.TestEqual(TEXT("準備統計と独立な連結検査が一致"), Ocean->Statistics().ComponentsAfter, Components);
		const std::span<const PlanetCore::UnitDirection> Anchors = Ocean->ConnectedOceanAnchors();
		Test.TestTrue(TEXT("元の水没節点と実測した内部水没点を保持"), Anchors.size() >= static_cast<std::size_t>(Wet));
		Test.TestEqual(TEXT("元の網の地点数を区別して保持"), Ocean->Statistics().InputGraphNodes, static_cast<std::uint32_t>(Count));
		std::size_t AnchorIndex = 0;
		for(std::size_t I = 0; I < Count; ++I) {
			if(!(Heights[I] < Definition.SeaLevelMeters)) continue;
			// 欠落した証拠を参照しない返戻
			if(!Test.TestTrue(TEXT("大洋標本の保持範囲内"), AnchorIndex < Anchors.size())) return false;
			const PlanetCore::Vec3 Expected = Directions[I].Vector(), Actual = Anchors[AnchorIndex++].Vector();
			Test.TestTrue(TEXT("適応網でも検査に使った実方向を保持"), Expected.X == Actual.X && Expected.Y == Actual.Y && Expected.Z == Actual.Z);
		}
		for(; AnchorIndex < Anchors.size(); ++AnchorIndex) {
			Test.TestTrue(TEXT("追加した大洋標本も同じ最終場で水没"), Source(Anchors[AnchorIndex]).ElevationMeters < Definition.SeaLevelMeters);
		}
		// 有限標本の大洋接続検査結果の返戻
		return !Test.HasAnyErrors();
	}
}

/**
 * 物理半径・乱数種・追加網の各正規経路の海域接続を照合する
 * @param Parameters 未使用の実行引数
 * @return 独立な辺内標本で全条件の海域が一成分の場合に真
 */
bool FPlanetOceanPassageContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 最小半径から大半径迄と，異なる地殻配置の検査
	const std::array<std::pair<std::uint64_t, double>, 5> Cases {
		{ { 42, 1E6 }, { 42, 1000 }, { 42, 1E9 }, { 0, 6371000 }, { 90210, 6371000 } }
	};
	for(const std::pair<std::uint64_t, double> &Case : Cases) {
		AddInfo(FString::Printf(TEXT("Passage case seed=%llu radius=%.0f"), static_cast<unsigned long long>(Case.first), Case.second));
		// 準備失敗を隠さず停止する返戻
		if(!VerifyOceanPassages(*this, Case.first, Case.second, false)) return false;
	}
	// 同じ連続場へ新しい標本網を追加する公開経路の検査結果の返戻
	return VerifyOceanPassages(*this, 42, 1E6, true);
}

#endif
