#include "Core/PlanetBasinFill.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBasinBatchContracts, "PlanetGenerator.BasinReconstruction.Batches",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 同じ順序の下限制約を異なる個数で分割し，標高・勾配・海岸・以前の下限の保持を照合する
 * @param Parameters 未使用の UE 検査引数
 * @return 一括適用と全分割が同じ面を作る場合に真
 */
bool FPlanetBasinBatchContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetBasinFill;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(3);
	// 基準の閉じた球面を作れない場合の返戻
	if(!TestTrue(TEXT("分割適用の元網を準備"), Mesh.has_value())) return false;
	std::vector<double> Original, Minimum;
	for(const UnitDirection Point : Mesh->Directions) {
		const double Height = 100 * Point.Vector().Z;
		Original.push_back(Height);
		Minimum.push_back(Height + (Height > 0 ? Height * Height * Height * 0.00001 : 0));
	}
	const std::optional<PlanetBasinFill> Initial = PlanetBasinFill::Create(3, Original, Minimum, 0, 5);
	// 海岸を持つ元の盛土を作れない場合の返戻
	if(!TestTrue(TEXT("分割適用の元の盛土を準備"), Initial.has_value())) return false;
	std::vector<UnitDirection> Locations;
	std::vector<double> Heights, Bounds;
	for(std::uint32_t Index = 0; Index < 257; ++Index) {
		const double Z = 0.015 + 0.97 * (Index + 0.37) / 257, Angle = Index * 2.399963229728653, Radial = std::sqrt(1 - Z * Z);
		const UnitDirection Point = *UnitDirection::FromVector({ Radial * std::cos(Angle), Radial * std::sin(Angle), Z });
		const double Height = 100 * Point.Vector().Z;
		Locations.push_back(Point);
		Heights.push_back(Height);
		Bounds.push_back(Height + (Height < 5 ? Height * 0.1 : 8 + Index % 13));
	}
	const std::optional<PlanetBasinFill> Whole = Initial->WithMinimumElevations(Locations, Heights, Bounds);
	// 一括適用の正規経路を作れない場合の返戻
	if(!TestTrue(TEXT("全下限を一括で適用"), Whole.has_value())) return false;
	for(const std::size_t BatchSize : { std::size_t{ 1 }, std::size_t{ 7 }, std::size_t{ 64 } }) {
		std::optional<PlanetBasinFill> Batched = Initial;
		for(std::size_t Start = 0; Start < Locations.size(); Start += BatchSize) {
			const std::size_t Count = std::min(BatchSize, Locations.size() - Start);
			Batched = Batched->WithMinimumElevations(
				std::span<const UnitDirection>(Locations).subspan(Start, Count),
				std::span<const double>(Heights).subspan(Start, Count),
				std::span<const double>(Bounds).subspan(Start, Count)
			);
			// 正規の順序を保った部分適用が拒否された場合の返戻
			if(!TestTrue(TEXT("順序を保った下限の部分適用"), Batched.has_value())) return false;
			for(std::size_t Previous = 0; Previous < Start + Count; ++Previous) {
				TestTrue(
					TEXT("後の部分適用が以前の全下限を壊さない"),
					Batched->ApplyElevation(Locations[Previous], Heights[Previous]) >= Bounds[Previous] - 2E-6
				);
			}
		}
		for(std::uint32_t Index = 0; Index < 1024; ++Index) {
			const double Z = -1 + 2 * (Index + 0.41) / 1024, Angle = Index * 2.399963229728653, Radial = std::sqrt(1 - Z * Z);
			const UnitDirection Point = *UnitDirection::FromVector({ Radial * std::cos(Angle), Radial * std::sin(Angle), Z });
			const Vec3 P = Point.Vector();
			const TerrainSample Source { 100 * P.Z, (Vec3{ 0, 0, 1 } - P * P.Z) * 100 };
			const TerrainSample A = Whole->Apply(Point, Source), B = Batched->Apply(Point, Source);
			TestEqual(TEXT("下限の分割個数が地点の標高を変えない"), A.ElevationMeters, B.ElevationMeters);
			TestEqual(TEXT("下限の分割個数が解析勾配を変えない"), (A.TangentialGradientPerRadian - B.TangentialGradientPerRadian).Length(), 0.0);
			if(Source.ElevationMeters > 0) continue;
			TestEqual(TEXT("分割適用でも海底は元の儘"), B.ElevationMeters, Source.ElevationMeters);
		}
	}
	AddInfo(
		TEXT("Basin batches: constraints=257 batchSizes=1,7,64 independentSamples=3072; exact heights and gradients required.")
	);
	// 一括・逐次の意味と海岸の保持を照合した返戻
	return !HasAnyErrors();
}

#endif
