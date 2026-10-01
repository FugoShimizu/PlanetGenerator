#include "Core/PlanetSpillPassage.h"
#include "Core/PlanetSpillSampleCache.h"
#include "Misc/AutomationTest.h"
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillSampleCacheContracts,
	"PlanetGenerator.ContinuousSpill.SampleCache",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 再利用の無効化・衝突・ほぼ同じ方向・異なる元場を，独立な標高と勾配で検査する
 * @param Parameters 未使用の実行引数
 * @return 完全一致の方向だけに元の値を返し，容量を守る場合に真
 */
bool FPlanetSpillSampleCacheContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection A = *PlanetCore::UnitDirection::FromVector({ 1E-6, 0, 1 });
	const PlanetCore::UnitDirection B = *PlanetCore::UnitDirection::FromVector({ std::nextafter(1E-6, 1.0), 0, 1 });
	TestTrue(TEXT("丸めれば同一に為る隣接方向を独立に用意"), A.Vector().X != B.Vector().X);
	const PlanetCore::TerrainSample First { std::nextafter(123.0, 124.0), { -0.0, 2.5, -3.75 } };
	const PlanetCore::TerrainSample Second { -19.5, { 1.25, -2.75, 4.5 } };
	// 構造体の詰物を読まず，標高と勾配の全ビットを独立に比較
	const auto Bits = [](const PlanetCore::TerrainSample &Value) -> std::array<std::uint64_t, 4> {
		// 標高と全勾配のビット列の返戻
		return {
			std::bit_cast<std::uint64_t>(Value.ElevationMeters),
			std::bit_cast<std::uint64_t>(Value.TangentialGradientPerRadian.X),
			std::bit_cast<std::uint64_t>(Value.TangentialGradientPerRadian.Y),
			std::bit_cast<std::uint64_t>(Value.TangentialGradientPerRadian.Z)
		};
	};
	for(const std::uint32_t Capacity : { 0U, 1U, 17U, PlanetCore::SpillSampleCache::MaximumCapacity }) {
		std::optional<PlanetCore::SpillSampleCache> Cache = PlanetCore::SpillSampleCache::Create(Capacity);
		// 正規容量を準備出来ない場合の返戻
		if(!TestTrue(TEXT("有界な保持容量"), Cache.has_value())) return false;
		TestTrue(TEXT("実保持領域は６４MiB以内"), Cache->GetMemoryBytes() <= PlanetCore::SpillSampleCache::MaximumByteCapacity);
		AddInfo(
			FString::Printf(
				TEXT("Exact sample capacity=%u allocatedBytes=%llu"),
				Capacity,
				static_cast<unsigned long long>(Cache->GetMemoryBytes())
			)
		);
		TestFalse(TEXT("未評価の方向は未保持"), Cache->Find(A).has_value());
		Cache->Store(A, First);
		const std::optional<PlanetCore::TerrainSample> Stored = Cache->Find(A);
		TestEqual(TEXT("無効化では何も保存しない"), Stored.has_value(), Capacity != 0);
		if(Stored) TestTrue(TEXT("標高と全勾配のビット一致"), Bits(*Stored) == Bits(First));
		TestFalse(TEXT("ほぼ同じ方向へ値を流用しない"), Cache->Find(B).has_value());
		Cache->Store(B, Second);
		const std::optional<PlanetCore::TerrainSample> Other = Cache->Find(B);
		if(Other) TestTrue(TEXT("衝突した枠でも異なる値を誤用しない"), Bits(*Other) == Bits(Second));
		if(Capacity == 1) TestFalse(TEXT("衝突後の旧方向は再評価が必要"), Cache->Find(A).has_value());
		const double Invalid = std::numeric_limits<double>::quiet_NaN();
		Cache->Store(B, { Invalid, { 1, 2, 3 } });
		Cache->Store(B, { 3, { Invalid, 2, 3 } });
		const std::optional<PlanetCore::TerrainSample> AfterInvalid = Cache->Find(B);
		TestEqual(TEXT("無効な標本は保持済の値を壊さない"), Other.has_value(), AfterInvalid.has_value());
		if(AfterInvalid) TestTrue(TEXT("非有限な標高も勾配も保持しない"), Bits(*AfterInvalid) == Bits(Second));
		TestTrue(TEXT("衝突を含む保持量は設定以内"), Cache->GetCount() <= Capacity);
		std::optional<PlanetCore::SpillSampleCache> NewSource = PlanetCore::SpillSampleCache::Create(Capacity);
		TestFalse(TEXT("別の生成定義へ旧標本を持越さない"), NewSource->Find(A).has_value());
	}
	TestFalse(
		TEXT("上限超過の枠は確保前に拒否"),
		PlanetCore::SpillSampleCache::Create(PlanetCore::SpillSampleCache::MaximumCapacity + 1).has_value()
	);
	// 有界な再利用の検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillSampleCachePassages,
	"PlanetGenerator.ContinuousSpill.CachedPassages",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 重なる探索窓を同じ解析場で測り，再利用前後の実経路と候補が変わらない事を検査する
 * @param Parameters 未使用の実行引数
 * @return 全成分のビット一致と元場の実呼出削減を確認した場合に真
 */
bool FPlanetSpillSampleCachePassages::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	std::vector<std::uint64_t> Expected;
	std::uint64_t OriginalCalls = 0;
	for(const std::uint32_t Capacity : { 0U, PlanetCore::SpillSampleCache::MaximumCapacity }) {
		std::optional<PlanetCore::SpillSampleCache> Cache = PlanetCore::SpillSampleCache::Create(Capacity);
		std::uint64_t Calls = 0, Hits = 0;
		// 同じ不変な解析場を全窓で共有し，完全一致の方向だけを再利用
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			if(const std::optional<PlanetCore::TerrainSample> Stored = Cache->Find(Direction)) {
				++Hits;
				// 同じ解析場の実測済標本の返戻
				return *Stored;
			}
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = 1E6 * D.X / D.Z, Y = 1E6 * D.Y / D.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			const PlanetCore::TerrainSample Value {
				4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400,
				{ 1E6 * DX / D.Z, 1E6 * DY / D.Z, -1E6 * (DX * D.X + DY * D.Y) / (D.Z * D.Z) }
			};
			Cache->Store(Direction, Value);
			// 同じ解析場の全成分を変更しない返戻
			return Value;
		};
		const PlanetCore::UnitDirection From = *PlanetCore::UnitDirection::FromVector({ -90E-6, 0, 1 });
		const PlanetCore::UnitDirection To = *PlanetCore::UnitDirection::FromVector({ 90E-6, 0, 1 });
		std::vector<std::uint64_t> Values;
		// 診断回数を除き，実際の数値成分を詰物無で比較
		const auto Number = [&](const double Value) -> void {
			Values.push_back(std::bit_cast<std::uint64_t>(Value));
			// 終了
			return;
		};
		const auto Direction = [&](const PlanetCore::UnitDirection D) -> void {
			Number(D.Vector().X);
			Number(D.Vector().Y);
			Number(D.Vector().Z);
			// 終了
			return;
		};
		for(const double Width : { 256.0, 512.0 }) {
			PlanetCore::SpillPassageSettings Settings;
			Settings.HalfWidthMeters = Width;
			Settings.SpacingMeters = 16;
			Settings.MaximumSampleStepMeters = 4;
			const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(From, To, Source, Settings);
			// 同じ窓の実測通路を作れない場合の返戻
			if(!TestTrue(TEXT("重なる窓の実測通路"), Passage.has_value())) return false;
			Direction(Passage->Peak);
			for(
				const double Value :
				{
					Passage->CrestMeters,
					Passage->PathLengthMeters,
					Passage->MaximumIndependentExcessMeters,
					Passage->BoundaryEscapeMeters[0],
					Passage->BoundaryEscapeMeters[1],
					Passage->OutsideLowerBoundMeters
				}
			) Number(Value);
			for(
				const std::uint64_t Value :
				{
					std::uint64_t{ Passage->IsGraphInteriorMinimum },
					std::uint64_t{ Passage->TouchesBoundary },
					std::uint64_t{ Passage->Nodes },
					std::uint64_t{ Passage->PathNodes },
					std::uint64_t{ Passage->EvaluatedEdges },
					std::uint64_t{ Passage->TotalEdges },
					static_cast<std::uint64_t>(Passage->Path.size()),
					static_cast<std::uint64_t>(Passage->PassCandidates.size())
				}
			) Values.push_back(Value);
			for(const PlanetCore::UnitDirection D : Passage->Path) Direction(D);
			for(const PlanetCore::UnitDirection D : Passage->PassCandidates) Direction(D);
		}
		if(!Capacity) {
			Expected = Values;
			OriginalCalls = Calls;
		} else {
			TestTrue(TEXT("全経路・最高点・中間候補・窓の条件がビット一致"), Values == Expected);
			TestTrue(TEXT("実際の元場呼出を削減"), Hits > 0 && Calls < OriginalCalls);
		}
		AddInfo(
			FString::Printf(
				TEXT("Cached passages capacity=%u calls=%llu baseline=%llu hits=%llu retained=%u components=%llu"),
				Capacity,
				Calls,
				OriginalCalls,
				Hits,
				Cache->GetCount(),
				static_cast<unsigned long long>(Values.size())
			)
		);
	}
	// 再利用前後の正規通路の検査結果の返戻
	return !HasAnyErrors();
}

#endif
