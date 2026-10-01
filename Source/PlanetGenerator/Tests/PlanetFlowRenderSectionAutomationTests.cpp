#include "Core/PlanetFlowView.h"
#include "Core/PlanetFlowWater.h"
#include "Core/PlanetSurfaceMaterial.h"
#include "PlanetFlowRenderSection.h"
#include "Async/Async.h"
#include "Misc/AutomationTest.h"
#include "ProceduralMeshComponent.h"
#include "UObject/UObjectGlobals.h"
#include <cstddef>
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 標準部品が作った全属性と，ワーカーで組立てた区画を照合する
	 * @param Test 結果を集約する検査
	 * @param Actual ワーカーの完成区画
	 * @param Expected 従来の標準 API が構成した区画
	 */
	void CompareSection(FAutomationTestBase &Test, const FProcMeshSection &Actual, const FProcMeshSection &Expected) {
		Test.TestEqual(TEXT("頂点数を保持"), Actual.ProcVertexBuffer.Num(), Expected.ProcVertexBuffer.Num());
		Test.TestTrue(TEXT("接続と表裏を保持"), Actual.ProcIndexBuffer == Expected.ProcIndexBuffer);
		Test.TestTrue(TEXT("包囲箱の下端を保持"), Actual.SectionLocalBox.Min == Expected.SectionLocalBox.Min);
		Test.TestTrue(TEXT("包囲箱の上端を保持"), Actual.SectionLocalBox.Max == Expected.SectionLocalBox.Max);
		Test.TestEqual(TEXT("包囲箱の有効状態を保持"), Actual.SectionLocalBox.IsValid, Expected.SectionLocalBox.IsValid);
		Test.TestEqual(TEXT("表示状態を保持"), Actual.bSectionVisible, Expected.bSectionVisible);
		Test.TestEqual(TEXT("衝突を描画から分離"), Actual.bEnableCollision, Expected.bEnableCollision);
		// 数の不一致を範囲外参照にしない終了
		if(Actual.ProcVertexBuffer.Num() != Expected.ProcVertexBuffer.Num()) return;
		for(int32 I = 0; I < Actual.ProcVertexBuffer.Num(); ++I) {
			const FProcMeshVertex &A = Actual.ProcVertexBuffer[I], &B = Expected.ProcVertexBuffer[I];
			Test.TestTrue(TEXT("位置の全成分を厳密に保持"), A.Position == B.Position);
			Test.TestTrue(TEXT("解析法線の全成分を厳密に保持"), A.Normal == B.Normal);
			Test.TestTrue(TEXT("接線の全成分を厳密に保持"), A.Tangent.TangentX == B.Tangent.TangentX);
			Test.TestEqual(TEXT("接線の向きを保持"), A.Tangent.bFlipTangentY, B.Tangent.bFlipTangentY);
			Test.TestEqual(TEXT("材質の量子化色を厳密に保持"), A.Color, B.Color);
			Test.TestTrue(TEXT("水深・所属又は未使用の模様座標を保持"), A.UV0 == B.UV0 && A.UV1 == B.UV1);
			Test.TestTrue(TEXT("惑星固定の模様座標を保持"), A.UV2 == B.UV2 && A.UV3 == B.UV3);
		}
		// 終了
		return;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowRenderSectionContracts, "PlanetGenerator.FlowChannel.RenderSection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 標準 API との属性互換，空の水面，中断・不正属性の非公開を確認する
 * @param Parameters 未使用の実行引数
 * @return 全ての検査経路を実行した場合に真
 */
bool FPlanetFlowRenderSectionContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 変換前の位置・法線・色を独立に渡す，非平面の小さな部品
	const TArray<FVector> Positions { { -123, 234, 345 }, { 456, -567, 678 }, { 789, 890, -901 }, { -345, -678, 123 } };
	const TArray<FVector> Normals {
		FVector(1, 2, 3).GetSafeNormal(),
		FVector(2, 3, 4).GetSafeNormal(),
		FVector(-2, 1, 3).GetSafeNormal(),
		FVector(3, -4, 5).GetSafeNormal()
	};
	const TArray<FLinearColor> Colors {
		{ 0.15F, 0.45F, 0.25F, 0.7F },
		{ 0, 1, 0, 1 },
		{ 0.9999F, 0.0001F, 0.5F, 0.499F },
		{ 0.123F, 0.456F, 0.789F, 0.987F }
	};
	const TArray<int32> Indices { 0, 2, 1, 0, 3, 2 };
	UProceduralMeshComponent *const Reference = NewObject<UProceduralMeshComponent>();
	// 標準部品が無い場合の返戻
	if(!TestNotNull(TEXT("比較用の標準描画部品"), Reference)) return false;
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		PlanetCore::FlowTerrainMesh Ground;
		Ground.Frame = *PlanetCore::GroundFrame::Create(*PlanetCore::UnitDirection::FromVector({ 1, 2, 3 }), Radius, 12.5);
		Ground.Sphere.Mesh.Triangles = { { 0, 1, 2 }, { 0, 2, 3 } };
		TArray<FVector> ExactPositions;
		TArray<FVector2D> UV0, UV1, UV2, UV3;
		TArray<FProcMeshTangent> Tangents;
		for(int32 I = 0; I < Positions.Num(); ++I) {
			const FVector P = Positions[I] * 0.01, N = Normals[I];
			Ground.Positions.push_back({ P.X, P.Y, P.Z });
			Ground.Normals.push_back({ N.X, N.Y, N.Z });
			Ground.PlanetNormals.push_back({ N.X, N.Y, N.Z });
			Ground.Materials.push_back({ Colors[I].R, Colors[I].G, Colors[I].B, Colors[I].A });
			Ground.Heights.push_back(12.5 + I);
			Ground.WaterCoordinates.push_back({ 0.125 * (I - 1), 0.25 * (I - 2) });
			UV0.Emplace(12.5 + I, 0.125 * (I - 1));
			UV1.Emplace(0.25 * (I - 2), 0);
			// センチメートルへの既存の乗算順を比較基準へ保持
			ExactPositions.Add(P * 100);
			UV2.Emplace(N.X, N.Y);
			UV3.Emplace(N.Z, 0);
			const PlanetCore::Vec3 T =
			Ground.Frame.VectorToLocal(PlanetCore::SurfaceMaterialTangent(*PlanetCore::UnitDirection::FromVector({ N.X, N.Y, N.Z })));
			Tangents.Emplace(FVector(T.X, T.Y, T.Z), false);
		}
		const std::optional<FProcMeshSection> Prepared = Async(
			EAsyncExecution::ThreadPool,
			[&Ground]() -> std::optional<FProcMeshSection> {
				// UObject を使わずに別スレッドで組立てた返戻
				return PreparePlanetFlowSection(Ground);
			}
		).Get();
		Reference->CreateMeshSection_LinearColor(0, ExactPositions, Indices, Normals, UV0, UV1, UV2, UV3, Colors, Tangents, false);
		if(TestTrue(TEXT("ワーカーで地面の全配列を準備"), Prepared.has_value())) CompareSection(*this, *Prepared, *Reference->GetProcMeshSection(0));
		// 同じ形状で水面の既定色と未使用属性も，標準経路と照合
		PlanetCore::FlowWaterMesh Water;
		Water.Frame = Ground.Frame;
		Water.Normals = Ground.Normals;
		Water.PlanetNormals = Ground.PlanetNormals;
		Water.Surface.Triangles = Ground.Sphere.Mesh.Triangles;
		for(const PlanetCore::Vec3 P : Ground.Positions) Water.Surface.Vertices.push_back({ P });
		const std::optional<FProcMeshSection> Wet = PreparePlanetFlowSection(Water);
		Reference->CreateMeshSection_LinearColor(0, ExactPositions, Indices, Normals, {}, {}, UV2, UV3, {}, Tangents, false);
		if(TestTrue(TEXT("水面の完全な配列を準備"), Wet.has_value())) CompareSection(*this, *Wet, *Reference->GetProcMeshSection(0));
		std::stop_source Cancel;
		Cancel.request_stop();
		TestFalse(TEXT("中断した地面を公開しない"), PreparePlanetFlowSection(Ground, Cancel.get_token()).has_value());
		TestFalse(TEXT("中断した水面を公開しない"), PreparePlanetFlowSection(Water, Cancel.get_token()).has_value());
		const double SavedDepth = Ground.WaterCoordinates[0][0];
		Ground.WaterCoordinates[0][0] = std::numeric_limits<double>::quiet_NaN();
		TestFalse(TEXT("非有限な水深を飽和で隠さない"), PreparePlanetFlowSection(Ground).has_value());
		Ground.WaterCoordinates[0][0] = SavedDepth;
		Ground.Heights[0] = 1E9;
		Ground.WaterCoordinates[0] = { -1E9, 1E9 };
		const std::optional<FProcMeshSection> Bounded = PreparePlanetFlowSection(Ground);
		TestTrue(
			TEXT("搬送する属性だけが半精度の有限範囲"),
			Bounded && Bounded->ProcVertexBuffer[0].UV0 == FVector2D(32768, -32768) &&
			Bounded->ProcVertexBuffer[0].UV1 == FVector2D(32768, 0) && Bounded->ProcVertexBuffer[0].Position == ExactPositions[0]
		);
		Ground.WaterCoordinates.pop_back();
		TestFalse(TEXT("水域の属性欠落を乾燥地面へ置換しない"), PreparePlanetFlowSection(Ground).has_value());
		Ground.WaterCoordinates.push_back({ 0, 0 });
		Ground.Normals.pop_back();
		TestFalse(TEXT("欠落した法線を既定値へ置換しない"), PreparePlanetFlowSection(Ground).has_value());
		Water.Surface.Triangles[0][0] = 4;
		TestFalse(TEXT("範囲外の接続を丸めない"), PreparePlanetFlowSection(Water).has_value());
		Water.Surface.Triangles[0] = { 0, 0, 2 };
		TestFalse(TEXT("重複した接続を欠落面へ変えない"), PreparePlanetFlowSection(Water).has_value());
		Water.Surface.Triangles[0] = { 0, 1, 2 };
		Water.Surface.Vertices[0].Position.X = std::numeric_limits<double>::quiet_NaN();
		TestFalse(TEXT("非有限な位置を公開しない"), PreparePlanetFlowSection(Water).has_value());
		Water.Surface.Vertices[0].Position = Ground.Positions[0];
		Water.PlanetNormals[0] = {};
		TestFalse(TEXT("接線を構成出来ない方向を拒否"), PreparePlanetFlowSection(Water).has_value());
	}
	const PlanetCore::FlowWaterMesh Empty;
	const std::optional<FProcMeshSection> EmptySection = PreparePlanetFlowSection(Empty);
	TestTrue(TEXT("陸だけの表示には空の水面を許可"), EmptySection.has_value() && EmptySection->ProcVertexBuffer.IsEmpty());
	Reference->DestroyComponent();
	AddInfo(
		TEXT(
			"RenderSection radii=3 vertices=24 exactAttributes=position/normal/tangent/color/uv0/uv1/uv2/uv3/bounds/index emptyWater=true"
		)
	);
	// 実部品との比較と不正入力経路を実行した返戻
	return true;
}

#endif
