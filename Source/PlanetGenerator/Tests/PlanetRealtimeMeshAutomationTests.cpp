#include "PlanetRealtimeMeshSection.h"
#include "Async/Async.h"
#include "DynamicMeshBuilder.h"
#include "Misc/AutomationTest.h"
#include "ProceduralMeshComponent.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "Rendering/StaticMeshVertexBuffer.h"
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * RMC の保持配列を，標準描画の GPU 頂点変換と比較する
	 * @param Test 検査結果の集約先
	 * @param Streams 実際に保持された配列
	 * @param Section 変換前の完全な描画区画
	 */
	void CompareStreams(
		FAutomationTestBase &Test,
		const RealtimeMesh::FRealtimeMeshStreamSet &Streams,
		const FProcMeshSection &Section
	) {
		const RealtimeMesh::FRealtimeMeshStream *const Positions = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::Position);
		const RealtimeMesh::FRealtimeMeshStream *const Tangents = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::Tangents);
		const RealtimeMesh::FRealtimeMeshStream *const Colors = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::Color);
		const RealtimeMesh::FRealtimeMeshStream *const UVs = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::TexCoords);
		const RealtimeMesh::FRealtimeMeshStream *const Triangles = Streams.Find(RealtimeMesh::FRealtimeMeshStreams::Triangles);
		// 欠落を範囲外参照にしない終了
		if(!Test.TestTrue(TEXT("全属性の配列が存在"), Positions && Tangents && Colors && UVs && Triangles)) return;
		const int32 Count = Section.ProcVertexBuffer.Num();
		if(
			!Test.TestTrue(
				TEXT("全配列の頂点数が一致"),
				Positions->Num() == Count && Tangents->Num() == Count && Colors->Num() == Count && UVs->Num() == Count
			)
		) return;
		FStaticMeshVertexBuffer ExpectedAttributes;
		ExpectedAttributes.Init(Count, 4);
		for(int32 I = 0; I < Count; ++I) {
			const FProcMeshVertex &V = Section.ProcVertexBuffer[I];
			// UE の ProceduralMeshComponent.cpp と同じ変換順序
			FDynamicMeshVertex Expected;
			Expected.Position = FVector3f(V.Position);
			Expected.TangentX = V.Tangent.TangentX;
			Expected.TangentZ = V.Normal;
			Expected.TangentZ.Vector.W = V.Tangent.bFlipTangentY ? -127 : 127;
			ExpectedAttributes.SetVertexTangents(
				I,
				Expected.TangentX.ToFVector3f(),
				Expected.GetTangentY(),
				Expected.TangentZ.ToFVector3f()
			);
			const FVector2f Coordinates[] { FVector2f(V.UV0), FVector2f(V.UV1), FVector2f(V.UV2), FVector2f(V.UV3) };
			for(uint32 UV = 0; UV < 4; ++UV) ExpectedAttributes.SetVertexUV(I, UV, Coordinates[UV]);
			Test.TestTrue(
				TEXT("位置の単精度変換が一致"),
				!FMemory::Memcmp(Positions->GetData() + I * Positions->GetStride(), &Expected.Position, sizeof(Expected.Position))
			);
			Test.TestTrue(
				TEXT("接線・法線・表裏の符号がビット一致"),
				Tangents->GetStride() == 2 * sizeof(FPackedNormal) && !FMemory::Memcmp(
					Tangents->GetData() + I * Tangents->GetStride(),
					static_cast<const uint8 *>(ExpectedAttributes.GetTangentData()) + I * 2 * sizeof(FPackedNormal),
					2 * sizeof(FPackedNormal)
				)
			);
			Test.TestTrue(
				TEXT("四組の模様座標がビット一致"),
				UVs->GetStride() == 4 * sizeof(FVector2DHalf) && !FMemory::Memcmp(
					UVs->GetData() + I * UVs->GetStride(),
					static_cast<const uint8 *>(ExpectedAttributes.GetTexCoordData()) + I * 4 * sizeof(FVector2DHalf),
					4 * sizeof(FVector2DHalf)
				)
			);
			Test.TestTrue(TEXT("量子化済の材質色が一致"), !FMemory::Memcmp(Colors->GetData() + I * Colors->GetStride(), &V.Color, sizeof(V.Color)));
		}
		Test.TestTrue(
			TEXT("三角形の接続と表裏を保持"),
			Triangles->Num() * Triangles->GetStride() == Section.ProcIndexBuffer.Num() * sizeof(uint32) &&
			!FMemory::Memcmp(Triangles->GetData(), Section.ProcIndexBuffer.GetData(), Section.ProcIndexBuffer.Num() * sizeof(uint32))
		);
		// 全属性比較の終了
		return;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetRealtimeSectionContracts, "PlanetGenerator.RealtimeMesh.Section",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * ワーカーでの変換・実部品への所有権移動・不正入力の拒否を検査する
 * @param Parameters 未使用の実行引数
 * @return 全検査経路を実行した場合に真
 */
bool FPlanetRealtimeSectionContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	FProcMeshSection Section;
	for(int32 I = 0; I < 257; ++I) {
		FProcMeshVertex V;
		V.Position = FVector(I * 123.456789, I * -234.567891, I * I * 0.000123456);
		V.Normal = FVector(I + 1, -I - 2, I + 3).GetSafeNormal();
		V.Tangent = FProcMeshTangent(FVector(-I - 2, -I - 1, 0).GetSafeNormal(), I % 2 != 0);
		// 量子化後の退化基底でも標準の符号再計算と一致する事を検査
		if(I == 255) V.Normal = FVector::ZeroVector;
		if(I == 256) V.Tangent = FProcMeshTangent(V.Normal, true);
		V.Color = FColor(I, 255 - I, I / 2, I % 255);
		V.UV0 = FVector2D(I * 0.1, -I * 0.2);
		V.UV1 = FVector2D(I * 0.3, -I * 0.4);
		V.UV2 = FVector2D(V.Normal.X, V.Normal.Y);
		V.UV3 = FVector2D(V.Normal.Z, I * 0.5);
		Section.ProcVertexBuffer.Add(V);
		Section.SectionLocalBox += V.Position;
		if(I > 1) Section.ProcIndexBuffer.Append({ 0, static_cast<uint32>(I), static_cast<uint32>(I - 1) });
	}
	std::optional<RealtimeMesh::FRealtimeMeshStreamSet> Streams = Async(
		EAsyncExecution::ThreadPool,
		[&Section]() -> std::optional<RealtimeMesh::FRealtimeMeshStreamSet> {
			// UObject に触れないワーカーからの返戻
			return PreparePlanetRealtimeSection(Section);
		}
	).Get();
	// 変換失敗を未定義の参照にしない返戻
	if(!TestTrue(TEXT("ワーカーでの変換成功"), Streams.has_value())) return false;
	CompareStreams(*this, *Streams, Section);
	URealtimeMeshComponent *const Component = NewObject<URealtimeMeshComponent>();
	URealtimeMeshSimple *const Mesh = Component->InitializeRealtimeMesh<URealtimeMeshSimple>();
	const FRealtimeMeshBufferSetKey Key = FRealtimeMeshBufferSetKey::Create(0, FName("Comparison"));
	Mesh->SetupMaterialSlot(0, FName("Ground"));
	Mesh->CreateBufferSet(Key, MoveTemp(*Streams));
	int32 Visits = 0;
	Mesh->ProcessMesh(
		Key,
		[this, &Section, &Visits](const RealtimeMesh::FRealtimeMeshStreamSet &Stored) -> void {
			++Visits;
			CompareStreams(*this, Stored, Section);
			// 保持配列の比較終了
			return;
		}
	);
	TestEqual(TEXT("実部品が保持する配列を読み取り"), Visits, 1);
	// 静的・動的の双方で，範囲外の属性保持と要素数を変える全体更新を検査
	for(
		const ERealtimeMeshSectionDrawType DrawType :
		{ ERealtimeMeshSectionDrawType::Static, ERealtimeMeshSectionDrawType::Dynamic }
	) {
		FProcMeshSection Updated = Section;
		const FRealtimeMeshBufferSetKey UpdateKey = FRealtimeMeshBufferSetKey::Create(
			0,
			DrawType == ERealtimeMeshSectionDrawType::Static ? FName("StaticUpdates") : FName("DynamicUpdates")
		);
		std::optional<RealtimeMesh::FRealtimeMeshStreamSet> Initial = PreparePlanetRealtimeSection(Updated);
		// 有効な元データから初期配列を得られなかった返戻
		if(!TestTrue(TEXT("更新検査の初期配列"), Initial.has_value())) return false;
		Mesh->CreateBufferSet(UpdateKey, MoveTemp(*Initial), FRealtimeMeshBufferSetConfig(DrawType));
		for(const int32 Begin : { 0, 125, 254 }) {
			for(int32 I = Begin; I < Begin + 3; ++I) {
				Updated.ProcVertexBuffer[I].Color = FColor(17, 33, 201, 87);
				Updated.ProcVertexBuffer[I].Position.Z += 2.5;
			}
			Mesh->EditMeshInPlaceRanged(
				UpdateKey,
				[&Updated, Begin](RealtimeMesh::FRealtimeMeshStreamSet &Stored) -> TMap<FRealtimeMeshStreamKey, FInt32Range> {
					const TArrayView<FVector3f> Positions =
					Stored.FindChecked(RealtimeMesh::FRealtimeMeshStreams::Position).GetArrayView<FVector3f>();
					const TArrayView<FColor> Colors = Stored.FindChecked(RealtimeMesh::FRealtimeMeshStreams::Color).GetArrayView<FColor>();
					for(int32 I = Begin; I < Begin + 3; ++I) {
						Positions[I] = FVector3f(Updated.ProcVertexBuffer[I].Position);
						Colors[I] = Updated.ProcVertexBuffer[I].Color;
					}
					// 先頭・内部・末尾の半開区間を指定する返戻
					return {
						{ RealtimeMesh::FRealtimeMeshStreams::Position, FInt32Range(Begin, Begin + 3) },
						{ RealtimeMesh::FRealtimeMeshStreams::Color, FInt32Range(Begin, Begin + 3) }
					};
				}
			);
			Mesh->ProcessMesh(
				UpdateKey,
				[this, &Updated, &Visits](const RealtimeMesh::FRealtimeMeshStreamSet &Stored) -> void {
					++Visits;
					CompareStreams(*this, Stored, Updated);
					// 更新後の比較終了
					return;
				}
			);
		}
		for(const int32 Count : { 129, 300 }) {
			const int32 PreviousCount = Updated.ProcVertexBuffer.Num();
			Updated.ProcVertexBuffer.SetNum(Count);
			for(int32 I = PreviousCount; I < Count; ++I) Updated.ProcVertexBuffer[I] = Section.ProcVertexBuffer[I % 257];
			Updated.ProcIndexBuffer.Reset();
			for(int32 I = 2; I < Count; ++I) Updated.ProcIndexBuffer.Append({ 0, static_cast<uint32>(I), static_cast<uint32>(I - 1) });
			std::optional<RealtimeMesh::FRealtimeMeshStreamSet> Replacement = PreparePlanetRealtimeSection(Updated);
			// 全体更新の変換失敗の返戻
			if(!TestTrue(TEXT("要素数変更後の配列"), Replacement.has_value())) return false;
			Mesh->UpdateBufferSet(UpdateKey, MoveTemp(*Replacement));
			Mesh->ProcessMesh(
				UpdateKey,
				[this, &Updated, &Visits](const RealtimeMesh::FRealtimeMeshStreamSet &Stored) -> void {
					++Visits;
					CompareStreams(*this, Stored, Updated);
					// 更新後の比較終了
					return;
				}
			);
		}
	}
	TestEqual(TEXT("静的・動的の範囲更新と縮小・拡大の全保持配列を読み取り"), Visits, 11);
	Component->DestroyComponent();
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("中断した変換を公開しない"), PreparePlanetRealtimeSection(Section, Cancel.get_token()).has_value());
	Section.bEnableCollision = true;
	TestFalse(TEXT("衝突の黙示的な脱落を拒否"), PreparePlanetRealtimeSection(Section).has_value());
	Section.bEnableCollision = false;
	Section.ProcIndexBuffer[0] = 257;
	TestFalse(TEXT("範囲外の接続を拒否"), PreparePlanetRealtimeSection(Section).has_value());
	Section.ProcIndexBuffer[0] = Section.ProcIndexBuffer[1];
	TestFalse(TEXT("重複接続を拒否"), PreparePlanetRealtimeSection(Section).has_value());
	Section.ProcIndexBuffer[0] = 0;
	Section.ProcVertexBuffer[0].UV3.X = std::numeric_limits<double>::infinity();
	TestFalse(TEXT("非有限な模様座標を拒否"), PreparePlanetRealtimeSection(Section).has_value());
	Section.ProcVertexBuffer[0].UV3.X = 1E10;
	TestFalse(TEXT("半精度の桁溢れを拒否"), PreparePlanetRealtimeSection(Section).has_value());
	Section.ProcVertexBuffer[0].UV3.X = 0;
	Section.ProcVertexBuffer[0].Position.X = std::numeric_limits<double>::max();
	TestFalse(TEXT("単精度の桁溢れを拒否"), PreparePlanetRealtimeSection(Section).has_value());
	Section.ProcVertexBuffer[0].Position.X = 0;
	Section.ProcIndexBuffer.Add(0);
	TestFalse(TEXT("不完全な三角形を拒否"), PreparePlanetRealtimeSection(Section).has_value());
	TestTrue(TEXT("空の水面を許可"), PreparePlanetRealtimeSection(FProcMeshSection{}).has_value());
	AddInfo(
		TEXT("RealtimeSection vertices=257 triangles=255 passes=worker/component/ranged/topology ")
		TEXT("attributes=all updateModes=static/dynamic storedVisits=11 GPU=false")
	);
	// 全検査経路を実行した返戻
	return true;
}

#endif
