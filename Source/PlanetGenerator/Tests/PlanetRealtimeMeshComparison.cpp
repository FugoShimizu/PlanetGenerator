#include "PlanetFlowTerrain.h"
#include "PlanetRealtimeMeshSection.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "ProceduralMeshComponent.h"
#include "RealtimeMeshComponent.h"
#include "RealtimeMeshSimple.h"
#include "RenderCommandFence.h"
#include <array>
#include <optional>

#if WITH_EDITOR

namespace {
	// 比較操作の間だけ保持する参照と転送完了状態
	struct MeshComparisonState {
		TWeakObjectPtr<APlanetFlowTerrain> Source;
		TWeakObjectPtr<AActor> Owner;
		std::array<TWeakObjectPtr<UProceduralMeshComponent>, 2> Originals;
		std::array<TWeakObjectPtr<UProceduralMeshComponent>, 2> References;
		std::array<TWeakObjectPtr<URealtimeMeshComponent>, 2> Candidates;
		std::array<TFuture<ERealtimeMeshProxyUpdateStatus>, 2> Futures;
		std::array<bool, 2> Visibility {};
		FRenderCommandFence Fence;
		bool AwaitingFence = false;
		bool Ready = false;
		bool SourceTickEnabled = false;
		bool Dynamic = false;
		int32 Revision = 0;
	};

	TUniquePtr<MeshComparisonState> Comparison;

	/**
	 * 比較開始後に地形が更新されていても，既に退役した表示面を再表示しない
	 */
	void RestoreSourceVisibility() {
		// 既に破棄された世界の終了
		if(!Comparison || !Comparison->Source.IsValid()) return;
		const std::array<UProceduralMeshComponent *, 2> Current {
			Comparison->Source->GetVisibleMesh(),
			Comparison->Source->GetVisibleWaterMesh()
		};
		for(int32 I = 0; I < 2; ++I) {
			if(Current[I] && Comparison->Originals[I].Get() == Current[I]) Current[I]->SetVisibility(Comparison->Visibility[I]);
		}
		// 現在も公開されている面だけを復元して終了
		return;
	}

	/**
	 * 所有する比較部品を解放し，元の表示状態へ戻す
	 */
	void ClearComparison() {
		// 未開始の終了
		if(!Comparison) return;
		RestoreSourceVisibility();
		if(Comparison->Source.IsValid()) Comparison->Source->SetActorTickEnabled(Comparison->SourceTickEnabled);
		if(Comparison->Owner.IsValid()) Comparison->Owner->Destroy();
		Comparison.Reset();
		// 終了
		return;
	}

	/**
	 * 同じ公開区画の二方式への転送を測る
	 * @param World 操作中の世界
	 * @param Output 結果の記録先
	 * @return 両面の転送要求を受理した場合に真
	 */
	bool UploadComparison(UWorld *const World, FOutputDevice &Output) {
		const bool Invalid =
		!Comparison || !Comparison->Source.IsValid() || !Comparison->Owner.IsValid() || Comparison->Owner->GetWorld() != World ||
		Comparison->Source->IsBusy || Comparison->Revision && !Comparison->Ready;
		// 失効した比較先と処理中の重複要求を拒否
		if(Invalid) return false;
		APlanetFlowTerrain *const Source = Comparison->Source.Get();
		const std::array<UProceduralMeshComponent *, 2> Inputs { Source->GetVisibleMesh(), Source->GetVisibleWaterMesh() };
		std::array<std::optional<RealtimeMesh::FRealtimeMeshStreamSet>, 2> Streams;
		double Preparation = 0, Pmc = 0, Rmc = 0;
		uint64 Vertices = 0, Triangles = 0;
		for(int32 I = 0; I < 2; ++I) {
			const FProcMeshSection *const Section = Inputs[I] ? Inputs[I]->GetProcMeshSection(0) : nullptr;
			// 両面を揃えない候補の返戻
			if(!Section || !Section->bSectionVisible || !Inputs[I]->GetMaterial(0)) return false;
			const double Started = FPlatformTime::Seconds();
			Streams[I] = PreparePlanetRealtimeSection(*Section);
			Preparation += (FPlatformTime::Seconds() - Started) * 1000;
			// 属性変換に失敗した候補の返戻
			if(!Streams[I]) return false;
			Vertices += Section->ProcVertexBuffer.Num();
			Triangles += Section->ProcIndexBuffer.Num() / 3;
		}
		RestoreSourceVisibility();
		for(int32 I = 0; I < 2; ++I) {
			Comparison->Originals[I] = Inputs[I];
			Comparison->Visibility[I] = Inputs[I]->IsVisible();
			UProceduralMeshComponent *const Reference = Comparison->References[I].Get();
			URealtimeMeshComponent *const Candidate = Comparison->Candidates[I].Get();
			Reference->SetVisibility(false);
			Candidate->SetVisibility(false);
			Reference->SetWorldTransform(Inputs[I]->GetComponentTransform());
			Candidate->SetWorldTransform(Inputs[I]->GetComponentTransform());
			Reference->SetCastShadow(Inputs[I]->CastShadow);
			Candidate->SetCastShadow(Inputs[I]->CastShadow);
			Reference->SetMaterial(0, Inputs[I]->GetMaterial(0));
			URealtimeMeshSimple *const Mesh = Candidate->GetRealtimeMeshAs<URealtimeMeshSimple>();
			Mesh->SetupMaterialSlot(0, FName("Surface"), Inputs[I]->GetMaterial(0));
			const FRealtimeMeshBufferSetKey Key = FRealtimeMeshBufferSetKey::Create(0, FName("Surface"));
			const double PmcStarted = FPlatformTime::Seconds();
			Reference->SetProcMeshSection(0, *Inputs[I]->GetProcMeshSection(0));
			Pmc += (FPlatformTime::Seconds() - PmcStarted) * 1000;
			const double RmcStarted = FPlatformTime::Seconds();
			Comparison->Futures[I] = Comparison->Revision ? Mesh->UpdateBufferSet(Key, MoveTemp(*Streams[I])) : Mesh->CreateBufferSet(
				Key,
				MoveTemp(*Streams[I]),
				FRealtimeMeshBufferSetConfig(Comparison->Dynamic ? ERealtimeMeshSectionDrawType::Dynamic : ERealtimeMeshSectionDrawType::Static)
			);
			Rmc += (FPlatformTime::Seconds() - RmcStarted) * 1000;
		}
		++Comparison->Revision;
		Comparison->Ready = false;
		Comparison->AwaitingFence = false;
		Output.Logf(
			TEXT("RmcComparison revision=%d vertices=%llu triangles=%llu preparationMs=%.6f pmcSubmitMs=%.6f rmcSubmitMs=%.6f"),
			Comparison->Revision,
			Vertices,
			Triangles,
			Preparation,
			Pmc,
			Rmc
		);
		// 転送要求受理の返戻；此の時点では描画完了を意味しない
		return true;
	}

	/**
	 * 固定した接続のまま中央１０２４頂点の色だけを反転し，範囲更新の費用を比較する
	 * @param Output 結果の記録先
	 * @return 両面の更新要求を受理した場合に真
	 */
	bool UploadColorRange(FOutputDevice &Output) {
		// 比較元が変更中の要求や，未完了の転送との重複を拒否
		if(!Comparison || !Comparison->Ready || !Comparison->Source.IsValid() || Comparison->Source->IsBusy) return false;
		const std::array<UProceduralMeshComponent *, 2> Inputs {
			Comparison->Source->GetVisibleMesh(),
			Comparison->Source->GetVisibleWaterMesh()
		};
		std::array<TArray<FColor>, 2> Colors;
		std::array<TArray<FVector>, 2> Positions;
		std::array<int32, 2> Starts {};
		const double PreparationStarted = FPlatformTime::Seconds();
		for(int32 I = 0; I < 2; ++I) {
			UProceduralMeshComponent *const Reference = Comparison->References[I].Get();
			const FProcMeshSection *const Section = Reference ? Reference->GetProcMeshSection(0) : nullptr;
			URealtimeMeshComponent *const Candidate = Comparison->Candidates[I].Get();
			// 元の形状が既に入れ替わった場合は全体更新を先に要求する
			const bool Invalid =
			!Section || Section->ProcVertexBuffer.Num() < 1024 || !Candidate || Comparison->Originals[I].Get() != Inputs[I];
			// 古い形状又は小さすぎる配列の返戻
			if(Invalid) return false;
			const int32 Count = Section->ProcVertexBuffer.Num();
			Starts[I] = (Count - 1024) / 2;
			Colors[I].SetNumUninitialized(Count);
			Positions[I].SetNumUninitialized(Count);
			for(int32 J = 0; J < Count; ++J) {
				// PMC は色だけの更新でも同じ頂点数の位置配列が必要
				Positions[I][J] = Section->ProcVertexBuffer[J].Position;
				const FColor Color = Section->ProcVertexBuffer[J].Color;
				Colors[I][J] = J >= Starts[I] && J < Starts[I] + 1024 ? FColor(255 - Color.R, 255 - Color.G, 255 - Color.B, Color.A) : Color;
			}
		}
		const double Preparation = (FPlatformTime::Seconds() - PreparationStarted) * 1000;
		const TArray<FVector> EmptyVectors;
		const TArray<FVector2D> EmptyCoordinates;
		const TArray<FProcMeshTangent> EmptyTangents;
		double Pmc = 0, Rmc = 0;
		uint64 StoredBytes = 0;
		bool Valid = true;
		for(int32 I = 0; I < 2; ++I) {
			UProceduralMeshComponent *const Reference = Comparison->References[I].Get();
			URealtimeMeshSimple *const Mesh = Comparison->Candidates[I]->GetRealtimeMeshAs<URealtimeMeshSimple>();
			const FRealtimeMeshBufferSetKey Key = FRealtimeMeshBufferSetKey::Create(0, FName("Surface"));
			const double PmcStarted = FPlatformTime::Seconds();
			Reference->UpdateMeshSection(0, Positions[I], EmptyVectors, EmptyCoordinates, Colors[I], EmptyTangents);
			Pmc += (FPlatformTime::Seconds() - PmcStarted) * 1000;
			const double RmcStarted = FPlatformTime::Seconds();
			Comparison->Futures[I] = Mesh->EditMeshInPlaceRanged(
				Key,
				[&Colors, &Starts, I](RealtimeMesh::FRealtimeMeshStreamSet &Streams) -> TMap<FRealtimeMeshStreamKey, FInt32Range> {
					RealtimeMesh::FRealtimeMeshStream &Stream = Streams.FindChecked(RealtimeMesh::FRealtimeMeshStreams::Color);
					const TArrayView<FColor> Values = Stream.GetArrayView<FColor>();
					for(int32 J = Starts[I]; J < Starts[I] + 1024; ++J) Values[J] = Colors[I][J];
					// 変更した半開区間だけを指定する返戻；上流は CPU の配列全体を複製する
					return { { RealtimeMesh::FRealtimeMeshStreams::Color, FInt32Range(Starts[I], Starts[I] + 1024) } };
				}
			);
			Rmc += (FPlatformTime::Seconds() - RmcStarted) * 1000;
			// 計時外で全色を比較し，指定外の頂点を誤って書き換えていない事を確認
			const FProcMeshSection *const Section = Reference->GetProcMeshSection(0);
			for(int32 J = 0; J < Colors[I].Num(); ++J) Valid = Valid && Section->ProcVertexBuffer[J].Color == Colors[I][J];
			bool Visited = false;
			Mesh->ProcessMesh(
				Key,
				[&Colors, &StoredBytes, &Valid, &Visited, I](const RealtimeMesh::FRealtimeMeshStreamSet &Streams) -> void {
					Visited = true;
					const RealtimeMesh::FRealtimeMeshStream &Stream = Streams.FindChecked(RealtimeMesh::FRealtimeMeshStreams::Color);
					StoredBytes += Stream.Num() * Stream.GetStride();
					Valid = Valid && Stream.Num() == Colors[I].Num() && Stream.GetStride() == sizeof(FColor) &&
					!FMemory::Memcmp(Stream.GetData(), Colors[I].GetData(), Colors[I].Num() * sizeof(FColor));
					// 保持配列の比較終了
					return;
				}
			);
			Valid = Valid && Visited;
		}
		++Comparison->Revision;
		Comparison->Ready = false;
		Comparison->AwaitingFence = false;
		Output.Logf(
			TEXT("RmcComparison rangeRevision=%d dynamic=%d changedVertices=2048 requestedBytes=8192 storedColorBytes=%llu ")
			TEXT("preparationMs=%.6f pmcSubmitMs=%.6f rmcSubmitMs=%.6f cpuColorsMatch=%d"),
			Comparison->Revision,
			Comparison->Dynamic,
			StoredBytes,
			Preparation,
			Pmc,
			Rmc,
			Valid
		);
		// CPU の保持色の一致を含む返戻；GPU の読戻しではない
		return Valid;
	}

	/**
	 * 検証用の一時部品だけを作成し，実画面の比較を操作する
	 * @param Args begin・begin_dynamic・update・range・status・rmc・pmc・original・clear の何れか
	 * @param World 操作中の世界
	 * @param Output 結果の記録先
	 */
	void CompareMeshCommand(const TArray<FString> &Args, UWorld *World, FOutputDevice &Output) {
		// 引数不備の終了
		if(!World || Args.Num() != 1) {
			Output.Log(TEXT("RmcComparison rejected: use one operation in an editor or play world"));
			// 引数不備の終了
			return;
		}
		const FString &Action = Args[0];
		if(Action == TEXT("clear")) {
			ClearComparison();
			Output.Log(TEXT("RmcComparison cleared"));
			// 所有資源解放後の終了
			return;
		}
		if(Action == TEXT("begin") || Action == TEXT("begin_dynamic")) {
			// 実行中の比較を黙示的に破棄しない終了
			if(Comparison && Comparison->Owner.IsValid()) {
				Output.Log(TEXT("RmcComparison rejected: clear the existing comparison first"));
				// 重複開始の終了
				return;
			}
			ClearComparison();
			APlanetFlowTerrain *Source = nullptr;
			for(TActorIterator<APlanetFlowTerrain> It(World); It; ++It) if(It->GetVisibleMesh() && It->PublishedRevision > 0) {
				// 複数の生成器から対象を推定しない終了
				if(Source) {
					Output.Log(TEXT("RmcComparison rejected: multiple published Flow terrain actors"));
					// 対象不定の終了
					return;
				}
				Source = *It;
			}
			if(!Source || Source->IsBusy) {
				Output.Log(TEXT("RmcComparison rejected: wait for one published, idle Flow terrain"));
				// 有効な公開面無しの終了
				return;
			}
			FActorSpawnParameters Parameters;
			Parameters.ObjectFlags |= RF_Transient;
			AActor *const Owner = World->SpawnActor<AActor>(Parameters);
			if(!Owner) return;
			Comparison = MakeUnique<MeshComparisonState>();
			Comparison->Owner = Owner;
			Comparison->Dynamic = Action == TEXT("begin_dynamic");
			Comparison->Source = Source;
			Comparison->SourceTickEnabled = Source->IsActorTickEnabled();
			Source->SetActorTickEnabled(false);
			for(int32 I = 0; I < 2; ++I) {
				UProceduralMeshComponent *const Reference = NewObject<UProceduralMeshComponent>(Owner, NAME_None, RF_Transient);
				URealtimeMeshComponent *const Candidate = NewObject<URealtimeMeshComponent>(Owner, NAME_None, RF_Transient);
				Owner->AddInstanceComponent(Reference);
				Owner->AddInstanceComponent(Candidate);
				Reference->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				Candidate->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				Reference->SetVisibility(false);
				Candidate->SetVisibility(false);
				Candidate->InitializeRealtimeMesh<URealtimeMeshSimple>();
				Reference->RegisterComponent();
				Candidate->RegisterComponent();
				Comparison->References[I] = Reference;
				Comparison->Candidates[I] = Candidate;
			}
			if(!UploadComparison(World, Output)) {
				Output.Log(TEXT("RmcComparison rejected: incomplete render-only sections or materials"));
				ClearComparison();
			}
			// 初回の非同期転送要求後の終了
			return;
		}
		// 失効した比較状態で操作しない終了
		if(!Comparison || !Comparison->Owner.IsValid() || Comparison->Owner->GetWorld() != World) return;
		if(Action == TEXT("range")) {
			Output.Logf(TEXT("RmcComparison rangeAccepted=%d"), UploadColorRange(Output));
			// 色の範囲指定更新後の終了
			return;
		}
		if(Action == TEXT("update")) {
			Output.Logf(TEXT("RmcComparison updateAccepted=%d"), UploadComparison(World, Output));
			// 更新要求後の終了
			return;
		}
		if(!Comparison->Ready && (Comparison->AwaitingFence || Comparison->Futures[0].IsReady() && Comparison->Futures[1].IsReady())) {
			if(!Comparison->AwaitingFence) {
				Output.Logf(
					TEXT("RmcComparison proxyResults=%d,%d"),
					static_cast<int32>(Comparison->Futures[0].Get()),
					static_cast<int32>(Comparison->Futures[1].Get())
				);
				World->SendAllEndOfFrameUpdates();
				Comparison->Fence.BeginFence(FRenderCommandFence::ESyncDepth::RHIThread);
				Comparison->AwaitingFence = true;
			} else Comparison->Ready = Comparison->Fence.IsFenceComplete();
		}
		if(Action == TEXT("status")) {
			Output.Logf(TEXT("RmcComparison ready=%d revision=%d"), Comparison->Ready, Comparison->Revision);
			// 状態照会後の終了
			return;
		}
		// 不完全な転送又は未知の表示指定で旧表示を消さない終了
		if(!Comparison->Ready || Action != TEXT("rmc") && Action != TEXT("pmc") && Action != TEXT("original")) return;
		for(int32 I = 0; I < 2; ++I) {
			if(Comparison->Originals[I].IsValid()) {
				Comparison->Originals[I]->SetVisibility(Action == TEXT("original") && Comparison->Visibility[I]);
			}
			Comparison->References[I]->SetVisibility(Action == TEXT("pmc") && Comparison->Visibility[I]);
			Comparison->Candidates[I]->SetVisibility(Action == TEXT("rmc") && Comparison->Visibility[I]);
		}
		Output.Logf(TEXT("RmcComparison visible=%s"), *Action);
		// 表示方式切替後の終了
		return;
	}

	FAutoConsoleCommandWithWorldArgsAndOutputDevice ComparisonCommand(
		TEXT("Planet.CompareRmc"),
		TEXT("Compare the same published Flow terrain: begin/begin_dynamic/update/range/status/rmc/pmc/original/clear"),
		FConsoleCommandWithWorldArgsAndOutputDeviceDelegate::CreateStatic(&CompareMeshCommand)
	);
}

#endif
