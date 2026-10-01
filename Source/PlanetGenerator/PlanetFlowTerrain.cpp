#include "PlanetFlowTerrain.h"
#include "Core/PlanetFlowView.h"
#include "Core/PlanetFlowWater.h"
#include "Core/PlanetSeasonalMap.h"
#include "PlanetCelestialComponent.h"
#include "PlanetExplorerCharacter.h"
#include "PlanetFlowRenderSection.h"
#include "PlanetGroundPatch.h"
#include "PlanetSeasonalTexture.h"
#include "Async/Async.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/SceneComponent.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "RenderCommandFence.h"
#include "UObject/ConstructorHelpers.h"
#include <cmath>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

namespace {
	struct FFlowViewRequest {
		// 不変場の所有権と，同じ惑星座標の表示要求
		std::shared_ptr<const PlanetCore::FlowSurfaceField> Field;
		std::shared_ptr<const PlanetCore::OceanConnectivityField> ClimateOcean;
		PlanetCore::LodView View;
		FTransform PlanetTransform;
		TWeakObjectPtr<APlanetGroundPatch> Ground;
		int64 Revision = 0;
		double InitialSeasonalPhase = 0;
	};

	struct FFlowClimate {
		// 同じ地形定義で準備した気候・被覆を，視点が変わっても共有
		std::shared_ptr<const PlanetCore::FlowSurfaceField> Field;
		std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean;
		std::shared_ptr<const PlanetCore::PlanetClimate> Climate;
		PlanetCore::PlanetSeasonalMap Map;
	};

	struct FFlowViewResult {
		// 完了した要求と未公開の候補
		FFlowViewRequest Request;
		std::optional<PlanetCore::FlowTerrainMesh> Mesh;
		PlanetCore::FlowViewDiagnostics Diagnostics;
		std::optional<PlanetCore::FlowWaterMesh> Water;
		std::optional<FProcMeshSection> TerrainSection;
		std::optional<FProcMeshSection> WaterSection;
		double BufferMilliseconds = 0;
		std::shared_ptr<PlanetCore::FlowWaterView> WaterBuilder;
		PlanetCore::WaterRefinementDiagnostics WaterDiagnostics;
		std::shared_ptr<const FFlowClimate> Climate;
		PlanetCore::ClimatePreparationStatistics ClimateDiagnostics;
		bool PreparedClimate = false;
		double ClimateMilliseconds = 0;
		bool PreparedWater = false;
		double WaterMilliseconds = 0;
		double Milliseconds = 0;
	};

	/**
	 * 元の成分をエンジンの倍精度座標へ渡す
	 * @param P 変換する座標
	 * @return 同じ三成分
	 */
	FVector ToVector(const PlanetCore::Vec3 P) {
		return { P.X, P.Y, P.Z };
	}

	/**
	 * 足場が現在も同じ不変場と惑星座標を公開しているか確認する
	 * @param Request 照合する候補の要求
	 * @return 世代と座標系が同じ場合に真
	 */
	bool HasCurrentGround(const FFlowViewRequest &Request) {
		const APlanetGroundPatch *const Ground = Request.Ground.Get();
		FTransform Transform;
		double Radius = 0;
		// 別の足場定義を同じ描画へ混ぜない返戻
		return Ground && Ground->GetPublishedFlowField() == Request.Field && Ground->GetPublishedPlanet(Transform, Radius) &&
		Radius == Request.View.RadiusMeters && Transform.Equals(Request.PlanetTransform, 0);
	}

	/**
	 * 同じ局所座標の水面候補を，非表示の描画部品へ転送する
	 * @param Target 非表示で更新する部品
	 * @param Material 元の水材質
	 * @param Mesh 同じ場から準備した水面
	 * @param Section ワーカーで準備した完全な描画配列
	 * @param SectionMilliseconds 部品更新の費用への加算先
	 * @return 材質と全頂点・三角形を転送した場合に真
	 * @note 頂点数 V と三角形数 T に対して O(V+T)
	 */
	bool UploadWater(
		UProceduralMeshComponent *const Target,
		UMaterialInterface *const Material,
		const PlanetCore::FlowWaterMesh &Mesh,
		const FProcMeshSection &Section,
		double &SectionMilliseconds
	) {
		// 元の材質と同じ動的パラメーターの準備
		UMaterialInstanceDynamic *Instance = Material ? Cast<UMaterialInstanceDynamic>(Target->GetMaterial(0)) : nullptr;
		if(Material && (!Instance || Instance->Parent != Material)) Instance = Target->CreateDynamicMaterialInstance(0, Material);
		// 有効な材質無しの返戻
		if(!Instance) return false;
		const auto Parameter = [&](const FName Name, const PlanetCore::Vec3 Value) -> void {
			Instance->SetVectorParameterValue(
				Name,
				FLinearColor(static_cast<float>(Value.X), static_cast<float>(Value.Y), static_cast<float>(Value.Z), 0)
			);
			// 終了
			return;
		};
		Parameter(TEXT("TextureOrigin"), Mesh.TextureOrigin);
		Parameter(TEXT("TextureAxisX"), Mesh.Frame.East);
		Parameter(TEXT("TextureAxisY"), Mesh.Frame.North);
		Parameter(TEXT("TextureAxisZ"), Mesh.Frame.Up);
		// 同じ元場の水面位置・解析法線・惑星固定の模様の転送
		const double SectionStarted = FPlatformTime::Seconds();
		Target->SetProcMeshSection(0, Section);
		SectionMilliseconds += (FPlatformTime::Seconds() - SectionStarted) * 1000;
		// 全候補の転送の返戻
		return true;
	}
}

struct FPlanetFlowTerrainState {
	// 唯一の生成ワーカーと最新の待機要求，描画へ転送中の候補
	std::optional<FFlowViewRequest> Pending;
	std::optional<FFlowViewRequest> Published;
	std::shared_ptr<PlanetCore::FlowTerrainView> Builder;
	std::shared_ptr<PlanetCore::FlowWaterView> WaterBuilder;
	std::shared_ptr<const PlanetCore::FlowSurfaceField> ClimateField;
	std::shared_ptr<const PlanetCore::OceanConnectivityField> ClimateOcean;
	std::shared_ptr<const FFlowClimate> Climate;
	std::shared_ptr<const FFlowClimate> PublishedClimate;
	TFuture<std::shared_ptr<FFlowViewResult>> Future;
	std::shared_ptr<FFlowViewResult> Upload;
	std::stop_source Stop;
	FRenderCommandFence Fence;
	bool HasWorker = false;
	int32 Active = 0;
};

/**
 * 現在公開する一枚の全球面を取得する
 * @return 公開済の描画部品；未準備なら空
 */
UProceduralMeshComponent *APlanetFlowTerrain::GetVisibleMesh() const {
	// 実際の公開状態から描画部品を返す
	return State && State->Published ? !State->Active ? BufferA.Get() : BufferB.Get() : nullptr;
}

/**
 * 同じ通番で公開した川・湖の水面を取得する
 * @return 公開済の部品；未準備なら空
 */
UProceduralMeshComponent *APlanetFlowTerrain::GetVisibleWaterMesh() const {
	// 地形と同じ公開バッファに属する水面の返戻
	return State && State->Published ? !State->Active ? WaterBufferA.Get() : WaterBufferB.Get() : nullptr;
}

/**
 * 最初の表示要求前に，同じ元海陸の気候と季節材質を構成する
 * @param Field 描画と衝突が共有する不変場
 * @param Ocean Field の生成に使用した元の海陸；呼出側も同じ所有元を使う事
 * @return 有効な尺度と材質を確保した場合に真；開始後の変更は現在の表示を保持して拒否
 */
bool APlanetFlowTerrain::ConfigureClimate(
	std::shared_ptr<const PlanetCore::FlowSurfaceField> Field,
	std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean
) {
	// 無効な構成で現在の材質と所有状態を変更しない返戻
	if(!State || RequestedRevision || !Field || !Ocean || Field->GetRadiusMeters() != Ocean->Base().Settings().RadiusMeters) {
		// 開始後の変更又は異なる尺度への返戻
		return false;
	}
	UMaterialInterface *const Material =
	LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Planet/Materials/M_PlanetSeasonalSurface.M_PlanetSeasonalSurface"));
	// 季節材質無しで構成済としない返戻
	if(!Material) return false;
	State->ClimateField = std::move(Field);
	State->ClimateOcean = std::move(Ocean);
	SurfaceMaterial = Material;
	// 最初の要求が気候を準備出来る構成の返戻
	return true;
}

/**
 * 同じ描画通番の地形・気候・位置を，準備中の候補と混ぜずに取得する
 * @return 公開済の環境；気候無し又は未公開なら空
 */
std::optional<FPlanetFlowEnvironment> APlanetFlowTerrain::GetPublishedEnvironment() const {
	// 公開済の地形と気候が無い場合の返戻
	if(!State || !State->Published || !State->PublishedClimate || !bClimateReady || !SeasonalTexture) return {};
	// 別の生成定義の被覆を現在の地面へ使わない返戻
	if(State->Published->Field != State->PublishedClimate->Field) return {};
	const FFlowViewRequest &Published = *State->Published;
	const UProceduralMeshComponent *const Mesh = GetVisibleMesh();
	UMaterialInstanceDynamic *const Material = Mesh ? Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(0)) : nullptr;
	// 実際の季節材質が公開されていない場合の返戻
	if(!Material || !Published.View.SurfaceObserverMeters) return {};
	FPlanetFlowEnvironment Result;
	Result.Surface = Published.Field;
	Result.Climate = State->PublishedClimate->Climate;
	Result.Seasons = std::shared_ptr<const PlanetCore::PlanetSeasonalMap>(State->PublishedClimate, &State->PublishedClimate->Map);
	Result.Settings = State->PublishedClimate->Ocean->Base().Settings();
	Result.PlanetTransform = Published.PlanetTransform;
	Result.ObserverMeters = ToVector(*Published.View.SurfaceObserverMeters);
	Result.SurfaceRadiusMeters = Published.View.SurfaceRadiusMeters;
	Result.Phase = Material->K2_GetScalarParameterValue(TEXT("SolarYearPhase"));
	Result.Texture = SeasonalTexture;
	// 有効な公開環境の返戻
	return Result;
}

/**
 * 一件の待機要求を開始し，元場の不変データだけを作業スレッドへ渡す
 */
void APlanetFlowTerrain::StartPending() {
	// 転送又は別の作業と，同じ再利用状態を並行して変更しない終了
	if(!State || State->HasWorker || State->Upload || !State->Pending) return;
	const FFlowViewRequest Request = *State->Pending;
	State->Pending.reset();
	if(!HasCurrentGround(Request)) {
		++DiscardedCandidates;
		Status = TEXT("Ground definition changed before generation; published terrain retained.");
		// 不正な元場へ生成を開始しない終了
		return;
	}
	if(!State->Builder || !State->Builder->Uses(Request.Field)) {
		State->Builder = PlanetCore::FlowTerrainView::Create(Request.Field);
		State->WaterBuilder.reset();
	}
	// 未準備の元場への終了
	if(!State->Builder) return;
	State->Stop = std::stop_source();
	const std::stop_token Stop = State->Stop.get_token();
	const std::shared_ptr<PlanetCore::FlowTerrainView> Builder = State->Builder;
	const std::shared_ptr<PlanetCore::FlowWaterView> WaterBuilder = State->WaterBuilder;
	const std::shared_ptr<const FFlowClimate> Climate = State->Climate;
	State->HasWorker = true;
	State->Future = Async(
		EAsyncExecution::ThreadPool,
		[Request, Builder, WaterBuilder, Climate, Stop]() -> std::shared_ptr<FFlowViewResult> {
			const double Started = FPlatformTime::Seconds();
			const std::shared_ptr<FFlowViewResult> Result = std::make_shared<FFlowViewResult>();
			Result->Request = Request;
			if(Request.ClimateOcean) {
				const double ClimateStarted = FPlatformTime::Seconds();
				if(Climate && Climate->Field == Request.Field && Climate->Ocean == Request.ClimateOcean) Result->Climate = Climate;
				else {
					// 気候の準備も同じワーカーで先に終え，地形生成と重ねない
					std::optional<PlanetCore::PlanetClimate> Prepared =
					PlanetCore::PlanetClimate::Create(*Request.ClimateOcean, {}, Stop, &Result->ClimateDiagnostics);
					std::optional<PlanetCore::PlanetSeasonalMap> Map;
					if(Prepared) Map = PlanetCore::PlanetSeasonalMap::Create(*Prepared, Stop);
					if(Prepared && Map && !Stop.stop_requested()) {
						Result->Climate = std::make_shared<const FFlowClimate>(
							FFlowClimate{
								Request.Field,
								Request.ClimateOcean,
								std::make_shared<const PlanetCore::PlanetClimate>(std::move(*Prepared)),
								std::move(*Map)
							}
						);
						Result->PreparedClimate = true;
					}
				}
				Result->ClimateMilliseconds = (FPlatformTime::Seconds() - ClimateStarted) * 1000;
				// 部分的な季節被覆で通常の地表を公開しない返戻
				if(!Result->Climate) return Result;
			}
			Result->Mesh = Builder->Build(Request.View, Stop, &Result->Diagnostics);
			if(Result->Mesh) {
				const double WaterStarted = FPlatformTime::Seconds();
				Result->WaterBuilder = WaterBuilder ? WaterBuilder : PlanetCore::FlowWaterView::Create(Request.Field, Stop);
				Result->PreparedWater = !WaterBuilder && Result->WaterBuilder != nullptr;
				if(Result->WaterBuilder) {
					Result->Water = Result->WaterBuilder->Build(*Result->Mesh, Request.View, Result->WaterDiagnostics, Stop, &Result->Diagnostics);
				}
				Result->WaterMilliseconds = (FPlatformTime::Seconds() - WaterStarted) * 1000;
			}
			if(Result->Mesh && Result->Water && !Stop.stop_requested()) {
				const double BufferStarted = FPlatformTime::Seconds();
				Result->TerrainSection = PreparePlanetFlowSection(*Result->Mesh, Stop);
				if(Result->TerrainSection) Result->WaterSection = PreparePlanetFlowSection(*Result->Water, Stop);
				Result->BufferMilliseconds = (FPlatformTime::Seconds() - BufferStarted) * 1000;
			}
			Result->Milliseconds = (FPlatformTime::Seconds() - Started) * 1000;
			// UObject に触れずに完成候補と計測だけを返す
			return Result;
		}
	);
	Status = TEXT("Preparing complete flow terrain; published terrain retained.");
	// 終了
	return;
}

/**
 * 足場と同じ定義で，身体とカメラを分けた表示要求を作る
 * @param Ground 公開済の共有河床の足場
 * @param ObserverWorld カメラのワールド位置，センチメートル
 * @param BodyWorld 精細範囲を予約する身体位置，センチメートル
 * @return 有効な最新要求を受理した場合に真；不正入力は公開状態を保持
 */
bool APlanetFlowTerrain::RequestView(APlanetGroundPatch *const Ground, const FVector ObserverWorld, const FVector BodyWorld) {
	// 公開状態を変える前に確認する入力の成立条件
	const bool Valid =
	State && SurfaceMaterial && WaterMaterial && IsValid(Ground) && !ObserverWorld.ContainsNaN() && !BodyWorld.ContainsNaN() &&
	RequestedRevision != MAX_int64;
	// 不正な要求で通番又は公開面を変更しない返戻
	if(!Valid) return false;
	// 物理寸法と異なる拡縮を表示しない返戻
	if(!GetActorScale3D().Equals(FVector::OneVector, 1E-9)) return false;
	FFlowViewRequest Request;
	double Radius = 0;
	Request.Field = Ground->GetPublishedFlowField();
	if(State->ClimateField) {
		// 構成した元場と異なる地形，非有限な季節の要求を拒否する返戻
		if(Request.Field != State->ClimateField || !std::isfinite(SolarYearPhase)) return false;
		Request.ClimateOcean = State->ClimateOcean;
		Request.InitialSeasonalPhase = SolarYearPhase - std::floor(SolarYearPhase);
	}
	// 現在の足場が共有河床を持たない場合の返戻
	if(!Request.Field || !Ground->GetPublishedPlanet(Request.PlanetTransform, Radius)) return false;
	const FVector Observer = Request.PlanetTransform.InverseTransformPosition(ObserverWorld) * 0.01;
	const FVector Body = Request.PlanetTransform.InverseTransformPosition(BodyWorld) * 0.01;
	Request.View.RadiusMeters = Radius;
	Request.View.ReliefBoundMeters = 40000 * Radius / (Radius + 250000);
	Request.View.ObserverMeters = { Observer.X, Observer.Y, Observer.Z };
	Request.View.SurfaceObserverMeters = PlanetCore::Vec3{ Body.X, Body.Y, Body.Z };
	Request.View.SurfaceRadiusMeters = 24;
	Request.View.LeafBudget = 32768;
	Request.View.PixelLeafBudget = 2048;
	Request.View.TargetEdgePixels = 64;
	Request.Ground = Ground;
	// 非有限又は中心の観察点を拒否する返戻
	if(!Request.View.IsValid()) return false;
	Request.Revision = ++RequestedRevision;
	GroundSource = Ground;
	State->Pending = Request;
	StartPending();
	IsBusy = State->HasWorker || State->Pending.has_value() || State->Upload != nullptr;
	// 最新の有効な表示要求の受理を返す
	return true;
}

/**
 * 候補を非表示側へ転送し，RHI の準備後に全球の表示を一括で切替える
 */
void APlanetFlowTerrain::PollView() {
	// 所有状態が解放された場合の終了
	if(!State) return;
	if(State->Upload && State->Fence.IsFenceComplete()) {
		if(State->Upload->Request.Revision == RequestedRevision && HasCurrentGround(State->Upload->Request)) {
			UProceduralMeshComponent *const Next = !State->Active ? BufferB.Get() : BufferA.Get();
			UProceduralMeshComponent *const Previous = !State->Active ? BufferA.Get() : BufferB.Get();
			Next->SetVisibility(true);
			Previous->SetVisibility(false);
			(!State->Active ? WaterBufferB.Get() : WaterBufferA.Get())->SetVisibility(true);
			(!State->Active ? WaterBufferA.Get() : WaterBufferB.Get())->SetVisibility(false);
			State->Active = 1 - State->Active;
			State->Published = State->Upload->Request;
			PublishedRevision = State->Upload->Request.Revision;
			LastWorkerMilliseconds = State->Upload->Milliseconds;
			LastWaterMilliseconds = State->Upload->WaterMilliseconds;
			State->PublishedClimate = State->Upload->Climate;
			SeasonalTexture = UploadSeasonalTexture;
			bClimateReady = State->PublishedClimate && SeasonalTexture;
			if(State->Upload->PreparedClimate) LastClimatePreparationMilliseconds = State->Upload->ClimateMilliseconds;
			Status =
			TEXT("Shared flow terrain and water published. Global hydrology, scenery and traversal performance require validation.");
		} else ++DiscardedCandidates;
		State->Upload.reset();
		UploadSeasonalTexture = nullptr;
	}
	if(State->HasWorker && State->Future.IsReady()) {
		const std::shared_ptr<FFlowViewResult> Ready = State->Future.Consume();
		State->HasWorker = false;
		if(Ready->PreparedWater) ++WaterPreparations;
		if(Ready->PreparedClimate) {
			++ClimatePreparationCount;
			LastClimatePreparationMilliseconds = Ready->ClimateMilliseconds;
		}
		if(Ready->Climate && Ready->Climate->Field == State->ClimateField && Ready->Climate->Ocean == State->ClimateOcean) {
			State->Climate = Ready->Climate;
		}
		if(Ready->WaterBuilder && State->Builder && State->Builder->Uses(Ready->Request.Field)) {
			State->WaterBuilder = Ready->WaterBuilder;
		}
		if(Ready->Request.Revision != RequestedRevision || !HasCurrentGround(Ready->Request)) ++DiscardedCandidates;
		else if(Ready->Request.ClimateOcean && !Ready->Climate) {
			Status = FString::Printf(
				TEXT("気候の準備に失敗しました：段階 %u，完了期数 %u．表示は直前の状態を保持しています．"),
				static_cast<unsigned>(Ready->ClimateDiagnostics.Failure),
				Ready->ClimateDiagnostics.CompletedPeriods
			);
		} else if(!Ready->Mesh) {
			Status = FString::Printf(
				TEXT("Flow terrain rejected: stage=%u error=%.12g m. Previous coverage retained."),
				static_cast<unsigned>(Ready->Diagnostics.Failure),
				Ready->Diagnostics.MaximumSurfaceErrorMeters
			);
		} else if(!Ready->Water) {
			Status = FString::Printf(
				TEXT("Flow water rejected: pass=%u failed=%llu height=%.12g shore=%.12g pixels=%.12g. Published pair retained."),
				Ready->WaterDiagnostics.Pass,
				static_cast<unsigned long long>(Ready->WaterDiagnostics.FailedTriangles),
				Ready->WaterDiagnostics.WaterErrorMeters,
				Ready->WaterDiagnostics.ShoreErrorMeters,
				Ready->WaterDiagnostics.MaximumShoreEdgePixels
			);
		} else if(!Ready->TerrainSection || !Ready->WaterSection) Status = TEXT("描画属性の準備に失敗しました．直前の地面と水面を保持しています．");
		else {
			const double Started = FPlatformTime::Seconds();
			double SectionMilliseconds = 0;
			UploadSeasonalTexture = nullptr;
			if(Ready->Climate) {
				UploadSeasonalTexture =
				Ready->Climate == State->PublishedClimate ? SeasonalTexture.Get() : CreatePlanetSeasonalTexture(Ready->Climate->Map);
				if(!UploadSeasonalTexture) {
					Status = TEXT("季節の表示配列を確保出来ません．直前の表示を保持しています．");
					StartPending();
					IsBusy = State->HasWorker || State->Pending.has_value();
					// 未完成の被覆で表示を切替えない終了
					return;
				}
			}
			const PlanetCore::FlowTerrainMesh &Mesh = *Ready->Mesh;
			const PlanetCore::GroundFrame &Frame = Mesh.Frame;
			UProceduralMeshComponent *const Next = !State->Active ? BufferB.Get() : BufferA.Get();
			Next->SetVisibility(false);
			const FMatrix Basis(ToVector(Frame.East), ToVector(Frame.North), ToVector(Frame.Up), FVector::ZeroVector);
			const FQuat Rotation = Ready->Request.PlanetTransform.GetRotation() * FQuat(Basis);
			const FVector Location = Ready->Request.PlanetTransform.TransformPosition(
				ToVector(Frame.Up) * (100 * (Frame.RadiusMeters + Frame.AnchorElevationMeters))
			);
			Next->SetWorldLocationAndRotation(Location, Rotation);
			UMaterialInstanceDynamic *Material = SurfaceMaterial ? Cast<UMaterialInstanceDynamic>(Next->GetMaterial(0)) : nullptr;
			if(SurfaceMaterial && (!Material || Material->Parent != SurfaceMaterial)) {
				Material = Next->CreateDynamicMaterialInstance(0, SurfaceMaterial);
			}
			if(!Material) {
				Status = TEXT("Surface material unavailable; published terrain retained.");
				StartPending();
				IsBusy = State->HasWorker || State->Pending.has_value();
				// 有効な材質を持たない候補を公開しない終了
				return;
			}
			const auto Parameter = [&](const FName Name, const PlanetCore::Vec3 Value) -> void {
				Material->SetVectorParameterValue(
					Name,
					FLinearColor(static_cast<float>(Value.X), static_cast<float>(Value.Y), static_cast<float>(Value.Z), 0)
				);
				// 終了
				return;
			};
			Parameter(TEXT("TextureOrigin"), Mesh.TextureOrigin);
			Parameter(TEXT("TextureAxisX"), Frame.East);
			Parameter(TEXT("TextureAxisY"), Frame.North);
			Parameter(TEXT("TextureAxisZ"), Frame.Up);
			Material->SetScalarParameterValue(TEXT("FlowHydrologyEnabled"), 1);
			Material->SetScalarParameterValue(TEXT("ClimateEnabled"), Ready->Climate ? 1 : 0);
			if(Ready->Climate) {
				Material->SetTextureParameterValue(TEXT("ClimateAtlas"), UploadSeasonalTexture);
				// 準備中に無効な入力へ変わっても，受理時の有効な季節を保持
				const double Phase =
				std::isfinite(SolarYearPhase) ? SolarYearPhase - std::floor(SolarYearPhase) : Ready->Request.InitialSeasonalPhase;
				Material->SetScalarParameterValue(TEXT("SolarYearPhase"), Phase);
				Material->SetScalarParameterValue(TEXT("ClimateInverseRadius"), 1 / Frame.RadiusMeters);
				Parameter(TEXT("ClimatePlanetOffset"), Frame.Up * (1 + Frame.AnchorElevationMeters / Frame.RadiusMeters));
			}
			const double SectionStarted = FPlatformTime::Seconds();
			Next->SetProcMeshSection(0, *Ready->TerrainSection);
			SectionMilliseconds += (FPlatformTime::Seconds() - SectionStarted) * 1000;
			UProceduralMeshComponent *const NextWater = !State->Active ? WaterBufferB.Get() : WaterBufferA.Get();
			NextWater->SetVisibility(false);
			NextWater->SetWorldLocationAndRotation(Location, Rotation);
			if(!UploadWater(NextWater, WaterMaterial, *Ready->Water, *Ready->WaterSection, SectionMilliseconds)) {
				Status = TEXT("Water material unavailable; published pair retained.");
				StartPending();
				IsBusy = State->HasWorker || State->Pending.has_value();
				// 片方だけの候補を公開しない終了
				return;
			}
			State->Upload = Ready;
			State->Fence.BeginFence(FRenderCommandFence::ESyncDepth::RHIThread);
			LastBufferPreparationMilliseconds = Ready->BufferMilliseconds;
			LastSectionUploadMilliseconds = SectionMilliseconds;
			LastUploadMilliseconds = (FPlatformTime::Seconds() - Started) * 1000;
		}
	}
	StartPending();
	IsBusy = State->HasWorker || State->Pending.has_value() || State->Upload != nullptr;
	// 終了
	return;
}

/**
 * 所有する作業を中断し，アクター破棄後へ重い作業を残さない
 */
void APlanetFlowTerrain::Cancel() {
	if(State) {
		State->Stop.request_stop();
		State->Pending.reset();
		if(State->HasWorker) {
			State->Future.Wait();
			State->Future.Consume();
			State->HasWorker = false;
		}
		State->Upload.reset();
	}
	IsBusy = false;
	// 終了
	return;
}

/**
 * エディタでの非同期転送も通常の更新で進める
 * @return エディタ更新を許可
 */
bool APlanetFlowTerrain::ShouldTickIfViewportsOnly() const {
	return true;
}

/**
 * 公開済の同じ気候配列の年周位相だけを更新し，地形の世代を変えない
 */
void APlanetFlowTerrain::ApplySeasonalMaterial() {
	// 未公開又は無効な位相では，最後の有効な被覆を保持した終了
	if(!bClimateReady || !std::isfinite(SolarYearPhase)) return;
	UProceduralMeshComponent *const Mesh = GetVisibleMesh();
	UMaterialInstanceDynamic *const Material = Mesh ? Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(0)) : nullptr;
	if(Material) Material->SetScalarParameterValue(TEXT("SolarYearPhase"), SolarYearPhase - std::floor(SolarYearPhase));
	// 終了
	return;
}

/**
 * 公開済の惑星中心・姿勢と身体の目線を，同じ天体暦へ対応させる
 * @return 描画と同じ有効な観測地点を設定出来た場合に真
 */
bool APlanetFlowTerrain::ConfigureCelestialObserver() {
	// 未公開の候補又は異なる足場の座標を照明へ混ぜない返戻
	if(!State || !State->Published || !HasCurrentGround(*State->Published)) return false;
	const FFlowViewRequest &Published = *State->Published;
	FVector Eye = Published.PlanetTransform.TransformPosition(ToVector(Published.View.ObserverMeters) * 100);
	if(IsValid(Explorer)) Eye = Explorer->GetActorLocation() + Explorer->GetActorUpVector() * 64;
	// 三人称・俯瞰でも身体の緯度・経度・高度を保持した返戻
	return CelestialClock->SetObserver(Published.PlanetTransform, Published.View.RadiusMeters, Eye);
}

/**
 * 公開した地表の身体位置で，同じ太陽・月・季節の時計を開始する
 * @return 実行中の有効な公開面で開始出来た場合に真
 */
bool APlanetFlowTerrain::StartCelestialClock() {
	// 未公開の表示で昼夜だけを先に開始しない返戻
	if(!ConfigureCelestialObserver()) return false;
	// 共通時計の開始結果の返戻
	return CelestialClock->StartCelestialClock();
}

/**
 * 公開を進め，準備済範囲の内側で身体と現在のカメラへ追従する
 * @param DeltaSeconds 前回の更新からの秒数
 */
void APlanetFlowTerrain::Tick(const float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	PollView();
	if(ConfigureCelestialObserver()) {
		CelestialClock->Advance(DeltaSeconds);
		if(IsValid(Explorer)) Explorer->SetCelestialClock(CelestialClock);
		if(CelestialClock->bCelestialClockActive || CelestialClock->SimulationUtc.GetTicks()) {
			SolarYearPhase = CelestialClock->SolarYearPhase;
		}
	}
	ApplySeasonalMaterial();
	// 身体の無い診断又は生成中は明示要求だけを処理する終了
	if(IsBusy || !IsValid(Explorer) || !IsValid(GroundSource) || !GetWorld() || !GetWorld()->IsGameWorld()) return;
	const FVector Body = Explorer->GetActorLocation();
	FVector Observer = Body + Explorer->GetActorUpVector() * 64;
	const APlayerController *const Controller = Cast<APlayerController>(Explorer->GetController());
	if(Controller && Controller->PlayerCameraManager) Observer = Controller->PlayerCameraManager->GetCameraLocation();
	else {
		TArray<UCameraComponent *> Cameras;
		Explorer->GetComponents(Cameras);
		for(const UCameraComponent *const Camera : Cameras) if(Camera->IsActive()) Observer = Camera->GetComponentLocation();
	}
	bool NeedsView = !State->Published || !HasCurrentGround(*State->Published);
	if(!NeedsView) {
		const FFlowViewRequest &Published = *State->Published;
		const FVector Focus = Published.PlanetTransform.TransformPosition(ToVector(*Published.View.SurfaceObserverMeters) * 100);
		const FVector Eye = Published.PlanetTransform.TransformPosition(ToVector(Published.View.ObserverMeters) * 100);
		NeedsView = FVector::Distance(Focus, Body) > 400 || FVector::Distance(Eye, Observer) > 400;
	}
	if(NeedsView) RequestView(GroundSource, Observer, Body);
	// 終了
	return;
}

/**
 * ゲーム世界の終了時に所有する非同期作業を解放する
 * @param Reason エンジンが通知した終了理由
 */
void APlanetFlowTerrain::EndPlay(const EEndPlayReason::Type Reason) {
	Cancel();
	Super::EndPlay(Reason);
	// 終了
	return;
}

/**
 * エディタ世界での破棄時も生成状態を解放する
 */
void APlanetFlowTerrain::BeginDestroy() {
	Cancel();
	State.Reset();
	Super::BeginDestroy();
	// 終了
	return;
}

/**
 * 保存しない全球描画の二面を準備する
 */
APlanetFlowTerrain::APlanetFlowTerrain() {
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;
	TerrainRoot = CreateDefaultSubobject<USceneComponent>(TEXT("FlowTerrainRoot"));
	SetRootComponent(TerrainRoot);
	CelestialClock = CreateDefaultSubobject<UPlanetCelestialComponent>(TEXT("PlanetCelestialClock"));
	CelestialClock->CreateScene(*this);
	BufferA = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("FlowTerrainA"));
	BufferB = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("FlowTerrainB"));
	WaterBufferA = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("FlowWaterA"));
	WaterBufferB = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("FlowWaterB"));
	static const ConstructorHelpers::FObjectFinder<UMaterialInterface> Water(
		TEXT("/Game/Planet/Materials/M_PlanetOcean.M_PlanetOcean")
	);
	if(Water.Succeeded()) WaterMaterial = Water.Object;
	for(UProceduralMeshComponent *const Buffer : { WaterBufferA.Get(), WaterBufferB.Get() }) {
		Buffer->SetupAttachment(TerrainRoot);
		Buffer->SetFlags(RF_Transient);
		Buffer->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Buffer->SetCanEverAffectNavigation(false);
		Buffer->SetVisibility(false);
		if(Water.Succeeded()) Buffer->SetMaterial(0, Water.Object);
	}
	static const ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(
		TEXT("/Game/Planet/Materials/M_PlanetSurface.M_PlanetSurface")
	);
	if(Material.Succeeded()) SurfaceMaterial = Material.Object;
	for(UProceduralMeshComponent *const Buffer : { BufferA.Get(), BufferB.Get() }) {
		Buffer->SetupAttachment(TerrainRoot);
		Buffer->SetFlags(RF_Transient);
		Buffer->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Buffer->SetCanEverAffectNavigation(false);
		Buffer->SetVisibility(false);
		if(Material.Succeeded()) Buffer->SetMaterial(0, Material.Object);
	}
	State = MakeShared<FPlanetFlowTerrainState>();
	// 終了
	return;
}
