#include "PlanetVegetation.h"
#include "Core/PlanetFlowSurfaceField.h"
#include "Core/PlanetScatter.h"
#include "Core/PlanetSeasonalMap.h"
#include "Core/PlanetSurfaceMaterial.h"
#include "Core/PlanetTreePalette.h"
#include "PlanetExplorerCharacter.h"
#include "PlanetFlowTerrain.h"
#include "PlanetWorld.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/CapsuleComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "HAL/PlatformTime.h"
#include "Kismet/GameplayStatics.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>

namespace {
	// 球面の配置セルを同定する型
	using FPlantKey = std::tuple<std::uint8_t, std::uint64_t, std::uint64_t>;

	struct FPreparedPlant {
		// 地表に接する根と，その法線
		FVector PositionMeters = FVector::ZeroVector;
		FVector Normal = FVector::UpVector;
		bool bAccepted = false;
	};

	struct FPreparedTree {
		// 同じ候補の樹種・姿勢・幹の衝突を再利用
		std::optional<PlanetCore::BiomeProfile> Profile;
		PlanetCore::BiomeSite Site;
		FVector PositionMeters = FVector::ZeroVector;
		FQuat Rotation = FQuat::Identity;
		TWeakObjectPtr<UCapsuleComponent> Collision;
		FVector Scale = FVector::OneVector;
		bool bConifer = false;
		bool bVariant = false;
		int32 Instance = INDEX_NONE;
	};
}

struct FPlanetVegetationState {
	// 公開地形と同じ気候・座標を各更新の冒頭で取得
	std::shared_ptr<const PlanetCore::RiverTerrainField> Source;
	std::optional<FPlanetFlowEnvironment> FlowEnvironment;
	PlanetCore::GeologySettings Settings;
	FTransform PlanetTransform;
	FVector SurfaceObserverMeters = FVector::ZeroVector;
	double SurfaceRadiusMeters = 0;
	double FoliageRadiusMeters = 0;
	double SeasonalPhase = 0;
	// 草の公開済候補と，時間を分けて準備する次の候補
	std::optional<PlanetCore::PlanetScatter> Scatter;
	std::optional<PlanetCore::ScatterQueryResult> Query;
	std::map<FPlantKey, FPreparedPlant> PublishedPlants;
	std::map<FPlantKey, FPreparedPlant> PendingPlants;
	FVector RequestedCenter = FVector::ZeroVector;
	FVector PublishedCenter = FVector::ZeroVector;
	FVector AnchorMeters = FVector::ZeroVector;
	TArray<FTransform> Transforms;
	TArray<FVector> Directions;
	std::size_t NextCandidate = 0;
	bool bVisibleA = true;
	// 樹木の候補と，最後に反映した季節
	std::optional<PlanetCore::PlanetScatter> TreeScatter;
	std::optional<PlanetCore::ScatterQueryResult> TreeQuery;
	std::map<FPlantKey, FPreparedTree> PublishedTrees;
	std::map<FPlantKey, FPreparedTree> PendingTrees;
	FVector TreeCenter = FVector::ZeroVector;
	FVector RequestedTreeCenter = FVector::ZeroVector;
	double RequestedTreeRadiusMeters = 0;
	std::size_t NextTree = 0;
	double TreePhase = -1;
};

/**
 * 衝突を持たない二つの草の表示面を構築する
 */
APlanetVegetation::APlanetVegetation() {
	// 身体と描画の更新後に，所有する一時配置を更新
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;
	State = MakeShared<FPlanetVegetationState>();
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("VegetationRoot")));
	GrassA = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("GrassA"));
	GrassB = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("GrassB"));
	Trees = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("Trees"));
	TreesVariant = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("TreesVariant"));
	Conifers = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("Conifers"));
	// 既存の三樹種と同じ資産・個体属性を利用
	Trees->SetupAttachment(GetRootComponent());
	Trees->SetMobility(EComponentMobility::Movable);
	Trees->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Trees->SetCanEverAffectNavigation(false);
	Trees->SetVisibleInRayTracing(false);
	Trees->SetNumCustomDataFloats(4);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> TreeMesh(
		TEXT("/Game/Planet/Vegetation/SM_BroadleafSapling.SM_BroadleafSapling")
	);
	if(TreeMesh.Succeeded()) Trees->SetStaticMesh(TreeMesh.Object);
	TreesVariant->SetupAttachment(GetRootComponent());
	TreesVariant->SetMobility(EComponentMobility::Movable);
	TreesVariant->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	TreesVariant->SetCanEverAffectNavigation(false);
	TreesVariant->SetVisibleInRayTracing(false);
	TreesVariant->SetNumCustomDataFloats(4);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> VariantMesh(
		TEXT("/Game/Planet/Vegetation/SM_BroadleafSaplingVariant.SM_BroadleafSaplingVariant")
	);
	if(VariantMesh.Succeeded()) TreesVariant->SetStaticMesh(VariantMesh.Object);
	Conifers->SetupAttachment(GetRootComponent());
	Conifers->SetMobility(EComponentMobility::Movable);
	Conifers->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Conifers->SetCanEverAffectNavigation(false);
	Conifers->SetVisibleInRayTracing(false);
	Conifers->SetNumCustomDataFloats(4);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> ConiferMesh(
		TEXT("/Game/Planet/Vegetation/SM_ConiferSapling.SM_ConiferSapling")
	);
	if(ConiferMesh.Succeeded()) Conifers->SetStaticMesh(ConiferMesh.Object);
	// 草は衝突無しの二つの表示部品を交互に公開
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Mesh(TEXT("/Game/Planet/Vegetation/SM_GrassClump.SM_GrassClump"));
	for(UInstancedStaticMeshComponent *const Grass : { GrassA.Get(), GrassB.Get() }) {
		Grass->SetupAttachment(GetRootComponent());
		Grass->SetMobility(EComponentMobility::Movable);
		Grass->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Grass->SetCanEverAffectNavigation(false);
		Grass->SetAffectDistanceFieldLighting(false);
		Grass->SetVisibleInRayTracing(false);
		Grass->SetNumCustomDataFloats(3);
		Grass->SetCullDistances(1200, 1800);
		Grass->SetVisibility(false);
		if(Mesh.Succeeded()) Grass->SetStaticMesh(Mesh.Object);
	}
	// 終了
	return;
}

/**
 * 準備済みの地表と身体へ接続する
 * @param World 同じ地形と気候を公開する惑星
 * @param Body 配置範囲の中心となる身体
 */
void APlanetVegetation::Initialize(APlanetWorld *const World, APlanetExplorerCharacter *const Body) {
	if(Planet) RemoveTickPrerequisiteActor(Planet);
	if(FlowPlanet) RemoveTickPrerequisiteActor(FlowPlanet);
	if(Explorer) RemoveTickPrerequisiteActor(Explorer);
	FlowPlanet = nullptr;
	Planet = World;
	Explorer = Body;
	SetFlags(RF_Transient);
	if(Planet) AddTickPrerequisiteActor(Planet);
	if(Explorer) AddTickPrerequisiteActor(Explorer);
	// 再接続時に動的材質を別の動的材質の親にせず，同じ一時材質を保持
	if(!GrassMaterial) if(UMaterialInterface *const Material = GrassA->GetMaterial(0)) {
		GrassMaterial = UMaterialInstanceDynamic::Create(Material, this);
		if(GrassMaterial) {
			GrassMaterial->SetFlags(RF_Transient);
			GrassA->SetMaterial(0, GrassMaterial);
			GrassB->SetMaterial(0, GrassMaterial);
		}
	}
	if(!TreeBarkMaterial) if(UMaterialInterface *const Bark = Trees->GetMaterial(0)) {
		TreeBarkMaterial = UMaterialInstanceDynamic::Create(Bark, this);
		if(TreeBarkMaterial) {
			TreeBarkMaterial->SetFlags(RF_Transient);
			Trees->SetMaterial(0, TreeBarkMaterial);
			TreesVariant->SetMaterial(0, TreeBarkMaterial);
			Conifers->SetMaterial(0, TreeBarkMaterial);
		}
	}
	if(!TreeLeafMaterial) if(UMaterialInterface *const Leaves = Trees->GetMaterial(1)) {
		TreeLeafMaterial = UMaterialInstanceDynamic::Create(Leaves, this);
		if(TreeLeafMaterial) {
			TreeLeafMaterial->SetFlags(RF_Transient);
			Trees->SetMaterial(1, TreeLeafMaterial);
			TreesVariant->SetMaterial(1, TreeLeafMaterial);
		}
	}
	if(!ConiferNeedleMaterial) if(UMaterialInterface *const Needles = Conifers->GetMaterial(1)) {
		ConiferNeedleMaterial = UMaterialInstanceDynamic::Create(Needles, this);
		if(ConiferNeedleMaterial) {
			ConiferNeedleMaterial->SetFlags(RF_Transient);
			Conifers->SetMaterial(1, ConiferNeedleMaterial);
		}
	}
	// 終了
	return;
}

/**
 * 合成した水系の公開地形と気候へ，同じ草木の配置処理を接続する
 * @param World 公開済又は準備中の描画元
 * @param Body 配置範囲の中心となる身体
 */
void APlanetVegetation::InitializeFlow(APlanetFlowTerrain *const World, APlanetExplorerCharacter *const Body) {
	Initialize(nullptr, Body);
	FlowPlanet = World;
	if(FlowPlanet) AddTickPrerequisiteActor(FlowPlanet);
	// 終了
	return;
}

/**
 * 現在公開した地形・気候・精密描画範囲を一組で読む
 * @param SourceChanged 配置の再準備が必要な定義変更
 * @return 同じ生成定義を取得した場合に真
 */
bool APlanetVegetation::RefreshEnvironment(bool &SourceChanged) {
	if(IsValid(FlowPlanet)) {
		std::optional<FPlanetFlowEnvironment> Current = FlowPlanet->GetPublishedEnvironment();
		// 未公開の気候を使わない返戻
		if(!Current || !Current->Texture.IsValid()) return false;
		SourceChanged = !State->FlowEnvironment || State->FlowEnvironment->Surface != Current->Surface ||
		!State->PlanetTransform.Equals(Current->PlanetTransform, 0);
		State->FlowEnvironment = std::move(Current);
		State->Source.reset();
		const FPlanetFlowEnvironment &Environment = *State->FlowEnvironment;
		State->Settings = Environment.Settings;
		State->PlanetTransform = Environment.PlanetTransform;
		State->SurfaceObserverMeters = Environment.ObserverMeters;
		State->SurfaceRadiusMeters = Environment.SurfaceRadiusMeters;
		State->FoliageRadiusMeters = 0;
		State->SeasonalPhase = Environment.Phase;
		// 同じ公開世代の返戻
		return true;
	}
	// 従来の地表と気候が準備出来ない場合の返戻
	if(!IsValid(Planet) || !Planet->bClimateReady || !Planet->GetSeasonalTexture()) return false;
	const std::shared_ptr<const PlanetCore::RiverTerrainField> Source = Planet->GetPublishedField();
	// 場が無い場合の返戻
	if(!Source) return false;
	SourceChanged =
	State->FlowEnvironment.has_value() || Source != State->Source || !State->PlanetTransform.Equals(Planet->GetActorTransform(), 0);
	State->FlowEnvironment.reset();
	State->Source = Source;
	State->Settings = Source->Base().Settings();
	State->PlanetTransform = Planet->GetActorTransform();
	State->SurfaceObserverMeters = Planet->VisibleSurfaceObserverMeters;
	State->SurfaceRadiusMeters = Planet->VisibleSurfaceRadiusMeters;
	State->FoliageRadiusMeters = Planet->VisibleFoliageRadiusMeters;
	State->SeasonalPhase = Planet->SolarYearPhase;
	// 従来の公開環境の返戻
	return true;
}

/**
 * 描画と同じ水域の支持範囲を使い，川岸を含む配置用の地表を読む
 * @param Direction 惑星固定の候補方向
 * @return 標高と水位；評価失敗では空
 */
std::optional<PlanetCore::RiverSurfaceSample> APlanetVegetation::SampleSurface(
	const PlanetCore::UnitDirection Direction
) const {
	// 従来の場の結果を保持する返戻
	if(!State->FlowEnvironment) {
		// 有効な従来場の評価結果の返戻
		return State->Source ? std::optional<PlanetCore::RiverSurfaceSample>(State->Source->EvaluateSurface(Direction)) : std::nullopt;
	}
	const PlanetCore::FlowSurfaceField &Field = *State->FlowEnvironment->Surface;
	const std::optional<PlanetCore::FlowWaterSample> Sample = Field.EvaluateWithSupport(Direction);
	// 不明な水域を乾いた地面に置換えない返戻
	if(!Sample || !std::isfinite(Sample->SupportMeters) || Sample->SupportMeters > 0 && !Sample->Section.HasWater) return {};
	PlanetCore::RiverSurfaceSample Result;
	Result.Terrain = Sample->Section.Terrain;
	if(Sample->SupportMeters > 0) {
		Result.ChannelCoverage = std::clamp(Sample->SupportMeters * 10, 0.0, 1.0);
		Result.WaterElevationMeters = Sample->Section.Water.ElevationMeters;
		Result.WaterSupport = 1;
	}
	// 地面の材質と同じ岸の判定値の返戻
	return Result;
}

/**
 * 地面と同じ季節被覆を，候補の年間適性に用いる
 * @param Direction 惑星固定の方向
 * @param Phase 年周位相
 * @param Coverage 被覆の四成分
 * @param Factors 湿潤・雪氷等の係数
 * @return 有効な季節試料の場合に真
 */
bool APlanetVegetation::SampleSeason(
	const FVector Direction,
	const double Phase,
	FLinearColor &Coverage,
	FLinearColor &Factors
) const {
	// 従来の季節評価の返戻
	if(!State->FlowEnvironment) return Planet && Planet->EvaluateSeasonalSurface(Direction, Phase, Coverage, Factors);
	const std::optional<PlanetCore::UnitDirection> Unit =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 不正な方向の返戻
	if(!Unit) return false;
	const std::optional<std::array<double, 8>> Sample = State->FlowEnvironment->Seasons->Sample(*Unit, Phase);
	// 無効な年周位相の返戻
	if(!Sample) return false;
	Coverage = FLinearColor((*Sample)[0], (*Sample)[1], (*Sample)[2], (*Sample)[3]);
	Factors = FLinearColor((*Sample)[4], (*Sample)[5], (*Sample)[6], (*Sample)[7]);
	// 同じ季節被覆の返戻
	return true;
}

/**
 * 同じ気候の樹種適性と季節応答を読む
 * @param Direction 惑星固定の方向
 * @return 生息環境の応答；無効な方向では空
 */
std::optional<PlanetCore::BiomeProfile> APlanetVegetation::SampleBiome(const FVector Direction) const {
	// 従来の気候応答の返戻
	if(!State->FlowEnvironment) return Planet ? Planet->GetBiomeProfile(Direction) : std::nullopt;
	const std::optional<PlanetCore::UnitDirection> Unit =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 同じ不変な気候から応答を返す
	return Unit ? State->FlowEnvironment->Climate->SampleBiome(*Unit) : std::nullopt;
}

/**
 * 描画元が所有する同じ一時配列を読む
 * @return 公開した配列；準備前では空
 */
UTexture2D *APlanetVegetation::GetSeasonalTexture() const {
	// 気候配列を重複作成しない返戻
	return State->FlowEnvironment ? State->FlowEnvironment->Texture.Get() : Planet ? Planet->GetSeasonalTexture() : nullptr;
}

/**
 * 現在公開している草の部品を読む
 * @return 公開側の描画部品
 */
UInstancedStaticMeshComponent *APlanetVegetation::GetVisibleGrass() const {
	// 公開側の部品の返戻
	return State->bVisibleA ? GrassA.Get() : GrassB.Get();
}

/**
 * 一候補を同じ地形と年間の被覆で評価し，既存の評価結果を再利用する
 */
void APlanetVegetation::PrepareCandidate() {
	const PlanetCore::ScatterCandidate &Candidate = State->Query->Candidates[State->NextCandidate++];
	const FPlantKey Key { Candidate.Cell.Face, Candidate.Cell.U, Candidate.Cell.V };
	const PlanetCore::Vec3 Direction = Candidate.Direction.Vector();
	const FVector Radial(Direction.X, Direction.Y, Direction.Z);
	FPreparedPlant Plant;
	if(
		const std::map<FPlantKey, FPreparedPlant>::const_iterator Found = State->PublishedPlants.find(Key);
		Found != State->PublishedPlants.end()
	) {
		Plant = Found->second;
		++ReusedCandidates;
	} else {
		const std::optional<PlanetCore::RiverSurfaceSample> Evaluated = SampleSurface(Candidate.Direction);
		// 欠けた標高を乾いた地面として使わない終了
		if(!Evaluated) return;
		const PlanetCore::RiverSurfaceSample &Sample = *Evaluated;
		++HeightEvaluations;
		const PlanetCore::GeologySettings &Settings = State->Settings;
		const double Elevation = Sample.Terrain.ElevationMeters, Radius = Settings.RadiusMeters;
		const double Slope = Sample.Terrain.TangentialGradientPerRadian.Length() / (Radius + Elevation);
		if(
			Elevation > Settings.SeaLevelMeters + 0.05 && Slope < 0.8 &&
			(!Sample.ChannelCoverage || Elevation > Sample.WaterElevationMeters + 0.05)
		) {
			const PlanetCore::SurfaceMaterialWeights Weights =
			PlanetCore::EvaluateSurfaceMaterial(Candidate.Direction, Radius, Settings.SeaLevelMeters, Sample);
			double AnnualGrass = 0;
			for(std::uint32_t Period = 0; Period < 12; ++Period) {
				FLinearColor Cover, Factors;
				if(SampleSeason(Radial, Period / 12.0, Cover, Factors)) {
					AnnualGrass = std::max(AnnualGrass, static_cast<double>((Cover.R + Cover.G) * (1 - Factors.B)));
				}
			}
			const double Density = FMath::Clamp(AnnualGrass * Weights.Soil * (1 - Weights.Wetness), 0.0, 1.0);
			Plant.bAccepted = Candidate.Selection < Density;
			Plant.PositionMeters = Radial * (Radius + Elevation);
			const PlanetCore::Vec3 Gradient = Sample.Terrain.TangentialGradientPerRadian;
			Plant.Normal = (Radial - FVector(Gradient.X, Gradient.Y, Gradient.Z) / (Radius + Elevation)).GetSafeNormal();
		}
	}
	State->PendingPlants.emplace(Key, Plant);
	if(Plant.bAccepted) {
		const FQuat Rotation = FQuat::FindBetweenNormals(FVector::UpVector, Plant.Normal) *
		FQuat(FVector::UpVector, Candidate.RotationTurns * 2 * UE_DOUBLE_PI);
		State->Transforms.Emplace(Rotation, (Plant.PositionMeters - State->AnchorMeters) * 100, FVector(Candidate.Size));
		State->Directions.Add(Radial);
	}
	// 終了
	return;
}

/**
 * 非表示側へ準備済みの草を転送し，完了した配置を公開する
 */
void APlanetVegetation::PublishGrass() {
	const double Start = FPlatformTime::Seconds();
	UInstancedStaticMeshComponent *const Previous = GetVisibleGrass();
	UInstancedStaticMeshComponent *const Next = State->bVisibleA ? GrassB.Get() : GrassA.Get();
	Next->ClearInstances();
	Next->SetWorldTransform(
		FTransform(State->PlanetTransform.GetRotation(), State->PlanetTransform.TransformPosition(State->AnchorMeters * 100))
	);
	const double Cleared = FPlatformTime::Seconds();
	Next->AddInstances(State->Transforms, false, false, false);
	const double Added = FPlatformTime::Seconds();
	for(int32 Index = 0; Index < State->Directions.Num(); ++Index) {
		const FVector Direction = State->Directions[Index];
		const float Values[] { static_cast<float>(Direction.X), static_cast<float>(Direction.Y), static_cast<float>(Direction.Z) };
		Next->SetCustomData(Index, MakeArrayView(Values), false);
	}
	const double Prepared = FPlatformTime::Seconds();
	LastUploadStagesMilliseconds = FVector(Cleared - Start, Added - Cleared, Prepared - Added) * 1000;
	Next->MarkRenderStateDirty();
	Next->SetVisibility(true);
	Previous->SetVisibility(false);
	State->bVisibleA = !State->bVisibleA;
	State->PublishedCenter = State->RequestedCenter;
	State->PublishedPlants = std::move(State->PendingPlants);
	State->PendingPlants.clear();
	GrassInstanceCount = State->Transforms.Num();
	State->Transforms.Reset();
	State->Directions.Reset();
	State->Query.reset();
	bPreparing = false;
	++PublishedRevision;
	LastUploadMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
	Status = TEXT("近傍の草を公開しました");
	// 終了
	return;
}

/**
 * 同じ季節配列と，公開済み近景の範囲を草の材質へ反映する
 */
void APlanetVegetation::UpdateAppearance() {
	// 準備前の材質を変更しない終了
	if(!GrassMaterial) return;
	if(UTexture2D *const Texture = GetSeasonalTexture()) {
		GrassMaterial->SetTextureParameterValue(TEXT("ClimateAtlas"), Texture);
		GrassMaterial->SetScalarParameterValue(TEXT("SolarYearPhase"), State->SeasonalPhase);
	}
	if(const APlayerCameraManager *const Camera = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0)) {
		const FVector Center = State->PlanetTransform.TransformPosition(State->SurfaceObserverMeters * 100);
		const FVector Relative = Center - Camera->GetCameraLocation();
		GrassMaterial->SetVectorParameterValue(TEXT("NearCenterRelative"), FLinearColor(Relative.X, Relative.Y, Relative.Z, 0));
		GrassMaterial->SetScalarParameterValue(TEXT("NearRadius"), FMath::Max(0.0, State->SurfaceRadiusMeters - 1) * 100);
	}
	// 終了
	return;
}

/**
 * 身体近傍だけを更新し，地表評価を一更新あたり約１ ms の小分けで進める
 * @param DeltaSeconds 前回の更新からの秒数
 */
void APlanetVegetation::Tick(const float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	bool SourceChanged = false;
	if(!IsValid(Explorer) || !GrassMaterial || !RefreshEnvironment(SourceChanged)) {
		GrassA->SetVisibility(false);
		GrassB->SetVisibility(false);
		ResetTrees();
		// 有効な地表と身体の準備を待つ終了
		return;
	}
	if(PublishedRevision) GetVisibleGrass()->SetVisibility(true);
	UpdateAppearance();
	if(SourceChanged) {
		ResetTrees();
		const PlanetCore::GeologySettings &Settings = State->Settings;
		State->Scatter = PlanetCore::PlanetScatter::Create({ Settings.Seed, 1, Settings.RadiusMeters, 0.65 });
		State->TreeScatter = PlanetCore::PlanetScatter::Create({ Settings.Seed, 2, Settings.RadiusMeters, TreeSpacingMeters });
		State->Query.reset();
		State->PublishedPlants.clear();
		State->PendingPlants.clear();
		State->Transforms.Reset();
		State->Directions.Reset();
		GrassA->ClearInstances();
		GrassB->ClearInstances();
		GrassInstanceCount = 0;
		PublishedRevision = 0;
		bPreparing = false;
	}
	// 不正な配置定義の場合の終了
	if(!State->Scatter) return;
	const FVector Body = State->PlanetTransform.InverseTransformPosition(Explorer->GetActorLocation()) / 100;
	const FVector Center = Body.GetSafeNormal() * State->Settings.RadiusMeters;
	if(!State->Query && (!PublishedRevision || FVector::Dist(Center, State->PublishedCenter) > 2)) {
		const double Start = FPlatformTime::Seconds();
		const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector({ Center.X, Center.Y, Center.Z });
		State->Query = State->Scatter->Query(Direction, 20, 100000);
		LastQueryMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
		if(!State->Query) {
			Status = TEXT("草の検索が配置予算を越えました");
			// 完了しない配置を公開しない終了
			return;
		}
		State->RequestedCenter = Center;
		State->AnchorMeters = Body;
		State->NextCandidate = 0;
		State->Transforms.Reset();
		State->Directions.Reset();
		State->PendingPlants.clear();
		bPreparing = true;
	}
	if(State->Query) {
		const double Start = FPlatformTime::Seconds();
		while(State->NextCandidate < State->Query->Candidates.size()) {
			PrepareCandidate();
			if(FPlatformTime::Seconds() - Start >= 0.001) break;
		}
		LastPreparationMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
		if(State->NextCandidate == State->Query->Candidates.size()) PublishGrass();
	}
	UpdateTrees();
	UpdateTreeAppearance();
	// 終了
	return;
}

/**
 * 公開している樹木の描画部品を読む
 * @return 樹木の描画部品
 */
UInstancedStaticMeshComponent *APlanetVegetation::GetTrees() const {
	// 樹木の描画部品の返戻
	return Trees;
}

/**
 * 別樹冠の広葉樹描画部品を読む
 * @return 対応する樹種の描画部品
 */
UInstancedStaticMeshComponent *APlanetVegetation::GetTreeVariant() const {
	// 描画部品の返戻
	return TreesVariant;
}

/**
 * 針葉樹の描画部品を読む
 * @return 対応する樹種の描画部品
 */
UInstancedStaticMeshComponent *APlanetVegetation::GetConifers() const {
	// 描画部品の返戻
	return Conifers;
}

/**
 * 以前の生成定義の樹木と，所有する幹の衝突だけを解除する
 */
void APlanetVegetation::ResetTrees() {
	for(const std::pair<const FPlantKey, FPreparedTree> &Entry : State->PublishedTrees) {
		if(UCapsuleComponent *const Collision = Entry.second.Collision.Get()) Collision->DestroyComponent();
	}
	State->PublishedTrees.clear();
	State->PendingTrees.clear();
	State->TreeQuery.reset();
	State->TreeCenter = FVector::ZeroVector;
	PublishedTreeCenterMeters = FVector::ZeroVector;
	PublishedTreeRadiusMeters = 0;
	State->TreePhase = -1;
	TreeInstanceCount = 0;
	TreePublishedRevision = 0;
	if(Trees->GetInstanceCount()) Trees->ClearInstances();
	if(TreesVariant->GetInstanceCount()) TreesVariant->ClearInstances();
	if(Conifers->GetInstanceCount()) Conifers->ClearInstances();
	// 終了
	return;
}

/**
 * 少数の樹木候補を小分けで評価し，広葉樹の適性と乾いた地表を満たす根だけを準備する
 */
void APlanetVegetation::UpdateTrees() {
	// 素材又は配置定義が未準備の場合の終了
	if(
		!State->TreeScatter || !Trees->GetStaticMesh() || !TreesVariant->GetStaticMesh() || !Conifers->GetStaticMesh() ||
		!TreeLeafMaterial || !ConiferNeedleMaterial || !TreeBarkMaterial
	) return;
	const PlanetCore::GeologySettings &Settings = State->Settings;
	const FVector Body = State->PlanetTransform.InverseTransformPosition(Explorer->GetActorLocation()) / 100;
	const FVector Center = Body.GetSafeNormal() * Settings.RadiusMeters;
	const double Start = FPlatformTime::Seconds();
	const double Radius = State->FoliageRadiusMeters > 0 ? FMath::Max(20.0, State->FoliageRadiusMeters - 4) : 20.0;
	if(
		!State->TreeQuery &&
		(!TreePublishedRevision || FVector::Dist(Center, State->TreeCenter) > 2 || Radius != PublishedTreeRadiusMeters)
	) {
		const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector({ Center.X, Center.Y, Center.Z });
		State->TreeQuery = State->TreeScatter->Query(Direction, Radius, 10000);
		State->RequestedTreeCenter = Center;
		State->RequestedTreeRadiusMeters = Radius;
		State->NextTree = 0;
		State->PendingTrees.clear();
	}
	// 完了した候補を保持出来ない場合の終了
	if(!State->TreeQuery) return;
	while(State->NextTree < State->TreeQuery->Candidates.size()) {
		const PlanetCore::ScatterCandidate &Candidate = State->TreeQuery->Candidates[State->NextTree++];
		const FPlantKey Key { Candidate.Cell.Face, Candidate.Cell.U, Candidate.Cell.V };
		FPreparedTree Tree;
		if(
			const std::map<FPlantKey, FPreparedTree>::const_iterator Found = State->PublishedTrees.find(Key);
			Found != State->PublishedTrees.end()
		) Tree = Found->second;
		else {
			const PlanetCore::Vec3 D = Candidate.Direction.Vector();
			const FVector Direction(D.X, D.Y, D.Z);
			const std::optional<PlanetCore::RiverSurfaceSample> Evaluated = SampleSurface(Candidate.Direction);
			// 無効な水域又は地形では此の候補を公開せず，次の更新へ進む
			if(!Evaluated) break;
			const PlanetCore::RiverSurfaceSample &Sample = *Evaluated;
			const double Elevation = Sample.Terrain.ElevationMeters;
			const double Slope = Sample.Terrain.TangentialGradientPerRadian.Length() / (Settings.RadiusMeters + Elevation);
			if(
				Elevation > Settings.SeaLevelMeters + 0.2 && Slope < 0.6 &&
				(!Sample.ChannelCoverage || Elevation > Sample.WaterElevationMeters + 0.2)
			) {
				const PlanetCore::SurfaceMaterialWeights Weights =
				PlanetCore::EvaluateSurfaceMaterial(Candidate.Direction, Settings.RadiusMeters, Settings.SeaLevelMeters, Sample);
				Tree.Site = { PlanetCore::SurfaceDomain::Land, Weights.Rock, Weights.Sand, Slope };
				Tree.Profile = SampleBiome(Direction);
				if(Tree.Profile) {
					const std::optional<PlanetCore::BiomeHabitat> Habitat = Tree.Profile->Habitat(Tree.Site);
					const std::optional<PlanetCore::TreeKind> Kind =
					Habitat ? PlanetCore::SelectTreeKind(*Habitat, Candidate.Selection, Candidate.RotationTurns) : std::nullopt;
					if(!Kind) Tree.Profile.reset();
					else {
						Tree.bConifer = *Kind == PlanetCore::TreeKind::Conifer;
						Tree.bVariant = !Tree.bConifer && Candidate.RotationTurns >= 0.5;
					}
				}
				if(Tree.Profile) {
					// 広葉樹の高さと枝張りを独立に揺らし，同じ樹形の反復を弱める
					const double Width = 0.78 + 0.44 * Candidate.RotationTurns, Height = 1.16 - 0.32 * Candidate.RotationTurns;
					Tree.Scale =
					Tree.bConifer ? FVector(Candidate.Size) : FVector(Candidate.Size * Width, Candidate.Size * Width, Candidate.Size * Height);
					// 根元を土中へ１０ cm 埋め，傾斜面でも根の切断面を露出させない
					Tree.PositionMeters = Direction * (Settings.RadiusMeters + Elevation - 0.1 * Tree.Scale.Z);
					Tree.Rotation =
					FQuat::FindBetweenNormals(FVector::UpVector, Direction) * FQuat(FVector::UpVector, Candidate.RotationTurns * 2 * UE_DOUBLE_PI);
				}
			}
		}
		State->PendingTrees.emplace(Key, std::move(Tree));
		if(FPlatformTime::Seconds() - Start >= 0.0005) break;
	}
	LastTreeSliceMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
	if(State->NextTree == State->TreeQuery->Candidates.size()) PublishTrees();
	// 終了
	return;
}

/**
 * 評価済み樹木を転送し，同じ木の幹の衝突部品は破棄せず再利用する
 */
void APlanetVegetation::PublishTrees() {
	const double Start = FPlatformTime::Seconds();
	for(const std::pair<const FPlantKey, FPreparedTree> &Entry : State->PublishedTrees) {
		if(!State->PendingTrees.contains(Entry.first)) if(UCapsuleComponent *const Collision = Entry.second.Collision.Get()) {
			Collision->DestroyComponent();
		}
	}
	State->PublishedTrees = std::move(State->PendingTrees);
	State->PendingTrees.clear();
	const FVector Anchor = State->RequestedTreeCenter;
	const FTransform Origin(State->PlanetTransform.GetRotation(), State->PlanetTransform.TransformPosition(Anchor * 100));
	for(UInstancedStaticMeshComponent *const Component : { Trees.Get(), TreesVariant.Get(), Conifers.Get() }) {
		Component->ClearInstances();
		Component->SetWorldTransform(Origin);
	}
	TArray<FTransform> BroadleafTransforms, VariantTransforms, ConiferTransforms;
	for(std::pair<const FPlantKey, FPreparedTree> &Entry : State->PublishedTrees) {
		FPreparedTree &Tree = Entry.second;
		if(!Tree.Profile) continue;
		TArray<FTransform> &Transforms = Tree.bConifer ? ConiferTransforms : Tree.bVariant ? VariantTransforms : BroadleafTransforms;
		Tree.Instance = Transforms.Num();
		Transforms.Emplace(Tree.Rotation, (Tree.PositionMeters - Anchor) * 100, Tree.Scale);
		const double SurfaceDistance =
		FVector::Distance(Tree.PositionMeters.GetSafeNormal(), Anchor.GetSafeNormal()) * State->Settings.RadiusMeters;
		if(SurfaceDistance > 24) {
			if(UCapsuleComponent *const Collision = Tree.Collision.Get()) Collision->DestroyComponent();
			Tree.Collision = nullptr;
			continue;
		}
		if(!Tree.Collision.IsValid()) {
			UCapsuleComponent *const Collision = NewObject<UCapsuleComponent>(this, NAME_None, RF_Transient);
			if(Collision) {
				AddInstanceComponent(Collision);
				Collision->SetupAttachment(GetRootComponent());
				Collision->SetMobility(EComponentMobility::Movable);
				Collision->SetCanEverAffectNavigation(false);
				Collision->CanCharacterStepUpOn = ECB_No;
				Collision->SetCollisionProfileName(TEXT("BlockAll"));
				const double HalfHeight = Tree.bConifer ? 325 : 255;
				const FQuat Rotation = State->PlanetTransform.GetRotation() * Tree.Rotation;
				const FVector TrunkDirection = Tree.bConifer ? FVector::UpVector : FVector(0.05, -0.035, 1).GetSafeNormal();
				const FVector ScaledTrunkDirection = TrunkDirection * Tree.Scale;
				Collision->SetCapsuleSize(
					(Tree.bConifer ? 16 : 13) * FMath::Max(Tree.Scale.X, Tree.Scale.Y),
					HalfHeight * ScaledTrunkDirection.Length()
				);
				const FVector Base = State->PlanetTransform.TransformPosition(Tree.PositionMeters * 100);
				Collision->SetWorldLocation(Base + Rotation.RotateVector(ScaledTrunkDirection * HalfHeight));
				Collision->SetWorldRotation(Rotation * FQuat::FindBetweenNormals(FVector::UpVector, ScaledTrunkDirection.GetSafeNormal()));
				Collision->RegisterComponent();
				Tree.Collision = Collision;
			}
		}
	}
	Trees->AddInstances(BroadleafTransforms, false, false, false);
	TreesVariant->AddInstances(VariantTransforms, false, false, false);
	Conifers->AddInstances(ConiferTransforms, false, false, false);
	TreeInstanceCount = BroadleafTransforms.Num() + VariantTransforms.Num() + ConiferTransforms.Num();
	State->TreeCenter = State->RequestedTreeCenter;
	PublishedTreeCenterMeters = Anchor;
	PublishedTreeRadiusMeters = State->RequestedTreeRadiusMeters;
	State->TreeQuery.reset();
	State->TreePhase = -1;
	++TreePublishedRevision;
	LastTreeUploadMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
	// 終了
	return;
}

/**
 * 幹と葉の可視範囲を精密な近景へ制限し，葉の色だけを地点の季節応答へ対応させる
 */
void APlanetVegetation::UpdateTreeAppearance() {
	// 未準備の場合の終了
	if(!TreeBarkMaterial || !TreeLeafMaterial || !ConiferNeedleMaterial || !TreePublishedRevision) return;
	if(const APlayerCameraManager *const Camera = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0)) {
		const FVector Focus = State->PlanetTransform.TransformPosition(State->SurfaceObserverMeters * 100);
		const FVector Near = Focus - Camera->GetCameraLocation(), Body = Explorer->GetActorLocation() - Camera->GetCameraLocation();
		for(
			UMaterialInstanceDynamic *const Material :
			{ TreeBarkMaterial.Get(), TreeLeafMaterial.Get(), ConiferNeedleMaterial.Get() }
		) {
			Material->SetVectorParameterValue(TEXT("NearCenterRelative"), FLinearColor(Near.X, Near.Y, Near.Z, 0));
			Material->SetVectorParameterValue(TEXT("BodyCenterRelative"), FLinearColor(Body.X, Body.Y, Body.Z, 0));
			const double Radius = State->FoliageRadiusMeters > 0 ? State->FoliageRadiusMeters - 4 : State->SurfaceRadiusMeters - 1;
			Material->SetScalarParameterValue(TEXT("NearRadius"), FMath::Max(0.0, Radius) * 100);
			Material->SetScalarParameterValue(TEXT("ClipEnabled"), 1);
		}
	}
	// 十分小さな位相変化の場合は，同じ色を再転送しない終了
	if(State->TreePhase >= 0 && std::abs(State->SeasonalPhase - State->TreePhase) < 0.00001) return;
	for(const std::pair<const FPlantKey, FPreparedTree> &Entry : State->PublishedTrees) {
		const FPreparedTree &Tree = Entry.second;
		if(!Tree.Profile || Tree.Instance == INDEX_NONE) continue;
		if(const std::optional<PlanetCore::SeasonalAppearance> Appearance = Tree.Profile->Appearance(State->SeasonalPhase, Tree.Site)) {
			const double Dry = 1 - Appearance->LeafGreenness, Autumn = Appearance->AutumnTint;
			const float Color[] {
				static_cast<float>(Tree.bConifer ? 1 + 0.08 * Dry : 1 + 0.5 * Dry + 0.6 * Autumn),
				static_cast<float>(Tree.bConifer ? 1 - 0.06 * Dry : 1 - 0.3 * Autumn),
				static_cast<float>(Tree.bConifer ? 1 - 0.12 * Dry : 1 - 0.5 * Dry - 0.4 * Autumn),
				static_cast<float>(Appearance->Snow)
			};
			UInstancedStaticMeshComponent *const Component =
			Tree.bConifer ? Conifers.Get() : Tree.bVariant ? TreesVariant.Get() : Trees.Get();
			Component->SetCustomData(Tree.Instance, MakeArrayView(Color), false);
		}
	}
	Trees->MarkRenderStateDirty();
	TreesVariant->MarkRenderStateDirty();
	Conifers->MarkRenderStateDirty();
	State->TreePhase = State->SeasonalPhase;
	// 終了
	return;
}
