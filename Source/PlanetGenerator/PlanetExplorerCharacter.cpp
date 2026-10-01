#include "PlanetExplorerCharacter.h"
#include "PlanetExplorerAnimation.h"
#include "PlanetCelestialComponent.h"
#include "PlanetCharacterMovement.h"
#include "PlanetGroundPatch.h"
#include "PlanetTerrain.h"
#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/InputComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputCoreTypes.h"
#include "UObject/ConstructorHelpers.h"

/**
 * 押下中のキーを前後・左右の移動量へ変換する
 */
void APlanetExplorerCharacter::UpdateKeyboard() {
	InputForward = static_cast<int>(IsForwardPressed) - static_cast<int>(IsBackwardPressed);
	InputRight = static_cast<int>(IsRightPressed) - static_cast<int>(IsLeftPressed);
	// 終了
	return;
}

/**
 * 視線と俯瞰距離を三つのカメラへ反映する
 */
void APlanetExplorerCharacter::UpdateCameras() {
	FirstCamera->SetRelativeRotation(FRotator(ViewPitch, 0, 0));
	ThirdArm->SetRelativeRotation(FRotator(ViewPitch, 0, 0));
	const FVector Offset = FRotator(OverviewPitch, OverviewYaw, 0).Vector() * OverviewDistance * 100;
	OverviewCamera->SetRelativeLocationAndRotation(Offset, (-Offset).Rotation());
	// 終了
	return;
}

/**
 * 太陽高度に応じて三視点の露出を連続補正する
 */
void APlanetExplorerCharacter::UpdateSceneExposure() {
	// 薄明だけを最大１段抑え，日中は計測済の自動露出を更に減光しない
	const UPlanetCelestialComponent *const Clock = ClockSource.Get();
	// 未接続又は停止中は最後の有効な露出を保持した終了
	if(!Clock || !Clock->bCelestialClockActive) return;
	const double Altitude = Clock->SunAltitudeDegrees;
	const float Bias =
	-static_cast<float>(FMath::Clamp((Altitude + 20.0) / 14.0, 0.0, 1.0) * (1 - FMath::Clamp((Altitude + 6.0) / 26.0, 0.0, 1.0)));
	for(UCameraComponent *const Camera : { FirstCamera.Get(), ThirdCamera.Get(), OverviewCamera.Get() }) {
		FPostProcessSettings &Settings = Camera->PostProcessSettings;
		Settings.bOverride_AutoExposureBias = true;
		if(FMath::Abs(Settings.AutoExposureBias - Bias) > 0.01F) Settings.AutoExposureBias = Bias;
	}
	// 終了
	return;
}

/**
 * 従来地形と共有河床の何れでも，同じ天体時計の露出を身体へ接続する
 * @param Clock 此の身体の地表を照らす時計；空なら接続を解除
 */
void APlanetExplorerCharacter::SetCelestialClock(UPlanetCelestialComponent *const Clock) {
	ClockSource = Clock;
	UpdateSceneExposure();
	// 終了
	return;
}

/**
 * 検証と入力経路から接線方向の移動量を受け取る
 * @param Forward 前後の入力
 * @param Right 左右の入力
 * @param bRun 走行する場合は真
 */
void APlanetExplorerCharacter::SetMoveInput(const double Forward, const double Right, const bool bRun) {
	// 移動入力が有限でない場合の終了
	if(!FMath::IsFinite(Forward) || !FMath::IsFinite(Right)) return;
	InputForward = FMath::Clamp(Forward, -1.0, 1.0);
	InputRight = FMath::Clamp(Right, -1.0, 1.0);
	IsRunning = bRun;
	// 終了
	return;
}

/**
 * 同じ身体を保って一人称・三人称・俯瞰を切り替える
 * @param Mode 切り替え先の視点
 */
void APlanetExplorerCharacter::SetViewMode(const EPlanetViewMode Mode) {
	// 無効な視点指定の場合の終了
	if(Mode != EPlanetViewMode::FirstPerson && Mode != EPlanetViewMode::ThirdPerson && Mode != EPlanetViewMode::Overview) return;
	ViewMode = Mode;
	FirstCamera->SetActive(Mode == EPlanetViewMode::FirstPerson);
	ThirdCamera->SetActive(Mode == EPlanetViewMode::ThirdPerson);
	OverviewCamera->SetActive(Mode == EPlanetViewMode::Overview);
	GetMesh()->SetOwnerNoSee(Mode == EPlanetViewMode::FirstPerson);
	UpdateCameras();
	// 終了
	return;
}

/**
 * 身体又は俯瞰カメラを回転させる
 * @param YawDegrees 水平方向の回転角度
 * @param PitchDegrees 垂直方向の回転角度
 */
void APlanetExplorerCharacter::Look(const double YawDegrees, const double PitchDegrees) {
	// 回転入力が有限でない場合の終了
	if(!FMath::IsFinite(YawDegrees) || !FMath::IsFinite(PitchDegrees)) return;
	if(ViewMode == EPlanetViewMode::Overview) {
		OverviewYaw = FMath::UnwindDegrees(OverviewYaw + YawDegrees);
		OverviewPitch = FMath::Clamp(OverviewPitch + PitchDegrees, 5.0, 89.0);
	} else {
		SetActorRotation((FQuat(GetActorUpVector(), FMath::DegreesToRadians(YawDegrees)) * GetActorQuat()).GetNormalized());
		ViewPitch = FMath::Clamp(ViewPitch + PitchDegrees, -85.0, 85.0);
	}
	UpdateCameras();
	// 終了
	return;
}

/**
 * 俯瞰時の観測距離を変更する
 * @param Value ズーム入力
 */
void APlanetExplorerCharacter::Zoom(const float Value) {
	if(ViewMode == EPlanetViewMode::Overview && FMath::IsFinite(Value)) {
		// 表示中の身体と同じ公開済半径に依る俯瞰範囲
		FTransform PlanetTransform;
		double Radius = 0;
		const double MaximumDistance = IsValid(Ground) && Ground->GetPublishedPlanet(PlanetTransform, Radius) ? Radius * 3 : 1000000.0;
		OverviewDistance = FMath::Clamp(OverviewDistance * FMath::Pow(0.85, Value), 20.0, MaximumDistance);
		UpdateCameras();
	}
	// 終了
	return;
}

/**
 * 前進キーの押下を反映する
 */
void APlanetExplorerCharacter::ForwardDown() {
	IsForwardPressed = true;
	UpdateKeyboard();
	// 終了
	return;
}

/**
 * 前進キーの解放を反映する
 */
void APlanetExplorerCharacter::ForwardUp() {
	IsForwardPressed = false;
	UpdateKeyboard();
	// 終了
	return;
}

/**
 * 後退キーの押下を反映する
 */
void APlanetExplorerCharacter::BackwardDown() {
	IsBackwardPressed = true;
	UpdateKeyboard();
	// 終了
	return;
}

/**
 * 後退キーの解放を反映する
 */
void APlanetExplorerCharacter::BackwardUp() {
	IsBackwardPressed = false;
	UpdateKeyboard();
	// 終了
	return;
}

/**
 * 左移動キーの押下を反映する
 */
void APlanetExplorerCharacter::LeftDown() {
	IsLeftPressed = true;
	UpdateKeyboard();
	// 終了
	return;
}

/**
 * 左移動キーの解放を反映する
 */
void APlanetExplorerCharacter::LeftUp() {
	IsLeftPressed = false;
	UpdateKeyboard();
	// 終了
	return;
}

/**
 * 右移動キーの押下を反映する
 */
void APlanetExplorerCharacter::RightDown() {
	IsRightPressed = true;
	UpdateKeyboard();
	// 終了
	return;
}

/**
 * 右移動キーの解放を反映する
 */
void APlanetExplorerCharacter::RightUp() {
	IsRightPressed = false;
	UpdateKeyboard();
	// 終了
	return;
}

/**
 * 走行キーの押下を反映する
 */
void APlanetExplorerCharacter::RunDown() {
	IsRunning = true;
	// 終了
	return;
}

/**
 * 走行キーの解放を反映する
 */
void APlanetExplorerCharacter::RunUp() {
	IsRunning = false;
	// 終了
	return;
}

/**
 * 跳躍キーの押下を移動成分へ渡す
 */
void APlanetExplorerCharacter::JumpDown() {
	Jump();
	// 終了
	return;
}

/**
 * 跳躍キーの解放を移動成分へ渡す
 */
void APlanetExplorerCharacter::JumpUp() {
	StopJumping();
	// 終了
	return;
}

/**
 * 一人称へ切替える
 */
void APlanetExplorerCharacter::FirstView() {
	SetViewMode(EPlanetViewMode::FirstPerson);
	// 終了
	return;
}

/**
 * 三人称へ切替える
 */
void APlanetExplorerCharacter::ThirdView() {
	SetViewMode(EPlanetViewMode::ThirdPerson);
	// 終了
	return;
}

/**
 * 俯瞰へ切替える
 */
void APlanetExplorerCharacter::OverviewView() {
	SetViewMode(EPlanetViewMode::Overview);
	// 終了
	return;
}

/**
 * マウスの軸入力を視点へ渡す
 * @param Value 有限な軸入力；不正値は視点操作側で拒否
 */
void APlanetExplorerCharacter::MouseX(const float Value) {
	Look(0.15 * Value, 0);
	// 終了
	return;
}

/**
 * マウスの軸入力を視点へ渡す
 * @param Value 有限な軸入力；不正値は視点操作側で拒否
 */
void APlanetExplorerCharacter::MouseY(const float Value) {
	Look(0, 0.15 * Value);
	// 終了
	return;
}

/**
 * 移動・跳躍・視点変更のキーを C++ 入力へ結び付ける
 * @param Input 入力を受け取る UE 部品
 */
void APlanetExplorerCharacter::SetupPlayerInputComponent(UInputComponent *const Input) {
	Super::SetupPlayerInputComponent(Input);
	Input->BindKey(EKeys::W, IE_Pressed, this, &APlanetExplorerCharacter::ForwardDown);
	Input->BindKey(EKeys::W, IE_Released, this, &APlanetExplorerCharacter::ForwardUp);
	Input->BindKey(EKeys::S, IE_Pressed, this, &APlanetExplorerCharacter::BackwardDown);
	Input->BindKey(EKeys::S, IE_Released, this, &APlanetExplorerCharacter::BackwardUp);
	Input->BindKey(EKeys::A, IE_Pressed, this, &APlanetExplorerCharacter::LeftDown);
	Input->BindKey(EKeys::A, IE_Released, this, &APlanetExplorerCharacter::LeftUp);
	Input->BindKey(EKeys::D, IE_Pressed, this, &APlanetExplorerCharacter::RightDown);
	Input->BindKey(EKeys::D, IE_Released, this, &APlanetExplorerCharacter::RightUp);
	Input->BindKey(EKeys::LeftShift, IE_Pressed, this, &APlanetExplorerCharacter::RunDown);
	Input->BindKey(EKeys::LeftShift, IE_Released, this, &APlanetExplorerCharacter::RunUp);
	Input->BindKey(EKeys::SpaceBar, IE_Pressed, this, &APlanetExplorerCharacter::JumpDown);
	Input->BindKey(EKeys::SpaceBar, IE_Released, this, &APlanetExplorerCharacter::JumpUp);
	Input->BindKey(EKeys::One, IE_Pressed, this, &APlanetExplorerCharacter::FirstView);
	Input->BindKey(EKeys::Two, IE_Pressed, this, &APlanetExplorerCharacter::ThirdView);
	Input->BindKey(EKeys::Three, IE_Pressed, this, &APlanetExplorerCharacter::OverviewView);
	Input->BindAxisKey(EKeys::MouseX, this, &APlanetExplorerCharacter::MouseX);
	Input->BindAxisKey(EKeys::MouseY, this, &APlanetExplorerCharacter::MouseY);
	Input->BindAxisKey(EKeys::MouseWheelAxis, this, &APlanetExplorerCharacter::Zoom);
	// 終了
	return;
}

/**
 * 公開済みの地形と足場を照合し，陸上に探索身体を生成する
 * @param PreparedGround 公開済みの足元衝突
 * @param LocalDirection 惑星の局所座標で示す着地点の方向
 * @param LocalForward 惑星の局所接線で示す初期視線
 * @return 着地条件を満たす身体，又は空
 */
APlanetExplorerCharacter *APlanetExplorerCharacter::SpawnOnPreparedGround(
	APlanetGroundPatch *const PreparedGround,
	const FVector LocalDirection,
	const FVector LocalForward
) {
	const bool HasInput =
	IsValid(PreparedGround) && !LocalDirection.ContainsNaN() && !LocalDirection.IsNearlyZero() && !LocalForward.ContainsNaN() &&
	!LocalForward.IsNearlyZero();
	// 着地点又は足場が無効な場合の返戻
	if(!HasInput) return nullptr;
	APlanetTerrain *const Planet = PreparedGround->Terrain.Get();
	UWorld *const World = PreparedGround->GetWorld();
	// 実行用 World が無い場合の返戻
	if(!World || !World->IsGameWorld()) return nullptr;
	FTransform PlanetTransform;
	double Radius = 0, Height = 0;
	const FVector D = LocalDirection.GetSafeNormal();
	const bool HasSurface =
	PreparedGround->GetPublishedPlanet(PlanetTransform, Radius) && PreparedGround->EvaluatePublishedElevation(D, Height) &&
	Height > 0;
	// 公開済の同じ定義と陸上の着地標高が無い場合の返戻
	if(!HasSurface) return nullptr;
	const FVector Point = PlanetTransform.TransformPosition(D * ((Radius + Height) * 100));
	const FVector Up = PlanetTransform.TransformVectorNoScale(D).GetSafeNormal();
	FVector Hit, Normal;
	if(!PreparedGround->TraceGround(Point + Up * 100, Point - Up * 100, Hit, Normal) || FVector::Distance(Hit, Point) > 2) {
		// 足場と地形の位置が一致しない場合の返戻
		return nullptr;
	}
	const double SlopeCos = FVector::DotProduct(Up, Normal);
	// 着地面が急斜面の場合の返戻
	if(SlopeCos < FMath::Cos(FMath::DegreesToRadians(44.0))) return nullptr;
	const FVector Position = Hit + Up * (62 + 34 / SlopeCos + 5);
	double Margin = 0;
	// 身体を覆う足場が無い場合の返戻
	if(!PreparedGround->QueryCoverage(Position, 1.34, Margin) || Margin <= 0) return nullptr;
	const FVector Tangent = FVector::VectorPlaneProject(LocalForward, D).GetSafeNormal();
	// 接線方向を持たない視線への返戻
	if(Tangent.IsNearlyZero()) return nullptr;
	const FVector Forward = PlanetTransform.TransformVectorNoScale(Tangent).GetSafeNormal();
	const FQuat Rotation = FRotationMatrix::MakeFromXZ(Forward, Up).ToQuat();
	// 着地点が塞がれている場合の返戻
	if(World->OverlapBlockingTestByChannel(Position, Rotation, ECC_Pawn, FCollisionShape::MakeCapsule(34, 96))) return nullptr;
	const FTransform Transform(Rotation, Position);
	APlanetExplorerCharacter *const Character = World->SpawnActorDeferred<APlanetExplorerCharacter>(
		StaticClass(),
		Transform,
		nullptr,
		nullptr,
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn
	);
	// 身体を生成出来ない場合の返戻
	if(!Character) return nullptr;
	Character->SetFlags(RF_Transient);
	Character->Ground = PreparedGround;
	Character->Terrain = Planet;
	Character->GetCharacterMovement()->SetGravityDirection(-Up);
	Character->FinishSpawning(Transform);
	// 検査済みの探索身体の返戻
	return Character;
}

/**
 * 球面上の移動入力と公開済み足場に対応する先読みを処理する
 * @param DeltaSeconds 前回更新からの秒数
 */
void APlanetExplorerCharacter::Tick(const float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	UpdateSceneExposure();
	TraversalSeconds += DeltaSeconds;
	// 足場が無い場合の終了
	if(!IsValid(Ground)) return;
	FTransform PlanetTransform;
	double Radius = 0;
	if(!Ground->GetPublishedPlanet(PlanetTransform, Radius)) {
		// 元の定義が替わった場合も，現在の正規の場へ再要求
		++CoverageMissFrames;
		if(!Ground->bBusy && Ground->RequestAtWorldPosition(GetActorLocation())) ++GroundRefreshRequests;
		// 新しい定義の公開を待つ場合の終了
		return;
	}
	const FVector Up = (GetActorLocation() - PlanetTransform.GetLocation()).GetSafeNormal();
	// 重力方向を求められない場合の終了
	if(Up.IsNearlyZero()) return;
	const FVector Forward = FVector::VectorPlaneProject(GetActorForwardVector(), Up).GetSafeNormal();
	const FVector Right = FVector::CrossProduct(Up, Forward).GetSafeNormal();
	const double Speed = IsRunning ? RunSpeedMetersPerSecond : WalkSpeedMetersPerSecond;
	if(FMath::IsFinite(Speed) && Speed > 0 && !(Speed > 20)) GetCharacterMovement()->MaxWalkSpeed = static_cast<float>(Speed * 100);
	if(ViewMode != EPlanetViewMode::Overview) {
		const FVector Input = (Forward * InputForward + Right * InputRight).GetClampedToMaxSize(1);
		AddMovementInput(Input, 1, GetController() == nullptr);
	}
	// 足場が現在の場又は身体位置を覆わない場合の再要求
	if(!Ground->QueryCoverage(GetActorLocation(), GetCapsuleComponent()->GetScaledCapsuleRadius() / 100, GroundMarginMeters)) {
		++CoverageMissFrames;
		if(!Ground->bBusy && Ground->RequestAtWorldPosition(GetActorLocation())) ++GroundRefreshRequests;
		// 足元の再準備を待つ場合の終了
		return;
	}
	if(GroundMarginMeters < 0) ++CoverageMissFrames;
	const double ConfiguredHorizon =
	FMath::IsFinite(PreparationHorizonSeconds) ? FMath::Clamp(PreparationHorizonSeconds, 0.1, 2.0) : 0.75;
	// 固定の０．７５秒だけでは起伏の細かい小惑星で準備が間に合わない
	// 前回の所要時間に物理の調理と公開の余裕を加える
	// 実測値による先読みであり，OS のスケジューリング遅延上限を保証しない
	const double MeasuredHorizon = FMath::IsFinite(Ground->LastWorkerMilliseconds) ?
	FMath::Max(0.0, Ground->LastWorkerMilliseconds) * 0.001 + 0.25 :
	ConfiguredHorizon;
	double MaximumSpeed = GetCharacterMovement()->MaxWalkSpeed / 100;
	for(const double Candidate : { WalkSpeedMetersPerSecond, RunSpeedMetersPerSecond }) {
		if(FMath::IsFinite(Candidate) && Candidate > 0 && !(Candidate > 20)) MaximumSpeed = FMath::Max(MaximumSpeed, Candidate);
	}
	// 許容速度で約１．６秒走れる範囲を，同じ公開済半径の上限内で確保
	const double MaximumExtent = FMath::Min(50.0, Radius * 0.05);
	const double DesiredExtent = FMath::Clamp(MaximumSpeed * 1.6, 16.0, MaximumExtent);
	Ground->HalfExtentMeters = FMath::Max(Ground->HalfExtentMeters, DesiredExtent);
	const double Horizon = FMath::Max(FMath::Max(ConfiguredHorizon, MeasuredHorizon), MaximumSpeed * 0.05);
	const double Reserve = FMath::Clamp(MaximumSpeed * Horizon + 2, Ground->HalfExtentMeters * 0.5, Ground->HalfExtentMeters * 0.9);
	if(!Ground->bBusy && GroundMarginMeters < Reserve) {
		const FVector Lead = (GetVelocity() * Horizon).GetClampedToMaxSize(Ground->HalfExtentMeters * 25);
		if(Ground->RequestAtWorldPosition(GetActorLocation() + Lead)) ++GroundRefreshRequests;
	}
	// 終了
	return;
}

/**
 * 足場の更新を身体の更新より先に実行する
 */
void APlanetExplorerCharacter::BeginPlay() {
	Super::BeginPlay();
	// UE の自動有効化後に，選択中のカメラだけを有効にする
	SetViewMode(ViewMode);
	if(IsValid(Terrain)) SetCelestialClock(Terrain->FindComponentByClass<UPlanetCelestialComponent>());
	if(IsValid(Ground)) AddTickPrerequisiteActor(Ground);
	// 終了
	return;
}

/**
 * 球面重力で歩く身体と三視点のカメラを準備する
 * @param ObjectInitializer UE のオブジェクト初期化
 */
APlanetExplorerCharacter::APlanetExplorerCharacter(const FObjectInitializer &ObjectInitializer) : Super(
	ObjectInitializer.SetDefaultSubobjectClass<UPlanetCharacterMovement>(ACharacter::CharacterMovementComponentName)
) {
	// 一定重力の身体と衝突寸法の初期化
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;
	bUseControllerRotationRoll = false;
	GetCapsuleComponent()->InitCapsuleSize(34, 96);
	UCharacterMovementComponent *const Movement = GetCharacterMovement();
	Movement->bRunPhysicsWithNoController = true;
	Movement->bOrientRotationToMovement = false;
	Movement->bUseControllerDesiredRotation = false;
	Movement->bEnablePhysicsInteraction = false;
	Movement->MaxWalkSpeed = 450;
	Movement->JumpZVelocity = 450;
	Movement->AirControl = 0.35F;
	Movement->MaxAcceleration = 2400;
	Movement->BrakingDecelerationWalking = 2400;
	Movement->MaxSimulationTimeStep = 1.0F / 60;
	Movement->MaxSimulationIterations = 24;
	// 同じ身体に追従する三視点の準備
	FirstCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FirstPersonCamera"));
	FirstCamera->SetupAttachment(GetCapsuleComponent());
	FirstCamera->SetRelativeLocation(FVector(0, 0, 64));
	ThirdArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("ThirdPersonArm"));
	ThirdArm->SetupAttachment(GetCapsuleComponent());
	ThirdArm->SetRelativeLocation(FVector(0, 0, 50));
	ThirdArm->TargetArmLength = 350;
	ThirdArm->ProbeSize = 12;
	ThirdArm->bDoCollisionTest = true;
	ThirdCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ThirdPersonCamera"));
	ThirdCamera->SetupAttachment(ThirdArm, USpringArmComponent::SocketName);
	OverviewCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("OverviewCamera"));
	OverviewCamera->SetupAttachment(GetCapsuleComponent());
	// 暗所を昼の様に増幅しない三視点共通の EV100 範囲
	for(UCameraComponent *const Camera : { FirstCamera.Get(), ThirdCamera.Get(), OverviewCamera.Get() }) {
		FPostProcessSettings &Settings = Camera->PostProcessSettings;
		Settings.bOverride_AutoExposureMinBrightness = true;
		Settings.bOverride_AutoExposureMaxBrightness = true;
		Settings.bOverride_AutoExposureBias = true;
		Settings.AutoExposureMinBrightness = 0;
		Settings.AutoExposureMaxBrightness = 16;
		Settings.AutoExposureBias = 0;
	}
	// 既存の身体資産と初期視点の設定
	static const ConstructorHelpers::FObjectFinder<USkeletalMesh> Body(
		TEXT("/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")
	);
	if(Body.Succeeded()) GetMesh()->SetSkeletalMesh(Body.Object);
	GetMesh()->SetAnimInstanceClass(UPlanetExplorerAnimation::StaticClass());
	GetMesh()->SetRelativeLocationAndRotation(FVector(0, 0, -96), FRotator(0, -90, 0));
	GetMesh()->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetViewMode(EPlanetViewMode::FirstPerson);
	// 終了
	return;
}
