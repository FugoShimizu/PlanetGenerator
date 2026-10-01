#include "PlanetExplorerAnimation.h"
#include "PlanetExplorerCharacter.h"
#include "Animation/BlendSpace.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Misc/AutomationTest.h"
#include <array>
#include <memory>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetExplorerAnimationContracts, "PlanetGenerator.Character.Animation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実際の骨姿勢を評価し，任意方向の身体で移動・上昇・落下・着地を検証する
 * @param Parameters 未使用の自動検査引数
 * @return 全ての姿勢と物理入力の独立条件を満たした場合に真
 */
bool FPlanetExplorerAnimationContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	UWorld *World = nullptr;
	if(GEngine) {
		for(const FWorldContext &Context : GEngine->GetWorldContexts()) {
			if(Context.WorldType == EWorldType::Editor) World = Context.World();
		}
	}
	// 実際の部品を生成出来ない場合の返戻
	if(!TestNotNull(TEXT("姿勢検査の世界"), World)) return false;
	const std::array<FQuat, 3> Rotations {
		FQuat::Identity,
		FQuat(FVector(1, 2, 3).GetSafeNormal(), 1.7),
		FQuat(FVector::RightVector, PI)
	};
	unsigned Frames = 0;
	double MaximumFootTravel = 0, MaximumRotationError = 0;
	for(const FQuat Rotation : Rotations) {
		FActorSpawnParameters Spawn;
		Spawn.ObjectFlags |= RF_Transient;
		Spawn.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		const auto Destroy = [](APlanetExplorerCharacter *const Body) -> void {
			if(IsValid(Body)) Body->Destroy();
			// 所有する一時アクターの解放の終了
			return;
		};
		const std::unique_ptr<APlanetExplorerCharacter, decltype(Destroy)> Body(
			World->SpawnActor<APlanetExplorerCharacter>(FVector::ZeroVector, Rotation.Rotator(), Spawn),
			Destroy
		);
		// 身体が無い場合の返戻
		if(!TestNotNull(TEXT("C++ の身体"), Body.get())) return false;
		Body->SetActorTickEnabled(false);
		USkeletalMeshComponent *const Mesh = Body->GetMesh();
		Mesh->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;
		Mesh->SetComponentTickEnabled(false);
		Mesh->InitAnim(true);
		UPlanetExplorerAnimation *const Animation = Cast<UPlanetExplorerAnimation>(Mesh->GetAnimInstance());
		// 既存の固定姿勢へ黙って戻らない返戻
		if(!TestNotNull(TEXT("ネイティブの姿勢評価"), Animation)) return false;
		const UBlendSpace *const Asset = Cast<UBlendSpace>(Animation->GetAnimationAsset());
		// 動作資産が無い状態を検証済にしない返戻
		if(!TestNotNull(TEXT("既存の歩行混合資産"), Asset)) return false;
		TestEqual(TEXT("移動方向の軸"), Asset->GetBlendParameter(0).DisplayName, FString(TEXT("Direction")));
		TestEqual(TEXT("速度の軸"), Asset->GetBlendParameter(1).DisplayName, FString(TEXT("Speed")));
		TestEqual(TEXT("資産の最大速度"), Asset->GetBlendParameter(1).Max, 600.0F);
		const int32 Foot = Mesh->GetBoneIndex(TEXT("foot_l"));
		// 実在しない骨を原点として比較しない返戻
		if(!TestTrue(TEXT("左足の骨が存在"), Foot != INDEX_NONE)) return false;
		UCharacterMovementComponent *const Movement = Body->GetCharacterMovement();
		Movement->SetMovementMode(MOVE_Walking);
		Movement->Velocity = FVector::ZeroVector;
		FVector PreviousFoot = FVector::ZeroVector;
		bool HasFoot = false;
		const auto Step = [&](const unsigned Count) -> void {
			for(unsigned I = 0; I < Count; ++I) {
				Mesh->TickAnimation(1.0F / 60, false);
				Mesh->RefreshBoneTransforms();
				const TArray<FTransform> Pose = Mesh->GetBoneSpaceTransforms();
				if(!TestTrue(TEXT("実際の骨配列を評価"), Pose.IsValidIndex(Foot))) continue;
				for(const FTransform &Bone : Pose) {
					TestFalse(TEXT("全ての骨が有限"), Bone.ContainsNaN());
					MaximumRotationError = FMath::Max(MaximumRotationError, FMath::Abs(Bone.GetRotation().SizeSquared() - 1));
				}
				const FVector Here = Mesh->GetSocketTransform(TEXT("foot_l"), RTS_Component).GetLocation();
				if(HasFoot) MaximumFootTravel = FMath::Max(MaximumFootTravel, FVector::Distance(Here, PreviousFoot));
				PreviousFoot = Here;
				HasFoot = true;
				++Frames;
			}
			// 姿勢と骨の読戻しの終了
			return;
		};
		Step(5);
		TestEqual(TEXT("静止時は地上姿勢"), Animation->MotionPhase, EPlanetLocomotionPhase::Ground);
		Movement->Velocity = Rotation.RotateVector(FVector(450, 0, 0));
		Step(60);
		TestTrue(TEXT("回転後も接線速度を保持"), FMath::Abs(Animation->TangentialSpeedCentimetersPerSecond - 450) < 1E-6);
		TestTrue(TEXT("回転後も前進方向を保持"), FMath::Abs(Animation->DirectionDegrees) < 1E-6);
		TestTrue(TEXT("地上姿勢の再生時刻が進む"), Animation->GetCurrentTime() > 0);
		Movement->Velocity = Rotation.RotateVector(FVector(-450, 0, 0));
		Step(5);
		TestTrue(TEXT("後退の姿勢方向"), FMath::Abs(FMath::Abs(Animation->DirectionDegrees) - 180) < 1E-6);
		Movement->Velocity = Rotation.RotateVector(FVector(0, 900, 0));
		Step(5);
		TestTrue(TEXT("横方向の走行姿勢"), FMath::Abs(Animation->DirectionDegrees - 90) < 1E-6);
		TestEqual(TEXT("走行時に足の周期が追従"), Animation->GetPlayRate(), 1.5F);
		Movement->SetMovementMode(MOVE_Falling);
		Movement->Velocity = Rotation.RotateVector(FVector(200, 0, 400));
		Step(12);
		TestEqual(TEXT("局所重力へ逆らう上昇姿勢"), Animation->MotionPhase, EPlanetLocomotionPhase::Rising);
		TestTrue(TEXT("上昇速度を接線速度に混ぜない"), FMath::Abs(Animation->TangentialSpeedCentimetersPerSecond - 200) < 1E-6);
		Movement->Velocity = Rotation.RotateVector(FVector(200, 0, -200));
		Step(12);
		TestEqual(TEXT("惑星裏側でも下降姿勢"), Animation->MotionPhase, EPlanetLocomotionPhase::Falling);
		Movement->SetMovementMode(MOVE_Walking);
		Movement->Velocity = FVector::ZeroVector;
		Step(5);
		TestEqual(TEXT("実際の着地で姿勢遷移"), Animation->MotionPhase, EPlanetLocomotionPhase::Landing);
		Step(30);
		TestEqual(TEXT("着地後は静止へ復帰"), Animation->MotionPhase, EPlanetLocomotionPhase::Ground);
		TestTrue(TEXT("姿勢評価が身体を移動しない"), Body->GetActorLocation().IsZero());
	}
	TestTrue(TEXT("固定姿勢ではなく足の骨が移動"), MaximumFootTravel > 1);
	TestTrue(TEXT("合成後も骨の回転が正規化"), MaximumRotationError < 1E-5);
	AddInfo(
		FString::Printf(
			TEXT("Native animation: rotations=3 frames=%u maxFootStepCm=%.12g maxRotationError=%.12g"),
			Frames,
			MaximumFootTravel,
			MaximumRotationError
		)
	);
	// 実資産・実骨と三方向の状態遷移を検査した返戻
	return true;
}

#endif
