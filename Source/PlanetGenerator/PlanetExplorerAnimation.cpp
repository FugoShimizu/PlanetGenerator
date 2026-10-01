#include "PlanetExplorerAnimation.h"
#include "Animation/AnimationPoseData.h"
#include "Animation/AnimSequence.h"
#include "Animation/AnimSingleNodeInstanceProxy.h"
#include "Animation/BlendSpace.h"
#include "AnimationRuntime.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "UObject/ConstructorHelpers.h"
#include <array>
#include <cmath>

/** 標準の歩行混合を保ち，離着地の姿勢だけを滑らかに合成する */
struct FPlanetExplorerAnimationProxy final : FAnimSingleNodeInstanceProxy {
private:

	// 評価期間だけの資産参照と，描画更新間で保持する姿勢状態
	std::array<UAnimSequence *, 3> AirClips {};
	std::array<float, 3> AirTimes {};
	std::array<float, 4> Weights { 1, 0, 0, 0 };
	EPlanetLocomotionPhase Phase = EPlanetLocomotionPhase::Ground;
	float PhaseSeconds = 0;
	bool WasFalling = false;

public:

	/**
	 * 標準の単一資産評価器へ所有インスタンスを接続する
	 * @param Instance 資産と描画部品を所有するインスタンス
	 */
	explicit FPlanetExplorerAnimationProxy(UAnimInstance *const Instance) : FAnimSingleNodeInstanceProxy(Instance) {
		// 終了
		return;
	}

	/**
	 * 並列評価より前に，回収を防いだ資産の参照を取得する
	 * @param Instance 同じ所有インスタンス
	 */
	virtual void InitializeObjects(UAnimInstance *const Instance) override {
		FAnimSingleNodeInstanceProxy::InitializeObjects(Instance);
		const UPlanetExplorerAnimation *const Animation = CastChecked<UPlanetExplorerAnimation>(Instance);
		AirClips = { Animation->Rise, Animation->Fall, Animation->Land };
		// 終了
		return;
	}

	/**
	 * 評価期間の終了後に，外部オブジェクトへの生の参照を解放する
	 */
	virtual void ClearObjects() override {
		AirClips.fill(nullptr);
		FAnimSingleNodeInstanceProxy::ClearObjects();
		// 終了
		return;
	}

	/**
	 * ゲームスレッドで身体の局所速度・実際の離着地と姿勢時計を取得する
	 * @param Instance 物理状態の取得先
	 * @param DeltaSeconds 今回の描画更新の経過時間
	 */
	virtual void PreUpdate(UAnimInstance *const Instance, const float DeltaSeconds) override {
		FAnimSingleNodeInstanceProxy::PreUpdate(Instance, DeltaSeconds);
		UPlanetExplorerAnimation *const Animation = CastChecked<UPlanetExplorerAnimation>(Instance);
		const ACharacter *const Character = Cast<ACharacter>(Animation->TryGetPawnOwner());
		const UCharacterMovementComponent *const Movement = Character ? Character->GetCharacterMovement() : nullptr;
		const FVector LocalVelocity =
		Character ? Character->GetActorQuat().UnrotateVector(Character->GetVelocity()) : FVector::ZeroVector;
		const double Speed = std::hypot(LocalVelocity.X, LocalVelocity.Y);
		const double Direction = Speed > 1 ? FMath::RadiansToDegrees(std::atan2(LocalVelocity.Y, LocalVelocity.X)) : 0;
		const bool IsFalling = Movement && Movement->IsFalling();
		const float Delta = FMath::IsFinite(DeltaSeconds) ? FMath::Max(0.0F, DeltaSeconds) : 0;
		EPlanetLocomotionPhase Next = EPlanetLocomotionPhase::Ground;
		if(IsFalling) Next = LocalVelocity.Z > 0 ? EPlanetLocomotionPhase::Rising : EPlanetLocomotionPhase::Falling;
		else if(WasFalling || Phase == EPlanetLocomotionPhase::Landing && PhaseSeconds < 0.18F) Next = EPlanetLocomotionPhase::Landing;
		if(Next != Phase) {
			PhaseSeconds = 0;
			if(Next != EPlanetLocomotionPhase::Ground) AirTimes[static_cast<unsigned>(Next) - 1] = 0;
			Phase = Next;
		}
		PhaseSeconds += Delta;
		WasFalling = IsFalling;
		// 姿勢の離散的な切替を，総和一の連続な重みへ変換
		const float Blend = 1 - std::exp(-Delta / 0.08F);
		for(float &Weight : Weights) Weight *= 1 - Blend;
		Weights[static_cast<unsigned>(Phase)] += Blend;
		for(unsigned I = 0; I < AirTimes.size(); ++I) {
			if(!AirClips[I]) continue;
			const float Length = AirClips[I]->GetPlayLength();
			AirTimes[I] += Delta;
			if(I == 1 && Length > 0) AirTimes[I] = std::fmod(AirTimes[I], Length);
			else AirTimes[I] = FMath::Min(AirTimes[I], Length);
		}
		SetBlendSpacePosition(FVector(Direction, Speed, 0));
		// 資産の最大速度を超す走行では，足の周期も速度へ追従
		SetPlayRate(static_cast<float>(FMath::Max(1.0, Speed / 600)));
		Animation->TangentialSpeedCentimetersPerSecond = Speed;
		Animation->VerticalSpeedCentimetersPerSecond = LocalVelocity.Z;
		Animation->DirectionDegrees = Direction;
		Animation->MotionPhase = Phase;
		// 終了
		return;
	}

	/**
	 * 元の歩行混合へ，物理状態から決まる上昇・落下・着地の姿勢を合成する
	 * @param Output 同じ骨構成の出力
	 * @return 姿勢を生成した場合に真
	 */
	virtual bool Evaluate(FPoseContext &Output) override {
		FAnimSingleNodeInstanceProxy::Evaluate(Output);
		FAnimationPoseData Result(Output);
		float Total = Weights[0];
		for(unsigned I = 0; I < AirClips.size(); ++I) {
			const float Weight = Weights[I + 1];
			if(!AirClips[I] || Weight <= ZERO_ANIMWEIGHT_THRESH) continue;
			FPoseContext Air(Output);
			Air.ResetToRefPose();
			FAnimationPoseData Pose(Air);
			AirClips[I]->GetAnimationPose(Pose, FAnimExtractContext(AirTimes[I], true));
			FAnimationRuntime::BlendTwoPosesTogetherInPlace(Result, Pose, Total / (Total + Weight));
			Total += Weight;
		}
		// 物理移動へ姿勢の根変位を加算しない描画結果の返戻
		return true;
	}
};

/**
 * 所有インスタンスへ，標準の資産混合を拡張した評価器を割当てる
 * @return エンジンが寿命を管理する評価器
 */
FAnimInstanceProxy *UPlanetExplorerAnimation::CreateAnimInstanceProxy() {
	return new FPlanetExplorerAnimationProxy(this);
}

/**
 * 描画部品の初期化後に，既存の歩行資産と物理優先の根変位設定を接続する
 */
void UPlanetExplorerAnimation::NativeInitializeAnimation() {
	Super::NativeInitializeAnimation();
	SetAnimationAsset(Locomotion, true, 1);
	SetRootMotionMode(ERootMotionMode::IgnoreRootMotion);
	// 終了
	return;
}

/**
 * 既存の移動資産を所有し，ブループリントを追加せずに評価する
 * @param ObjectInitializer エンジンの構築情報
 */
UPlanetExplorerAnimation::UPlanetExplorerAnimation(const FObjectInitializer &ObjectInitializer) : Super(ObjectInitializer) {
	static const ConstructorHelpers::FObjectFinder<UBlendSpace> GroundAsset(
		TEXT("/Game/Characters/Mannequins/Anims/Unarmed/BS_Idle_Walk_Run.BS_Idle_Walk_Run")
	);
	static const ConstructorHelpers::FObjectFinder<UAnimSequence> RiseAsset(
		TEXT("/Game/Characters/Mannequins/Anims/Unarmed/Jump/MM_Jump.MM_Jump")
	);
	static const ConstructorHelpers::FObjectFinder<UAnimSequence> FallAsset(
		TEXT("/Game/Characters/Mannequins/Anims/Unarmed/Jump/MM_Fall_Loop.MM_Fall_Loop")
	);
	static const ConstructorHelpers::FObjectFinder<UAnimSequence> LandAsset(
		TEXT("/Game/Characters/Mannequins/Anims/Unarmed/Jump/MM_Land.MM_Land")
	);
	Locomotion = GroundAsset.Object;
	Rise = RiseAsset.Object;
	Fall = FallAsset.Object;
	Land = LandAsset.Object;
	// 終了
	return;
}
