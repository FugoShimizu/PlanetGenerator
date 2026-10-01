#include "PlanetCharacterMovement.h"
#include "Core/PlanetGround.h"
#include "PlanetExplorerCharacter.h"
#include "PlanetGroundPatch.h"
#include "Components/PrimitiveComponent.h"

/**
 * 地表面の重力加速度を半径に依らず一定にする
 * @return 下向きの加速度，単位はセンチメートル毎秒毎秒
 */
float UPlanetCharacterMovement::GetGravityZ() const {
	return static_cast<float>(-100 * PlanetCore::SurfaceGravityMetersPerSecondSquared);
}

/**
 * 現在の球面方向へ適用する加速度を取得する
 * @return 同じ一定強度のワールド加速度
 */
FVector UPlanetCharacterMovement::GetPlanetGravityVector() const {
	return GetGravityDirection() * -GetGravityZ();
}

/**
 * 現在の接地先が再生成する静的な地面か確認する
 * @return 地形の足場が移動基準の場合に真
 */
bool UPlanetCharacterMovement::IsPlanetGroundBase() const {
	const UPrimitiveComponent *const Base = Cast<UPrimitiveComponent>(GetMovementBaseObject());
	// 再生成する静的な足場への所属の返戻
	return Base && Base->GetOwner() && Base->GetOwner()->IsA<APlanetGroundPatch>();
}

/**
 * 地形の原点更新を，身体を運ぶ床の移動として適用しない
 * @param DeltaSeconds エンジンが通知する経過秒数
 */
void UPlanetCharacterMovement::UpdateBasedMovement(const float DeltaSeconds) {
	if(!IsPlanetGroundBase()) Super::UpdateBasedMovement(DeltaSeconds);
	// 終了
	return;
}

/**
 * 再生成する静的な足場の見掛の変位を，跳躍の速度へ加えない
 * @return 地形上では零，通常の移動床ではエンジンの速度
 */
FVector UPlanetCharacterMovement::GetImpartedMovementBaseVelocity() const {
	return IsPlanetGroundBase() ? FVector::ZeroVector : Super::GetImpartedMovementBaseVelocity();
}

/**
 * 公開済足場と同じ惑星中心へ重力と身体の上向きを合わせ，通常の衝突移動を行う
 * @param DeltaTime エンジンが通知する経過秒数
 * @param TickType 更新の種別
 * @param ThisTickFunction 実行中の更新登録
 */
void UPlanetCharacterMovement::TickComponent(
	const float DeltaTime,
	const ELevelTick TickType,
	FActorComponentTickFunction *const ThisTickFunction
) {
	// 公開済の同じ惑星定義に基づく姿勢の更新
	const APlanetExplorerCharacter *const Explorer = Cast<APlanetExplorerCharacter>(CharacterOwner);
	FTransform PlanetTransform;
	double Radius = 0;
	if(Explorer && IsValid(Explorer->Ground) && UpdatedComponent && Explorer->Ground->GetPublishedPlanet(PlanetTransform, Radius)) {
		const FVector Up = (UpdatedComponent->GetComponentLocation() - PlanetTransform.GetLocation()).GetSafeNormal();
		if(!Up.IsNearlyZero()) {
			SetGravityDirection(-Up);
			const FQuat Old = UpdatedComponent->GetComponentQuat();
			const FQuat New = (FQuat::FindBetweenNormals(Old.GetUpVector(), Up) * Old).GetNormalized();
			FHitResult Hit;
			MoveUpdatedComponent(FVector::ZeroVector, New, true, &Hit);
		}
	}
	// エンジン標準の身体寸法を持つ移動と衝突解決
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// 終了
	return;
}
