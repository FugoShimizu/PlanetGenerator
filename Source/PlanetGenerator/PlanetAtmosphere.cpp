#include "PlanetAtmosphere.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/StrongObjectPtr.h"
#include <array>
#include <cmath>

namespace {
	// 既存の小半径描画で確認した採光距離の設定
	constexpr std::array<const TCHAR *, 2> SkySettingNames {
		TEXT("r.SkyAtmosphere.DistantSkyLightLUT.Altitude"),
		TEXT("r.VolumetricCloud.DistanceToSampleMaxCount")
	};

	struct FAtmosphereSource {
		// 適用前の部品・姿勢・光学的な厚さ
		TWeakObjectPtr<USkyAtmosphereComponent> Component;
		FTransform Transform;
		ESkyAtmosphereTransformMode Mode;
		float BottomRadius;
		float Height;
		float RayleighHeight;
		float MieHeight;
		float RayleighScale;
		float MieScale;
		float MieAbsorption;
		float OtherAbsorption;
		FTentDistribution Tent;
		void Apply(const FVector Center, const double RadiusMeters, const float Scale, const bool Restoring) const;

		explicit FAtmosphereSource(USkyAtmosphereComponent &Source);
	};

	struct FCloudSource {
		// 雲材質は部品から差替えても，元の動的な材質を GC から保持
		TWeakObjectPtr<UVolumetricCloudComponent> Component;
		TStrongObjectPtr<UMaterialInterface> Material;
		TWeakObjectPtr<UMaterialInstanceDynamic> ScaledMaterial;
		float Base;
		float Height;
		bool PerSampleTransmittance;
		void Restore() const;
		float Apply(UObject &Owner, const float Scale);

		explicit FCloudSource(UVolumetricCloudComponent &Source);
	};

	/**
	 * 元の大気の寸法と係数を，一度だけ取得する
	 * @param Source 同じゲーム世界の大気
	 */
	FAtmosphereSource::FAtmosphereSource(USkyAtmosphereComponent &Source) : Component(&Source),
	Transform(Source.GetComponentTransform()),
	Mode(Source.TransformMode),
	BottomRadius(Source.BottomRadius),
	Height(Source.AtmosphereHeight),
	RayleighHeight(Source.RayleighExponentialDistribution),
	MieHeight(Source.MieExponentialDistribution),
	RayleighScale(Source.RayleighScatteringScale),
	MieScale(Source.MieScatteringScale),
	MieAbsorption(Source.MieAbsorptionScale),
	OtherAbsorption(Source.OtherAbsorptionScale),
	Tent(Source.OtherTentDistribution) {}

	/**
	 * 元の寸法から大気を設定し，繰返適用でも縮尺を重複して掛けない
	 * @param Center 公開済の惑星中心，単位 cm
	 * @param RadiusMeters 海面半径
	 * @param Scale 長さへ掛け，密度へ逆比を掛ける係数
	 * @param Restoring 初期状態への復元なら真
	 */
	void FAtmosphereSource::Apply(const FVector Center, const double RadiusMeters, const float Scale, const bool Restoring) const {
		// 生存する元の大気部品
		USkyAtmosphereComponent *const Target = Component.Get();
		// 既に除去された大気には触れない終了
		if(!Target) return;
		Target->TransformMode = Restoring ? Mode : ESkyAtmosphereTransformMode::PlanetCenterAtComponentTransform;
		Target->SetBottomRadius(Restoring ? BottomRadius : static_cast<float>(RadiusMeters / 1000));
		Target->SetAtmosphereHeight(Restoring ? Height : FMath::Max(0.1F, Height * Scale));
		Target->SetRayleighExponentialDistribution(RayleighHeight * Scale);
		Target->SetMieExponentialDistribution(MieHeight * Scale);
		Target->SetRayleighScatteringScale(RayleighScale / Scale);
		Target->SetMieScatteringScale(MieScale / Scale);
		Target->SetMieAbsorptionScale(MieAbsorption / Scale);
		Target->SetOtherAbsorptionScale(OtherAbsorption / Scale);
		Target->OtherTentDistribution = Tent;
		Target->OtherTentDistribution.TipAltitude *= Scale;
		Target->OtherTentDistribution.Width *= Scale;
		if(Restoring) Target->SetWorldTransform(Transform);
		else Target->SetWorldLocation(Center);
		Target->MarkRenderStateDirty();
		// 終了
		return;
	}

	/**
	 * 雲の元の寸法・材質・透過設定を保持する
	 * @param Source 同じゲーム世界の雲
	 */
	FCloudSource::FCloudSource(UVolumetricCloudComponent &Source) : Component(&Source),
	Material(Source.GetMaterial()),
	Base(Source.LayerBottomAltitude),
	Height(Source.LayerHeight),
	PerSampleTransmittance(Source.bUsePerSampleAtmosphericLightTransmittance) {}

	/**
	 * 生存する雲へ，変更前の寸法と同じ材質参照を戻す
	 */
	void FCloudSource::Restore() const {
		if(UVolumetricCloudComponent *const Target = Component.Get()) {
			Target->SetLayerBottomAltitude(Base);
			Target->SetLayerHeight(Height);
			Target->SetbUsePerSampleAtmosphericLightTransmittance(PerSampleTransmittance);
			Target->SetMaterial(Material.Get());
		}
		// 終了
		return;
	}

	/**
	 * 元の雲から同じ密度模様を拡縮し，動的材質の多重な親子化を避ける
	 * @param Owner 一時材質を所有する Actor
	 * @param Scale 惑星半径に応じた長さの係数
	 * @return 小半径で雲底を最低２０ m へ補正した上昇量，単位 km
	 */
	float FCloudSource::Apply(UObject &Owner, const float Scale) {
		// 生存する元の雲部品
		UVolumetricCloudComponent *const Target = Component.Get();
		// 除去済の雲は採光距離へ寄与しない返戻
		if(!Target) return 0;
		const float ScaledBase = Base * Scale, VisibleBase = FMath::Max(0.02F, ScaledBase);
		Target->SetLayerBottomAltitude(VisibleBase);
		Target->SetLayerHeight(FMath::Max(0.01F, Height * Scale));
		Target->SetbUsePerSampleAtmosphericLightTransmittance(true);
		if(Material) {
			UMaterialInstanceDynamic *Instance = ScaledMaterial.Get();
			if(!Instance) {
				UMaterialInstanceDynamic *const OriginalDynamic = Cast<UMaterialInstanceDynamic>(Material.Get());
				Instance = UMaterialInstanceDynamic::Create(OriginalDynamic ? OriginalDynamic->Parent.Get() : Material.Get(), &Owner);
				if(Instance) {
					Instance->SetFlags(RF_Transient);
					if(OriginalDynamic) Instance->CopyParameterOverrides(OriginalDynamic);
					ScaledMaterial = Instance;
				}
			}
			if(Instance) {
				float Value = 0;
				if(Material->GetScalarParameterValue(FMaterialParameterInfo(TEXT("Layout_CloudGlobalScale")), Value)) {
					Instance->SetScalarParameterValue(TEXT("Layout_CloudGlobalScale"), Value * Scale);
				}
				if(Material->GetScalarParameterValue(FMaterialParameterInfo(TEXT("Cloud_GlobalDensity")), Value)) {
					Instance->SetScalarParameterValue(TEXT("Cloud_GlobalDensity"), Value / Scale);
				}
				Target->SetMaterial(Instance);
			}
		}
		// 雲底の最低高度補正の返戻
		return VisibleBase - ScaledBase;
	}
}

struct FPlanetAtmosphereState {
	// 同じ世界で取得した初期値と，所有する一時設定の識別子
	TWeakObjectPtr<UWorld> World;
	TArray<FAtmosphereSource> Atmospheres;
	TArray<FCloudSource> Clouds;
	std::array<float, 2> OriginalSettings {};
	FName Tag;
	FVector Center = FVector::ZeroVector;
	double Radius = 0;
};

/**
 * 所有した一時設定と生存中の部品だけを，適用前の状態へ戻す
 */
void FPlanetAtmosphere::Restore() {
	// 未適用なら復元対象無しの終了
	if(!State) return;
	for(const FAtmosphereSource &Source : State->Atmospheres) Source.Apply({}, 0, 1, true);
	for(const FCloudSource &Source : State->Clouds) Source.Restore();
	for(const TCHAR *const Name : SkySettingNames) {
		if(IConsoleVariable *const Setting = IConsoleManager::Get().FindConsoleVariable(Name)) {
			Setting->Unset(ECVF_SetByTemp, State->Tag);
		}
	}
	State.Reset();
	// 終了
	return;
}

/**
 * 公開済の惑星中心・半径へ既存の大気と雲を揃える
 * @param Owner 同じゲーム世界の所有 Actor
 * @param CenterWorld 公開済の惑星中心，単位 cm
 * @param RadiusMeters 公開済の海面半径，単位 m
 * @return 有効な同じ世界へ適用出来た場合に真；既存の空が無い検査世界でも時計を妨げない
 */
bool FPlanetAtmosphere::Apply(AActor &Owner, const FVector CenterWorld, const double RadiusMeters) {
	// 初回と同じゲーム世界の確認
	UWorld *const World = Owner.GetWorld();
	// 編集世界を変更しない返戻
	if(!World || !World->IsGameWorld()) return false;
	// 不正な尺度や中心を反映しない返戻
	if(!std::isfinite(RadiusMeters) || RadiusMeters < 1000 || CenterWorld.ContainsNaN()) return false;
	// 別の世界の初期値を流用しない返戻
	if(State && State->World.Get() != World) return false;
	// 同じ座標では描画状態と設定履歴を再構築しない返戻
	if(State && State->Radius == RadiusMeters && State->Center.Equals(CenterWorld, 0)) return true;
	if(!State) {
		State = MakeShared<FPlanetAtmosphereState>();
		State->World = World;
		State->Tag = FName(*(Owner.GetPathName() + TEXT(":PlanetAtmosphere")));
		for(TActorIterator<ASkyAtmosphere> Iterator(World); Iterator; ++Iterator) State->Atmospheres.Emplace(*Iterator->GetComponent());
		for(TActorIterator<AVolumetricCloud> Iterator(World); Iterator; ++Iterator) {
			if(UVolumetricCloudComponent *const Cloud = Iterator->FindComponentByClass<UVolumetricCloudComponent>()) {
				State->Clouds.Emplace(*Cloud);
			}
		}
		for(unsigned I = 0; I < SkySettingNames.size(); ++I) {
			if(IConsoleVariable *const Setting = IConsoleManager::Get().FindConsoleVariable(SkySettingNames[I])) {
				State->OriginalSettings[I] = Setting->GetFloat();
			}
		}
	}
	// 同じ初期値から寸法・密度・採光距離を再計算
	const float Scale = static_cast<float>(FMath::Clamp(RadiusMeters / 1000000, 0.001, 1.0));
	for(const FAtmosphereSource &Source : State->Atmospheres) Source.Apply(CenterWorld, RadiusMeters, Scale, false);
	float CloudShift = 0;
	for(FCloudSource &Source : State->Clouds) CloudShift = FMath::Max(CloudShift, Source.Apply(Owner, Scale));
	for(unsigned I = 0; I < SkySettingNames.size(); ++I) {
		if(IConsoleVariable *const Setting = IConsoleManager::Get().FindConsoleVariable(SkySettingNames[I])) {
			Setting->Set(State->OriginalSettings[I] * Scale + (!I ? CloudShift : 0), ECVF_SetByTemp, State->Tag);
		}
	}
	State->Center = CenterWorld;
	State->Radius = RadiusMeters;
	// 同じ世界への適用結果の返戻
	return true;
}
