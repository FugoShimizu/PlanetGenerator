#include "PlanetSeasonalTexture.h"
#include "Core/PlanetSeasonalMap.h"
#include "Engine/Texture2D.h"
#include "Math/Float16Color.h"
#include "TextureResource.h"
#include <array>
#include <span>

/**
 * 生成した配列を保存せず，一時テクスチャへ転送する
 * @param Map 準備済みの季節配列
 * @return 一時テクスチャ，確保失敗の場合は空
 */
UTexture2D *CreatePlanetSeasonalTexture(const PlanetCore::PlanetSeasonalMap &Map) {
	// 保存しない配列を GPU と同じ半精度へ変換
	const std::span<const std::array<float, 4>> Source = Map.Data();
	TArray<FFloat16Color> Colors;
	Colors.Reserve(static_cast<int32>(Source.size()));
	for(const std::array<float, 4> &Pixel : Source) Colors.Emplace(FLinearColor(Pixel[0], Pixel[1], Pixel[2], Pixel[3]));
	UTexture2D *const Texture = UTexture2D::CreateTransient(
		PlanetCore::PlanetSeasonalMap::Width,
		PlanetCore::PlanetSeasonalMap::Height * PlanetCore::PlanetSeasonalMap::Layers,
		PF_FloatRGBA
	);
	// 確保失敗の返戻
	if(!Texture) return nullptr;
	// 経度の周期と極点を保つ一時テクスチャの構成
	Texture->SetFlags(RF_Transient);
	Texture->SRGB = false;
	Texture->NeverStream = true;
	Texture->Filter = TF_Bilinear;
	Texture->AddressX = TA_Wrap;
	Texture->AddressY = TA_Clamp;
	// 同じ層順での転送と描画資源の更新
	FTexture2DMipMap &Mip = Texture->GetPlatformData()->Mips[0];
	void *const Pixels = Mip.BulkData.Lock(LOCK_READ_WRITE);
	FMemory::Memcpy(Pixels, Colors.GetData(), Colors.Num() * sizeof(FFloat16Color));
	Mip.BulkData.Unlock();
	Texture->UpdateResource();
	// 保存しない一時テクスチャの返戻
	return Texture;
}
