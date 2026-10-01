#include "PlanetWorld.h"
#include "Core/PlanetClimate.h"
#include "Core/PlanetErosionPreparation.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetRunoff.h"
#include "Core/PlanetSeasonalMap.h"
#include "PlanetCelestialComponent.h"
#include "PlanetExplorerCharacter.h"
#include "PlanetSeasonalTexture.h"
#include "Async/Async.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "Kismet/GameplayStatics.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

namespace {
	struct FPlanetClimateResult {
		// 同じ元地形の気候・流量・被覆と準備費用
		std::shared_ptr<const PlanetCore::RiverTerrainField> Source;
		std::shared_ptr<const PlanetCore::PlanetClimate> Climate;
		std::optional<PlanetCore::PlanetSeasonalMap> Map;
		std::shared_ptr<const PlanetCore::PlanetRunoff> Runoff;
		double RunoffMilliseconds = 0;
		PlanetCore::ClimatePreparationStatistics Statistics;
		double Milliseconds = 0;
	};
} // namespace

struct FPlanetWorldState {
	// 非同期準備と，同じ公開定義に束縛した気候資源
	std::stop_source Cancellation;
	TFuture<FPlanetClimateResult> Future;
	std::shared_ptr<const PlanetCore::RiverTerrainField> RequestedSource;
	std::shared_ptr<const PlanetCore::RiverTerrainField> PublishedSource;
	std::shared_ptr<const PlanetCore::PlanetClimate> Climate;
	std::optional<PlanetCore::PlanetSeasonalMap> Map;
	std::shared_ptr<const PlanetCore::PlanetRunoff> Runoff;
};

/**
 * 地形・季節材質と，開始時迄は非表示の太陽・月の照明を構築する
 */
APlanetWorld::APlanetWorld() {
	ClimateState = MakeShared<FPlanetWorldState>();
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(
		TEXT("/Game/Planet/Materials/" "M_PlanetSeasonalSurface.M_PlanetSeasonalSurface")
	);
	if(Material.Succeeded()) SurfaceMaterial = Material.Object;
	CelestialClock = CreateDefaultSubobject<UPlanetCelestialComponent>(TEXT("PlanetCelestialClock"));
	CelestialClock->CreateScene(*this);
	// 終了
	return;
}

/**
 * 既存の公開設定を共通時計へ渡し，身体を地球参照へ対応させる
 */
void APlanetWorld::ConfigureCelestialObserver() {
	// 同じ惑星姿勢の身体参照と，旧公開設定の転送
	FVector Eye = GetActorTransform().TransformPosition(FVector::UpVector * RadiusMeters * 100);
	const APlanetExplorerCharacter *const Explorer = Cast<APlanetExplorerCharacter>(UGameplayStatics::GetPlayerPawn(GetWorld(), 0));
	if(Explorer && Explorer->Terrain == this) Eye = Explorer->GetActorLocation() + Explorer->GetActorUpVector() * 64;
	CelestialClock->SetObserver(GetActorTransform(), RadiusMeters, Eye);
	CelestialClock->TimeScale = TimeScale;
	CelestialClock->bAdvanceTime = bAdvanceTime;
	// 終了
	return;
}

/**
 * 着地準備中も，探索中と同じ惑星中心・半径の大気を使用する
 * @return 有効な同じゲーム世界の空を準備出来た場合に真
 */
bool APlanetWorld::PrepareCelestialAtmosphere() {
	ConfigureCelestialObserver();
	// 共通の大気準備結果の返戻
	return CelestialClock->PrepareAtmosphere();
}

/**
 * 既存の操作画面と検証へ，共通時計の公開状態を反映する
 */
void APlanetWorld::ReadCelestialClock() {
	bCelestialClockActive = CelestialClock->bCelestialClockActive;
	SimulationUtc = CelestialClock->SimulationUtc;
	ClockMessage = CelestialClock->ClockMessage;
	SunDirectionWorld = CelestialClock->SunDirectionWorld;
	MoonDirectionWorld = CelestialClock->MoonDirectionWorld;
	MoonToSunDirectionWorld = CelestialClock->MoonToSunDirectionWorld;
	MoonIlluminatedFraction = CelestialClock->MoonIlluminatedFraction;
	MoonAngularDiameterDegrees = CelestialClock->MoonAngularDiameterDegrees;
	SunAltitudeDegrees = CelestialClock->SunAltitudeDegrees;
	MoonAltitudeDegrees = CelestialClock->MoonAltitudeDegrees;
	GreenwichSiderealHours = CelestialClock->GreenwichSiderealHours;
	EarthObserverDegreesMeters = CelestialClock->EarthObserverDegreesMeters;
	// 未開始の季節単独検査は手動位相を維持
	if(bCelestialClockActive || CelestialClock->SimulationUtc.GetTicks()) SolarYearPhase = CelestialClock->SolarYearPhase;
	// 終了
	return;
}

/**
 * 設定した初期日時から，昼夜・月・季節の共通時計を開始する
 * @return 有効な観測地点・日時・倍率で開始出来た場合に真
 */
bool APlanetWorld::StartCelestialClock() {
	ConfigureCelestialObserver();
	CelestialClock->InitialUtc = InitialUtc;
	// 共通時計の操作結果と公開値の同期
	const bool Started = CelestialClock->StartCelestialClock();
	ReadCelestialClock();
	// 共通時計の開始結果の返戻
	return Started;
}

/**
 * 生成地形を変えず，昼夜・月・季節の日時を変更する
 * @param Utc １９００～２１００年の UTC 日時
 * @return 実行中の惑星へ有効な日時を設定出来た場合に真
 */
bool APlanetWorld::SetSimulationUtc(const FDateTime Utc) {
	ConfigureCelestialObserver();
	// 共通時計の操作結果と公開値の同期
	const bool Started = CelestialClock->SetSimulationUtc(Utc);
	ReadCelestialClock();
	// 共通時計の設定結果の返戻
	return Started;
}

/**
 * 身体の目線と同じ日時で天体照明・天球・季節を更新する
 * @param DeltaSeconds 前回の更新からの秒数
 */
void APlanetWorld::UpdateCelestialClock(const float DeltaSeconds) {
	ConfigureCelestialObserver();
	CelestialClock->Advance(DeltaSeconds);
	ReadCelestialClock();
	// 終了
	return;
}

/**
 * 同じ不変な地形の気候だけを公開し，古い生成定義の結果を破棄する
 */
void APlanetWorld::UpdateClimate() {
	if(ClimateState) {
		const std::shared_ptr<const PlanetCore::RiverTerrainField> Surface = GetPublishedField();
		if(bClimateBusy && ClimateState->RequestedSource != Surface) ClimateState->Cancellation.request_stop();
		if(bClimateBusy && ClimateState->Future.IsReady()) {
			FPlanetClimateResult Ready = ClimateState->Future.Consume();
			bClimateBusy = false;
			if(Ready.Source == Surface && Ready.Climate && Ready.Map) {
				UTexture2D *const Texture = CreatePlanetSeasonalTexture(*Ready.Map);
				if(Texture) {
					SeasonalTexture = Texture;
					ClimateState->PublishedSource = std::move(Ready.Source);
					ClimateState->Climate = std::move(Ready.Climate);
					ClimateState->Map = std::move(Ready.Map);
					ClimateState->Runoff = std::move(Ready.Runoff);
					LastRunoffPreparationMilliseconds = Ready.RunoffMilliseconds;
					LastClimatePreparationMilliseconds = Ready.Milliseconds;
					++ClimatePreparationCount;
					if(ClimateState->Runoff) {
						++RunoffPreparationCount;
						RunoffMessage = TEXT("海への年水収支を満たす流量を準備しました");
						const PlanetCore::RunoffStatistics &Stats = ClimateState->Runoff->GetStatistics();
						AnnualOceanRunoffCubicMeters = Stats.AnnualOceanRunoffCubicMeters;
						RunoffWaterBudgetRelativeResidual = Stats.WaterBudgetRelativeResidual;
					} else RunoffMessage = TEXT("流量の準備又は水収支の検査に失敗しました");
					ClimateMessage = TEXT("季節の地表被覆を準備しました");
				} else ClimateMessage = TEXT("季節テクスチャの確保に失敗しました");
			} else if(Ready.Source == Surface) {
				ClimateMessage = FString::Printf(
					TEXT("気候の準備失敗：段階 %u，輸送 %u，完了期数 %u"),
					static_cast<unsigned>(Ready.Statistics.Failure),
					static_cast<unsigned>(Ready.Statistics.TransportFailure),
					Ready.Statistics.CompletedPeriods
				);
			}
		}
		bClimateReady = Surface && Surface == ClimateState->PublishedSource && SeasonalTexture;
		bRunoffReady = bClimateReady && ClimateState->Runoff != nullptr;
		if(Surface && !bClimateBusy && ClimateState->RequestedSource != Surface) {
			ClimateState->RequestedSource = Surface;
			ClimateState->Cancellation = std::stop_source();
			const std::stop_token Token = ClimateState->Cancellation.get_token();
			bClimateBusy = true;
			ClimateMessage = TEXT("全球の季節被覆を準備しています");
			ClimateState->Future = Async(
				EAsyncExecution::ThreadPool,
				/**
				 * UObject に触れず，同じ地形の気候を共有して季節の材質配列を準備する
				 * @return 成功又は失敗の準備結果
				 */
				[Surface, Token]() -> FPlanetClimateResult {
					FPlanetClimateResult Result;
					Result.Source = Surface;
					const double Start = FPlatformTime::Seconds();
					const std::shared_ptr<const PlanetCore::PlanetErosionPreparation> &Prepared = Surface->GetErosionPreparation();
					if(Prepared) {
						Result.Climate = Prepared->GetClimate();
						Result.Runoff = Prepared->GetRunoff();
						Result.RunoffMilliseconds = Prepared->GetStatistics().RunoffMilliseconds;
						Result.Statistics = Result.Climate->PreparationStatistics();
					} else {
						std::optional<PlanetCore::PlanetClimate> Climate =
						PlanetCore::PlanetClimate::Create(Surface->OceanBase(), {}, Token, &Result.Statistics);
						if(Climate) Result.Climate = std::make_shared<const PlanetCore::PlanetClimate>(std::move(*Climate));
					}
					if(Result.Climate) Result.Map = PlanetCore::PlanetSeasonalMap::Create(*Result.Climate, Token);
					if(Result.Climate && Result.Map && !Prepared) {
						const double FlowStart = FPlatformTime::Seconds();
						std::optional<PlanetCore::PlanetRunoff> Runoff = PlanetCore::PlanetRunoff::Create(Surface->Drainage(), *Result.Climate, Token);
						if(Runoff) Result.Runoff = std::make_shared<const PlanetCore::PlanetRunoff>(std::move(*Runoff));
						Result.RunoffMilliseconds = (FPlatformTime::Seconds() - FlowStart) * 1000;
					}
					const double EarlierMilliseconds =
					Prepared ? Prepared->GetStatistics().ClimateMilliseconds + Prepared->GetStatistics().RunoffMilliseconds : 0;
					Result.Milliseconds = EarlierMilliseconds + (FPlatformTime::Seconds() - Start) * 1000;
					// 同じ生成定義の結果と準備時間の返戻
					return Result;
				}
			);
		}
	}
	// 終了
	return;
}

/**
 * 現在の表示面へ，同じ地形の季節被覆と現在の年周位相を渡す
 */
void APlanetWorld::ApplySeasonalMaterial() {
	UProceduralMeshComponent *const Mesh = GetVisibleMesh();
	UMaterialInstanceDynamic *const Material = Mesh ? Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(0)) : nullptr;
	if(Material) {
		const bool ValidPhase = std::isfinite(SolarYearPhase);
		Material->SetScalarParameterValue(TEXT("ClimateEnabled"), bClimateReady && ValidPhase ? 1 : 0);
		if(bClimateReady && ValidPhase) {
			Material->SetTextureParameterValue(TEXT("ClimateAtlas"), SeasonalTexture);
			Material->SetScalarParameterValue(TEXT("SolarYearPhase"), SolarYearPhase - std::floor(SolarYearPhase));
			const double Radius = ClimateState->PublishedSource->Base().Settings().RadiusMeters;
			const FVector Offset = Mesh->GetRelativeLocation() / (100 * Radius);
			Material->SetVectorParameterValue(TEXT("ClimatePlanetOffset"), FLinearColor(Offset.X, Offset.Y, Offset.Z, 0));
			Material->SetScalarParameterValue(TEXT("ClimateInverseRadius"), 1 / Radius);
		}
	}
	// 終了
	return;
}

/**
 * 公開した季節配列を地点で評価する
 * @param Direction 惑星座標の方向
 * @param Phase 年周位相
 * @param Coverage 草・枯草・落葉・雪の出力先
 * @param Factors 氷・湿潤・砂漠適性・樹木適性の出力先
 * @return 同じ生成定義の気候を評価出来た場合に真
 */
bool APlanetWorld::EvaluateSeasonalSurface(
	const FVector Direction,
	const double Phase,
	FLinearColor &Coverage,
	FLinearColor &Factors
) const {
	// 別の生成定義又は未準備の場合の返戻
	if(!bClimateReady || !ClimateState || !ClimateState->Map || GetPublishedField() != ClimateState->PublishedSource) return false;
	const std::optional<PlanetCore::UnitDirection> Unit =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 不正な方向の場合の返戻
	if(!Unit) return false;
	const std::optional<std::array<double, 8>> Sample = ClimateState->Map->Sample(*Unit, Phase);
	// 非有限な位相の場合の返戻
	if(!Sample) return false;
	Coverage = FLinearColor((*Sample)[0], (*Sample)[1], (*Sample)[2], (*Sample)[3]);
	Factors = FLinearColor((*Sample)[4], (*Sample)[5], (*Sample)[6], (*Sample)[7]);
	// 公開した季節配列の評価結果の返戻
	return true;
}

/**
 * 地形の更新後に共通時計・気候・材質を反映する
 * @param DeltaSeconds 前回の更新からの秒数
 */
void APlanetWorld::Tick(const float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	UpdateCelestialClock(DeltaSeconds);
	UpdateClimate();
	ApplySeasonalMaterial();
	// 終了
	return;
}

/**
 * ワールド終了時に気候の準備へ中断を要求する
 * @param Reason 除去する理由
 */
void APlanetWorld::EndPlay(const EEndPlayReason::Type Reason) {
	if(ClimateState) ClimateState->Cancellation.request_stop();
	Super::EndPlay(Reason);
	// 終了
	return;
}

/**
 * 気候の準備を中断し，所有資源を解放する
 */
void APlanetWorld::BeginDestroy() {
	if(ClimateState) ClimateState->Cancellation.request_stop();
	ClimateState.Reset();
	Super::BeginDestroy();
	// 終了
	return;
}

/**
 * 同じ生成定義の季節配列を，地形と植生の材質で共有する
 * @return 準備済みの一時テクスチャ，未準備又は異なる生成定義では空
 */
UTexture2D *APlanetWorld::GetSeasonalTexture() const {
	// 公開中の地形と一致する一時配列だけを返戻
	return bClimateReady && ClimateState && GetPublishedField() == ClimateState->PublishedSource ? SeasonalTexture.Get() : nullptr;
}

/**
 * 植生の生息適性と季節応答を，同じ生成定義の気候から取得する
 * @param Direction 惑星座標の方向
 * @return 不変な気候応答，準備前又は不正な方向では空
 */
std::optional<PlanetCore::BiomeProfile> APlanetWorld::GetBiomeProfile(const FVector Direction) const {
	// 異なる生成定義の気候を混ぜない返戻
	if(!bClimateReady || !ClimateState || !ClimateState->Climate || GetPublishedField() != ClimateState->PublishedSource) return {};
	const std::optional<PlanetCore::UnitDirection> Unit =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 不正な方向の場合の返戻
	if(!Unit) return {};
	// 植生の適性と季節応答を持つ地点の気候の返戻
	return ClimateState->Climate->SampleBiome(*Unit);
}

/**
 * 公開中の地形と対応する流量を共有する
 * @return 同じ生成定義の不変な流量，未準備では空
 */
std::shared_ptr<const PlanetCore::PlanetRunoff> APlanetWorld::GetRunoffField() const {
	// 異なる生成定義の流量を使わない返戻
	return bRunoffReady && ClimateState && GetPublishedField() == ClimateState->PublishedSource ? ClimateState->Runoff : nullptr;
}

/**
 * 河道の上流端における流量を共通時計の年周位相で評価する
 * @param ReachIndex 公開中の河道番号
 * @param Phase 共通の年周位相
 * @param CubicMetersPerSecond 季節の流量の出力先
 * @param AnnualMeanCubicMetersPerSecond 参照年の平均流量の出力先
 * @return 同じ地形の流量を取得出来た場合に真
 */
bool APlanetWorld::EvaluateRiverDischarge(
	const int32 ReachIndex,
	const double Phase,
	double &CubicMetersPerSecond,
	double &AnnualMeanCubicMetersPerSecond
) const {
	CubicMetersPerSecond = 0;
	AnnualMeanCubicMetersPerSecond = 0;
	const std::shared_ptr<const PlanetCore::PlanetRunoff> Flow = GetRunoffField();
	const std::shared_ptr<const PlanetCore::RiverTerrainField> Source = GetPublishedField();
	// 未準備又は範囲外の河道の返戻
	if(!Flow || !Source || ReachIndex < 0 || static_cast<std::size_t>(ReachIndex) >= Source->Drainage().Reaches().size()) {
		// 対応する流量が無い場合の返戻
		return false;
	}
	const std::uint32_t Node = Source->Drainage().Reaches()[ReachIndex].Upstream;
	const std::optional<double> Sample = Flow->Sample(Node, Phase);
	// 不正な位相の返戻
	if(!Sample) return false;
	CubicMetersPerSecond = *Sample;
	for(const double Value : Flow->GetPeriods()[Node]) AnnualMeanCubicMetersPerSecond += Value / PlanetCore::ClimatePeriodCount;
	// 公開中の河道の流量を取得した結果の返戻
	return true;
}
