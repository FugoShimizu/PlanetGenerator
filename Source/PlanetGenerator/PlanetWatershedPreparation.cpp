#include "PlanetWatershedPreparation.h"
#include "Core/PlanetFlowWatershed.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Async/Async.h"
#include "HAL/PlatformTime.h"
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

namespace {
	struct FWatershedResult {
		// 作業の完了内容と失敗段階；公開する地形は所有権付き
		std::shared_ptr<const PlanetCore::PreparedFlowWatershed> Prepared;
		PlanetCore::FlowWatershedDiagnostics Diagnostics;
		FString Failure;
		double Milliseconds = 0;
	};

	/**
	 * UObject を参照せず，同じ生成条件から地質・大洋・一水系を準備する
	 * @param Definition 固定した生成条件
	 * @param Direction 地点からの水系探索の開始方向
	 * @param Stop 全準備段階へ伝播する中断要求
	 * @return 成功した所有場，又は失敗段階を持つ結果
	 */
	std::shared_ptr<FWatershedResult> PrepareWatershed(
		const PlanetCore::GeologySettings Definition,
		const PlanetCore::UnitDirection Direction,
		const std::stop_token Stop
	) {
		// 地質と造山場の準備
		const double Started = FPlatformTime::Seconds();
		const std::shared_ptr<FWatershedResult> Result = std::make_shared<FWatershedResult>();
		const auto Finish = [&](const FString &Failure) -> std::shared_ptr<FWatershedResult> {
			Result->Failure = Failure;
			Result->Milliseconds = (FPlatformTime::Seconds() - Started) * 1000;
			// 完了時刻と失敗段階を持つ結果の返戻
			return Result;
		};
		std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition, Stop);
		// 不完全な元場の返戻
		if(!Base) return Finish(TEXT("地質の準備を完了出来ませんでした．"));
		const std::shared_ptr<const PlanetCore::GeologyField> Geology =
		std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
		std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology, Stop);
		// 不完全な造山場の返戻
		if(!Ridges) return Finish(TEXT("山地の準備を完了出来ませんでした．"));
		// 大陸・海底と接続した大洋の準備
		std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
			Geology,
			std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
			Stop,
			PlanetCore::PredrainageReliefMode::MesoAndLand,
			PlanetCore::OceanSamplingMode::SharedDrainage
		);
		// 不完全な大洋の返戻
		if(!Ocean) return Finish(TEXT("大洋の接続を確認出来ませんでした．"));
		// 地点に接続する一水系の準備
		std::optional<PlanetCore::PreparedFlowWatershed> Prepared = PlanetCore::PrepareFlowWatershed(
			Direction,
			std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*Ocean)),
			Stop,
			&Result->Diagnostics
		);
		if(!Prepared) {
			const PlanetCore::FlowWatershedDiagnostics &Report = Result->Diagnostics;
			// 未確認の盆地を架空の河道で補わない失敗結果の返戻
			return Finish(
				FString::Printf(
					TEXT("地点から大洋までの水系を確認出来ませんでした．段階 %u，入口 %u，接続 %u，河道 %u．"),
					static_cast<unsigned>(Report.Failure),
					static_cast<unsigned>(Report.Seed.Failure),
					static_cast<unsigned>(Report.Network.Failure),
					static_cast<unsigned>(Report.Channels.Failure)
				)
			);
		}
		// 完成した定義だけの共有公開
		Result->Prepared = std::make_shared<const PlanetCore::PreparedFlowWatershed>(std::move(*Prepared));
		// 費用を含む完了結果の返戻
		return Finish({});
	}
}

struct FPlanetWatershedPreparationState {
	// 一件の作業と公開済の不変場
	std::stop_source Stop;
	TFuture<std::shared_ptr<FWatershedResult>> Future;
	std::shared_ptr<const PlanetCore::PreparedFlowWatershed> Prepared;
	bool HasWorker = false;
};

/**
 * 地点からの準備を受理し，ゲームスレッド上では数値条件だけを確認する
 * @param Seed 惑星の乱数種
 * @param RadiusMeters 惑星半径，１０００〜１０００００００００メートル
 * @param Direction 通常の陸上地点の方向
 * @return 有効な初回要求を受理した場合に真；作業中・公開済・不正条件なら偽
 */
bool APlanetWatershedPreparation::RequestPreparation(const int64 Seed, const double RadiusMeters, const FVector Direction) {
	// 同じ所有者に複数の生成を重ねない返戻
	if(IsBusy || IsReady || IsActorBeingDestroyed()) return false;
	PlanetCore::GeologySettings Definition;
	Definition.Seed = static_cast<uint64>(Seed);
	Definition.RadiusMeters = RadiusMeters;
	const std::optional<PlanetCore::UnitDirection> Unit =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 入力拒否で既存の公開定義を変更しない返戻
	if(!Definition.IsValid() || !Unit) return false;
	State = MakeShared<FPlanetWatershedPreparationState>();
	const std::stop_token Stop = State->Stop.get_token();
	State->HasWorker = true;
	IsBusy = true;
	PreparationMilliseconds = 0;
	Status = TEXT("地質・大洋・水系を準備しています．");
	// UObject 又は呼出時の局所参照を捕捉しない作業の開始
	State->Future = Async(
		EAsyncExecution::ThreadPool,
		[Definition, Unit, Stop]() -> std::shared_ptr<FWatershedResult> {
			return PrepareWatershed(Definition, *Unit, Stop);
		}
	);
	// 作業開始の受理結果の返戻
	return true;
}

/**
 * 待機せず完了を確認し，全段階が成功した不変場だけを公開する
 */
void APlanetWatershedPreparation::PollPreparation() {
	// 実行中のワーカーをゲームスレッドで待たない終了
	if(!State || !State->HasWorker || !State->Future.IsReady()) return;
	const std::shared_ptr<FWatershedResult> Result = State->Future.Consume();
	State->HasWorker = false;
	IsBusy = false;
	PreparationMilliseconds = Result->Milliseconds;
	if(Result->Prepared && !State->Stop.stop_requested()) {
		State->Prepared = Result->Prepared;
		IsReady = true;
		Status = TEXT("地点から大洋へ繋がる水系を準備しました．");
	} else Status = Result->Failure;
	// 終了
	return;
}

/**
 * 所有作業の終了を確認して準備器を空に戻す；外部が共有する完成場は有効のまま保持する
 */
void APlanetWatershedPreparation::CancelPreparation() {
	if(State) {
		State->Stop.request_stop();
		if(State->HasWorker) {
			State->Future.Wait();
			State->Future.Consume();
			State->HasWorker = false;
		}
		State.Reset();
	}
	IsBusy = IsReady = false;
	Status = TEXT("水系の準備を停止しました．");
	// 終了
	return;
}

/**
 * 足場・描画・気候へ同じ寿命の定義を渡す
 * @return 完成した共有場；未完成・中断時は空
 */
std::shared_ptr<const PlanetCore::PreparedFlowWatershed> APlanetWatershedPreparation::GetPrepared() const {
	return State ? State->Prepared : nullptr;
}

/**
 * 非同期の完了だけを確認する
 * @param DeltaSeconds 前回の更新からの秒数
 */
void APlanetWatershedPreparation::Tick(const float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	PollPreparation();
	// 終了
	return;
}

/**
 * 世界の終了前に所有ワーカーを解放する
 * @param Reason 終了理由
 */
void APlanetWatershedPreparation::EndPlay(const EEndPlayReason::Type Reason) {
	CancelPreparation();
	Super::EndPlay(Reason);
	// 終了
	return;
}

/**
 * 実行世界を開始しなかった場合にも所有作業を残さない
 */
void APlanetWatershedPreparation::BeginDestroy() {
	CancelPreparation();
	Super::BeginDestroy();
	// 終了
	return;
}

/**
 * プレイ開始前の明示的な破棄でも，ガベージ回収を待たず所有作業を終了する
 */
void APlanetWatershedPreparation::Destroyed() {
	CancelPreparation();
	Super::Destroyed();
	// 終了
	return;
}

/**
 * 重い計算を開始せず，完了確認の更新だけを設定する
 */
APlanetWatershedPreparation::APlanetWatershedPreparation() {
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;
	// 終了
	return;
}
