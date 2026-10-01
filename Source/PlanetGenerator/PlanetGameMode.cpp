#include "PlanetGameMode.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetScatter.h"
#include "Core/PlanetScenicView.h"
#include "PlanetExplorationHud.h"
#include "PlanetExplorerCharacter.h"
#include "PlanetGroundPatch.h"
#include "PlanetVegetation.h"
#include "PlanetWorld.h"
#include "Components/DirectionalLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "ProfilingDebugging/CsvProfiler.h"
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>

namespace {
	// 着地と可視性の判定に用いる距離・日照・探索上限
	constexpr double LandingEyeHeightMeters = 1.65;
	constexpr double LandingSurfaceRadiusMeters = 24;
	constexpr double LandingDaylightSunDot = 0.17364817766693033;
	constexpr double LandingScenicScoreWeight = 2.3;
	constexpr int32 LandingCandidateCount = 2048;

	TAutoConsoleVariable<int32> CVarPlanetLandingCandidate(
		TEXT("planet.ExplorationLandingCandidate"),
		-1,
		TEXT("Use one fixed landing candidate for terrain and skyline inspection; -1 restores the normal search."),
		ECVF_Default
	);

	constexpr double LandingTreeSafetyMeters = 2;
	constexpr double LandingSightlineMeters = 12;
	constexpr double LandingBlockedSightlinePenalty = 0.9;
} // namespace

/**
 * 操作開始迄の待機と，探索用の画面表示を設定する
 */
APlanetGameMode::APlanetGameMode() {
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;
	DefaultPawnClass = nullptr;
	HUDClass = APlanetExplorationHud::StaticClass();
	// 終了
	return;
}

/**
 * 準備失敗を表示し，原因を実行ログへ残す
 * @param Message 操作者へ示す失敗内容
 * @param Details 診断用の詳細
 */
void APlanetGameMode::FailPreparation(const FString &Message, const FString &Details) {
	ExplorationStage = EPlanetExplorationStage::Failed;
	PreparationMessage = Message;
	UE_LOG(LogTemp, Error, TEXT("Planet exploration preparation: %s | %s"), *Message, *Details);
	// 終了
	return;
}

/**
 * 地表の準備前に既定の Pawn を生成せず，接続した操作者を保持する
 * @param NewPlayer 探索を開始する操作者
 */
void APlanetGameMode::HandleStartingNewPlayer_Implementation(APlayerController *const NewPlayer) {
	if(IsValid(NewPlayer) && !ExplorerController) ExplorerController = NewPlayer;
	// 終了
	return;
}

/**
 * 配置済みの惑星設定を使い，探索の準備を開始する
 */
void APlanetGameMode::BeginPlay() {
	Super::BeginPlay();
	PreparationStartedSeconds = FPlatformTime::Seconds();
	for(TActorIterator<APlanetWorld> Iterator(GetWorld()); Iterator; ++Iterator) {
		if(Planet) {
			FailPreparation(TEXT("探索する惑星が複数配置されています"), TEXT("Expected exactly one PlanetWorld actor"));
			// 対象を曖昧に選択しない終了
			return;
		}
		Planet = *Iterator;
	}
	if(!Planet) {
		FailPreparation(TEXT("探索する惑星が配置されていません"), TEXT("The exploration map requires one PlanetWorld actor"));
		// 惑星が無い場合の終了
		return;
	}
	if(!Planet->StartCelestialClock()) {
		FailPreparation(TEXT("天体と季節の共通時計を開始出来ませんでした"), Planet->ClockMessage);
		// 無効な初期日時又は時間倍率の場合の終了
		return;
	}
	// 探索中は惑星に属する二灯を使用し，Editor 用の太陽を重複させない
	for(TActorIterator<ADirectionalLight> Iterator(GetWorld()); Iterator; ++Iterator) {
		Iterator->GetLightComponent()->SetVisibility(false);
	}
	AddTickPrerequisiteActor(Planet);
	Planet->bFollowPlayerCamera = false;
	Planet->bGenerateOnBeginPlay = false;
	Planet->bRefineNearSurface = false;
	Planet->FoliagePrecisionRadiusMeters = 64;
	SurfaceLeafBudget = Planet->LeafBudget;
	Planet->LeafBudget = 512;
	// 最小半径でも海面近景の範囲外から全球を準備し，着地点を決めてから近景へ進む
	Planet->ObserverMeters = FVector(0, 0, Planet->RadiusMeters * 1.6);
	if(!Planet->bGenerationBusy && !Planet->PublishedRevision && !Planet->RequestGeneration()) {
		FailPreparation(TEXT("指定した条件で惑星を準備出来ませんでした"), Planet->GenerationMessage);
	}
	// 終了
	return;
}

/**
 * 全球候補又は其の近傍の方向を，同じ安全条件と景観式で評価する
 * @param Direction 評価する惑星座標の単位方向
 * @param Index 元の全球候補番号
 * @param OffsetMeters 元の候補からの接線距離，単位 m
 */
void APlanetGameMode::ConsiderLandingDirection(const FVector &Direction, const int32 Index, const double OffsetMeters) {
	// 候補の標高・傾斜と季節被覆
	double Elevation = 0;
	FVector Normal;
	// 水没又は無効な地表の候補を使わない終了
	if(!Planet->EvaluateDirection(Direction, Elevation, Normal) || Elevation < 2) return;
	// 急斜面へ着地しない終了
	if(FVector::DotProduct(Direction, Normal) < 0.95) return;
	FLinearColor Coverage, Factors;
	// 対応する季節被覆が無い候補を使わない終了
	if(!Planet->EvaluateSeasonalSurface(Direction, Planet->SolarYearPhase, Coverage, Factors)) return;
	// 歩行可能性・植生・日照に依る基本評価
	const double BaseScore = 1 + Coverage.R + 0.5 * Factors.A - Coverage.A - Factors.R - 0.2 * Factors.B;
	const double ForestSuitability = FMath::Clamp(static_cast<double>(Factors.A), 0.0, 1.0);
	const double OpenViewFactor = 1 - 0.7 * ForestSuitability, SunDot = FVector::DotProduct(Direction, LandingSunDirection);
	const bool bDaylit = SunDot >= LandingDaylightSunDot;
	// 昼間に歩ける地点が有れば夜側へ戻らない
	// 無ければ夜側を代替に使う
	if(bBestLandingDaylit && !bDaylit) return;
	// 同じ日照区分で最大加点でも最良候補へ届かない地点は省く
	if(bDaylit == bBestLandingDaylit && BaseScore + 0.12 + LandingScenicScoreWeight * OpenViewFactor <= BestLandingScore) return;
	// 接線四方向の歩行可能な近傍と，遮られない稜線
	const double Radius = Planet->RadiusMeters, Span = FMath::Min(100.0, Radius * 0.01);
	const FVector First = FVector::CrossProduct(Direction, FVector::UpVector).GetSafeNormal();
	const FVector Second = FVector::CrossProduct(Direction, First).GetSafeNormal();
	const FVector Axes[] = { First, -First, Second, -Second };
	double Minimum = Elevation, Maximum = Elevation;
	bool bAccessible = true;
	for(const FVector &Axis : Axes) {
		const FVector ProbeDirection = (Direction * Radius + Axis * Span).GetSafeNormal();
		double ProbeElevation = 0;
		FVector ProbeNormal;
		if(
			!Planet->EvaluateDirection(ProbeDirection, ProbeElevation, ProbeNormal) || ProbeElevation < 2 ||
			FVector::DotProduct(ProbeDirection, ProbeNormal) < 0.95
		) {
			bAccessible = false;
			break;
		}
		Minimum = FMath::Min(Minimum, ProbeElevation);
		Maximum = FMath::Max(Maximum, ProbeElevation);
	}
	// 近傍が歩けない候補を，基礎点数だけで採用しない終了
	if(!bAccessible) return;
	const double Relief = Maximum - Minimum;
	const double Scenic = FMath::Clamp((Relief - 1.5) / 3.5, 0.0, 1.0) * FMath::Clamp((12 - Relief) / 4, 0.0, 1.0);
	const std::shared_ptr<const PlanetCore::RiverTerrainField> Field = Planet->GetPublishedField();
	// 表示と異なる標高で景観を採点しない終了
	if(!Field) return;
	const PlanetCore::UnitDirection Unit = *PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 同じ不変場の標高専用照会で，法線の不要な視線評価を行う
	const std::function<double(PlanetCore::UnitDirection)> Height = [&Field](const PlanetCore::UnitDirection Probe) -> double {
		// 公開済の地形と同じ標高の返戻
		return Field->EvaluateElevation(Probe);
	};
	std::array<PlanetCore::ScenicView, 4> Views;
	double MaximumViewScore = 0;
	for(int32 AxisIndex = 0; AxisIndex < 4; ++AxisIndex) {
		const FVector Axis = Axes[AxisIndex];
		const std::optional<PlanetCore::ScenicView> View =
		PlanetCore::EvaluateScenicView(Unit, { Axis.X, Axis.Y, Axis.Z }, Radius, Elevation, LandingEyeHeightMeters, Height);
		// 視線の標本が欠けた地点を有利に扱わない終了
		if(!View) return;
		Views[AxisIndex] = *View;
		MaximumViewScore = FMath::Max(MaximumViewScore, View->Score);
	}
	// 樹木に依る減点前の上限で，採用され得ない地点を除外する
	const double PotentialScore = BaseScore + 0.12 * Scenic + OpenViewFactor * LandingScenicScoreWeight * MaximumViewScore;
	if(bDaylit == bBestLandingDaylit && PotentialScore <= BestLandingScore) return;
	// 当選し得る地点だけを散布器へ問い合わせる
	const std::optional<PlanetCore::PlanetScatter> Trees = PlanetCore::PlanetScatter::Create(
		{ static_cast<uint64>(Planet->Seed), 2, Planet->RadiusMeters, APlanetVegetation::TreeSpacingMeters }
	);
	if(!Trees) return;
	const std::optional<PlanetCore::ScatterQueryResult> NearbyTrees = Trees->Query(Unit, LandingSightlineMeters, 10000);
	if(!NearbyTrees) return;
	bool Blocked[4] = {};
	for(const PlanetCore::ScatterCandidate &Tree : NearbyTrees->Candidates) {
		const PlanetCore::Vec3 TreeDirection = Tree.Direction.Vector();
		const FVector Offset = (FVector(TreeDirection.X, TreeDirection.Y, TreeDirection.Z) - Direction) * Radius;
		// 身体に接する幹の有無は視線評価より優先する
		if(Offset.SizeSquared() < LandingTreeSafetyMeters * LandingTreeSafetyMeters) return;
		for(int32 AxisIndex = 0; AxisIndex < 4; ++AxisIndex) {
			const double Ahead = FVector::DotProduct(Offset, Axes[AxisIndex]);
			if(Ahead <= 0 || Ahead >= LandingSightlineMeters) continue;
			const FVector Right = FVector::CrossProduct(Direction, Axes[AxisIndex]);
			if(FMath::Abs(FVector::DotProduct(Offset, Right)) < 1.5 + 0.12 * Ahead) Blocked[AxisIndex] = true;
		}
	}
	double BestViewScore = -2;
	int32 BestViewAxis = 0;
	for(int32 AxisIndex = 0; AxisIndex < 4; ++AxisIndex) {
		const double ViewBonus = OpenViewFactor * LandingScenicScoreWeight * Views[AxisIndex].Score;
		const double ViewScore = ViewBonus - (Blocked[AxisIndex] ? LandingBlockedSightlinePenalty * ForestSuitability : 0);
		if(ViewScore > BestViewScore) {
			BestViewScore = ViewScore;
			BestViewAxis = AxisIndex;
		}
	}
	const double Score = BaseScore + 0.12 * Scenic + BestViewScore;
	if(bDaylit && !bBestLandingDaylit || bDaylit == bBestLandingDaylit && Score > BestLandingScore) {
		BestLandingScore = Score;
		bBestLandingDaylit = bDaylit;
		bBestSightlineBlocked = Blocked[BestViewAxis];
		BestSunDot = SunDot;
		BestLandingIndex = Index;
		BestLandingOffsetMeters = OffsetMeters;
		BestVisibleRiseMeters = Views[BestViewAxis].MaximumExposedRiseMeters;
		LandingDirection = Direction;
		LandingForward = Axes[BestViewAxis];
		LandingElevation = Elevation;
	}
	// 終了
	return;
}

/**
 * 固定した全球候補から，水面より高く緩やかな探索開始地点を選ぶ
 * @param Index ０～２０４７の候補番号
 */
void APlanetGameMode::ConsiderLandingCandidate(const int32 Index) {
	// 明示された固定候補と，全球の等面積に近い候補方向
	const int32 Requested = CVarPlanetLandingCandidate.GetValueOnGameThread();
	if(Requested > -1 && Requested < LandingCandidateCount && Index != Requested) return;
	const double Z = -1 + 2 * (Index + 0.5) / LandingCandidateCount, Angle = Index * 2.399963229728653;
	const double Width = std::sqrt(1 - Z * Z);
	const FVector Direction(Width *std::cos(Angle), Width *std::sin(Angle), Z);
	ConsiderLandingDirection(Direction, Index, 0);
	// 終了
	return;
}

/**
 * 大気・雲・採光距離を同じ縮尺へ揃え，小半径でも昼夜と光学的な厚さを保つ
 */
void APlanetGameMode::PrepareSky() {
	Planet->PrepareCelestialAtmosphere();
	// 終了
	return;
}

/**
 * 選択した地表を通常の照明の足元へ配置し，近景を準備する
 */
void APlanetGameMode::PrepareLandingSurface() {
	if(BestLandingScore < 0) {
		FailPreparation(TEXT("歩行を開始出来る陸上地点が見付かりませんでした"), TEXT("No valid landing candidate"));
		// 不正な開始位置を使わない終了
		return;
	}
	// 最良地点の正面が木で塞がる時だけ，同じ地形評価式で近傍の林間を探す
	const double LocalStartedSeconds = FPlatformTime::Seconds();
	if(bBestSightlineBlocked) {
		const FVector Center = LandingDirection;
		const int32 ParentIndex = BestLandingIndex;
		const double Radius = Planet->RadiusMeters;
		const FVector First = FVector::CrossProduct(Center, FVector::UpVector).GetSafeNormal();
		const FVector Second = FVector::CrossProduct(Center, First).GetSafeNormal();
		for(const double OffsetMeters : { 5.0, 12.0, 25.0, 50.0 }) for(int32 AngleIndex = 0; AngleIndex < 16; ++AngleIndex) {
			const double Angle = 2 * UE_DOUBLE_PI * AngleIndex / 16;
			const FVector Along = First * std::cos(Angle) + Second * std::sin(Angle);
			const FVector Direction = (Center * Radius + Along * OffsetMeters).GetSafeNormal();
			ConsiderLandingDirection(Direction, ParentIndex, OffsetMeters);
		}
	}
	LandingSearchCpuMilliseconds += (FPlatformTime::Seconds() - LocalStartedSeconds) * 1000;
	UE_LOG(
		LogTemp,
		Display,
		TEXT("Planet landing search: %d global candidates wall %.3f ms cpu %.3f ms"),
		LandingCandidateCount,
		(FPlatformTime::Seconds() - LandingSearchStartedSeconds) * 1000,
		LandingSearchCpuMilliseconds
	);
	const FVector Forward = LandingForward;
	UE_LOG(
		LogTemp,
		Display,
		TEXT("Planet landing candidate %d: score %.6f rise %.3f m sun %.3f day %d block %d offset %.1f m"),
		BestLandingIndex,
		BestLandingScore,
		BestVisibleRiseMeters,
		BestSunDot,
		bBestLandingDaylit ? 1 : 0,
		bBestSightlineBlocked ? 1 : 0,
		BestLandingOffsetMeters
	);
	const FQuat Rotation = FRotationMatrix::MakeFromXZ(Forward, LandingDirection).ToQuat().Inverse();
	const FVector Point = LandingDirection * ((Planet->RadiusMeters + LandingElevation) * 100);
	Planet->SetActorRotation(Rotation);
	Planet->SetActorLocation(-Rotation.RotateVector(Point));
	PrepareSky();
	Planet->ObserverMeters = LandingDirection * (Planet->RadiusMeters + LandingElevation + LandingEyeHeightMeters);
	Planet->bRefineNearSurface = true;
	Planet->LeafBudget = SurfaceLeafBudget;
	if(Planet->RequestGeneration()) {
		SurfaceRevision = Planet->RequestedRevision;
		ExplorationStage = EPlanetExplorationStage::Surface;
		PreparationMessage = TEXT("着地地点の地表を準備しています");
	} else FailPreparation(TEXT("着地地点の地表を準備出来ませんでした"), Planet->GenerationMessage);
	// 終了
	return;
}

/**
 * 表示済みの近景と同じ地形から，足元の衝突を準備する
 */
void APlanetGameMode::PrepareGround() {
	Ground = GetWorld()->SpawnActor<APlanetGroundPatch>();
	if(!Ground) {
		FailPreparation(TEXT("足場を確保出来ませんでした"), TEXT("Ground actor allocation failed"));
		// 確保失敗の終了
		return;
	}
	Ground->SetFlags(RF_Transient);
	Ground->Terrain = Planet;
	Ground->AnchorDirection = LandingDirection;
	if(Ground->RequestPatch()) {
		ExplorationStage = EPlanetExplorationStage::Ground;
		PreparationMessage = TEXT("着地地点の足場を準備しています");
	} else FailPreparation(TEXT("着地地点の足場を準備出来ませんでした"), Ground->Status);
	// 終了
	return;
}

/**
 * 実際の衝突と身体の空間を確認してから，操作者へ探索用の身体を渡す
 */
void APlanetGameMode::EnterExploration() {
	// 操作者の接続を待つ終了
	if(!IsValid(ExplorerController)) return;
	Explorer = APlanetExplorerCharacter::SpawnOnPreparedGround(Ground, LandingDirection, LandingForward);
	if(!Explorer) {
		FailPreparation(TEXT("着地地点の足場又は身体の空間を確認出来ませんでした"), Ground->Status);
		// 足場が確認出来ない身体を公開しない終了
		return;
	}
	if(APlanetVegetation *const Vegetation = GetWorld()->SpawnActor<APlanetVegetation>()) Vegetation->Initialize(Planet, Explorer);
	ExplorerController->Possess(Explorer);
	ExplorerController->SetInputMode(FInputModeGameOnly());
	ExplorerController->bShowMouseCursor = false;
	Planet->bFollowPlayerCamera = true;
	ExplorationStage = EPlanetExplorationStage::Ready;
	PreparationMessage.Empty();
	UE_LOG(LogTemp, Display, TEXT("Planet exploration ready"));
	#if CSV_PROFILER
	int32 ProfileFrames = 0;
	if(
		FParse::Value(FCommandLine::Get(), TEXT("PlanetCsvFramesOnReady="), ProfileFrames) && ProfileFrames > 0 &&
		ProfileFrames < 3601 && !FCsvProfiler::Get()->IsCapturing()
	) FCsvProfiler::Get()->BeginCapture(ProfileFrames);
	int32 RunFrames = 0;
	if(FParse::Value(FCommandLine::Get(), TEXT("PlanetRunFramesOnReady="), RunFrames) && RunFrames > 0 && RunFrames < 3601) {
		AutomaticRunFramesRemaining = RunFrames;
		AutomaticRunRequestedFrames = RunFrames;
		AutomaticRunOrigin = Explorer->GetActorLocation();
		AutomaticRunInitialCoverageMisses = Explorer->CoverageMissFrames;
		AutomaticRunInitialGroundRefreshRequests = Explorer->GroundRefreshRequests;
		Explorer->SetMoveInput(1, 0, true);
		UE_LOG(
			LogTemp,
			Display,
			TEXT("Planet benchmark run started: frames=%d speed_mps=%.3f"),
			RunFrames,
			Explorer->RunSpeedMetersPerSecond
		);
	}
	#endif
	// 終了
	return;
}

/**
 * 生成・地表・足場の完了を順に確認し，探索を開始する
 * @param DeltaSeconds 前回の更新からの秒数
 */
void APlanetGameMode::Tick(const float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	if(ExplorationStage == EPlanetExplorationStage::Ready) {
		#if CSV_PROFILER
		if(AutomaticRunFramesRemaining > 0 && IsValid(Explorer)) {
			--AutomaticRunFramesRemaining;
			if(!AutomaticRunFramesRemaining) {
				Explorer->SetMoveInput(0, 0, false);
				UE_LOG(
					LogTemp,
					Display,
					TEXT("Planet benchmark run complete: frames=%d chord_meters=%.3f coverage_misses=%lld ground_refresh=%lld"),
					AutomaticRunRequestedFrames,
					FVector::Dist(AutomaticRunOrigin, Explorer->GetActorLocation()) / 100,
					Explorer->CoverageMissFrames - AutomaticRunInitialCoverageMisses,
					Explorer->GroundRefreshRequests - AutomaticRunInitialGroundRefreshRequests
				);
			}
		}
		#endif
		// 探索中の生成と身体更新は各 Actor が行う終了
		return;
	}
	// 準備失敗後に操作を開始しない終了
	if(ExplorationStage == EPlanetExplorationStage::Failed) return;
	if(!IsValid(Planet)) {
		FailPreparation(TEXT("準備中の惑星が失われました"), TEXT("Planet actor is no longer valid"));
		// 消滅した惑星を使わない終了
		return;
	}
	if(FPlatformTime::Seconds() - PreparationStartedSeconds > 240) {
		FailPreparation(TEXT("惑星の準備が時間内に終わりませんでした"), Planet->GenerationMessage + TEXT(" | ") + Planet->ClimateMessage);
		// 無期限に準備中と表示しない終了
		return;
	}
	if(ExplorationStage == EPlanetExplorationStage::Planet) {
		if(Planet->bClimateReady && !Planet->bGenerationBusy) {
			ExplorationStage = EPlanetExplorationStage::LandingSite;
			// 全候補を同じ太陽方向で評価し，走査中の時間経過に左右されない様にする
			LandingSunDirection = Planet->GetActorQuat().UnrotateVector(Planet->SunDirectionWorld).GetSafeNormal();
			LandingSearchStartedSeconds = FPlatformTime::Seconds();
			PreparationMessage = TEXT("歩行を開始する地点を選んでいます");
		} else if(!Planet->bGenerationBusy && !Planet->PublishedRevision) {
			FailPreparation(TEXT("惑星の地形を準備出来ませんでした"), Planet->GenerationMessage);
		}
	} else if(ExplorationStage == EPlanetExplorationStage::LandingSite) {
		const double ScanStartedSeconds = FPlatformTime::Seconds();
		const int32 End = FMath::Min(LandingCandidate + 64, LandingCandidateCount);
		while(LandingCandidate < End && FPlatformTime::Seconds() - ScanStartedSeconds < 0.002) {
			ConsiderLandingCandidate(LandingCandidate++);
		}
		LandingSearchCpuMilliseconds += (FPlatformTime::Seconds() - ScanStartedSeconds) * 1000;
		if(LandingCandidate == LandingCandidateCount) PrepareLandingSurface();
	} else if(ExplorationStage == EPlanetExplorationStage::Surface && !Planet->bGenerationBusy) {
		if(Planet->PublishedRevision == SurfaceRevision && Planet->VisibleSurfaceRadiusMeters >= LandingSurfaceRadiusMeters) {
			PrepareGround();
		} else FailPreparation(TEXT("着地地点に必要な地表精度を確保出来ませんでした"), Planet->GenerationMessage);
	} else if(ExplorationStage == EPlanetExplorationStage::Ground && !Ground->bBusy) {
		if(Ground->PublishedRevision) EnterExploration();
		else FailPreparation(TEXT("着地地点の足場を公開出来ませんでした"), Ground->Status);
	}
	// 終了
	return;
}
