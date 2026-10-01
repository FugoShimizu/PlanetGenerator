#include "Core/PlanetCelestial.h"
#include "Core/PlanetFlowChannelField.h"
#include "Core/PlanetFlowWatershed.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetSeasonalMap.h"
#include "PlanetCelestialComponent.h"
#include "PlanetCharacterMovement.h"
#include "PlanetExplorerCharacter.h"
#include "PlanetFlowTerrain.h"
#include "PlanetGroundPatch.h"
#include "PlanetSphericalSurfacePreview.h"
#include "PlanetTerrain.h"
#include "PlanetVegetation.h"
#include "PlanetWatershedPreparation.h"
#include "Camera/CameraComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/AutomationTest.h"
#include "ProceduralMeshComponent.h"
#include "UObject/UObjectGlobals.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/** 所有するゲーム世界で実際の身体を歩かせ，共有場と従来場を照合する */
	class FFlowWalkingCommand final : public IAutomationLatentCommand {
	private:

		// 検査の所有物と，実時間を越えて物理を先へ進めない時計
		FAutomationTestBase &Test;
		UWorld *World;
		const bool UsesFlow;
		const bool UsesRenderer;
		const bool RunsLongRoute;
		const bool UsesNetwork;
		const bool UsesLocation;
		std::shared_ptr<const PlanetCore::OceanConnectivityField> LocationOcean;
		std::shared_ptr<const PlanetCore::FlowNetworkField> LocationField;
		APlanetSphericalSurfacePreview *Preview = nullptr;
		APlanetWatershedPreparation *Preparation = nullptr;
		unsigned PreparationPolls = 0;
		double MaximumPreparationPollMilliseconds = 0;
		APlanetTerrain *Terrain = nullptr;
		APlanetGroundPatch *Ground = nullptr;
		APlanetExplorerCharacter *Character = nullptr;
		APlanetFlowTerrain *Renderer = nullptr;
		APlanetVegetation *Vegetation = nullptr;
		APlanetGroundPatch *PlantGround = nullptr;
		APlanetExplorerCharacter *PlantBody = nullptr;
		FVector PlantDirection = FVector::ZeroVector;
		int64 PlantGrassRevision = 0;
		int64 PlantTreeRevision = 0;
		FVector Anchor = FVector::ZeroVector;
		FVector StartPosition = FVector::ZeroVector;
		FVector PreviousPosition = FVector::ZeroVector;
		double Started = 0;
		double LastTick = 0;
		double TravelMeters = 0;
		double MaximumGroundErrorMeters = 0;
		double MaximumRenderGapMeters = 0;
		double MaximumWaterErrorMeters = 0;
		double MaximumTexturePhaseErrorMeters = 0;
		double MaximumGravityError = 0;
		double MaximumGravityMagnitudeError = 0;
		double MinimumMarginMeters = 1E9;
		double JumpApexMeters = 0;
		unsigned Stage = 0;
		unsigned Frames = 0;
		unsigned AirFrames = 0;
		unsigned RetainedFrames = 0;
		unsigned CameraChecks = 0;
		unsigned RenderRetainedFrames = 0;
		unsigned RenderSamples = 0;
		unsigned WaterSamples = 0;
		unsigned SeasonalSamples = 0;
		double MaximumClimateDirectionError = 0;
		int64 InitialRenderRevision = 0;
		bool HasLanded = false;
		void Cleanup();
		bool PrepareLocation();
		void CheckCameras();
		void CheckSeasonalOnlyUpdate();
		void CheckCelestialClock();
		void InspectVegetation();
		bool BeginDryVegetation();
		bool BeginWalking();
		bool InspectRender(const FVector Surface, const FVector Up, const FVector GroundImpact);
		bool InspectBody();
		bool InspectWater(const FVector Surface, const FVector Up);
		void Finish(const bool ReleaseWorld=true);

	public:

		virtual bool Update() override;

		FFlowWalkingCommand(
			FAutomationTestBase &Owner,
			UWorld *const TestWorld,
			const bool UseFlow,
			const bool UseRenderer=false,
			const bool RunLongRoute=false,
			const bool UseNetwork=false,
			const bool UseLocation=false
		);

		virtual ~FFlowWalkingCommand() override;
	};

	/**
	 * 所有する世界とアクターを，世界の登録を維持した順序で解放する
	 */
	void FFlowWalkingCommand::Cleanup() {
		if(Preparation) Preparation->CancelPreparation();
		if(World) {
			World->EndPlay(EEndPlayReason::Quit);
			World->DestroyWorld(false);
			if(GEngine) {
				GEngine->DestroyWorldContext(World);
				Test.TestNull(TEXT("歩行世界の登録を解放"), GEngine->GetWorldContextFromWorld(World));
			}
			Test.TestFalse(TEXT("歩行世界を根参照から解放"), World->IsRooted());
		}
		World = nullptr;
		Preview = nullptr;
		Preparation = nullptr;
		Terrain = nullptr;
		Ground = nullptr;
		Character = nullptr;
		Renderer = nullptr;
		Vegetation = nullptr;
		PlantGround = nullptr;
		PlantBody = nullptr;
		LocationField.reset();
		LocationOcean.reset();
		// 終了
		return;
	}

	/**
	 * 普通の陸上地点から，診断アクター無しで水系と同じ局所足場を準備する
	 * @return 全準備と足場の要求を受理した場合に真；不成立なら偽
	 */
	bool FFlowWalkingCommand::PrepareLocation() {
		// 完了した定義の共有取得と，準備アクターから独立した寿命の確認
		std::shared_ptr<const PlanetCore::PreparedFlowWatershed> Watershed = Preparation->GetPrepared();
		// 部分的な水系を足場として使用しない返戻
		if(!Test.TestTrue(TEXT("普通の陸上地点から全河道を準備"), Watershed != nullptr)) return false;
		Test.TestFalse(TEXT("公開済の定義へ別要求を重ねない"), Preparation->RequestPreparation(7, 1000, FVector::UpVector));
		Test.AddInfo(
			FString::Printf(
				TEXT("Location async preparation: milliseconds=%.9g polls=%u maxPollMs=%.9g"),
				Preparation->PreparationMilliseconds,
				PreparationPolls,
				MaximumPreparationPollMilliseconds
			)
		);
		Test.TestTrue(TEXT("準備中に主スレッドが複数回応答"), PreparationPolls > 1);
		LocationField = Watershed->Field;
		LocationOcean = Watershed->Ocean;
		const double Radius = LocationField->GetRadiusMeters();
		Preparation->CancelPreparation();
		Test.TestFalse(TEXT("解放した準備器は作業を保持しない"), Preparation->IsBusy || Preparation->IsReady);
		Test.TestTrue(TEXT("解放した準備器の公開参照は空"), Preparation->GetPrepared() == nullptr);
		Test.TestTrue(TEXT("所有する準備アクターを破棄"), Preparation->Destroy());
		Preparation = nullptr;
		// 発見した流出口の百メートル上流を歩行開始位置に選定
		std::optional<PlanetCore::UnitDirection> Start;
		for(const PlanetCore::FlowWaterGeometry &Water : LocationField->GetWaterGeometries()) {
			const std::optional<PlanetCore::FlowRouteProjection> Pass = Water.Route.FindFootpoint(Watershed->InitialPass, 0.1);
			if(!Pass || Pass->DistanceMeters < 100) continue;
			const std::optional<PlanetCore::TerrainFlowCurveSample> Point = Water.Route.Sample(Pass->DistanceMeters - 100);
			if(Point) Start = Point->Direction;
			break;
		}
		// 準備結果の局所寿命を越える共有所有の確認
		Watershed.reset();
		Test.TestNull(TEXT("診断アクターを生成しない"), Preview);
		// 確認した河道を発見出来ない場合の返戻
		if(!Test.TestTrue(TEXT("発見した流出口から歩行地点を選定"), Start.has_value())) return false;
		const PlanetCore::Vec3 Direction = Start->Vector();
		Anchor = FVector(Direction.X, Direction.Y, Direction.Z);
		const std::optional<PlanetCore::FlowChannelSection> Sample = LocationField->Evaluate(*Start);
		// 不整合な河床へ身体を配置しない返戻
		if(!Test.TestTrue(TEXT("発見した河道の開始標高"), Sample.has_value())) return false;
		const std::optional<PlanetCore::GroundFrame> Frame =
		PlanetCore::GroundFrame::Create(*Start, Radius, Sample->Terrain.ElevationMeters);
		// 座標系の欠落への返戻
		if(!Test.TestTrue(TEXT("地点の局所座標"), Frame.has_value())) return false;
		const auto Vector = [](const PlanetCore::Vec3 V) -> FVector {
			return { V.X, V.Y, V.Z };
		};
		const FMatrix Basis(Vector(Frame->East), Vector(Frame->North), Vector(Frame->Up), FVector::ZeroVector);
		const FQuat Rotation = FRotator(17, 31, -9).Quaternion() * FQuat(Basis).Inverse();
		const FVector Origin =
		FVector(34567, -45678, 56789) - Rotation.RotateVector(Anchor * (Radius + Sample->Terrain.ElevationMeters) * 100);
		// 同じ所有場を実衝突へ渡した結果の返戻
		return Test.TestTrue(TEXT("診断面無しの実足場を要求"), Ground->RequestFlowPatch(LocationField, Anchor, FTransform(Rotation, Origin)));
	}

	/**
	 * 視点の切替が身体を移動させず，カメラを一つだけ有効にする事を確認する
	 */
	void FFlowWalkingCommand::CheckCameras() {
		const FVector Position = Character->GetActorLocation();
		for(const EPlanetViewMode Mode : { EPlanetViewMode::FirstPerson, EPlanetViewMode::ThirdPerson, EPlanetViewMode::Overview }) {
			Character->SetViewMode(Mode);
			Test.TestEqual(TEXT("視点切替が身体を移動させない"), Character->GetActorLocation(), Position);
			TArray<UCameraComponent *> Cameras;
			Character->GetComponents(Cameras);
			unsigned Active = 0;
			for(const UCameraComponent *const Camera : Cameras) {
				if(Camera->IsActive()) ++Active;
				const FPostProcessSettings &Exposure = Camera->PostProcessSettings;
				Test.TestTrue(TEXT("三視点で露出の下限を有効化"), Exposure.bOverride_AutoExposureMinBrightness);
				Test.TestTrue(TEXT("三視点で露出の上限を有効化"), Exposure.bOverride_AutoExposureMaxBrightness);
				Test.TestEqual(TEXT("夜を過増幅しない EV100 下限"), Exposure.AutoExposureMinBrightness, 0.0F);
				Test.TestEqual(TEXT("昼の EV100 上限"), Exposure.AutoExposureMaxBrightness, 16.0F);
			}
			Test.TestEqual(TEXT("有効な視点が一つ"), Active, 1U);
			++CameraChecks;
		}
		Character->SetViewMode(EPlanetViewMode::FirstPerson);
		// 終了
		return;
	}

	/**
	 * 同じ公開面の二つの季節を確認し，頂点・衝突・生成要求が不変である事を検査する
	 */
	void FFlowWalkingCommand::CheckSeasonalOnlyUpdate() {
		Test.TestTrue(TEXT("同じ元海陸の気候が公開済"), Renderer->bClimateReady);
		Test.TestEqual(TEXT("待機要求間でも気候は一回だけ準備"), Renderer->ClimatePreparationCount, 1);
		UProceduralMeshComponent *const Mesh = Renderer->GetVisibleMesh();
		UMaterialInstanceDynamic *const Material = Mesh ? Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(0)) : nullptr;
		// 未公開の季節材質を検査済としない終了
		if(!Test.TestNotNull(TEXT("季節用の実材質"), Material)) return;
		const UTexture2D *const Texture = Cast<UTexture2D>(Material->K2_GetTextureParameterValue(TEXT("ClimateAtlas")));
		// 欠落した配列への終了
		if(!Test.TestNotNull(TEXT("保存しない季節配列"), Texture)) return;
		Test.TestTrue(TEXT("配列は一時資産"), Texture->HasAnyFlags(RF_Transient));
		Test.TestEqual(TEXT("気候配列の幅"), Texture->GetSizeX(), static_cast<int32>(PlanetCore::PlanetSeasonalMap::Width));
		Test.TestEqual(
			TEXT("気候配列の全層"),
			Texture->GetSizeY(),
			static_cast<int32>(PlanetCore::PlanetSeasonalMap::Height * PlanetCore::PlanetSeasonalMap::Layers)
		);
		const int64 Requested = Renderer->RequestedRevision, Published = Renderer->PublishedRevision;
		const int64 Collision = Ground->PublishedRevision;
		const FProcMeshSection *const Section = Mesh->GetProcMeshSection(0);
		// 頂点が無い面への終了
		if(!Test.TestTrue(TEXT("季節変更前の表示頂点"), Section && !Section->ProcVertexBuffer.IsEmpty())) return;
		const TArray<FProcMeshVertex> Vertices = Section->ProcVertexBuffer;
		const TArray<uint32> Indices = Section->ProcIndexBuffer;
		for(const double Phase : { 0.0, 0.5 }) {
			Renderer->SolarYearPhase = Phase;
			Renderer->Tick(0);
			Test.TestEqual(TEXT("同じ材質の年周位相だけ変更"), Material->K2_GetScalarParameterValue(TEXT("SolarYearPhase")), static_cast<float>(Phase));
			Test.TestEqual(TEXT("気候被覆を有効化"), Material->K2_GetScalarParameterValue(TEXT("ClimateEnabled")), 1.0F);
			Test.TestTrue(TEXT("季節変更で気候配列を作り直さない"), Material->K2_GetTextureParameterValue(TEXT("ClimateAtlas")) == Texture);
		}
		// 非有限な位相を材質へ渡さず，最後の有効値を保持
		Renderer->SolarYearPhase = std::numeric_limits<double>::quiet_NaN();
		Renderer->Tick(0);
		Test.TestEqual(TEXT("非有限な季節で最後の有効な被覆を保持"), Material->K2_GetScalarParameterValue(TEXT("SolarYearPhase")), 0.5F);
		Renderer->SolarYearPhase = 0.5;
		const FProcMeshSection *const After = Renderer->GetVisibleMesh()->GetProcMeshSection(0);
		bool Same = After && After->ProcIndexBuffer == Indices && After->ProcVertexBuffer.Num() == Vertices.Num();
		if(Same) for(int32 I = 0; I < Vertices.Num(); ++I) {
			Same =
			Same && After->ProcVertexBuffer[I].Position == Vertices[I].Position && After->ProcVertexBuffer[I].Normal == Vertices[I].Normal;
		}
		Test.TestTrue(TEXT("季節変更で表示頂点と接続を変えない"), Same);
		Test.TestEqual(TEXT("季節変更で生成を要求しない"), Renderer->RequestedRevision, Requested);
		Test.TestEqual(TEXT("季節変更で表示世代を変えない"), Renderer->PublishedRevision, Published);
		Test.TestEqual(TEXT("季節変更で衝突世代を変えない"), Ground->PublishedRevision, Collision);
		Test.AddInfo(
			FString::Printf(
				TEXT("Flow climate prepared=%d milliseconds=%.12g atlas=%d/%d seasonalOnly=%u"),
				Renderer->ClimatePreparationCount,
				Renderer->LastClimatePreparationMilliseconds,
				Texture->GetSizeX(),
				Texture->GetSizeY(),
				static_cast<unsigned>(Same)
			)
		);
		// 終了
		return;
	}

	/**
	 * 実際の公開地表の身体へ時計を接続し，昼夜・季節だけが変わる事を確認する
	 */
	void FFlowWalkingCommand::CheckCelestialClock() {
		// 実際の開始直後の有効視点を，切替操作より先に照合
		TArray<UCameraComponent *> InitialCameras;
		PlantBody->GetComponents(InitialCameras);
		int32 InitiallyActive = 0;
		for(const UCameraComponent *const Camera : InitialCameras) if(Camera->IsActive()) ++InitiallyActive;
		Test.TestEqual(TEXT("新しい身体も開始直後から単一視点"), InitiallyActive, 1);
		Renderer->Explorer = PlantBody;
		UPlanetCelestialComponent *const Clock = Renderer->CelestialClock;
		Clock->bAdvanceTime = false;
		Clock->InitialUtc = FDateTime(2026, 6, 21, 12);
		Test.TestTrue(TEXT("公開河床の共通時計を開始"), Renderer->StartCelestialClock());
		const std::optional<FPlanetFlowEnvironment> Environment = Renderer->GetPublishedEnvironment();
		// 有効な公開環境でだけ地表と時計を比較する終了
		if(!Test.TestTrue(TEXT("時計も同じ公開環境を参照"), Environment.has_value())) return;
		const FVector Eye = PlantBody->GetActorLocation() + PlantBody->GetActorUpVector() * 64;
		const FVector Local = Environment->PlanetTransform.InverseTransformPosition(Eye) / 100;
		const FVector Direction = Local.GetSafeNormal();
		const PlanetCore::CelestialObserver Observer {
			std::asin(Direction.Z) * 180 / std::numbers::pi,
			std::atan2(Direction.Y, Direction.X) * 180 / std::numbers::pi,
			Local.Length() - Environment->Settings.RadiusMeters
		};
		const FVector ExpectedObserver(Observer.LatitudeDegrees, Observer.LongitudeDegrees, Observer.HeightMeters);
		const int64 RenderRevision = Renderer->PublishedRevision, RequestRevision = Renderer->RequestedRevision;
		const int64 GroundRevision = PlantGround->PublishedRevision, GrassRevision = Vegetation->PublishedRevision;
		const int64 TreeRevision = Vegetation->TreePublishedRevision;
		const FProcMeshSection Surface = *Renderer->GetVisibleMesh()->GetProcMeshSection(0);
		const FProcMeshSection Water = *Renderer->GetVisibleWaterMesh()->GetProcMeshSection(0);
		const std::array<FDateTime, 5> Dates {
			FDateTime(2026, 6, 21, 8),
			FDateTime(2026, 6, 21, 12),
			FDateTime(2026, 6, 21, 14),
			FDateTime(2026, 6, 22, 0),
			FDateTime(2026, 12, 21, 12)
		};
		bool SawDay = false, SawTwilight = false, SawNight = false;
		double MaxDirectionError = 0;
		for(const FDateTime Date : Dates) {
			Test.TestTrue(TEXT("日時の変更を受理"), Clock->SetSimulationUtc(Date));
			Renderer->Tick(0);
			Vegetation->Tick(0);
			Test.AddInfo(
				FString::Printf(
					TEXT("Flow exposure: utc=%s altitude=%.9g bias=%.9g"),
					*Date.ToIso8601(),
					Clock->SunAltitudeDegrees,
					InitialCameras[0]->PostProcessSettings.AutoExposureBias
				)
			);
			// 同じ太陽高度の露出が，非選択のカメラも含めて届く事を確認
			for(const UCameraComponent *const Camera : InitialCameras) {
				const FPostProcessSettings &Exposure = Camera->PostProcessSettings;
				Test.TestEqual(TEXT("水系の夜も EV100 下限を維持"), Exposure.AutoExposureMinBrightness, 0.0F);
				Test.TestEqual(TEXT("水系の昼も EV100 上限を維持"), Exposure.AutoExposureMaxBrightness, 16.0F);
				Test.TestTrue(
					TEXT("薄明の補正は減光一段以内で有限"),
					FMath::IsFinite(Exposure.AutoExposureBias) && Exposure.AutoExposureBias >= -1.0F && Exposure.AutoExposureBias <= 0.0F
				);
				if(Clock->SunAltitudeDegrees <= -20.0) {
					SawNight = true;
					Test.TestEqual(TEXT("夜の共通露出へ更新"), Exposure.AutoExposureBias, 0.0F);
				} else if(Clock->SunAltitudeDegrees >= 20.0) {
					SawDay = true;
					Test.TestEqual(TEXT("日中の自動露出を追加で減光しない"), Exposure.AutoExposureBias, 0.0F);
				} else {
					SawTwilight = true;
					Test.TestTrue(TEXT("薄明の減光を保持"), Exposure.AutoExposureBias < 0);
				}
			}
			const std::optional<PlanetCore::CelestialState> Expected =
			PlanetCore::PlanetCelestial::Evaluate((Date - FDateTime(2000, 1, 1, 12)).GetTotalDays(), Observer);
			// 天体暦の参照地点が有効な場合だけ方向を比較
			if(Expected) {
				const double Longitude = Observer.LongitudeDegrees * std::numbers::pi / 180;
				const FVector East(-std::sin(Longitude), std::cos(Longitude), 0);
				const FVector North = FVector::CrossProduct(Direction, East).GetSafeNormal();
				const std::array<PlanetCore::Vec3, 2> ExpectedDirections { Expected->Sun.Direction, Expected->Moon.Direction };
				const std::array<FVector, 2> Actual { Clock->SunDirectionWorld, Clock->MoonDirectionWorld };
				for(unsigned I = 0; I < 2; ++I) {
					const PlanetCore::Vec3 D = ExpectedDirections[I];
					const FVector WorldDirection =
					Environment->PlanetTransform.TransformVectorNoScale(East * D.X + North * D.Y + Direction * D.Z).GetSafeNormal();
					MaxDirectionError = FMath::Max(MaxDirectionError, FVector::Distance(Actual[I], WorldDirection));
				}
				Test.TestEqual(TEXT("地面も同じ日時の季節"), Renderer->SolarYearPhase, Expected->SolarYearPhase);
			} else Test.AddError(TEXT("時計の参照地点を天体暦が受理しない"));
			Test.TestTrue(TEXT("三視点に依らない身体の地球参照"), Clock->EarthObserverDegreesMeters.Equals(ExpectedObserver, 1E-6));
		}
		Test.TestTrue(TEXT("公開地表から太陽・月の方向へ同じ姿勢を反映"), MaxDirectionError < 1E-10);
		const FDateTime LastValid = Clock->SimulationUtc;
		Test.TestFalse(TEXT("年代外の日時を拒否"), Clock->SetSimulationUtc(FDateTime(1800, 1, 1)));
		Test.TestEqual(TEXT("無効な日時では最後の日時を保持"), Clock->SimulationUtc.GetTicks(), LastValid.GetTicks());
		Clock->bAdvanceTime = true;
		Clock->TimeScale = 60;
		Clock->Advance(0.5F);
		Test.TestEqual(TEXT("実時間から共通日時を加算"), (Clock->SimulationUtc - LastValid).GetTotalSeconds(), 30.0);
		const FDateTime BeforeInvalid = Clock->SimulationUtc;
		Clock->TimeScale = std::numeric_limits<double>::infinity();
		Clock->Advance(1);
		Test.TestFalse(TEXT("不正な倍率で停止"), Clock->bCelestialClockActive);
		Test.TestEqual(TEXT("不正な倍率を時刻へ加えない"), Clock->SimulationUtc.GetTicks(), BeforeInvalid.GetTicks());
		Test.TestEqual(TEXT("日時は描画再生成を要求しない"), Renderer->RequestedRevision, RequestRevision);
		Test.TestEqual(TEXT("日時は地表の世代を変えない"), Renderer->PublishedRevision, RenderRevision);
		Test.TestEqual(TEXT("日時は足場の世代を変えない"), PlantGround->PublishedRevision, GroundRevision);
		Test.TestEqual(TEXT("日時は草の配置を変えない"), Vegetation->PublishedRevision, GrassRevision);
		Test.TestEqual(TEXT("日時は樹木の配置を変えない"), Vegetation->TreePublishedRevision, TreeRevision);
		const std::array<const FProcMeshSection *, 2> Before { &Surface, &Water };
		const std::array<const FProcMeshSection *, 2> After {
			Renderer->GetVisibleMesh()->GetProcMeshSection(0),
			Renderer->GetVisibleWaterMesh()->GetProcMeshSection(0)
		};
		for(unsigned Part = 0; Part < 2; ++Part) {
			Test.TestTrue(TEXT("日時は三角形を変えない"), Before[Part]->ProcIndexBuffer == After[Part]->ProcIndexBuffer);
			Test.TestEqual(TEXT("日時は頂点数を変えない"), Before[Part]->ProcVertexBuffer.Num(), After[Part]->ProcVertexBuffer.Num());
			for(int32 I = 0; I < Before[Part]->ProcVertexBuffer.Num() && I < After[Part]->ProcVertexBuffer.Num(); ++I) {
				Test.TestEqual(TEXT("日時は頂点を変えない"), Before[Part]->ProcVertexBuffer[I].Position, After[Part]->ProcVertexBuffer[I].Position);
				Test.TestEqual(TEXT("日時は法線を変えない"), Before[Part]->ProcVertexBuffer[I].Normal, After[Part]->ProcVertexBuffer[I].Normal);
			}
		}
		// 地形不変の検査後に視点を変え，時計の観測地点が身体から離れない事を確認
		Clock->TimeScale = 60;
		Clock->bAdvanceTime = false;
		Test.TestTrue(TEXT("停止後も有効な日時で再開"), Clock->SetSimulationUtc(LastValid));
		for(const EPlanetViewMode Mode : { EPlanetViewMode::ThirdPerson, EPlanetViewMode::Overview, EPlanetViewMode::FirstPerson }) {
			PlantBody->SetViewMode(Mode);
			Renderer->Tick(0);
			Test.TestTrue(TEXT("視点を離しても身体の観測地点を保持"), Clock->EarthObserverDegreesMeters.Equals(ExpectedObserver, 1E-6));
			Test.TestEqual(TEXT("視点切替で日時を初期化しない"), Clock->SimulationUtc.GetTicks(), LastValid.GetTicks());
		}
		Test.TestTrue(TEXT("昼・薄明・夜の全状態を実日時で確認"), SawDay && SawTwilight && SawNight);
		Test.AddInfo(
			FString::Printf(
				TEXT("FlowCelestial dates=%u views=3 exposureCameras=3 initialActiveCameras=1 ")
				TEXT("latitude=%.12g longitude=%.12g height=%.12g directionError=%.12g immutableSections=2"),
				static_cast<unsigned>(Dates.size()),
				ExpectedObserver.X,
				ExpectedObserver.Y,
				ExpectedObserver.Z,
				MaxDirectionError
			)
		);
		// 終了
		return;
	}

	/**
	 * 元の失敗地点の水深を記録し，同じ水系の乾いた岸に別の実足場を用意する
	 * @return 配置可能な岸の準備を要求した場合に真
	 */
	bool FFlowWalkingCommand::BeginDryVegetation() {
		const std::optional<FPlanetFlowEnvironment> Environment = Renderer->GetPublishedEnvironment();
		// 不整合な環境へ進めない返戻
		if(!Test.TestTrue(TEXT("配置試験の公開環境"), Environment.has_value())) return false;
		const FVector Local = Environment->PlanetTransform.InverseTransformPosition(Character->GetActorLocation()) * 0.01;
		const FVector Up = Local.GetSafeNormal();
		const FVector East = FVector::CrossProduct(FVector::UpVector, Up).GetSafeNormal();
		const FVector North = FVector::CrossProduct(Up, East);
		const double Radius = Environment->Settings.RadiusMeters;
		double MinimumDepth = 1E9;
		unsigned Wet = 0;
		// 実際の候補域を独立な二十五方向で調べ，棄却理由を保存
		for(int X = -2; X < 3; ++X) for(int Y = -2; Y < 3; ++Y) {
			const FVector P = Up * Radius + East * (X * 7) + North * (Y * 7);
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector({ P.X, P.Y, P.Z });
			const std::optional<PlanetCore::FlowChannelSection> V = Environment->Surface->Evaluate(D);
			const std::optional<double> Support = Environment->Surface->WaterSupport(D);
			if(V && Support && *Support > 0 && V->HasWater) {
				const double Depth = V->Water.ElevationMeters - V->Terrain.ElevationMeters;
				MinimumDepth = std::min(MinimumDepth, Depth);
				if(Depth > 0) ++Wet;
			}
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Original vegetation fixture: grass=%d trees=%d wetSamples=%u/25 minDepth=%.12g"),
				Vegetation->GrassInstanceCount,
				Vegetation->TreeInstanceCount,
				Wet,
				MinimumDepth
			)
		);
		Test.TestEqual(TEXT("元の配置試験地点は水没"), Wet, 25U);
		Test.TestEqual(TEXT("水没地点に草を置かない"), Vegetation->GrassInstanceCount, 0);
		Test.TestEqual(TEXT("水没地点に木を置かない"), Vegetation->TreeInstanceCount, 0);
		bool Found = false;
		unsigned Probes = 0;
		for(const double Distance : { 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0, 2048.0, 4096.0 }) {
			for(unsigned I = 0; I < 16 && !Found; ++I) {
				const double Angle = I * std::numbers::pi / 8;
				const FVector P = (Up * Radius + (East * std::cos(Angle) + North * std::sin(Angle)) * Distance).GetSafeNormal();
				const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector({ P.X, P.Y, P.Z });
				const std::optional<PlanetCore::FlowChannelSection> V = Environment->Surface->Evaluate(D);
				const std::optional<double> Support = Environment->Surface->WaterSupport(D);
				++Probes;
				if(!V || !Support || V->Terrain.ElevationMeters <= Environment->Settings.SeaLevelMeters + 1) continue;
				if(*Support > 0 && (!V->HasWater || V->Terrain.ElevationMeters <= V->Water.ElevationMeters + 1)) continue;
				if(V->Terrain.TangentialGradientPerRadian.Length() / (Radius + V->Terrain.ElevationMeters) > 0.2) continue;
				const std::optional<std::array<double, 8>> Cover = Environment->Seasons->Sample(D, 0.5);
				if(!Cover || (*Cover)[0] + (*Cover)[1] < 0.2) continue;
				PlantDirection = P;
				Found = true;
			}
			if(Found) break;
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Dry vegetation site: probes=%u direction=%.17g,%.17g,%.17g"),
				Probes,
				PlantDirection.X,
				PlantDirection.Y,
				PlantDirection.Z
			)
		);
		// 有効な岸を探せない場合の返戻
		if(!Test.TestTrue(TEXT("同じ場に乾いた緩斜面を発見"), Found)) return false;
		FActorSpawnParameters Spawn;
		Spawn.ObjectFlags |= RF_Transient;
		PlantGround = World->SpawnActor<APlanetGroundPatch>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn);
		// 不成立な足場への返戻
		if(!Test.TestNotNull(TEXT("岸の足場"), PlantGround)) return false;
		Test.TestTrue(
			TEXT("同じ河床場で岸の衝突を要求"),
			PlantGround->RequestFlowPatch(Environment->Surface, PlantDirection, Environment->PlanetTransform)
		);
		Stage = 7;
		// 新しい実足場の準備開始の返戻
		return !Test.HasAnyErrors();
	}

	/**
	 * 実際の個体の根と水域を独立に照合し，季節変更で根の位置が変わらない事を検査する
	 */
	void FFlowWalkingCommand::InspectVegetation() {
		const std::optional<FPlanetFlowEnvironment> Environment = Renderer->GetPublishedEnvironment();
		// 未公開の環境への終了
		if(!Test.TestTrue(TEXT("植生と同じ公開環境"), Environment.has_value())) return;
		Test.TestTrue(TEXT("描画と衝突と植生が同じ場を共有"), Environment->Surface == PlantGround->GetPublishedFlowField());
		UInstancedStaticMeshComponent *const Grass = Vegetation->GetVisibleGrass();
		Test.TestTrue(TEXT("実際の草を配置"), Grass && Grass->GetInstanceCount() > 0);
		double MaximumRootError = 0, MinimumDryMargin = 1E9;
		unsigned Roots = 0;
		for(
			UInstancedStaticMeshComponent *const Mesh :
			{ Grass, Vegetation->GetTrees(), Vegetation->GetTreeVariant(), Vegetation->GetConifers() }
		) for(int32 Index = 0; Index < Mesh->GetInstanceCount(); ++Index) {
			FTransform Instance;
			Test.TestTrue(TEXT("実際の個体変換を取得"), Mesh->GetInstanceTransform(Index, Instance, true));
			const FVector Local = Environment->PlanetTransform.InverseTransformPosition(Instance.GetLocation()) * 0.01;
			const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector({ Local.X, Local.Y, Local.Z });
			const std::optional<PlanetCore::FlowChannelSection> Sample = Environment->Surface->Evaluate(Direction);
			const std::optional<double> Support = Environment->Surface->WaterSupport(Direction);
			if(!Test.TestTrue(TEXT("根元の不変な地形と水域"), Sample.has_value() && Support.has_value())) continue;
			const double Sink = Mesh == Grass ? 0 : 0.1 * Instance.GetScale3D().Z;
			const double Error = std::abs(Local.Length() + Sink - Environment->Settings.RadiusMeters - Sample->Terrain.ElevationMeters);
			MaximumRootError = std::max(MaximumRootError, Error);
			Test.TestTrue(TEXT("根が同じ地表に接地"), Error <= 0.02);
			const double Water = *Support > 0 && Sample->HasWater ?
			std::max(Environment->Settings.SeaLevelMeters, Sample->Water.ElevationMeters) :
			Environment->Settings.SeaLevelMeters;
			const double DryMargin = Sample->Terrain.ElevationMeters - Water;
			MinimumDryMargin = std::min(MinimumDryMargin, DryMargin);
			Test.TestTrue(TEXT("草木を水中へ配置しない"), DryMargin > (Mesh == Grass ? 0.049 : 0.199));
			++Roots;
		}
		for(
			UInstancedStaticMeshComponent *const Mesh :
			{ Grass, Vegetation->GetTrees(), Vegetation->GetTreeVariant(), Vegetation->GetConifers() }
		) for(int32 Slot = 0; Slot < Mesh->GetNumMaterials(); ++Slot) {
			const UMaterialInstanceDynamic *const Material = Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(Slot));
			Test.TestTrue(TEXT("再接続後も有効な元材質を保持"), Material && Material->Parent && !Material->Parent->IsA<UMaterialInstanceDynamic>());
		}
		// 季節を変えても同じ配列と同じ個体変換だけを使う
		const int64 Revision = Vegetation->PublishedRevision, TreeRevision = Vegetation->TreePublishedRevision;
		const int64 Evaluations = Vegetation->HeightEvaluations;
		TArray<FTransform> Positions;
		for(int32 Index = 0; Index < Grass->GetInstanceCount(); ++Index) {
			FTransform Transform;
			Grass->GetInstanceTransform(Index, Transform, true);
			Positions.Add(Transform);
		}
		Renderer->SolarYearPhase = 0;
		Renderer->Tick(0);
		Vegetation->Tick(0);
		UMaterialInstanceDynamic *const Material = Cast<UMaterialInstanceDynamic>(Grass->GetMaterial(0));
		if(Test.TestNotNull(TEXT("草の季節材質"), Material)) {
			Test.TestEqual(TEXT("草も地面と同じ季節"), Material->K2_GetScalarParameterValue(TEXT("SolarYearPhase")), 0.0F);
			Test.TestTrue(TEXT("草と地面が同じ配列を共有"), Material->K2_GetTextureParameterValue(TEXT("ClimateAtlas")) == Environment->Texture.Get());
		}
		Test.TestEqual(TEXT("季節変更で草の再配置無し"), Vegetation->PublishedRevision, Revision);
		Test.TestEqual(TEXT("季節変更で木の再配置無し"), Vegetation->TreePublishedRevision, TreeRevision);
		Test.TestEqual(TEXT("季節変更で標高の再評価無し"), Vegetation->HeightEvaluations, Evaluations);
		bool SamePositions = Positions.Num() == Grass->GetInstanceCount();
		for(int32 Index = 0; Index < Positions.Num() && SamePositions; ++Index) {
			FTransform Transform;
			SamePositions = Grass->GetInstanceTransform(Index, Transform, true) && Transform.Equals(Positions[Index], 0);
		}
		Test.TestTrue(TEXT("季節変更で根の位置と向きが不変"), SamePositions);
		Renderer->SolarYearPhase = 0.5;
		Renderer->Tick(0);
		Vegetation->Tick(0);
		Test.AddInfo(
			FString::Printf(
				TEXT("Flow vegetation: grass=%d trees=%d roots=%u rootError=%.12g dryMargin=%.12g seasonalOnly=%u"),
				Vegetation->GrassInstanceCount,
				Vegetation->TreeInstanceCount,
				Roots,
				MaximumRootError,
				MinimumDryMargin,
				static_cast<unsigned>(SamePositions)
			)
		);
		// 終了
		return;
	}

	/**
	 * 準備済の地面へ身体を生成し，実際のゲーム世界を開始する
	 * @return 身体を安全に生成出来た場合に真
	 */
	bool FFlowWalkingCommand::BeginWalking() {
		World->InitializeActorsForPlay(FURL());
		World->BeginPlay();
		World->GetWorldSettings()->NotifyBeginPlay();
		Test.TestTrue(TEXT("実際のゲーム世界を開始"), World->HasBegunPlay());
		Character = APlanetExplorerCharacter::SpawnOnPreparedGround(Ground, Anchor, FVector::RightVector);
		// 身体を生成出来ない場合の返戻
		if(!Test.TestNotNull(TEXT("公開済の河床へ身体を生成"), Character)) return false;
		Test.TestTrue(TEXT("身体の開始通知を実行"), Character->HasActorBegunPlay());
		// 視点切替の検査で，生成直後の複数有効化を隠さない
		TArray<UCameraComponent *> InitialCameras;
		Character->GetComponents(InitialCameras);
		int32 InitiallyActive = 0;
		for(const UCameraComponent *const Camera : InitialCameras) if(Camera->IsActive()) ++InitiallyActive;
		Test.TestEqual(TEXT("視点検査前から一つのカメラだけ有効"), InitiallyActive, 1);
		StartPosition = Character->GetActorLocation();
		PreviousPosition = StartPosition;
		CheckCameras();
		if(Renderer) {
			Renderer->Explorer = Character;
			InitialRenderRevision = Renderer->PublishedRevision;
			if(Preview) Preview->SetActorHiddenInGame(true);
		}
		LastTick = FPlatformTime::Seconds();
		Stage = 3;
		// 身体と同じ場で移動を始める返戻
		return true;
	}

	/**
	 * 公開中の実描画面へ線分を当て，同じ足元の実衝突と比較する
	 * @param Surface 元場で求めた地表位置
	 * @param Up 惑星の半径方向
	 * @param GroundImpact 足元の物理面の命中位置
	 * @return 単精度の実描画面と衝突の差が二センチメートル以内の場合に真
	 */
	bool FFlowWalkingCommand::InspectRender(const FVector Surface, const FVector Up, const FVector GroundImpact) {
		UProceduralMeshComponent *const Mesh = Renderer->GetVisibleMesh();
		// 表示部品の欠落を認めない返戻
		if(!Test.TestTrue(TEXT("歩行中も描画面を公開"), Mesh && Mesh->IsVisible())) return false;
		const FProcMeshSection *const Section = Mesh->GetProcMeshSection(0);
		// 空の描画候補への返戻
		if(!Test.TestNotNull(TEXT("歩行中の実三角形"), Section)) return false;
		unsigned Visible = 0;
		TArray<UProceduralMeshComponent *> Components;
		Renderer->GetComponents(Components);
		for(const UProceduralMeshComponent *const Component : Components) if(Component->IsVisible()) ++Visible;
		Test.TestEqual(TEXT("地面と水面を一枚ずつ表示"), Visible, 2U);
		// 同じ世代の水面を保持し，実際の水位も確認する返戻
		if(!InspectWater(Surface, Up)) return false;
		const FTransform Transform = Mesh->GetComponentTransform();
		const FVector Start = Transform.InverseTransformPosition(Surface + 1000 * Up);
		const FVector End = Transform.InverseTransformPosition(Surface - 1000 * Up);
		bool Hit = false;
		FVector Impact, Normal;
		for(int32 I = 0; I < Section->ProcIndexBuffer.Num(); I += 3) {
			FVector P[3];
			for(unsigned K = 0; K < 3; ++K) {
				const FVector V = Section->ProcVertexBuffer[Section->ProcIndexBuffer[I + K]].Position;
				P[K] = FVector(static_cast<float>(V.X), static_cast<float>(V.Y), static_cast<float>(V.Z));
			}
			if(FMath::SegmentTriangleIntersection(Start, End, P[0], P[1], P[2], Impact, Normal)) {
				Hit = true;
				break;
			}
		}
		// 地形の穴を実衝突だけの成功で代用しない返戻
		if(!Test.TestTrue(TEXT("身体の下の単精度描画面に命中"), Hit)) return false;
		MaximumRenderGapMeters =
		std::max(MaximumRenderGapMeters, FVector::Distance(Transform.TransformPosition(Impact), GroundImpact) * 0.01);
		// GPU に渡す単精度パラメーターで，実衝突と同じ惑星の固定模様を復元
		UMaterialInstanceDynamic *const Material = Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(0));
		// 診断色又は異なる材質への返戻
		if(!Test.TestTrue(TEXT("物理材質を使用"), Material && Material->Parent == Renderer->SurfaceMaterial)) return false;
		const auto Parameter = [&](const FName Name) -> FVector {
			const FLinearColor Value = Material->K2_GetVectorParameterValue(Name);
			// 実際の材質パラメーターの返戻
			return { Value.R, Value.G, Value.B };
		};
		const FVector Local = Impact * 0.01;
		const FVector Texture =
		Parameter(TEXT("TextureOrigin")) + Parameter(TEXT("TextureAxisX")) * Local.X + Parameter(TEXT("TextureAxisY")) * Local.Y +
		Parameter(TEXT("TextureAxisZ")) * Local.Z;
		FTransform Planet;
		double Radius = 0;
		// 所有する座標が失われた場合の返戻
		if(!Test.TestTrue(TEXT("材質と同じ惑星座標を取得"), Ground->GetPublishedPlanet(Planet, Radius))) return false;
		const FVector Expected = Planet.InverseTransformPosition(Transform.TransformPosition(Impact)) * 0.01;
		if(UsesNetwork) {
			Test.TestTrue(TEXT("移動後の表示にも同じ気候を保持"), Renderer->bClimateReady);
			const FVector Relative = Parameter(TEXT("TextureAxisX")) * Local.X + Parameter(TEXT("TextureAxisY")) * Local.Y +
			Parameter(TEXT("TextureAxisZ")) * Local.Z;
			const FVector ClimateDirection =
			(Parameter(TEXT("ClimatePlanetOffset")) + Relative * Material->K2_GetScalarParameterValue(TEXT("ClimateInverseRadius")))
			.GetSafeNormal();
			MaximumClimateDirectionError =
			std::max(MaximumClimateDirectionError, FVector::Distance(ClimateDirection, Expected.GetSafeNormal()));
			Test.TestTrue(TEXT("原点変更後の気候方向が同じ地点"), MaximumClimateDirectionError < 3E-7);
			Test.TestEqual(TEXT("公開の切替後も季節を保持"), Material->K2_GetScalarParameterValue(TEXT("SolarYearPhase")), 0.5F);
			Test.TestNotNull(TEXT("公開の切替後も気候配列を保持"), Material->K2_GetTextureParameterValue(TEXT("ClimateAtlas")));
			++SeasonalSamples;
		}
		for(unsigned K = 0; K < 3; ++K) {
			MaximumTexturePhaseErrorMeters =
			std::max(MaximumTexturePhaseErrorMeters, std::abs(std::remainder(Texture[K] - Expected[K], 512.0)));
		}
		Test.TestTrue(TEXT("移動中の模様が惑星固定で一ミリメートル以内"), MaximumTexturePhaseErrorMeters < 0.001);
		++RenderSamples;
		if(Renderer->IsBusy) ++RenderRetainedFrames;
		// 実際の描画と実衝突の照合結果の返戻
		return MaximumRenderGapMeters < 0.02;
	}

	/**
	 * 歩行位置の水面を実頂点へ照会し，共有水位と同じ世代の表示を確認する
	 * @param Surface 同じ場の地表位置
	 * @param Up 惑星の半径方向
	 * @return 所属する水面が有り，単精度水位の差が二センチメートル以内なら真
	 * @note 実三角形数 T に対して O(T)
	 */
	bool FFlowWalkingCommand::InspectWater(const FVector Surface, const FVector Up) {
		// 公開部品と同じ座標基準の確認
		UProceduralMeshComponent *const Water = Renderer->GetVisibleWaterMesh();
		UProceduralMeshComponent *const TerrainMesh = Renderer->GetVisibleMesh();
		// 片方だけの描画を認めない返戻
		if(!Test.TestTrue(TEXT("地面と同時に水面を公開"), Water && Water->IsVisible() && Water != TerrainMesh)) return false;
		Test.TestTrue(TEXT("同じ世代の地面と水面の原点"), Water->GetComponentTransform().Equals(TerrainMesh->GetComponentTransform(), 0));
		UMaterialInstanceDynamic *const Material = Cast<UMaterialInstanceDynamic>(Water->GetMaterial(0));
		// 診断色又は材質欠落への返戻
		if(!Test.TestTrue(TEXT("既存の水材質で描画"), Material && Material->Parent == Renderer->WaterMaterial)) return false;
		const FProcMeshSection *const Section = Water->GetProcMeshSection(0);
		// 水系を持つ定義の空の水面への返戻
		if(!Test.TestNotNull(TEXT("公開した実水面三角形"), Section)) return false;
		FTransform Planet;
		double Radius = 0;
		// 水位の参照元欠落への返戻
		if(!Test.TestTrue(TEXT("水位と同じ惑星座標"), Ground->GetPublishedPlanet(Planet, Radius))) return false;
		const FVector D = Planet.InverseTransformPosition(Surface).GetSafeNormal();
		const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector({ D.X, D.Y, D.Z });
		const std::shared_ptr<const PlanetCore::FlowSurfaceField> Field = Ground->GetPublishedFlowField();
		const std::optional<PlanetCore::FlowChannelSection> Expected = Field->Evaluate(Direction);
		const std::optional<double> Support = Field->WaterSupport(Direction);
		// 照会の欠落を水域外として扱わない返戻
		if(!Test.TestTrue(TEXT("歩行中も水位を評価"), Expected && Support)) return false;
		// 水際の有限幅を除き，実際に水没する地点だけを水面へ照会する返戻
		if(!Expected->HasWater || *Support <= 0.03 || Expected->Water.ElevationMeters - Expected->Terrain.ElevationMeters <= 0.03) {
			// 正の水面を要求しない地点への返戻
			return true;
		}
		const FVector WaterPoint = Planet.TransformPosition(D * (100 * (Radius + Expected->Water.ElevationMeters)));
		const FTransform Transform = Water->GetComponentTransform();
		const FVector Start = Transform.InverseTransformPosition(WaterPoint + 100 * Up);
		const FVector End = Transform.InverseTransformPosition(WaterPoint - 100 * Up);
		bool Hit = false;
		FVector Impact, Normal;
		for(int32 I = 0; I < Section->ProcIndexBuffer.Num(); I += 3) {
			FVector P[3];
			for(unsigned K = 0; K < 3; ++K) {
				const FVector V = Section->ProcVertexBuffer[Section->ProcIndexBuffer[I + K]].Position;
				P[K] = FVector(static_cast<float>(V.X), static_cast<float>(V.Y), static_cast<float>(V.Z));
			}
			if(FMath::SegmentTriangleIntersection(Start, End, P[0], P[1], P[2], Impact, Normal)) {
				Hit = true;
				break;
			}
		}
		// 水面の欠落への返戻
		if(!Test.TestTrue(TEXT("歩行中の単精度水面へ命中"), Hit)) return false;
		MaximumWaterErrorMeters =
		std::max(MaximumWaterErrorMeters, FVector::Distance(Transform.TransformPosition(Impact), WaterPoint) * 0.01);
		++WaterSamples;
		// 実頂点と独立な水位の精度の返戻
		return Test.TestTrue(TEXT("歩行中の水位誤差二センチメートル以内"), MaximumWaterErrorMeters < 0.02);
	}

	/**
	 * 移動した身体の下を独立に評価し，重力・足場・描画元と照合する
	 * @return 公開元と身体が一致し，被覆と実衝突が維持された場合に真
	 */
	bool FFlowWalkingCommand::InspectBody() {
		FTransform Transform;
		double Radius = 0, Height = 0, Margin = 0;
		// 定義の欠落を観測標本の除外にしない返戻
		if(!Test.TestTrue(TEXT("歩行中も公開済の定義を保持"), Ground->GetPublishedPlanet(Transform, Radius))) return false;
		const FVector Position = Character->GetActorLocation();
		const FVector Direction = Transform.InverseTransformPosition(Position).GetSafeNormal();
		const FVector Up = (Position - Transform.GetLocation()).GetSafeNormal();
		// 評価出来ない地点で失敗する返戻
		if(!Test.TestTrue(TEXT("歩行中も同じ元場を評価"), Ground->EvaluatePublishedElevation(Direction, Height))) return false;
		const FVector Surface = Transform.TransformPosition(Direction * (100 * (Radius + Height)));
		FVector Impact, Normal;
		Test.TestTrue(TEXT("移動先も足場の範囲内"), Ground->QueryCoverage(Position, 0.34, Margin) && Margin > 0);
		MinimumMarginMeters = std::min(MinimumMarginMeters, Margin);
		if(!Test.TestTrue(TEXT("移動先の河床へ線分が命中"), Ground->TraceGround(Surface + 100 * Up, Surface - 100 * Up, Impact, Normal))) {
			// 足場の欠落を失敗する返戻
			return false;
		}
		MaximumGroundErrorMeters = std::max(MaximumGroundErrorMeters, FVector::Distance(Surface, Impact) * 0.01);
		if(UsesRenderer) Test.TestTrue(TEXT("歩行中の描画と衝突が一致"), InspectRender(Surface, Up, Impact));
		if(UsesFlow) {
			double FlowHeight = 0, Water = 0, Coverage = 0, Original = 0;
			if(UsesLocation) {
				const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
				const std::optional<PlanetCore::FlowChannelSection> Sample = LocationField->Evaluate(D);
				// 独立な所有元から照会出来ない場合の返戻
				if(!Test.TestTrue(TEXT("地点から作った元場を移動中も照会"), Sample.has_value())) return false;
				FlowHeight = Sample->Terrain.ElevationMeters;
			} else Test.TestTrue(TEXT("移動先の描画元も同じ場を評価"), Preview->EvaluateFlowDirection(Direction, FlowHeight, Water, Coverage, Original));
			Test.TestEqual(TEXT("歩行と描画元の標高が一致"), FlowHeight, Height);
		}
		const UPlanetCharacterMovement *const Movement = CastChecked<UPlanetCharacterMovement>(Character->GetCharacterMovement());
		const FVector Gravity = Movement->GetPlanetGravityVector() * 0.01;
		MaximumGravityError = std::max(MaximumGravityError, (Gravity + 9.80665 * Up).Length());
		MaximumGravityMagnitudeError = std::max(MaximumGravityMagnitudeError, FMath::Abs(Gravity.Length() - 9.80665));
		const double Clearance = FVector::DotProduct(Position - Impact, Up) * 0.01 - 0.96;
		Test.TestTrue(TEXT("身体が河床へ沈まない"), Clearance > -0.02);
		if(Frames > 60) TravelMeters += FVector::VectorPlaneProject(Position - PreviousPosition, Up).Length() * 0.01;
		if(Frames > 150 && Movement->IsFalling()) {
			++AirFrames;
			JumpApexMeters = std::max(JumpApexMeters, Clearance);
		}
		if(Frames > 210 && Movement->IsMovingOnGround() && AirFrames) HasLanded = true;
		if(Ground->bBusy) ++RetainedFrames;
		PreviousPosition = Position;
		// 実際の移動地点を照合した結果の返戻
		return !Test.HasAnyErrors();
	}

	/**
	 * 歩行・跳躍・足場更新の数値を記録し，所有物を解放する
	 * @param ReleaseWorld 植生の追加検査が無い場合に，所有世界も解放
	 */
	void FFlowWalkingCommand::Finish(const bool ReleaseWorld) {
		Test.TestTrue(TEXT("接線方向へ二十メートル以上移動"), TravelMeters > 20);
		if(RunsLongRoute) {
			Test.TestTrue(TEXT("走行で百二十メートル以上移動"), TravelMeters > 120);
			Test.TestTrue(TEXT("走行中に五回以上描画更新"), Renderer->PublishedRevision > InitialRenderRevision + 4);
		}
		Test.TestTrue(TEXT("身体位置に追従して足場を再生成"), Character->GroundRefreshRequests > 0 && Ground->PublishedRevision > 1);
		Test.TestEqual(TEXT("移動中の足場欠落無し"), Character->CoverageMissFrames, static_cast<int64>(0));
		Test.TestTrue(TEXT("足場更新中も旧足場で移動"), RetainedFrames > 0);
		Test.TestTrue(TEXT("跳躍して着地"), AirFrames > 10 && JumpApexMeters > 0.5 && HasLanded);
		// UE の方向更新は各成分 1E-4 以内を保持する
		// 半径百万メートルの一刻み分も含む方向上限
		Test.TestTrue(TEXT("固定重力の方向がエンジンの丸め範囲内"), MaximumGravityError < 0.0018);
		Test.TestTrue(TEXT("重力の強度は元の厳密な条件を保持"), MaximumGravityMagnitudeError < 1E-5);
		Test.TestTrue(TEXT("歩行先の河床誤差二センチメートル以内"), MaximumGroundErrorMeters < 0.02);
		Test.TestEqual(TEXT("三視点を歩行前後に確認"), CameraChecks, 6U);
		if(UsesRenderer) {
			Test.TestTrue(TEXT("身体の移動で全球表示も二回以上更新"), Renderer->PublishedRevision > InitialRenderRevision + 1);
			Test.TestTrue(TEXT("描画の生成中も旧面で歩行"), RenderRetainedFrames > 0);
			Test.TestEqual(TEXT("全物理更新で実描画面を照合"), RenderSamples, Frames);
			Test.TestTrue(TEXT("古い要求を破棄して最新だけを公開"), Renderer->DiscardedCandidates > 0);
			Test.TestTrue(TEXT("実際に水位も独立検査"), WaterSamples > 100);
			Test.TestEqual(TEXT("視点変更で河道・岸索引を再準備しない"), Renderer->WaterPreparations, static_cast<int64>(1));
			Test.AddInfo(
				FString::Printf(
					TEXT("Rendered water: samples=%u error=%.12g preparations=%lld workerMs=%.9g"),
					WaterSamples,
					MaximumWaterErrorMeters,
					Renderer->WaterPreparations,
					Renderer->LastWaterMilliseconds
				)
			);
			Test.AddInfo(
				FString::Printf(
					TEXT("Rendered walking: samples=%u retained=%u published=%lld discarded=%lld renderCollisionGap=%.12g ")
					TEXT("texturePhaseError=%.12g workerMs=%.9g uploadMs=%.9g bufferMs=%.9g sectionMs=%.9g"),
					RenderSamples,
					RenderRetainedFrames,
					Renderer->PublishedRevision,
					Renderer->DiscardedCandidates,
					MaximumRenderGapMeters,
					MaximumTexturePhaseErrorMeters,
					Renderer->LastWorkerMilliseconds,
					Renderer->LastUploadMilliseconds,
					Renderer->LastBufferPreparationMilliseconds,
					Renderer->LastSectionUploadMilliseconds
				)
			);
		}
		if(UsesNetwork) {
			Test.TestEqual(TEXT("全物理更新で季節の参照地点を照合"), SeasonalSamples, Frames);
			Test.TestEqual(TEXT("移動中も気候の準備は一回"), Renderer->ClimatePreparationCount, 1);
			Test.AddInfo(
				FString::Printf(
					TEXT("Flow climate walking: samples=%u maxDirectionError=%.12g preparations=%d"),
					SeasonalSamples,
					MaximumClimateDirectionError,
					Renderer->ClimatePreparationCount
				)
			);
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Walking: flow=%d frames=%u travel=%.12g displacement=%.12g refresh=%lld published=%lld retained=%u ")
				TEXT("air=%u apex=%.12g landed=%d groundError=%.12g gravityError=%.12g magnitudeError=%.12g minMargin=%.12g"),
				static_cast<int>(UsesFlow),
				Frames,
				TravelMeters,
				FVector::Distance(StartPosition, Character->GetActorLocation()) * 0.01,
				Character->GroundRefreshRequests,
				Ground->PublishedRevision,
				RetainedFrames,
				AirFrames,
				JumpApexMeters,
				static_cast<int>(HasLanded),
				MaximumGroundErrorMeters,
				MaximumGravityError,
				MaximumGravityMagnitudeError,
				MinimumMarginMeters
			)
		);
		if(ReleaseWorld) Cleanup();
		// 終了
		return;
	}

	/**
	 * 元場と物理の準備を待ち，実時間以下の速さで固定刻みの歩行を進める
	 * @return 全段階の終了又は失敗時に真；待機中は偽
	 */
	bool FFlowWalkingCommand::Update() {
		const double Now = FPlatformTime::Seconds();
		if(Now - Started > 180 || Test.HasAnyErrors()) {
			if(!Test.HasAnyErrors()) Test.AddError(TEXT("歩行検査の待機上限"));
			Cleanup();
			// 未完了を失敗として終える返戻
			return true;
		}
		if(!Stage) {
			FActorSpawnParameters Spawn;
			Spawn.ObjectFlags |= RF_Transient;
			Ground = World->SpawnActor<APlanetGroundPatch>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn);
			// アクター生成の失敗を解放へ進める返戻
			if(!Test.TestNotNull(TEXT("歩行用の足場"), Ground)) return false;
			if(UsesLocation) {
				Preparation = World->SpawnActor<APlanetWatershedPreparation>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn);
				// 準備器の生成失敗への返戻
				if(!Test.TestNotNull(TEXT("非同期の水系準備器"), Preparation)) return false;
				const FVector Location(0.54178538420982891, 0.81195543427398209, -0.21724863684165494);
				Test.TestFalse(TEXT("最小半径未満の準備を拒否"), Preparation->RequestPreparation(42, 999, Location));
				Test.TestFalse(TEXT("方向無しの準備を拒否"), Preparation->RequestPreparation(42, 1E6, FVector::ZeroVector));
				Test.TestTrue(TEXT("中断する実作業を開始"), Preparation->RequestPreparation(42, 1E6, Location));
				Test.TestFalse(TEXT("作業中に別要求を重ねない"), Preparation->RequestPreparation(7, 1E6, Location));
				Test.TestTrue(TEXT("未完成の定義を公開しない"), Preparation->GetPrepared() == nullptr);
				Preparation->CancelPreparation();
				Test.TestFalse(TEXT("中断後に作業と部分結果を残さない"), Preparation->IsBusy || Preparation->IsReady);
				Test.TestTrue(TEXT("プレイ開始前に破棄する作業を開始"), Preparation->RequestPreparation(42, 1E6, Location));
				Test.TestTrue(TEXT("プレイ開始前の準備器を破棄"), Preparation->Destroy());
				Test.TestFalse(TEXT("回収前でも破棄した準備器の作業は終了"), Preparation->IsBusy || Preparation->IsReady);
				Test.TestFalse(TEXT("破棄済の準備器から再開しない"), Preparation->RequestPreparation(42, 1E6, Location));
				Preparation = World->SpawnActor<APlanetWatershedPreparation>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn);
				// 再作成の失敗への返戻
				if(!Test.TestNotNull(TEXT("正規要求用の準備器"), Preparation)) return false;
				const double RequestStarted = FPlatformTime::Seconds();
				Test.TestTrue(TEXT("中断後の正規要求を受理"), Preparation->RequestPreparation(42, 1E6, Location));
				Test.AddInfo(
					FString::Printf(TEXT("Location async request: milliseconds=%.9g"), (FPlatformTime::Seconds() - RequestStarted) * 1000)
				);
				Stage = 10;
			} else if(UsesFlow) {
				Preview = World->SpawnActor<APlanetSphericalSurfacePreview>(FVector(34567, -45678, 56789), FRotator(17, 31, -9), Spawn);
				// 描画元を準備出来ない場合の返戻
				if(!Test.TestNotNull(TEXT("歩行と同じ河床の描画"), Preview)) return false;
				const FVector Pass(0.54180127453542282, 0.81192128360203597, -0.21733662403243018);
				Test.TestTrue(TEXT("連続な元場を準備"), Preview->BuildPreview(42, 1E6, Pass, 200, 16, false, false, false, true));
				Test.TestTrue(TEXT("接続した水系を準備"), Preview->BuildConnectedFlowChannelPreview(Pass, -100, 64, 32));
				if(UsesNetwork) Test.TestTrue(TEXT("全河道の合成場を準備"), Preview->PrepareConnectedFlowNetwork());
				Anchor = Preview->GetDiagnosticAnchorDirection();
				Test.TestTrue(TEXT("河床の足場を要求"), Preview->RequestFlowCollision(Ground));
				Stage = 2;
			} else {
				Terrain = World->SpawnActor<APlanetTerrain>(FVector::ZeroVector, FRotator(17, 31, -9), Spawn);
				// 従来経路の生成失敗への返戻
				if(!Test.TestNotNull(TEXT("従来の地形"), Terrain)) return false;
				Terrain->bGenerateOnBeginPlay = false;
				Terrain->bFollowPlayerCamera = false;
				Terrain->bRefineNearSurface = false;
				Terrain->LeafBudget = 128;
				Terrain->ObserverMeters = FVector(0, 0, 2E6);
				Test.TestTrue(TEXT("従来の地形を要求"), Terrain->RequestGeneration());
				Stage = 1;
			}
			// 非同期準備を待つ返戻
			return false;
		}
		if(Stage == 10) {
			const double PollStarted = FPlatformTime::Seconds();
			Preparation->PollPreparation();
			MaximumPreparationPollMilliseconds =
			std::max(MaximumPreparationPollMilliseconds, (FPlatformTime::Seconds() - PollStarted) * 1000);
			++PreparationPolls;
			// 主スレッドを止めず，次の完了確認へ進む返戻
			if(Preparation->IsBusy) return false;
			Test.TestTrue(*Preparation->Status, Preparation->IsReady);
			// 地点からの準備失敗を次の更新で解放する返戻
			if(Test.HasAnyErrors() || !PrepareLocation()) return false;
			Stage = 2;
			// 同じ場の非同期衝突調理を待つ返戻
			return false;
		}
		if(Stage == 1) {
			Terrain->PollGeneration();
			// 従来場の公開を待つ返戻
			if(Terrain->bGenerationBusy) return false;
			Test.TestTrue(TEXT("従来場を公開"), Terrain->PublishedRevision > 0);
			Anchor = FVector(1, 2, 3).GetSafeNormal();
			double Height = 0;
			FVector Normal;
			Test.TestTrue(TEXT("従来の着地点を評価"), Terrain->EvaluateDirection(Anchor, Height, Normal) && Height > 0);
			const FVector Point = Terrain->GetActorTransform().TransformVectorNoScale(Anchor * (100 * (Terrain->RadiusMeters + Height)));
			Terrain->SetActorLocation(FVector(34567, -45678, 56789) - Point);
			Ground->Terrain = Terrain;
			Ground->AnchorDirection = Anchor;
			Test.TestTrue(TEXT("従来の足場を要求"), Ground->RequestPatch());
			Stage = 2;
			// 調理を待つ返戻
			return false;
		}
		if(Stage == 2) {
			Ground->PollPatch();
			// 実際の衝突が公開される迄待つ返戻
			if(Ground->bBusy) return false;
			Test.TestTrue(TEXT("着地点の実衝突を公開"), Ground->PublishedRevision > 0);
			if(UsesRenderer) {
				FActorSpawnParameters Spawn;
				Spawn.ObjectFlags |= RF_Transient;
				Renderer = World->SpawnActor<APlanetFlowTerrain>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn);
				// 描画生成の失敗への返戻
				if(!Test.TestNotNull(TEXT("全球の非同期描画"), Renderer)) return false;
				if(UsesLocation) {
					Test.TestFalse(TEXT("気候の元場無しを拒否"), Renderer->ConfigureClimate(nullptr, LocationOcean));
					Test.TestFalse(TEXT("気候の大洋無しを拒否"), Renderer->ConfigureClimate(LocationField, nullptr));
					Test.TestTrue(TEXT("地点から作った同じ海陸で気候を構成"), Renderer->ConfigureClimate(LocationField, LocationOcean));
				} else if(UsesNetwork) {
					Test.TestFalse(TEXT("気候の接続先無しを拒否"), Preview->ConfigureFlowClimate(nullptr));
					Test.TestTrue(TEXT("最初の要求前に同じ海陸の気候を構成"), Preview->ConfigureFlowClimate(Renderer));
				}
				FTransform Transform;
				double Radius = 0, Height = 0;
				Test.TestTrue(TEXT("描画の惑星座標"), Ground->GetPublishedPlanet(Transform, Radius));
				Test.TestTrue(TEXT("描画の着地点"), Ground->EvaluatePublishedElevation(Anchor, Height));
				const FVector Body = Transform.TransformPosition(Anchor * (100 * (Radius + Height + 1)));
				const FVector Eye = Body + Transform.TransformVectorNoScale(Anchor) * 64;
				Test.TestTrue(TEXT("最初の描画要求"), Renderer->RequestView(Ground, Eye, Body));
				if(UsesLocation) Test.TestFalse(TEXT("生成開始後の気候切替を拒否"), Renderer->ConfigureClimate(LocationField, LocationOcean));
				else if(UsesNetwork) Test.TestFalse(TEXT("生成開始後に別の気候構成へ切替えない"), Preview->ConfigureFlowClimate(Renderer));
				Test.TestTrue(TEXT("待機する描画要求"), Renderer->RequestView(Ground, Eye, Body));
				Test.TestTrue(TEXT("最新の描画要求"), Renderer->RequestView(Ground, Eye, Body));
				const int64 Revision = Renderer->RequestedRevision;
				UMaterialInterface *const Material = Renderer->SurfaceMaterial;
				Renderer->SurfaceMaterial = nullptr;
				Test.TestFalse(TEXT("材質の無い描画を拒否"), Renderer->RequestView(Ground, Eye, Body));
				Renderer->SurfaceMaterial = Material;
				UMaterialInterface *const WaterMaterial = Renderer->WaterMaterial;
				Renderer->WaterMaterial = nullptr;
				Test.TestFalse(TEXT("水材質の無い描画を拒否"), Renderer->RequestView(Ground, Eye, Body));
				Renderer->WaterMaterial = WaterMaterial;
				Test.TestFalse(TEXT("無い足場の描画要求を拒否"), Renderer->RequestView(nullptr, Eye, Body));
				Test.TestFalse(TEXT("惑星中心のカメラを拒否"), Renderer->RequestView(Ground, Transform.GetLocation(), Body));
				Renderer->SetActorScale3D(FVector(2));
				Test.TestFalse(TEXT("拡縮した描画を拒否"), Renderer->RequestView(Ground, Eye, Body));
				Renderer->SetActorScale3D(FVector::OneVector);
				Test.TestEqual(TEXT("拒否で描画通番を変更しない"), Renderer->RequestedRevision, Revision);
				Stage = 5;
			} else BeginWalking();
			// 初回の準備又は物理更新へ進む返戻
			return false;
		}
		if(Stage == 5) {
			Renderer->PollView();
			// 有効な描画の公開を待つ返戻
			if(Renderer->IsBusy) return false;
			Test.TestTrue(TEXT("最新の有効な全球面を公開"), Renderer->PublishedRevision == Renderer->RequestedRevision);
			// 不成立なら次の更新で所有物を解放する返戻
			if(Test.HasAnyErrors()) return false;
			if(UsesNetwork) CheckSeasonalOnlyUpdate();
			BeginWalking();
			// 実物理の更新へ進む返戻
			return false;
		}
		if(Stage == 6) {
			Renderer->PollView();
			// 最後の有効な表示世代の完了を待つ返戻
			if(Renderer->IsBusy) return false;
			Vegetation->Tick(1.0F / 60);
			// 所有する配置処理の小分けを維持して待つ返戻
			if(!Vegetation->PublishedRevision || !Vegetation->TreePublishedRevision) return false;
			BeginDryVegetation();
			// 水中の棄却と，別の乾いた配置地点を確認する返戻
			return false;
		}
		if(Stage == 7) {
			PlantGround->PollPatch();
			// 実際の乾いた足場を待つ返戻
			if(PlantGround->bBusy) return false;
			PlantBody = APlanetExplorerCharacter::SpawnOnPreparedGround(PlantGround, PlantDirection, FVector::RightVector);
			// 根の確認用の身体を安全に生成出来ない場合の返戻
			if(!Test.TestNotNull(TEXT("乾いた岸の実足場へ配置用の身体"), PlantBody)) return false;
			Test.TestTrue(
				TEXT("乾いた地点の同じ描画を要求"),
				Renderer->RequestView(
					PlantGround,
					PlantBody->GetActorLocation() + PlantBody->GetActorUpVector() * 64,
					PlantBody->GetActorLocation()
				)
			);
			Stage = 8;
			// 同じ公開環境の準備を待つ返戻
			return false;
		}
		if(Stage == 8) {
			Renderer->PollView();
			// 乾いた地点の精密な表示を待つ返戻
			if(Renderer->IsBusy) return false;
			Test.TestEqual(TEXT("配置地点の新しい表示を公開"), Renderer->PublishedRevision, Renderer->RequestedRevision);
			Vegetation->InitializeFlow(Renderer, PlantBody);
			PlantGrassRevision = Vegetation->PublishedRevision;
			PlantTreeRevision = Vegetation->TreePublishedRevision;
			Stage = 9;
			// 配置を小分けで進める返戻
			return false;
		}
		if(Stage == 9) {
			Vegetation->Tick(1.0F / 60);
			// 新しい地点の候補を最後迄確認する返戻
			if(Vegetation->bPreparing || Vegetation->PublishedRevision <= PlantGrassRevision) return false;
			// 樹木の候補も新しい地点の準備完了を待つ返戻
			if(Vegetation->TreePublishedRevision <= PlantTreeRevision) return false;
			InspectVegetation();
			CheckCelestialClock();
			Cleanup();
			// 歩行と水中の棄却及び岸での配置検査の返戻
			return true;
		}
		// 非同期作業に対し，物理だけを実時間より速く進めない返戻
		if(Now - LastTick < 1.0 / 60) return false;
		LastTick = Now;
		if(Frames == 60) Character->SetMoveInput(1, 0, RunsLongRoute);
		if(Frames == 150) Character->Jump();
		if(Frames == 156) Character->StopJumping();
		if(Frames == 300) Character->SetViewMode(EPlanetViewMode::ThirdPerson);
		if(RunsLongRoute && Frames == 600) Character->SetViewMode(EPlanetViewMode::Overview);
		if(RunsLongRoute && Frames == 660) Character->SetViewMode(EPlanetViewMode::FirstPerson);
		if(Frames == (RunsLongRoute ? 1140U : 420U)) Character->SetMoveInput(0, 0, false);
		World->Tick(LEVELTICK_All, 1.0F / 60);
		++Frames;
		InspectBody();
		if(Frames == (RunsLongRoute ? 1200U : 480U)) {
			CheckCameras();
			if(UsesNetwork) {
				Finish(false);
				Renderer->Explorer = nullptr;
				FActorSpawnParameters Spawn;
				Spawn.ObjectFlags |= RF_Transient;
				Vegetation = World->SpawnActor<APlanetVegetation>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn);
				// 未準備の植生を次の更新で解放する返戻
				if(!Test.TestNotNull(TEXT("同じ地形の植生"), Vegetation)) return false;
				Vegetation->InitializeFlow(Renderer, Character);
				Stage = 6;
				// 歩行検査を変更せず，配置を追加で確認する返戻
				return false;
			}
			Finish();
			// 歩行の全段階が終了した場合の返戻
			return true;
		}
		// 次の実時間の更新を待つ返戻
		return false;
	}

	/**
	 * 所有世界と元場の種別を束縛する
	 * @param Owner 自動検査の結果受取先
	 * @param TestWorld 自身が終了時に解放するゲーム世界
	 * @param UseFlow 接続河床なら真，従来場なら偽
	 * @param UseRenderer 全球の実描画も照合する指定
	 * @param RunLongRoute 通常の歩行検査を保持し，別の検査で走行と俯瞰からの復帰を確認
	 * @param UseNetwork 単独河道と別に，全河道の合成場を確認
	 * @param UseLocation 診断アクターを介さず，陸上地点から水系を準備
	 */
	FFlowWalkingCommand::FFlowWalkingCommand(
		FAutomationTestBase &Owner,
		UWorld *const TestWorld,
		const bool UseFlow,
		const bool UseRenderer,
		const bool RunLongRoute,
		const bool UseNetwork,
		const bool UseLocation
	) : Test(Owner),
	World(TestWorld),
	UsesFlow(UseFlow),
	UsesRenderer(UseRenderer),
	RunsLongRoute(RunLongRoute),
	UsesNetwork(UseNetwork),
	UsesLocation(UseLocation),
	Started(FPlatformTime::Seconds()) {}

	/**
	 * 検査が中断されても所有世界を解放する
	 */
	FFlowWalkingCommand::~FFlowWalkingCommand() {
		Cleanup();
		// 終了
		return;
	}

	/**
	 * 他の実行世界を変更せず，物理が動作する検査世界を登録する
	 * @param Test 結果受取先
	 * @param UsesFlow 接続河床を使う指定
	 * @param UsesRenderer 全球描画の生成と移動中の照合を行う指定
	 * @param RunsLongRoute より長い走行を別の検査で行う指定
	 * @param UsesNetwork 全河道の合成場を使う指定
	 * @param UsesLocation 陸上地点から準備した水系を使う指定
	 * @return 遅延検査を登録出来た場合に真
	 */
	bool StartWalking(
		FAutomationTestBase &Test,
		const bool UsesFlow,
		const bool UsesRenderer=false,
		const bool RunsLongRoute=false,
		const bool UsesNetwork=false,
		const bool UsesLocation=false
	) {
		// 世界の登録先が無い場合の返戻
		if(!Test.TestNotNull(TEXT("歩行検査のエンジン"), GEngine)) return false;
		const UWorld::InitializationValues Values =
		UWorld::InitializationValues().CreateNavigation(false).CreateAISystem(false).AllowAudioPlayback(false).ShouldSimulatePhysics(
			true
		).EnableTraceCollision(true).SetTransactional(false);
		// 前の検査の世界が回収前でも，同じパッケージ内で名前を再利用しない
		const FName WorldName = MakeUniqueObjectName(GetTransientPackage(), UWorld::StaticClass(), TEXT("PlanetWalkingWorld"));
		UWorld *const World =
		UWorld::CreateWorld(EWorldType::Game, false, WorldName, GetTransientPackage(), true, ERHIFeatureLevel::Num, &Values);
		// 世界生成の失敗への返戻
		if(!Test.TestNotNull(TEXT("歩行用の実物理世界"), World)) return false;
		GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
		ADD_LATENT_AUTOMATION_COMMAND(
			FFlowWalkingCommand(Test, World, UsesFlow, UsesRenderer, RunsLongRoute, UsesNetwork, UsesLocation)
		);
		// 完了までの検査と解放を移譲した結果の返戻
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowWalkingContracts,
	"PlanetGenerator.FlowChannel.Walking",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 新しい河床に対する実歩行の検査を登録する
 * @param Parameters 未使用の実行引数
 * @return 登録出来た場合に真；合否は遅延検査の完了で決定
 */
bool FPlanetFlowWalkingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 共有河床の歩行検査の登録結果の返戻
	return StartWalking(*this, true);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLegacyWalkingContracts,
	"PlanetGenerator.FlowChannel.LegacyWalking",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 従来の地形に対する歩行経路の回帰検査を登録する
 * @param Parameters 未使用の実行引数
 * @return 登録出来た場合に真；合否は遅延検査の完了で決定
 */
bool FPlanetLegacyWalkingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 従来場の歩行検査の登録結果の返戻
	return StartWalking(*this, false);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowRenderedWalkingContracts,
	"PlanetGenerator.FlowChannel.RenderedWalking",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 新河床の実移動と，移動に追従する全球描画を同じ実行世界で検査する
 * @param Parameters 未使用の実行引数
 * @return 遅延検査を登録出来た場合に真
 */
bool FPlanetFlowRenderedWalkingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 描画を伴う実歩行検査の登録結果の返戻
	return StartWalking(*this, true, true);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowRenderedRunningContracts,
	"PlanetGenerator.FlowChannel.RenderedRunning",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 百二十メートルを超える走行と，俯瞰からの復帰を実描画・衝突と共に検査する
 * @param Parameters 未使用の実行引数
 * @return 遅延検査を登録出来た場合に真
 */
bool FPlanetFlowRenderedRunningContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 既存の短距離試験を変更せず追加する走行検査の返戻
	return StartWalking(*this, true, true, true);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowNetworkRenderedWalkingContracts,
	"PlanetGenerator.FlowChannel.NetworkRenderedWalking",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 全水系の合成場に対して，同じ実描画と歩行の検査を登録する
 * @param Parameters 未使用の実行引数
 * @return 検査を登録出来た場合に真；合否は実物理の完了で決定
 */
bool FPlanetFlowNetworkRenderedWalkingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 単独河道の基準を維持した全水系の検査の返戻
	return StartWalking(*this, true, true, false, true);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLocationWatershedWalkingContracts,
	"PlanetGenerator.FlowChannel.LocationRenderedWalking",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 地点からの水系準備を，実衝突・描画・歩行・気候・植生へ接続する
 * @param Parameters 未使用の実行引数
 * @return 遅延検査を登録出来た場合に真
 */
bool FPlanetLocationWatershedWalkingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 従来の検査を保持した，新しい準備入口の検査の返戻
	return StartWalking(*this, true, true, false, true, true);
}

#endif
