#include "PlanetTerrain.h"
#include "Core/PlanetDrainage.h"
#include "Core/PlanetEvaluationCache.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOceanSurface.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetStreamingFocus.h"
#include "Core/PlanetSurfaceMaterial.h"
#include "Core/PlanetView.h"
#include "Core/PlanetWaterCoverage.h"
#include "Core/PlanetWaterLod.h"
#include "PlanetExplorerCharacter.h"
#include "PlanetGroundPatch.h"
#include "Async/Async.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "RenderCommandFence.h"
#include "UObject/ConstructorHelpers.h"
#include <limits>
#include <memory>
#include <stop_token>
#include <unordered_map>

namespace {
	struct FPlanetRequest {
		int64 Revision = 0;
		PlanetCore::GeologySettings Settings;
		PlanetCore::LodView View;
		bool bRefineNearSurface = true;
		bool bDiagnosticHeightColors = false;
		bool bRenderRiverWater = false;
		double FoliagePrecisionRadiusMeters = 0;
		bool bPrioritizeSurface = false;
	};

	struct FSurfacePreparationResult {
		PlanetCore::SurfacePreparationProgress Progress;
		uint64 NewVisits = 0;
		uint64 NewEvaluations = 0;
		double Milliseconds = 0;
		bool bInterrupted = false;
	};

	struct FOceanRenderData {
		TArray<FVector> Vertices;
		TArray<FVector> Normals;
		TArray<FVector2D> UV2;
		TArray<FVector2D> UV3;
		TArray<int32> Indices;
		TArray<FProcMeshTangent> Tangents;
		PlanetCore::GroundFrame Frame;
		PlanetCore::WaterRefinementDiagnostics RiverDiagnostics;
		bool bReusedRiverPartition = false;
		bool bRejectedRetainedPartition = false;
		std::uint32_t CoarsenedParents = 0;
		double CoarseningMilliseconds = 0;
		PlanetCore::Vec3 TextureOrigin;
		FTransform Transform;
		double BuildMilliseconds = 0;
		double MaximumFloatErrorMeters = 0;
		double SkeletonMilliseconds = 0;
		double ProjectionMilliseconds = 0;
	};

	/** 最後に公開した不変場の完全分割を１件だけ保持し，視点変更時も精度を再検査する */
	struct FRiverWaterPartition {
		std::shared_ptr<const PlanetCore::RiverTerrainField> Field;
		PlanetCore::LodTree Tree;
		PlanetCore::AdaptiveSphereMesh Mesh;
		PlanetCore::Vec3 LastTrimObserver;
	};

	/** 地表の検査後に水面へ渡す私有分割
	 * 待機中は最新の１件だけ保持する */
	struct FPlanetWaterInput {
		FPlanetRequest Request;
		PlanetCore::LodTree Tree;
		PlanetCore::AdaptiveSphereMesh Mesh;
	};

	enum class EPlanetPublication { Combined, Surface, Water };

	struct FPlanetResult {
		EPlanetPublication Publication = EPlanetPublication::Combined;
		FPlanetRequest Request;
		FOceanRenderData Water;
		std::shared_ptr<FPlanetWaterInput> WaterInput;
		std::shared_ptr<PlanetCore::TerrainEvaluationCache> WaterEvaluations;
		bool bReusedOceanTopology = false;
		bool bReusedPreviousOceanTopology = false;
		uint32 OceanRefinementPasses = 0;
		uint32 OceanRefinedParents = 0;
		uint64 NeighborCacheHits = 0;
		uint64 CachedNeighborTiles = 0;
		PlanetCore::OceanReuseDiagnostics OceanInitialDiagnostics;
		std::shared_ptr<PlanetCore::OceanSurfaceCache> WaterCache;
		std::shared_ptr<const FRiverWaterPartition> RiverPartition;
		FString FailureReason;
		std::shared_ptr<const PlanetCore::GeologyField> Geology;
		std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean;
		std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage;
		std::shared_ptr<const PlanetCore::RiverTerrainField> Surface;
		std::shared_ptr<PlanetCore::LodSurfaceCache> SurfaceCache;
		std::shared_ptr<PlanetCore::TerrainEvaluationCache> EvaluationCache;
		TArray<FVector> Vertices;
		TArray<FVector> Normals;
		TArray<FVector2D> UV0;
		TArray<FVector2D> UV1;
		TArray<FVector2D> UV2;
		TArray<FVector2D> UV3;
		TArray<int32> Indices;
		TArray<FLinearColor> Colors;
		TArray<FProcMeshTangent> Tangents;
		int32 LeafCount = 0;
		int32 MaximumLevel = 0;
		FTransform MeshTransform = FTransform::Identity;
		PlanetCore::Vec3 TextureOrigin;
		std::optional<PlanetCore::GroundFrame> Frame;
		double MaximumSurfaceProbeError = 0;
		double MaximumSurfaceFloatError = 0;
		double WorkerMilliseconds = 0;
		double PreparationMilliseconds = 0;
		double GeologyMilliseconds = 0;
		double OceanMilliseconds = 0;
		double DrainageMilliseconds = 0;
		double SurfaceMilliseconds = 0;
		double SelectionMilliseconds = 0;
		double TopologyMilliseconds = 0;
		double VertexMilliseconds = 0;
		double ValidationMilliseconds = 0;
		uint64 HeightEvaluations = 0;
		uint64 SurfaceCacheHits = 0;
		uint64 CachedTiles = 0;
		uint64 StitchCacheHits = 0;
		uint32 StitchPasses = 0;
		uint32 StitchSplits = 0;
		uint64 ValidationHeightEvaluations = 0;
		uint64 ValidationEdgeHits = 0;
		PlanetCore::EvaluationCacheStats EvaluationStats;
		std::size_t CachedSamples = 0;
		std::size_t SampleCapacity = 0;
		std::size_t CachedWaterSamples = 0;
		std::size_t WaterSampleCapacity = 0;
	};

	/**
	 * 地形計算の座標を UE の座標型へ移す
	 * @param V 変換する座標
	 * @return 同じ成分を持つ UE 座標
	 */
	FVector ToUnreal(PlanetCore::Vec3 V) {
		return FVector(V.X, V.Y, V.Z);
	}

	/**
	 * 描画部品が所有する材質へ地表の座標基準を設定する
	 * @param Mesh 対象の描画部品
	 * @param Source 使用する元材質
	 * @param Frame 地表の局所基準，全球座標を使う場合は空
	 * @param Origin 模様の位相原点
	 */
	void ApplyTextureFrame(
		UProceduralMeshComponent *Mesh,
		UMaterialInterface *Source,
		const PlanetCore::GroundFrame *Frame,
		PlanetCore::Vec3 Origin
	) {
		// 現在の準備条件を満たさない場合の終了
		if(!Source) return;
		UMaterialInstanceDynamic *Material = Cast<UMaterialInstanceDynamic>(Mesh->GetMaterial(0));
		if(!Material || Material->Parent != Source) Material = Mesh->CreateDynamicMaterialInstance(0, Source);
		// 現在の準備条件を満たさない場合の終了
		if(!Material) return;
		const auto Parameter = [&](FName Name, PlanetCore::Vec3 V) {
			Material->SetVectorParameterValue(
				Name,
				FLinearColor(static_cast<float>(V.X), static_cast<float>(V.Y), static_cast<float>(V.Z), 0)
			);
			// 終了
			return;
		};
		Parameter(TEXT("TextureOrigin"), Origin);
		Parameter(TEXT("TextureAxisX"), Frame ? Frame->East : PlanetCore::Vec3{ 1, 0, 0 });
		Parameter(TEXT("TextureAxisY"), Frame ? Frame->North : PlanetCore::Vec3{ 0, 1, 0 });
		Parameter(TEXT("TextureAxisZ"), Frame ? Frame->Up : PlanetCore::Vec3{ 0, 0, 1 });
		// 終了
		return;
	}

	/**
	 * 地表とは別の可変キャッシュを使い，私有分割から完全な水面を準備する
	 * @param Result 対応する地表の検査済み記録
	 * @param Input 水面の初期分割と元の要求
	 * @param PreviousOceanFrame 最後に公開した水面の原点
	 * @param ReuseWaterCache 水面専用ワーカーだけが変更する球面キャッシュ
	 * @param ReuseRiverPartition 同じ場の公開済み水面分割
	 * @param Stop 水面専用の中断要求
	 * @return 有効な水面候補，又は拒否理由，中断時は空
	 */
	std::shared_ptr<FPlanetResult> GeneratePlanetWater(
		const std::shared_ptr<FPlanetResult> Result,
		const FPlanetWaterInput &Input,
		std::optional<PlanetCore::GroundFrame> PreviousOceanFrame,
		std::shared_ptr<PlanetCore::OceanSurfaceCache> ReuseWaterCache,
		std::shared_ptr<const FRiverWaterPartition> ReuseRiverPartition,
		const std::stop_token Stop
	) {
		const FPlanetRequest &Request = Input.Request;
		const std::optional<PlanetCore::UnitDirection> Anchor =
		PlanetCore::UnitDirection::FromVector(Request.View.SurfaceObserverMeters.value_or(Request.View.ObserverMeters));
		// 水面評価の支持方向・共有場が無い場合，候補を公開しない返戻
		if(!Anchor || !Result->WaterEvaluations || Stop.stop_requested()) return {};
		if(Request.bRenderRiverWater) Result->WaterEvaluations->GrowWaterSamples(PlanetCore::TerrainEvaluationCache::MaximumSamples);
		const PlanetCore::EvaluationCacheStats BeforeEvaluations = Result->WaterEvaluations->Statistics();
		const auto Reject = [&](FString Reason) {
			Result->FailureReason = MoveTemp(Reason);
			// 未完成の水面を公開しない拒否理由の返戻
			return Result;
		};
		const double OceanStart = FPlatformTime::Seconds();
		if(
			Request.bRenderRiverWater && ReuseRiverPartition && ReuseRiverPartition->Field == Result->Surface &&
			(Request.View.ObserverMeters - ReuseRiverPartition->LastTrimObserver).Length() >
			2 * std::max(PlanetCore::StreamingFocus::BodyRadiusMeters, Result->Request.View.SurfaceRadiusMeters)
		) {
			const double TrimStart = FPlatformTime::Seconds();
			std::optional<PlanetCore::WaterLodTrim> Trimmed = PlanetCore::TrimWaterLod(
				ReuseRiverPartition->Tree,
				Request.View,
				*Anchor,
				Result->Request.View.SurfaceRadiusMeters + 8,
				6,
				Stop
			);
			// 再利用する水面分割の縮約に失敗した場合の返戻
			if(!Trimmed) return Reject(TEXT("Retained water coarsening cancelled or rejected."));
			Result->Water.CoarsenedParents = Trimmed->CoarsenedParents;
			if(Trimmed->CoarsenedParents > 0) {
				std::optional<PlanetCore::AdaptiveSphereMesh> Updated =
				PlanetCore::UpdateAdaptiveSphere(Trimmed->Tree, ReuseRiverPartition->Mesh, Stop);
				// 縮約後の被覆を組立てられない場合の返戻
				if(!Updated) return Reject(TEXT("Coarsened water topology assembly rejected."));
				const std::shared_ptr<FRiverWaterPartition> Candidate = std::make_shared<FRiverWaterPartition>();
				Candidate->Field = ReuseRiverPartition->Field;
				Candidate->Tree = std::move(Trimmed->Tree);
				Candidate->Mesh = std::move(*Updated);
				Candidate->LastTrimObserver = Request.View.ObserverMeters;
				ReuseRiverPartition = Candidate;
			}
			Result->Water.CoarseningMilliseconds = (FPlatformTime::Seconds() - TrimStart) * 1000;
		}
		if(!ReuseWaterCache || !ReuseWaterCache->Uses(Request.Settings.RadiusMeters, Request.Settings.SeaLevelMeters)) {
			ReuseWaterCache =
			std::make_shared<PlanetCore::OceanSurfaceCache>(Request.Settings.RadiusMeters, Request.Settings.SeaLevelMeters);
		}
		Result->WaterCache = std::move(ReuseWaterCache);
		std::optional<PlanetCore::OceanSurfaceMesh> OceanMesh;
		FOceanRenderData &Water = Result->Water;
		// 高さや水位が変わった場には古い分割を渡さず，同じ場でも現在の視点で再検査する
		const bool CanReuseRiver = Request.bRenderRiverWater && ReuseRiverPartition && ReuseRiverPartition->Field == Result->Surface;
		for(unsigned Attempt = 0; Attempt < (CanReuseRiver ? 2U : 1U); ++Attempt) {
			// 水面候補の再構築前に中断された場合の返戻
			if(Stop.stop_requested()) return {};
			const bool bTryRetained = CanReuseRiver && !Attempt;
			OceanMesh.reset();
			if(bTryRetained) {
				const double ProjectionStart = FPlatformTime::Seconds();
				OceanMesh = PlanetCore::BuildOceanSurfaceFromSkeleton(
					Request.View,
					Request.Settings.SeaLevelMeters,
					ReuseRiverPartition->Tree,
					ReuseRiverPartition->Mesh,
					PreviousOceanFrame,
					Stop,
					&Result->OceanInitialDiagnostics
				);
				Water.ProjectionMilliseconds += (FPlatformTime::Seconds() - ProjectionStart) * 1000;
			} else if(Request.bRenderRiverWater) {
				const double SkeletonStart = FPlatformTime::Seconds();
				// DrainageField の BuildUniformSphere(6) と共通の節点を初期分割へ含める
				std::optional<PlanetCore::LodTree> Skeleton = PlanetCore::PrepareWaterSkeletonTree(Input.Tree, 6, Stop);
				// 排水骨格を予算内で分割出来ない場合の返戻
				if(!Skeleton) return Reject(TEXT("River skeleton partition exceeded its bounded budget or was cancelled."));
				std::optional<PlanetCore::AdaptiveSphereMesh> SkeletonMesh = PlanetCore::BuildAdaptiveSphere(*Skeleton);
				// 河道の初期分割を三角形へ変換出来ない場合の返戻
				if(!SkeletonMesh) return Reject(TEXT("River skeleton partition could not be triangulated."));
				Result->Water.SkeletonMilliseconds += (FPlatformTime::Seconds() - SkeletonStart) * 1000;
				const double ProjectionStart = FPlatformTime::Seconds();
				OceanMesh = PlanetCore::BuildOceanSurfaceFromSkeleton(
					Request.View,
					Request.Settings.SeaLevelMeters,
					*Skeleton,
					*SkeletonMesh,
					PreviousOceanFrame,
					Stop,
					&Result->OceanInitialDiagnostics
				);
				Result->Water.ProjectionMilliseconds += (FPlatformTime::Seconds() - ProjectionStart) * 1000;
			} else {
				OceanMesh = PlanetCore::BuildOceanSurfaceFromTerrain(
					Request.View,
					Request.Settings.SeaLevelMeters,
					Input.Tree,
					Input.Mesh,
					PreviousOceanFrame,
					Stop,
					&Result->OceanInitialDiagnostics,
					Result->WaterCache.get()
				);
				if(!OceanMesh && !Stop.stop_requested()) {
					OceanMesh = PlanetCore::BuildOceanSurface(
						Request.View,
						Request.Settings.SeaLevelMeters,
						PreviousOceanFrame,
						Stop,
						Result->WaterCache.get()
					);
				}
			}
			// 精度又は被覆を満たさない候補の拒否理由の返戻
			if(!OceanMesh) {
				if(bTryRetained && !Stop.stop_requested()) {
					Water.bRejectedRetainedPartition = true;
					continue;
				}
				// 完全な水面候補を作れない場合，旧表示を保持する返戻
				return Reject(TEXT("Ocean LOD or precision failed; previous complete pair retained."));
			}
			Result->bReusedOceanTopology = OceanMesh->ReusedTopology;
			Result->bReusedPreviousOceanTopology = OceanMesh->ReusedPreviousTopology;
			Result->OceanRefinementPasses = OceanMesh->RefinementPasses;
			Result->OceanRefinedParents = OceanMesh->RefinedTerrainParents;
			Water.Frame = OceanMesh->Frame;
			const PlanetCore::GroundFrame &WF = Water.Frame;
			const std::optional<PlanetCore::SurfaceTextureFrame> WaterTexture = PlanetCore::SurfaceTextureFrame::Create(
				*PlanetCore::UnitDirection::FromVector(WF.Up),
				WF.RadiusMeters,
				WF.AnchorElevationMeters
			);
			// 有効な候補又は支持領域が無い場合の返戻
			if(!WaterTexture) return {};
			Water.TextureOrigin = WaterTexture->Phase;
			Water.MaximumFloatErrorMeters = OceanMesh->MaximumFloatErrorMeters;
			const FMatrix WaterBasis(ToUnreal(WF.East), ToUnreal(WF.North), ToUnreal(WF.Up), FVector::ZeroVector);
			Water.Transform = FTransform(FQuat(WaterBasis), ToUnreal(WF.Up) * (WF.RadiusMeters + WF.AnchorElevationMeters) * 100);
			if(Request.bRenderRiverWater) {
				PlanetCore::WaterRefinementDiagnostics &Diagnostics = Water.RiverDiagnostics;
				const std::function<PlanetCore::WaterCoverageSample(PlanetCore::UnitDirection)> EvaluateWater =
				[&](const PlanetCore::UnitDirection D) {
					const PlanetCore::RiverWaterHeights Sample = Result->WaterEvaluations->EvaluateWater(D);
					const double W = Sample.WaterSupport > 0 ? Sample.WaterElevationMeters : Request.Settings.SeaLevelMeters;
					// 地形と同じ不変場に基付く水面と符号付き水深の返戻
					return PlanetCore::WaterCoverageSample{ WF.SurfaceToLocal(D, W), W - Sample.TerrainElevationMeters };
				};
				const PlanetCore::WaterCoverageView WaterView {
					Request.View.ObserverMeters,
					Request.View.FocalLengthPixels,
					8,
					std::max(0.0, Request.Settings.RadiusMeters - Result->Surface->AbsoluteReliefBoundMeters())
				};
				std::vector<std::uint32_t> ChannelCandidates;
				ChannelCandidates.reserve(64);
				const PlanetCore::WaterDetailEvaluator RequiredWaterEdge = [&](
					const PlanetCore::UnitDirection Center,
					const double Cap,
					const std::array<PlanetCore::UnitDirection, 3> &Triangle,
					const std::array<double, 3> &Depths
				) {
					const double Radius = Request.Settings.RadiusMeters, Unlimited = std::numeric_limits<double>::infinity();
					const double Relief = Result->Surface->AbsoluteReliefBoundMeters();
					const PlanetCore::Vec3 C = Center.Vector(), Position = C * Radius, Ray = Position - WaterView.ObserverMeters;
					const double RaySquared = Ray.Dot(Ray);
					const double T = RaySquared > 0 ? std::clamp(-WaterView.ObserverMeters.Dot(Ray) / RaySquared, 0.0, 1.0) : 0;
					const bool Occluded =
					T > 0 && T < 1 && (WaterView.ObserverMeters + Ray * T).Length() + Cap * Radius + Relief < WaterView.OccluderRadiusMeters;
					// 遮蔽された遠景の河道条件を省く返戻
					if(Occluded) return Unlimited;
					const PlanetCore::Vec3 Padding { Cap, Cap, Cap };
					const bool ValidCandidates = Result->Surface->GatherChannelCandidates(C - Padding, C + Padding, ChannelCandidates);
					// 候補照会の不合格を保持する返戻
					if(!ValidCandidates) return std::numeric_limits<double>::quiet_NaN();
					double Limit = Unlimited;
					for(const std::uint32_t Index : ChannelCandidates) {
						const PlanetCore::RiverTerrainChannel &Channel = Result->Surface->Channels()[Index];
						const double Angle = std::clamp(std::atan2(C.Dot(Channel.Tangent), C.Dot(Channel.Start)), 0.0, Channel.ArcAngle);
						const PlanetCore::Vec3 Closest = Channel.Planform ?
						Channel.Planform->EvaluateAngle(Angle).Direction.Vector() :
						Channel.Start * std::cos(Angle) + Channel.Tangent * std::sin(Angle);
						// 流路の発見は中心線を含み得る面だけに要求し，乾いた岸迄河床幅の格子を敷かない
						const double DiscoveryCap = Channel.Planform ? Cap + 2 * Channel.Planform->MaximumDeviationRadians() : Cap;
						if((C - Closest).Length() > DiscoveryCap + 1E-14) continue;
						if(
							!Channel.Planform && !PlanetCore::ArcIntersectsSphericalTriangle(
								*PlanetCore::UnitDirection::FromVector(Channel.Start),
								*PlanetCore::UnitDirection::FromVector(Channel.End),
								Triangle
							)
						) continue;
						const std::optional<PlanetCore::RiverHeights> Heights = Result->Drainage->SampleHeights(Index, Angle / Channel.ArcAngle);
						// 対応する縦断が無い河道を見落さない返戻
						if(!Heights) return std::numeric_limits<double>::quiet_NaN();
						const double Distance = std::max(
							0.01,
							(Closest * (Radius + Heights->WaterMeters) - WaterView.ObserverMeters).Length() - Cap * Radius - Channel.InnerHalfWidthMeters
						);
						if(2 * Channel.InnerHalfWidthMeters * WaterView.FocalLengthPixels < Distance) continue;
						// 現在の線形水域が河道全区間を覆う場合，河道内部の等方細分化を追加しない
						if(
							!Channel.Planform && PlanetCore::ArcCoveredByLinearWater(
								*PlanetCore::UnitDirection::FromVector(Channel.Start),
								*PlanetCore::UnitDirection::FromVector(Channel.End),
								Triangle,
								Depths
							)
						) continue;
						if(
							Channel.Planform &&
							Channel.Planform->IsCoveredByLinearWater(std::min(Channel.ArcAngle, Channel.Planform->ArcRadians()), Triangle, Depths)
						) continue;
						Limit = std::min(Limit, Channel.BedHalfWidthMeters * 0.5);
					}
					// 視認可能な河道を先に確保する必要辺長の返戻
					return Limit;
				};
				std::optional<PlanetCore::AdaptiveWaterCoverage> Coverage = PlanetCore::BuildAdaptiveWaterCoverage(
					std::move(OceanMesh->Sphere),
					WF,
					*Anchor,
					Result->Request.View.SurfaceRadiusMeters,
					EvaluateWater,
					Diagnostics,
					Stop,
					&WaterView,
					RequiredWaterEdge,
					std::move(OceanMesh->Hierarchy)
				);
				// 精度不足と予算超過を黙って公開せず，最後の誤差を含む拒否理由の返戻
				if(!Coverage) {
					if(bTryRetained && !Stop.stop_requested()) {
						Water.bRejectedRetainedPartition = true;
						continue;
					}
					FString Reason = FString::Printf(
						TEXT(
							"River water refinement failed: stage %u, lod_status %u, pass %u, faces %llu, water %.9g m, "
							"shore %.9g m, float %.9g m, edge %.9g px. "
							"Source %llu vertices / %llu triangles, planned %llu vertices, feature faces %llu, shoreline edges %llu. "
							"Shore A=(%.17g,%.17g,%.17g) B=(%.17g,%.17g,%.17g). "
							"Source directions=(%.17g,%.17g,%.17g);(%.17g,%.17g,%.17g);(%.17g,%.17g,%.17g)."
						),
						static_cast<unsigned>(Diagnostics.RefinementStage),
						static_cast<unsigned>(Diagnostics.LastLodEditStatus),
						Diagnostics.Pass,
						static_cast<unsigned long long>(Diagnostics.FailedTriangles),
						Diagnostics.WaterErrorMeters,
						Diagnostics.ShoreErrorMeters,
						Diagnostics.FloatErrorMeters,
						Diagnostics.MaximumShoreEdgePixels,
						static_cast<unsigned long long>(Diagnostics.SourceVertices),
						static_cast<unsigned long long>(Diagnostics.SourceTriangles),
						static_cast<unsigned long long>(Diagnostics.PlannedVertices),
						static_cast<unsigned long long>(Diagnostics.FeatureTriangles),
						static_cast<unsigned long long>(Diagnostics.ProjectedEdges),
						Diagnostics.WorstShoreStartMeters.X,
						Diagnostics.WorstShoreStartMeters.Y,
						Diagnostics.WorstShoreStartMeters.Z,
						Diagnostics.WorstShoreEndMeters.X,
						Diagnostics.WorstShoreEndMeters.Y,
						Diagnostics.WorstShoreEndMeters.Z,
						Diagnostics.WorstShoreSourceDirections[0].X,
						Diagnostics.WorstShoreSourceDirections[0].Y,
						Diagnostics.WorstShoreSourceDirections[0].Z,
						Diagnostics.WorstShoreSourceDirections[1].X,
						Diagnostics.WorstShoreSourceDirections[1].Y,
						Diagnostics.WorstShoreSourceDirections[1].Z,
						Diagnostics.WorstShoreSourceDirections[2].X,
						Diagnostics.WorstShoreSourceDirections[2].Y,
						Diagnostics.WorstShoreSourceDirections[2].Z
					);
					for(const PlanetCore::WaterRefinementPass &Pass : Diagnostics.Passes) {
						Reason += FString::Printf(
							TEXT(" P%u[v=%llu,f=%llu,px=%.8g,level=%u,c=(%.9g,%.9g,%.9g)]"),
							Pass.Pass,
							static_cast<unsigned long long>(Pass.SourceVertices),
							static_cast<unsigned long long>(Pass.FailedTriangles),
							Pass.Pixels,
							Pass.RefinementLevel,
							Pass.CenterMeters.X,
							Pass.CenterMeters.Y,
							Pass.CenterMeters.Z
						);
					}
					// 水際精度・被覆の不合格理由を保持する返戻
					return Reject(Reason);
				}
				std::vector<PlanetCore::UnitDirection> WaterDirections;
				WaterDirections.reserve(Coverage->Wet.Vertices.size());
				OceanMesh->Positions.clear();
				OceanMesh->Positions.reserve(Coverage->Wet.Vertices.size());
				Water.MaximumFloatErrorMeters = Coverage->MaximumFloatErrorMeters;
				for(const PlanetCore::WaterCoverageVertex &Vertex : Coverage->Wet.Vertices) {
					// 水際頂点の準備中に中断された場合の返戻
					if(Stop.stop_requested()) return {};
					const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(WF.LocalToPlanet(Vertex.Position));
					WaterDirections.push_back(D);
					OceanMesh->Positions.push_back(Vertex.Position);
				}
				OceanMesh->Sphere.Directions = std::move(WaterDirections);
				OceanMesh->Sphere.Triangles = std::move(Coverage->Wet.Triangles);
				// 再利用に必要な水面階層が失われた場合の返戻
				if(!Coverage->Hierarchy) return Reject(TEXT("Validated river water lost its source hierarchy."));
				const std::shared_ptr<FRiverWaterPartition> Retained = std::make_shared<FRiverWaterPartition>();
				Retained->Field = Result->Surface;
				Retained->LastTrimObserver = ReuseRiverPartition && ReuseRiverPartition->Field == Result->Surface ?
				ReuseRiverPartition->LastTrimObserver :
				Request.View.ObserverMeters;
				if(Water.CoarseningMilliseconds > 0) Retained->LastTrimObserver = Request.View.ObserverMeters;
				Retained->Tree = std::move(Coverage->Hierarchy->Tree);
				Retained->Mesh.Mesh = std::move(Coverage->Partition);
				Retained->Mesh.TriangleOwners = std::move(Coverage->Hierarchy->TriangleOwners);
				Retained->Mesh.StitchMaskCounts = Coverage->Hierarchy->StitchMaskCounts;
				Result->RiverPartition = Retained;
				Water.bReusedRiverPartition = bTryRetained;
			}
			break;
		}
		// 再利用と再構築の両方に失敗した場合の返戻
		if(!OceanMesh) return Reject(TEXT("No complete water candidate after bounded reuse attempts."));
		const PlanetCore::GroundFrame &WF = Water.Frame;
		const int32 WaterCount = static_cast<int32>(OceanMesh->Positions.size());
		Water.Vertices.Reserve(WaterCount);
		Water.Normals.Reserve(WaterCount);
		Water.Tangents.Reserve(WaterCount);
		Water.UV2.Reserve(WaterCount);
		Water.UV3.Reserve(WaterCount);
		for(int32 i = 0; i < WaterCount; ++i) {
			// 有効な候補又は支持領域が無い場合の返戻
			if(!(i & 127) && Stop.stop_requested()) return {};
			const PlanetCore::UnitDirection D = OceanMesh->Sphere.Directions[i];
			const PlanetCore::Vec3 N = D.Vector();
			Water.Vertices.Add(ToUnreal(OceanMesh->Positions[i]) * 100);
			Water.Normals.Add(ToUnreal(WF.VectorToLocal(N)));
			Water.UV2.Emplace(N.X, N.Y);
			Water.UV3.Emplace(N.Z, 0);
			Water.Tangents.Emplace(ToUnreal(WF.VectorToLocal(PlanetCore::SurfaceMaterialTangent(D))), false);
		}
		Water.Indices.Reserve(static_cast<int32>(OceanMesh->Sphere.Triangles.size() * 3));
		for(const std::array<std::uint32_t, 3> &T : OceanMesh->Sphere.Triangles) {
			Water.Indices.Add(static_cast<int32>(T[0]));
			Water.Indices.Add(static_cast<int32>(T[2]));
			Water.Indices.Add(static_cast<int32>(T[1]));
		}
		Water.BuildMilliseconds = (FPlatformTime::Seconds() - OceanStart) * 1000;
		const PlanetCore::EvaluationCacheStats AfterEvaluations = Result->WaterEvaluations->Statistics();
		Result->CachedWaterSamples = Result->WaterEvaluations->WaterSampleCount();
		Result->WaterSampleCapacity = Result->WaterEvaluations->WaterSampleCapacity();
		Result->EvaluationStats.WaterSamples = AfterEvaluations.WaterSamples - BeforeEvaluations.WaterSamples;
		Result->EvaluationStats.WaterHits = AfterEvaluations.WaterHits - BeforeEvaluations.WaterHits;
		Result->WorkerMilliseconds += Water.BuildMilliseconds;
		// 地表の値を変更せず，完成した水面候補の返戻
		return Result;
	}

	/**
	 * 生成定義と視点に対応する描画・水面の候補を準備する
	 * @param Request 今回の生成要求
	 * @param ReuseGeology 再利用可能な地質
	 * @param ReuseOcean 再利用可能な海陸
	 * @param ReuseDrainage 再利用可能な排水
	 * @param ReuseSurface 再利用可能な最終地形
	 * @param ReuseCache 同じ地形用の LOD 評価キャッシュ
	 * @param ReuseEvaluations 同じ地形用の地点評価キャッシュ
	 * @param PreviousFrame 直前の地表原点
	 * @param PreviousOceanFrame 直前の海面原点
	 * @param ReuseWaterCache 再利用可能な海面キャッシュ
	 * @param ReuseRiverPartition 同じ不変場で成功した有界な水面分割
	 * @param ReuseWaterEvaluations 水面専用の地点評価キャッシュ
	 * @param Stop 中断要求
	 * @return 検査した候補又は拒否理由，中断や準備失敗時は空
	 */
	std::shared_ptr<FPlanetResult> GeneratePlanet(
		FPlanetRequest Request,
		std::shared_ptr<const PlanetCore::GeologyField> ReuseGeology,
		std::shared_ptr<const PlanetCore::OceanConnectivityField> ReuseOcean,
		std::shared_ptr<const PlanetCore::DrainageNetwork> ReuseDrainage,
		std::shared_ptr<const PlanetCore::RiverTerrainField> ReuseSurface,
		std::shared_ptr<PlanetCore::LodSurfaceCache> ReuseCache,
		std::shared_ptr<PlanetCore::TerrainEvaluationCache> ReuseEvaluations,
		std::optional<PlanetCore::GroundFrame> PreviousFrame,
		std::optional<PlanetCore::GroundFrame> PreviousOceanFrame,
		std::shared_ptr<PlanetCore::OceanSurfaceCache> ReuseWaterCache,
		std::shared_ptr<const FRiverWaterPartition> ReuseRiverPartition,
		std::shared_ptr<PlanetCore::TerrainEvaluationCache> ReuseWaterEvaluations,
		std::stop_token Stop
	) {
		const double Start = FPlatformTime::Seconds();
		const std::shared_ptr<const PlanetCore::RiverTerrainField> OriginalSurface = ReuseSurface;
		const std::shared_ptr<FPlanetResult> Result = std::make_shared<FPlanetResult>();
		Result->Request = Request;
		if(!ReuseGeology || ReuseGeology->Settings().IdentityHash() != Request.Settings.IdentityHash()) {
			const double StageStart = FPlatformTime::Seconds();
			std::optional<PlanetCore::GeologyField> Prepared = PlanetCore::GeologyField::Create(Request.Settings, Stop);
			// 有効な候補又は支持領域が無い場合の返戻
			if(!Prepared) return {};
			ReuseGeology = std::make_shared<const PlanetCore::GeologyField>(std::move(*Prepared));
			Result->GeologyMilliseconds = (FPlatformTime::Seconds() - StageStart) * 1000;
			UE_LOG(
				LogTemp,
				Display,
				TEXT("Planet preparation revision %lld seed %llu: Geology %.3f ms"),
				static_cast<long long>(Request.Revision),
				static_cast<unsigned long long>(Request.Settings.Seed),
				Result->GeologyMilliseconds
			);
			ReuseOcean.reset();
			ReuseDrainage.reset();
			ReuseSurface.reset();
		}
		if(!ReuseOcean) {
			const double StageStart = FPlatformTime::Seconds();
			std::optional<PlanetCore::OceanConnectivityField> Prepared = PlanetCore::OceanConnectivityField::Create(ReuseGeology, Stop);
			// 有効な候補又は支持領域が無い場合の返戻
			if(!Prepared) return {};
			ReuseOcean = std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*Prepared));
			Result->OceanMilliseconds = (FPlatformTime::Seconds() - StageStart) * 1000;
			UE_LOG(
				LogTemp,
				Display,
				TEXT("Planet preparation revision %lld seed %llu: Ocean %.3f ms"),
				static_cast<long long>(Request.Revision),
				static_cast<unsigned long long>(Request.Settings.Seed),
				Result->OceanMilliseconds
			);
			ReuseDrainage.reset();
			ReuseSurface.reset();
			Result->PreparationMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
		}
		if(!ReuseDrainage) {
			const double StageStart = FPlatformTime::Seconds();
			std::optional<PlanetCore::DrainageNetwork> Prepared = PlanetCore::DrainageNetwork::Create(*ReuseOcean, Stop);
			// 有効な候補又は支持領域が無い場合の返戻
			if(!Prepared) return {};
			ReuseDrainage = std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*Prepared));
			Result->DrainageMilliseconds = (FPlatformTime::Seconds() - StageStart) * 1000;
			UE_LOG(
				LogTemp,
				Display,
				TEXT("Planet preparation revision %lld seed %llu: Drainage %.3f ms"),
				static_cast<long long>(Request.Revision),
				static_cast<unsigned long long>(Request.Settings.Seed),
				Result->DrainageMilliseconds
			);
			Result->PreparationMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
		}
		if(!ReuseSurface) {
			const double StageStart = FPlatformTime::Seconds();
			std::optional<PlanetCore::RiverTerrainField> Prepared =
			PlanetCore::RiverTerrainField::CreateWithErosion(ReuseOcean, ReuseDrainage, Stop);
			// 有効な候補又は支持領域が無い場合の返戻
			if(!Prepared) return {};
			ReuseSurface = std::make_shared<const PlanetCore::RiverTerrainField>(std::move(*Prepared));
			Result->SurfaceMilliseconds = (FPlatformTime::Seconds() - StageStart) * 1000;
			UE_LOG(
				LogTemp,
				Display,
				TEXT("Planet preparation revision %lld seed %llu: Surface %.3f ms"),
				static_cast<long long>(Request.Revision),
				static_cast<unsigned long long>(Request.Settings.Seed),
				Result->SurfaceMilliseconds
			);
			// 地形と河川照会で同じ更新後の水位を共有し，所有者の寿命も維持する
			ReuseDrainage = std::shared_ptr<const PlanetCore::DrainageNetwork>(ReuseSurface, &ReuseSurface->Drainage());
			Result->PreparationMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
		}
		Result->Surface = std::move(ReuseSurface);
		Result->Drainage = std::move(ReuseDrainage);
		Result->Geology = std::move(ReuseGeology);
		Result->Ocean = std::move(ReuseOcean);
		if(!ReuseCache || Result->Surface != OriginalSurface) {
			ReuseCache = std::make_shared<PlanetCore::LodSurfaceCache>(
				Request.Settings.RadiusMeters,
				[Surface = Result->Surface](PlanetCore::UnitDirection D) {
					return Surface->EvaluateElevation(D);
				}
			);
		}
		Result->SurfaceCache = std::move(ReuseCache);
		if(!ReuseEvaluations || Result->Surface != OriginalSurface) {
			ReuseEvaluations = std::make_shared<PlanetCore::TerrainEvaluationCache>(Result->Surface);
			PreviousFrame.reset();
		}
		Result->EvaluationCache = std::move(ReuseEvaluations);
		const PlanetCore::EvaluationCacheStats BeforeEvaluations = Result->EvaluationCache->Statistics();
		Result->Request.View.ReliefBoundMeters = Result->Surface->AbsoluteReliefBoundMeters();
		Result->Request.View.SurfaceRadiusMeters = 0;
		Result->Request.View.SurfacePathEndMeters.reset();
		const PlanetCore::Vec3 SurfaceObserver = Request.View.SurfaceObserverMeters.value_or(Request.View.ObserverMeters);
		const std::optional<PlanetCore::UnitDirection> Anchor = PlanetCore::UnitDirection::FromVector(SurfaceObserver);
		const PlanetCore::TerrainSample AnchorSample = Result->Surface->Evaluate(*Anchor);
		const double SurfaceDistance =
		std::abs(SurfaceObserver.Length() - Request.Settings.RadiusMeters - AnchorSample.ElevationMeters);
		if(Request.bRefineNearSurface && SurfaceDistance <= 256) {
			Result->Request.View.SurfaceRadiusMeters =
			std::max(PlanetCore::StreamingFocus::BodyRadiusMeters, Request.View.SurfaceRadiusMeters);
			Result->Request.View.SurfacePathEndMeters = Request.View.SurfacePathEndMeters;
			Result->Request.View.SurfaceEdgeMeters = 2;
			if(Request.FoliagePrecisionRadiusMeters > 0) {
				Result->Request.View.FoliageRadiusMeters = std::max(
					Result->Request.View.SurfaceRadiusMeters,
					std::min(Request.FoliagePrecisionRadiusMeters, Request.Settings.RadiusMeters * 0.02)
				);
			}
			Result->Frame =
			PlanetCore::SelectRenderFrame(*Anchor, Request.Settings.RadiusMeters, AnchorSample.ElevationMeters, PreviousFrame);
			// 有効な候補又は支持領域が無い場合の返戻
			if(!Result->Frame) return {};
			const PlanetCore::GroundFrame &F = *Result->Frame;
			const FMatrix Basis(ToUnreal(F.East), ToUnreal(F.North), ToUnreal(F.Up), FVector::ZeroVector);
			Result->MeshTransform = FTransform(FQuat(Basis), ToUnreal(F.Up) * (F.RadiusMeters + F.AnchorElevationMeters) * 100);
		}
		const auto Reject = [&](FString Reason) {
			Result->FailureReason = MoveTemp(Reason);
			// 準備した評価結果の返戻
			return Result;
		};
		double StageStart = FPlatformTime::Seconds();
		std::optional<PlanetCore::LodSelection> Selection =
		PlanetCore::SelectLodForView(Result->Request.View, Stop, {}, Result->SurfaceCache.get());
		// 有効な候補又は支持領域が無い場合の返戻
		if(Stop.stop_requested()) return {};
		Result->SelectionMilliseconds = (FPlatformTime::Seconds() - StageStart) * 1000;
		// 精度又は被覆を満たさない候補の拒否理由の返戻
		if(!Selection) return Reject(TEXT("LOD selection failed."));
		if(!Selection->SurfaceResolutionSatisfied) {
			// 精度又は被覆を満たさない候補の拒否理由の返戻
			return Reject(
				FString::Printf(
					TEXT("Near LOD budget exhausted: edge %.9g m, height error %.9g m, stitched error %.9g m, leaves %llu."),
					Selection->MaximumSurfaceEdgeMeters,
					Selection->MaximumSurfaceErrorMeters,
					Selection->MaximumStitchErrorMeters,
					static_cast<unsigned long long>(Selection->Tree.Leaves().size())
				)
			);
		}
		if(!Selection->FoliageResolutionSatisfied) {
			// 植生の支持面が精度・予算を満たさない場合の返戻
			return Reject(
				FString::Printf(
					TEXT("Foliage LOD budget exhausted: edge %.9g m, height error %.9g m, stitched error %.9g m, leaves %llu."),
					Selection->MaximumFoliageEdgeMeters,
					Selection->MaximumFoliageErrorMeters,
					Selection->MaximumFoliageStitchErrorMeters,
					static_cast<unsigned long long>(Selection->Tree.Leaves().size())
				)
			);
		}
		Result->HeightEvaluations = Selection->SurfaceHeightEvaluations;
		Result->SurfaceCacheHits = Selection->SurfaceProbeCacheHits;
		Result->NeighborCacheHits = Selection->NeighborCacheHits;
		Result->CachedNeighborTiles = Selection->CachedNeighborTiles;
		Result->CachedTiles = Selection->CachedTiles;
		Result->StitchCacheHits = Selection->StitchProbeCacheHits;
		Result->StitchPasses = Selection->StitchRefinementPasses;
		Result->StitchSplits = Selection->StitchRefinedParents;
		StageStart = FPlatformTime::Seconds();
		std::optional<PlanetCore::AdaptiveSphereMesh> Sphere = PlanetCore::BuildAdaptiveSphere(Selection->Tree);
		// 有効な候補又は支持領域が無い場合の返戻
		if(Stop.stop_requested()) return {};
		Result->TopologyMilliseconds = (FPlatformTime::Seconds() - StageStart) * 1000;
		// 精度又は被覆を満たさない候補の拒否理由の返戻
		if(!Sphere) return Reject(TEXT("Shared LOD triangulation failed."));
		Result->LeafCount = static_cast<int32>(Selection->Tree.Leaves().size());
		Result->MaximumLevel = static_cast<int32>(Selection->DeepestLevel);
		const PlanetCore::SphereMesh &Mesh = Sphere->Mesh;
		const int32 Count = static_cast<int32>(Mesh.Directions.size());
		Result->Vertices.Reserve(Count);
		Result->Normals.Reserve(Count);
		Result->Colors.Reserve(Count);
		Result->Tangents.Reserve(Count);
		Result->UV2.Reserve(Count);
		Result->UV3.Reserve(Count);
		const PlanetCore::UnitDirection TextureAnchor =
		Result->Frame ? *PlanetCore::UnitDirection::FromVector(Result->Frame->Up) : *Anchor;
		const double TextureHeight = Result->Frame ? Result->Frame->AnchorElevationMeters : AnchorSample.ElevationMeters;
		const std::optional<PlanetCore::SurfaceTextureFrame> TextureFrame =
		PlanetCore::SurfaceTextureFrame::Create(TextureAnchor, Request.Settings.RadiusMeters, TextureHeight);
		// 精度又は被覆を満たさない候補の拒否理由の返戻
		if(!TextureFrame) return Reject(TEXT("Invalid surface texture frame."));
		Result->TextureOrigin = Result->Frame ? TextureFrame->Phase : PlanetCore::Vec3{};
		Result->Indices.Reserve(static_cast<int32>(Mesh.Triangles.size() * 3));
		const std::optional<PlanetCore::SurfaceCorridor> NearRegion = Result->Frame ?
		PlanetCore::SurfaceCorridor::Create(
			SurfaceObserver,
			Result->Request.View.SurfacePathEndMeters.value_or(SurfaceObserver),
			Request.Settings.RadiusMeters,
			Result->Request.View.SurfaceRadiusMeters
		) :
		std::nullopt;
		// 近景描画と足元の共通支持範囲が無い場合の返戻
		if(Result->Frame && !NearRegion) return Reject(TEXT("Invalid near-surface corridor."));
		StageStart = FPlatformTime::Seconds();
		for(int32 i = 0; i < Count; ++i) {
			// 有効な候補又は支持領域が無い場合の返戻
			if(!(i & 127) && Stop.stop_requested()) return {};
			const PlanetCore::UnitDirection D = Mesh.Directions[i];
			const PlanetCore::RiverSurfaceSample SurfaceSample = Result->EvaluationCache->EvaluateSurface(D);
			const PlanetCore::TerrainSample Sample = SurfaceSample.Terrain;
			const PlanetCore::Vec3 Position = D.Vector() * (Request.Settings.RadiusMeters + Sample.ElevationMeters);
			const std::optional<PlanetCore::UnitDirection> Normal =
			PlanetCore::UnitDirection::FromVector(Position - Sample.TangentialGradientPerRadian);
			// 有効な候補又は支持領域が無い場合の返戻
			if(!Normal) return {};
			const PlanetCore::Vec3 Local = Result->Frame ? Result->Frame->SurfaceToLocal(D, Sample.ElevationMeters) : Position;
			const PlanetCore::Vec3 LocalNormal = Result->Frame ? Result->Frame->VectorToLocal(Normal->Vector()) : Normal->Vector();
			const FVector V = ToUnreal(Local) * 100, N = ToUnreal(LocalNormal);
			if(NearRegion && NearRegion->Intersects(D.Vector(), 8)) {
				const FVector FloatPosition { FVector3f(V) };
				Result->MaximumSurfaceFloatError = FMath::Max(Result->MaximumSurfaceFloatError, FVector::Distance(V, FloatPosition) / 100);
				if(Result->MaximumSurfaceFloatError > 0.001) {
					// 精度又は被覆を満たさない候補の拒否理由の返戻
					return Reject(FString::Printf(TEXT("Near float position error %.9g m exceeds 0.001 m."), Result->MaximumSurfaceFloatError));
				}
			}
			// 有効な候補又は支持領域が無い場合の返戻
			if(V.ContainsNaN() || N.ContainsNaN()) return {};
			Result->Vertices.Add(V);
			Result->Normals.Add(N);
			const PlanetCore::UnitDirection PlanetNormal = *Normal;
			const PlanetCore::Vec3 PN = PlanetNormal.Vector();
			Result->UV2.Emplace(PN.X, PN.Y);
			Result->UV3.Emplace(PN.Z, 0);
			const PlanetCore::SurfaceMaterialWeights W =
			PlanetCore::EvaluateSurfaceMaterial(D, Request.Settings.RadiusMeters, Request.Settings.SeaLevelMeters, SurfaceSample);
			if(Request.bDiagnosticHeightColors) {
				const double Height = Sample.ElevationMeters * (Request.Settings.RadiusMeters + 250000) / Request.Settings.RadiusMeters;
				Result->Colors.Add(
					Height < 0 ?
					FMath::Lerp(
						FLinearColor(0.012F, 0.045F, 0.11F),
						FLinearColor(0.09F, 0.34F, 0.39F),
						static_cast<float>(FMath::Clamp((Height + 6000) / 6000, 0.0, 1.0))
					) :
					FMath::Lerp(
						FLinearColor(0.16F, 0.26F, 0.07F),
						FLinearColor(0.72F, 0.62F, 0.46F),
						static_cast<float>(FMath::Clamp(Height / 4500, 0.0, 1.0))
					)
				);
			} else {
				Result->Colors.Emplace(
					static_cast<float>(W.Rock),
					static_cast<float>(W.Sand),
					static_cast<float>(W.Soil),
					static_cast<float>(W.Wetness)
				);
			}
			PlanetCore::Vec3 Tangent = PlanetCore::SurfaceMaterialTangent(PlanetNormal);
			if(Result->Frame) Tangent = Result->Frame->VectorToLocal(Tangent);
			Result->Tangents.Emplace(ToUnreal(Tangent), false);
		}
		for(const std::array<std::uint32_t, 3> &T : Mesh.Triangles) {
			Result->Indices.Add(static_cast<int32>(T[0]));
			Result->Indices.Add(static_cast<int32>(T[2]));
			Result->Indices.Add(static_cast<int32>(T[1]));
		}
		Result->VertexMilliseconds = (FPlatformTime::Seconds() - StageStart) * 1000;
		StageStart = FPlatformTime::Seconds();
		if(Result->Frame) {
			const PlanetCore::GroundFrame &F = *Result->Frame;
			const double R0 = F.RadiusMeters + F.AnchorElevationMeters;
			const PlanetCore::Vec3 FocusLocal = F.SurfaceToLocal(*Anchor, AnchorSample.ElevationMeters);
			const PlanetCore::Vec3 PathEnd = Result->Request.View.SurfacePathEndMeters.value_or(SurfaceObserver);
			const std::optional<PlanetCore::UnitDirection> PathEndDirection = PlanetCore::UnitDirection::FromVector(PathEnd);
			// 先読み終点の球面方向が不正な場合の返戻
			if(!PathEndDirection) return Reject(TEXT("Invalid near-surface corridor."));
			const PlanetCore::SurfaceCorridor &Region = *NearRegion;
			const PlanetCore::Vec3 EndLocal =
			F.SurfaceToLocal(*PathEndDirection, Result->EvaluationCache->EvaluateElevation(*PathEndDirection));
			const PlanetCore::Vec3 PathSpan = EndLocal - FocusLocal;
			const double PathLengthSquared = PathSpan.Dot(PathSpan);
			// 鍵は此の候補の実際の float 頂点だけを表し，異なるフレーム・原点・地形間で誤差を再利用しない
			std::unordered_map<std::uint64_t, double> EdgeErrors;
			EdgeErrors.reserve(std::min<std::size_t>(Mesh.Triangles.size(), 8192));
			for(std::size_t i = 0; i < Mesh.Triangles.size(); ++i) {
				// 有効な候補又は支持領域が無い場合の返戻
				if(!(i & 63) && Stop.stop_requested()) return {};
				const std::array<std::uint32_t, 3> T = Mesh.Triangles[i];
				// 必須の近景に接する三角形を，GPU と同じ局所座標及び明示的な float 変換で検査する
				std::array<PlanetCore::Vec3, 3> P;
				for(unsigned k = 0; k < 3; ++k) {
					const FVector3f V(Result->Vertices[static_cast<int32>(T[k])]);
					P[k] = { V.X / 100.0, V.Y / 100.0, V.Z / 100.0 };
				}
				const PlanetCore::Vec3 Center = (P[0] + P[1] + P[2]) * (1.0 / 3);
				double Cap = 0;
				for(const PlanetCore::Vec3 V : P) Cap = std::max(Cap, (V - Center).Length());
				const double PathFraction =
				PathLengthSquared > 0 ? FMath::Clamp((Center - FocusLocal).Dot(PathSpan) / PathLengthSquared, 0.0, 1.0) : 0;
				if((Center - FocusLocal - PathSpan * PathFraction).Length() > Region.TubeRadiusMeters + Region.SagittaMeters + 8 + Cap) {
					continue;
				}
				const std::array<PlanetCore::Vec3, 4> Probes { (P[0] + P[1]) * 0.5, (P[1] + P[2]) * 0.5, (P[2] + P[0]) * 0.5, Center };
				for(unsigned k = 0; k < Probes.size(); ++k) {
					const std::uint64_t Key = k < 3 ? std::uint64_t(std::min(T[k], T[(k + 1) % 3])) << 32 | std::max(T[k], T[(k + 1) % 3]) : 0;
					if(k < 3) {
						const std::unordered_map<std::uint64_t, double>::iterator it = EdgeErrors.find(Key);
						if(it != EdgeErrors.end()) {
							Result->MaximumSurfaceProbeError = std::max(Result->MaximumSurfaceProbeError, it->second);
							++Result->ValidationEdgeHits;
							continue;
						}
					}
					const PlanetCore::Vec3 Q = Probes[k];
					const double Increment = 2 * R0 * Q.Z + Q.Dot(Q), Distance = std::sqrt(R0 * R0 + Increment);
					const double H = F.AnchorElevationMeters + Increment / (Distance + R0);
					const std::optional<PlanetCore::UnitDirection> D = PlanetCore::UnitDirection::FromVector(F.LocalToPlanet(Q));
					// 有効な候補又は支持領域が無い場合の返戻
					if(!D) return {};
					// 接続三角形は予約範囲の外へ伸び得る為，近景の円内に有る地点へ誤差基準を適用する
					if(!Region.Contains(D->Vector())) continue;
					const double Error = std::abs(Result->EvaluationCache->EvaluateElevation(*D) - H);
					++Result->ValidationHeightEvaluations;
					if(!std::isfinite(Error) || Error > 0.02) {
						// 精度又は被覆を満たさない候補の拒否理由の返戻
						return Reject(
							FString::Printf(
								TEXT(
									"Near render height error %.9g m exceeds 0.02 m; triangle %llu, level %u, root %u " "path %llu, direction %.17g %.17g %.17g."
								),
								Error,
								static_cast<unsigned long long>(i),
								static_cast<unsigned>(Sphere->TriangleOwners[i].Level),
								static_cast<unsigned>(Sphere->TriangleOwners[i].RootFace),
								static_cast<unsigned long long>(Sphere->TriangleOwners[i].Path),
								D->Vector().X,
								D->Vector().Y,
								D->Vector().Z
							)
						);
					}
					if(k < 3) EdgeErrors.emplace(Key, Error);
					Result->MaximumSurfaceProbeError = std::max(Result->MaximumSurfaceProbeError, Error);
				}
			}
		}
		// 有効な候補又は支持領域が無い場合の返戻
		if(Stop.stop_requested()) return {};
		Result->ValidationMilliseconds = (FPlatformTime::Seconds() - StageStart) * 1000;
		const PlanetCore::EvaluationCacheStats AfterEvaluations = Result->EvaluationCache->Statistics();
		Result->CachedSamples = Result->EvaluationCache->SampleCount();
		Result->SampleCapacity = Result->EvaluationCache->SampleCapacity();
		Result->EvaluationStats = {
			AfterEvaluations.Samples - BeforeEvaluations.Samples,
			AfterEvaluations.SampleHits - BeforeEvaluations.SampleHits,
			AfterEvaluations.Heights - BeforeEvaluations.Heights,
			AfterEvaluations.HeightHits - BeforeEvaluations.HeightHits
		};
		Result->WorkerMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
		Result->WaterInput = std::make_shared<FPlanetWaterInput>();
		Result->WaterInput->Request = Request;
		Result->WaterInput->Tree = std::move(Selection->Tree);
		// 地表と水面の二つのワーカーが同じ隣接 LRU を変更しない
		Result->WaterInput->Tree.DetachNeighborCache();
		Result->WaterInput->Mesh = std::move(*Sphere);
		if(
			Request.bPrioritizeSurface && Request.bRenderRiverWater && ReuseRiverPartition &&
			ReuseRiverPartition->Field == Result->Surface && OriginalSurface == Result->Surface
		) {
			Result->Publication = EPlanetPublication::Surface;
			// 地表ワーカーを解放し，公開後の水面専用待機列へ渡す候補の返戻
			return Result;
		}
		Result->WaterEvaluations = OriginalSurface == Result->Surface && ReuseWaterEvaluations ?
		std::move(ReuseWaterEvaluations) :
		std::make_shared<PlanetCore::TerrainEvaluationCache>(Result->Surface, 0, 0);
		const std::shared_ptr<FPlanetWaterInput> WaterInput = std::move(Result->WaterInput);
		// 最初の定義・別 Seed・別半径では地表と水面を揃えて公開する候補の返戻
		return GeneratePlanetWater(
			Result,
			*WaterInput,
			PreviousOceanFrame,
			std::move(ReuseWaterCache),
			std::move(ReuseRiverPartition),
			Stop
		);
	}
}

struct FPlanetTerrainState {
	std::optional<FPlanetRequest> Latest;
	std::stop_source Cancellation;
	TFuture<std::shared_ptr<FPlanetResult>> Future;
	TFuture<FSurfacePreparationResult> PreparationFuture;
	std::shared_ptr<PlanetCore::LodSurfacePreparation> Preparation;
	PlanetCore::SurfacePreparationProgress PreparationProgress;
	PlanetCore::LodView PublishedView;
	bool bWorker = false;
	bool bPreparing = false;
	std::shared_ptr<FPlanetResult> Upload;
	FRenderCommandFence Fence;
	int32 ActiveMesh = 0;
	int32 ActiveWater = 0;
	std::stop_source WaterCancellation;
	TFuture<std::shared_ptr<FPlanetResult>> WaterFuture;
	bool bWaterWorker = false;
	std::shared_ptr<FPlanetResult> LatestWater;
	std::shared_ptr<PlanetCore::TerrainEvaluationCache> WaterEvaluations;
	std::shared_ptr<const PlanetCore::GeologyField> Geology;
	std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean;
	std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage;
	std::shared_ptr<const PlanetCore::RiverTerrainField> Surface;
	std::shared_ptr<PlanetCore::LodSurfaceCache> SurfaceCache;
	std::shared_ptr<PlanetCore::TerrainEvaluationCache> EvaluationCache;
	std::optional<PlanetCore::GroundFrame> PublishedFrame;
	std::optional<PlanetCore::GroundFrame> PublishedOceanFrame;
	std::shared_ptr<PlanetCore::OceanSurfaceCache> WaterCache;
	std::shared_ptr<const FRiverWaterPartition> RiverPartition;
	FVector PublishedObserver = FVector::ZeroVector, PublishedSurfaceObserver = FVector::ZeroVector;
};

/**
 * 地形と海面の表示バッファを準備する
 */
APlanetTerrain::APlanetTerrain() {
	PrimaryActorTick.bCanEverTick = true;
	// 所有する Pawn が切替わる際に旧カメラの粗い視点で近景を置換しない様，カメラ更新後の TG_PostUpdateWork を使う
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;
	PlanetRoot = CreateDefaultSubobject<USceneComponent>(TEXT("PlanetRoot"));
	SetRootComponent(PlanetRoot);
	MeshA = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("TerrainBufferA"));
	MeshB = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("TerrainBufferB"));
	OceanA = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("OceanBufferA"));
	OceanB = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("OceanBufferB"));
	for(UProceduralMeshComponent *const Mesh : { MeshA.Get(), MeshB.Get(), OceanA.Get(), OceanB.Get() }) {
		Mesh->SetupAttachment(PlanetRoot);
		Mesh->SetFlags(RF_Transient);
		Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Mesh->SetCanEverAffectNavigation(false);
		Mesh->SetVisibility(true);
	}
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(
		TEXT("/Game/Planet/Materials/M_PlanetSurface.M_PlanetSurface")
	);
	if(Material.Succeeded()) SurfaceMaterial = Material.Object;
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> WaterMaterial(
		TEXT("/Game/Planet/Materials/M_PlanetOcean.M_PlanetOcean")
	);
	if(WaterMaterial.Succeeded()) OceanMaterial = WaterMaterial.Object;
	OceanA->SetCastShadow(false);
	OceanB->SetCastShadow(false);
	State = MakeShared<FPlanetTerrainState>();
	// 終了
	return;
}

/**
 * 開始時の設定に従って地形生成を要求する
 */
void APlanetTerrain::BeginPlay() {
	Super::BeginPlay();
	if(bGenerateOnBeginPlay) RequestGeneration();
	// 終了
	return;
}

/**
 * プレイ終了時に準備中の地形へ中断を要求する
 * @param Reason 終了理由
 */
void APlanetTerrain::EndPlay(const EEndPlayReason::Type Reason) {
	CancelGeneration();
	Super::EndPlay(Reason);
	// 終了
	return;
}

/**
 * 除去時に非同期準備を中断し，所有する状態を解放する
 */
void APlanetTerrain::BeginDestroy() {
	CancelGeneration();
	State.Reset();
	Super::BeginDestroy();
	// 終了
	return;
}

/**
 * 現在の準備と待機要求を取消す
 */
void APlanetTerrain::CancelGeneration() {
	if(State) {
		State->Cancellation.request_stop();
		State->Latest.reset();
		State->WaterCancellation.request_stop();
		State->LatestWater.reset();
	}
	// 終了
	return;
}

/**
 * 現在の設定と身体・カメラから生成要求を作る
 * @return 入力が有効で要求を受理した場合に真
 */
bool APlanetTerrain::RequestGeneration() {
	// 入力又は公開状態が不正な場合の返戻
	if(!State) return false;
	FPlanetRequest Request;
	Request.Settings.Seed = static_cast<std::uint64_t>(Seed);
	Request.Settings.RadiusMeters = RadiusMeters;
	Request.View.ObserverMeters = { ObserverMeters.X, ObserverMeters.Y, ObserverMeters.Z };
	Request.View.RadiusMeters = RadiusMeters;
	Request.View.ReliefBoundMeters = 40000 * RadiusMeters / (RadiusMeters + 250000);
	Request.View.LeafBudget = static_cast<std::size_t>(LeafBudget);
	Request.View.MaximumLevel = static_cast<std::uint8_t>(MaximumLevel);
	Request.View.FocalLengthPixels = FocalLengthPixels;
	Request.View.TargetEdgePixels = TargetEdgePixels;
	Request.bRefineNearSurface = bRefineNearSurface;
	Request.FoliagePrecisionRadiusMeters = FoliagePrecisionRadiusMeters;
	Request.bDiagnosticHeightColors = bDiagnosticHeightColors;
	Request.bRenderRiverWater = bRenderRiverWater;
	if(bFollowPlayerCamera && GetWorld() && GetWorld()->IsGameWorld()) {
		const APlayerController *const Controller = GetWorld()->GetFirstPlayerController();
		const APlanetExplorerCharacter *const Body = Controller ? Cast<APlanetExplorerCharacter>(Controller->GetPawn()) : nullptr;
		if(Body && Body->Terrain == this) {
			const FVector Position = GetActorTransform().InverseTransformPosition(Body->GetActorLocation()) / 100;
			const FVector Velocity = GetActorTransform().InverseTransformVectorNoScale(Body->GetVelocity()) / 100;
			const FVector Forward = GetActorTransform().InverseTransformVectorNoScale(Body->GetActorForwardVector());
			const double Horizon = FMath::Max(1.5, FMath::Max(0.0, LastWorkerMilliseconds) * 0.001 + 0.25);
			double AllowedSpeed = 0;
			for(const double Candidate : { Body->WalkSpeedMetersPerSecond, Body->RunSpeedMetersPerSecond }) {
				if(Candidate > 20) continue;
				if(FMath::IsFinite(Candidate) && Candidate > 0) AllowedSpeed = FMath::Max(AllowedSpeed, Candidate);
			}
			const std::optional<PlanetCore::StreamingFocus> Focus = PlanetCore::PredictStreamingFocus(
				{ Position.X, Position.Y, Position.Z },
				{ Velocity.X, Velocity.Y, Velocity.Z },
				{ Forward.X, Forward.Y, Forward.Z },
				RadiusMeters,
				Horizon,
				AllowedSpeed
			);
			if(!Focus) {
				GenerationMessage = TEXT("Invalid streaming focus; published terrain retained.");
				// 不正な移動条件からの生成要求を拒否する返戻
				return false;
			}
			Request.View.SurfaceObserverMeters = Focus->StartMeters;
			Request.View.SurfacePathEndMeters = Focus->EndMeters;
			Request.View.SurfaceRadiusMeters = Focus->RadiusMeters;
			Request.bPrioritizeSurface = true;
		}
	}
	if(
		!Request.Settings.IsValid() || MaximumLevel < 1 || MaximumLevel > 32 || !Request.View.IsValid() ||
		!FMath::IsFinite(FoliagePrecisionRadiusMeters) || FoliagePrecisionRadiusMeters < 0 || FoliagePrecisionRadiusMeters > 256 ||
		!GetActorScale3D().Equals(FVector::OneVector, 1E-9) || RequestedRevision == MAX_int64
	) {
		GenerationMessage = TEXT("Invalid request rejected; published terrain retained.");
		// 入力又は公開状態が不正な場合の返戻
		return false;
	}
	if(
		!Request.bPrioritizeSurface || !Request.bRenderRiverWater || !State->Geology ||
		State->Geology->Settings().IdentityHash() != Request.Settings.IdentityHash()
	) {
		State->WaterCancellation.request_stop();
		State->LatestWater.reset();
	}
	Request.Revision = ++RequestedRevision;
	State->Latest = Request;
	State->Cancellation.request_stop();
	bGenerationBusy = true;
	StartPendingGeneration();
	// 要求又は照会が成功した結果の返戻
	return true;
}

/**
 * 利用可能なワーカーへ最新の待機要求を渡す
 */
void APlanetTerrain::StartPendingGeneration() {
	// 現在の準備条件を満たさない場合の終了
	if(!State || State->bWorker || State->bPreparing || State->Upload || !State->Latest) return;
	const FPlanetRequest Request = *State->Latest;
	// 定義変更の初期準備は，前の水面ワーカーが専用キャッシュを解放してから行う
	if(
		State->bWaterWorker && (
			!Request.bPrioritizeSurface || !Request.bRenderRiverWater || !State->Geology ||
			State->Geology->Settings().IdentityHash() != Request.Settings.IdentityHash()
		)
	) return;
	State->Latest.reset();
	State->Cancellation = std::stop_source();
	const std::stop_token Token = State->Cancellation.get_token();
	const std::shared_ptr<const PlanetCore::GeologyField> Geology = State->Geology;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = State->Ocean;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage = State->Drainage;
	const std::shared_ptr<const PlanetCore::RiverTerrainField> Surface = State->Surface;
	const std::shared_ptr<PlanetCore::LodSurfaceCache> SurfaceCache = State->SurfaceCache;
	const std::shared_ptr<PlanetCore::TerrainEvaluationCache> EvaluationCache = State->EvaluationCache;
	const std::optional<PlanetCore::GroundFrame> PreviousFrame = State->PublishedFrame;
	const std::optional<PlanetCore::GroundFrame> PreviousOceanFrame = State->PublishedOceanFrame;
	const std::shared_ptr<PlanetCore::OceanSurfaceCache> WaterCache = State->WaterCache;
	const std::shared_ptr<const FRiverWaterPartition> RiverPartition = State->RiverPartition;
	const std::shared_ptr<PlanetCore::TerrainEvaluationCache> WaterEvaluations = State->WaterEvaluations;
	State->bWorker = true;
	// 地表の可変キャッシュは地表ワーカー１件だけで扱う
	// 水面ワーカーとは不変場だけを共有する
	State->Future = Async(
		EAsyncExecution::ThreadPool,
		[
			Request,
			Geology,
			Ocean,
			Drainage,
			Surface,
			SurfaceCache,
			EvaluationCache,
			PreviousFrame,
			PreviousOceanFrame,
			WaterCache,
			RiverPartition,
			WaterEvaluations,
			Token
		] {
			// 不変の共有場と要求版を渡した非同期候補の返戻
			return GeneratePlanet(
				Request,
				Geology,
				Ocean,
				Drainage,
				Surface,
				SurfaceCache,
				EvaluationCache,
				PreviousFrame,
				PreviousOceanFrame,
				WaterCache,
				RiverPartition,
				WaterEvaluations,
				Token
			);
		}
	);
	GenerationMessage = TEXT("Generating on a worker; existing terrain remains visible.");
	// 終了
	return;
}

/**
 * 完成した地表に対応する最新１件の水面を，専用キャッシュで準備する
 */
void APlanetTerrain::StartPendingWaterGeneration() {
	if(
		!State || State->bWaterWorker || !State->LatestWater || State->Upload && State->Upload->Publication == EPlanetPublication::Water
	) return;
	const std::shared_ptr<FPlanetResult> Result = std::move(State->LatestWater);
	if(!bRenderRiverWater || Result->Surface != State->Surface || !Result->WaterInput) return;
	const std::shared_ptr<FPlanetWaterInput> Input = std::move(Result->WaterInput);
	Result->WaterEvaluations = State->WaterEvaluations;
	if(!Result->WaterEvaluations) {
		Result->WaterEvaluations = std::make_shared<PlanetCore::TerrainEvaluationCache>(Result->Surface, 0, 0);
	}
	const std::shared_ptr<PlanetCore::OceanSurfaceCache> Cache = State->WaterCache;
	const std::shared_ptr<const FRiverWaterPartition> Partition = State->RiverPartition;
	const std::optional<PlanetCore::GroundFrame> Frame = State->PublishedOceanFrame;
	State->WaterCancellation = std::stop_source();
	const std::stop_token Token = State->WaterCancellation.get_token();
	State->bWaterWorker = true;
	State->WaterFuture = Async(
		EAsyncExecution::ThreadPool,
		[Result, Input, Cache, Partition, Frame, Token] {
			return GeneratePlanetWater(Result, *Input, Frame, Cache, Partition, Token);
		}
	);
	// 地表１件・水面１件を上限とし，待機中の水面要求は最新の１件へ置換える終了
	return;
}

/**
 * 移動先の地形評価を公開せずに先読みする
 * @param BodyMeters 惑星座標の身体位置
 * @param VelocityMeters 惑星座標の秒速
 */
void APlanetTerrain::StartSurfacePreparation(FVector BodyMeters, FVector VelocityMeters) {
	// 現在の準備条件を満たさない場合の終了
	if(!State || State->bWorker || State->bPreparing || State->Upload || State->Latest || !State->SurfaceCache) return;
	const double SurfaceRadius = State->PublishedView.SurfaceRadiusMeters;
	// 公開された近景の支持範囲が無い場合の終了
	if(SurfaceRadius <= 0) return;
	const FVector Up = BodyMeters.GetSafeNormal();
	const FVector Tangent = FVector::VectorPlaneProject(VelocityMeters, Up);
	// 現在の準備条件を満たさない場合の終了
	if(Tangent.ContainsNaN() || Tangent.Size() < 0.25) return;
	// 先読みは現在の身体に必要な近景を移動・縮小させず，現在位置の地形と足場の準備を優先する
	const FVector Target = (BodyMeters + Tangent.GetSafeNormal() * 48).GetSafeNormal() * BodyMeters.Size();
	bool NewTarget = !State->Preparation || !State->Preparation->Uses(State->SurfaceCache);
	if(!NewTarget) {
		const PlanetCore::LodView &Prior = State->Preparation->TargetView();
		const std::optional<PlanetCore::UnitDirection> Direction =
		PlanetCore::UnitDirection::FromVector(Prior.SurfaceObserverMeters.value_or(Prior.ObserverMeters));
		const double Drift = FVector::Distance(Target.GetSafeNormal(), ToUnreal(Direction->Vector())) * Prior.RadiusMeters;
		NewTarget = Drift > (State->PreparationProgress.Complete ? 12.0 : 24.0);
		// 現在の準備条件を満たさない場合の終了
		if(!NewTarget && State->PreparationProgress.Complete) return;
	}
	if(NewTarget) {
		PlanetCore::LodView View = State->PublishedView;
		View.SurfaceObserverMeters = PlanetCore::Vec3{ Target.X, Target.Y, Target.Z };
		View.SurfacePathEndMeters.reset();
		State->Preparation = PlanetCore::LodSurfacePreparation::Create(View, State->SurfaceCache);
		State->PreparationProgress = {};
		// 現在の準備条件を満たさない場合の終了
		if(!State->Preparation) return;
	}
	State->Cancellation = std::stop_source();
	const std::stop_token Token = State->Cancellation.get_token();
	const std::shared_ptr<PlanetCore::LodSurfacePreparation> Preparation = State->Preparation;
	State->bPreparing = true;
	bPreparationBusy = true;
	State->PreparationFuture = Async(
		EAsyncExecution::ThreadPool,
		[Preparation, Token] {
			FSurfacePreparationResult Result;
			const double Start = FPlatformTime::Seconds();
			const PlanetCore::SurfacePreparationProgress Before = Preparation->Advance(0);
			do Result.Progress = Preparation->Advance(256, Token);
			while(!Result.Progress.Complete && !Token.stop_requested());
			Result.NewVisits = Result.Progress.VisitedTiles - Before.VisitedTiles;
			Result.NewEvaluations = Result.Progress.HeightEvaluations - Before.HeightEvaluations;
			Result.bInterrupted = Token.stop_requested() && !Result.Progress.Complete;
			Result.Milliseconds = (FPlatformTime::Seconds() - Start) * 1000;
			// 先読みの進捗と実測時間の返戻
			return Result;
		}
	);
	// 終了
	return;
}

/**
 * 終了した準備を回収し，転送済みの有効な表示面へ切替える
 */
void APlanetTerrain::PollGeneration() {
	// 現在の準備条件を満たさない場合の終了
	if(!State) return;
	if(State->bPreparing && State->PreparationFuture.IsReady()) {
		const FSurfacePreparationResult Ready = State->PreparationFuture.Consume();
		State->bPreparing = false;
		bPreparationBusy = false;
		State->PreparationProgress = Ready.Progress;
		++PreparationSlices;
		if(Ready.bInterrupted) ++InterruptedPreparations;
		PreparedTileVisits += static_cast<int64>(Ready.NewVisits);
		PreparedHeightEvaluations += static_cast<int64>(Ready.NewEvaluations);
		LastPreparationMilliseconds = Ready.Milliseconds;
	}
	const auto CanPublish = [&](const std::shared_ptr<FPlanetResult> &Candidate) {
		// 公開候補が無い場合の返戻
		if(!Candidate) return false;
		// 地表候補が最新要求に対応するかの返戻
		if(Candidate->Publication != EPlanetPublication::Water) return Candidate->Request.Revision == RequestedRevision;
		PlanetCore::GeologySettings Current;
		Current.Seed = static_cast<std::uint64_t>(Seed);
		Current.RadiusMeters = RadiusMeters;
		// 古い視点の水面は同じ不変場の時だけ許し，地表の版や座標を巻戻さない
		return bRenderRiverWater && Candidate->Surface == State->Surface &&
		Candidate->Request.Settings.IdentityHash() == Current.IdentityHash() && Candidate->Request.Revision > PublishedWaterRevision &&
		Candidate->Request.Revision <= PublishedRevision;
	};
	if(State->Upload && State->Fence.IsFenceComplete()) {
		if(CanPublish(State->Upload)) {
			const double Start = FPlatformTime::Seconds();
			const FPlanetResult &Ready = *State->Upload;
			if(Ready.Publication != EPlanetPublication::Water) {
				UProceduralMeshComponent *const Next = !State->ActiveMesh ? MeshB.Get() : MeshA.Get();
				UProceduralMeshComponent *const Previous = !State->ActiveMesh ? MeshA.Get() : MeshB.Get();
				Next->SetMeshSectionVisible(0, true);
				Previous->SetMeshSectionVisible(0, false);
				State->ActiveMesh = 1 - State->ActiveMesh;
				LastSurfaceWorkerMilliseconds = Ready.WorkerMilliseconds;
				if(Ready.Publication == EPlanetPublication::Surface) ++EarlySurfacePublications;
			}
			if(Ready.Publication != EPlanetPublication::Surface) {
				UProceduralMeshComponent *const NextWater = !State->ActiveWater ? OceanB.Get() : OceanA.Get();
				UProceduralMeshComponent *const PreviousWater = !State->ActiveWater ? OceanA.Get() : OceanB.Get();
				NextWater->SetMeshSectionVisible(0, true);
				PreviousWater->SetMeshSectionVisible(0, false);
				State->ActiveWater = 1 - State->ActiveWater;
				PublishedWaterRevision = Ready.Request.Revision;
			}
			if(Ready.Publication != EPlanetPublication::Water) {
				State->Geology = Ready.Geology;
				State->Ocean = Ready.Ocean;
				State->Drainage = Ready.Drainage;
				State->Surface = Ready.Surface;
				State->SurfaceCache = Ready.SurfaceCache;
				State->EvaluationCache = Ready.EvaluationCache;
				State->PublishedFrame = Ready.Frame;
			}
			if(Ready.Publication != EPlanetPublication::Surface) {
				State->PublishedOceanFrame = Ready.Water.Frame;
				State->WaterCache = Ready.WaterCache;
				State->WaterEvaluations = Ready.WaterEvaluations;
				State->RiverPartition = Ready.RiverPartition;
				VisibleOceanTriangleCount = Ready.Water.Indices.Num() / 3;
				LastOceanBuildMilliseconds = Ready.Water.BuildMilliseconds;
				MaximumOceanFloatErrorMeters = Ready.Water.MaximumFloatErrorMeters;
			}
			if(Ready.Publication != EPlanetPublication::Water) {
				if(State->Preparation && !State->Preparation->Uses(State->SurfaceCache)) {
					State->Preparation.reset();
					State->PreparationProgress = {};
				}
				DrainageNodeCount = static_cast<int32>(Ready.Drainage->Nodes().size());
				RiverReachCount = static_cast<int32>(Ready.Drainage->Reaches().size());
				RiverMouthCount = static_cast<int32>(Ready.Drainage->Mouths().size());
				MaximumRequiredBankRaiseMeters = Ready.Drainage->Statistics().MaximumRequiredBankRaiseMeters;
				State->PublishedView = Ready.Request.View;
				State->PublishedObserver = ToUnreal(Ready.Request.View.ObserverMeters);
				State->PublishedSurfaceObserver =
				ToUnreal(Ready.Request.View.SurfaceObserverMeters.value_or(Ready.Request.View.ObserverMeters));
				VisibleSurfaceObserverMeters = State->PublishedSurfaceObserver;
				VisibleSurfacePathEndMeters = ToUnreal(
					Ready.Request.View.SurfacePathEndMeters.value_or(
						Ready.Request.View.SurfaceObserverMeters.value_or(Ready.Request.View.ObserverMeters)
					)
				);
				PublishedRevision = Ready.Request.Revision;
				VisibleVertexCount = Ready.Vertices.Num();
				VisibleTriangleCount = Ready.Indices.Num() / 3;
				VisibleSurfaceRadiusMeters = Ready.Request.View.SurfaceRadiusMeters;
				VisibleFoliageRadiusMeters = Ready.Request.View.FoliageRadiusMeters;
				MaximumSurfaceProbeErrorMeters = Ready.MaximumSurfaceProbeError;
				MaximumSurfaceFloatErrorMeters = Ready.MaximumSurfaceFloatError;
				VisibleLeafCount = Ready.LeafCount;
				VisibleMaximumLevel = Ready.MaximumLevel;
				LastWorkerMilliseconds = Ready.WorkerMilliseconds;
				LastFieldPreparationMilliseconds = Ready.PreparationMilliseconds;
				if(Ready.Publication == EPlanetPublication::Surface) {
					State->LatestWater = std::make_shared<FPlanetResult>(Ready);
					State->LatestWater->Publication = EPlanetPublication::Water;
					// 水面は不変場と私有分割だけを使い，地表の可変キャッシュを所有しない
					State->LatestWater->SurfaceCache.reset();
					State->LatestWater->EvaluationCache.reset();
					State->LatestWater->Vertices.Empty();
					State->LatestWater->Normals.Empty();
					State->LatestWater->Indices.Empty();
					State->LatestWater->Colors.Empty();
					State->LatestWater->Tangents.Empty();
					State->LatestWater->UV0.Empty();
					State->LatestWater->UV1.Empty();
					State->LatestWater->UV2.Empty();
					State->LatestWater->UV3.Empty();
				}
			}
			LastSwapMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
			GenerationMessage = FString::Printf(
				TEXT(
					"Terrain published. Worker stages (ms): selection %.3f, topology %.3f, vertices %.3f, validation %.3f. "
					"LOD heights %llu, cached probes %llu, tiles %llu, stitch passes %u splits %u cache %llu. "
					"Validation heights %llu edge hits %llu. Exact reuse vertex hits %llu misses %llu height hits %llu misses %llu. "
					"Erosion, final materials and morphing remain incomplete."
				),
				Ready.SelectionMilliseconds,
				Ready.TopologyMilliseconds,
				Ready.VertexMilliseconds,
				Ready.ValidationMilliseconds,
				static_cast<unsigned long long>(Ready.HeightEvaluations),
				static_cast<unsigned long long>(Ready.SurfaceCacheHits),
				static_cast<unsigned long long>(Ready.CachedTiles),
				Ready.StitchPasses,
				Ready.StitchSplits,
				static_cast<unsigned long long>(Ready.StitchCacheHits),
				static_cast<unsigned long long>(Ready.ValidationHeightEvaluations),
				static_cast<unsigned long long>(Ready.ValidationEdgeHits),
				static_cast<unsigned long long>(Ready.EvaluationStats.SampleHits),
				static_cast<unsigned long long>(Ready.EvaluationStats.Samples),
				static_cast<unsigned long long>(Ready.EvaluationStats.HeightHits),
				static_cast<unsigned long long>(Ready.EvaluationStats.Heights)
			);
			if(Ready.Publication != EPlanetPublication::Surface) {
				GenerationMessage += FString::Printf(
					TEXT(
						" Ocean topology reused=%d, ocean build %.3f ms, refinement passes %u parents %u, "
						"initial near failures %llu pixel failures %llu max pixels %.6g, previous ocean partition=%d."
					),
					Ready.bReusedOceanTopology ? 1 : 0,
					Ready.Water.BuildMilliseconds,
					Ready.OceanRefinementPasses,
					Ready.OceanRefinedParents,
					static_cast<unsigned long long>(Ready.OceanInitialDiagnostics.RejectedNearTriangles),
					static_cast<unsigned long long>(Ready.OceanInitialDiagnostics.RejectedProjectionTriangles),
					Ready.OceanInitialDiagnostics.ProjectionErrorPixels,
					Ready.bReusedPreviousOceanTopology ? 1 : 0
				);
				GenerationMessage += FString::Printf(
					TEXT(" Water coarsening %u parents, %.3f ms."),
					Ready.Water.CoarsenedParents,
					Ready.Water.CoarseningMilliseconds
				);
				if(Ready.Request.bRenderRiverWater) {
					GenerationMessage += FString::Printf(
						TEXT(" River water passes %u, water error %.9g m, shore residual %.9g m, float error %.9g m, shore edge %.9g px."),
						Ready.Water.RiverDiagnostics.Pass,
						Ready.Water.RiverDiagnostics.WaterErrorMeters,
						Ready.Water.RiverDiagnostics.ShoreErrorMeters,
						Ready.Water.RiverDiagnostics.FloatErrorMeters,
						Ready.Water.RiverDiagnostics.MaximumShoreEdgePixels
					);
				}
				if(Ready.Request.bRenderRiverWater) {
					GenerationMessage += FString::Printf(
						TEXT(
							" Water stages (ms): skeleton %.3f, projection %.3f, initial samples %.3f, clipping %.3f, "
							"checks %.3f, refinement %.3f. Retained river partition=%d, rejected retained partition=%d. "
							"Source checks hits=%llu evaluated=%llu."
						),
						Ready.Water.SkeletonMilliseconds,
						Ready.Water.ProjectionMilliseconds,
						Ready.Water.RiverDiagnostics.StageMilliseconds[0],
						Ready.Water.RiverDiagnostics.StageMilliseconds[1],
						Ready.Water.RiverDiagnostics.StageMilliseconds[2],
						Ready.Water.RiverDiagnostics.StageMilliseconds[3],
						Ready.Water.bReusedRiverPartition ? 1 : 0,
						Ready.Water.bRejectedRetainedPartition ? 1 : 0,
						static_cast<unsigned long long>(Ready.Water.RiverDiagnostics.SourceCheckHits),
						static_cast<unsigned long long>(Ready.Water.RiverDiagnostics.SourceCheckEvaluations)
					);
				}
			} else GenerationMessage += TEXT(" Validated surface published ahead of water; previous complete water retained.");
			if(Ready.Publication != EPlanetPublication::Surface && Ready.Request.bRenderRiverWater) {
				GenerationMessage += FString::Printf(
					TEXT(" Scalar water cache %llu / %llu entries, hits %llu misses %llu."),
					static_cast<unsigned long long>(Ready.CachedWaterSamples),
					static_cast<unsigned long long>(Ready.WaterSampleCapacity),
					static_cast<unsigned long long>(Ready.EvaluationStats.WaterHits),
					static_cast<unsigned long long>(Ready.EvaluationStats.WaterSamples)
				);
			}
			GenerationMessage += FString::Printf(
				TEXT(" Exact sample cache %llu / %llu entries."),
				static_cast<unsigned long long>(Ready.CachedSamples),
				static_cast<unsigned long long>(Ready.SampleCapacity)
			);
			GenerationMessage += FString::Printf(
				TEXT(" Selection neighbor cache hits %llu tiles %llu."),
				static_cast<unsigned long long>(Ready.NeighborCacheHits),
				static_cast<unsigned long long>(Ready.CachedNeighborTiles)
			);
			GenerationMessage += FString::Printf(
				TEXT(" Field preparation (ms): geology %.3f, ocean %.3f, drainage %.3f, surface %.3f."),
				Ready.GeologyMilliseconds,
				Ready.OceanMilliseconds,
				Ready.DrainageMilliseconds,
				Ready.SurfaceMilliseconds
			);
			if(Ready.Publication != EPlanetPublication::Water) LastPublishedGenerationDetails = GenerationMessage;
			if(Ready.Publication != EPlanetPublication::Surface) LastPublishedWaterDetails = GenerationMessage;
		} else ++DiscardedRequests;
		State->Upload.reset();
	}
	if(!State->Upload) {
		std::shared_ptr<FPlanetResult> Ready;
		bool bReceived = false;
		if(!bReceived && State->bWorker && State->Future.IsReady()) {
			Ready = State->Future.Consume();
			State->bWorker = false;
			bReceived = true;
		}
		if(!bReceived && State->bWaterWorker && State->WaterFuture.IsReady()) {
			Ready = State->WaterFuture.Consume();
			State->bWaterWorker = false;
			bReceived = true;
		}
		if(bReceived) {
			if(!CanPublish(Ready) || !Ready->FailureReason.IsEmpty()) {
				++DiscardedRequests;
				if(!State->Latest) {
					GenerationMessage = Ready && Ready->Request.Revision == RequestedRevision && !Ready->FailureReason.IsEmpty() ?
					Ready->FailureReason + TEXT(" Previous terrain retained.") :
					TEXT("Generation cancelled or rejected; previous terrain retained.");
				}
			} else {
				const double Start = FPlatformTime::Seconds();
				if(Ready->Publication != EPlanetPublication::Water) {
					UProceduralMeshComponent *const Next = !State->ActiveMesh ? MeshB.Get() : MeshA.Get();
					Next->SetRelativeTransform(Ready->MeshTransform);
					// 非表示バッファも自身の座標基準を保持し，公開中の材質の原点を変更しない
					ApplyTextureFrame(Next, SurfaceMaterial, Ready->Frame ? &*Ready->Frame : nullptr, Ready->TextureOrigin);
					Next->CreateMeshSection_LinearColor(
						0,
						Ready->Vertices,
						Ready->Indices,
						Ready->Normals,
						Ready->UV0,
						Ready->UV1,
						Ready->UV2,
						Ready->UV3,
						Ready->Colors,
						Ready->Tangents,
						false
					);
					Next->SetMeshSectionVisible(0, false);
					// 通常は遅延する描画状態の再構築を非表示中に実行し，実際の代理部品の作成後へフェンスを置く
					Next->RecreateRenderState_Concurrent();
				}
				if(Ready->Publication != EPlanetPublication::Surface) {
					UProceduralMeshComponent *const NextWater = !State->ActiveWater ? OceanB.Get() : OceanA.Get();
					const FOceanRenderData &Water = Ready->Water;
					NextWater->SetRelativeTransform(Water.Transform);
					ApplyTextureFrame(NextWater, OceanMaterial, &Water.Frame, Water.TextureOrigin);
					NextWater->CreateMeshSection_LinearColor(
						0,
						Water.Vertices,
						Water.Indices,
						Water.Normals,
						{},
						{},
						Water.UV2,
						Water.UV3,
						{},
						Water.Tangents,
						false
					);
					NextWater->SetMeshSectionVisible(0, false);
					NextWater->RecreateRenderState_Concurrent();
				}
				State->Upload = std::move(Ready);
				State->Fence.BeginFence(FRenderCommandFence::ESyncDepth::RHIThread);
				LastUploadMilliseconds = (FPlatformTime::Seconds() - Start) * 1000;
				GenerationMessage = TEXT("New terrain submitted; old complete coverage retained until fence completion.");
			}
		}
	}
	StartPendingWaterGeneration();
	StartPendingGeneration();
	bWaterGenerationBusy = State->bWaterWorker || State->LatestWater != nullptr ||
	State->Upload && State->Upload->Publication == EPlanetPublication::Water;
	bGenerationBusy =
	State->bWorker || State->Upload && State->Upload->Publication != EPlanetPublication::Water || State->Latest.has_value();
	// 終了
	return;
}

/**
 * 身体と更新済みカメラに従って地形の準備を進める
 * @param DeltaSeconds 前回の更新からの秒数
 */
void APlanetTerrain::Tick(float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	PollGeneration();
	if(State && State->bPreparing && (!bEnableSurfacePreparation || !bFollowPlayerCamera)) State->Cancellation.request_stop();
	// 現在の準備条件を満たさない場合の終了
	if(!State || !State->Ocean || !bFollowPlayerCamera || !GetWorld() || !GetWorld()->IsGameWorld()) return;
	const APlayerController *const Controller = GetWorld()->GetFirstPlayerController();
	// 現在の準備条件を満たさない場合の終了
	if(!Controller || !Controller->PlayerCameraManager) return;
	ObserverMeters = GetActorTransform().InverseTransformPosition(Controller->PlayerCameraManager->GetCameraLocation()) / 100;
	int32 Width = 0, Height = 0;
	Controller->GetViewportSize(Width, Height);
	if(Width > 0) {
		FocalLengthPixels = Width * 0.5 / FMath::Tan(FMath::DegreesToRadians(Controller->PlayerCameraManager->GetFOVAngle() * 0.5));
	}
	const APlanetExplorerCharacter *const Body = Cast<APlanetExplorerCharacter>(Controller->GetPawn());
	const bool HasBody = Body && Body->Terrain == this, GroundBusy = HasBody && IsValid(Body->Ground) && Body->Ground->bBusy;
	if(State->bPreparing && (GroundBusy || !HasBody)) State->Cancellation.request_stop();
	const FVector SurfaceObserver =
	HasBody ? GetActorTransform().InverseTransformPosition(Body->GetActorLocation()) / 100 : ObserverMeters;
	const double CameraThreshold =
	VisibleSurfaceRadiusMeters > 0 ? 12.0 : FMath::Max(5.0, FMath::Abs(ObserverMeters.Size() - RadiusMeters) * 0.1);
	bool NeedsBodyCoverage = false;
	if(HasBody && VisibleSurfaceRadiusMeters > 0) {
		const FVector Velocity = GetActorTransform().InverseTransformVectorNoScale(Body->GetVelocity()) / 100;
		const FVector Forward = GetActorTransform().InverseTransformVectorNoScale(Body->GetActorForwardVector());
		const double AllowedSpeed = FMath::Max(Body->WalkSpeedMetersPerSecond, Body->RunSpeedMetersPerSecond);
		const double Horizon = FMath::Max(1.5, LastWorkerMilliseconds * 0.001 + 0.25);
		const std::optional<PlanetCore::StreamingFocus> Predicted = PlanetCore::PredictStreamingFocus(
			{ SurfaceObserver.X, SurfaceObserver.Y, SurfaceObserver.Z },
			{ Velocity.X, Velocity.Y, Velocity.Z },
			{ Forward.X, Forward.Y, Forward.Z },
			RadiusMeters,
			Horizon,
			AllowedSpeed
		);
		const PlanetCore::Vec3 Start = State->PublishedView.SurfaceObserverMeters.value_or(State->PublishedView.ObserverMeters);
		const std::optional<PlanetCore::SurfaceCorridor> Published = PlanetCore::SurfaceCorridor::Create(
			Start,
			State->PublishedView.SurfacePathEndMeters.value_or(Start),
			RadiusMeters,
			VisibleSurfaceRadiusMeters
		);
		const std::optional<PlanetCore::UnitDirection> Future =
		Predicted ? PlanetCore::UnitDirection::FromVector(Predicted->EndMeters) : std::nullopt;
		NeedsBodyCoverage = !Published || !Future || !Published->Contains(Future->Vector(), 2);
	}
	// 公開済み通路が進行先を覆わない時にだけ次の候補を要求する
	if(!bGenerationBusy && (NeedsBodyCoverage || FVector::Distance(ObserverMeters, State->PublishedObserver) > CameraThreshold)) {
		RequestGeneration();
	}
	if(!bGenerationBusy && !GroundBusy && HasBody && bEnableSurfacePreparation) {
		StartSurfacePreparation(SurfaceObserver, GetActorTransform().InverseTransformVector(Body->GetVelocity()) / 100);
	}
	// 終了
	return;
}

/**
 * 現在公開している地表の描画部品を取得する
 * @return 公開中の地表，未準備の場合は空
 */
UProceduralMeshComponent *APlanetTerrain::GetVisibleMesh() const {
	// 公開済の地表が無い場合の返戻
	if(!State || !PublishedRevision) return nullptr;
	// 公開中の描画バッファの返戻
	return !State->ActiveMesh ? MeshA.Get() : MeshB.Get();
}

/**
 * 公開中の地形と一致する標高と法線を評価する
 * @param Direction 惑星座標の評価方向
 * @param ElevationMeters 標高の出力先
 * @param SurfaceNormal 惑星座標の単位法線の出力先
 * @return 有効な地点の評価に成功した場合に真
 */
bool APlanetTerrain::EvaluateDirection(FVector Direction, double &ElevationMeters, FVector &SurfaceNormal) const {
	// 入力又は公開状態が不正な場合の返戻
	if(!State || !State->Ocean || !State->Geology) return false;
	const std::optional<PlanetCore::UnitDirection> D =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 入力又は公開状態が不正な場合の返戻
	if(!D) return false;
	const PlanetCore::TerrainSample Sample = State->Surface->Evaluate(*D);
	const std::optional<PlanetCore::UnitDirection> N = PlanetCore::UnitDirection::FromVector(
		D->Vector() * (State->Geology->Settings().RadiusMeters + Sample.ElevationMeters) - Sample.TangentialGradientPerRadian
	);
	// 入力又は公開状態が不正な場合の返戻
	if(!N) return false;
	ElevationMeters = Sample.ElevationMeters;
	SurfaceNormal = ToUnreal(N->Vector());
	// 要求又は照会が成功した結果の返戻
	return true;
}

/**
 * 表示と衝突に共通の不変な地形を共有する
 * @return 公開中の地形，未準備の場合は空
 */
std::shared_ptr<const PlanetCore::RiverTerrainField> APlanetTerrain::GetPublishedField() const {
	return State ? State->Surface : nullptr;
}

/**
 * 公開地形と対応する排水を共有する
 * @return 侵食後の河床・水位を持つ排水，未準備の場合は空
 */
std::shared_ptr<const PlanetCore::DrainageNetwork> APlanetTerrain::GetDrainageNetwork() const {
	return State ? State->Drainage : nullptr;
}

/**
 * エディタの表示中も非同期生成の完了を受取る
 * @return ビューポートのみの更新を許可する真
 */
bool APlanetTerrain::ShouldTickIfViewportsOnly() const {
	// エディタ内の非同期処理も更新する指定の返戻
	return true;
}

/**
 * 元の球面標本とは別に追加した大洋の流出口数を取得する
 * @return 公開済みの海岸節点数，未準備時は０
 */
int32 APlanetTerrain::GetCoastalOutletNodeCount() const {
	// 公開中の排水と対応する追加節点数の返戻
	return State && State->Drainage ? static_cast<int32>(State->Drainage->Statistics().CoastalOutletNodes) : 0;
}

/**
 * 公開中の河道に沿う水位と河床を評価する
 * @param ReachIndex 河道番号
 * @param DownstreamFraction 河道の上流端０から下流端１迄の割合
 * @param LocalWaterPositionMeters 惑星座標の水面位置の出力先
 * @param WaterElevationMeters 水位の出力先
 * @param BedElevationMeters 河床標高の出力先
 * @param DownstreamTangent 下流方向の接線の出力先
 * @return 指定した河道と位置を評価出来た場合に真
 */
bool APlanetTerrain::EvaluateRiverProfile(
	int32 ReachIndex,
	double DownstreamFraction,
	FVector &LocalWaterPositionMeters,
	double &WaterElevationMeters,
	double &BedElevationMeters,
	FVector &DownstreamTangent
) const {
	// 入力又は公開状態が不正な場合の返戻
	if(!State || !State->Drainage || ReachIndex < 0) return false;
	const std::optional<PlanetCore::RiverProfileSample> Sample =
	State->Drainage->SampleReach(static_cast<std::size_t>(ReachIndex), DownstreamFraction);
	// 入力又は公開状態が不正な場合の返戻
	if(!Sample) return false;
	LocalWaterPositionMeters = ToUnreal(Sample->Direction) * (State->Drainage->Settings().RadiusMeters + Sample->WaterMeters);
	WaterElevationMeters = Sample->WaterMeters;
	BedElevationMeters = Sample->BedMeters;
	DownstreamTangent = ToUnreal(Sample->DownstreamTangent);
	// 要求又は照会が成功した結果の返戻
	return true;
}

/**
 * 現在公開している球状の海面の描画部品を取得する
 * @return 公開中の海面，未準備の場合は空
 */
UProceduralMeshComponent *APlanetTerrain::GetVisibleOceanMesh() const {
	return State && PublishedRevision > 0 ? !State->ActiveWater ? OceanA.Get() : OceanB.Get() : nullptr;
}

/**
 * 公開している同じ地形と海・河川の基準水位を評価する
 * @param Direction 惑星座標の評価方向
 * @param WaterMeters 球の基準半径からの水位の出力先
 * @param GroundMeters 同じ地点の地形標高の出力先
 * @return 公開済みの有効な地点を評価出来た場合に真
 */
bool APlanetTerrain::EvaluateWaterDirection(FVector Direction, double &WaterMeters, double &GroundMeters) const {
	// 未準備又は不正な方向の返戻
	if(!State || !State->Surface) return false;
	const std::optional<PlanetCore::UnitDirection> D =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 水位照会に必要な球面方向が不正な場合の返戻
	if(!D) return false;
	const PlanetCore::RiverSurfaceSample Sample = State->Surface->EvaluateSurface(*D);
	WaterMeters = Sample.WaterSupport > 0 ? Sample.WaterElevationMeters : State->Surface->Base().Settings().SeaLevelMeters;
	GroundMeters = Sample.Terrain.ElevationMeters;
	// 同じ不変場からの水位と地面の返戻
	return true;
}
