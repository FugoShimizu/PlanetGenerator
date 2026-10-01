#include "PlanetGroundPatch.h"
#include "Core/PlanetFlowSurfaceField.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetRiverTerrain.h"
#include "PlanetTerrain.h"
#include "Async/Async.h"
#include "Chaos/Triangle.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "PhysicsEngine/BodySetup.h"
#include "ProceduralMeshComponent.h"
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

struct FGroundRequest {
	// 不変な場，尺度と公開時の照合に使う要求
	int64 Revision = 0;
	std::shared_ptr<const PlanetCore::RiverTerrainField> Field;
	std::shared_ptr<const PlanetCore::FlowSurfaceField> Flow;
	double RadiusMeters = 0;
	PlanetCore::Vec3 Anchor;
	PlanetCore::GroundPatchSettings Settings;
	FTransform PlanetTransform;
};

namespace {
	struct FGroundResult {
		// 生成要求と検査済の候補面
		FGroundRequest Request;
		PlanetCore::GroundPatch Patch;
		double Milliseconds = 0;
	};

	/**
	 * 同じ倍精度の成分をエンジンの座標へ渡す
	 * @param Point 核心部の座標
	 * @return 成分を変更しない座標
	 */
	FVector ToEngineVector(const PlanetCore::Vec3 Point) {
		return FVector(Point.X, Point.Y, Point.Z);
	}
}

struct FPlanetGroundState {
	// 一件の作業・最新要求・調理候補と公開済足場
	std::optional<FGroundRequest> Pending;
	std::optional<FGroundRequest> PublishedRequest;
	std::shared_ptr<const PlanetCore::FlowSurfaceField> Flow;
	FTransform FlowTransform;
	PlanetCore::GroundFrame PublishedFrame;
	std::stop_source Stop;
	TFuture<std::shared_ptr<FGroundResult>> Future;
	std::shared_ptr<FGroundResult> Cooking;
	TWeakObjectPtr<UBodySetup> PriorBody;
	bool HasWorker = false;
	bool UsesAsyncCook = false;
	int32 Active = 0;
	double CookStarted = 0;
};

namespace {
	struct FGroundRayResult {
		// 同じ調理済三角形への命中と探索件数
		bool HasHit = false;
		bool HasRawGeometryHit = false;
		FVector Impact = FVector::ZeroVector;
		FVector Normal = FVector::ZeroVector;
		int32 Candidates = 0;
		int32 TotalTriangles = 0;
	};

	/**
	 * 同じ調理済三角形へ保守的な候補収集と倍精度の線分判定を適用する
	 * 最悪 O(N)，N は調理済三角形数；全探索の診断も同じ面と交差式を使う
	 * @param Mesh 公開済の衝突面
	 * @param Start ワールド線分の始点，単位はセンチメートル
	 * @param End ワールド線分の終点，単位はセンチメートル
	 * @param UsesAllTriangles 探索木を迂回して全三角形と比較する診断指定
	 * @return 命中と候補数；不正な入力又は未調理の場合は非命中
	 */
	FGroundRayResult TraceCookedGround(
		UProceduralMeshComponent *const Mesh,
		const FVector Start,
		const FVector End,
		const bool UsesAllTriangles
	) {
		FGroundRayResult Result;
		if(!Mesh || Start.ContainsNaN() || End.ContainsNaN() || !Mesh->GetComponentScale().Equals(FVector::OneVector, 1E-9)) {
			// 同じ入力から得た検査結果の返戻
			return Result;
		}
		const FBodyInstance *const Instance = Mesh->GetBodyInstance();
		UBodySetup *const Body = Mesh->GetBodySetup();
		// 同じ入力から得た検査結果の返戻
		if(!Instance || !Instance->IsValidBodyInstance() || !Body) return Result;
		const FTransform Transform = Instance->GetUnrealWorldTransform();
		const Chaos::FVec3 LocalStart = Transform.InverseTransformPositionNoScale(Start);
		const Chaos::FVec3 Delta = Transform.InverseTransformVectorNoScale(End - Start);
		const double Length = Delta.Size();
		// 同じ入力から得た検査結果の返戻
		if(!FMath::IsFinite(Length) || Length <= UE_SMALL_NUMBER) return Result;
		const Chaos::FVec3 Direction = Delta / Length, LocalEnd = LocalStart + Delta;
		Chaos::FAABB3 Query(LocalStart, LocalStart);
		Query.GrowToInclude(LocalEnd);
		double Scale = 1;
		for(int32 k = 0; k < 3; ++k) Scale = FMath::Max(Scale, FMath::Max(FMath::Abs(LocalStart[k]), FMath::Abs(LocalEnd[k])));
		// 単精度の探索木変換を覆う候補箱だけの拡大；面と最終交差の許容値は不変
		Query.Thicken(16 * std::numeric_limits<float>::epsilon() * Scale);
		// 同じ調理済三角形への最近命中の探索
		double Best = Length + 1;
		for(const Chaos::FTriangleMeshImplicitObjectPtr &Geometry : Body->TriMeshGeometries) {
			if(!Geometry) continue;
			double RawTime = 0;
			Chaos::FVec3 RawPoint, RawNormal;
			int32 RawFace = INDEX_NONE;
			if(UsesAllTriangles && Geometry->Raycast(LocalStart, Direction, Length, 0, RawTime, RawPoint, RawNormal, RawFace)) {
				Result.HasRawGeometryHit = true;
			}
			const Chaos::FTrimeshIndexBuffer &Buffer = Geometry->Elements();
			const Chaos::FTriangleMeshImplicitObject::ParticlesType &Particles = Geometry->Particles();
			Result.TotalTriangles += Buffer.GetNumTriangles();
			TArray<int32> Candidates;
			if(UsesAllTriangles) {
				Candidates.Reserve(Buffer.GetNumTriangles());
				for(int32 i = 0; i < Buffer.GetNumTriangles(); ++i) Candidates.Add(i);
			} else Geometry->FindOverlappingTriangles(Query, Candidates);
			Result.Candidates += Candidates.Num();
			// 大小二種類のエンジン索引型を保つ交差照会
			const auto Check = [&]<typename IndexType>(const TArray<Chaos::TVec3<IndexType>> &Elements) -> void {
				for(const int32 Index : Candidates) {
					const Chaos::TVec3<IndexType> Triangle = Elements[Index];
					const Chaos::FVec3 A(Particles.GetX(Triangle[0])), B(Particles.GetX(Triangle[1])), C(Particles.GetX(Triangle[2]));
					double Time = 0;
					Chaos::FVec3 Normal;
					if(Chaos::RayTriangleIntersection(LocalStart, Direction, Length, A, B, C, Time, Normal) && Time < Best) {
						Best = Time;
						Result.HasHit = true;
						Result.Impact = Transform.TransformPositionNoScale(LocalStart + Direction * Time);
						Result.Normal = Transform.TransformVectorNoScale(Normal);
					}
				}
				// 終了
				return;
			};
			if(Buffer.RequiresLargeIndices()) Check(Buffer.GetLargeIndexBuffer());
			else Check(Buffer.GetSmallIndexBuffer());
		}
		// 同じ入力から得た検査結果の返戻
		return Result;
	}
}

/**
 * 破棄又は終了に先立ち，所有する作業を中止して未着手の要求を解放する
 */
void APlanetGroundPatch::Cancel() {
	if(State) {
		State->Stop.request_stop();
		State->Pending.reset();
	}
	// 終了
	return;
}

/**
 * 生成時の不変場と座標変換が現在の要求元に対応する事を確認する
 * @param Request 検査する生成要求
 * @return 其の場と変換を引続き使用出来る場合に真
 */
bool APlanetGroundPatch::IsCurrentSource(const FGroundRequest &Request) const {
	// 所有する河道場の現在性の返戻
	if(Request.Flow) return State && Request.Flow == State->Flow && Request.PlanetTransform.Equals(State->FlowTransform, 1E-9);
	// 通常の惑星生成で公開中の場と変換への照合の返戻
	return IsValid(Terrain) && Request.Field == Terrain->GetPublishedField() &&
	Request.PlanetTransform.Equals(Terrain->GetActorTransform(), 1E-9);
}

/**
 * 空いている作業枠へ最新要求を移し，地形精度の検査を非同期で実行する
 */
void APlanetGroundPatch::StartPending() {
	// 終了
	if(!State || State->HasWorker || State->Cooking || !State->Pending) return;
	// 一件の生成枠への要求移譲
	const FGroundRequest Request = *State->Pending;
	State->Pending.reset();
	State->Stop = std::stop_source();
	const std::stop_token Stop = State->Stop.get_token();
	State->HasWorker = true;
	// エンジンの参照を触らない，候補面と誤差の非同期計算
	State->Future = Async(
		EAsyncExecution::ThreadPool,
		[Request, Stop]() -> std::shared_ptr<FGroundResult> {
			const double Start = FPlatformTime::Seconds();
			PlanetCore::GroundPatchSettings Settings = Request.Settings;
			std::optional<PlanetCore::GroundPatch> Patch;
			while(!Stop.stop_requested()) {
				Patch = PlanetCore::BuildGroundPatch(
					*PlanetCore::UnitDirection::FromVector(Request.Anchor),
					Request.RadiusMeters,
					Settings,
					[&Request](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
						// 通常の惑星場の返戻
						if(!Request.Flow) return Request.Field->Evaluate(D);
						const std::optional<PlanetCore::FlowChannelSection> Sample = Request.Flow->Evaluate(D, false);
						// 欠落を元の地面へ読替えず，生成側で拒否する値を返す
						return Sample ? Sample->Terrain : PlanetCore::TerrainSample{ std::numeric_limits<double>::quiet_NaN(), {} };
					},
					Stop,
					[&Request](const PlanetCore::UnitDirection D) -> double {
						// 通常の惑星場の標高の返戻
						if(!Request.Flow) return Request.Field->EvaluateElevation(D);
						const std::optional<PlanetCore::FlowChannelSection> Sample = Request.Flow->Evaluate(D, false, false);
						// 河床の高さだけを使う内部誤差検査への返戻
						return Sample ? Sample->Terrain.ElevationMeters : std::numeric_limits<double>::quiet_NaN();
					}
				);
				if(Patch || Settings.CellsPerSide > 127) break;
				Settings.CellsPerSide = FMath::Min<std::uint32_t>(128, Settings.CellsPerSide * 2);
			}
			// 不成立又は未準備の場合の返戻
			if(!Patch || Stop.stop_requested()) return {};
			const std::shared_ptr<FGroundResult> Result = std::make_shared<FGroundResult>();
			Result->Request = Request;
			Result->Request.Settings = Settings;
			Result->Patch = std::move(*Patch);
			Result->Milliseconds = (FPlatformTime::Seconds() - Start) * 1000;
			// 同じ入力から得た検査結果の返戻
			return Result;
		}
	);
	Status = TEXT("Preparing local collision on a worker; existing ground retained.");
	// 終了
	return;
}

/**
 * 完了した作業と物理の調理状態を確認し，最新の有効な候補だけを公開する
 */
void APlanetGroundPatch::PollPatch() {
	// 終了
	if(!State) return;
	UProceduralMeshComponent *Next = !State->Active ? BufferB.Get() : BufferA.Get();
	// 調理中の候補の完了と要求元の照合
	if(State->Cooking) {
		UBodySetup *const Body = Next->GetBodySetup();
		const bool IsCooked = Body && Body->bCreatedPhysicsMeshes && (!State->UsesAsyncCook || Body != State->PriorBody.Get());
		const bool IsCurrent = State->Cooking->Request.Revision == RequestedRevision && IsCurrentSource(State->Cooking->Request);
		if(IsCooked && !IsCurrent) {
			Status = TEXT("Obsolete or timed out collision candidate discarded; old ground retained.");
			State->Cooking.reset();
		} else if(!IsCooked && FPlatformTime::Seconds() - State->CookStarted > 30) {
			// 未完了の調理候補を再利用しない有界な待機；公開済足場の保持
			Status = TEXT("Physics cook exceeded deadline; previous ground retained. Pending requests wait for this cook to resolve.");
		} else if(IsCooked) {
			Next->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
			if(Next->GetBodyInstance() && Next->GetBodyInstance()->IsValidBodyInstance()) {
				UProceduralMeshComponent *const Old = !State->Active ? BufferA.Get() : BufferB.Get();
				Old->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				State->Active = 1 - State->Active;
				PublishedRevision = State->Cooking->Request.Revision;
				MaximumProbeErrorMeters = State->Cooking->Patch.MaximumProbeErrorMeters;
				LastWorkerMilliseconds = State->Cooking->Milliseconds;
				State->PublishedRequest = State->Cooking->Request;
				State->PublishedFrame = State->Cooking->Patch.Frame;
				State->Cooking.reset();
				Status = TEXT("Local collision ready. Walking and matching local rendering remain separate acceptance tests.");
			} else Next->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		}
	}
	// 完了した候補の転送と独立な調理開始
	if(State->HasWorker && State->Future.IsReady()) {
		std::shared_ptr<FGroundResult> Ready = State->Future.Consume();
		State->HasWorker = false;
		if(Ready && Ready->Request.Revision == RequestedRevision && IsCurrentSource(Ready->Request)) {
			Next = !State->Active ? BufferB.Get() : BufferA.Get();
			Next->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			const PlanetCore::GroundFrame &Frame = Ready->Patch.Frame;
			const FMatrix Basis(ToEngineVector(Frame.East), ToEngineVector(Frame.North), ToEngineVector(Frame.Up), FVector::ZeroVector);
			const FQuat Rotation = Ready->Request.PlanetTransform.GetRotation() * FQuat(Basis);
			const FVector Location = Ready->Request.PlanetTransform.TransformPosition(
				ToEngineVector(Frame.Up) * (Frame.RadiusMeters + Frame.AnchorElevationMeters) * 100
			);
			// 公開済足場を移動させない，非表示候補だけの原点更新
			Next->SetWorldLocationAndRotation(Location, Rotation);
			TArray<FVector> Positions, Normals;
			TArray<int32> Indices;
			Positions.Reserve(static_cast<int32>(Ready->Patch.Positions.size()));
			Normals.Reserve(Positions.Max());
			for(const PlanetCore::Vec3 P : Ready->Patch.Positions) Positions.Add(ToEngineVector(P) * 100);
			for(const PlanetCore::Vec3 N : Ready->Patch.Normals) Normals.Add(ToEngineVector(N));
			for(const std::array<std::uint32_t, 3> &Triangle : Ready->Patch.Triangles) {
				Indices.Add(Triangle[0]);
				Indices.Add(Triangle[2]);
				Indices.Add(Triangle[1]);
			}
			State->PriorBody = Next->GetBodySetup();
			State->UsesAsyncCook = GetWorld() && GetWorld()->IsGameWorld();
			const TArray<FVector2D> UV;
			const TArray<FLinearColor> Colors;
			const TArray<FProcMeshTangent> Tangents;
			State->Cooking = std::move(Ready);
			State->CookStarted = FPlatformTime::Seconds();
			Next->CreateMeshSection_LinearColor(0, Positions, Indices, Normals, UV, Colors, Tangents, true);
			Status = TEXT("Collision submitted for cooking; old ground retained until physics is valid.");
		} else if(!State->Pending) Status = TEXT("Candidate cancelled or rejected by accuracy probes; old collision retained.");
	}
	StartPending();
	bBusy = State->HasWorker || State->Cooking != nullptr || State->Pending.has_value();
	// 終了
	return;
}

/**
 * 公開済の足場を取得する
 * @return 調理済で有効な衝突面；未公開時は空
 */
UProceduralMeshComponent *APlanetGroundPatch::GetGroundMesh() const {
	// 公開状態に対応する結果の返戻
	return State && PublishedRevision > 0 ? !State->Active ? BufferA.Get() : BufferB.Get() : nullptr;
}

/**
 * 現在の足場が所有する共有河床を，描画側の生成定義と照合する
 * @return 同じ公開済の河床；未準備・世代変更・従来場の場合は空
 */
std::shared_ptr<const PlanetCore::FlowSurfaceField> APlanetGroundPatch::GetPublishedFlowField() const {
	// 不正な公開状態を別の場として共有しない返戻
	if(!State || !State->PublishedRequest || !IsCurrentSource(*State->PublishedRequest)) return {};
	// 調理済の足場と同じ所有権の返戻
	return State->PublishedRequest->Flow;
}

/**
 * 公開済足場が使用する惑星の尺度と座標系を取得する
 * @param PlanetTransform 有効な定義のワールド変換
 * @param RadiusMeters 同じ地点評価を使う惑星半径
 * @return 現在も同じ定義を持つ足場が公開済の場合に真；失敗時は出力を変更しない
 */
bool APlanetGroundPatch::GetPublishedPlanet(FTransform &PlanetTransform, double &RadiusMeters) const {
	// 未公開又は定義が変更された足場への返戻
	if(!State || !State->PublishedRequest || !IsCurrentSource(*State->PublishedRequest)) return false;
	PlanetTransform = State->PublishedRequest->PlanetTransform;
	RadiusMeters = State->PublishedRequest->RadiusMeters;
	// 実際に衝突が使用する座標と尺度の返戻
	return true;
}

/**
 * 足場と同じ不変場から，描画解像度に依らない標高を照会する
 * @param Direction 惑星座標の半径方向
 * @param ElevationMeters 成功時の標高，単位はメートル
 * @return 公開済の同じ場で有限な標高を取得出来た場合に真；失敗時は出力を変更しない
 */
bool APlanetGroundPatch::EvaluatePublishedElevation(const FVector Direction, double &ElevationMeters) const {
	const std::optional<PlanetCore::UnitDirection> Query =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 不正な方向又は未公開の足場への返戻
	if(!Query || !State || !State->PublishedRequest || !IsCurrentSource(*State->PublishedRequest)) return false;
	const FGroundRequest &Request = *State->PublishedRequest;
	double Height = 0;
	if(Request.Flow) {
		const std::optional<PlanetCore::FlowChannelSection> Sample = Request.Flow->Evaluate(*Query, false, false);
		// 河床の評価欠落への返戻
		if(!Sample) return false;
		Height = Sample->Terrain.ElevationMeters;
	} else Height = Request.Field->EvaluateElevation(*Query);
	// 有限な球面を構成しない標高への返戻
	if(!FMath::IsFinite(Height) || !(Request.RadiusMeters + Height > 0)) return false;
	ElevationMeters = Height;
	// 公開済の実足場と同じ標高の返戻
	return true;
}

/**
 * 公開済足場の座標系で身体の半径を覆う余裕を求める
 * @param WorldPosition 身体のワールド位置，単位はセンチメートル
 * @param FootprintRadiusMeters 身体の射影半径，単位はメートル
 * @param MarginMeters 成功時の境界余裕，負の場合は範囲外
 * @return 現在も有効な場と有限な身体範囲から余裕を計算出来た場合に真
 */
bool APlanetGroundPatch::QueryCoverage(
	const FVector WorldPosition,
	const double FootprintRadiusMeters,
	double &MarginMeters
) const {
	const bool HasValidFootprint =
	State && State->PublishedRequest && !WorldPosition.ContainsNaN() && FMath::IsFinite(FootprintRadiusMeters) &&
	FootprintRadiusMeters >= 0;
	// 未公開又は不正な身体範囲の場合の返戻
	if(!HasValidFootprint) return false;
	// 公開済足場の半径方向射影
	const FGroundRequest &Request = *State->PublishedRequest;
	// 不成立又は未準備の場合の返戻
	if(!IsCurrentSource(Request)) return false;
	const FVector Direction = Request.PlanetTransform.InverseTransformPosition(WorldPosition).GetSafeNormal();
	const PlanetCore::GroundFrame &Frame = State->PublishedFrame;
	const double Denominator = FVector::DotProduct(Direction, ToEngineVector(Frame.Up));
	// 不成立又は未準備の場合の返戻
	if(Denominator <= 0) return false;
	const double X = Frame.RadiusMeters * FVector::DotProduct(Direction, ToEngineVector(Frame.East)) / Denominator;
	const double Y = Frame.RadiusMeters * FVector::DotProduct(Direction, ToEngineVector(Frame.North)) / Denominator;
	// 海面下と局所射影の拡大を含む，身体の半径に対する保守的な範囲縮小
	const double Distance = Request.PlanetTransform.InverseTransformPosition(WorldPosition).Size() / 100;
	// 不成立又は未準備の場合の返戻
	if(Distance <= FootprintRadiusMeters) return false;
	const double Expansion =
	FootprintRadiusMeters * Frame.RadiusMeters / (Distance - FootprintRadiusMeters) / (Denominator * Denominator);
	MarginMeters = Request.Settings.HalfExtentMeters - FMath::Max(FMath::Abs(X), FMath::Abs(Y)) - Expansion;
	// 有限な範囲余裕を計算出来た結果の返戻
	return FMath::IsFinite(MarginMeters);
}

/**
 * エンジン標準の線分照会を公開済衝突へ適用する
 * @param Start 線分の始点，単位はセンチメートル
 * @param End 線分の終点，単位はセンチメートル
 * @param Impact 命中時のワールド位置
 * @param Normal 命中時のワールド法線
 * @return 公開済の面へ標準の探索が命中した場合に真
 */
bool APlanetGroundPatch::NativeTraceGround(const FVector Start, const FVector End, FVector &Impact, FVector &Normal) const {
	UProceduralMeshComponent *const Mesh = GetGroundMesh();
	// 不成立又は未準備の場合の返戻
	if(!Mesh) return false;
	FHitResult Hit;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(PlanetGroundProbe), true);
	// 不成立又は未準備の場合の返戻
	if(!Mesh->LineTraceComponent(Hit, Start, End, Params)) return false;
	Impact = Hit.ImpactPoint;
	Normal = Hit.ImpactNormal;
	// 照会又は要求の成功の返戻
	return true;
}

/**
 * 標準経路が欠落した線分を，同じ調理済三角形で再照合する
 * @param Start ワールド線分の始点，単位はセンチメートル
 * @param End ワールド線分の終点，単位はセンチメートル
 * @param Impact 命中時のワールド位置
 * @param Normal 命中時のワールド法線
 * @return 変更していない衝突面に命中した場合に真
 */
bool APlanetGroundPatch::TraceGround(const FVector Start, const FVector End, FVector &Impact, FVector &Normal) const {
	// 照会又は要求の成功の返戻
	if(NativeTraceGround(Start, End, Impact, Normal)) return true;
	const FGroundRayResult Hit = TraceCookedGround(GetGroundMesh(), Start, End, false);
	// 不成立又は未準備の場合の返戻
	if(!Hit.HasHit) return false;
	Impact = Hit.Impact;
	Normal = Hit.Normal;
	// 照会又は要求の成功の返戻
	return true;
}

/**
 * 標準探索・候補探索・全探索の命中差を診断する
 * 最悪 O(N)，N は公開済の三角形数
 * @param Start ワールド線分の始点，単位はセンチメートル
 * @param End ワールド線分の終点，単位はセンチメートル
 * @return 命中と候補件数を記録した既存の診断用 JSON
 */
FString APlanetGroundPatch::InspectGroundRay(const FVector Start, const FVector End) const {
	FVector Impact, Normal;
	const bool HasNativeHit = NativeTraceGround(Start, End, Impact, Normal);
	const FGroundRayResult Candidates = TraceCookedGround(GetGroundMesh(), Start, End, false);
	const FGroundRayResult Brute = TraceCookedGround(GetGroundMesh(), Start, End, true);
	// 探索経路毎の診断結果の返戻
	return FString::Printf(
		TEXT("{\"native_hit\":%s,\"raw_geometry_hit\":%s,\"candidate_hit\":%s,\"brute_hit\":%s,\"candidates\":%d,\"triangles\":%d}"),
		HasNativeHit ? TEXT("true") : TEXT("false"),
		Brute.HasRawGeometryHit ? TEXT("true") : TEXT("false"),
		Candidates.HasHit ? TEXT("true") : TEXT("false"),
		Brute.HasHit ? TEXT("true") : TEXT("false"),
		Candidates.Candidates,
		Brute.TotalTriangles
	);
}

/**
 * 身体の寸法を持つカプセルを公開済衝突へ掃引する
 * @param Start 身体中心の始点，単位はセンチメートル
 * @param End 身体中心の終点，単位はセンチメートル
 * @param Up 身体の上向き
 * @param RadiusCm カプセル半径，正のセンチメートル
 * @param HalfHeightCm 半高，半径以上
 * @param Impact 命中時のワールド位置
 * @param Normal 命中時のワールド法線
 * @return 有効な入力で公開済の面に命中した場合に真
 */
bool APlanetGroundPatch::SweepGroundCapsule(
	const FVector Start,
	const FVector End,
	const FVector Up,
	const double RadiusCm,
	const double HalfHeightCm,
	FVector &Impact,
	FVector &Normal
) const {
	UProceduralMeshComponent *const Mesh = GetGroundMesh();
	const bool HasValidCapsule =
	Mesh && !Start.ContainsNaN() && !End.ContainsNaN() && !Up.ContainsNaN() && !Up.IsNearlyZero() && FMath::IsFinite(RadiusCm) &&
	FMath::IsFinite(HalfHeightCm) && RadiusCm > 0 && HalfHeightCm >= RadiusCm;
	// 未公開又は不正な身体寸法の場合の返戻
	if(!HasValidCapsule) return false;
	const FCollisionShape Shape = FCollisionShape::MakeCapsule(static_cast<float>(RadiusCm), static_cast<float>(HalfHeightCm));
	FHitResult Hit;
	const FQuat Rotation = FQuat::FindBetweenNormals(FVector::UpVector, Up.GetSafeNormal());
	// 不成立又は未準備の場合の返戻
	if(!Mesh->SweepComponent(Hit, Start, End, Rotation, Shape, true)) return false;
	Impact = Hit.ImpactPoint;
	Normal = Hit.ImpactNormal;
	// 照会又は要求の成功の返戻
	return true;
}

/**
 * プレイ中の検査に限って地形又は足場を一時生成する
 * @param WorldContextObject 検査対象の実行世界を持つ参照
 * @param ActorClass 許可済の地形又は足場の型
 * @param Location ワールド位置，単位はセンチメートル
 * @return 生成した一時アクター；不正な世界・型又は生成失敗では空
 */
AActor *APlanetGroundPatch::SpawnProbeActor(
	UObject *const WorldContextObject,
	const TSubclassOf<AActor> ActorClass,
	const FVector Location
) {
	if(!GEngine || Location.ContainsNaN() || ActorClass != APlanetTerrain::StaticClass() && ActorClass != StaticClass()) {
		// 不成立又は未準備の場合の返戻
		return nullptr;
	}
	UWorld *World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::ReturnNull);
	// 不成立又は未準備の場合の返戻
	if(!World || !World->IsGameWorld()) return nullptr;
	const FTransform Transform(Location);
	AActor *const Actor =
	World->SpawnActorDeferred<AActor>(ActorClass, Transform, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	// 不成立又は未準備の場合の返戻
	if(!Actor) return nullptr;
	Actor->SetFlags(RF_Transient);
	if(APlanetTerrain *const TerrainActor = Cast<APlanetTerrain>(Actor)) TerrainActor->bGenerateOnBeginPlay = false;
	Actor->FinishSpawning(Transform);
	// 生成した一時アクターの返戻
	return Actor;
}

/**
 * 公開済の惑星場を最新一件の衝突要求として保持する
 * @return 有効な場と単位拡縮の座標を受理した場合に真；不正時は旧足場を保持
 */
bool APlanetGroundPatch::RequestPatch() {
	// 不成立又は未準備の場合の返戻
	if(!State || !IsValid(Terrain) || RequestedRevision == MAX_int64) return false;
	// 公開中の場と足元寸法の取得
	std::shared_ptr<const PlanetCore::RiverTerrainField> Field = Terrain->GetPublishedField();
	const std::optional<PlanetCore::UnitDirection> Anchor =
	PlanetCore::UnitDirection::FromVector({ AnchorDirection.X, AnchorDirection.Y, AnchorDirection.Z });
	PlanetCore::GroundPatchSettings Settings;
	Settings.HalfExtentMeters = HalfExtentMeters;
	Settings.CellsPerSide = static_cast<std::uint32_t>(CellsPerSide);
	if(
		!Field || !Anchor || !Settings.IsValid(Field->Base().Settings().RadiusMeters) ||
		!Terrain->GetActorScale3D().Equals(FVector::OneVector, 1E-9) || !GetActorScale3D().Equals(FVector::OneVector, 1E-9)
	) {
		Status = TEXT("Invalid or unprepared definition; previous collision retained.");
		// 不成立又は未準備の場合の返戻
		return false;
	}
	const double Radius = Field->Base().Settings().RadiusMeters;
	// 最新要求の置換と旧作業への中止通知
	State->Flow.reset();
	State->Pending =
	FGroundRequest{ ++RequestedRevision, std::move(Field), {}, Radius, Anchor->Vector(), Settings, Terrain->GetActorTransform() };
	State->Stop.request_stop();
	bBusy = true;
	StartPending();
	// 照会又は要求の成功の返戻
	return true;
}

/**
 * 共有河道場を，同じ旧衝突保持と非同期の調理経路へ渡す
 * @param Field 検証済で不変な河床場
 * @param Direction 足元の球面方向
 * @param PlanetTransform 惑星座標からワールド座標への変換，単位拡縮
 * @return 有効な要求を最新の待機枠へ追加した場合に真
 */
bool APlanetGroundPatch::RequestFlowPatch(
	const std::shared_ptr<const PlanetCore::FlowSurfaceField> &Field,
	const FVector Direction,
	const FTransform &PlanetTransform
) {
	// 破棄中又は通番を使切った状態への返戻
	if(!State || !Field || RequestedRevision == MAX_int64) return false;
	const std::optional<PlanetCore::UnitDirection> Anchor =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	PlanetCore::GroundPatchSettings Settings;
	Settings.HalfExtentMeters = HalfExtentMeters;
	Settings.CellsPerSide = static_cast<std::uint32_t>(CellsPerSide);
	const bool IsValidRequest = Anchor && Settings.IsValid(Field->GetRadiusMeters()) && PlanetTransform.IsValid() &&
	PlanetTransform.GetScale3D().Equals(FVector::OneVector, 1E-9) && GetActorScale3D().Equals(FVector::OneVector, 1E-9);
	if(!IsValidRequest) {
		Status = TEXT("Invalid flow field collision request; previous collision retained.");
		// 不正な入力で既存の足場を交換しない返戻
		return false;
	}
	// 同じ地点評価を所有する最新要求の設定
	State->Flow = Field;
	State->FlowTransform = PlanetTransform;
	State->Pending =
	FGroundRequest{ ++RequestedRevision, {}, Field, Field->GetRadiusMeters(), Anchor->Vector(), Settings, PlanetTransform };
	State->Stop.request_stop();
	bBusy = true;
	StartPending();
	// 有界な最新要求への追加成功の返戻
	return true;
}

/**
 * 身体の予測位置を，現在の生成定義を保った足場要求へ変換する
 * @param WorldPosition 有限な身体又は先読みのワールド位置
 * @return 同じ不変場へ有効な最新要求を発行出来た場合に真；旧衝突は公開迄保持
 */
bool APlanetGroundPatch::RequestAtWorldPosition(const FVector WorldPosition) {
	// 破棄中又は不正な身体位置への返戻
	if(!State || WorldPosition.ContainsNaN()) return false;
	if(State->Flow) {
		const FVector Direction = State->FlowTransform.InverseTransformPosition(WorldPosition);
		// 身体位置に追従する共有河床の要求結果の返戻
		return RequestFlowPatch(State->Flow, Direction, State->FlowTransform);
	}
	// 正規の惑星場が無い場合の返戻
	if(!IsValid(Terrain)) return false;
	const FVector Direction = Terrain->GetActorTransform().InverseTransformPosition(WorldPosition).GetSafeNormal();
	// 惑星中心を足元方向に代用しない返戻
	if(Direction.IsNearlyZero()) return false;
	AnchorDirection = Direction;
	// 従来の生成経路を保った再要求の結果の返戻
	return RequestPatch();
}

/**
 * 足場の診断をエディタの視点更新中にも進める
 * @return エディタ内の更新を許可する真
 */
bool APlanetGroundPatch::ShouldTickIfViewportsOnly() const {
	// 照会又は要求の成功の返戻
	return true;
}

/**
 * 毎フレームの候補を確認し，調理済の場合だけ足場を交換する
 * @param DeltaSeconds エンジンが通知する経過秒数
 */
void APlanetGroundPatch::Tick(const float DeltaSeconds) {
	Super::Tick(DeltaSeconds);
	PollPatch();
	// 終了
	return;
}

/**
 * プレイ終了に合わせて所有する生成だけを中止する
 * @param Reason エンジンが通知する終了理由
 */
void APlanetGroundPatch::EndPlay(const EEndPlayReason::Type Reason) {
	Cancel();
	Super::EndPlay(Reason);
	// 終了
	return;
}

/**
 * 所有する作業へ中止を通知し，参照を解放する
 */
void APlanetGroundPatch::BeginDestroy() {
	Cancel();
	State.Reset();
	Super::BeginDestroy();
	// 終了
	return;
}

/**
 * 非表示の二面と不変な生成状態を準備し，足元の更新中も旧衝突を保持する
 */
APlanetGroundPatch::APlanetGroundPatch() {
	// 所有する非表示面と衝突属性の初期化
	PrimaryActorTick.bCanEverTick = true;
	PatchRoot = CreateDefaultSubobject<USceneComponent>(TEXT("PatchRoot"));
	SetRootComponent(PatchRoot);
	BufferA = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("GroundA"));
	BufferB = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("GroundB"));
	for(UProceduralMeshComponent *const Mesh : { BufferA.Get(), BufferB.Get() }) {
		Mesh->SetupAttachment(PatchRoot);
		Mesh->SetFlags(RF_Transient);
		Mesh->SetVisibility(false);
		Mesh->SetHiddenInGame(true);
		Mesh->SetCollisionObjectType(ECC_WorldStatic);
		Mesh->SetCollisionResponseToAllChannels(ECR_Block);
		Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Mesh->SetCanEverAffectNavigation(false);
		Mesh->bUseAsyncCooking = true;
		Mesh->bUseComplexAsSimpleCollision = true;
	}
	State = MakeShared<FPlanetGroundState>();
	// 終了
	return;
}
