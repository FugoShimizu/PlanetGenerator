#include "PlanetContinentalPreview.h"
#include "Core/PlanetCore.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetSphericalContinentalField.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

namespace {
	/**
	 * 診断メッシュへベクトルを変換する
	 * @param Vector 同じ尺度の Core ベクトル
	 * @return UE の倍精度ベクトル
	 */
	FVector ToUnreal(const PlanetCore::Vec3 Vector) {
		return { Vector.X, Vector.Y, Vector.Z };
	}

	/**
	 * 標高だけを示す固定の比較色を選ぶ
	 * @param Height 基準海面からの標高
	 * @param Radius 惑星の物理半径
	 * @return 診断色，海面材質やバイオーム分類ではない
	 */
	FLinearColor HeightColor(const double Height, const double Radius) {
		if(Height < 0) {
			const double Depth = std::min(4000.0, 0.04 * Radius);
			const float Fraction = static_cast<float>(std::clamp(1 + Height / Depth, 0.0, 1.0));
			// 比較用海底の高さ色の返戻
			return FMath::Lerp(FLinearColor(0.015F, 0.04F, 0.12F), FLinearColor(0.10F, 0.40F, 0.43F), Fraction);
		}
		const double Reference = 2000 * std::min(1.0, Radius / 1000000);
		const float Fraction = static_cast<float>(std::clamp(Height / Reference, 0.0, 1.0));
		// 比較用の低地と山地の高さ色の返戻
		return FMath::Lerp(FLinearColor(0.16F, 0.30F, 0.06F), FLinearColor(0.75F, 0.62F, 0.43F), Fraction);
	}
}

/**
 * 一時メッシュを所有し，保存又は衝突面の生成を行わない
 */
APlanetContinentalPreview::APlanetContinentalPreview() {
	PrimaryActorTick.bCanEverTick = false;
	PreviewMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ContinentalCandidate"));
	SetRootComponent(PreviewMesh);
	PreviewMesh->SetFlags(RF_Transient);
	PreviewMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	PreviewMesh->SetCanEverAffectNavigation(false);
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(
		TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial")
	);
	if(Material.Succeeded()) PreviewMesh->SetMaterial(0, Material.Object);
	// 終了
	return;
}

/**
 * 同じ生成定義を再利用し，変更時だけ有限源を準備する
 * @param Seed 生成 Seed
 * @param Radius 惑星の物理半径
 * @return 同じ候補又は準備済みの新しい不変場，失敗時は空
 */
std::shared_ptr<const PlanetCore::SphericalContinentalField> APlanetContinentalPreview::PrepareDefinition(
	const std::uint64_t Seed,
	const double Radius
) {
	PreparationMilliseconds = 0;
	// 未変更の不変場の返戻
	if(Field && PreparedSeed == Seed && PreparedRadiusMeters == Radius) return Field;
	const double Started = FPlatformTime::Seconds();
	PlanetCore::SphericalContinentalFieldError Error = PlanetCore::SphericalContinentalFieldError::None;
	std::optional<PlanetCore::SphericalContinentalField> Candidate =
	PlanetCore::SphericalContinentalField::CreateSeeded(Seed, Radius, Error);
	PreparationMilliseconds = (FPlatformTime::Seconds() - Started) * 1000;
	if(!Candidate) {
		LastBuildMessage = FString::Printf(TEXT("Continental candidate rejected: reason=%u."), static_cast<unsigned>(Error));
		// 未完成の生成定義の返戻
		return {};
	}
	// アクターの寿命と独立した候補の返戻
	return std::make_shared<const PlanetCore::SphericalContinentalField>(std::move(*Candidate));
}

/**
 * 検査を済ませた一件の診断表示と，同じ不変場を公開する
 * @param Candidate 全頂点の照会を通った候補
 * @param Seed 同じ候補の Seed
 * @param Vertices ローカル UE 座標の頂点
 * @param Triangles UE の外向き時計回りの面
 * @param Normals ローカルの正規法線
 * @param Colors 診断用の高さ色
 */
void APlanetContinentalPreview::PublishMesh(
	const std::shared_ptr<const PlanetCore::SphericalContinentalField> &Candidate,
	const std::uint64_t Seed,
	const TArray<FVector> &Vertices,
	const TArray<int32> &Triangles,
	const TArray<FVector> &Normals,
	const TArray<FLinearColor> &Colors
) {
	// 候補全体を同じメッシュへ公開
	const TArray<FVector2D> UVs;
	const TArray<FProcMeshTangent> Tangents;
	PreviewMesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UVs, Colors, Tangents, false);
	Field = Candidate;
	PreparedSeed = Seed;
	PreparedRadiusMeters = Candidate->GetRadiusMeters();
	OceanFraction = Candidate->GetStatistics().OceanFraction;
	GeneratedVertices = Vertices.Num();
	GeneratedTriangles = Triangles.Num() / 3;
	HasSucceededLastBuild = true;
	LastBuildMessage =
	TEXT("Analytic continental candidate shown. Erosion, rivers, natural appearance and walking are not accepted.");
	// 終了
	return;
}

/**
 * 全惑星の構成を小さな表示球へ写し，同じ解析法線と符号を比較する
 * @param Seed 生成 Seed
 * @param Radius 惑星の物理半径
 * @param SubdivisionLevel 診断用の一様球面の深さ，０以上６以下
 * @param HeightScale 表示だけの標高倍率，数値場は不変
 */
void APlanetContinentalPreview::BuildGlobalPreview(
	const int64 Seed,
	const double Radius,
	const int32 SubdivisionLevel,
	const double HeightScale
) {
	// 表示入力の検査
	HasSucceededLastBuild = false;
	const bool IsInvalid =
	SubdivisionLevel < 0 || SubdivisionLevel > 6 || !std::isfinite(HeightScale) || HeightScale <= 0 || HeightScale > 128;
	if(IsInvalid) {
		LastBuildMessage = TEXT("Invalid global comparison display parameters.");
		// 無効な診断表示の終了
		return;
	}
	const std::shared_ptr<const PlanetCore::SphericalContinentalField> Candidate =
	PrepareDefinition(static_cast<std::uint64_t>(Seed), Radius);
	// 未準備の候補で旧表示を置換しない終了
	if(!Candidate) return;
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetCore::SphereMesh> Sphere =
	PlanetCore::BuildUniformSphere(static_cast<std::uint8_t>(SubdivisionLevel));
	// 診断球面を準備出来ない場合の終了
	if(!Sphere) {
		LastBuildMessage = TEXT("Could not prepare the bounded comparison sphere.");
		// 未準備の球面の終了
		return;
	}
	TArray<FVector> Vertices, Normals;
	TArray<int32> Triangles;
	TArray<FLinearColor> Colors;
	Vertices.Reserve(static_cast<int32>(Sphere->Directions.size()));
	Normals.Reserve(Vertices.Max());
	Colors.Reserve(Vertices.Max());
	Triangles.Reserve(static_cast<int32>(Sphere->Triangles.size() * 3));
	for(const PlanetCore::UnitDirection Direction : Sphere->Directions) {
		const std::optional<PlanetCore::TerrainSample> Sample = Candidate->Evaluate(Direction);
		// 全頂点を同じ場で評価出来ない場合の終了
		if(!Sample) {
			LastBuildMessage = TEXT("A valid comparison vertex could not be evaluated; previous geometry retained.");
			// 未解決の比較頂点を公開しない終了
			return;
		}
		const double DisplayRadius = Radius + HeightScale * Sample->ElevationMeters;
		const std::optional<PlanetCore::UnitDirection> Normal =
		PlanetCore::UnitDirection::FromVector(Direction.Vector() * DisplayRadius - Sample->TangentialGradientPerRadian * HeightScale);
		if(DisplayRadius <= 0 || !Normal) {
			LastBuildMessage = TEXT("Height display scale creates an invalid comparison sphere; previous geometry retained.");
			// 内外の反転した候補を公開しない終了
			return;
		}
		// 表示球の半径は１０００㎝，物理半径又は高さ場の変更ではない
		Vertices.Add(ToUnreal(Direction.Vector() * (DisplayRadius / Radius * 1000)));
		Normals.Add(ToUnreal(Normal->Vector()));
		Colors.Add(HeightColor(Sample->ElevationMeters, Radius));
	}
	for(const std::array<std::uint32_t, 3> &Triangle : Sphere->Triangles) {
		Triangles.Add(static_cast<int32>(Triangle[0]));
		Triangles.Add(static_cast<int32>(Triangle[2]));
		Triangles.Add(static_cast<int32>(Triangle[1]));
	}
	PublishMesh(Candidate, static_cast<std::uint64_t>(Seed), Vertices, Triangles, Normals, Colors);
	AnchorDirection = FVector::ZeroVector;
	AnchorElevationMeters = 0;
	MeshMilliseconds = (FPlatformTime::Seconds() - Started) * 1000;
	// 終了
	return;
}

/**
 * 実際の高さ倍率１で地表の部分を比較する，衝突又は Streaming の受入れ面ではない
 * @param Seed 生成 Seed
 * @param Radius 惑星の物理半径
 * @param Direction 地表の局所原点の方向
 * @param HalfExtent 半表示幅，正の有限なメートル値
 * @param Cells 一辺の規則的な比較セル数，４以上１２８以下
 */
void APlanetContinentalPreview::BuildLocalPreview(
	const int64 Seed,
	const double Radius,
	const FVector Direction,
	const double HalfExtent,
	const int32 Cells
) {
	// 局所表示の入力と同じ不変場の準備
	HasSucceededLastBuild = false;
	const std::optional<PlanetCore::UnitDirection> Anchor =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	const bool IsInvalid =
	!Anchor || !std::isfinite(HalfExtent) || HalfExtent <= 0 || HalfExtent > 0.3 * Radius || HalfExtent > 100000 || Cells < 4 ||
	Cells > 128;
	if(IsInvalid) {
		LastBuildMessage = TEXT("Invalid local comparison display parameters.");
		// 無効な局所表示の終了
		return;
	}
	const std::shared_ptr<const PlanetCore::SphericalContinentalField> Candidate =
	PrepareDefinition(static_cast<std::uint64_t>(Seed), Radius);
	// 同じ高さ場を準備出来ない場合の終了
	if(!Candidate) return;
	const double Started = FPlatformTime::Seconds();
	const std::optional<double> Height = Candidate->EvaluateElevation(*Anchor);
	const std::optional<PlanetCore::GroundFrame> Frame =
	Height ? PlanetCore::GroundFrame::Create(*Anchor, Radius, *Height) : std::nullopt;
	// 物理尺度の局所座標を準備出来ない場合の終了
	if(!Frame) {
		LastBuildMessage = TEXT("The local comparison frame could not be prepared.");
		// 未解決の局所座標の終了
		return;
	}
	TArray<FVector> Vertices, Normals;
	TArray<int32> Triangles;
	TArray<FLinearColor> Colors;
	const int32 Side = Cells + 1;
	Vertices.Reserve(Side * Side);
	Normals.Reserve(Side * Side);
	Colors.Reserve(Side * Side);
	Triangles.Reserve(Cells * Cells * 6);
	for(int32 Y = 0; Y < Side; ++Y) for(int32 X = 0; X < Side; ++X) {
		const PlanetCore::UnitDirection Point =
		Frame->DirectionAt(HalfExtent * (2.0 * X / Cells - 1), HalfExtent * (2.0 * Y / Cells - 1));
		const std::optional<PlanetCore::TerrainSample> Sample = Candidate->Evaluate(Point);
		// 正規の地表点を照会出来ない場合の終了
		if(!Sample) {
			LastBuildMessage = TEXT("A valid comparison vertex could not be evaluated; previous geometry retained.");
			// 未解決の比較頂点を公開しない終了
			return;
		}
		const std::optional<PlanetCore::UnitDirection> Normal = PlanetCore::UnitDirection::FromVector(
			Point.Vector() * (Radius + Sample->ElevationMeters) - Sample->TangentialGradientPerRadian
		);
		// 解析的な法線が表現範囲を超える場合の終了
		if(!Normal) {
			LastBuildMessage = TEXT("The analytic comparison normal is not representable.");
			// 不正な法線を公開しない終了
			return;
		}
		Vertices.Add(ToUnreal(Frame->SurfaceToLocal(Point, Sample->ElevationMeters)) * 100);
		Normals.Add(ToUnreal(Frame->VectorToLocal(Normal->Vector())));
		Colors.Add(HeightColor(Sample->ElevationMeters, Radius));
	}
	for(int32 Y = 0; Y < Cells; ++Y) for(int32 X = 0; X < Cells; ++X) {
		const int32 Corner = Y * Side + X;
		Triangles.Append({ Corner, Corner + Side, Corner + 1, Corner + 1, Corner + Side, Corner + Side + 1 });
	}
	PublishMesh(Candidate, static_cast<std::uint64_t>(Seed), Vertices, Triangles, Normals, Colors);
	AnchorDirection = ToUnreal(Anchor->Vector());
	AnchorElevationMeters = *Height;
	MeshMilliseconds = (FPlatformTime::Seconds() - Started) * 1000;
	// 終了
	return;
}

/**
 * 同じ場の高さと物理的な解析法線を取得する
 * @param Direction 実惑星の正規化前の方向
 * @param ElevationMeters メートル単位の標高の出力
 * @param Normal 惑星座標の正規法線の出力
 * @return 正規入力を同じ準備場で評価出来る場合に真
 */
bool APlanetContinentalPreview::CanEvaluateDirection(const FVector Direction, double &ElevationMeters, FVector &Normal) const {
	const std::optional<PlanetCore::UnitDirection> Unit =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// まだ準備していない又は無効な方向の返戻
	if(!Field || !Unit) return false;
	const std::optional<PlanetCore::TerrainSample> Sample = Field->Evaluate(*Unit);
	// 未解決の高さを公開しない返戻
	if(!Sample) return false;
	const std::optional<PlanetCore::UnitDirection> UnitNormal = PlanetCore::UnitDirection::FromVector(
		Unit->Vector() * (PreparedRadiusMeters + Sample->ElevationMeters) - Sample->TangentialGradientPerRadian
	);
	// 表現出来ない法線の返戻
	if(!UnitNormal) return false;
	ElevationMeters = Sample->ElevationMeters;
	Normal = ToUnreal(UnitNormal->Vector());
	// 同じ不変場の照会成功の返戻
	return true;
}

/**
 * 局所比較表示の原点方向を取得する
 * @return 実惑星方向，全球表示又は未準備なら零
 */
FVector APlanetContinentalPreview::GetAnchorDirection() const {
	return AnchorDirection;
}
