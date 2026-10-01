#include "PlanetSphericalSurfacePreview.h"
#include "Core/PlanetCriticalPoint.h"
#include "Core/PlanetDescentPath.h"
#include "Core/PlanetDrainage.h"
#include "Core/PlanetFlowChannel.h"
#include "Core/PlanetFlowChannelField.h"
#include "Core/PlanetFlowCurve.h"
#include "Core/PlanetFlowNetworkField.h"
#include "Core/PlanetFlowOutlet.h"
#include "Core/PlanetFlowPreparation.h"
#include "Core/PlanetFlowRoute.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSphericalDrainage.h"
#include "Core/PlanetSphericalSpline.h"
#include "Core/PlanetSphericalWarp.h"
#include "Core/PlanetSpillLake.h"
#include "Core/PlanetSpillNetwork.h"
#include "Core/PlanetSurfaceMaterial.h"
#include "Core/PlanetTerrainContour.h"
#include "Core/PlanetWaterCoverage.h"
#include "PlanetFlowTerrain.h"
#include "PlanetGroundPatch.h"
#include "GeomTools.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

/** 視点に依存しない一件の河道・湖岸準備と，其の完全な入力識別 */
struct FPlanetFlowPreparation {
	// 同じ元場・乱数・流出口・受入水域の識別
	std::shared_ptr<const PlanetCore::PlanetSphericalDrainage> Source;
	std::shared_ptr<const PlanetCore::SpillLakeShore> ReceivingLake;
	FVector Pass = FVector::ZeroVector;
	std::uint64_t Seed = 0;
	double RadiusMeters = 0;
	double DownstreamWaterMeters = 0;
	bool IsPositiveBranch = false;
	// 所有する不変場と表示位置を選ぶ曲線の対応
	std::shared_ptr<const PlanetCore::FlowChannelField> Field;
	std::shared_ptr<const PlanetCore::TerrainFlowRoute> Route;
	double PassDistanceMeters = 0;
	double UpstreamWaterMeters = 0;
	std::uint32_t PoolCount = 0;
};

/**
 * 同じ河床場を，描画・物理と独立した生成ワーカーへ共有する
 * @return 現在の表示と同じ不変場；未準備の場合は空
 */
std::shared_ptr<const PlanetCore::FlowChannelField> APlanetSphericalSurfacePreview::GetFlowField() const {
	// アクターの寿命と独立に保持出来る元場の返戻
	return FlowPreview;
}

/**
 * 単独河道又は準備済の全水系を，同じ描画・物理へ渡す
 * @return 現在の診断定義を所有する場，未準備なら空
 */
std::shared_ptr<const PlanetCore::FlowSurfaceField> APlanetSphericalSurfacePreview::GetFlowSurfaceField() const {
	return FlowNetworkPreview ? FlowNetworkPreview : FlowPreview;
}

/**
 * 確認済の有限網の全河道を，同じ元場の単一照会へ準備する
 * @return 全流出口を準備して照会を切替えた場合に真
 */
bool APlanetSphericalSurfacePreview::PrepareConnectedFlowNetwork() {
	if(!FlowPreview || !Surface || !ConnectedNetwork || ConnectedNetworkSource != Surface || Warp) {
		LastBuildMessage = TEXT("Prepare the connected source watershed before its complete flow surface.");
		// 元場を失った網を使わない返戻
		return false;
	}
	// 同じ準備済の定義を視点更新で再生成しない返戻
	if(FlowNetworkPreview) return true;
	// 元場を値で所有し，エディタのアクター解放後も生成ワーカーが使用可能
	const auto Source = [Owned = Surface](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		return Owned->EvaluateOriginal(D);
	};
	const auto Height = [Owned = Surface](const PlanetCore::UnitDirection D) -> double {
		return Owned->EvaluateOriginalElevation(D);
	};
	PlanetCore::FlowNetworkSettings Settings;
	Settings.Channel.Outlet.RadiusMeters = PreparedRadiusMeters;
	Settings.Channel.Seed = PreparedSeed;
	PlanetCore::FlowNetworkDiagnostics Report;
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetCore::PreparedFlowNetwork> Network =
	PlanetCore::PrepareFlowNetwork(*ConnectedNetwork, Source, Height, Settings, {}, &Report);
	if(!Network) {
		LastBuildMessage = FString::Printf(
			TEXT("Complete flow network rejected: reason=%u basin=%u prepared=%u channel=%u."),
			static_cast<unsigned>(Report.Failure),
			Report.Basin,
			Report.PreparedChannels,
			static_cast<unsigned>(Report.Channel.Failure)
		);
		// 部分的な水系で既存の不変場を交換しない返戻
		return false;
	}
	std::optional<PlanetCore::FlowNetworkField> Field = PlanetCore::FlowNetworkField::Create(*Network);
	if(!Field) {
		LastBuildMessage = TEXT("Complete flow network field could not retain a consistent source and channel set.");
		// 合成定義を作れない場合の返戻
		return false;
	}
	FlowNetworkPreview = std::make_shared<const PlanetCore::FlowNetworkField>(std::move(*Field));
	PreparationMilliseconds = (FPlatformTime::Seconds() - Started) * 1000;
	// 単独河道の局所比較格子を合成地形へ重ねず，共有描画アクターへ渡す
	SurfaceMesh->SetVisibility(false);
	WaterMesh->SetVisibility(false);
	LastBuildMessage = FString::Printf(
		TEXT("Complete flow field prepared: channels=%u coastal=%u preparationMs=%.6f."),
		Report.PreparedChannels,
		Report.CoastalChannels,
		PreparationMilliseconds
	);
	// 元場と全河道を所有する定義への切替成功の返戻
	return true;
}

/**
 * 同じ座標写像を標高と解析法線の両方へ適用する
 * @param Field 元の連続地形
 * @param Mapping 可逆な球面写像，空なら元の方向を使う
 * @param Direction 表示する実惑星の方向
 * @param ShouldShowSource 補間前の元地形を直接比較する場合に真
 * @return 写像の連鎖微分を含む地形標本
 */
PlanetCore::TerrainSample APlanetSphericalSurfacePreview::SampleField(
	const PlanetCore::PlanetSphericalDrainage &Field,
	const PlanetCore::PlanetSphericalWarp *const Mapping,
	const PlanetCore::UnitDirection Direction,
	const bool ShouldShowSource
) {
	// 比較元を変更しない返戻
	if(!Mapping) return ShouldShowSource ? Field.EvaluateOriginal(Direction) : Field.Evaluate(Direction);
	const PlanetCore::SphericalWarpMapping Mapped = Mapping->MapWithDerivatives(Direction);
	PlanetCore::TerrainSample Sample =
	ShouldShowSource ? Field.EvaluateOriginal(Mapped.Direction) : Field.Evaluate(Mapped.Direction);
	Sample.TangentialGradientPerRadian = Mapped.PullbackGradient(Sample.TangentialGradientPerRadian);
	// 表示位置に対応する値と法線の返戻
	return Sample;
}

/**
 * 海域・河口・排水で共有する比較用の C1 地形を準備する
 * @param Settings 有効な惑星定義
 * @param ShouldRefineRelief 既存排水辺を保持する内部細分化を追加する場合に真
 * @param ShouldPreserveSlopes 元勾配を共有辺の単調性に従って制限し保持する場合に真
 * @param ShouldConstrainDrainage 単調性制約を実際の下流辺だけに適用する場合に真
 * @param ShouldSplitEdges 両側の面を同時に分け，辺の途中の起伏も保持する場合に真
 * @param ShouldRelaxHeights 元河道と開削点を保護して不要な盛土を下げる場合に真
 * @param ShouldAdaptBeforeDrainage 流路を固定する前に元起伏を標本化する場合に真
 * @param ShouldEstimateSlopes 適応網の値から接線を推定する場合に真
 * @param ReliefBudget 追加節点の上限
 * @return 準備に成功した不変場，失敗時は空
 */
std::shared_ptr<const PlanetCore::PlanetSphericalDrainage> APlanetSphericalSurfacePreview::PrepareField(
	const PlanetCore::GeologySettings &Settings,
	const bool ShouldRefineRelief,
	const bool ShouldPreserveSlopes,
	const bool ShouldConstrainDrainage,
	const bool ShouldSplitEdges,
	const bool ShouldRelaxHeights,
	const bool ShouldAdaptBeforeDrainage,
	const bool ShouldEstimateSlopes,
	const int32 ReliefBudget
) {
	std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Settings);
	// 地質を準備出来ない場合の返戻
	if(!PreparedGeology) return {};
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*PreparedGeology));
	std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山を準備出来ない場合の返戻
	if(!PreparedRidges) return {};
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*PreparedRidges));
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		Ridges,
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 共有網の海域を準備出来ない場合の返戻
	if(!PreparedOcean) return {};
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	PlanetCore::SphericalReliefSettings Refinement;
	Refinement.RadiusMeters = Settings.RadiusMeters;
	Refinement.SeaLevelMeters = Settings.SeaLevelMeters;
	Refinement.MinimumSlope = 1E-7 * std::min(1.0, 1E6 / Settings.RadiusMeters);
	Refinement.TargetErrorMeters *= Settings.RadiusMeters / (Settings.RadiusMeters + 250000);
	Refinement.ShouldPreserveVertexSlopes = ShouldPreserveSlopes;
	Refinement.ShouldLimitToDrainageEdges = ShouldConstrainDrainage;
	Refinement.ShouldRefineEdges = ShouldSplitEdges;
	Refinement.ShouldRelaxAddedHeights = ShouldRelaxHeights;
	Refinement.MaximumAddedVertices = static_cast<std::uint32_t>(ReliefBudget);
	if(!ShouldRefineRelief) Refinement.MaximumAddedVertices = 0;
	if(ShouldAdaptBeforeDrainage) {
		const PlanetCore::SphericalGradientMode Mode =
		ShouldEstimateSlopes ? PlanetCore::SphericalGradientMode::Sampled : PlanetCore::SphericalGradientMode::Analytic;
		std::optional<PlanetCore::PlanetSphericalDrainage> Adaptive =
		PlanetCore::PlanetSphericalDrainage::CreateWithAdaptiveDrainage(Ocean, Refinement, Mode);
		// 適応網からの面を準備出来ない場合の返戻
		if(!Adaptive) return {};
		// 元起伏の標本化を先に行った不変場の返戻
		return std::make_shared<const PlanetCore::PlanetSphericalDrainage>(std::move(*Adaptive));
	}
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 同じ元網を準備出来ない場合の返戻
	if(!Mesh) return {};
	std::vector<double> Heights;
	Heights.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) Heights.push_back(Ocean->EvaluateElevation(Direction));
	const std::optional<PlanetCore::SphericalSplineField> Source = PlanetCore::SphericalSplineField::Create(6, Heights);
	// 排水前の補間面を準備出来ない場合の返戻
	if(!Source) return {};
	const std::optional<PlanetCore::DrainageNetwork> Network =
	PlanetCore::DrainageNetwork::CreateForSphericalSurface(*Ocean, *Source);
	// 同じ海岸へ流れる排水を準備出来ない場合の返戻
	if(!Network) return {};
	std::optional<PlanetCore::DrainageNetwork> Balanced = Network->WithBalancedTerrainProfiles(
		[&Source](const PlanetCore::UnitDirection Direction) -> double {
			// 海岸と同じ標高を縦断へ渡す返戻
			return Source->Evaluate(Direction).ElevationMeters;
		}
	);
	// 共有縦断を準備出来ない場合の返戻
	if(!Balanced) return {};
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*Balanced));
	std::optional<PlanetCore::PlanetSphericalDrainage> Prepared = ShouldRefineRelief || ShouldPreserveSlopes ?
	PlanetCore::PlanetSphericalDrainage::CreateWithRefinedRelief(Ocean, Drainage, Refinement) :
	PlanetCore::PlanetSphericalDrainage::Create(Ocean, Drainage);
	// 有効な合成面を準備出来ない場合の返戻
	if(!Prepared) return {};
	// 同じ生成定義で再利用する比較場の返戻
	return std::make_shared<const PlanetCore::PlanetSphericalDrainage>(std::move(*Prepared));
}

/**
 * 公開済の診断場の標高と解析勾配を照会する
 * @param Direction 惑星中心からの方向
 * @param ElevationMeters 評価した標高
 * @param GradientPerRadian 球面接線の解析勾配
 * @return 準備済の場と有効な方向が有る場合に真
 */
bool APlanetSphericalSurfacePreview::EvaluateDirection(
	const FVector Direction,
	double &ElevationMeters,
	FVector &GradientPerRadian
) const {
	// 未準備の診断への返戻
	if(!Surface) return false;
	const std::optional<PlanetCore::UnitDirection> Unit =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 無効な方向への返戻
	if(!Unit) return false;
	const std::optional<PlanetCore::FlowChannelSection> Channel = FlowPreview ? FlowPreview->Evaluate(*Unit, false) : std::nullopt;
	// 描画中の河道の照会失敗を元場で置換えない返戻
	if(FlowPreview && !Channel) return false;
	const PlanetCore::TerrainSample Sample = Channel ? Channel->Terrain : SampleField(*Surface, Warp.get(), *Unit, IsShowingSource);
	ElevationMeters = Sample.ElevationMeters;
	const PlanetCore::Vec3 Gradient = Sample.TangentialGradientPerRadian;
	GradientPerRadian = FVector(Gradient.X, Gradient.Y, Gradient.Z);
	// 有効な場の照会結果の返戻
	return true;
}

/**
 * 変更しない診断地形へ，閉じた低地候補の球面水面を一時表示する
 * 計算量は等高線追跡と UE の多角形分割に依り，岸は最大２０４８点に制限する
 * @param Initial 湖岸上の初期方向
 * @param WaterMeters 一定の診断水位；大洋への流出を保証する水位ではない
 * @return 元場上の閉曲線・半球内の正面積・分割・球面誤差を確認して表示した場合に真
 */
bool APlanetSphericalSurfacePreview::BuildContourWaterPreview(const FVector Initial, const double WaterMeters) {
	// 地形と水面が別の場を参照する事を防ぐ入口
	const std::optional<PlanetCore::UnitDirection> Start =
	PlanetCore::UnitDirection::FromVector({ Initial.X, Initial.Y, Initial.Z });
	const std::optional<PlanetCore::UnitDirection> Anchor =
	PlanetCore::UnitDirection::FromVector({ PreparedAnchorDirection.X, PreparedAnchorDirection.Y, PreparedAnchorDirection.Z });
	const bool HasValidWaterLevel = std::isfinite(WaterMeters) && WaterMeters + PreparedRadiusMeters > 0;
	// 未準備又は不正な診断への返戻
	if(!Surface || !IsShowingSource || !Start || !Anchor || !HasValidWaterLevel) return false;
	// 同じ元場の連続した岸の準備
	PlanetCore::TerrainContourSettings Settings;
	Settings.RadiusMeters = PreparedRadiusMeters;
	Settings.LevelMeters = WaterMeters;
	Settings.MaximumLengthMeters = 5000;
	Settings.MaximumVertices = 2048;
	const std::optional<PlanetCore::TerrainContour> Contour = PlanetCore::TraceTerrainContour(
		*Start,
		[this](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			return SampleField(*Surface, Warp.get(), Direction, true);
		},
		Settings
	);
	// 部分経路を湖へ閉じない返戻
	if(!Contour || Contour->End != PlanetCore::TerrainContourEnd::Closed) return false;
	const std::optional<double> Area = PlanetCore::MeasureTerrainContourArea(*Contour, PreparedRadiusMeters);
	// 高地を囲む岸又は大き過ぎる診断範囲への返戻
	if(!Area || !(*Area > 0) || *Area > 1000000) return false;
	// 地形表示と同じ原点及び模様の位相
	const std::optional<PlanetCore::GroundFrame> Frame =
	PlanetCore::GroundFrame::Create(*Anchor, PreparedRadiusMeters, AnchorElevationMeters);
	const std::optional<PlanetCore::SurfaceTextureFrame> Texture =
	PlanetCore::SurfaceTextureFrame::Create(*Anchor, PreparedRadiusMeters, AnchorElevationMeters);
	UMaterialInterface *const Material =
	LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Planet/Materials/M_PlanetOcean.M_PlanetOcean"));
	// 表示に必要な原点又は既存材質が無い場合の返戻
	if(!Frame || !Texture || !Material) return false;
	// 心射図法で球面の岸を投影し，標準の凹多角形分割へ渡す
	TArray<FVector2D> Polygon, Triangles;
	std::vector<std::array<double, 2>> Projected;
	for(std::uint32_t i = 0; i + 1 < Contour->Samples.size(); ++i) {
		const PlanetCore::Vec3 D = Contour->Samples[i].Direction.Vector();
		// 反対側の半球を同じ平面へ折返す射影を拒否する返戻
		if(!(D.Dot(Frame->Up) > 0)) return false;
		const double Scale = PreparedRadiusMeters / D.Dot(Frame->Up);
		const FVector2D Point(D.Dot(Frame->East) * Scale, D.Dot(Frame->North) * Scale);
		// 既に描いた地形の外側へ水面を公開しない返戻
		if(Point.ContainsNaN() || FMath::Max(FMath::Abs(Point.X), FMath::Abs(Point.Y)) > PreparedHalfExtentMeters) return false;
		Polygon.Add(Point);
		Projected.push_back({ Point.X, Point.Y });
	}
	// 自交差又は標準分割の失敗時の返戻
	if(!PlanetCore::IsSimplePlanarContour(Projected) || !FGeomTools2D::TriangulatePoly(Triangles, Polygon, true)) return false;
	// 多角形の内部を欠落させた分割への返戻
	if(Triangles.Num() != 3 * (Polygon.Num() - 2)) return false;
	// 基準水位の球面頂点と，既存の水材質が要求する法線・接線
	TArray<FVector> Vertices, Normals;
	TArray<FVector2D> UV2, UV3;
	TArray<FProcMeshTangent> Tangents;
	TArray<int32> Indices;
	double MaximumSphereError = 0;
	for(int32 i = 0; i < Triangles.Num(); i += 3) {
		// 球面上の元の３方向
		const std::array<PlanetCore::UnitDirection, 3> Directions {
			Frame->DirectionAt(Triangles[i].X, Triangles[i].Y),
			Frame->DirectionAt(Triangles[i + 1].X, Triangles[i + 1].Y),
			Frame->DirectionAt(Triangles[i + 2].X, Triangles[i + 2].Y)
		};
		double MaximumChordSquared = 0;
		for(unsigned j = 0; j < 3; ++j) {
			const PlanetCore::Vec3 Delta = Directions[j].Vector() - Directions[(j + 1) % 3].Vector();
			MaximumChordSquared = std::max(MaximumChordSquared, Delta.Dot(Delta));
			const PlanetCore::Vec3 Position = Frame->SurfaceToLocal(Directions[j], WaterMeters);
			const PlanetCore::Vec3 N = Directions[j].Vector(), LocalNormal = Frame->VectorToLocal(N);
			const PlanetCore::Vec3 LocalTangent = Frame->VectorToLocal(PlanetCore::SurfaceMaterialTangent(Directions[j]));
			Vertices.Emplace(Position.X * 100, Position.Y * 100, Position.Z * 100);
			Normals.Emplace(LocalNormal.X, LocalNormal.Y, LocalNormal.Z);
			UV2.Emplace(N.X, N.Y);
			UV3.Emplace(N.Z, 0);
			Tangents.Emplace(FVector(LocalTangent.X, LocalTangent.Y, LocalTangent.Z), false);
		}
		// 三角形全体の半径損失の保守的上限
		const double Variance = MaximumChordSquared / 3;
		MaximumSphereError =
		std::max(MaximumSphereError, (PreparedRadiusMeters + WaterMeters) * Variance / (1 + std::sqrt(1 - Variance)));
		Indices.Append({ i, i + 2, i + 1 });
	}
	// 2 cm を超える球面近似を表示しない返戻
	if(MaximumSphereError > 0.02) return false;
	// 所有する一時材質への位相と局所基底の設定
	UMaterialInstanceDynamic *const Instance = WaterMesh->CreateDynamicMaterialInstance(0, Material);
	// 材質を準備出来ない場合の返戻
	if(!Instance) return false;
	const auto Parameter = [&](const FName Name, const PlanetCore::Vec3 Value) -> void {
		Instance->SetVectorParameterValue(
			Name,
			FLinearColor(static_cast<float>(Value.X), static_cast<float>(Value.Y), static_cast<float>(Value.Z), 0)
		);
		// 終了
		return;
	};
	Parameter(TEXT("TextureOrigin"), Texture->Phase);
	Parameter(TEXT("TextureAxisX"), Frame->East);
	Parameter(TEXT("TextureAxisY"), Frame->North);
	Parameter(TEXT("TextureAxisZ"), Frame->Up);
	WaterMesh->CreateMeshSection_LinearColor(0, Vertices, Indices, Normals, {}, {}, UV2, UV3, {}, Tangents, false);
	DiagnosticWaterVertices = Vertices.Num();
	DiagnosticWaterColumns = 0;
	DiagnosticShoreVertices = static_cast<int32>(Contour->Samples.size());
	DiagnosticWaterAreaMetersSquared = *Area;
	DiagnosticWaterSphereErrorMeters = MaximumSphereError;
	DiagnosticShoreHeightResidualMeters = Contour->MaximumHeightResidualMeters;
	LastBuildMessage = TEXT("Source contour water diagnostic. Outlet flow and all interior boundary components remain unverified.");
	// 元地形を変更しない水面の表示結果の返戻
	return true;
}

/**
 * 元場の鞍点からの下降曲線を，表示と同じ局所座標の診断点列へ変換する
 * @param Pass 精査する鞍点付近の球面方向
 * @param IsPositiveBranch 最小曲率方向の正側を追う場合に真
 * @param StopElevationMeters 追跡を止める元地形の標高
 * @return 元地表上の Actor 局所座標；失敗・範囲外・点数超過は空，実水面ではない
 */
TArray<FVector> APlanetSphericalSurfacePreview::TraceFlowRoute(
	const FVector Pass,
	const bool IsPositiveBranch,
	const double StopElevationMeters
) const {
	const std::optional<PlanetCore::UnitDirection> Candidate = PlanetCore::UnitDirection::FromVector({ Pass.X, Pass.Y, Pass.Z });
	const std::optional<PlanetCore::UnitDirection> Anchor =
	PlanetCore::UnitDirection::FromVector({ PreparedAnchorDirection.X, PreparedAnchorDirection.Y, PreparedAnchorDirection.Z });
	// 対応する元場又は有効な方向が無い場合の返戻
	if(!Surface || !IsShowingSource || !Candidate || !Anchor || !std::isfinite(StopElevationMeters)) return {};
	const auto Field = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
		// 表示中と同じ不変な元場の返戻
		return SampleField(*Surface, Warp.get(), Direction, true);
	};
	PlanetCore::TerrainCriticalSettings CriticalSettings;
	CriticalSettings.RadiusMeters = PreparedRadiusMeters;
	CriticalSettings.MaximumDisplacementMeters = std::min(CriticalSettings.MaximumDisplacementMeters, PreparedRadiusMeters * 0.125);
	const std::optional<PlanetCore::TerrainCriticalPoint> Critical =
	PlanetCore::RefineTerrainCriticalPoint(*Candidate, Field, CriticalSettings);
	const std::optional<PlanetCore::GroundFrame> Frame =
	PlanetCore::GroundFrame::Create(*Anchor, PreparedRadiusMeters, AnchorElevationMeters);
	// 鞍点でない停留点又は無効な原点への返戻
	if(!Critical || Critical->Kind != PlanetCore::TerrainCriticalKind::Saddle || !Frame) return {};
	const PlanetCore::Vec3 Along = Critical->MinimumCurvatureDirection * (IsPositiveBranch ? 1 : -1);
	const PlanetCore::UnitDirection Start =
	*PlanetCore::UnitDirection::FromVector(Critical->Direction.Vector() + Along * (5 / PreparedRadiusMeters));
	PlanetCore::TerrainDescentSettings Settings;
	Settings.RadiusMeters = PreparedRadiusMeters;
	Settings.PositionToleranceMeters = 1E-5;
	Settings.MaximumLengthMeters = std::min(Settings.MaximumLengthMeters, PreparedRadiusMeters * 0.5);
	Settings.StopElevationMeters = StopElevationMeters;
	const std::optional<PlanetCore::TerrainDescentPath> Path = PlanetCore::TraceTerrainDescent(Start, Field, Settings);
	// 空の追跡結果を公開しない返戻
	if(!Path || Path->Samples.empty()) return {};
	TArray<FVector> Result;
	PlanetCore::UnitDirection Previous = Critical->Direction;
	PlanetCore::Vec3 PreviousTangent = Along;
	for(const PlanetCore::TerrainDescentSample &Next : Path->Samples) {
		const PlanetCore::Vec3 NextTangent = Field(Next.Direction).TangentialGradientPerRadian * -1;
		const std::optional<PlanetCore::TerrainFlowSegment> Curve =
		PlanetCore::TerrainFlowSegment::Create(Previous, Next.Direction, PreviousTangent, NextTangent, PreparedRadiusMeters);
		// 退化した接続を直線で隠さない返戻
		if(!Curve) return {};
		const std::uint32_t Steps = static_cast<std::uint32_t>(std::ceil(Curve->GetParameterLengthMeters() / 5));
		for(std::uint32_t i = Result.IsEmpty() ? 0 : 1; i <= Steps; ++i) {
			const PlanetCore::TerrainFlowCurveSample Point = *Curve->Evaluate(static_cast<double>(i) / Steps);
			const PlanetCore::TerrainSample Terrain = Field(Point.Direction);
			const double Slope = Terrain.TangentialGradientPerRadian.Dot(Point.DirectionDerivativePerMeter);
			const PlanetCore::Vec3 D = Point.Direction.Vector();
			// 反対側の半球を近景の範囲内と誤認しない返戻
			if(!(D.Dot(Frame->Up) > 0)) return {};
			const double Scale = PreparedRadiusMeters / D.Dot(Frame->Up);
			const bool IsOutside = std::max(std::abs(D.Dot(Frame->East)), std::abs(D.Dot(Frame->North))) * Scale > PreparedHalfExtentMeters;
			// 描画済の地形の外，元場を上る区間又は点数予算超過への返戻
			if(IsOutside || Slope > 1E-10 || Result.Num() > 8191) return {};
			const PlanetCore::Vec3 Local = Frame->SurfaceToLocal(Point.Direction, Terrain.ElevationMeters);
			Result.Emplace(Local.X * 100, Local.Y * 100, Local.Z * 100);
		}
		Previous = Next.Direction;
		PreviousTangent = NextTangent;
	}
	// 実地形上の位置だけを返し，水位又は河床と読み替えない診断点列の返戻
	return Result;
}

/**
 * 元の谷筋の一部を，有限幅の河床と連続水位で近景表示する
 * @param Pass 追跡する鞍点付近の方向，最小曲率の正側を使う
 * @param DistanceMeters 鞍点を零とする観察距離，湖側は負
 * @param DownstreamWaterMeters 比較用の下流静水位
 * @param HalfExtentMeters 表示する局所範囲の半幅
 * @param Cells 基準の分割数，１６〜１２８へ制限した後に生成時誤差１センチメートルで局所細分化
 * @param UseHeightOnlyQueries 標高専用照会を使う場合に真；偽は同じ形状の全微分との計測比較用
 * @param IsPositiveBranch 鞍点の正側を下流にする場合に真
 * @param ReceivingLake 同じ下流水位で確認した受入湖の岸，旧比較経路では空
 * @return 河床と水面を公開した場合に真；本番の水系完成を意味しない
 */
bool APlanetSphericalSurfacePreview::BuildFlowChannel(
	const FVector Pass,
	const double DistanceMeters,
	const double DownstreamWaterMeters,
	const double HalfExtentMeters,
	const int32 Cells,
	const bool UseHeightOnlyQueries,
	const bool IsPositiveBranch,
	const std::shared_ptr<const PlanetCore::SpillLakeShore> &ReceivingLake
) {
	const std::optional<PlanetCore::UnitDirection> Candidate = PlanetCore::UnitDirection::FromVector({ Pass.X, Pass.Y, Pass.Z });
	const bool HasValidWindow =
	std::isfinite(HalfExtentMeters) && HalfExtentMeters > 0 && HalfExtentMeters < PreparedRadiusMeters * 0.05 && Cells > 15 &&
	Cells < 257;
	if(
		!Surface || Warp || !Candidate || !IsShowingSource && !FlowPreview || !HasValidWindow || !std::isfinite(DownstreamWaterMeters)
	) {
		LastBuildMessage = TEXT("A source field, a valid pass and a bounded local grid are required.");
		// 無効な診断入力への返戻
		return false;
	}
	const double Started = FPlatformTime::Seconds();
	const PlanetCore::Vec3 CandidateVector = Candidate->Vector();
	const FVector NormalizedPass(CandidateVector.X, CandidateVector.Y, CandidateVector.Z);
	const bool ReusesFlow =
	FlowPreparation && FlowPreparation->Source == Surface && FlowPreparation->ReceivingLake == ReceivingLake &&
	FlowPreparation->Pass == NormalizedPass && FlowPreparation->Seed == PreparedSeed &&
	FlowPreparation->RadiusMeters == PreparedRadiusMeters && FlowPreparation->DownstreamWaterMeters == DownstreamWaterMeters &&
	FlowPreparation->IsPositiveBranch == IsPositiveBranch;
	std::shared_ptr<const FPlanetFlowPreparation> Preparation = FlowPreparation;
	// 視点・格子・微分照会方式を含めない，生成定義だけに基づく一件の再利用
	if(!ReusesFlow) {
		PlanetCore::FlowPreparationSettings Settings;
		Settings.Seed = PreparedSeed;
		Settings.Outlet.RadiusMeters = PreparedRadiusMeters;
		Settings.Outlet.DownstreamWaterMeters = DownstreamWaterMeters;
		Settings.Outlet.IsPositiveBranch = IsPositiveBranch;
		const auto OwnedSource = [Field = Surface](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			// 非同期の利用期間も所有する元場の返戻
			return Field->EvaluateOriginal(Direction);
		};
		const auto OwnedHeight = [Field = Surface](const PlanetCore::UnitDirection Direction) -> double {
			// 同じ元場の標高専用評価の返戻
			return Field->EvaluateOriginalElevation(Direction);
		};
		PlanetCore::FlowPreparationDiagnostics Diagnostics;
		const std::optional<PlanetCore::PreparedFlowChannel> Channel =
		PlanetCore::PrepareFlowChannel(*Candidate, OwnedSource, OwnedHeight, Settings, ReceivingLake, {}, &Diagnostics);
		if(!Channel) {
			LastBuildMessage = FString::Printf(
				TEXT("Channel preparation rejected: stage=%u outlet=%u shore=%u."),
				static_cast<unsigned>(Diagnostics.Failure),
				static_cast<unsigned>(Diagnostics.Outlet.Failure),
				static_cast<unsigned>(Diagnostics.Shore.Failure)
			);
			// 不完全な準備を公開しない返戻
			return false;
		}
		const std::shared_ptr<FPlanetFlowPreparation> Ready = std::make_shared<FPlanetFlowPreparation>();
		Ready->Source = Surface;
		Ready->ReceivingLake = ReceivingLake;
		Ready->Pass = NormalizedPass;
		Ready->Seed = PreparedSeed;
		Ready->RadiusMeters = PreparedRadiusMeters;
		Ready->DownstreamWaterMeters = DownstreamWaterMeters;
		Ready->IsPositiveBranch = IsPositiveBranch;
		Ready->Field = Channel->Field;
		Ready->Route = Channel->Route;
		Ready->PassDistanceMeters = Channel->PassDistanceMeters;
		Ready->UpstreamWaterMeters = Channel->UpstreamWaterMeters;
		Ready->PoolCount = Channel->PoolCount;
		Preparation = std::move(Ready);
	}
	const std::optional<PlanetCore::TerrainFlowCurveSample> Center =
	Preparation->Route->Sample(Preparation->PassDistanceMeters + DistanceMeters);
	if(!Center) {
		LastBuildMessage = TEXT("The requested distance from the source saddle is outside the prepared route.");
		// 鞍点を零とする観察距離が範囲外の場合の返戻
		return false;
	}
	const std::shared_ptr<const PlanetCore::FlowChannelField> &Prepared = Preparation->Field;
	const std::optional<PlanetCore::FlowChannelSection> AnchorSample = Prepared->Evaluate(Center->Direction);
	if(!AnchorSample) {
		LastBuildMessage = TEXT("The water profile is inconsistent at the requested route center.");
		// 中心の水位・河床が不整合な場合の返戻
		return false;
	}
	const double CenterHeight = AnchorSample->Terrain.ElevationMeters;
	const std::optional<PlanetCore::GroundFrame> Frame =
	PlanetCore::GroundFrame::Create(Center->Direction, PreparedRadiusMeters, CenterHeight);
	const std::optional<PlanetCore::SurfaceTextureFrame> Texture =
	PlanetCore::SurfaceTextureFrame::Create(Center->Direction, PreparedRadiusMeters, CenterHeight);
	UMaterialInterface *const Material =
	LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Planet/Materials/M_PlanetOcean.M_PlanetOcean"));
	if(!Frame || !Texture || !Material) {
		LastBuildMessage = TEXT("The local frame or existing water material is unavailable.");
		// 有効な局所描画を構成出来ない場合の返戻
		return false;
	}
	const double PreparedAt = FPlatformTime::Seconds();
	std::uint64_t GroundQueries = 0, WaterQueries = 0;
	PlanetCore::GroundPatchSettings GroundSettings;
	GroundSettings.HalfExtentMeters = HalfExtentMeters;
	// 独立な内部点の２センチメートル検査に余裕を持たせ，生成側では１センチメートルを要求する
	GroundSettings.MaximumProbeErrorMeters = 0.01;
	GroundSettings.CellsPerSide = static_cast<std::uint32_t>(std::min(Cells, 128));
	const auto Terrain = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
		++GroundQueries;
		const std::optional<PlanetCore::FlowChannelSection> Section = Prepared->Evaluate(Direction, false);
		// 不正な縦断を元地形へ戻さず細分化の失敗へ伝える返戻
		return Section ? Section->Terrain : PlanetCore::TerrainSample{ std::numeric_limits<double>::quiet_NaN(), {} };
	};
	const auto TerrainHeight = [&](const PlanetCore::UnitDirection Direction) -> double {
		++GroundQueries;
		const std::optional<PlanetCore::FlowChannelSection> Section = Prepared->Evaluate(Direction, false, !UseHeightOnlyQueries);
		// 法線を使わない内部誤差検査への標高の返戻
		return Section ? Section->Terrain.ElevationMeters : std::numeric_limits<double>::quiet_NaN();
	};
	PlanetCore::GroundPatchDiagnostics GroundDiagnostics;
	const std::optional<PlanetCore::GroundPatch> Ground = PlanetCore::BuildGroundPatch(
		Center->Direction,
		PreparedRadiusMeters,
		GroundSettings,
		Terrain,
		{},
		TerrainHeight,
		&GroundDiagnostics
	);
	const double GroundAt = FPlatformTime::Seconds();
	if(!Ground) {
		LastBuildMessage = FString::Printf(
			TEXT("Local channel rejected: reason=%u pass=%u vertices=%llu triangles=%llu lastError=%.12g m."),
			static_cast<unsigned>(GroundDiagnostics.Failure),
			GroundDiagnostics.RefinementPass,
			static_cast<unsigned long long>(GroundDiagnostics.Vertices),
			static_cast<unsigned long long>(GroundDiagnostics.Triangles),
			GroundDiagnostics.MaximumErrorMeters
		);
		for(const PlanetCore::Vec3 Point : GroundDiagnostics.WorstTriangle) {
			LastBuildMessage += FString::Printf(TEXT(" vertex=(%.17g,%.17g,%.17g)"), Point.X, Point.Y, Point.Z);
		}
		for(const double Error : GroundDiagnostics.WorstProbeErrors) LastBuildMessage += FString::Printf(TEXT(" error=%.17g"), Error);
		// 未達の地面を既存の有効な描画へ交換しない返戻
		return false;
	}
	TArray<FVector> Vertices, Normals;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> Colors;
	TArray<int32> Indices;
	const int32 VertexCount = static_cast<int32>(Ground->Positions.size());
	Vertices.Reserve(VertexCount);
	Normals.Reserve(VertexCount);
	Tangents.Reserve(VertexCount);
	Colors.Reserve(VertexCount);
	Indices.Reserve(static_cast<int32>(Ground->Triangles.size() * 3));
	for(std::size_t i = 0; i < Ground->Positions.size(); ++i) {
		const PlanetCore::Vec3 Position = Ground->Positions[i], N = Ground->Normals[i];
		const PlanetCore::UnitDirection Normal = *PlanetCore::UnitDirection::FromVector(N);
		const PlanetCore::Vec3 T = PlanetCore::SurfaceMaterialTangent(Normal);
		Vertices.Emplace(Position.X * 100, Position.Y * 100, Position.Z * 100);
		Normals.Emplace(N.X, N.Y, N.Z);
		Tangents.Emplace(FVector(T.X, T.Y, T.Z), false);
		Colors.Emplace(0.22F, 0.23F, 0.24F, 1.0F);
	}
	for(const std::array<std::uint32_t, 3> &Triangle : Ground->Triangles) {
		// Core の反時計回りを UE の表向きへ揃える
		Indices.Append({ static_cast<int32>(Triangle[0]), static_cast<int32>(Triangle[2]), static_cast<int32>(Triangle[1]) });
	}
	// 湖側の広い水面も，同じ局所範囲と誤差基準で細分化する
	// 地面の共有分割から始め，水位と所属を検査しながら岸だけを追加細分化する
	PlanetCore::SphereMesh WaterPartition;
	WaterPartition.Triangles = Ground->Triangles;
	WaterPartition.Directions.reserve(Ground->Positions.size());
	for(const PlanetCore::Vec3 Position : Ground->Positions) {
		WaterPartition.Directions.push_back(*PlanetCore::UnitDirection::FromVector(Frame->LocalToPlanet(Position)));
	}
	const auto WaterSample = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::WaterCoverageSample {
		++WaterQueries;
		const std::optional<PlanetCore::FlowChannelSection> Section = Prepared->Evaluate(Direction, true, !UseHeightOnlyQueries);
		const std::optional<double> Support = Prepared->WaterSupport(Direction);
		// 未定義を乾いた点へ置換えず，生成器の拒否へ伝える返戻
		if(!Section || !Section->HasWater || !Support) return { {}, std::numeric_limits<double>::quiet_NaN() };
		const double Height = Section->Water.ElevationMeters;
		// 水位と，水深・所属を共に満たす符号の返戻
		return { Frame->SurfaceToLocal(Direction, Height), std::min(Height - Section->Terrain.ElevationMeters, *Support) };
	};
	PlanetCore::WaterRefinementDiagnostics WaterDiagnostics;
	const std::optional<PlanetCore::AdaptiveWaterCoverage> WaterCandidate = PlanetCore::BuildAdaptiveWaterCoverage(
		std::move(WaterPartition),
		*Frame,
		Center->Direction,
		HalfExtentMeters * 1.5,
		WaterSample,
		WaterDiagnostics
	);
	const double WaterAt = FPlatformTime::Seconds();
	if(!WaterCandidate || WaterCandidate->Wet.Triangles.empty()) {
		LastBuildMessage = FString::Printf(
			TEXT("Water coverage rejected: pass=%u vertices=%llu triangles=%llu failed=%llu waterError=%.12g shoreError=%.12g"),
			WaterDiagnostics.Pass,
			static_cast<unsigned long long>(WaterDiagnostics.SourceVertices),
			static_cast<unsigned long long>(WaterDiagnostics.SourceTriangles),
			static_cast<unsigned long long>(WaterDiagnostics.FailedTriangles),
			WaterDiagnostics.WaterErrorMeters,
			WaterDiagnostics.ShoreErrorMeters
		);
		// 所属又は水位の精度を満たさない面を公開しない返戻
		return false;
	}
	const PlanetCore::WaterCoverageMesh &WaterCoverage = WaterCandidate->Wet;
	TArray<FVector> WaterVertices, WaterNormals;
	TArray<FVector2D> UV2, UV3;
	TArray<FProcMeshTangent> WaterTangents;
	TArray<int32> WaterIndices;
	WaterVertices.Reserve(static_cast<int32>(WaterCoverage.Vertices.size()));
	WaterNormals.Reserve(static_cast<int32>(WaterCoverage.Vertices.size()));
	WaterTangents.Reserve(static_cast<int32>(WaterCoverage.Vertices.size()));
	WaterIndices.Reserve(static_cast<int32>(WaterCoverage.Triangles.size() * 3));
	for(const PlanetCore::WaterCoverageVertex &Vertex : WaterCoverage.Vertices) {
		const PlanetCore::Vec3 Position = Vertex.Position;
		const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Frame->LocalToPlanet(Position));
		const std::optional<PlanetCore::FlowChannelSection> Section = Prepared->Evaluate(Direction);
		// 切出した岸の照会失敗を不正な法線で隠さない返戻
		if(!Section || !Section->HasWater) return false;
		const PlanetCore::UnitDirection WorldNormal = *PlanetCore::UnitDirection::FromVector(
			Direction.Vector() * (PreparedRadiusMeters + Section->Water.ElevationMeters) - Section->Water.TangentialGradientPerRadian
		);
		const PlanetCore::Vec3 N = Frame->VectorToLocal(WorldNormal.Vector());
		const PlanetCore::Vec3 T = Frame->VectorToLocal(PlanetCore::SurfaceMaterialTangent(WorldNormal));
		WaterVertices.Emplace(Position.X * 100, Position.Y * 100, Position.Z * 100);
		WaterNormals.Emplace(N.X, N.Y, N.Z);
		WaterTangents.Emplace(FVector(T.X, T.Y, T.Z), false);
		UV2.Emplace(WorldNormal.Vector().X, WorldNormal.Vector().Y);
		UV3.Emplace(WorldNormal.Vector().Z, 0);
	}
	for(const std::array<std::uint32_t, 3> &Triangle : WaterCoverage.Triangles) {
		// 水面も Core と UE の表裏の違いを同じ順序で変換する
		WaterIndices.Append({ static_cast<int32>(Triangle[0]), static_cast<int32>(Triangle[2]), static_cast<int32>(Triangle[1]) });
	}
	UMaterialInstanceDynamic *const Instance = WaterMesh->CreateDynamicMaterialInstance(0, Material);
	// 材質の生成失敗への返戻
	if(!Instance) return false;
	const auto Parameter = [&](const FName Name, const PlanetCore::Vec3 Value) -> void {
		Instance->SetVectorParameterValue(
			Name,
			FLinearColor(static_cast<float>(Value.X), static_cast<float>(Value.Y), static_cast<float>(Value.Z), 0)
		);
		// 終了
		return;
	};
	Parameter(TEXT("TextureOrigin"), Texture->Phase);
	Parameter(TEXT("TextureAxisX"), Frame->East);
	Parameter(TEXT("TextureAxisY"), Frame->North);
	Parameter(TEXT("TextureAxisZ"), Frame->Up);
	SurfaceMesh->CreateMeshSection_LinearColor(0, Vertices, Indices, Normals, {}, Colors, Tangents, false);
	WaterMesh->CreateMeshSection_LinearColor(
		0,
		WaterVertices,
		WaterIndices,
		WaterNormals,
		{},
		{},
		UV2,
		UV3,
		{},
		WaterTangents,
		false
	);
	FlowPreview = Prepared;
	FlowNetworkPreview.reset();
	SurfaceMesh->SetVisibility(true);
	WaterMesh->SetVisibility(true);
	FlowPreparation = Preparation;
	if(!ReusesFlow && DiagnosticFlowPreparations < MAX_int64) ++DiagnosticFlowPreparations;
	DiagnosticOutletWaterMeters = Preparation->UpstreamWaterMeters;
	DiagnosticReceivingWaterMeters = DownstreamWaterMeters;
	DiagnosticConnectedSites = 0;
	DiagnosticRouteEndMeters = Preparation->Route->GetLengthMeters() - Preparation->PassDistanceMeters;
	DiagnosticReceivingShoreVertices = ReceivingLake ? static_cast<int32>(ReceivingLake->GetContour().Samples.size()) : 0;
	PreparedAnchorDirection = FVector(Center->Direction.Vector().X, Center->Direction.Vector().Y, Center->Direction.Vector().Z);
	PreparedHalfExtentMeters = HalfExtentMeters;
	AnchorElevationMeters = CenterHeight;
	GeneratedVertices = Vertices.Num();
	DiagnosticWaterVertices = WaterVertices.Num();
	DiagnosticWaterColumns = 0;
	DiagnosticShoreVertices = 0;
	DiagnosticWaterAreaMetersSquared = DiagnosticWaterSphereErrorMeters = DiagnosticShoreHeightResidualMeters = 0;
	PreparationMilliseconds = (PreparedAt - Started) * 1000;
	MeshMilliseconds = (FPlatformTime::Seconds() - PreparedAt) * 1000;
	IsShowingSource = false;
	LastBuildMessage = FString::Printf(
		TEXT(
			"Outlet comparison: pools=%u waterPasses=%u shoreError=%.12g. Global water system and hydraulic calibration remain separate."
		),
		Preparation->PoolCount,
		WaterDiagnostics.Pass,
		WaterDiagnostics.ShoreErrorMeters
	);
	LastBuildMessage += FString::Printf(
		TEXT(" Stages ground=%.6f water=%.6f publish=%.6f ms, queries=%llu/%llu, flowReused=%d."),
		(GroundAt - PreparedAt) * 1000,
		(WaterAt - GroundAt) * 1000,
		MeshMilliseconds - (WaterAt - PreparedAt) * 1000,
		static_cast<unsigned long long>(GroundQueries),
		static_cast<unsigned long long>(WaterQueries),
		static_cast<int>(ReusesFlow)
	);
	// 同じ解析場からの河床・水面表示の返戻
	return true;
}

/**
 * 指定水位による従来の比較経路で，河床と水面を表示する
 * @param Pass 追跡する鞍点候補
 * @param DistanceMeters 鞍点からの観察距離
 * @param DownstreamWaterMeters 比較する受入水位
 * @param HalfExtentMeters 局所表示の半幅
 * @param Cells 初期格子の分割数
 * @param UseHeightOnlyQueries 標高専用の内部照会を使う場合に真
 * @param IsPositiveBranch 鞍点の正側を下流とする場合に真
 * @return 有効な局所面を公開出来た場合に真
 */
bool APlanetSphericalSurfacePreview::BuildFlowChannelPreview(
	const FVector Pass,
	const double DistanceMeters,
	const double DownstreamWaterMeters,
	const double HalfExtentMeters,
	const int32 Cells,
	const bool UseHeightOnlyQueries,
	const bool IsPositiveBranch
) {
	// 指定された受入水位に依る比較結果の返戻
	return BuildFlowChannel(
		Pass,
		DistanceMeters,
		DownstreamWaterMeters,
		HalfExtentMeters,
		Cells,
		UseHeightOnlyQueries,
		IsPositiveBranch,
		{}
	);
}

/**
 * 大洋への接続と実際の受入水位を準備し，元場の湖と流出口を表示する
 * @param InitialPass 出発する水系の鞍点候補
 * @param DistanceMeters 実流出口を零とする観察距離
 * @param HalfExtentMeters 表示する局所範囲の半幅
 * @param Cells 局所面の初期分割数
 * @return 確認済の水系と同じ水位で表示を準備出来た場合に真
 */
bool APlanetSphericalSurfacePreview::BuildConnectedFlowChannelPreview(
	const FVector InitialPass,
	const double DistanceMeters,
	const double HalfExtentMeters,
	const int32 Cells
) {
	const std::optional<PlanetCore::UnitDirection> Candidate =
	PlanetCore::UnitDirection::FromVector({ InitialPass.X, InitialPass.Y, InitialPass.Z });
	const bool ValidWindow = std::isfinite(DistanceMeters) && std::isfinite(HalfExtentMeters) && HalfExtentMeters > 0 &&
	HalfExtentMeters < PreparedRadiusMeters * 0.05 && Cells > 15 && Cells < 257;
	if(!Surface || Warp || !Candidate || !IsShowingSource && !FlowPreview || !ValidWindow) {
		LastBuildMessage = TEXT("A source field, an initial pass and a bounded observation window are required.");
		// 有効な元場と表示範囲が無い場合の返戻
		return false;
	}
	const PlanetCore::Vec3 D = Candidate->Vector();
	const FVector Normalized(D.X, D.Y, D.Z);
	const bool Reuse = ConnectedNetwork && ConnectedNetworkSource == Surface && ConnectedInitialPass == Normalized;
	const double Started = FPlatformTime::Seconds();
	if(!Reuse) {
		// 同じ排水網で大洋に属する標本だけを，元場でも水没している事を確認
		std::vector<PlanetCore::UnitDirection> Anchors;
		std::set<std::tuple<double, double, double>> Keys;
		const PlanetCore::DrainageNetwork &Coarse = Surface->GetDrainageNetwork();
		for(const PlanetCore::DrainageNode &Node : Coarse.Nodes()) {
			if(!Node.Ocean) continue;
			const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector(Node.Direction);
			if(!Direction || !(Surface->EvaluateOriginalElevation(*Direction) < Coarse.Settings().SeaLevelMeters - 1)) continue;
			const PlanetCore::Vec3 P = Direction->Vector();
			Anchors.push_back(*Direction);
			Keys.emplace(P.X, P.Y, P.Z);
		}
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			// 河床及び表示と同じ元場の返戻
			return Surface->EvaluateOriginal(Direction);
		};
		const auto IsOcean = [&](const PlanetCore::UnitDirection Direction) -> bool {
			const PlanetCore::Vec3 P = Direction.Vector();
			// 同じ準備済排水網で所属を確認した標本の返戻
			return Keys.contains({ P.X, P.Y, P.Z });
		};
		PlanetCore::SpillNetworkSettings Settings;
		Settings.RadiusMeters = PreparedRadiusMeters;
		Settings.SeaLevelMeters = Coarse.Settings().SeaLevelMeters;
		PlanetCore::SpillNetworkDiagnostics Report;
		std::optional<PlanetCore::SpillNetwork> Network =
		PlanetCore::PrepareSpillNetwork(*Candidate, Source, Anchors, IsOcean, Settings, {}, &Report);
		if(!Network) {
			LastBuildMessage = FString::Printf(
				TEXT("Connected watershed preparation failed: reason=%u step=%u evaluations=%llu."),
				static_cast<unsigned>(Report.Failure),
				Report.Steps,
				static_cast<unsigned long long>(Report.Evaluations)
			);
			// 不完全な水系を仮の下流水位で表示しない返戻
			return false;
		}
		ConnectedNetwork = std::make_shared<const PlanetCore::SpillNetwork>(std::move(*Network));
		ConnectedNetworkSource = Surface;
		ConnectedInitialPass = Normalized;
		ConnectedReceivingLake.reset();
	}
	const double NetworkMilliseconds = Reuse ? 0 : (FPlatformTime::Seconds() - Started) * 1000;
	// 初期峠の高い側の実極小が属する水域から，実際の流出口を取得
	const std::array<std::uint32_t, 2> &Sites = ConnectedNetwork->Graph.BranchSites.front();
	const PlanetCore::ContinuousSpillGraph &Graph = ConnectedNetwork->Graph;
	const std::uint32_t SourceSite =
	Graph.Sites[Sites[0]].ElevationMeters >= Graph.Sites[Sites[1]].ElevationMeters ? Sites[0] : Sites[1];
	const std::optional<PlanetCore::SpillNetworkOutlet> Outlet = PlanetCore::GetSpillNetworkOutlet(*ConnectedNetwork, SourceSite);
	if(!Outlet || !(Outlet->DownstreamWaterMeters < Outlet->UpstreamWaterMeters)) {
		LastBuildMessage = TEXT("The selected water body has no verified outlet with a positive water-level drop.");
		// 同水位の湖の連結を河川として描かない返戻
		return false;
	}
	// 同じ排水網の受入湖自身の流出口から，視点に依存しない岸を準備
	if(!ConnectedReceivingLake) {
		const std::uint32_t Site = Graph.BranchSites[Outlet->Connection][Outlet->IsPositiveDownstream ? 1 : 0];
		const std::optional<PlanetCore::SpillNetworkOutlet> Receiving = PlanetCore::GetSpillNetworkOutlet(*ConnectedNetwork, Site);
		if(!Receiving) {
			LastBuildMessage = TEXT("This lake comparison requires a receiving lake with a verified outlet; ocean rendering is separate.");
			// 受入海面を湖岸として捏造しない返戻
			return false;
		}
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			return Surface->EvaluateOriginal(Direction);
		};
		PlanetCore::TerrainContourSettings Settings;
		Settings.RadiusMeters = PreparedRadiusMeters;
		Settings.MaximumLengthMeters = std::min(1000000.0, PreparedRadiusMeters * 4);
		PlanetCore::SpillLakeDiagnostics Report;
		const PlanetCore::SpillConnection &Exit = ConnectedNetwork->Connections[Receiving->Connection];
		std::optional<PlanetCore::SpillLakeShore> Lake =
		PlanetCore::PrepareSpillLakeShore(Exit.Pass, !Receiving->IsPositiveDownstream, Source, Settings, {}, &Report);
		const PlanetCore::SpillConnectionBranch &Incoming =
		ConnectedNetwork->Connections[Outlet->Connection].Branches[Outlet->IsPositiveDownstream ? 1 : 0];
		if(!Lake || !Lake->Contains(Incoming.Path.Samples.back().Direction)) {
			LastBuildMessage = FString::Printf(
				TEXT("Receiving lake shore rejected: reason=%u end=%u vertices=%u length=%.12g."),
				static_cast<unsigned>(Report.Failure),
				static_cast<unsigned>(Report.End),
				Report.Vertices,
				Report.LengthMeters
			);
			// 閉じていない岸又は別の水域を公開しない返戻
			return false;
		}
		ConnectedReceivingLake = std::make_shared<const PlanetCore::SpillLakeShore>(std::move(*Lake));
	}
	const PlanetCore::Vec3 Pass = ConnectedNetwork->Connections[Outlet->Connection].Pass.Direction.Vector();
	const double ConnectedPreparationMilliseconds = (FPlatformTime::Seconds() - Started) * 1000;
	const bool Built = BuildFlowChannel(
		FVector(Pass.X, Pass.Y, Pass.Z),
		DistanceMeters,
		Outlet->DownstreamWaterMeters,
		HalfExtentMeters,
		Cells,
		true,
		Outlet->IsPositiveDownstream,
		ConnectedReceivingLake
	);
	// 表示の誤差又は形状予算を満たせない場合の返戻
	if(!Built) return false;
	DiagnosticConnectedSites = static_cast<int32>(Graph.Sites.size());
	PreparationMilliseconds += ConnectedPreparationMilliseconds;
	LastBuildMessage = FString::Printf(
		TEXT("Verified watershed sites=%u sourceWater=%.12g receivingWater=%.12g reused=%u networkMs=%.6f shoreMs=%.6f. "),
		DiagnosticConnectedSites,
		Outlet->UpstreamWaterMeters,
		Outlet->DownstreamWaterMeters,
		static_cast<unsigned>(Reuse),
		NetworkMilliseconds,
		ConnectedPreparationMilliseconds - NetworkMilliseconds
	) + LastBuildMessage;
	// 実際の下流水位を使った表示成功の返戻
	return true;
}

/**
 * 表示中の河床・水位・元標高を同じ方向で比較する
 * @param Direction 惑星方向
 * @param TerrainMeters 補正後の標高
 * @param WaterMeters 水位候補
 * @param Coverage 開削の重み
 * @param OriginalMeters 元の標高
 * @return 有効な河道表示と照会点が有る場合に真
 */
bool APlanetSphericalSurfacePreview::EvaluateFlowDirection(
	const FVector Direction,
	double &TerrainMeters,
	double &WaterMeters,
	double &Coverage,
	double &OriginalMeters
) const {
	const std::shared_ptr<const PlanetCore::FlowSurfaceField> ActiveField = GetFlowSurfaceField();
	const std::optional<PlanetCore::UnitDirection> D =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 有効な河道表示以外への返戻
	if(!ActiveField || !D) return false;
	const std::optional<PlanetCore::FlowChannelSection> Section = ActiveField->Evaluate(*D);
	// 不正な断面への返戻
	if(!Section) return false;
	TerrainMeters = Section->Terrain.ElevationMeters;
	WaterMeters = Section->Water.ElevationMeters;
	Coverage = Section->BedCoverage;
	OriginalMeters = Surface->EvaluateOriginal(*D).ElevationMeters;
	// 同じ場の比較値の返戻
	return true;
}

/**
 * 診断と同じ元海陸を，最初の描画要求前に季節材質へ接続する
 * @param Terrain 同じ共有場を表示するアクター
 * @return 元場と描画先が有効で，気候を構成出来た場合に真
 */
bool APlanetSphericalSurfacePreview::ConfigureFlowClimate(APlanetFlowTerrain *const Terrain) const {
	// 同じ所有元を持つ入力だけを渡す返戻
	return Surface && IsValid(Terrain) && Terrain->ConfigureClimate(GetFlowSurfaceField(), Surface->GetOriginalOcean());
}

/**
 * 表示と同じ不変場を，局所座標が一致する足元の衝突生成へ渡す
 * @param Patch 旧足場を保持しながら更新する衝突アクター
 * @return 同じ河床の衝突要求を発行出来た場合に真
 */
bool APlanetSphericalSurfacePreview::RequestFlowCollision(APlanetGroundPatch *const Patch) const {
	const std::shared_ptr<const PlanetCore::FlowSurfaceField> ActiveField = GetFlowSurfaceField();
	const std::optional<PlanetCore::UnitDirection> Anchor =
	PlanetCore::UnitDirection::FromVector({ PreparedAnchorDirection.X, PreparedAnchorDirection.Y, PreparedAnchorDirection.Z });
	// 表示していない場又は拡縮した座標を足場として使用しない返戻
	if(!ActiveField || !IsValid(Patch) || !Anchor || !GetActorScale3D().Equals(FVector::OneVector, 1E-9)) return false;
	const std::optional<PlanetCore::GroundFrame> Frame =
	PlanetCore::GroundFrame::Create(*Anchor, ActiveField->GetRadiusMeters(), AnchorElevationMeters);
	// 同じ局所座標を構成出来ない場合の返戻
	if(!Frame) return false;
	const auto Vector = [](const PlanetCore::Vec3 V) -> FVector {
		return { V.X, V.Y, V.Z };
	};
	const FMatrix Basis(Vector(Frame->East), Vector(Frame->North), Vector(Frame->Up), FVector::ZeroVector);
	const FQuat Rotation = GetActorQuat() * FQuat(Basis).Inverse();
	const FVector Location =
	GetActorLocation() - Rotation.RotateVector(Vector(Frame->Up) * (Frame->RadiusMeters + Frame->AnchorElevationMeters) * 100);
	// 描画の局所原点を含む同じ球面定義を衝突生成へ渡した結果の返戻
	return Patch->RequestFlowPatch(ActiveField, PreparedAnchorDirection, FTransform(Rotation, Location));
}

/**
 * 視点とは独立な湖と河道の支持距離を照会する
 * @param Direction 惑星方向
 * @param SignedMeters 内側が正の支持距離
 * @return 準備済の場と有限な局所照会の場合に真
 */
bool APlanetSphericalSurfacePreview::EvaluateFlowSupport(const FVector Direction, double &SignedMeters) const {
	const std::shared_ptr<const PlanetCore::FlowSurfaceField> ActiveField = GetFlowSurfaceField();
	const std::optional<PlanetCore::UnitDirection> D =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	// 河道比較以外への返戻
	if(!ActiveField || !D) return false;
	const std::optional<double> Support = ActiveField->WaterSupport(*D);
	// 不正な域又は不整合な照会への返戻
	if(!Support || !std::isfinite(*Support)) return false;
	SignedMeters = *Support;
	// 同じ元地形と流出口に依る値の返戻
	return true;
}

/**
 * 表示の局所座標を球面へ戻す為の正確な原点方向を返す
 * @return 準備済の倍精度方向；未準備なら零
 */
FVector APlanetSphericalSurfacePreview::GetDiagnosticAnchorDirection() const {
	// 描画で共有した原点方向の返戻
	return PreparedAnchorDirection;
}

/**
 * 任意の地表方向の周辺を，同じ C++ 場と解析法線で描画する
 * @param Seed 地形の種
 * @param RadiusMeters 1000～1000000000 m の半径
 * @param Direction 観察する地表方向
 * @param HalfExtentMeters 半径の１割以下の診断半幅
 * @param Cells 各軸の分割数，１６～２５６
 * @param ShouldRefineRelief 元の内部起伏を追加した比較面を表示する場合に真
 * @param ShouldWarpCoordinates 可逆な球面座標変形を比較する場合に真
 * @param ShouldPreserveSlopes 単調性を制限した節点勾配を使う場合に真
 * @param ShouldShowSource 同じ生成定義の元地形を直接表示する場合に真
 * @return 実際に診断メッシュを公開した場合に真
 */
bool APlanetSphericalSurfacePreview::BuildPreview(
	const int64 Seed,
	const double RadiusMeters,
	const FVector Direction,
	const double HalfExtentMeters,
	const int32 Cells,
	const bool ShouldRefineRelief,
	const bool ShouldWarpCoordinates,
	const bool ShouldPreserveSlopes,
	const bool ShouldShowSource
) {
	PlanetCore::GeologySettings Settings;
	Settings.Seed = static_cast<std::uint64_t>(Seed);
	Settings.RadiusMeters = RadiusMeters;
	const std::optional<PlanetCore::UnitDirection> Anchor =
	PlanetCore::UnitDirection::FromVector({ Direction.X, Direction.Y, Direction.Z });
	const bool HasValidExtent = std::isfinite(HalfExtentMeters) && HalfExtentMeters > 0 && HalfExtentMeters <= 0.1 * RadiusMeters;
	if(
		!Settings.IsValid() || !Anchor || !HasValidExtent || Cells < 16 || Cells > 256 || ReliefVertexBudget < 0 ||
		ReliefVertexBudget > 16384
	) {
		LastBuildMessage = TEXT("Invalid diagnostic parameters.");
		// 不正な入力で公開済の描画を交換しない返戻
		return false;
	}
	const double Started = FPlatformTime::Seconds();
	std::shared_ptr<const PlanetCore::PlanetSphericalDrainage> Candidate = Surface;
	if(
		!Candidate || PreparedSeed != Settings.Seed || PreparedRadiusMeters != RadiusMeters ||
		HasPreparedRefinedRelief != ShouldRefineRelief || HasPreparedVertexSlopes != ShouldPreserveSlopes ||
		HasPreparedDrainageOnly != ShouldLimitToDrainageEdges || HasPreparedEdgeRefinement != ShouldRefineEdges ||
		PreparedReliefVertexBudget != ReliefVertexBudget || HasPreparedRelaxation != ShouldRelaxAddedHeights ||
		HasPreparedAdaptiveDrainage != ShouldAdaptBeforeDrainage || HasPreparedEstimatedGradients != ShouldEstimateGradients
	) {
		Candidate = PrepareField(
			Settings,
			ShouldRefineRelief,
			ShouldPreserveSlopes,
			ShouldLimitToDrainageEdges,
			ShouldRefineEdges,
			ShouldRelaxAddedHeights,
			ShouldAdaptBeforeDrainage,
			ShouldEstimateGradients,
			ReliefVertexBudget
		);
	}
	if(!Candidate) {
		LastBuildMessage = TEXT("Spherical drainage preparation failed.");
		// 準備に失敗した面を公開しない返戻
		return false;
	}
	std::shared_ptr<const PlanetCore::PlanetSphericalWarp> CandidateWarp;
	if(ShouldWarpCoordinates) {
		PlanetCore::SphericalWarpSettings WarpSettings;
		WarpSettings.Seed = Settings.Seed;
		WarpSettings.RadiusMeters = RadiusMeters;
		const std::optional<PlanetCore::PlanetSphericalWarp> PreparedWarp = PlanetCore::PlanetSphericalWarp::Create(WarpSettings);
		// 可逆な座標写像を準備出来ない場合の返戻
		if(!PreparedWarp) return false;
		CandidateWarp = std::make_shared<const PlanetCore::PlanetSphericalWarp>(*PreparedWarp);
	}
	const double Prepared = FPlatformTime::Seconds();
	const double CenterHeight = SampleField(*Candidate, CandidateWarp.get(), *Anchor, ShouldShowSource).ElevationMeters;
	const std::optional<PlanetCore::GroundFrame> Frame = PlanetCore::GroundFrame::Create(*Anchor, RadiusMeters, CenterHeight);
	// 有効な局所原点が無い場合の返戻
	if(!Frame) return false;
	const int32 Side = Cells + 1, Count = Side * Side;
	TArray<FVector> Vertices, Normals;
	TArray<FLinearColor> Colors;
	TArray<FVector2D> UVs;
	TArray<FProcMeshTangent> Tangents;
	TArray<int32> Indices;
	Vertices.Reserve(Count);
	Normals.Reserve(Count);
	Colors.Reserve(Count);
	Tangents.Reserve(Count);
	Indices.Reserve(Cells * Cells * 6);
	for(int32 Row = 0; Row < Side; ++Row) for(int32 Column = 0; Column < Side; ++Column) {
		const double East = HalfExtentMeters * (2.0 * Column / Cells - 1), North = HalfExtentMeters * (2.0 * Row / Cells - 1);
		const PlanetCore::UnitDirection Point = Frame->DirectionAt(East, North);
		const PlanetCore::TerrainSample Sample = SampleField(*Candidate, CandidateWarp.get(), Point, ShouldShowSource);
		const PlanetCore::Vec3 Local = Frame->SurfaceToLocal(Point, Sample.ElevationMeters);
		const std::optional<PlanetCore::UnitDirection> UnitNormal = PlanetCore::UnitDirection::FromVector(
			Point.Vector() * (RadiusMeters + Sample.ElevationMeters) - Sample.TangentialGradientPerRadian
		);
		// 不正な面法線を公開しない返戻
		if(!UnitNormal) return false;
		const PlanetCore::Vec3 LocalNormal = Frame->VectorToLocal(UnitNormal->Vector());
		const FVector Position(Local.X * 100, Local.Y * 100, Local.Z * 100), Normal(LocalNormal.X, LocalNormal.Y, LocalNormal.Z);
		// 非有限な頂点を公開しない返戻
		if(Position.ContainsNaN() || Normal.ContainsNaN()) return false;
		Vertices.Add(Position);
		Normals.Add(Normal);
		const double Elevation = Sample.ElevationMeters * (RadiusMeters + 250000) / RadiusMeters;
		const float LandWeight = static_cast<float>(FMath::Clamp(Elevation / 4500, 0.0, 1.0));
		const float SeaWeight = static_cast<float>(FMath::Clamp((Elevation + 6000) / 6000, 0.0, 1.0));
		Colors.Add(
			Elevation < 0 ?
			FMath::Lerp(FLinearColor(0.012F, 0.045F, 0.11F), FLinearColor(0.09F, 0.34F, 0.39F), SeaWeight) :
			FMath::Lerp(FLinearColor(0.16F, 0.26F, 0.07F), FLinearColor(0.72F, 0.62F, 0.46F), LandWeight)
		);
		const FVector Axis = FMath::Abs(Normal.Z) < 0.8 ? FVector::UpVector : FVector::ForwardVector;
		Tangents.Emplace(FVector::CrossProduct(Axis, Normal).GetSafeNormal(), false);
	}
	for(int32 Row = 0; Row < Cells; ++Row) for(int32 Column = 0; Column < Cells; ++Column) {
		const int32 A = Row * Side + Column, B = A + 1, C = A + Side, D = C + 1;
		// UE の表向きは時計回り，局所の上向きが描画面となる
		Indices.Append({ A, C, B, B, C, D });
	}
	WaterMesh->ClearAllMeshSections();
	SurfaceMesh->CreateMeshSection_LinearColor(0, Vertices, Indices, Normals, UVs, Colors, Tangents, false);
	PreparedAnchorDirection = FVector(Anchor->Vector().X, Anchor->Vector().Y, Anchor->Vector().Z);
	PreparedHalfExtentMeters = HalfExtentMeters;
	DiagnosticWaterVertices = DiagnosticShoreVertices = DiagnosticWaterColumns = 0;
	DiagnosticWaterAreaMetersSquared = DiagnosticWaterSphereErrorMeters = DiagnosticShoreHeightResidualMeters = 0;
	Surface = std::move(Candidate);
	// 元場が変わった水系を保持して再利用しない
	if(ConnectedNetworkSource != Surface) {
		ConnectedNetwork.reset();
		ConnectedNetworkSource.reset();
		ConnectedInitialPass = FVector::ZeroVector;
		ConnectedReceivingLake.reset();
	}
	FlowPreview.reset();
	FlowNetworkPreview.reset();
	SurfaceMesh->SetVisibility(true);
	WaterMesh->SetVisibility(true);
	if(FlowPreparation && FlowPreparation->Source != Surface) FlowPreparation.reset();
	DiagnosticConnectedSites = DiagnosticReceivingShoreVertices = 0;
	DiagnosticOutletWaterMeters = DiagnosticReceivingWaterMeters = DiagnosticRouteEndMeters = 0;
	Warp = std::move(CandidateWarp);
	PreparedSeed = Settings.Seed;
	PreparedRadiusMeters = RadiusMeters;
	HasPreparedRefinedRelief = ShouldRefineRelief;
	HasPreparedVertexSlopes = ShouldPreserveSlopes;
	HasPreparedDrainageOnly = ShouldLimitToDrainageEdges;
	HasPreparedEdgeRefinement = ShouldRefineEdges;
	HasPreparedRelaxation = ShouldRelaxAddedHeights;
	HasPreparedAdaptiveDrainage = ShouldAdaptBeforeDrainage;
	HasPreparedEstimatedGradients = ShouldEstimateGradients;
	PreparedReliefVertexBudget = ReliefVertexBudget;
	IsShowingSource = ShouldShowSource;
	const PlanetCore::SphericalReliefStatistics &Relief = Surface->GetStatistics().Relief;
	AddedReliefVertices = static_cast<int32>(Relief.AddedVertices);
	MaximumReliefRaiseMeters = Relief.MaximumConditioningRaiseMeters;
	RelaxedReliefVertices = static_cast<int32>(Relief.RelaxedVertices);
	MaximumReliefRelaxationMeters = Relief.MaximumRelaxationMeters;
	MaximumReliefRaiseDirection =
	FVector(Relief.MaximumRaiseDirection.X, Relief.MaximumRaiseDirection.Y, Relief.MaximumRaiseDirection.Z);
	MaximumReliefCutMeters = Relief.MaximumConditioningCutMeters;
	MaximumReliefCutDirection = FVector(Relief.MaximumCutDirection.X, Relief.MaximumCutDirection.Y, Relief.MaximumCutDirection.Z);
	PreparationMilliseconds = (Prepared - Started) * 1000;
	MeshMilliseconds = (FPlatformTime::Seconds() - Prepared) * 1000;
	AnchorElevationMeters = CenterHeight;
	GeneratedVertices = Count;
	LastBuildMessage = TEXT("Spherical drainage diagnostic surface. Display only; runtime terrain remains separate.");
	// 実際に描画へ渡した診断面の返戻
	return true;
}

/**
 * 保存しない診断メッシュを初期化する
 */
APlanetSphericalSurfacePreview::APlanetSphericalSurfacePreview() {
	PrimaryActorTick.bCanEverTick = false;
	SurfaceMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("SphericalSurfaceDiagnostic"));
	SetRootComponent(SurfaceMesh);
	SurfaceMesh->SetFlags(RF_Transient);
	SurfaceMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SurfaceMesh->SetCanEverAffectNavigation(false);
	WaterMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ContourWaterDiagnostic"));
	WaterMesh->SetupAttachment(SurfaceMesh);
	WaterMesh->SetFlags(RF_Transient);
	WaterMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	WaterMesh->SetCanEverAffectNavigation(false);
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(
		TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial")
	);
	if(Material.Succeeded()) SurfaceMesh->SetMaterial(0, Material.Object);
	// 終了
	return;
}
