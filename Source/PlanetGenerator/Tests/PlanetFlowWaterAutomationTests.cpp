#include "Core/PlanetFlowWater.h"
#include "PlanetSphericalSurfacePreview.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 実際の単精度水面三角形を半径方向へ照会する
	 * @param Mesh 公開候補の水面
	 * @param Direction 照会方向
	 * @param Height 元の連続場の水位
	 * @return 交点が有る場合の半径方向の差；被覆が無い場合は空
	 * @note 実三角形数 T に対して O(T)，生成器の索引を使わない独立検査
	 */
	std::optional<double> TraceWater(
		const PlanetCore::FlowWaterMesh &Mesh,
		const PlanetCore::UnitDirection Direction,
		const double Height
	) {
		// 同じ方向の独立な半径レイと単精度頂点の準備
		const PlanetCore::Vec3 Query = Mesh.Frame.SurfaceToLocal(Direction, Height);
		const PlanetCore::Vec3 Ray = Mesh.Frame.VectorToLocal(Direction.Vector());
		const auto Vertex = [&](const std::uint32_t Id) -> PlanetCore::Vec3 {
			const PlanetCore::Vec3 P = Mesh.Surface.Vertices[Id].Position;
			// UE と同じセンチメートル単精度の返戻
			return { static_cast<float>(P.X * 100) / 100.0, static_cast<float>(P.Y * 100) / 100.0, static_cast<float>(P.Z * 100) / 100.0 };
		};
		// 実三角形の面と重心座標に依る交差判定
		for(const std::array<std::uint32_t, 3> Triangle : Mesh.Surface.Triangles) {
			const PlanetCore::Vec3 A = Vertex(Triangle[0]), B = Vertex(Triangle[1]), C = Vertex(Triangle[2]);
			const PlanetCore::Vec3 U = B - A, V = C - A, Normal = U.Cross(V);
			const double Denominator = Normal.Dot(Ray);
			if(std::abs(Denominator) < 1E-20) continue;
			const double Offset = Normal.Dot(A - Query) / Denominator;
			const PlanetCore::Vec3 P = Query + Ray * Offset - A;
			const double UU = U.Dot(U), VV = V.Dot(V), UV = U.Dot(V), PU = P.Dot(U), PV = P.Dot(V), Det = UU * VV - UV * UV;
			if(!(Det > 0)) continue;
			const double S = (PU * VV - PV * UV) / Det, T = (PV * UU - PU * UV) / Det;
			// 境界の丸めだけを許した実三角形の交点の返戻
			if(S >= -1E-8 && T >= -1E-8 && S + T <= 1 + 1E-8) return Offset;
		}
		// 描画面が無い方向の返戻
		return {};
	}

	/**
	 * 水面の内部点と，同じ分割の実地面三角形の前後関係を確認する
	 * @param Test 結果受取先
	 * @param Ground 最終分割で再評価した地面
	 * @param Water 同じ所有面を参照する水面
	 * @return 単精度化した地面が水面を二センチメートル以上塞がない場合に真
	 * @note 水面三角形数 T に対して O(T)，各面の内部３点を独立に照合
	 */
	bool CheckWaterOcclusion(
		FAutomationTestBase &Test,
		const PlanetCore::FlowTerrainMesh &Ground,
		const PlanetCore::FlowWaterMesh &Water
	) {
		// 実際の単精度位置と三角形の対応の準備
		const auto Quantize = [](const PlanetCore::Vec3 P) -> PlanetCore::Vec3 {
			// UE と同じ座標丸めの返戻
			return { static_cast<float>(P.X * 100) / 100.0, static_cast<float>(P.Y * 100) / 100.0, static_cast<float>(P.Z * 100) / 100.0 };
		};
		double MaximumCover = -1E9;
		std::uint64_t Samples = 0;
		// 対応が無い公開候補への返戻
		if(!Test.TestEqual(TEXT("水面の全三角形に元面の対応"), Water.Surface.SourceTriangles.size(), Water.Surface.Triangles.size())) return false;
		for(std::size_t I = 0; I < Water.Surface.Triangles.size(); ++I) {
			const std::uint32_t Source = Water.Surface.SourceTriangles[I];
			// 水面の分割だけを新しくして地面を古い面に残した場合の返戻
			if(!Test.TestTrue(TEXT("水面と地面が同じ元面を共有"), Source < Ground.Sphere.Mesh.Triangles.size())) return false;
			const std::array<std::uint32_t, 3> GI = Ground.Sphere.Mesh.Triangles[Source], WI = Water.Surface.Triangles[I];
			const PlanetCore::Vec3 A = Quantize(Ground.Positions[GI[0]]), B = Quantize(Ground.Positions[GI[1]]);
			const PlanetCore::Vec3 C = Quantize(Ground.Positions[GI[2]]), Normal = (B - A).Cross(C - A);
			for(
				const std::array<double, 3> Weight :
				std::array<std::array<double, 3>, 3>{ { { 0.2, 0.3, 0.5 }, { 0.6, 0.1, 0.3 }, { 0.15, 0.7, 0.15 } } }
			) {
				const PlanetCore::Vec3 P =
				Quantize(Water.Surface.Vertices[WI[0]].Position) * Weight[0] + Quantize(Water.Surface.Vertices[WI[1]].Position) * Weight[1] +
				Quantize(Water.Surface.Vertices[WI[2]].Position) * Weight[2];
				const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(Water.Frame.LocalToPlanet(P));
				const PlanetCore::Vec3 Ray = Water.Frame.VectorToLocal(D.Vector());
				const double Denominator = Normal.Dot(Ray);
				// 退化した地面を被覆成功にしない返戻
				if(!Test.TestTrue(TEXT("元の地面の面が非退化"), std::abs(Denominator) > 1E-20)) return false;
				MaximumCover = std::max(MaximumCover, Normal.Dot(A - P) / Denominator);
				++Samples;
			}
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Flow water occlusion: sharedGround=%llu water=%llu probes=%llu maximumCover=%.12g"),
				static_cast<unsigned long long>(Ground.Sphere.Mesh.Triangles.size()),
				static_cast<unsigned long long>(Water.Surface.Triangles.size()),
				static_cast<unsigned long long>(Samples),
				MaximumCover
			)
		);
		// 空の成功と地面に埋まる水面を拒否する返戻
		return Test.TestTrue(TEXT("千点以上の共有面を確認"), Samples > 1000) && Test.TestTrue(TEXT("実地面が実水面を二センチメートル以上塞がない"), MaximumCover < 0.02);
	}

	/**
	 * 俯瞰した河岸の実頂点を，細分化で使わない内部位置から検査する
	 * @param Test 結果受取先
	 * @param Ground 水面と共通の最終分割
	 * @param View 実際のカメラ位置と投影尺度
	 * @return 独立な標高差が半画素・法線差が０．１５ラジアン以内なら真
	 * @note 三角形数 T に対して O(T)，元場の照会費用を除く
	 */
	bool CheckBankShape(FAutomationTestBase &Test, const PlanetCore::FlowTerrainMesh &Ground, const PlanetCore::LodView &View) {
		// 描画に使う単精度位置と独立な内部試料の準備
		const auto Quantize = [](const PlanetCore::Vec3 P) -> PlanetCore::Vec3 {
			// 実頂点と同じセンチメートル単精度の返戻
			return { static_cast<float>(P.X * 100) / 100.0, static_cast<float>(P.Y * 100) / 100.0, static_cast<float>(P.Z * 100) / 100.0 };
		};
		std::uint64_t Samples = 0;
		double MaximumPixels = 0, MaximumNormalAngle = 0;
		for(const std::array<std::uint32_t, 3> T : Ground.Sphere.Mesh.Triangles) {
			const PlanetCore::Vec3 A = Quantize(Ground.Positions[T[0]]), B = Quantize(Ground.Positions[T[1]]);
			const PlanetCore::Vec3 C = Quantize(Ground.Positions[T[2]]);
			for(
				const std::array<double, 3> Weight :
				std::array<std::array<double, 3>, 3>{ { { 0.2, 0.3, 0.5 }, { 0.6, 0.1, 0.3 }, { 0.15, 0.7, 0.15 } } }
			) {
				const PlanetCore::Vec3 P = Ground.Frame.LocalToPlanet(A * Weight[0] + B * Weight[1] + C * Weight[2]);
				const double Distance = (P - View.ObserverMeters).Length();
				if(Distance > 750) continue;
				const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(P);
				const std::optional<PlanetCore::FlowChannelSection> Exact = Ground.SourceField->Evaluate(Direction, false);
				// 未定義の岸を標本から除外しない返戻
				if(!Test.TestTrue(TEXT("河岸の独立な場"), Exact.has_value())) return false;
				if(!(Exact->BedCoverage > 0)) continue;
				const double Error = std::abs(P.Length() - View.RadiusMeters - Exact->Terrain.ElevationMeters);
				MaximumPixels = std::max(MaximumPixels, Error * View.FocalLengthPixels / Distance);
				const PlanetCore::UnitDirection Normal = *PlanetCore::UnitDirection::FromVector(
					Ground.PlanetNormals[T[0]] * Weight[0] + Ground.PlanetNormals[T[1]] * Weight[1] + Ground.PlanetNormals[T[2]] * Weight[2]
				);
				const PlanetCore::UnitDirection ExactNormal = *PlanetCore::UnitDirection::FromVector(
					Direction.Vector() * (View.RadiusMeters + Exact->Terrain.ElevationMeters) - Exact->Terrain.TangentialGradientPerRadian
				);
				MaximumNormalAngle = std::max(MaximumNormalAngle, std::acos(std::clamp(Normal.Vector().Dot(ExactNormal.Vector()), -1.0, 1.0)));
				++Samples;
			}
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Flow bank shape: samples=%llu errorPixels=%.12g normalRadians=%.12g"),
				static_cast<unsigned long long>(Samples),
				MaximumPixels,
				MaximumNormalAngle
			)
		);
		// 欠落した標本で成功にせず，投影した形状差を確認する返戻
		return Test.TestTrue(TEXT("河岸を百点以上で独立検査"), Samples > 100) && Test.TestTrue(TEXT("河岸の標高差が半画素以内"), MaximumPixels < 0.5) &&
		Test.TestTrue(TEXT("河岸の独立な補間法線差"), MaximumNormalAngle < 0.15);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowWaterContracts,
	"PlanetGenerator.FlowChannel.GlobalWater",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実地形の細い河道と両端の湖を，同じ全球分割から発見して水位精度を確認する
 * @param Parameters 未使用の実行引数
 * @return 被覆・近景精度・拒否後の正規経路を満たした場合に真
 */
bool FPlanetFlowWaterContracts::RunTest(const FString &Parameters) {
	// エディタの検査世界の取得
	static_cast<void>(Parameters);
	UWorld *World = nullptr;
	if(GEngine) {
		for(const FWorldContext &Context : GEngine->GetWorldContexts()) {
			if(Context.WorldType == EWorldType::Editor && Context.World()) World = Context.World();
		}
	}
	// 検査世界無しの返戻
	if(!TestNotNull(TEXT("検査世界"), World)) return false;
	// 所有する一時アクターの準備
	FActorSpawnParameters Spawn;
	Spawn.ObjectFlags |= RF_Transient;
	const auto Destroy = [](APlanetSphericalSurfacePreview *const Actor) -> void {
		if(IsValid(Actor)) Actor->Destroy();
		// 所有アクターの解放の終了
		return;
	};
	const std::unique_ptr<APlanetSphericalSurfacePreview, decltype(Destroy)> Actor(
		World->SpawnActor<APlanetSphericalSurfacePreview>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn),
		Destroy
	);
	// 一時アクターを作れない場合の返戻
	if(!TestNotNull(TEXT("元場の準備"), Actor.get())) return false;
	// 実地形で確認済の出口と同じ連続場の準備
	const FVector Pass(0.54180127453542282, 0.81192128360203597, -0.21733662403243018);
	const bool Prepared = Actor->BuildPreview(42, 1E6, Pass, 200, 16, false, false, false, true) &&
	Actor->BuildConnectedFlowChannelPreview(Pass, 1500, 32, 32);
	// 部分的な元場の使用を拒否する返戻
	if(!TestTrue(TEXT("接続済の河道を準備"), Prepared)) return false;
	// 同じ場を所有する地面と水面の生成器
	const std::shared_ptr<const PlanetCore::FlowChannelField> Field = Actor->GetFlowField();
	const std::unique_ptr<PlanetCore::FlowTerrainView> TerrainBuilder = PlanetCore::FlowTerrainView::Create(Field);
	const double IndexStart = FPlatformTime::Seconds();
	const std::unique_ptr<PlanetCore::FlowWaterView> WaterBuilder = PlanetCore::FlowWaterView::Create(Field);
	// 索引又は場の準備を確認する返戻
	if(!TestTrue(TEXT("水域の発見用索引"), TerrainBuilder && WaterBuilder)) return false;
	AddInfo(
		FString::Printf(
			TEXT("Flow water guides=%llu error=%.12g seconds=%.9g"),
			static_cast<unsigned long long>(WaterBuilder->GuideCount()),
			WaterBuilder->GetMaximumGuideErrorMeters(),
			FPlatformTime::Seconds() - IndexStart
		)
	);
	// 拒否する準備入力の検査
	TestFalse(TEXT("元場無しの準備を拒否"), static_cast<bool>(PlanetCore::FlowWaterView::Create({})));
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("中断済の準備を拒否"), static_cast<bool>(PlanetCore::FlowWaterView::Create(Field, Cancel.get_token())));
	// 足元と描画視点を分けた通常要求の準備
	const FVector Initial = Actor->GetDiagnosticAnchorDirection();
	const PlanetCore::UnitDirection Anchor = *PlanetCore::UnitDirection::FromVector({ Initial.X, Initial.Y, Initial.Z });
	PlanetCore::LodView View;
	View.RadiusMeters = Field->GetRadiusMeters();
	View.ObserverMeters = Anchor.Vector() * (View.RadiusMeters + Actor->AnchorElevationMeters + 1.8);
	View.SurfaceObserverMeters = Anchor.Vector() * (View.RadiusMeters + Actor->AnchorElevationMeters + 1);
	View.SurfaceRadiusMeters = 24;
	View.LeafBudget = 32768;
	View.PixelLeafBudget = 2048;
	View.TargetEdgePixels = 64;
	std::optional<PlanetCore::FlowTerrainMesh> Terrain = TerrainBuilder->Build(View);
	// 地形候補未達への返戻
	if(!TestTrue(TEXT("同じ河床の全球候補"), Terrain.has_value())) return false;
	// 再利用元が別の場又は不完全な属性の場合は，正規の候補と混ぜない
	PlanetCore::FlowTerrainMesh InvalidPrevious = *Terrain;
	InvalidPrevious.SourceField.reset();
	PlanetCore::FlowViewDiagnostics RejectedGround;
	TestFalse(
		TEXT("別の場の地面頂点を再利用しない"),
		PlanetCore::BuildFlowTerrainPartition(Field, View, Terrain->Sphere, Terrain->Tree, {}, RejectedGround, &InvalidPrevious)
		.has_value()
	);
	InvalidPrevious.SourceField = Field;
	InvalidPrevious.Heights.clear();
	TestFalse(
		TEXT("不完全な頂点属性を再利用しない"),
		PlanetCore::BuildFlowTerrainPartition(Field, View, Terrain->Sphere, Terrain->Tree, {}, RejectedGround, &InvalidPrevious)
		.has_value()
	);
	// 異なる定義及び中断の拒否と，続く正規生成の確認
	PlanetCore::WaterRefinementDiagnostics Diagnostics;
	PlanetCore::FlowTerrainMesh Wrong = *Terrain;
	Wrong.SourceField.reset();
	TestFalse(TEXT("同じ半径でも異なる定義の水面を拒否"), WaterBuilder->Build(Wrong, View, Diagnostics).has_value());
	TestFalse(TEXT("中断済の水面生成を拒否"), WaterBuilder->Build(*Terrain, View, Diagnostics, Cancel.get_token()).has_value());
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetCore::FlowWaterMesh> Water = WaterBuilder->Build(*Terrain, View, Diagnostics);
	AddInfo(
		FString::Printf(
			TEXT("Flow global water: pass=%u source=%llu/%llu failed=%llu feature=%llu waterError=%.12g shoreError=%.12g ")
			TEXT("floatError=%.12g edgePixels=%.12g seconds=%.9g stages=%.9g/%.9g/%.9g/%.9g"),
			Diagnostics.Pass,
			static_cast<unsigned long long>(Diagnostics.SourceVertices),
			static_cast<unsigned long long>(Diagnostics.SourceTriangles),
			static_cast<unsigned long long>(Diagnostics.FailedTriangles),
			static_cast<unsigned long long>(Diagnostics.FeatureTriangles),
			Diagnostics.WaterErrorMeters,
			Diagnostics.ShoreErrorMeters,
			Diagnostics.FloatErrorMeters,
			Diagnostics.MaximumShoreEdgePixels,
			FPlatformTime::Seconds() - Started,
			Diagnostics.StageMilliseconds[0],
			Diagnostics.StageMilliseconds[1],
			Diagnostics.StageMilliseconds[2],
			Diagnostics.StageMilliseconds[3]
		)
	);
	// 未達を部分的な表示の成功にしない返戻
	if(!TestTrue(TEXT("全球候補から川・湖の水面を切出"), Water.has_value())) return false;
	TestTrue(TEXT("非空の水面"), !Water->Surface.Triangles.empty());
	// 近景でも地面と水面の同じ最終分割を確認する返戻
	if(!CheckWaterOcclusion(*this, *Terrain, *Water)) return false;
	TestTrue(TEXT("同じ位置の法線数"), Water->Normals.size() == Water->Surface.Vertices.size());
	// 発見用標本と異なる河道点の被覆検査
	const PlanetCore::FlowWaterGeometry Geometry = Field->GetWaterGeometry();
	unsigned RiverSamples = 0, NearSamples = 0;
	double MaximumNearError = 0;
	for(unsigned I = 0; I < 257; ++I) {
		const double Distance = Geometry.Route.GetLengthMeters() * (I + 0.37) / 257;
		const PlanetCore::UnitDirection Direction = Geometry.Route.Sample(Distance)->Direction;
		const std::optional<PlanetCore::FlowChannelSection> Sample = Field->Evaluate(Direction);
		const std::optional<double> Support = Field->WaterSupport(Direction);
		// 独立標本の欠落を除外して合格にしない返戻
		if(!TestTrue(TEXT("独立な河道標本"), Sample && Support && Sample->HasWater)) return false;
		if(*Support <= 0.05 || Sample->Water.ElevationMeters - Sample->Terrain.ElevationMeters <= 0.05) continue;
		const std::optional<double> Hit = TraceWater(*Water, Direction, Sample->Water.ElevationMeters);
		// 全球の細い川を頂点標本の外で確認する返戻
		if(!TestTrue(FString::Printf(TEXT("河道の水面被覆 %u / %.9g m"), I, Distance), Hit.has_value())) return false;
		++RiverSamples;
	}
	// 格子頂点に一致しない独立な近景標本の検査
	for(int X = -20; X < 21; ++X) for(int Y = -20; Y < 21; ++Y) {
		const PlanetCore::UnitDirection Direction = Water->Frame.DirectionAt(X + 0.137, Y + 0.293);
		if((Direction.Vector() - Anchor.Vector()).Length() * View.RadiusMeters > 23) continue;
		const std::optional<PlanetCore::FlowChannelSection> Sample = Field->Evaluate(Direction);
		const std::optional<double> Support = Field->WaterSupport(Direction);
		// 近景標本の欠落への返戻
		if(!TestTrue(TEXT("独立な近景標本"), Sample && Support)) return false;
		if(!Sample->HasWater || *Support <= 0.03 || Sample->Water.ElevationMeters - Sample->Terrain.ElevationMeters <= 0.03) continue;
		const std::optional<double> Hit = TraceWater(*Water, Direction, Sample->Water.ElevationMeters);
		// 近景の欠けた水面への返戻
		if(!TestTrue(TEXT("独立な近景の水面被覆"), Hit.has_value())) return false;
		MaximumNearError = std::max(MaximumNearError, std::abs(*Hit));
		++NearSamples;
	}
	// 標本数の下限と水位誤差の照合
	TestTrue(TEXT("河道を二百点以上で検査"), RiverSamples > 200);
	TestTrue(TEXT("近景の水面を百点以上で検査"), NearSamples > 100);
	TestTrue(TEXT("独立な単精度水面も二センチメートル以内"), MaximumNearError < 0.02);
	AddInfo(
		FString::Printf(
			TEXT("Flow water independent: vertices=%llu triangles=%llu river=%u near=%u error=%.12g"),
			static_cast<unsigned long long>(Water->Surface.Vertices.size()),
			static_cast<unsigned long long>(Water->Surface.Triangles.size()),
			RiverSamples,
			NearSamples,
			MaximumNearError
		)
	);
	// 実画面で欠落した俯瞰と同じ距離の回帰検査
	View.ObserverMeters = Terrain->Frame.LocalToPlanet({ -800, -1400, 1600 });
	std::optional<PlanetCore::FlowTerrainMesh> FarTerrain = TerrainBuilder->Build(View);
	// 俯瞰の地面を生成出来ない場合の返戻
	if(!TestTrue(TEXT("俯瞰の地形候補"), FarTerrain.has_value())) return false;
	const std::optional<PlanetCore::FlowWaterMesh> FarWater = WaterBuilder->Build(*FarTerrain, View, Diagnostics);
	// 水面の欠落を検査対象から除外しない返戻
	if(!TestTrue(TEXT("俯瞰の共通分割"), FarWater.has_value())) return false;
	CheckWaterOcclusion(*this, *FarTerrain, *FarWater);
	// 三角形の岸が目立った中距離の実画面も，同じ身体位置で確認
	View.ObserverMeters = Terrain->Frame.LocalToPlanet({ -120, -220, 300 });
	std::optional<PlanetCore::FlowTerrainMesh> MiddleTerrain = TerrainBuilder->Build(View);
	// 正規の中距離要求を受理出来ない場合の返戻
	if(!TestTrue(TEXT("中距離の地面候補"), MiddleTerrain.has_value())) return false;
	const std::optional<PlanetCore::FlowWaterMesh> MiddleWater = WaterBuilder->Build(*MiddleTerrain, View, Diagnostics);
	// 河岸を含む共通分割が不成立の場合の返戻
	if(!TestTrue(TEXT("中距離の河岸を含む分割"), MiddleWater.has_value())) return false;
	CheckBankShape(*this, *MiddleTerrain, View);
	CheckWaterOcclusion(*this, *MiddleTerrain, *MiddleWater);
	// 視点と独立した残差を再利用し，同じ視点の全出力が変わらない事を照合
	const PlanetCore::FlowWaterReuseStatistics BeforeReuse = WaterBuilder->GetReuseStatistics();
	std::optional<PlanetCore::FlowTerrainMesh> RepeatedTerrain = TerrainBuilder->Build(View);
	// 正規の再生成入力を準備出来ない場合の返戻
	if(!TestTrue(TEXT("再利用比較用の地面"), RepeatedTerrain.has_value())) return false;
	const double RepeatStart = FPlatformTime::Seconds();
	PlanetCore::FlowViewDiagnostics RepeatGround;
	const std::optional<PlanetCore::FlowWaterMesh> RepeatedWater =
	WaterBuilder->Build(*RepeatedTerrain, View, Diagnostics, {}, &RepeatGround);
	// 再利用時の欠落を無視しない返戻
	if(!TestTrue(TEXT("再利用した水面と河岸"), RepeatedWater.has_value())) return false;
	const double RepeatSeconds = FPlatformTime::Seconds() - RepeatStart;
	const PlanetCore::FlowWaterReuseStatistics AfterReuse = WaterBuilder->GetReuseStatistics();
	// 再利用無しでも，同じ最終分割の全属性が一致する事を別に確認
	PlanetCore::FlowViewDiagnostics FreshReport;
	const std::optional<PlanetCore::FlowTerrainMesh> FreshGround =
	PlanetCore::BuildFlowTerrainPartition(Field, View, RepeatedTerrain->Sphere, RepeatedTerrain->Tree, {}, FreshReport);
	// 未完成の比較面を合格として使わない返戻
	if(!TestTrue(TEXT("再利用無しの地面比較"), FreshGround.has_value())) return false;
	const auto SameVector = [](const PlanetCore::Vec3 A, const PlanetCore::Vec3 B) -> bool {
		// 丸めを許さない各成分の照合結果の返戻
		return A.X == B.X && A.Y == B.Y && A.Z == B.Z;
	};
	bool Identical = MiddleTerrain->Sphere.Mesh.Triangles == RepeatedTerrain->Sphere.Mesh.Triangles &&
	MiddleWater->Surface.Triangles == RepeatedWater->Surface.Triangles &&
	MiddleWater->Surface.SourceTriangles == RepeatedWater->Surface.SourceTriangles &&
	MiddleTerrain->Positions.size() == RepeatedTerrain->Positions.size() &&
	FreshGround->Positions.size() == RepeatedTerrain->Positions.size() &&
	MiddleWater->Surface.Vertices.size() == RepeatedWater->Surface.Vertices.size();
	if(Identical) {
		for(std::size_t I = 0; I < MiddleTerrain->Positions.size(); ++I) {
			const PlanetCore::SurfaceMaterialWeights &A = MiddleTerrain->Materials[I], &B = RepeatedTerrain->Materials[I];
			const PlanetCore::SurfaceMaterialWeights &C = FreshGround->Materials[I];
			Identical = Identical && SameVector(MiddleTerrain->Positions[I], RepeatedTerrain->Positions[I]) &&
			SameVector(MiddleTerrain->Normals[I], RepeatedTerrain->Normals[I]) &&
			MiddleTerrain->Heights[I] == RepeatedTerrain->Heights[I] && A.Rock == B.Rock && A.Sand == B.Sand && A.Soil == B.Soil &&
			A.Wetness == B.Wetness && SameVector(FreshGround->Positions[I], RepeatedTerrain->Positions[I]) &&
			SameVector(FreshGround->Normals[I], RepeatedTerrain->Normals[I]) && FreshGround->Heights[I] == RepeatedTerrain->Heights[I] &&
			C.Rock == B.Rock && C.Sand == B.Sand && C.Soil == B.Soil && C.Wetness == B.Wetness;
		}
		for(std::size_t I = 0; I < MiddleWater->Surface.Vertices.size(); ++I) {
			Identical = Identical && SameVector(MiddleWater->Surface.Vertices[I].Position, RepeatedWater->Surface.Vertices[I].Position) &&
			SameVector(MiddleWater->Normals[I], RepeatedWater->Normals[I]);
		}
	}
	TestTrue(TEXT("河岸の残差再利用でも全頂点・法線・接合が完全一致"), Identical);
	TestTrue(TEXT("河岸の残差を視点要求間で再利用"), AfterReuse.BankProbeHits > BeforeReuse.BankProbeHits);
	TestEqual(TEXT("保持枠内の同じ視点は新しい河岸残差を評価しない"), AfterReuse.BankProbeEvaluations, BeforeReuse.BankProbeEvaluations);
	TestTrue(TEXT("河岸の再利用 RAM が上限内"), AfterReuse.BankSamples < 262145 && AfterReuse.BankChecks < 131073);
	TestTrue(TEXT("現在の視点でも前回の完全分割を再利用"), AfterReuse.PartitionReuses > BeforeReuse.PartitionReuses);
	TestTrue(TEXT("保持する完全分割も有限"), AfterReuse.RetainedLeaves > 0 && AfterReuse.RetainedLeaves < 131073);
	TestEqual(TEXT("同じ分割の地面は元場を再照会しない"), RepeatGround.VertexSampleEvaluations, std::uint64_t{ 0 });
	TestEqual(
		TEXT("全ての地面頂点の不変属性を再利用"),
		RepeatGround.VertexSampleReuses,
		static_cast<std::uint64_t>(RepeatedTerrain->Positions.size())
	);
	TestTrue(TEXT("再利用した地面にも現在の近景精度検査"), RepeatGround.ValidationSamples > 0);
	TestTrue(TEXT("水面の再分割後も共有辺の中点だけを再利用"), RepeatGround.ValidationReuses > 0);
	TestEqual(
		TEXT("水面との共通地面の検査点を省略しない"),
		RepeatGround.ValidationSamples,
		RepeatGround.ValidationEvaluations + RepeatGround.ValidationReuses
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow shared validation: probes=%llu evaluations=%llu reuse=%llu"),
			static_cast<unsigned long long>(RepeatGround.ValidationSamples),
			static_cast<unsigned long long>(RepeatGround.ValidationEvaluations),
			static_cast<unsigned long long>(RepeatGround.ValidationReuses)
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow ground reuse: hits=%llu evaluations=%llu validation=%llu"),
			static_cast<unsigned long long>(RepeatGround.VertexSampleReuses),
			static_cast<unsigned long long>(RepeatGround.VertexSampleEvaluations),
			static_cast<unsigned long long>(RepeatGround.ValidationSamples)
		)
	);
	TestTrue(TEXT("水位と法線の元場を視点間で再利用"), AfterReuse.WaterSampleHits > BeforeReuse.WaterSampleHits);
	TestTrue(TEXT("水位標本の保持量が有限"), AfterReuse.WaterSamples > 0 && AfterReuse.WaterSamples < 262145);
	AddInfo(
		FString::Printf(
			TEXT("Flow water reuse: hits=%llu evaluations=%llu supports=%llu samples=%llu"),
			static_cast<unsigned long long>(AfterReuse.WaterSampleHits - BeforeReuse.WaterSampleHits),
			static_cast<unsigned long long>(AfterReuse.WaterSampleEvaluations - BeforeReuse.WaterSampleEvaluations),
			static_cast<unsigned long long>(AfterReuse.WaterSupportEvaluations - BeforeReuse.WaterSupportEvaluations),
			static_cast<unsigned long long>(AfterReuse.WaterSamples)
		)
	);
	AddInfo(
		FString::Printf(
			TEXT(
				"Flow bank reuse: seconds=%.9g hits=%llu evaluations=%llu samples=%llu checks=%llu partitions=%llu leaves=%llu identical=%d"
			),
			RepeatSeconds,
			static_cast<unsigned long long>(AfterReuse.BankProbeHits - BeforeReuse.BankProbeHits),
			static_cast<unsigned long long>(AfterReuse.BankProbeEvaluations - BeforeReuse.BankProbeEvaluations),
			static_cast<unsigned long long>(AfterReuse.BankSamples),
			static_cast<unsigned long long>(AfterReuse.BankChecks),
			static_cast<unsigned long long>(AfterReuse.PartitionReuses),
			static_cast<unsigned long long>(AfterReuse.RetainedLeaves),
			Identical ? 1 : 0
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow repeat costs: water=%.9g discovery=%.9g bank=%.9g normals=%.9g ground=%.9g ms"),
			AfterReuse.StageMilliseconds[0] - BeforeReuse.StageMilliseconds[0],
			AfterReuse.StageMilliseconds[1] - BeforeReuse.StageMilliseconds[1],
			AfterReuse.StageMilliseconds[2] - BeforeReuse.StageMilliseconds[2],
			AfterReuse.StageMilliseconds[3] - BeforeReuse.StageMilliseconds[3],
			AfterReuse.StageMilliseconds[4] - BeforeReuse.StageMilliseconds[4]
		)
	);
	// 独立標本・拒否・正規生成を合わせた結果の返戻
	return !HasAnyErrors();
}

#endif
