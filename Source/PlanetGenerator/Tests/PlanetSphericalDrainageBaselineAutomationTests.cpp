#include "Core/PlanetCore.h"
#include "Core/PlanetDrainage.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOceanDistanceTerrain.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetSphericalMesh.h"
#include "Core/PlanetSphericalRefinement.h"
#include "Core/PlanetSphericalDrainageBaseline.h"
#include "Core/PlanetSphericalContinentalField.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <unordered_map>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	struct FineDrainageSurveyMesh {
		std::vector<PlanetCore::UnitDirection> Directions;
		std::vector<std::array<std::uint32_t, 3>> Triangles;
	};

	/**
	 * 通常のレベル６網の全共有辺を一度だけ分割して閉球面を作る
	 * @return 海域準備と同じ中点規則を持つ一段細分化網
	 */
	std::optional<FineDrainageSurveyMesh> BuildFineDrainageSurveyMesh() {
		std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(6);
		// 通常網を準備出来ない場合の返戻
		if(!Base) return {};
		FineDrainageSurveyMesh Result { std::move(Base->Directions), {} };
		std::unordered_map<std::uint64_t, std::uint32_t> Midpoints;
		Midpoints.reserve(Base->Triangles.size() * 3 / 2);
		Result.Triangles.reserve(Base->Triangles.size() * 4);
		const auto Midpoint = [&Result, &Midpoints](const std::uint32_t A, const std::uint32_t B) -> std::optional<std::uint32_t> {
			const std::uint64_t Key = static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B);
			const std::unordered_map<std::uint64_t, std::uint32_t>::const_iterator Existing = Midpoints.find(Key);
			// 複数の面が共有する辺で同じ頂点を使う返戻
			if(Existing != Midpoints.end()) return Existing->second;
			const std::optional<PlanetCore::UnitDirection> Direction =
			PlanetCore::UnitDirection::FromVector(Result.Directions[A].Vector() + Result.Directions[B].Vector());
			// 無効な球面中点を公開しない返戻
			if(!Direction || Result.Directions.size() >= PlanetCore::SphericalDrainageBaselineMaximumVertices) return {};
			const std::uint32_t Index = static_cast<std::uint32_t>(Result.Directions.size());
			Result.Directions.push_back(*Direction);
			Midpoints.emplace(Key, Index);
			// 新しい共有中点番号の返戻
			return Index;
		};
		for(const std::array<std::uint32_t, 3> &Face : Base->Triangles) {
			const std::uint32_t A = Face[0], B = Face[1], C = Face[2];
			const std::optional<std::uint32_t> AB = Midpoint(A, B), BC = Midpoint(B, C), CA = Midpoint(C, A);
			// 一つでも共有中点を作れなかった場合の返戻
			if(!AB || !BC || !CA) return {};
			Result.Triangles.push_back({ A, *AB, *CA });
			Result.Triangles.push_back({ *AB, B, *BC });
			Result.Triangles.push_back({ *CA, *BC, C });
			Result.Triangles.push_back({ *AB, *BC, *CA });
		}
		// 重複中点の無い閉球面の返戻
		return Result;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSphericalDrainageBaselineContracts,
	"PlanetGenerator.SphericalDrainageBaseline.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 閉球面の最終節点から構成する区分線形基準面の連続性と大洋到達を検査する
 * @param Parameters 未使用の UE 引数
 * @return 位相と有限経路の検査結果，C1・景観・実河川の完成とは別
 */
bool FPlanetSphericalDrainageBaselineContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::SphereMesh;
	using PlanetCore::SphericalDrainageBaseline;
	using PlanetCore::SphericalDrainageBaselineError;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const std::optional<SphereMesh> Mesh = PlanetCore::BuildUniformSphere(2);
	// 閉球面の独立な検査網が無い場合の返戻
	if(!TestTrue(TEXT("閉じた検査用球面網を準備"), Mesh.has_value())) return false;
	const Vec3 Tilt { 0.073, 0.119, 1 };
	std::vector<double> Heights;
	Heights.reserve(Mesh->Directions.size());
	for(const UnitDirection Direction : Mesh->Directions) Heights.push_back(Direction.Vector().Dot(Tilt) - 0.3);
	SphericalDrainageBaselineError Error = SphericalDrainageBaselineError::None;
	const std::optional<SphericalDrainageBaseline> Base =
	SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Heights, 0, Error);
	// 非対称な海陸境界と下降を持つ正規面を受理出来ない場合の返戻
	if(!TestTrue(TEXT("非対称球面の区分線形基準面を受理"), Base.has_value())) {
		AddError(FString::Printf(TEXT("Drainage baseline reason=%u."), static_cast<unsigned>(Error)));
		// 基準面の生成失敗を隠さず，後続の節点検査を中止する返戻
		return false;
	}
	TestTrue(TEXT("正規準備の理由"), Error == SphericalDrainageBaselineError::None);
	TestEqual(TEXT("全節点の個数"), Base->GetStatistics().Vertices, static_cast<std::uint32_t>(Mesh->Directions.size()));
	TestTrue(TEXT("海と陸の双方が存在"), Base->GetStatistics().OceanVertices > 0 && Base->GetStatistics().NonOceanVertices > 0);
	const std::span<const std::uint32_t> Downstream = Base->GetDownstreamVertices();
	std::uint64_t RouteSteps = 0, FaceSamples = 0, SharedEdges = 0;
	std::uint32_t MaximumSteps = 0;
	for(std::uint32_t Start = 0; Start < Heights.size(); ++Start) {
		if(Heights[Start] < 0) {
			TestEqual(TEXT("大洋頂点は自身を根にする"), Downstream[Start], Start);
			continue;
		}
		std::uint32_t Current = Start, Steps = 0;
		while(Heights[Current] >= 0 && Steps < Heights.size()) {
			const std::uint32_t Next = Downstream[Current];
			// 全陸節点の辺が厳密に下る事を確認出来ない場合の返戻
			if(!TestTrue(TEXT("最終標高の下流辺は厳密下降"), Next < Heights.size() && Heights[Next] < Heights[Current])) return false;
			Current = Next;
			++Steps;
			++RouteSteps;
		}
		TestTrue(TEXT("全陸頂点が有限回で大洋へ到達"), Heights[Current] < 0);
		MaximumSteps = std::max(MaximumSteps, Steps);
	}
	for(std::size_t Face = 0; Face < Mesh->Triangles.size(); ++Face) {
		const std::array<std::uint32_t, 3> &Corners = Mesh->Triangles[Face];
		const std::array<double, 3> Weights { 0.2, 0.3, 0.5 };
		const std::optional<double> Height = Base->EvaluateFace(Face, Weights);
		// 面内の正規化重みが有限の最終高さを返さない場合の返戻
		if(!TestTrue(TEXT("面内を連続な重みで評価"), Height && std::isfinite(*Height))) return false;
		unsigned Lowest = 0;
		for(unsigned Corner = 1; Corner < 3; ++Corner) if(Heights[Corners[Corner]] < Heights[Corners[Lowest]]) Lowest = Corner;
		double Previous = *Height;
		for(const double Fraction : { 0.25, 0.5, 0.75, 1.0 }) {
			std::array<double, 3> Along {};
			for(unsigned Corner = 0; Corner < 3; ++Corner) Along[Corner] = Weights[Corner] * (1 - Fraction);
			Along[Lowest] += Fraction;
			const std::optional<double> Lower = Base->EvaluateFace(Face, Along);
			// 面内部から最低頂点への下降を確認出来ない場合の返戻
			if(!TestTrue(TEXT("区分線形面の解析的な下降"), Lower && *Lower <= Previous + 1E-12)) return false;
			if(*Height > Heights[Corners[Lowest]] + 1E-10 && Fraction < 1) TestTrue(TEXT("最低頂点へ向かう区間は厳密下降"), *Lower < Previous);
			Previous = *Lower;
			++FaceSamples;
		}
	}
	// 共有辺の両面が同じ標高を返すか，対応する頂点重みで独立に照合
	struct EdgeFace {
		std::uint32_t A = 0;
		std::uint32_t B = 0;
		std::uint32_t Face = 0;
	};
	std::vector<EdgeFace> Edges;
	Edges.reserve(3 * Mesh->Triangles.size());
	for(std::uint32_t Face = 0; Face < Mesh->Triangles.size(); ++Face) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::array<std::uint32_t, 3> &Corners = Mesh->Triangles[Face];
		const std::uint32_t A = Corners[Side], B = Corners[(Side + 1) % 3];
		Edges.push_back({ std::min(A, B), std::max(A, B), Face });
	}
	std::sort(
		Edges.begin(),
		Edges.end(),
		[](const EdgeFace &A, const EdgeFace &B) -> bool {
			// 同じ共有辺の二面を隣り合わせる返戻
			return std::array{ A.A, A.B, A.Face } < std::array{ B.A, B.B, B.Face };
		}
	);
	for(std::size_t Index = 0; Index < Edges.size(); Index += 2) {
		const EdgeFace &First = Edges[Index], &Second = Edges[Index + 1];
		// 開いた又は非対応の辺を連続性試料として扱わない返戻
		if(!TestTrue(TEXT("共有辺が二面に対応"), First.A == Second.A && First.B == Second.B)) return false;
		std::array<double, 2> Samples {};
		for(unsigned Owner = 0; Owner < 2; ++Owner) {
			const EdgeFace &Edge = Owner ? Second : First;
			std::array<double, 3> Weights {};
			for(unsigned Corner = 0; Corner < 3; ++Corner) {
				const std::uint32_t Vertex = Mesh->Triangles[Edge.Face][Corner];
				Weights[Corner] = Vertex == Edge.A ? 0.37 : Vertex == Edge.B ? 0.63 : 0;
			}
			Samples[Owner] = Base->EvaluateFace(Edge.Face, Weights).value_or(std::numeric_limits<double>::quiet_NaN());
		}
		TestTrue(TEXT("共有辺の両面の値が一致"), std::isfinite(Samples[0]) && std::abs(Samples[0] - Samples[1]) < 1E-12);
		++SharedEdges;
	}
	TestEqual(TEXT("全球閉包の共有辺数"), SharedEdges, static_cast<std::uint64_t>(Base->GetStatistics().Edges));
	TestFalse(TEXT("負の面外重みを拒否"), Base->EvaluateFace(0, { -0.1, 0.5, 0.6 }).has_value());
	TestFalse(TEXT("不正規化を拒否"), Base->EvaluateFace(0, { 0.2, 0.2, 0.2 }).has_value());
	TestFalse(TEXT("面の範囲外を拒否"), Base->EvaluateFace(Mesh->Triangles.size(), { 1, 0, 0 }).has_value());
	std::uint32_t Peak = 0;
	for(std::uint32_t Index = 1; Index < Heights.size(); ++Index) if(Heights[Index] > Heights[Peak]) Peak = Index;
	std::vector<double> Altered = Heights;
	Altered[Peak] = 0.001;
	TestFalse(
		TEXT("海から孤立した乾いた窪地を拒否"),
		SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Altered, 0, Error).has_value()
	);
	TestTrue(TEXT("乾いた窪地の拒否理由"), Error == SphericalDrainageBaselineError::LandSink);
	Altered[Peak] = -0.001;
	TestFalse(
		TEXT("海から孤立した水没点を拒否"),
		SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Altered, 0, Error).has_value()
	);
	TestTrue(TEXT("孤立海域の拒否理由"), Error == SphericalDrainageBaselineError::DisconnectedOcean);
	Altered = Heights;
	const EdgeFace &LandEdge = *std::find_if(
		Edges.begin(),
		Edges.end(),
		[&](const EdgeFace &Edge) -> bool {
			// 二端とも乾いた辺を試験対象に選ぶ返戻
			return Heights[Edge.A] > 0 && Heights[Edge.B] > 0;
		}
	);
	Altered[LandEdge.A] = Altered[LandEdge.B];
	TestFalse(
		TEXT("平坦な乾いた辺を拒否"),
		SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Altered, 0, Error).has_value()
	);
	TestTrue(TEXT("平坦な辺の拒否理由"), Error == SphericalDrainageBaselineError::LevelLandEdge);
	Altered = Heights;
	Altered[Peak] = std::numeric_limits<double>::quiet_NaN();
	TestFalse(
		TEXT("非有限な標高を拒否"),
		SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Altered, 0, Error).has_value()
	);
	TestTrue(TEXT("非有限標高の拒否理由"), Error == SphericalDrainageBaselineError::InvalidInput);
	TestFalse(
		TEXT("開いた全球網を拒否"),
		SphericalDrainageBaseline::Create(
			Mesh->Directions,
			std::span(Mesh->Triangles).first(Mesh->Triangles.size() - 1),
			Heights,
			0,
			Error
		).has_value()
	);
	TestTrue(TEXT("開いた網の拒否理由"), Error == SphericalDrainageBaselineError::InvalidMesh);
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断した構成を拒否"),
		SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Heights, 0, Error, Cancelled.get_token()).has_value()
	);
	TestTrue(TEXT("中断の拒否理由"), Error == SphericalDrainageBaselineError::Cancelled);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Spherical drainage baseline: vertices=%u edges=%u faces=%u wet=%u dry=%u routeSteps=%llu ")
			TEXT("maximumSteps=%u faceSamples=%llu sharedEdges=%llu seconds=%.9g. Piecewise linear reference only."),
			Base->GetStatistics().Vertices,
			Base->GetStatistics().Edges,
			Base->GetStatistics().Triangles,
			Base->GetStatistics().OceanVertices,
			Base->GetStatistics().NonOceanVertices,
			static_cast<unsigned long long>(RouteSteps),
			MaximumSteps,
			static_cast<unsigned long long>(FaceSamples),
			static_cast<unsigned long long>(SharedEdges),
			Seconds
		)
	);
	// 位相基準の有限検査を，滑らかな本番地形の完成とは区別した返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSphericalDrainageBaselineSeededSurvey,
	"PlanetGenerator.SphericalDrainageBaseline.SeededSurvey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実際の Seed 球面場を有限な全球網へ標本化し，排水基準面の受理・拒否を測る
 * @param Parameters 未使用の UE 引数
 * @return 標本化と調査が実行された事であり，受理結果の合格とは区別する
 */
bool FPlanetSphericalDrainageBaselineSeededSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(4);
	// 閉球面の実地形調査網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形用の閉球面網"), Mesh.has_value())) return false;
	const std::array<std::pair<std::uint64_t, double>, 4> Cases {
		std::pair<std::uint64_t, double>{ 7, 1000 },
		std::pair<std::uint64_t, double>{ 42, 1E6 },
		std::pair<std::uint64_t, double>{ 137, 6371000 },
		std::pair<std::uint64_t, double>{ 2026, 1E9 }
	};
	for(const auto &[Seed, Radius] : Cases) {
		PlanetCore::SphericalContinentalFieldError FieldError = PlanetCore::SphericalContinentalFieldError::None;
		const std::optional<PlanetCore::SphericalContinentalField> Field =
		PlanetCore::SphericalContinentalField::CreateSeeded(Seed, Radius, FieldError);
		// 実地形の場が無ければ排水調査結果を捏造しない返戻
		if(!TestTrue(TEXT("Seed球面場を準備"), Field.has_value())) return false;
		std::vector<double> Heights;
		Heights.reserve(Mesh->Directions.size());
		for(const PlanetCore::UnitDirection Direction : Mesh->Directions) {
			const std::optional<double> Height = Field->EvaluateElevation(Direction);
			// 全球の標本値が欠けた条件を有効な調査と扱わない返戻
			if(!TestTrue(TEXT("Seed球面場の有限な標本"), Height && std::isfinite(*Height))) return false;
			Heights.push_back(*Height);
		}
		PlanetCore::SphericalDrainageBaselineError DrainageError = PlanetCore::SphericalDrainageBaselineError::None;
		const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline =
		PlanetCore::SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Heights, 0, DrainageError);
		AddInfo(
			FString::Printf(
				TEXT("Seeded drainage survey: seed=%llu radius=%.9g vertices=%u accepted=%u reason=%u wet=%u dry=%u."),
				static_cast<unsigned long long>(Seed),
				Radius,
				static_cast<unsigned>(Heights.size()),
				static_cast<unsigned>(Baseline.has_value()),
				static_cast<unsigned>(DrainageError),
				Baseline ? Baseline->GetStatistics().OceanVertices : 0,
				Baseline ? Baseline->GetStatistics().NonOceanVertices : 0
			)
		);
	}
	// 観測された受理と拒否を本番の全球排水保証と混同しない返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSphericalDrainageBaselineProductionSurvey,
	"PlanetGenerator.SphericalDrainageBaseline.ProductionSurvey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 通常の侵食後の最終 H を全球網へ標本化し，同じ厳密下降条件との差を測る
 * @param Parameters 未使用の UE 引数
 * @return 実地形の診断を実行出来た事であり，排水条件の受理とは別
 */
bool FPlanetSphericalDrainageBaselineProductionSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
	// 地質の準備失敗を地形の排水結果として報告しない返戻
	if(!TestTrue(TEXT("通常地質を準備"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 大洋接続の準備失敗を最終 H の測定と混同しない返戻
	if(!TestTrue(TEXT("通常大洋を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 元の排水網が無い場合に表面の実測へ進まない返戻
	if(!TestTrue(TEXT("通常排水網を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
	const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	// 侵食と河谷を含む最終地形が無い場合の返戻
	if(!TestTrue(TEXT("通常の最終地形を準備"), Field.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 全球の共通網が無い場合の返戻
	if(!TestTrue(TEXT("通常の全球格子を準備"), Mesh.has_value())) return false;
	std::vector<double> Heights;
	Heights.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) {
		const double Height = Field->EvaluateElevation(Direction);
		// 有限な最終標高が欠ければ排水結果と混同しない返戻
		if(!TestTrue(TEXT("通常最終標高が有限"), std::isfinite(Height))) return false;
		Heights.push_back(Height);
	}
	PlanetCore::SphericalDrainageBaselineError Error = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> Baseline = PlanetCore::SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		Heights,
		Field->Drainage().Settings().SeaLevelMeters,
		Error
	);
	AddInfo(
		FString::Printf(
			TEXT("Production drainage survey: seed=42 radius=1000000 vertices=%u accepted=%u reason=%u."),
			static_cast<unsigned>(Heights.size()),
			static_cast<unsigned>(Baseline.has_value()),
			static_cast<unsigned>(Error)
		)
	);
	// 受理されない実地形を完成済みと誤認しない診断の返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSphericalDrainageBaselineFineMeshSurvey,
	"PlanetGenerator.SphericalDrainageBaseline.FineMeshSurvey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 共有球面網の一段細分化でも同じ排水条件と Seed 実地形を調査する
 * @param Parameters 未使用の UE 引数
 * @return 大規模網の基準契約と標本化の実行結果，実地形の受理とは区別する
 */
bool FPlanetSphericalDrainageBaselineFineMeshSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const std::optional<FineDrainageSurveyMesh> Mesh = BuildFineDrainageSurveyMesh();
	// 一段細分化した全球共有網が無い場合の返戻
	if(!TestTrue(TEXT("細分全球網を準備"), Mesh.has_value())) return false;
	const PlanetCore::Vec3 Tilt { 0.073, 0.119, 1 };
	std::vector<double> Synthetic;
	Synthetic.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) Synthetic.push_back(Direction.Vector().Dot(Tilt) - 0.3);
	PlanetCore::SphericalDrainageBaselineError Error = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> SyntheticBaseline =
	PlanetCore::SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Synthetic, 0, Error);
	// 大きな網で閉包と下降の基準契約を満たせない場合の返戻
	if(!TestTrue(TEXT("細分全球網の既知下降面を受理"), SyntheticBaseline.has_value())) return false;
	PlanetCore::SphericalContinentalFieldError FieldError = PlanetCore::SphericalContinentalFieldError::None;
	const std::optional<PlanetCore::SphericalContinentalField> Field =
	PlanetCore::SphericalContinentalField::CreateSeeded(42, 1E6, FieldError);
	// Seed 地形の準備失敗を位相の拒否と混同しない返戻
	if(!TestTrue(TEXT("細分用Seed球面場を準備"), Field.has_value())) return false;
	std::vector<double> Heights;
	Heights.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Directions) {
		const std::optional<double> Height = Field->EvaluateElevation(Direction);
		// 細分標本が欠けた場合の返戻
		if(!TestTrue(TEXT("細分地形の有限標高"), Height && std::isfinite(*Height))) return false;
		Heights.push_back(*Height);
	}
	const std::optional<PlanetCore::SphericalDrainageBaseline> SeededBaseline =
	PlanetCore::SphericalDrainageBaseline::Create(Mesh->Directions, Mesh->Triangles, Heights, 0, Error);
	const std::optional<PlanetCore::SphereMesh> Coarse = PlanetCore::BuildUniformSphere(6);
	// 比較する粗い共有網が無い場合の返戻
	if(!TestTrue(TEXT("細分前の全球網を準備"), Coarse.has_value())) return false;
	PlanetCore::SphericalDrainageBaselineError CoarseError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> CoarseBaseline = PlanetCore::SphericalDrainageBaseline::Create(
		Coarse->Directions,
		Coarse->Triangles,
		std::span<const double>(Heights).first(Coarse->Directions.size()),
		0,
		CoarseError
	);
	const std::chrono::steady_clock::time_point AdaptiveStarted = std::chrono::steady_clock::now();
	PlanetCore::SphericalReliefSettings AdaptiveSettings;
	AdaptiveSettings.RadiusMeters = 1E6;
	AdaptiveSettings.TargetErrorMeters = 0.5;
	AdaptiveSettings.MaximumAddedVertices = 2048;
	AdaptiveSettings.MaximumDepth = 3;
	const auto Source = [&Field](const PlanetCore::UnitDirection Direction) -> double {
		const std::optional<double> Height = Field->EvaluateElevation(Direction);
		// 無効な元標高を網構築側で拒否させる返戻
		return Height.value_or(std::numeric_limits<double>::quiet_NaN());
	};
	const std::optional<PlanetCore::SphericalReliefSamplingMesh> Adaptive =
	PlanetCore::PlanetSphericalRefinement::CreateSamplingMesh(
		Coarse->Directions,
		Coarse->Triangles,
		std::span<const double>(Heights).first(Coarse->Directions.size()),
		Source,
		AdaptiveSettings
	);
	PlanetCore::SphericalDrainageBaselineError AdaptiveError = PlanetCore::SphericalDrainageBaselineError::None;
	std::optional<PlanetCore::SphericalDrainageBaseline> AdaptiveBaseline;
	if(Adaptive) {
		AdaptiveBaseline = PlanetCore::SphericalDrainageBaseline::Create(
			Adaptive->Directions,
			Adaptive->Triangles,
			Adaptive->ElevationsMeters,
			0,
			AdaptiveError
		);
	}
	const double AdaptiveSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - AdaptiveStarted).count();
	AddInfo(
		FString::Printf(
			TEXT("Adaptive drainage sampling: prepared=%u added=%u sourceEvaluations=%llu ")
			TEXT("accepted=%u reason=%u seconds=%.9g. Diagnostic only."),
			static_cast<unsigned>(Adaptive.has_value()),
			Adaptive ? Adaptive->Statistics.AddedVertices : 0,
			Adaptive ? static_cast<unsigned long long>(Adaptive->Statistics.SourceEvaluations) : 0,
			static_cast<unsigned>(AdaptiveBaseline.has_value()),
			static_cast<unsigned>(AdaptiveError),
			AdaptiveSeconds
		)
	);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage survey: seed=42 radius=1000000 vertices=%u faces=%u syntheticAccepted=1 ")
			TEXT("seededAccepted=%u seededReason=%u coarseAccepted=%u coarseReason=%u seconds=%.9g."),
			static_cast<unsigned>(Mesh->Directions.size()),
			static_cast<unsigned>(Mesh->Triangles.size()),
			static_cast<unsigned>(SeededBaseline.has_value()),
			static_cast<unsigned>(Error),
			static_cast<unsigned>(CoarseBaseline.has_value()),
			static_cast<unsigned>(CoarseError),
			Seconds
		)
	);
	if(!SeededBaseline && Error == PlanetCore::SphericalDrainageBaselineError::LandSink) {
		std::vector<double> NeighborMinimum(Heights.size(), std::numeric_limits<double>::infinity());
		for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			NeighborMinimum[A] = std::min(NeighborMinimum[A], Heights[B]);
			NeighborMinimum[B] = std::min(NeighborMinimum[B], Heights[A]);
		}
		std::uint32_t FirstSink = std::numeric_limits<std::uint32_t>::max(), SinkCount = 0;
		double MaximumDiscreteDepth = 0;
		for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
			if(Heights[Index] < 0 || NeighborMinimum[Index] < Heights[Index]) continue;
			if(FirstSink == std::numeric_limits<std::uint32_t>::max()) FirstSink = Index;
			++SinkCount;
			MaximumDiscreteDepth = std::max(MaximumDiscreteDepth, NeighborMinimum[Index] - Heights[Index]);
		}
		// 拒否理由と独立に数えた離散窪地が一致しない場合の返戻
		if(!TestTrue(TEXT("細分拒否を独立な窪地数で再現"), SinkCount > 0)) return false;
		const PlanetCore::Vec3 Direction = Mesh->Directions[FirstSink].Vector();
		double MinimumEdgeAngle = std::numeric_limits<double>::infinity();
		double MinimumEdgeInteriorHeight = std::numeric_limits<double>::infinity();
		for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			if(A != FirstSink && B != FirstSink) continue;
			const std::uint32_t Neighbor = A == FirstSink ? B : A;
			const PlanetCore::Vec3 End = Mesh->Directions[Neighbor].Vector();
			MinimumEdgeAngle = std::min(MinimumEdgeAngle, std::acos(std::clamp(Direction.Dot(End), -1.0, 1.0)));
			for(unsigned Section = 1; Section < 32; ++Section) {
				const double Fraction = Section / 32.0;
				const std::optional<PlanetCore::UnitDirection> Point =
				PlanetCore::UnitDirection::FromVector(Direction * (1 - Fraction) + End * Fraction);
				// 辺内の方向が無い場合の返戻
				if(!TestTrue(TEXT("細分窪地の辺内方向"), Point.has_value())) return false;
				const std::optional<double> Height = Field->EvaluateElevation(*Point);
				// 辺内の標高が有限でない場合の返戻
				if(!TestTrue(TEXT("細分窪地の辺内標高"), Height && std::isfinite(*Height))) return false;
				MinimumEdgeInteriorHeight = std::min(MinimumEdgeInteriorHeight, *Height);
			}
		}
		const PlanetCore::Vec3 Reference = std::abs(Direction.Z) < 0.8 ? PlanetCore::Vec3{ 0, 0, 1 } : PlanetCore::Vec3{ 0, 1, 0 };
		const std::optional<PlanetCore::UnitDirection> FirstAxis = PlanetCore::UnitDirection::FromVector(Reference.Cross(Direction));
		const std::optional<PlanetCore::TerrainSample> Analytic = Field->Evaluate(Mesh->Directions[FirstSink]);
		// 接線基底と元の解析勾配が無い場合の返戻
		if(!TestTrue(TEXT("細分窪地の解析近傍を準備"), FirstAxis && Analytic && std::isfinite(MinimumEdgeAngle))) return false;
		const PlanetCore::Vec3 SecondAxis = Direction.Cross(FirstAxis->Vector());
		const double ProbeAngle = 0.25 * MinimumEdgeAngle;
		double MinimumNearbyHeight = std::numeric_limits<double>::infinity();
		for(unsigned Bearing = 0; Bearing < 64; ++Bearing) {
			const double Angle = 2 * std::numbers::pi_v<double> * Bearing / 64;
			const PlanetCore::Vec3 Offset = FirstAxis->Vector() * std::cos(Angle) + SecondAxis * std::sin(Angle);
			const std::optional<PlanetCore::UnitDirection> Point =
			PlanetCore::UnitDirection::FromVector(Direction * std::cos(ProbeAngle) + Offset * std::sin(ProbeAngle));
			// 独立な近傍照会を有限で準備出来ない場合の返戻
			if(!TestTrue(TEXT("細分窪地の近傍方向"), Point.has_value())) return false;
			const std::optional<double> Height = Field->EvaluateElevation(*Point);
			// 格子に無い近傍の標高が欠けた場合の返戻
			if(!TestTrue(TEXT("細分窪地の近傍標高"), Height && std::isfinite(*Height))) return false;
			MinimumNearbyHeight = std::min(MinimumNearbyHeight, *Height);
		}
		AddInfo(
			FString::Printf(
				TEXT("Fine drainage sink: count=%u first=%u firstHeight=%.9gm neighborMin=%.9gm ")
				TEXT("maxDiscreteDepth=%.9gm gradient=%.9g nearMinimum=%.9gm edgeInteriorMinimum=%.9gm probeAngle=%.9grad ")
				TEXT("direction=(%.9g,%.9g,%.9g)."),
				SinkCount,
				FirstSink,
				Heights[FirstSink],
				NeighborMinimum[FirstSink],
				MaximumDiscreteDepth,
				Analytic->TangentialGradientPerRadian.Length(),
				MinimumNearbyHeight,
				MinimumEdgeInteriorHeight,
				ProbeAngle,
				Direction.X,
				Direction.Y,
				Direction.Z
			)
		);
	}
	const std::chrono::steady_clock::time_point DistanceStarted = std::chrono::steady_clock::now();
	std::vector<double> Costs;
	Costs.reserve(Mesh->Directions.size());
	for(const PlanetCore::UnitDirection Point : Mesh->Directions) {
		const PlanetCore::Vec3 D = Point.Vector();
		Costs.push_back(0.4 + std::exp(0.55 * std::sin(9 * D.X + 6 * D.Y) * std::cos(7 * D.Z - 3 * D.X)));
	}
	PlanetCore::OceanDistanceTerrainSettings DistanceSettings;
	DistanceSettings.RadiusMeters = 1E6;
	PlanetCore::OceanDistanceTerrainError DistanceError = PlanetCore::OceanDistanceTerrainError::None;
	const std::optional<PlanetCore::OceanDistanceTerrain> DistanceTerrain =
	PlanetCore::OceanDistanceTerrain::Create(Mesh->Directions, Mesh->Triangles, Heights, Costs, DistanceSettings, DistanceError);
	// 大規模網の構成的な大洋距離が準備出来ない場合の返戻
	if(!TestTrue(TEXT("細分全球網の大洋距離地形"), DistanceTerrain.has_value())) return false;
	PlanetCore::SphericalDrainageBaselineError ConstructedError = PlanetCore::SphericalDrainageBaselineError::None;
	const std::optional<PlanetCore::SphericalDrainageBaseline> ConstructedBaseline = PlanetCore::SphericalDrainageBaseline::Create(
		Mesh->Directions,
		Mesh->Triangles,
		DistanceTerrain->GetHeightsMeters(),
		0,
		ConstructedError
	);
	// 同じ高さを面内まで共有する下降契約が無い場合の返戻
	if(!TestTrue(TEXT("細分全球網の構成的排水"), ConstructedBaseline.has_value())) return false;
	const double DistanceSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - DistanceStarted).count();
	AddInfo(
		FString::Printf(
			TEXT("Fine ocean distance terrain: vertices=%u ocean=%u land=%u edges=%u ")
			TEXT("maxHeight=%.9gm minDrop=%.9gm seconds=%.9g. PL diagnostic."),
			DistanceTerrain->GetStatistics().Vertices,
			DistanceTerrain->GetStatistics().OceanVertices,
			DistanceTerrain->GetStatistics().LandVertices,
			DistanceTerrain->GetStatistics().Edges,
			DistanceTerrain->GetStatistics().MaximumLandHeightMeters,
			DistanceTerrain->GetStatistics().MinimumLandDropMeters,
			DistanceSeconds
		)
	);
	// 細分の Seed 標本の受理結果を全連続地形の保証へ拡張しない返戻
	return true;
}

#endif
