#include "Core/PlanetMoistureTransport.h"
#include "Core/PlanetCore.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <map>

#if WITH_DEV_AUTOMATION_TESTS
namespace {
	using PlanetCore::MoistureTransportCell;
	using PlanetCore::MoistureTransportColumn;
	using PlanetCore::MoistureTransportFailure;
	using PlanetCore::MoistureTransportLink;
	using PlanetCore::MoistureTransportResult;
	using PlanetCore::MoistureTransportSettings;
	using PlanetCore::PlanetMoistureTransport;

	/**
	 * 小規模な輸送問題を密行列の部分ピボット消去で独立に解く
	 * @param Cells ８地点の大気柱
	 * @param Links 双方向の輸送係数
	 * @return 各地点の水蒸気と雲水
	 */
	std::array<MoistureTransportColumn, 8> SolveReference(
		const std::array<MoistureTransportCell, 8> &Cells, const std::vector<MoistureTransportLink> &Links
	) {
		// 反復解法と異なる全変数の連立式
		std::array<std::array<double, 17>, 16> Matrix{};
		for(std::size_t i = 0; i < Cells.size(); ++i) {
			Matrix[i][i] = Cells[i].CondensationPerDay;
			Matrix[i][i + 8] = -Cells[i].ReevaporationPerDay;
			Matrix[i + 8][i] = -Cells[i].CondensationPerDay;
			Matrix[i + 8][i + 8] = Cells[i].ReevaporationPerDay + Cells[i].PrecipitationPerDay;
			Matrix[i][16] = Cells[i].EvaporationMillimetersPerDay;
		}
		for(const MoistureTransportLink &Link : Links) for(const std::size_t Offset : { 0, 8 }) {
			Matrix[Link.First + Offset][Link.First + Offset] += Link.ForwardSteradiansPerDay / Cells[Link.First].AreaSteradians;
			Matrix[Link.Second + Offset][Link.Second + Offset] += Link.ReverseSteradiansPerDay / Cells[Link.Second].AreaSteradians;
			Matrix[Link.Second + Offset][Link.First + Offset] -= Link.ForwardSteradiansPerDay / Cells[Link.Second].AreaSteradians;
			Matrix[Link.First + Offset][Link.Second + Offset] -= Link.ReverseSteradiansPerDay / Cells[Link.First].AreaSteradians;
		}
		for(std::size_t i = 0; i < Matrix.size(); ++i) {
			std::size_t Pivot = i;
			for(std::size_t j = i + 1; j < Matrix.size(); ++j) if(std::abs(Matrix[j][i]) > std::abs(Matrix[Pivot][i])) Pivot = j;
			std::swap(Matrix[i], Matrix[Pivot]);
			const double Divisor = Matrix[i][i];
			for(std::size_t j = i; j < 17; ++j) Matrix[i][j] /= Divisor;
			for(std::size_t j = 0; j < Matrix.size(); ++j) if(j != i) {
				const double Factor = Matrix[j][i];
				for(std::size_t k = i; k < 17; ++k) Matrix[j][k] -= Factor * Matrix[i][k];
			}
		}
		std::array<MoistureTransportColumn, 8> Columns{};
		for(std::size_t i = 0; i < Columns.size(); ++i) Columns[i] = { Matrix[i][16], Matrix[i + 8][16] };
		// 独立解の返戻
		return Columns;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetMoistureTransportContracts, "PlanetGenerator.MoistureTransport.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 水分輸送の独立解・閉鎖収支・降水応答・入力拒否を検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，個別の不一致は UE に記録
 */
bool FPlanetMoistureTransportContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Begin = FPlatformTime::Seconds();
	MoistureTransportSettings Settings;
	Settings.ResidualToleranceMillimetersPerDay = 1.0E-11;
	Settings.RelativeBudgetTolerance = 1.0E-10;

	// 無輸送の解析解と無降水の正規経路
	for(const double Evaporation : { 0.0, 1.0E-8, 4.0 }) {
		const std::array<MoistureTransportCell, 1> Cells{ MoistureTransportCell{ 0.25, Evaporation, 0.5, 10, 2 } };
		const MoistureTransportResult Result = PlanetMoistureTransport::Solve(Cells, {}, Settings);
		// 正規入力で拒否された場合の返戻
		if(!TestTrue(TEXT("Isolated analytic column converges"), Result.HasConverged())) return false;
		TestTrue(TEXT("Isolated cloud analytic solution"), std::abs(Result.Columns[0].CloudMillimeters - Evaporation / 2) < 1.0E-12);
		TestTrue(TEXT("Isolated vapor analytic solution"), std::abs(Result.Columns[0].VaporMillimeters - Evaporation * 12) < 1.0E-12);
	}

	// 海から一方向に輸送する閉じた環状領域
	std::array<MoistureTransportCell, 8> Cells{};
	Cells[0].EvaporationMillimetersPerDay = 8;
	std::vector<MoistureTransportLink> Links;
	for(std::uint32_t i = 0; i < Cells.size(); ++i) Links.push_back({ i, (i + 1) % 8, 1, 0 });
	const MoistureTransportResult Flat = PlanetMoistureTransport::Solve(Cells, Links, Settings);
	// 基準輸送が準備出来ない場合の返戻
	if(!TestTrue(TEXT("Closed ring converges"), Flat.HasConverged())) return false;
	for(std::size_t i = 2; i < Cells.size(); ++i) {
		TestTrue(TEXT("Continental downstream rainfall diminishes"),
			Flat.Columns[i].CloudMillimeters < Flat.Columns[i - 1].CloudMillimeters);
	}
	Cells[2].ReevaporationPerDay = 0.5;
	const MoistureTransportResult Mountain = PlanetMoistureTransport::Solve(Cells, Links, Settings);
	// 山越え輸送が準備出来ない場合の返戻
	if(!TestTrue(TEXT("Orographic column converges"), Mountain.HasConverged())) return false;
	TestTrue(TEXT("Reduced cloud reevaporation increases windward rainfall"),
		Mountain.Columns[2].CloudMillimeters > Flat.Columns[2].CloudMillimeters);
	double LeewardFlat = 0, LeewardMountain = 0;
	for(std::size_t i = 4; i < Cells.size(); ++i) {
		LeewardFlat += Flat.Columns[i].CloudMillimeters;
		LeewardMountain += Mountain.Columns[i].CloudMillimeters;
	}
	TestTrue(TEXT("Windward removal reduces remote leeward rainfall"), LeewardMountain < LeewardFlat);

	// 異面積・双方向の独立解と，隣接順序・向きに依らない再現
	for(std::size_t i = 0; i < Cells.size(); ++i) Cells[i].AreaSteradians = 0.25 + i * 0.125;
	for(MoistureTransportLink &Link : Links) Link.ReverseSteradiansPerDay = 0.2;
	const MoistureTransportResult Unequal = PlanetMoistureTransport::Solve(Cells, Links, Settings);
	// 異面積輸送が準備出来ない場合の返戻
	if(!TestTrue(TEXT("Unequal area bidirectional transport converges"), Unequal.HasConverged())) return false;
	const std::array<MoistureTransportColumn, 8> Reference = SolveReference(Cells, Links);
	double MaximumReferenceError = 0;
	for(std::size_t i = 0; i < Cells.size(); ++i) {
		MaximumReferenceError = std::max(MaximumReferenceError,
			std::abs(Unequal.Columns[i].VaporMillimeters - Reference[i].VaporMillimeters));
		MaximumReferenceError = std::max(MaximumReferenceError,
			std::abs(Unequal.Columns[i].CloudMillimeters - Reference[i].CloudMillimeters));
	}
	TestTrue(TEXT("Agreement with independent pivoted dense solve"), MaximumReferenceError < 1.0E-8);
	std::reverse(Links.begin(), Links.end());
	for(MoistureTransportLink &Link : Links) {
		std::swap(Link.First, Link.Second);
		std::swap(Link.ForwardSteradiansPerDay, Link.ReverseSteradiansPerDay);
	}
	const MoistureTransportResult Reordered = PlanetMoistureTransport::Solve(Cells, Links, Settings);
	// 並替え後に拒否された場合の返戻
	if(!TestTrue(TEXT("Reordered transport converges"), Reordered.HasConverged())) return false;
	for(std::size_t i = 0; i < Cells.size(); ++i) {
		TestTrue(TEXT("Canonical vapor bit pattern"),
			std::bit_cast<std::uint64_t>(Unequal.Columns[i].VaporMillimeters) ==
			std::bit_cast<std::uint64_t>(Reordered.Columns[i].VaporMillimeters));
		TestTrue(TEXT("Canonical cloud bit pattern"),
			std::bit_cast<std::uint64_t>(Unequal.Columns[i].CloudMillimeters) ==
			std::bit_cast<std::uint64_t>(Reordered.Columns[i].CloudMillimeters));
	}

	// 未収束・中断・不正入力を，正常解と区別
	MoistureTransportSettings OneSweep = Settings;
	OneSweep.MaximumSweeps = 1;
	TestTrue(TEXT("Iteration budget exhaustion rejects publication"),
		PlanetMoistureTransport::Solve(Cells, Links, OneSweep).Failure == MoistureTransportFailure::NotConverged);
	std::stop_source Cancellation;
	Cancellation.request_stop();
	TestTrue(TEXT("Cancelled solve rejects publication"),
		PlanetMoistureTransport::Solve(Cells, Links, Settings, Cancellation.get_token()).Failure ==
		MoistureTransportFailure::Cancelled);
	TestFalse(TEXT("Empty cells rejected"), PlanetMoistureTransport::Solve({}, {}, Settings).HasConverged());
	std::vector<MoistureTransportLink> InvalidLinks = Links;
	InvalidLinks.push_back(Links.front());
	TestTrue(TEXT("Duplicate link rejected"),
		PlanetMoistureTransport::Solve(Cells, InvalidLinks, Settings).Failure == MoistureTransportFailure::InvalidInput);
	InvalidLinks = Links;
	InvalidLinks[0].Second = InvalidLinks[0].First;
	TestFalse(TEXT("Self link rejected"), PlanetMoistureTransport::Solve(Cells, InvalidLinks, Settings).HasConverged());
	InvalidLinks[0].Second = 8;
	TestFalse(TEXT("Out of range index rejected"), PlanetMoistureTransport::Solve(Cells, InvalidLinks, Settings).HasConverged());
	InvalidLinks = Links;
	InvalidLinks[0].ForwardSteradiansPerDay = -1;
	TestFalse(TEXT("Negative flow rejected"), PlanetMoistureTransport::Solve(Cells, InvalidLinks, Settings).HasConverged());
	InvalidLinks[0].ForwardSteradiansPerDay = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("Nonfinite flow rejected"), PlanetMoistureTransport::Solve(Cells, InvalidLinks, Settings).HasConverged());
	std::array<MoistureTransportCell, 8> InvalidCells = Cells;
	InvalidCells[0].AreaSteradians = 0;
	TestFalse(TEXT("Zero area rejected"), PlanetMoistureTransport::Solve(InvalidCells, Links, Settings).HasConverged());
	InvalidCells = Cells;
	InvalidCells[0].PrecipitationPerDay = 0;
	TestFalse(TEXT("Missing precipitation sink rejected"),
		PlanetMoistureTransport::Solve(InvalidCells, Links, Settings).HasConverged());

	// 球面の閉じた輸送網，面積は三角形の球面過剰から独立に積算
	const std::optional<PlanetCore::SphereMesh> Sphere = PlanetCore::BuildUniformSphere(4);
	// 球面の準備に失敗した場合の返戻
	if(!TestTrue(TEXT("Canonical sphere prepares"), Sphere.has_value())) return false;
	std::vector<MoistureTransportCell> SphereCells(Sphere->Directions.size());
	for(MoistureTransportCell &Cell : SphereCells) Cell.AreaSteradians = 0;
	std::map<std::pair<std::uint32_t, std::uint32_t>, bool> Edges;
	for(const std::array<std::uint32_t, 3> &Triangle : Sphere->Triangles) {
		const PlanetCore::Vec3 First = Sphere->Directions[Triangle[0]].Vector();
		const PlanetCore::Vec3 Second = Sphere->Directions[Triangle[1]].Vector();
		const PlanetCore::Vec3 Third = Sphere->Directions[Triangle[2]].Vector();
		const double Area = 2 * std::atan2(std::abs(First.Dot(Second.Cross(Third))),
			1 + First.Dot(Second) + Second.Dot(Third) + Third.Dot(First));
		for(std::size_t i = 0; i < Triangle.size(); ++i) {
			SphereCells[Triangle[i]].AreaSteradians += Area / 3;
			Edges[std::minmax(Triangle[i], Triangle[(i + 1) % 3])] = true;
		}
	}
	std::vector<MoistureTransportLink> SphereLinks;
	for(const std::pair<const std::pair<std::uint32_t, std::uint32_t>, bool> &Edge : Edges) {
		SphereLinks.push_back({ Edge.first.first, Edge.first.second, 0.001, 0.0003 });
	}
	double TotalArea = 0;
	for(std::size_t i = 0; i < SphereCells.size(); ++i) {
		SphereCells[i].EvaporationMillimetersPerDay = Sphere->Directions[i].Vector().X < 0.2 ? 4 : 0;
		TotalArea += SphereCells[i].AreaSteradians;
	}
	TestTrue(TEXT("Sphere area covers four pi"), std::abs(TotalArea - 12.566370614359172) < 1.0E-10);
	TestTrue(TEXT("Sphere link count exceeds node count"), SphereLinks.size() > SphereCells.size());
	const double SphereBegin = FPlatformTime::Seconds();
	const MoistureTransportResult Global = PlanetMoistureTransport::Solve(SphereCells, SphereLinks, Settings);
	const double SphereSeconds = FPlatformTime::Seconds() - SphereBegin;
	// 全球の収束失敗を記録して返戻
	if(!TestTrue(TEXT("Closed spherical atmosphere converges"), Global.HasConverged())) {
		AddError(FString::Printf(TEXT("Sphere failure=%u sweeps=%u residual=%.9g budget=%.9g"),
			static_cast<unsigned>(Global.Failure), Global.Sweeps, Global.MaximumResidualMillimetersPerDay, Global.RelativeBudgetError));
		return false;
	}
	double IndependentEvaporation = 0, IndependentPrecipitation = 0;
	for(std::size_t i = 0; i < SphereCells.size(); ++i) {
		TestTrue(TEXT("Nonnegative atmospheric water"),
			Global.Columns[i].VaporMillimeters >= 0 && Global.Columns[i].CloudMillimeters >= 0);
		IndependentEvaporation += SphereCells[i].AreaSteradians * SphereCells[i].EvaporationMillimetersPerDay;
		IndependentPrecipitation += SphereCells[i].AreaSteradians * SphereCells[i].PrecipitationPerDay *
			Global.Columns[i].CloudMillimeters;
	}
	TestTrue(TEXT("Independent closed sphere water budget"),
		std::abs(IndependentEvaporation - IndependentPrecipitation) / IndependentEvaporation < Settings.RelativeBudgetTolerance);
	AddInfo(FString::Printf(
		TEXT("Moisture transport: nodes=%u links=%u sweeps=%u residual=%.9g budget=%.9g reference=%.9g sphere=%.6fs total=%.6fs"),
		static_cast<unsigned>(SphereCells.size()), static_cast<unsigned>(SphereLinks.size()), Global.Sweeps,
		Global.MaximumResidualMillimetersPerDay, Global.RelativeBudgetError, MaximumReferenceError,
		SphereSeconds, FPlatformTime::Seconds() - Begin
	));
	// 検査実行結果の返戻
	return true;
}
#endif
