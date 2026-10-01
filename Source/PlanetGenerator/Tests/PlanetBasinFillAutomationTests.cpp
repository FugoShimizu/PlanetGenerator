#include "Core/PlanetBasinFill.h"
#include "Core/PlanetDrainage.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 海岸が赤道に一致する解析的な元地形を評価する
	 * @param Direction 単位方向
	 * @return 海面０に対する標高と解析接線勾配
	 */
	PlanetCore::TerrainSample AnalyticalCoast(const PlanetCore::UnitDirection Direction) {
		const PlanetCore::Vec3 P = Direction.Vector();
		// 平面から切出した球面上の解析的な標高の返戻
		return { 100 * P.Z, (PlanetCore::Vec3{ 0, 0, 1 } - P * P.Z) * 100 };
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBasinFillContracts, "PlanetGenerator.BasinReconstruction.LandFill",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 連続な盛土を解析海岸と実惑星へ適用し，海底保持・非負性・勾配を確認する
 * @param Parameters 未使用の UE 検査引数
 * @return 全ての標本拘束と海岸・勾配の検査を満たす場合に真
 */
bool FPlanetBasinFillContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetBasinFill;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(4);
	// 解析海岸の固定球面網を準備の失敗時の返戻
	if(!TestTrue(TEXT("解析海岸の固定球面網を準備"), Mesh.has_value())) return false;
	std::vector<double> Original, Minimum;
	for(const UnitDirection Direction : Mesh->Directions) {
		const double Height = AnalyticalCoast(Direction).ElevationMeters;
		Original.push_back(Height);
		Minimum.push_back(Height + (Height > 0 ? 20 * std::pow(Height / 100, 3) : 0));
	}
	const std::optional<PlanetBasinFill> Fill = PlanetBasinFill::Create(4, Original, Minimum, 0, 5);
	// 海岸保持の非負な盛土を準備の失敗時の返戻
	if(!TestTrue(TEXT("海岸保持の非負な盛土を準備"), Fill.has_value())) return false;
	for(std::size_t Index = 0; Index < Original.size(); ++Index) {
		TestTrue(
			TEXT("正値化と海岸遷移後にも標本拘束を保持"),
			std::abs(Fill->ApplyElevation(Mesh->Directions[Index], Original[Index]) - Minimum[Index]) <= 2E-6
		);
	}
	double MaximumGradientError = 0;
	for(std::uint32_t Index = 0; Index < 512; ++Index) {
		const double Z = -1 + 2 * (Index + 0.37) / 512, Angle = Index * 2.399963229728653, Radial = std::sqrt(1 - Z * Z);
		const UnitDirection Direction = *UnitDirection::FromVector({ Radial * std::cos(Angle), Radial * std::sin(Angle), Z });
		const Vec3 P = Direction.Vector();
		const TerrainSample Before = AnalyticalCoast(Direction), After = Fill->Apply(Direction, Before);
		TestTrue(TEXT("どの照会地点にも非負の盛土だけを加える"), After.ElevationMeters >= Before.ElevationMeters);
		TestEqual(TEXT("値だけの経路が勾配付き経路と一致"), Fill->ApplyElevation(Direction, Before.ElevationMeters), After.ElevationMeters);
		if(Before.ElevationMeters <= 0) {
			TestTrue(
				TEXT("元の海底の値と接線を厳密に保持"),
				Before.ElevationMeters == After.ElevationMeters &&
				!(Before.TangentialGradientPerRadian - After.TangentialGradientPerRadian).Length()
			);
		}
		const Vec3 Axis = std::abs(P.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
		const Vec3 Tangent = UnitDirection::FromVector(P.Cross(Axis))->Vector();
		for(const Vec3 Along : { Tangent, P.Cross(Tangent) }) {
			const auto Difference = [&](const double Step) {
				const UnitDirection BeforeDirection = *UnitDirection::FromVector(P * std::cos(Step) - Along * std::sin(Step));
				const UnitDirection AfterDirection = *UnitDirection::FromVector(P * std::cos(Step) + Along * std::sin(Step));
				const double Low = Fill->ApplyElevation(BeforeDirection, AnalyticalCoast(BeforeDirection).ElevationMeters);
				const double High = Fill->ApplyElevation(AfterDirection, AnalyticalCoast(AfterDirection).ElevationMeters);
				// 元地形と盛土の両方を変位させた独立差分の返戻
				return (High - Low) / (2 * Step);
			};
			const double Independent = (4 * Difference(5E-6) - Difference(1E-5)) / 3;
			const double Analytical = After.TangentialGradientPerRadian.Dot(Along);
			const double Error = std::abs(Analytical - Independent) / std::max(1.0, std::abs(Analytical));
			MaximumGradientError = std::max(MaximumGradientError, Error);
			TestTrue(TEXT("海岸遷移と正値化を含む解析微分が一致"), Error <= 1E-5);
		}
	}
	const UnitDirection Shore = *UnitDirection::FromVector({ 1, 0, 0 });
	const TerrainSample ShoreSource = AnalyticalCoast(Shore), ShoreFilled = Fill->Apply(Shore, ShoreSource);
	TestTrue(
		TEXT("厳密な海岸位置で標高と法線を保持"),
		ShoreFilled.ElevationMeters == 0 &&
		!(ShoreFilled.TangentialGradientPerRadian - ShoreSource.TangentialGradientPerRadian).Length()
	);
	std::vector<double> Invalid = Minimum;
	for(std::size_t Index = 0; Index < Original.size(); ++Index) if(Original[Index] < 0) {
		Invalid[Index] = Original[Index] + 1;
		break;
	}
	TestFalse(TEXT("海底を盛土する不正な指定を拒否"), PlanetBasinFill::Create(4, Original, Invalid, 0, 5).has_value());
	TestFalse(TEXT("海岸幅０を拒否"), PlanetBasinFill::Create(4, Original, Minimum, 0, 0).has_value());
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("中断済みの盛土を拒否"), PlanetBasinFill::Create(4, Original, Minimum, 0, 5, Cancel.get_token()).has_value());
	// 標本では全て高い地形でも，標本間の窪地を同じ盆地底迄埋める回帰検査
	const std::vector<double> HighGround(Mesh->Directions.size(), 100), BasinFloor(Mesh->Directions.size(), 50);
	const std::optional<PlanetBasinFill> InteriorFill = PlanetBasinFill::Create(4, HighGround, BasinFloor, 0, 5);
	// 標本間の盆地を評価する準備に失敗した場合の返戻
	if(!TestTrue(TEXT("元地形より低い盆地底も保持する"), InteriorFill.has_value())) return false;
	const UnitDirection Pit = *UnitDirection::FromVector(
		Mesh->Directions[Mesh->Triangles[0][0]].Vector() + Mesh->Directions[Mesh->Triangles[0][1]].Vector() +
		Mesh->Directions[Mesh->Triangles[0][2]].Vector()
	);
	double Width = 1;
	for(const UnitDirection Direction : Mesh->Directions) {
		Width = std::min(Width, (Direction.Vector() - Pit.Vector()).Length() * 0.5);
	}
	const auto PitHeight = [&](const UnitDirection Direction) {
		const double R = (Direction.Vector() - Pit.Vector()).Length() / Width;
		// 全ての固定標本を含む支持外の元地形の返戻
		if(R >= 1) return 100.0;
		const double Rest = 1 - R, Square = Rest * Rest;
		// 固定標本間にだけ存在する解析的な窪地の返戻
		return 100 - 90 * Square * Square * (1 + 4 * R);
	};
	for(const UnitDirection Direction : Mesh->Directions) TestEqual(TEXT("窪地が全ての固定標本から離れて居る"), PitHeight(Direction), 100.0);
	TestTrue(TEXT("標本間の深い窪地を必要な盆地底迄盛土する"), std::abs(InteriorFill->ApplyElevation(Pit, PitHeight(Pit)) - 50) <= 2E-6);
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = 1000000;
	const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Definition);
	// 実惑星の地質を準備の失敗時の返戻
	if(!TestTrue(TEXT("実惑星の地質を準備"), Geology.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
	// 実惑星の元海域を準備の失敗時の返戻
	if(!TestTrue(TEXT("実惑星の元海域を準備"), Ocean.has_value())) return false;
	const std::optional<PlanetCore::DrainageNetwork> Network = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 実惑星の元排水を準備の失敗時の返戻
	if(!TestTrue(TEXT("実惑星の元排水を準備"), Network.has_value())) return false;
	const std::optional<PlanetCore::DrainageNetwork> Balanced = Network->WithBalancedTerrainProfiles(
		[&Ocean](const UnitDirection Direction) {
			return Ocean->EvaluateElevation(Direction);
		}
	);
	// 実河道の内部と合流を同時に準備の失敗時の返戻
	if(!TestTrue(TEXT("実河道の内部と合流を同時に準備"), Balanced.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> GlobalMesh = PlanetCore::BuildUniformSphere(6);
	// 排水の元地点と同じ全球網を準備の失敗時の返戻
	if(!TestTrue(TEXT("排水の元地点と同じ全球網を準備"), GlobalMesh.has_value())) return false;
	std::vector<double> Heights, Required;
	Heights.reserve(GlobalMesh->Directions.size());
	Required.reserve(GlobalMesh->Directions.size());
	for(std::size_t Index = 0; Index < GlobalMesh->Directions.size(); ++Index) {
		const PlanetCore::DrainageNode &Node = Balanced->Nodes()[Index];
		const double Height = Ocean->EvaluateElevation(GlobalMesh->Directions[Index]);
		const double Head = std::max(0.0, Node.WaterMeters - Definition.SeaLevelMeters);
		const double Freeboard = Balanced->Settings().BankFreeboardMeters;
		Heights.push_back(Height);
		Required.push_back(Node.Ocean ? Height : Node.WaterMeters + Freeboard * Head / (Head + Freeboard));
	}
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetBasinFill> PlanetFill = PlanetBasinFill::Create(6, Heights, Required, Definition.SeaLevelMeters, 1);
	const double Milliseconds = (FPlatformTime::Seconds() - Started) * 1000;
	// 実惑星の共有水位を支える面状の盛土を準備の失敗時の返戻
	if(!TestTrue(TEXT("実惑星の共有水位を支える面状の盛土を準備"), PlanetFill.has_value())) return false;
	double MaximumFill = 0, MaximumResidual = 0, MaximumRiverDeficit = 0, MaximumSlope = 0;
	std::size_t WorstReach = 0;
	unsigned WorstKnot = 0;
	double WorstOriginal = 0, WorstFilled = 0, WorstBed = 0, WorstWater = 0;
	for(std::size_t Index = 0; Index < GlobalMesh->Directions.size(); ++Index) {
		const UnitDirection Direction = GlobalMesh->Directions[Index];
		const TerrainSample Before = Ocean->Evaluate(Direction), After = PlanetFill->Apply(Direction, Before);
		TestTrue(
			TEXT("実惑星の海陸符号を変更しない"),
			Before.ElevationMeters < Definition.SeaLevelMeters == After.ElevationMeters < Definition.SeaLevelMeters
		);
		MaximumResidual = std::max(MaximumResidual, std::abs(After.ElevationMeters - std::max(Heights[Index], Required[Index])));
		MaximumFill = std::max(MaximumFill, After.ElevationMeters - Before.ElevationMeters);
		MaximumSlope = std::max(MaximumSlope, After.TangentialGradientPerRadian.Length() / Definition.RadiusMeters);
	}
	for(std::size_t Reach = 0; Reach < Balanced->Reaches().size(); ++Reach) for(unsigned Knot = 0; Knot < 65; ++Knot) {
		const std::optional<PlanetCore::RiverProfileSample> Sample = Balanced->SampleReach(Reach, Knot / 64.0);
		// 実河道の途中を独立に照会の失敗時の返戻
		if(!TestTrue(TEXT("実河道の途中を独立に照会"), Sample.has_value())) return false;
		const UnitDirection Direction = *UnitDirection::FromVector(Sample->Direction);
		const double Height = PlanetFill->ApplyElevation(Direction, Ocean->EvaluateElevation(Direction));
		if(Sample->BedMeters - Height > MaximumRiverDeficit) {
			MaximumRiverDeficit = Sample->BedMeters - Height;
			WorstReach = Reach;
			WorstKnot = Knot;
			WorstOriginal = Ocean->EvaluateElevation(Direction);
			WorstFilled = Height;
			WorstBed = Sample->BedMeters;
			WorstWater = Sample->WaterMeters;
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Basin fill worst: reach=%llu knot=%u original=%.9fm filled=%.9fm bed=%.9fm water=%.9fm upstream=%u ") TEXT("downstream=%u."),
			static_cast<unsigned long long>(WorstReach),
			WorstKnot,
			WorstOriginal,
			WorstFilled,
			WorstBed,
			WorstWater,
			Balanced->Reaches()[WorstReach].Upstream,
			Balanced->Reaches()[WorstReach].Downstream
		)
	);
	TestTrue(TEXT("実際の地点評価でも粗網の盛土拘束が一致"), MaximumResidual <= 2E-6);
	AddInfo(
		FString::Printf(
			TEXT("Land fill: analyticalGradientRelative=%.12g sites=%u prepare=%.6fms residual=%.12gm ")
			TEXT("maxFill=%.6fm maximumNodeSlope=%.9f unresolvedRiverBedDeficit=%.6fm. Not production terrain."),
			MaximumGradientError,
			PlanetFill->Statistics().Sites,
			Milliseconds,
			MaximumResidual,
			MaximumFill,
			MaximumSlope,
			MaximumRiverDeficit
		)
	);
	std::vector<UnitDirection> ConstraintDirections;
	std::vector<double> ConstraintOriginal, ConstraintMinimum;
	std::size_t OceanConflicts = 0;
	double MaximumOceanConflict = 0;
	const double ConstraintStarted = FPlatformTime::Seconds();
	for(std::size_t Reach = 0; Reach < Balanced->Reaches().size(); ++Reach) {
		const std::optional<PlanetCore::RiverCrossSectionWidths> Widths = Balanced->ChannelWidths(Reach);
		// 河岸の正規入力を作れない場合の返戻
		if(!TestTrue(TEXT("追加拘束の河岸幅を準備"), Widths.has_value())) return false;
		const double Angle = Widths->InnerHalfWidthMeters / Definition.RadiusMeters;
		for(unsigned Knot = 0; Knot < 65; ++Knot) {
			const std::optional<PlanetCore::RiverProfileSample> Sample = Balanced->SampleReach(Reach, Knot / 64.0);
			// 河道内部の拘束を取得出来ない場合の返戻
			if(!TestTrue(TEXT("追加拘束の河道内部を準備"), Sample.has_value())) return false;
			const Vec3 Across = UnitDirection::FromVector(Sample->Direction.Cross(Sample->DownstreamTangent))->Vector();
			const double Head = std::max(0.0, Sample->WaterMeters - Definition.SeaLevelMeters);
			const double Freeboard = Balanced->Settings().BankFreeboardMeters;
			const double RequiredHeight = Sample->WaterMeters + Freeboard * Head / (Head + Freeboard);
			for(const double Side : { -1.0, 0.0, 1.0 }) {
				const UnitDirection Direction =
				*UnitDirection::FromVector(Sample->Direction * std::cos(Side * Angle) + Across * std::sin(Side * Angle));
				const double Before = Ocean->EvaluateElevation(Direction);
				if(Before <= Definition.SeaLevelMeters) {
					if(Head > 1E-6) {
						++OceanConflicts;
						MaximumOceanConflict = std::max(MaximumOceanConflict, Head);
					}
					continue;
				}
				ConstraintDirections.push_back(Direction);
				ConstraintOriginal.push_back(Before);
				ConstraintMinimum.push_back(RequiredHeight);
			}
		}
	}
	const std::optional<PlanetBasinFill> Constrained =
	PlanetFill->WithMinimumElevations(ConstraintDirections, ConstraintOriginal, ConstraintMinimum);
	const double ConstraintMilliseconds = (FPlatformTime::Seconds() - ConstraintStarted) * 1000;
	// 追加地点の下限を満たせない場合の返戻
	if(!TestTrue(TEXT("河道内部と両岸の最低標高を同じ盆地へ反映"), Constrained.has_value())) return false;
	double MaximumConstraintDeficit = 0, IndependentBedDeficit = 0, MaximumNodeSurplus = 0;
	for(std::size_t Index = 0; Index < ConstraintDirections.size(); ++Index) {
		MaximumConstraintDeficit = std::max(
			MaximumConstraintDeficit,
			ConstraintMinimum[Index] - Constrained->ApplyElevation(ConstraintDirections[Index], ConstraintOriginal[Index])
		);
	}
	for(std::size_t Index = 0; Index < GlobalMesh->Directions.size(); ++Index) {
		const UnitDirection Direction = GlobalMesh->Directions[Index];
		const double Before = PlanetFill->ApplyElevation(Direction, Heights[Index]);
		const double After = Constrained->ApplyElevation(Direction, Heights[Index]);
		TestTrue(TEXT("河道拘束の追加が元の盆地の下限を壊さない"), After >= Before - 2E-6);
		MaximumNodeSurplus = std::max(MaximumNodeSurplus, After - Before);
	}
	for(std::size_t Reach = 0; Reach < Balanced->Reaches().size(); ++Reach) for(unsigned Knot = 0; Knot < 257; ++Knot) {
		const std::optional<PlanetCore::RiverProfileSample> Sample = Balanced->SampleReach(Reach, Knot / 256.0);
		// 準備標本間の独立検査を取得出来ない場合の返戻
		if(!TestTrue(TEXT("河道拘束間を４倍の密度で照会"), Sample.has_value())) return false;
		const UnitDirection Direction = *UnitDirection::FromVector(Sample->Direction);
		const double Height = Constrained->ApplyElevation(Direction, Ocean->EvaluateElevation(Direction));
		IndependentBedDeficit = std::max(IndependentBedDeficit, Sample->BedMeters - Height);
	}
	TestTrue(TEXT("全追加地点の最低標高を実際の面が満たす"), MaximumConstraintDeficit <= 2E-6);
	TestFalse(TEXT("追加拘束でも海底の埋立を拒否"), Fill->WithMinimumElevations(Mesh->Directions, Original, Invalid).has_value());
	TestFalse(TEXT("追加拘束でも中断を拒否"), Fill->WithMinimumElevations(Mesh->Directions, Original, Minimum, Cancel.get_token()).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Constrained basin floor: constraints=%llu raised=%u prepare=%.6fms deficit=%.12gm ")
			TEXT("independentBedDeficit=%.9fm maxAdditionalNodeFill=%.9fm bound=%.9fm oceanConflicts=%llu ")
			TEXT("maxOceanConflict=%.9fm. Not production terrain."),
			static_cast<unsigned long long>(ConstraintDirections.size()),
			Constrained->Statistics().LowerBounds,
			ConstraintMilliseconds,
			MaximumConstraintDeficit,
			IndependentBedDeficit,
			MaximumNodeSurplus,
			Constrained->Statistics().AbsoluteBoundMeters,
			static_cast<unsigned long long>(OceanConflicts),
			MaximumOceanConflict
		)
	);
	// 補間の成立と未解決の河道内部の不足量を区別した検査結果の返戻
	return !HasAnyErrors();
}

#endif
