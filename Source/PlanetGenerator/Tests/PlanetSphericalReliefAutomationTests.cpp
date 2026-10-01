#include "Core/PlanetDrainage.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSphericalDrainage.h"
#include "Core/PlanetSphericalRefinement.h"
#include "Core/PlanetSphericalSpline.h"
#include "Core/PlanetSphericalWarp.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalReliefMeshContracts, "PlanetGenerator.SphericalRelief.MeshContracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 内部細分化の辺・下降先・有限性・補間接続と，独立試料での起伏再現を検査する
 * @param Parameters 未使用の UE 引数
 * @return 全契約が成立した場合に真，全惑星の排水又は景観の完成を意味しない
 */
bool FPlanetSphericalReliefMeshContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetSphericalRefinement;
	using PlanetCore::SphericalReliefSettings;
	using PlanetCore::SphericalSplineField;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	using Triangle = std::array<std::uint32_t, 3>;
	using Edge = std::pair<std::uint32_t, std::uint32_t>;
	const std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(2);
	// 試験入力を準備出来ない場合の返戻
	if(!TestTrue(TEXT("基準網を準備"), Base.has_value())) return false;
	const Triangle Focus = Base->Triangles[17];
	const UnitDirection Center = *UnitDirection::FromVector(
		Base->Directions[Focus[0]].Vector() + Base->Directions[Focus[1]].Vector() + Base->Directions[Focus[2]].Vector()
	);
	const auto Source = [Center](const UnitDirection Direction) -> double {
		const double DistanceSquared = (Direction.Vector() - Center.Vector()).Dot(Direction.Vector() - Center.Vector());
		// 節点の間に位置する解析的な山頂の返戻
		return 100 + 40 * Direction.Vector().Z + 1000 * std::exp(-DistanceSquared / (2 * 0.045 * 0.045));
	};
	std::vector<double> Heights;
	for(const UnitDirection Direction : Base->Directions) Heights.push_back(Source(Direction));
	SphericalReliefSettings Settings;
	Settings.TargetErrorMeters = 0.5;
	Settings.MaximumAddedVertices = 512;
	const std::optional<PlanetSphericalRefinement> Refined =
	PlanetSphericalRefinement::Create(Base->Directions, Base->Triangles, Heights, Source, Settings);
	// 正規入力から内部細分化を準備出来ない場合の返戻
	if(!TestTrue(TEXT("内部点を追加した網を準備"), Refined.has_value())) return false;
	const std::size_t OriginalCount = Base->Directions.size(), Count = Refined->GetDirections().size();
	TestTrue(TEXT("予算内で点を追加"), Count > OriginalCount && Count <= OriginalCount + Settings.MaximumAddedVertices);
	TestEqual(TEXT("閉球面の Euler 特性を保持"), Refined->GetTriangles().size(), 2 * (Count - 2));
	for(std::size_t Vertex = 0; Vertex < OriginalCount; ++Vertex) {
		TestTrue(TEXT("元の節点位置を保持"), !(Base->Directions[Vertex].Vector() - Refined->GetDirections()[Vertex].Vector()).Length());
		TestEqual(TEXT("元の標高を保持"), Refined->GetElevations()[Vertex], Heights[Vertex]);
	}
	double MinimumAngle = 3.141592653589793;
	for(const Triangle &Face : Refined->GetTriangles()) for(unsigned Corner = 0; Corner < 3; ++Corner) {
		const Vec3 Anchor = Refined->GetDirections()[Face[Corner]].Vector();
		const Vec3 A = Refined->GetDirections()[Face[(Corner + 1) % 3]].Vector() - Anchor;
		const Vec3 B = Refined->GetDirections()[Face[(Corner + 2) % 3]].Vector() - Anchor;
		MinimumAngle = std::min(MinimumAngle, std::atan2(A.Cross(B).Length(), A.Dot(B)));
	}
	TestTrue(TEXT("内部点の追加で極端な細長い面を作らない"), MinimumAngle > 5 * 3.141592653589793 / 180);
	std::map<Edge, std::pair<unsigned, int>> Edges;
	for(const Triangle &Face : Refined->GetTriangles()) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::uint32_t First = Face[Side], Second = Face[(Side + 1) % 3];
		const Edge Key { std::min(First, Second), std::max(First, Second) };
		++Edges[Key].first;
		Edges[Key].second += First < Second ? 1 : -1;
	}
	for(const std::pair<const Edge, std::pair<unsigned, int>> &Entry : Edges) {
		TestTrue(TEXT("全共有辺が逆向きの２面で閉じる"), Entry.second.first == 2 && !Entry.second.second);
	}
	for(const Triangle &Face : Base->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::uint32_t First = Face[Side], Second = Face[(Side + 1) % 3];
		TestTrue(TEXT("元辺を分割も除去もしない"), Edges.contains({ std::min(First, Second), std::max(First, Second) }));
	}
	for(std::uint32_t Vertex = static_cast<std::uint32_t>(OriginalCount); Vertex < Count; ++Vertex) {
		const std::uint32_t Parent = Refined->GetDownstreamVertices()[Vertex];
		TestTrue(TEXT("追加点の下降先は先行する節点"), Parent < Vertex);
		TestTrue(TEXT("追加点の下降先を実辺として保持"), Edges.contains({ Parent, Vertex }));
		TestTrue(TEXT("追加点から厳密に標高が下がる"), Refined->GetElevations()[Parent] < Refined->GetElevations()[Vertex]);
	}
	const std::optional<SphericalSplineField> Before = SphericalSplineField::Create(2, Heights);
	const std::optional<SphericalSplineField> After =
	SphericalSplineField::CreateOnMesh(2, Refined->GetDirections(), Refined->GetTriangles(), Refined->GetElevations());
	// 比較する連続場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("両方の C1 場を準備"), Before.has_value() && After.has_value())) return false;
	double BeforeSquared = 0, AfterSquared = 0, MaximumJoin = 0, MaximumNormalAngle = 0;
	for(unsigned Index = 0; Index < 8192; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / 8192, Angle = Index * 2.399963229728653, Radius = std::sqrt(1 - Z * Z);
		const UnitDirection Direction = *UnitDirection::FromVector({ Radius * std::cos(Angle), Radius * std::sin(Angle), Z });
		const double Original = Source(Direction);
		BeforeSquared += std::pow(Before->Evaluate(Direction).ElevationMeters - Original, 2);
		AfterSquared += std::pow(After->Evaluate(Direction).ElevationMeters - Original, 2);
	}
	TestTrue(TEXT("節点選択に使わない試料で再現誤差を削減"), AfterSquared < 0.64 * BeforeSquared);
	using Owner = std::pair<std::size_t, unsigned>;
	std::map<Edge, Owner> Owners;
	for(std::size_t Face = 0; Face < Refined->GetTriangles().size(); ++Face) {
		const Triangle &Corners = Refined->GetTriangles()[Face];
		for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t First = Corners[Side], Second = Corners[(Side + 1) % 3];
			const Edge Key { std::min(First, Second), std::max(First, Second) };
			const std::map<Edge, Owner>::const_iterator Found = Owners.find(Key);
			if(Found == Owners.end()) {
				Owners.emplace(Key, Owner{ Face, Side });
				continue;
			}
			const std::size_t OtherFace = Found->second.first;
			const unsigned OtherSide = Found->second.second;
			for(unsigned Half = 0; Half < 2; ++Half) {
				const std::uint32_t Anchor = Corners[(Side + Half) % 3];
				const unsigned OtherHalf = Refined->GetTriangles()[OtherFace][OtherSide] == Anchor ? 0 : 1;
				const std::size_t Part = 6 * Face + 2 * Side + Half, OtherPart = 6 * OtherFace + 2 * OtherSide + OtherHalf;
				const std::array<Vec3, 3> Domain = *After->GetPieceDomain(Part);
				for(const double Fraction : { 0.17, 0.5, 0.89 }) {
					const UnitDirection Point = *UnitDirection::FromVector(Domain[0] * (1 - Fraction) + Domain[1] * Fraction);
					const PlanetCore::TerrainSample Left = *After->EvaluatePiece(Part, Point);
					const PlanetCore::TerrainSample Right = *After->EvaluatePiece(OtherPart, Point);
					MaximumJoin = std::max(MaximumJoin, std::abs(Left.ElevationMeters - Right.ElevationMeters));
					const UnitDirection LeftNormal =
					*UnitDirection::FromVector(Point.Vector() * (Settings.RadiusMeters + Left.ElevationMeters) - Left.TangentialGradientPerRadian);
					const UnitDirection RightNormal = *UnitDirection::FromVector(
						Point.Vector() * (Settings.RadiusMeters + Right.ElevationMeters) - Right.TangentialGradientPerRadian
					);
					MaximumNormalAngle = std::max(
						MaximumNormalAngle,
						std::atan2(LeftNormal.Vector().Cross(RightNormal.Vector()).Length(), LeftNormal.Vector().Dot(RightNormal.Vector()))
					);
				}
			}
			Owners.erase(Found);
		}
	}
	TestTrue(TEXT("補間区画の値を連続に接続"), MaximumJoin < 1E-6);
	TestTrue(TEXT("補間区画の法線を連続に接続"), MaximumNormalAngle < 1E-6);
	TestTrue(TEXT("未対応の共有辺を残さない"), Owners.empty());
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断入力を拒否"),
		PlanetSphericalRefinement::Create(Base->Directions, Base->Triangles, Heights, Source, Settings, Cancelled.get_token())
		.has_value()
	);
	TestFalse(
		TEXT("標高数の不一致を拒否"),
		PlanetSphericalRefinement::Create(Base->Directions, Base->Triangles, {}, Source, Settings).has_value()
	);
	TestFalse(
		TEXT("非有限な元関数を拒否"),
		PlanetSphericalRefinement::Create(
			Base->Directions,
			Base->Triangles,
			Heights,
			[](const UnitDirection) -> double {
				return std::numeric_limits<double>::quiet_NaN();
			},
			Settings
		).has_value()
	);
	Settings.MaximumAddedVertices = 0;
	const std::optional<PlanetSphericalRefinement> Unchanged =
	PlanetSphericalRefinement::Create(Base->Directions, Base->Triangles, Heights, Source, Settings);
	TestTrue(TEXT("追加予算０も正規入力として受理"), Unchanged.has_value() && !Unchanged->GetStatistics().AddedVertices);
	AddInfo(
		FString::Printf(
			TEXT("Spherical relief mesh: original=%llu added=%u samples=%llu rmsBefore=%.12g rmsAfter=%.12g join=%.12g normal=%.12g ")
			TEXT("minimumAngle=%.12g minimumCreatedQuality=%.12g"),
			static_cast<unsigned long long>(OriginalCount),
			Refined->GetStatistics().AddedVertices,
			static_cast<unsigned long long>(Refined->GetStatistics().SourceEvaluations),
			std::sqrt(BeforeSquared / 8192),
			std::sqrt(AfterSquared / 8192),
			MaximumJoin,
			MaximumNormalAngle,
			MinimumAngle,
			Refined->GetStatistics().MinimumCreatedTriangleQuality
		)
	);
	// 位相・下降辺・再現性・接続の検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FPlanetSphericalReliefTerrainComparison, "PlanetGenerator.SphericalRelief.TerrainComparison",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 同じ Seed の最小半径と基準半径を比較対象へ登録する
 * @param Names 表示用の検査名
 * @param Commands RunTest へ渡す識別子
 */
void FPlanetSphericalReliefTerrainComparison::GetTests(TArray<FString> &Names, TArray<FString> &Commands) const {
	Names.Add(TEXT("MinimumRadius"));
	Commands.Add(TEXT("MinimumRadius"));
	Names.Add(TEXT("ReferenceRadius"));
	Commands.Add(TEXT("ReferenceRadius"));
	Names.Add(TEXT("MinimumRadiusWithSlopes"));
	Commands.Add(TEXT("MinimumRadiusWithSlopes"));
	Names.Add(TEXT("ReferenceRadiusWithSlopes"));
	Commands.Add(TEXT("ReferenceRadiusWithSlopes"));
	Names.Add(TEXT("MinimumRadiusWithDrainageSlopes"));
	Commands.Add(TEXT("MinimumRadiusWithDrainageSlopes"));
	Names.Add(TEXT("ReferenceRadiusWithDrainageSlopes"));
	Commands.Add(TEXT("ReferenceRadiusWithDrainageSlopes"));
	Names.Add(TEXT("MinimumRadiusWithEdgeSlopes"));
	Commands.Add(TEXT("MinimumRadiusWithEdgeSlopes"));
	Names.Add(TEXT("ReferenceRadiusWithEdgeSlopes"));
	Commands.Add(TEXT("ReferenceRadiusWithEdgeSlopes"));
	Names.Add(TEXT("MinimumRadiusWithDenseEdgeSlopes"));
	Commands.Add(TEXT("MinimumRadiusWithDenseEdgeSlopes"));
	Names.Add(TEXT("ReferenceRadiusWithDenseEdgeSlopes"));
	Commands.Add(TEXT("ReferenceRadiusWithDenseEdgeSlopes"));
	Names.Add(TEXT("MinimumRadiusWithDenseRelaxedEdgeSlopes"));
	Commands.Add(TEXT("MinimumRadiusWithDenseRelaxedEdgeSlopes"));
	Names.Add(TEXT("ReferenceRadiusWithDenseRelaxedEdgeSlopes"));
	Commands.Add(TEXT("ReferenceRadiusWithDenseRelaxedEdgeSlopes"));
	Names.Add(TEXT("MinimumRadiusWithDenseSourceRelaxedEdgeSlopes"));
	Commands.Add(TEXT("MinimumRadiusWithDenseSourceRelaxedEdgeSlopes"));
	Names.Add(TEXT("ReferenceRadiusWithDenseSourceRelaxedEdgeSlopes"));
	Commands.Add(TEXT("ReferenceRadiusWithDenseSourceRelaxedEdgeSlopes"));
	// 終了
	return;
}

/**
 * 元地質に対する独立誤差と，追加細分化後の河道辺の単調性を比較する
 * @param Parameters 半径を選択する検査名
 * @return 選択した惑星の比較条件が成立する場合に真，景観・全点流出の受入れとは別
 */
bool FPlanetSphericalReliefTerrainComparison::RunTest(const FString &Parameters) {
	using PlanetCore::DrainageNetwork;
	using PlanetCore::GeologyField;
	using PlanetCore::OceanConnectivityField;
	using PlanetCore::PlanetSphericalDrainage;
	using PlanetCore::SphericalSplineField;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = Parameters.StartsWith(TEXT("MinimumRadius")) ? 1000 : 1000000;
	const std::optional<GeologyField> PreparedGeology = GeologyField::Create(Definition);
	// 元地質が無い場合の返戻
	if(!TestTrue(TEXT("元地質を準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const GeologyField> Geology = std::make_shared<const GeologyField>(*PreparedGeology);
	const std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 山稜が無い場合の返戻
	if(!TestTrue(TEXT("元山稜を準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(*PreparedRidges);
	std::optional<OceanConnectivityField> PreparedOcean = OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		Ridges,
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 共有海域が無い場合の返戻
	if(!TestTrue(TEXT("共有海域を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const OceanConnectivityField> Ocean =
	std::make_shared<const OceanConnectivityField>(std::move(*PreparedOcean));
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 元網が無い場合の返戻
	if(!TestTrue(TEXT("元網を準備"), Mesh.has_value())) return false;
	std::vector<double> Heights;
	for(const UnitDirection Direction : Mesh->Directions) Heights.push_back(Ocean->EvaluateElevation(Direction));
	const std::optional<SphericalSplineField> Source = SphericalSplineField::Create(6, Heights);
	// 元補間が無い場合の返戻
	if(!TestTrue(TEXT("元補間を準備"), Source.has_value())) return false;
	const std::optional<DrainageNetwork> Network = DrainageNetwork::CreateForSphericalSurface(*Ocean, *Source);
	// 共有排水が無い場合の返戻
	if(!TestTrue(TEXT("共有排水を準備"), Network.has_value())) return false;
	const bool ShouldFitSource = Parameters.Contains(TEXT("Source"));
	std::optional<DrainageNetwork> PreparedDrainage = Network->WithBalancedTerrainProfiles(
		[&Source, &Ocean, ShouldFitSource](const UnitDirection Direction) -> double {
			// 粗い補間と実際の元場を同じ縦断調整へ渡す比較値の返戻
			return ShouldFitSource ? Ocean->EvaluateElevation(Direction) : Source->Evaluate(Direction).ElevationMeters;
		}
	);
	AddInfo(FString::Printf(TEXT("Spherical relief source-profile fit: enabled=%d"), ShouldFitSource));
	// 平衡縦断が無い場合の返戻
	if(!TestTrue(TEXT("平衡縦断を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const DrainageNetwork> Drainage = std::make_shared<const DrainageNetwork>(std::move(*PreparedDrainage));
	const std::optional<PlanetSphericalDrainage> Before = PlanetSphericalDrainage::Create(Ocean, Drainage);
	PlanetCore::SphericalReliefSettings Settings;
	Settings.RadiusMeters = Definition.RadiusMeters;
	Settings.SeaLevelMeters = Definition.SeaLevelMeters;
	Settings.MinimumSlope = Drainage->Settings().MinimumSlope;
	Settings.ShouldPreserveVertexSlopes = Parameters.EndsWith(TEXT("Slopes"));
	Settings.ShouldRefineEdges = Parameters.EndsWith(TEXT("EdgeSlopes"));
	Settings.ShouldRelaxAddedHeights = Parameters.Contains(TEXT("Relaxed"));
	if(Parameters.Contains(TEXT("Dense"))) Settings.MaximumAddedVertices = 16384;
	Settings.ShouldLimitToDrainageEdges = Settings.ShouldRefineEdges || Parameters.EndsWith(TEXT("WithDrainageSlopes"));
	Settings.TargetErrorMeters *= Definition.RadiusMeters / (Definition.RadiusMeters + 250000);
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const std::optional<PlanetSphericalDrainage> After =
	PlanetSphericalDrainage::CreateWithRefinedRelief(Ocean, Drainage, Settings);
	const double PreparationMilliseconds =
	std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count();
	// 比較する両方の場が無い場合の返戻
	if(!TestTrue(TEXT("追加前後の比較場を準備"), Before.has_value() && After.has_value())) return false;
	double BeforeSquared = 0, AfterSquared = 0, BeforeMaximum = 0, AfterMaximum = 0, MaximumSlope = 0, GlobalBeforeSquared = 0;
	double GlobalAfterSquared = 0, GlobalBeforeMaximum = 0, GlobalAfterMaximum = 0, BeforeMaximumSlope = 0, MaximumLandSlope = 0;
	double SourceSlopeAtMaximum = 0, BaseSlopeAtMaximum = 0, HeightAtMaximumSlope = 0, SourceAtMaximumSlope = 0;
	Vec3 MaximumSlopeDirection;
	std::uint32_t Land = 0, Wet = 0, ChangedWetSigns = 0;
	for(unsigned Index = 0; Index < 65536; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / 65536, Angle = Index * 2.399963229728653, Radius = std::sqrt(1 - Z * Z);
		const UnitDirection Point = *UnitDirection::FromVector({ Radius * std::cos(Angle), Radius * std::sin(Angle), Z });
		const PlanetCore::TerrainSample OriginalSample = Ocean->Evaluate(Point), OldSample = Before->Evaluate(Point);
		const double Original = OriginalSample.ElevationMeters, Old = OldSample.ElevationMeters;
		const PlanetCore::TerrainSample Actual = After->Evaluate(Point);
		const double Current = Actual.ElevationMeters;
		const bool HasFiniteSamples = std::isfinite(Current) && std::isfinite(Original) && std::isfinite(Old) &&
		std::isfinite(Actual.TangentialGradientPerRadian.Length()) && std::isfinite(OldSample.TangentialGradientPerRadian.Length());
		// 非有限値で最大誤差の集計をすり抜けない返戻
		if(!TestTrue(TEXT("独立試料は有限"), HasFiniteSamples)) return false;
		if(Current < Definition.SeaLevelMeters != Old < Definition.SeaLevelMeters) ++ChangedWetSigns;
		if(Current < Definition.SeaLevelMeters) ++Wet;
		const double Slope = Actual.TangentialGradientPerRadian.Length() / (Definition.RadiusMeters + Current);
		const double OldSlope = OldSample.TangentialGradientPerRadian.Length() / (Definition.RadiusMeters + Old);
		BeforeMaximumSlope = std::max(BeforeMaximumSlope, OldSlope);
		if(Current > Definition.SeaLevelMeters) MaximumLandSlope = std::max(MaximumLandSlope, Slope);
		if(Slope > MaximumSlope) {
			MaximumSlope = Slope;
			MaximumSlopeDirection = Point.Vector();
			HeightAtMaximumSlope = Current;
			SourceAtMaximumSlope = Original;
			SourceSlopeAtMaximum = OriginalSample.TangentialGradientPerRadian.Length() / (Definition.RadiusMeters + Original);
			BaseSlopeAtMaximum = OldSlope;
		}
		const double GlobalOldDifference = std::abs(Old - Original), GlobalDifference = std::abs(Current - Original);
		GlobalBeforeSquared += GlobalOldDifference * GlobalOldDifference;
		GlobalAfterSquared += GlobalDifference * GlobalDifference;
		GlobalBeforeMaximum = std::max(GlobalBeforeMaximum, GlobalOldDifference);
		GlobalAfterMaximum = std::max(GlobalAfterMaximum, GlobalDifference);
		if(Original <= Definition.SeaLevelMeters || Old <= Definition.SeaLevelMeters) continue;
		const double OldDifference = std::abs(Old - Original), Difference = std::abs(Current - Original);
		BeforeSquared += OldDifference * OldDifference;
		AfterSquared += Difference * Difference;
		BeforeMaximum = std::max(BeforeMaximum, OldDifference);
		AfterMaximum = std::max(AfterMaximum, Difference);
		++Land;
	}
	if(Definition.RadiusMeters == 1000) {
		const UnitDirection Regression = *UnitDirection::FromVector({ -0.42387724883895705, -0.2520242494741739, -0.8699493408203125 });
		const PlanetCore::TerrainSample Sample = After->Evaluate(Regression);
		const double Slope = Sample.TangentialGradientPerRadian.Length() / (Definition.RadiusMeters + Sample.ElevationMeters);
		TestTrue(TEXT("元傾斜０．１９の地点に人工的な絶壁を作らない"), Slope < 1);
		AddInfo(
			FString::Printf(TEXT("Spherical relief narrow-triangle regression: slope=%.12g height=%.12g"), Slope, Sample.ElevationMeters)
		);
	}
	TestTrue(TEXT("独立陸上試料の RMS を削減"), AfterSquared < 0.81 * BeforeSquared);
	TestTrue(TEXT("最大の起伏再現誤差を悪化させない"), AfterMaximum <= BeforeMaximum + 1E-6);
	TestTrue(TEXT("海の割合は６〜７割"), Wet >= 0.6 * 65536 && Wet <= 0.7 * 65536);
	TestEqual(TEXT("独立試料の海域符号を保持"), ChangedWetSigns, std::uint32_t{ 0 });
	PlanetCore::SphericalWarpSettings WarpSettings;
	WarpSettings.Seed = Definition.Seed;
	WarpSettings.RadiusMeters = Definition.RadiusMeters;
	const std::optional<PlanetCore::PlanetSphericalWarp> Warp = PlanetCore::PlanetSphericalWarp::Create(WarpSettings);
	// 同じ物理尺度の写像を準備出来ない場合の返戻
	if(!TestTrue(TEXT("可逆な球面写像を準備"), Warp.has_value())) return false;
	double MaximumWarpRouteDifference = 0;
	std::size_t WarpedRouteSamples = 0;
	double MaximumUphill = 0, MinimumDrop = std::numeric_limits<double>::infinity();
	std::size_t ProfileSamples = 0;
	const std::span<const PlanetCore::DrainageNode> Nodes = Drainage->Nodes();
	for(const PlanetCore::DrainageNode &Node : Nodes) {
		if(Node.Ocean) continue;
		const Vec3 Start = Node.Direction, End = Nodes[Node.Downstream].Direction;
		double Previous = After->EvaluateElevation(*UnitDirection::FromVector(Start));
		const double First = Previous;
		for(unsigned Step = 1; Step < 33; ++Step) {
			const double Fraction = Step / 32.0;
			const UnitDirection Point = *UnitDirection::FromVector(Start * (1 - Fraction) + End * Fraction);
			const double Height = After->EvaluateElevation(Point);
			if(!(Step % 8)) {
				const UnitDirection WorldPoint = Warp->ToWorld(Point);
				const double WarpedHeight = After->EvaluateElevation(Warp->ToSource(WorldPoint));
				MaximumWarpRouteDifference = std::max(MaximumWarpRouteDifference, std::abs(WarpedHeight - Height));
				++WarpedRouteSamples;
			}
			MaximumUphill = std::max(MaximumUphill, Height - Previous);
			Previous = Height;
			++ProfileSamples;
		}
		MinimumDrop = std::min(MinimumDrop, First - Previous);
	}
	TestTrue(TEXT("追加起伏後も元の全下流辺を下降"), MaximumUphill < 1E-7 && MinimumDrop > 0);
	TestTrue(TEXT("曲げた全河道も同じ順序の下降標高を持つ"), MaximumWarpRouteDifference < 1E-6 && WarpedRouteSamples > 0);
	AddInfo(
		FString::Printf(
			TEXT("Spherical warped drainage: radius=%.12g samples=%llu maximumHeightDifference=%.12g"),
			Definition.RadiusMeters,
			static_cast<unsigned long long>(WarpedRouteSamples),
			MaximumWarpRouteDifference
		)
	);
	PlanetCore::SphericalReliefSettings Wrong = Settings;
	Wrong.RadiusMeters *= 2;
	TestFalse(TEXT("異なる半径の追加設定を拒否"), PlanetSphericalDrainage::CreateWithRefinedRelief(Ocean, Drainage, Wrong).has_value());
	const PlanetCore::SphericalDrainageStatistics &Stats = After->GetStatistics();
	AddInfo(
		FString::Printf(
			TEXT("Spherical relief relaxation: enabled=%d protected=%u changed=%u lowering=%.12g"),
			Settings.ShouldRelaxAddedHeights,
			Stats.Relief.ProtectedAddedVertices,
			Stats.Relief.RelaxedVertices,
			Stats.Relief.MaximumRelaxationMeters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spherical relief conditioning: raise=%.12g cut=%.12g raiseDirection=(%.17g,%.17g,%.17g) ")
			TEXT("cutDirection=(%.17g,%.17g,%.17g) budget=%u"),
			Stats.Relief.MaximumConditioningRaiseMeters,
			Stats.Relief.MaximumConditioningCutMeters,
			Stats.Relief.MaximumRaiseDirection.X,
			Stats.Relief.MaximumRaiseDirection.Y,
			Stats.Relief.MaximumRaiseDirection.Z,
			Stats.Relief.MaximumCutDirection.X,
			Stats.Relief.MaximumCutDirection.Y,
			Stats.Relief.MaximumCutDirection.Z,
			Settings.MaximumAddedVertices
		)
	);
	if(Settings.ShouldPreserveVertexSlopes) TestTrue(TEXT("零接線へ退化せず元勾配を残す"), Stats.Relief.PreservedSlopeVertices > 0);
	AddInfo(
		FString::Printf(
			TEXT("Spherical constrained slopes: enabled=%d retained=%u wetChanges=%u"),
			Settings.ShouldPreserveVertexSlopes,
			Stats.Relief.PreservedSlopeVertices,
			ChangedWetSigns
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spherical relief terrain: radius=%.12g added=%u evaluations=%llu land=%u beforeRms=%.12g afterRms=%.12g ")
			TEXT("beforeMax=%.12g afterMax=%.12g slope=%.12g wet=%.9g uphill=%.12g minDrop=%.12g profiles=%llu ")
			TEXT("conditioningRaise=%.12g remainingPriority=%.12g prepareMs=%.9g bytes=%llu minimumCreatedQuality=%.12g"),
			Definition.RadiusMeters,
			Stats.Relief.AddedVertices,
			static_cast<unsigned long long>(Stats.Relief.SourceEvaluations),
			Land,
			std::sqrt(BeforeSquared / Land),
			std::sqrt(AfterSquared / Land),
			BeforeMaximum,
			AfterMaximum,
			MaximumSlope,
			Wet / 655.36,
			MaximumUphill,
			MinimumDrop,
			static_cast<unsigned long long>(ProfileSamples),
			Stats.Relief.MaximumConditioningRaiseMeters,
			Stats.Relief.LargestRemainingPriorityMeters,
			PreparationMilliseconds,
			static_cast<unsigned long long>(Stats.SplineResidentBytes),
			Stats.Relief.MinimumCreatedTriangleQuality
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Spherical relief global: radius=%.12g beforeRms=%.12g afterRms=%.12g beforeMax=%.12g afterMax=%.12g ")
			TEXT("beforeMaxSlope=%.12g landMaxSlope=%.12g worstDirection=(%.17g,%.17g,%.17g) ")
			TEXT("worstHeight=%.12g worstSourceHeight=%.12g sourceSlope=%.12g oldSlopeAtWorst=%.12g"),
			Definition.RadiusMeters,
			std::sqrt(GlobalBeforeSquared / 65536),
			std::sqrt(GlobalAfterSquared / 65536),
			GlobalBeforeMaximum,
			GlobalAfterMaximum,
			BeforeMaximumSlope,
			MaximumLandSlope,
			MaximumSlopeDirection.X,
			MaximumSlopeDirection.Y,
			MaximumSlopeDirection.Z,
			HeightAtMaximumSlope,
			SourceAtMaximumSlope,
			SourceSlopeAtMaximum,
			BaseSlopeAtMaximum
		)
	);
	// 同一生成条件で測った品質・単調性・保持量の検査結果の返戻
	return !HasAnyErrors();
}

#endif
