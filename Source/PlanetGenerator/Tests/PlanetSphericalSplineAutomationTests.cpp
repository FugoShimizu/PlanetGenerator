#include "Core/PlanetDrainage.h"
#include "Core/PlanetDrainageSampling.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSphericalDrainage.h"
#include "Core/PlanetSphericalSpline.h"
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
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalSplineContracts, "PlanetGenerator.SphericalSpline.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 球面６分割の補間・接合・微分と，頂点からの放射方向に於ける解析的な高低関係を検査する
 * @param Parameters 未使用の UE 引数
 * @return 全ての数値条件を満たす場合に真，排水や景観の完成を意味しない
 */
bool FPlanetSphericalSplineContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::SphericalSplineField;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	TestFalse(TEXT("範囲外の準備規模を拒否"), SphericalSplineField::Create(7, {}).has_value());
	TestFalse(TEXT("欠けた節点値を拒否"), SphericalSplineField::Create(0, {}).has_value());
	std::vector<double> Bad(12, 0);
	Bad[3] = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("非有限な節点値を拒否"), SphericalSplineField::Create(0, Bad).has_value());
	Bad[3] = 0;
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(TEXT("中断された準備を拒否"), SphericalSplineField::Create(0, Bad, Cancelled.get_token()).has_value());
	double MaximumValueError = 0, MaximumJoinValue = 0, MaximumJoinGradient = 0, MaximumDifferenceError = 0, MaximumRadialError = 0;
	double MaximumNodeGradient = 0;
	std::size_t Joins = 0, Samples = 0, LargestBytes = 0;
	for(const std::uint8_t Level : { 0, 2, 4, 6 }) {
		const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(Level);
		// 比較する共有網が無い場合の返戻
		if(!TestTrue(TEXT("共有球面網を準備"), Mesh.has_value())) return false;
		std::vector<double> Heights;
		Heights.reserve(Mesh->Directions.size());
		for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			Heights.push_back(600 * std::sin(static_cast<double>(Index) * 1.618) + 230 * Mesh->Directions[Index].Vector().Z);
		}
		const std::optional<SphericalSplineField> Field = SphericalSplineField::Create(Level, Heights);
		// 構成した補間が無い場合の返戻
		if(!TestTrue(TEXT("零接線の球面二次場を準備"), Field.has_value())) return false;
		LargestBytes = std::max(LargestBytes, Field->GetStatistics().ResidentBytes);
		TestEqual(TEXT("原三角形の６小区画を保持"), Field->GetStatistics().Pieces, static_cast<std::uint32_t>(6 * Mesh->Triangles.size()));
		TestFalse(TEXT("範囲外の診断番号を拒否"), Field->GetPieceDomain(Field->GetStatistics().Pieces).has_value());
		TestFalse(TEXT("範囲外の式照会を拒否"), Field->EvaluatePiece(Field->GetStatistics().Pieces, Mesh->Directions.front()).has_value());
		const double Low = *std::min_element(Heights.begin(), Heights.end()), High = *std::max_element(Heights.begin(), Heights.end());
		for(std::size_t Index = 0; Index < Heights.size(); ++Index) {
			const TerrainSample Actual = Field->Evaluate(Mesh->Directions[Index]);
			MaximumValueError = std::max(MaximumValueError, std::abs(Actual.ElevationMeters - Heights[Index]));
			MaximumNodeGradient = std::max(MaximumNodeGradient, Actual.TangentialGradientPerRadian.Length());
		}
		const std::size_t Stride = std::max<std::size_t>(1, Field->GetStatistics().Pieces / 4096);
		for(std::size_t Index = 0; Index < Field->GetStatistics().Pieces; Index += Stride) {
			const std::optional<std::array<Vec3, 3>> Domain = Field->GetPieceDomain(Index);
			// 実際に構成した小区画を取得出来ない場合の返戻
			if(!TestTrue(TEXT("小区画を復元"), Domain.has_value())) return false;
			const Vec3 A = (*Domain)[0], D = (*Domain)[1], Z = (*Domain)[2];
			const UnitDirection Point = *UnitDirection::FromVector(A * 0.21 + D * 0.33 + Z * 0.46);
			const TerrainSample Actual = Field->Evaluate(Point);
			const TerrainSample Direct = *Field->EvaluatePiece(Index, Point);
			MaximumValueError = std::max(MaximumValueError, std::abs(Actual.ElevationMeters - Direct.ElevationMeters));
			TestTrue(TEXT("頂点範囲を越える補間極値を作らない標本"), Actual.ElevationMeters >= Low - 1E-7 && Actual.ElevationMeters <= High + 1E-7);
			const Vec3 P = Point.Vector();
			const UnitDirection Tangent = *UnitDirection::FromVector(A - P * A.Dot(P));
			constexpr double Step = 1E-6;
			const auto Difference = [&](const double H) -> double {
				const UnitDirection Before = *UnitDirection::FromVector(P * std::cos(H) - Tangent.Vector() * std::sin(H));
				const UnitDirection After = *UnitDirection::FromVector(P * std::cos(H) + Tangent.Vector() * std::sin(H));
				// 点照会を独立に二回行う中央差分の返戻
				return (Field->Evaluate(After).ElevationMeters - Field->Evaluate(Before).ElevationMeters) / (2 * H);
			};
			const double Independent = (4 * Difference(Step * 0.5) - Difference(Step)) / 3;
			MaximumDifferenceError =
			std::max(MaximumDifferenceError, std::abs(Independent - Actual.TangentialGradientPerRadian.Dot(Tangent.Vector())));
			// 原頂点の接線が零なら，頂点からの大円上で H は h0 ＋ k sin²(theta) になる
			const Vec3 Anchor = A;
			const double Cosine = std::clamp(Anchor.Dot(P), -1.0, 1.0);
			const Vec3 Across = P - Anchor * Cosine;
			const double Sine = Across.Length();
			const Vec3 Ray = Across * (1 / Sine);
			const double Angle = std::atan2(Sine, Cosine);
			const UnitDirection Half = *UnitDirection::FromVector(Anchor * std::cos(Angle * 0.5) + Ray * std::sin(Angle * 0.5));
			const double AtAnchor = Field->Evaluate(*UnitDirection::FromVector(Anchor)).ElevationMeters;
			const double Expected = AtAnchor + (Actual.ElevationMeters - AtAnchor) * std::pow(std::sin(Angle * 0.5) / Sine, 2);
			MaximumRadialError = std::max(MaximumRadialError, std::abs(Field->Evaluate(Half).ElevationMeters - Expected));
			++Samples;
		}
		// 全ての共有辺を一度ずつ照合し，根面境界も同じ経路で検査する
		using EdgeKey = std::pair<std::uint32_t, std::uint32_t>;
		using Owner = std::pair<std::size_t, unsigned>;
		std::map<EdgeKey, Owner> Edges;
		const auto Compare = [&](const std::size_t Left, const std::size_t Right, const Vec3 Start, const Vec3 End) -> void {
			for(const double T : { 0.0, 0.13, 0.5, 0.91, 1.0 }) {
				const UnitDirection Point = *UnitDirection::FromVector(Start * (1 - T) + End * T);
				const TerrainSample A = *Field->EvaluatePiece(Left, Point), B = *Field->EvaluatePiece(Right, Point);
				MaximumJoinValue = std::max(MaximumJoinValue, std::abs(A.ElevationMeters - B.ElevationMeters));
				MaximumJoinGradient = std::max(MaximumJoinGradient, (A.TangentialGradientPerRadian - B.TangentialGradientPerRadian).Length());
				++Joins;
			}
			// 終了
			return;
		};
		for(std::size_t Face = 0; Face < Mesh->Triangles.size(); ++Face) {
			const std::array<std::uint32_t, 3> &Triangle = Mesh->Triangles[Face];
			for(unsigned Side = 0; Side < 3; ++Side) {
				const std::array<Vec3, 3> Domain = *Field->GetPieceDomain(6 * Face + 2 * Side);
				// 同じ辺の両端側の小区画は，分割点から内心への辺を共有する
				Compare(6 * Face + 2 * Side, 6 * Face + 2 * Side + 1, Domain[1], Domain[2]);
				// 同じ頂点から内心への辺の両側
				Compare(6 * Face + 2 * Side, 6 * Face + 2 * ((Side + 2) % 3) + 1, Domain[0], Domain[2]);
				const std::uint32_t A = Triangle[Side], B = Triangle[(Side + 1) % 3];
				const EdgeKey Key { std::min(A, B), std::max(A, B) };
				const std::map<EdgeKey, Owner>::const_iterator Found = Edges.find(Key);
				if(Found == Edges.end()) {
					Edges.emplace(Key, Owner{ Face, Side });
					continue;
				}
				const std::size_t OtherFace = Found->second.first;
				const unsigned OtherSide = Found->second.second;
				for(unsigned Half = 0; Half < 2; ++Half) {
					const std::uint32_t Vertex = Triangle[(Side + Half) % 3];
					const unsigned OtherHalf = Mesh->Triangles[OtherFace][OtherSide] == Vertex ? 0 : 1;
					const std::array<Vec3, 3> Part = *Field->GetPieceDomain(6 * Face + 2 * Side + Half);
					Compare(6 * Face + 2 * Side + Half, 6 * OtherFace + 2 * OtherSide + OtherHalf, Part[0], Part[1]);
				}
				Edges.erase(Found);
			}
		}
		TestTrue(TEXT("共有辺が全て対となる"), Edges.empty());
	}
	const std::vector<double> ConstantHeights(162, 1234.5);
	const std::optional<SphericalSplineField> Constant = SphericalSplineField::Create(2, ConstantHeights);
	// 定数再現を準備出来ない場合の返戻
	if(!TestTrue(TEXT("定数場を準備"), Constant.has_value())) return false;
	for(unsigned Index = 0; Index < 1024; ++Index) {
		const UnitDirection Point =
		*UnitDirection::FromVector({ std::sin(Index * 1.7), std::cos(Index * 0.31), std::sin(Index * 0.91) });
		const TerrainSample Sample = Constant->Evaluate(Point);
		TestEqual(TEXT("定数標高を厳密に再現"), Sample.ElevationMeters, 1234.5);
		TestEqual(TEXT("定数の接線は零"), Sample.TangentialGradientPerRadian.Length(), 0.0);
	}
	TestTrue(TEXT("補間と探索の標高が一致"), MaximumValueError <= 1E-7);
	TestTrue(TEXT("原頂点の零接線を保持"), MaximumNodeGradient <= 1E-6);
	TestTrue(TEXT("内部辺・元三角形辺の標高を接続"), MaximumJoinValue <= 1E-7);
	TestTrue(TEXT("球面の共有接線を接続"), MaximumJoinGradient <= 1E-6);
	TestTrue(TEXT("独立差分が解析勾配へ収束"), MaximumDifferenceError <= 0.001);
	TestTrue(TEXT("区画内の放射方向に余分な折返しを作らない"), MaximumRadialError <= 1E-7);
	TestTrue(TEXT("比較用の最大場の保持量が96MiB以内"), LargestBytes <= 96 * 1024 * 1024);
	AddInfo(
		FString::Printf(
			TEXT("Spherical spline: samples=%llu joins=%llu value=%.12g nodeGradient=%.12g ")
			TEXT("joinValue=%.12g joinGradient=%.12g difference=%.12g radial=%.12g bytes=%llu. ")
			TEXT("Interpolation contracts only; no global drainage or visual acceptance."),
			static_cast<unsigned long long>(Samples),
			static_cast<unsigned long long>(Joins),
			MaximumValueError,
			MaximumNodeGradient,
			MaximumJoinValue,
			MaximumJoinGradient,
			MaximumDifferenceError,
			MaximumRadialError,
			static_cast<unsigned long long>(LargestBytes)
		)
	);
	// 球面補間に関する限定検査の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalSplineRefinedMesh, "PlanetGenerator.SphericalSpline.RefinedMesh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 共有辺上の不均等な追加点を使い，索引の被覆・節点再現・不正網の拒否を検査する
 * @param Parameters 未使用の UE 引数
 * @return 元面と追加面の探索が成立し，不正な構造を受入れない場合に真
 */
bool FPlanetSphericalSplineRefinedMesh::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::SphericalSplineField;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(2);
	// 元の閉じた網を構成出来ない場合の返戻
	if(!TestTrue(TEXT("局所分割の基準を準備"), Base.has_value())) return false;
	std::vector<UnitDirection> Directions = Base->Directions;
	std::vector<std::array<std::uint32_t, 3>> Triangles;
	const std::uint32_t A = Base->Triangles[0][0], B = Base->Triangles[0][1], Added = static_cast<std::uint32_t>(Directions.size());
	Directions.push_back(*UnitDirection::FromVector(Directions[A].Vector() * 0.97 + Directions[B].Vector() * 0.03));
	unsigned SplitFaces = 0;
	for(const std::array<std::uint32_t, 3> Triangle : Base->Triangles) {
		bool HasSharedEdge = false;
		for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t Start = Triangle[Side], End = Triangle[(Side + 1) % 3], Opposite = Triangle[(Side + 2) % 3];
			if(!(Start == A && End == B || Start == B && End == A)) continue;
			Triangles.push_back({ Start, Added, Opposite });
			Triangles.push_back({ Added, End, Opposite });
			HasSharedEdge = true;
			++SplitFaces;
			break;
		}
		if(!HasSharedEdge) Triangles.push_back(Triangle);
	}
	TestEqual(TEXT("共有辺の両側だけを分割"), SplitFaces, 2U);
	std::vector<double> Heights;
	for(const UnitDirection Direction : Directions) Heights.push_back(300 * Direction.Vector().Z + 71 * Direction.Vector().X);
	Heights[Added] = 13;
	const std::optional<SphericalSplineField> Field = SphericalSplineField::CreateOnMesh(2, Directions, Triangles, Heights);
	// 有効な局所分割を拒否する場合の返戻
	if(!TestTrue(TEXT("不均等な局所分割を受入れ"), Field.has_value())) return false;
	double MaximumError = 0, MaximumInteriorError = 0;
	for(std::size_t Index = 0; Index < Directions.size(); ++Index) {
		MaximumError = std::max(MaximumError, std::abs(Field->Evaluate(Directions[Index]).ElevationMeters - Heights[Index]));
	}
	for(std::size_t Index = 0; Index < Field->GetStatistics().Pieces; ++Index) {
		const std::array<Vec3, 3> Domain = *Field->GetPieceDomain(Index);
		const UnitDirection Point = *UnitDirection::FromVector(Domain[0] * 0.31 + Domain[1] * 0.29 + Domain[2] * 0.4);
		MaximumInteriorError = std::max(
			MaximumInteriorError,
			std::abs(Field->Evaluate(Point).ElevationMeters - Field->EvaluatePiece(Index, Point)->ElevationMeters)
		);
	}
	using EdgeKey = std::pair<std::uint32_t, std::uint32_t>;
	using Owner = std::pair<std::size_t, unsigned>;
	std::map<EdgeKey, Owner> Edges;
	double MaximumJoinValue = 0, MaximumJoinGradient = 0;
	std::size_t Joins = 0;
	const auto Compare = [&](const std::size_t Left, const std::size_t Right, const Vec3 Start, const Vec3 End) -> void {
		for(const double Fraction : { 0.0, 0.13, 0.5, 0.91, 1.0 }) {
			const UnitDirection Point = *UnitDirection::FromVector(Start * (1 - Fraction) + End * Fraction);
			const PlanetCore::TerrainSample First = *Field->EvaluatePiece(Left, Point), Second = *Field->EvaluatePiece(Right, Point);
			MaximumJoinValue = std::max(MaximumJoinValue, std::abs(First.ElevationMeters - Second.ElevationMeters));
			MaximumJoinGradient =
			std::max(MaximumJoinGradient, (First.TangentialGradientPerRadian - Second.TangentialGradientPerRadian).Length());
			++Joins;
		}
		// 終了
		return;
	};
	for(std::size_t Face = 0; Face < Triangles.size(); ++Face) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::array<Vec3, 3> Domain = *Field->GetPieceDomain(6 * Face + 2 * Side);
		Compare(6 * Face + 2 * Side, 6 * Face + 2 * Side + 1, Domain[1], Domain[2]);
		Compare(6 * Face + 2 * Side, 6 * Face + 2 * ((Side + 2) % 3) + 1, Domain[0], Domain[2]);
		const std::uint32_t Start = Triangles[Face][Side], End = Triangles[Face][(Side + 1) % 3];
		const EdgeKey Key { std::min(Start, End), std::max(Start, End) };
		const std::map<EdgeKey, Owner>::const_iterator Found = Edges.find(Key);
		if(Found == Edges.end()) {
			Edges.emplace(Key, Owner{ Face, Side });
			continue;
		}
		const std::size_t OtherFace = Found->second.first;
		const unsigned OtherSide = Found->second.second;
		for(unsigned Half = 0; Half < 2; ++Half) {
			const std::uint32_t Vertex = Triangles[Face][(Side + Half) % 3];
			const unsigned OtherHalf = Triangles[OtherFace][OtherSide] == Vertex ? 0 : 1;
			const std::array<Vec3, 3> Part = *Field->GetPieceDomain(6 * Face + 2 * Side + Half);
			Compare(6 * Face + 2 * Side + Half, 6 * OtherFace + 2 * OtherSide + OtherHalf, Part[0], Part[1]);
		}
		Edges.erase(Found);
	}
	TestTrue(TEXT("追加点を含む共有辺が全て対"), Edges.empty());
	TestTrue(TEXT("不均等な小区画の値を接続"), MaximumJoinValue <= 1E-7);
	TestTrue(TEXT("不均等な小区画の接線を接続"), MaximumJoinGradient <= 1E-6);
	TestTrue(TEXT("追加点を含む全節点を再現"), MaximumError <= 1E-7);
	TestTrue(TEXT("全小区画の索引が直接評価と一致"), MaximumInteriorError <= 1E-7);
	TestFalse(
		TEXT("欠けた面を拒否"),
		SphericalSplineField::CreateOnMesh(2, Directions, std::span(Triangles).first(Triangles.size() - 1), Heights).has_value()
	);
	const std::array<std::uint32_t, 3> Original = Triangles[0];
	Triangles[0][0] = static_cast<std::uint32_t>(Directions.size());
	TestFalse(TEXT("範囲外の頂点を拒否"), SphericalSplineField::CreateOnMesh(2, Directions, Triangles, Heights).has_value());
	Triangles[0] = { Original[1], Original[0], Original[2] };
	TestFalse(TEXT("反転した共有面を拒否"), SphericalSplineField::CreateOnMesh(2, Directions, Triangles, Heights).has_value());
	Triangles[0] = Triangles[1];
	TestFalse(TEXT("面の重複で穴を隠さない"), SphericalSplineField::CreateOnMesh(2, Directions, Triangles, Heights).has_value());
	Triangles[0] = Original;
	TestFalse(TEXT("探索階層の境界を跨ぐ面を拒否"), SphericalSplineField::CreateOnMesh(3, Directions, Triangles, Heights).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Refined spherical spline: vertices=%llu pieces=%u nodeError=%.12g interiorError=%.12g ")
			TEXT("joins=%llu joinValue=%.12g joinGradient=%.12g."),
			static_cast<unsigned long long>(Directions.size()),
			Field->GetStatistics().Pieces,
			MaximumError,
			MaximumInteriorError,
			static_cast<unsigned long long>(Joins),
			MaximumJoinValue,
			MaximumJoinGradient
		)
	);
	// 正規経路と不正入力の両方を検査した結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FPlanetSphericalSplineDrainage, "PlanetGenerator.SphericalSpline.Drainage",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 最小半径・基準半径・地球半径・大きな座標の条件を列挙する
 * @param OutBeautifiedNames 表示する条件名
 * @param OutTestCommands 検査本体へ渡す同じ条件名
 */
void FPlanetSphericalSplineDrainage::GetTests(TArray<FString> &OutBeautifiedNames, TArray<FString> &OutTestCommands) const {
	const std::array<FString, 4> Names { TEXT("MinimumRadius"), TEXT("ReferenceRadius"), TEXT("EarthRadius"), TEXT("LargeRadius") };
	for(const FString &Name : Names) {
		OutBeautifiedNames.Add(Name);
		OutTestCommands.Add(Name);
	}
	// 終了
	return;
}

/**
 * 実惑星の共有縦断を零接線補間へ渡し，連続な下流辺と元地形からの変更量を比較する
 * @param Parameters 列挙した半径と Seed の条件
 * @return 排水辺の構造と数値条件が成立する場合に真，景観と最終河川の受入れではない
 */
bool FPlanetSphericalSplineDrainage::RunTest(const FString &Parameters) {
	using PlanetCore::GeologyField;
	using PlanetCore::GeologySettings;
	using PlanetCore::OceanConnectivityField;
	using PlanetCore::PlanetOrogenicRidges;
	using PlanetCore::DrainageNetwork;
	using PlanetCore::SphericalSplineField;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	using Clock = std::chrono::steady_clock;
	const Clock::time_point Started = Clock::now();
	GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	if(Parameters == TEXT("MinimumRadius")) Settings.RadiusMeters = 1000;
	else if(Parameters == TEXT("EarthRadius")) {
		Settings.Seed = 7;
		Settings.RadiusMeters = 6371000;
	} else if(Parameters == TEXT("LargeRadius")) Settings.RadiusMeters = 1000000000;
	else if(Parameters != TEXT("ReferenceRadius")) {
		AddError(TEXT("Unknown spherical drainage case."));
		// 未定義の条件を基準惑星の成功で代用しない返戻
		return false;
	}
	AddInfo(
		FString::Printf(
			TEXT("Spherical drainage case: seed=%llu radius=%.9gm."),
			static_cast<unsigned long long>(Settings.Seed),
			Settings.RadiusMeters
		)
	);
	const std::optional<GeologyField> PreparedGeology = GeologyField::Create(Settings);
	// 比較の基盤地質を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実惑星の地質を準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const GeologyField> Geology = std::make_shared<const GeologyField>(*PreparedGeology);
	const std::optional<PlanetOrogenicRidges> PreparedRidges = PlanetOrogenicRidges::Create(*Geology);
	// 造山を排水より先に決められない場合の返戻
	if(!TestTrue(TEXT("実惑星の造山場を準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetOrogenicRidges> Ridges = std::make_shared<const PlanetOrogenicRidges>(*PreparedRidges);
	const std::optional<OceanConnectivityField> Ocean = OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		Ridges,
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 同じ細部を排水前に含められない場合の返戻
	if(!TestTrue(TEXT("早期起伏と接続海域を準備"), Ocean.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(6);
	// 海域と同じ元網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("海域と排水の共有網を準備"), Base.has_value())) return false;
	std::vector<double> SourceHeights;
	SourceHeights.reserve(Base->Directions.size());
	for(const UnitDirection Direction : Base->Directions) SourceHeights.push_back(Ocean->EvaluateElevation(Direction));
	const std::optional<SphericalSplineField> Source = SphericalSplineField::Create(6, SourceHeights);
	// 排水前の連続面を構成出来ない場合の返戻
	if(!TestTrue(TEXT("海岸と排水で同じ補間面を準備"), Source.has_value())) return false;
	std::stop_source CancelledSurface;
	CancelledSurface.request_stop();
	TestFalse(
		TEXT("中断した同一面の排水を拒否"),
		DrainageNetwork::CreateForSphericalSurface(*Ocean, *Source, CancelledSurface.get_token()).has_value()
	);
	std::vector<double> DifferentHeights = SourceHeights;
	++DifferentHeights[0];
	const std::optional<SphericalSplineField> DifferentSurface = SphericalSplineField::Create(6, DifferentHeights);
	// 不一致の検査入力を構成出来ない場合の返戻
	if(!TestTrue(TEXT("別の元標高を持つ場を準備"), DifferentSurface.has_value())) return false;
	TestFalse(TEXT("元標高が一致しない補間を拒否"), DrainageNetwork::CreateForSphericalSurface(*Ocean, *DifferentSurface).has_value());
	const std::optional<DrainageNetwork> Network = DrainageNetwork::CreateForSphericalSurface(*Ocean, *Source);
	// 同じ基準の下流を持たない場合の返戻
	if(!TestTrue(TEXT("大洋への排水木を準備"), Network.has_value())) return false;
	const std::optional<DrainageNetwork> Balanced = Network->WithBalancedTerrainProfiles(
		[&](const UnitDirection Direction) -> double {
			// 縦断と同じ元地形の標高の返戻
			return Source->Evaluate(Direction).ElevationMeters;
		}
	);
	// 全節点の共有縦断を準備出来ない場合の返戻
	if(!TestTrue(TEXT("全節点の共有縦断を準備"), Balanced.has_value())) return false;
	// 元の全節点を含まない排水を別の幾何へ対応させない返戻
	if(!TestTrue(TEXT("排水網が元球面の全節点を保持"), Base && Balanced->Nodes().size() >= Base->Directions.size())) return false;
	std::vector<UnitDirection> Outlets;
	for(std::size_t Index = Base->Directions.size(); Index < Balanced->Nodes().size(); ++Index) {
		const PlanetCore::DrainageNode &Node = Balanced->Nodes()[Index];
		// 追加節点を全て河口と見做す前提が異なる場合の返戻
		if(!TestTrue(TEXT("追加節点は海岸出口"), Node.Ocean && Node.Downstream == Index)) return false;
		Outlets.push_back(*UnitDirection::FromVector(Node.Direction));
	}
	const std::optional<PlanetCore::DrainageSamplingMesh> Mesh = PlanetCore::BuildDrainageSamplingMesh(Outlets);
	// 海岸節点を省略した網で検査を進めない返戻
	if(!TestTrue(TEXT("河口を含む共有球面網を準備"), Mesh.has_value())) return false;
	std::vector<std::uint32_t> NodeToMesh(Balanced->Nodes().size());
	for(std::uint32_t Index = 0; Index < NodeToMesh.size(); ++Index) {
		NodeToMesh[Index] = Index < Base->Directions.size() ? Index : Mesh->SampleNodes[Index - Base->Directions.size()];
	}
	const double Sea = Balanced->Settings().SeaLevelMeters, Freeboard = Balanced->Settings().BankFreeboardMeters;
	std::vector<double> Heights(Mesh->Directions.size(), std::numeric_limits<double>::quiet_NaN());
	double MaximumRaise = 0, MaximumCut = 0, SquareChange = 0;
	for(std::size_t Index = 0; Index < Balanced->Nodes().size(); ++Index) {
		const PlanetCore::DrainageNode &Node = Balanced->Nodes()[Index];
		const std::uint32_t Vertex = NodeToMesh[Index];
		// 別の幾何を同じ節点番号として扱わない返戻
		if(!TestTrue(TEXT("節点の方向が一致"), (Node.Direction - Mesh->Directions[Vertex].Vector()).Length() <= 1E-12)) return false;
		const double Head = std::max(0.0, Node.WaterMeters - Sea);
		const double Height = Node.Ocean ?
		Index < Base->Directions.size() ? Node.OriginalElevationMeters : Node.WaterMeters - Node.DepthMeters :
		Node.WaterMeters + Freeboard * Head / (Head + Freeboard);
		// 同一点の複数出口が矛盾する標高を渡さない返戻
		if(std::isfinite(Heights[Vertex]) && !TestEqual(TEXT("重複節点の標高が一致"), Heights[Vertex], Height)) return false;
		Heights[Vertex] = Height;
		const double Change = Height - Node.OriginalElevationMeters;
		MaximumRaise = std::max(MaximumRaise, Change);
		MaximumCut = std::max(MaximumCut, -Change);
		SquareChange += Change * Change;
	}
	double MinimumOutletDepth = std::numeric_limits<double>::infinity(), MaximumOutletBedDifference = 0;
	for(std::size_t Index = Base->Directions.size(); Index < Balanced->Nodes().size(); ++Index) {
		const PlanetCore::DrainageNode &Node = Balanced->Nodes()[Index];
		TestEqual(TEXT("河口の水位は一定海面を維持"), Node.WaterMeters, Sea);
		MinimumOutletDepth = std::min(MinimumOutletDepth, Sea - Heights[NodeToMesh[Index]]);
	}
	for(std::size_t Index = 0; Index < Balanced->Reaches().size(); ++Index) {
		const PlanetCore::RiverReach &Reach = Balanced->Reaches()[Index];
		if(Reach.Downstream < Base->Directions.size()) continue;
		const std::optional<PlanetCore::RiverHeights> End = Balanced->SampleHeights(Index, 1);
		// 河口の縦断標高を取得出来ない場合の返戻
		if(!TestTrue(TEXT("河口の既存縦断を参照"), End.has_value())) return false;
		MaximumOutletBedDifference =
		std::max(MaximumOutletBedDifference, std::abs(Heights[NodeToMesh[Reach.Downstream]] - End->BedMeters));
	}
	TestTrue(TEXT("河口の地形は海面より水深分だけ低い"), MinimumOutletDepth > 0);
	TestTrue(TEXT("河口河床は既存縦断の終端と一致"), MaximumOutletBedDifference <= 1E-7);
	const Clock::time_point BeforeSpline = Clock::now();
	const std::optional<SphericalSplineField> Field =
	SphericalSplineField::CreateOnMesh(6, Mesh->Directions, Mesh->Triangles, Heights);
	// 節点から連続面を作れない場合の返戻
	if(!TestTrue(TEXT("実惑星の連続補間を準備"), Field.has_value())) return false;
	const Clock::time_point Prepared = Clock::now();
	const std::shared_ptr<const OceanConnectivityField> SharedOcean = std::make_shared<const OceanConnectivityField>(*Ocean);
	const std::shared_ptr<const DrainageNetwork> SharedDrainage = std::make_shared<const DrainageNetwork>(*Balanced);
	const Clock::time_point BeforeComposition = Clock::now();
	const std::optional<PlanetCore::PlanetSphericalDrainage> Terrain =
	PlanetCore::PlanetSphericalDrainage::Create(SharedOcean, SharedDrainage);
	// 正規の共有場を受理出来ない場合の返戻
	if(!TestTrue(TEXT("海底を含む共有地形場を準備"), Terrain.has_value())) return false;
	const double CompositionMilliseconds = std::chrono::duration<double, std::milli>(Clock::now() - BeforeComposition).count();
	const double BlendStart = Terrain->GetStatistics().SeabedBlendStartMeters;
	const double MinimumBlendWidth = Terrain->GetStatistics().SeabedMinimumBlendWidthMeters;
	const double DifferenceStep = std::min(0.1 / Settings.RadiusMeters, 1E-6);
	TestFalse(TEXT("欠けた元海域を拒否"), PlanetCore::PlanetSphericalDrainage::Create({}, SharedDrainage).has_value());
	TestFalse(TEXT("欠けた縦断を拒否"), PlanetCore::PlanetSphericalDrainage::Create(SharedOcean, {}).has_value());
	TestFalse(
		TEXT("開削専用の未平衡縦断を拒否"),
		PlanetCore::PlanetSphericalDrainage::Create(SharedOcean, std::make_shared<const DrainageNetwork>(*Network)).has_value()
	);
	std::stop_source CancelComposition;
	CancelComposition.request_stop();
	TestFalse(
		TEXT("地形場準備の中断を拒否"),
		PlanetCore::PlanetSphericalDrainage::Create(SharedOcean, SharedDrainage, CancelComposition.get_token()).has_value()
	);
	TestEqual(TEXT("同じ全節点を共有地形場へ保持"), Terrain->GetStatistics().Nodes, static_cast<std::uint32_t>(Heights.size()));
	using EdgeKey = std::pair<std::uint32_t, std::uint32_t>;
	std::map<EdgeKey, Vec3> Splits;
	for(std::size_t Face = 0; Face < Mesh->Triangles.size(); ++Face) {
		const std::array<std::uint32_t, 3> &Triangle = Mesh->Triangles[Face];
		for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Triangle[Side], B = Triangle[(Side + 1) % 3];
			const EdgeKey Key { std::min(A, B), std::max(A, B) };
			if(!Splits.contains(Key)) Splits.emplace(Key, (*Field->GetPieceDomain(6 * Face + 2 * Side))[1]);
		}
	}
	std::size_t Land = 0, Profiles = 0;
	double MaximumRouteChange = 0, MaximumUphill = 0, MaximumPositiveSlope = 0;
	double MinimumDrop = std::numeric_limits<double>::infinity();
	for(std::uint32_t Index = 0; Index < Balanced->Nodes().size(); ++Index) {
		const PlanetCore::DrainageNode &Node = Balanced->Nodes()[Index];
		if(Node.Ocean) continue;
		++Land;
		const std::uint32_t Down = Node.Downstream;
		const std::uint32_t StartVertex = NodeToMesh[Index], EndVertex = NodeToMesh[Down];
		const EdgeKey Key { std::min(StartVertex, EndVertex), std::max(StartVertex, EndVertex) };
		const std::map<EdgeKey, Vec3>::const_iterator Found = Splits.find(Key);
		// 実際の共有辺ではない下流経路を通過扱いにしない返戻
		if(!TestTrue(TEXT("下流が実際の共有辺を持つ"), Found != Splits.end())) return false;
		const PlanetCore::DrainageNode &Parent = Balanced->Nodes()[Down];
		TestTrue(TEXT("海までの残距離が下流で減少"), Parent.DistanceToOceanMeters < Node.DistanceToOceanMeters);
		const Vec3 A = Node.Direction, B = Parent.Direction, Mid = Found->second;
		const double Top = Field->Evaluate(*UnitDirection::FromVector(A)).ElevationMeters;
		const double Middle = Field->Evaluate(*UnitDirection::FromVector(Mid)).ElevationMeters;
		const double Bottom = Field->Evaluate(*UnitDirection::FromVector(B)).ElevationMeters;
		MinimumDrop = std::min(MinimumDrop, Top - Bottom);
		TestTrue(TEXT("二つの二次区間がそれぞれ厳密に下流へ下降"), Top > Middle && Middle > Bottom);
		// 各半辺は h_a ＋ k sin²(theta) なので，正の区間長と端点の順が区間全体の符号を決める
		double Previous = Top;
		for(unsigned Step = 1; Step < 33; ++Step) {
			const double T = static_cast<double>(Step) / 32;
			const UnitDirection Direction = *UnitDirection::FromVector(A * (1 - T) + B * T);
			const TerrainSample Sample = Field->Evaluate(Direction);
			MaximumRouteChange = std::max(MaximumRouteChange, std::abs(Terrain->EvaluateElevation(Direction) - Sample.ElevationMeters));
			const Vec3 Point = Direction.Vector();
			const UnitDirection Tangent = *UnitDirection::FromVector(B - A - Point * Point.Dot(B - A));
			MaximumUphill = std::max(MaximumUphill, Sample.ElevationMeters - Previous);
			MaximumPositiveSlope = std::max(MaximumPositiveSlope, Sample.TangentialGradientPerRadian.Dot(Tangent.Vector()));
			Previous = Sample.ElevationMeters;
			++Profiles;
		}
	}
	std::vector<std::uint32_t> OceanParents(Heights.size());
	std::iota(OceanParents.begin(), OceanParents.end(), 0U);
	const auto FindOceanRoot = [&](std::uint32_t Index) -> std::uint32_t {
		while(OceanParents[Index] != Index) {
			OceanParents[Index] = OceanParents[OceanParents[Index]];
			Index = OceanParents[Index];
		}
		// 接続した水没節点の代表番号の返戻
		return Index;
	};
	for(const std::array<std::uint32_t, 3> Triangle : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::uint32_t A = Triangle[Side], B = Triangle[(Side + 1) % 3];
		if(Heights[A] < Sea && Heights[B] < Sea) OceanParents[FindOceanRoot(A)] = FindOceanRoot(B);
	}
	std::size_t WetComponents = 0, UnconnectedOutlets = 0;
	std::vector<bool> HasWetNeighbor(Heights.size(), false);
	for(const std::array<std::uint32_t, 3> Triangle : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::uint32_t A = Triangle[Side], B = Triangle[(Side + 1) % 3];
		if(Heights[A] < Sea) HasWetNeighbor[B] = true;
		if(Heights[B] < Sea) HasWetNeighbor[A] = true;
	}
	for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
		if(Heights[Index] < Sea && FindOceanRoot(Index) == Index) ++WetComponents;
	}
	for(const std::uint32_t Outlet : Mesh->SampleNodes) {
		if(HasWetNeighbor[Outlet]) continue;
		++UnconnectedOutlets;
		const Vec3 Point = Mesh->Directions[Outlet].Vector();
		AddInfo(
			FString::Printf(
				TEXT("Unconnected spline outlet: vertex=%u direction=(%.17g,%.17g,%.17g) source=%.12gm."),
				Outlet,
				Point.X,
				Point.Y,
				Point.Z,
				Ocean->EvaluateElevation(Mesh->Directions[Outlet])
			)
		);
		std::vector<std::uint32_t> Neighbors;
		for(const std::array<std::uint32_t, 3> Triangle : Mesh->Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			if(Triangle[Side] != Outlet) continue;
			Neighbors.push_back(Triangle[(Side + 1) % 3]);
			Neighbors.push_back(Triangle[(Side + 2) % 3]);
		}
		std::sort(Neighbors.begin(), Neighbors.end());
		Neighbors.erase(std::unique(Neighbors.begin(), Neighbors.end()), Neighbors.end());
		for(const std::uint32_t Neighbor : Neighbors) {
			const Vec3 Other = Mesh->Directions[Neighbor].Vector();
			double MinimumSource = std::numeric_limits<double>::infinity(), MinimumFraction = 0;
			for(unsigned Step = 1; Step < 65; ++Step) {
				const double Fraction = static_cast<double>(Step) / 64;
				const UnitDirection Direction = *UnitDirection::FromVector(Point * (1 - Fraction) + Other * Fraction);
				const double Height = Ocean->EvaluateElevation(Direction);
				if(Height < MinimumSource) {
					MinimumSource = Height;
					MinimumFraction = Fraction;
				}
			}
			AddInfo(
				FString::Printf(
					TEXT("Spline outlet neighbor: outlet=%u neighbor=%u baseHeight=%.12gm source=%.12gm ")
					TEXT("edgeMinimum=%.12gm fraction=%.9g direction=(%.17g,%.17g,%.17g)."),
					Outlet,
					Neighbor,
					Heights[Neighbor],
					Ocean->EvaluateElevation(Mesh->Directions[Neighbor]),
					MinimumSource,
					MinimumFraction,
					Other.X,
					Other.Y,
					Other.Z
				)
			);
		}
	}
	std::size_t Wet = 0;
	std::size_t ChangedWetSigns = 0, DeepSamples = 0, TransitionSamples = 0;
	double MaximumWetDifference = 0, MaximumLandDifference = 0, MaximumDeepDifference = 0, MaximumScalarDifference = 0;
	double MaximumTransitionAngle = 0, MaximumComposedSlope = 0, MaximumDifference = 0, SquareDifference = 0, MaximumSlope = 0;
	double Checksum = 0;
	constexpr unsigned Samples = 65536;
	for(unsigned Index = 0; Index < Samples; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / Samples, Phi = Index * 2.399963229728653, Ring = std::sqrt(std::max(0.0, 1 - Z * Z));
		const UnitDirection Direction = *UnitDirection::FromVector({ Ring * std::cos(Phi), Ring * std::sin(Phi), Z });
		const TerrainSample Sample = Field->Evaluate(Direction);
		const double Difference = Sample.ElevationMeters - Ocean->EvaluateElevation(Direction);
		MaximumDifference = std::max(MaximumDifference, std::abs(Difference));
		SquareDifference += Difference * Difference;
		MaximumSlope = std::max(MaximumSlope, Sample.TangentialGradientPerRadian.Length() / Settings.RadiusMeters);
		if(Sample.ElevationMeters < Sea) ++Wet;
		Checksum += Sample.ElevationMeters;
		const TerrainSample Composed = Terrain->Evaluate(Direction);
		const double SourceHeight = Ocean->EvaluateElevation(Direction);
		const double ComposedDifference = std::abs(Composed.ElevationMeters - SourceHeight);
		if(Sample.ElevationMeters < Sea != Composed.ElevationMeters < Sea) ++ChangedWetSigns;
		if(Sample.ElevationMeters < Sea) MaximumWetDifference = std::max(MaximumWetDifference, ComposedDifference);
		else MaximumLandDifference = std::max(MaximumLandDifference, ComposedDifference);
		MaximumScalarDifference =
		std::max(MaximumScalarDifference, std::abs(Terrain->EvaluateElevation(Direction) - Composed.ElevationMeters));
		MaximumComposedSlope = std::max(MaximumComposedSlope, Composed.TangentialGradientPerRadian.Length() / Settings.RadiusMeters);
		// 和は二乗和平方根以上なので，可変帯の外を独立な十分条件で選ぶ
		const double GuaranteedDeep = BlendStart + MinimumBlendWidth + std::abs(SourceHeight - Sample.ElevationMeters);
		if(Sample.ElevationMeters < Sea - GuaranteedDeep && SourceHeight < Sea - GuaranteedDeep) {
			++DeepSamples;
			MaximumDeepDifference = std::max(MaximumDeepDifference, ComposedDifference);
		}
		const bool IsTransition =
		Sample.ElevationMeters < Sea - BlendStart && SourceHeight < Sea - BlendStart && ComposedDifference > 1E-7;
		if(!IsTransition || TransitionSamples > 255) continue;
		++TransitionSamples;
		const Vec3 Point = Direction.Vector(), Axis = std::abs(Point.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 };
		const Vec3 First = UnitDirection::FromVector(Axis.Cross(Point))->Vector(), Second = Point.Cross(First);
		for(const Vec3 Tangent : { First, Second }) {
			const auto DifferenceAt = [&](const double Step) -> double {
				const UnitDirection Before = *UnitDirection::FromVector(Point * std::cos(Step) - Tangent * std::sin(Step));
				const UnitDirection After = *UnitDirection::FromVector(Point * std::cos(Step) + Tangent * std::sin(Step));
				// 値だけの独立な点照会に依る中央差分の返戻
				return (Terrain->EvaluateElevation(After) - Terrain->EvaluateElevation(Before)) / (2 * Step);
			};
			const double DifferenceGradient = (4 * DifferenceAt(DifferenceStep * 0.5) - DifferenceAt(DifferenceStep)) / 3;
			const double ActualGradient = Composed.TangentialGradientPerRadian.Dot(Tangent);
			MaximumTransitionAngle = std::max(
				MaximumTransitionAngle,
				std::abs(std::atan(DifferenceGradient / Settings.RadiusMeters) - std::atan(ActualGradient / Settings.RadiusMeters))
			);
		}
	}
	TestTrue(TEXT("海域推定が６～７割"), Wet * 10 >= Samples * 6 && Wet * 10 <= Samples * 7);
	TestEqual(TEXT("海底復元で水没領域の符号を変えない"), ChangedWetSigns, std::size_t{ 0 });
	TestTrue(TEXT("海底復元の深海標本を実際に検査"), DeepSamples > 0);
	TestTrue(TEXT("海底復元の遷移標本を実際に検査"), TransitionSamples > 0);
	TestTrue(TEXT("深い場所の元海底を保持"), MaximumDeepDifference <= 1E-7);
	TestTrue(TEXT("下流の陸・河口の経路を保持"), MaximumRouteChange <= 1E-7);
	TestTrue(TEXT("値だけの照会と微分付き照会が一致"), MaximumScalarDifference <= 1E-7);
	TestTrue(TEXT("海底復元の連鎖微分が独立差分に一致"), MaximumTransitionAngle <= 1E-6);
	if(Settings.RadiusMeters == 1000 || Settings.RadiusMeters == 1000000) {
		const Vec3 Failed = Settings.RadiusMeters == 1000 ?
		Vec3{ 0.78471469978649544, -0.15209987489898105, 0.60090637207031272 } :
		Vec3{ -0.032578233628146616, -0.11698254782719268, -0.99259948730468761 };
		const UnitDirection Point = *UnitDirection::FromVector(Failed);
		const TerrainSample Fixed = Terrain->Evaluate(Point), Original = Ocean->Evaluate(Point);
		const double Slope = Fixed.TangentialGradientPerRadian.Length() / (Settings.RadiusMeters + Fixed.ElevationMeters);
		TestTrue(TEXT("標高差の大きい海底接続に人工的な絶壁を作らない"), Slope < 1.5);
		AddInfo(
			FString::Printf(
				TEXT("Spherical seabed regression: radius=%.12g slope=%.12g height=%.12g source=%.12g"),
				Settings.RadiusMeters,
				Slope,
				Fixed.ElevationMeters,
				Original.ElevationMeters
			)
		);
	}
	AddInfo(
		FString::Printf(
			TEXT("Spherical drainage composition: prepare=%.6fms changedWetSigns=%llu deepSamples=%llu transitions=%llu ")
			TEXT("routeChange=%.12gm deepDifference=%.12gm scalarDifference=%.12gm transitionAngle=%.12grad ")
			TEXT("wetDifference=%.12gm landDifference=%.12gm maximumSlope=%.12g."),
			CompositionMilliseconds,
			static_cast<unsigned long long>(ChangedWetSigns),
			static_cast<unsigned long long>(DeepSamples),
			static_cast<unsigned long long>(TransitionSamples),
			MaximumRouteChange,
			MaximumDeepDifference,
			MaximumScalarDifference,
			MaximumTransitionAngle,
			MaximumWetDifference,
			MaximumLandDifference,
			MaximumComposedSlope
		)
	);
	TestTrue(TEXT("下流辺の内部に上り返しが無い"), MaximumUphill <= 1E-8);
	TestTrue(TEXT("下流方向の解析勾配が非正"), MaximumPositiveSlope <= 1E-6);
	TestTrue(TEXT("準備元の大洋が接続"), Ocean->Statistics().ComponentsAfter == 1);
	AddInfo(
		FString::Printf(
			TEXT("Spherical spline drainage: nodes=%llu land=%llu profiles=%llu minimumDrop=%.12gm uphill=%.12gm ")
			TEXT("positiveSlope=%.12gm/rad nodalRaise=%.9gm nodalCut=%.9gm nodalRms=%.9gm ")
			TEXT("sampleDifference=%.9gm sampleRms=%.9gm sea=%.9g%% slope=%.9g totalPreparation=%.6fms spline=%.6fms ")
			TEXT("bytes=%llu checksum=%.12g wetComponents=%llu outletsWithoutWetNeighbor=%llu ")
			TEXT("minimumOutletDepth=%.12gm outletBedDifference=%.12gm. ")
			TEXT("Base interpolation only; original shore/river geometry and appearance not accepted."),
			static_cast<unsigned long long>(Heights.size()),
			static_cast<unsigned long long>(Land),
			static_cast<unsigned long long>(Profiles),
			MinimumDrop,
			MaximumUphill,
			MaximumPositiveSlope,
			MaximumRaise,
			MaximumCut,
			std::sqrt(SquareChange / Balanced->Nodes().size()),
			MaximumDifference,
			std::sqrt(SquareDifference / Samples),
			100.0 * Wet / Samples,
			MaximumSlope,
			std::chrono::duration<double, std::milli>(Prepared - Started).count(),
			std::chrono::duration<double, std::milli>(Prepared - BeforeSpline).count(),
			static_cast<unsigned long long>(Field->GetStatistics().ResidentBytes),
			Checksum,
			static_cast<unsigned long long>(WetComponents),
			static_cast<unsigned long long>(UnconnectedOutlets),
			MinimumOutletDepth,
			MaximumOutletBedDifference
		)
	);
	TestEqual(TEXT("補間元の水没節点が一つの成分"), WetComponents, std::size_t{ 1 });
	TestEqual(TEXT("全河口が負標高の面へ接続"), UnconnectedOutlets, std::size_t{ 0 });
	// 実惑星の共有節点と連続辺に関する比較結果の返戻
	return !HasAnyErrors();
}

#endif
