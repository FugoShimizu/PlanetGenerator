#include "Core/PlanetSphericalRefinement.h"
#include "Core/PlanetGradientProjection.h"
#include "Core/PlanetSphericalSpline.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalSlopeContracts, "PlanetGenerator.SphericalSpline.ConstrainedSlopes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 元勾配の制限を，海岸・不規則網・二次式の全区間単調性と接合で検査する
 * @param Parameters 未使用の UE 引数
 * @return 全検査に成功した場合に真，面内部からの流出や景観の受入れとは別
 */
bool FPlanetSphericalSlopeContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::PlanetSphericalRefinement;
	using PlanetCore::SphericalSplineField;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	using Triangle = std::array<std::uint32_t, 3>;
	using Edge = std::pair<std::uint32_t, std::uint32_t>;
	using Owner = std::pair<std::size_t, unsigned>;
	const std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(2);
	// 元網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("元網を準備"), Base.has_value())) return false;
	const Triangle Focus = Base->Triangles[17];
	const UnitDirection Center = *UnitDirection::FromVector(
		Base->Directions[Focus[0]].Vector() + Base->Directions[Focus[1]].Vector() + Base->Directions[Focus[2]].Vector()
	);
	const auto Source = [Center](const UnitDirection Direction) -> TerrainSample {
		const Vec3 P = Direction.Vector(), Difference = P - Center.Vector();
		const double Gaussian = 500 * std::exp(-Difference.Dot(Difference) / (2 * 0.045 * 0.045));
		const Vec3 Ambient = Vec3{ 0, 0, 200 } - Difference * (Gaussian / (0.045 * 0.045));
		// 海岸と節点間の山頂を含む独立な解析場の返戻
		return { 80 + 200 * P.Z + Gaussian, Ambient - P * P.Dot(Ambient) };
	};
	std::vector<double> Heights;
	for(const UnitDirection Direction : Base->Directions) Heights.push_back(Source(Direction).ElevationMeters);
	PlanetCore::SphericalReliefSettings Settings;
	Settings.TargetErrorMeters = 0.5;
	Settings.MaximumAddedVertices = 512;
	const std::optional<PlanetSphericalRefinement> Refined = PlanetSphericalRefinement::Create(
		Base->Directions,
		Base->Triangles,
		Heights,
		[&Source](const UnitDirection Direction) -> double {
			// 同じ解析場の標高を細分化へ渡す返戻
			return Source(Direction).ElevationMeters;
		},
		Settings
	);
	// 不規則な検査網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("内部起伏を含む網を準備"), Refined.has_value() && Refined->GetStatistics().AddedVertices > 0)) return false;
	const double Sea = Settings.SeaLevelMeters;
	const std::span<const std::uint32_t> AddedRoutes = Refined->GetDownstreamVertices();
	std::vector<std::uint32_t> Required(AddedRoutes.begin(), AddedRoutes.end());
	for(const Triangle &Face : Refined->GetTriangles()) for(unsigned Side = 0; Side < 3; ++Side) {
		const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
		if(Refined->GetElevations()[B] < Refined->GetElevations()[Required[A]]) Required[A] = B;
	}
	double MaximumReverseDerivative = 0, MaximumOutsideHeight = 0, MaximumJoin = 0, MaximumGradientJoin = 0, MaximumGrowth = 0;
	double MaximumRadial = 0, MaximumNegativeCoefficient = 0;
	std::uint64_t Intervals = 0, Retained = 0, CoastVertices = 0;
	for(const bool ShouldUseRoutes : { false, true }) for(const double Strength : { 1.0, 100.0, -100.0 }) {
		std::vector<Vec3> Gradients;
		for(const UnitDirection Direction : Refined->GetDirections()) {
			Gradients.push_back(Source(Direction).TangentialGradientPerRadian * Strength);
		}
		const std::optional<std::vector<Vec3>> Limited =
		ShouldUseRoutes ? Refined->ConstrainDrainageGradients(Gradients, 0, Required) : Refined->ConstrainGradients(Gradients, 0);
		// 正規の制限経路を準備出来ない場合の返戻
		if(!TestTrue(TEXT("元勾配を制限"), Limited.has_value())) return false;
		const std::optional<SphericalSplineField> Field = SphericalSplineField::CreateOnMeshWithGradients(
			2,
			Refined->GetDirections(),
			Refined->GetTriangles(),
			Refined->GetElevations(),
			*Limited
		);
		// 制限済の接線を補間出来ない場合の返戻
		if(!TestTrue(TEXT("制限済の面を準備"), Field.has_value())) return false;
		for(std::size_t Vertex = 0; Vertex < Gradients.size(); ++Vertex) {
			const Vec3 G = (*Limited)[Vertex];
			TestTrue(TEXT("制限結果は有限"), std::isfinite(G.X) && std::isfinite(G.Y) && std::isfinite(G.Z));
			MaximumGrowth = std::max(MaximumGrowth, G.Length() - Gradients[Vertex].Length());
			MaximumRadial = std::max(MaximumRadial, std::abs(G.Dot(Refined->GetDirections()[Vertex].Vector())));
			if(G.Length() > 0) ++Retained;
		}
		std::map<Edge, Owner> Owners;
		for(std::size_t FaceIndex = 0; FaceIndex < Refined->GetTriangles().size(); ++FaceIndex) {
			const Triangle &Face = Refined->GetTriangles()[FaceIndex];
			if(
				ShouldUseRoutes && Refined->GetElevations()[Face[0]] > Sea && Refined->GetElevations()[Face[1]] > Sea &&
				Refined->GetElevations()[Face[2]] > Sea
			) for(unsigned Subdivision = 0; Subdivision < 6; ++Subdivision) {
				const std::size_t Part = 6 * FaceIndex + Subdivision;
				const std::array<Vec3, 3> Domain = *Field->GetPieceDomain(Part);
				std::array<double, 3> Diagonal;
				for(unsigned Corner = 0; Corner < 3; ++Corner) {
					Diagonal[Corner] = Field->EvaluatePiece(Part, *UnitDirection::FromVector(Domain[Corner]))->ElevationMeters - Sea;
					MaximumNegativeCoefficient = std::max(MaximumNegativeCoefficient, -Diagonal[Corner]);
				}
				for(unsigned Side = 0; Side < 3; ++Side) {
					const unsigned Next = (Side + 1) % 3;
					const Vec3 Sum = Domain[Side] + Domain[Next];
					const double Middle = Field->EvaluatePiece(Part, *UnitDirection::FromVector(Sum))->ElevationMeters - Sea;
					const double Coefficient = 0.5 * (Sum.Dot(Sum) * Middle - Diagonal[Side] - Diagonal[Next]);
					MaximumNegativeCoefficient = std::max(MaximumNegativeCoefficient, -Coefficient);
				}
			}
			if(
				Refined->GetElevations()[Face[0]] <= Sea || Refined->GetElevations()[Face[1]] <= Sea || Refined->GetElevations()[Face[2]] <= Sea
			) for(const std::uint32_t Vertex : Face) {
				TestTrue(TEXT("海岸と海底の節点接線を保持"), !(*Limited)[Vertex].Length());
				++CoastVertices;
			}
			for(unsigned Side = 0; Side < 3; ++Side) {
				const std::uint32_t First = Face[Side], Second = Face[(Side + 1) % 3];
				const Edge Key { std::min(First, Second), std::max(First, Second) };
				const std::map<Edge, Owner>::const_iterator Other = Owners.find(Key);
				for(unsigned Half = 0; Half < 2; ++Half) {
					const std::uint32_t Anchor = Face[(Side + Half) % 3], End = Face[(Side + 1 - Half) % 3];
					const double FirstHeight = Refined->GetElevations()[Anchor], LastHeight = Refined->GetElevations()[End];
					const double Sign = LastHeight > FirstHeight ? 1 : LastHeight < FirstHeight ? -1 : 0;
					const std::size_t Part = 6 * FaceIndex + 2 * Side + Half;
					const std::array<Vec3, 3> Domain = *Field->GetPieceDomain(Part);
					const Vec3 A = Domain[0], B = Domain[1];
					const Vec3 Plane = UnitDirection::FromVector(A.Cross(B - A))->Vector();
					const TerrainSample AtStart = *Field->EvaluatePiece(Part, *UnitDirection::FromVector(A));
					const TerrainSample AtSplit = *Field->EvaluatePiece(Part, *UnitDirection::FromVector(B));
					const double DA = AtStart.TangentialGradientPerRadian.Dot(Plane.Cross(A));
					const double DB = AtSplit.TangentialGradientPerRadian.Dot(Plane.Cross(B));
					if(!ShouldUseRoutes || Required[Anchor] == End || Required[End] == Anchor) {
						if(Sign) MaximumReverseDerivative = std::max({ MaximumReverseDerivative, -Sign * DA, -Sign * DB });
						else MaximumReverseDerivative = std::max({ MaximumReverseDerivative, std::abs(DA), std::abs(DB) });
						MaximumOutsideHeight = std::max(
							{
								MaximumOutsideHeight,
								std::min(FirstHeight, LastHeight) - AtSplit.ElevationMeters,
								AtSplit.ElevationMeters - std::max(FirstHeight, LastHeight)
							}
						);
						// 両端微分と標高差が同符号なら，大円上の有理二次式の微分 Bernstein 係数も同符号となる
						++Intervals;
					}
					if(Other == Owners.end()) continue;
					const unsigned OtherHalf = Refined->GetTriangles()[Other->second.first][Other->second.second] == Anchor ? 0 : 1;
					const std::size_t OtherPart = 6 * Other->second.first + 2 * Other->second.second + OtherHalf;
					for(const double Fraction : { 0.19, 0.51, 0.87 }) {
						const UnitDirection Point = *UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
						const TerrainSample Left = *Field->EvaluatePiece(Part, Point), Right = *Field->EvaluatePiece(OtherPart, Point);
						MaximumJoin = std::max(MaximumJoin, std::abs(Left.ElevationMeters - Right.ElevationMeters));
						MaximumGradientJoin =
						std::max(MaximumGradientJoin, (Left.TangentialGradientPerRadian - Right.TangentialGradientPerRadian).Length());
					}
				}
				if(Other == Owners.end()) Owners.emplace(Key, Owner{ FaceIndex, Side });
				else Owners.erase(Other);
			}
		}
		TestTrue(TEXT("全共有辺を照合"), Owners.empty());
		std::stop_source Cancelled;
		Cancelled.request_stop();
		TestFalse(TEXT("中断を受理"), Refined->ConstrainGradients(Gradients, 0, Cancelled.get_token()).has_value());
		TestFalse(TEXT("欠落を拒否"), Refined->ConstrainGradients({}, 0).has_value());
		TestFalse(TEXT("非有限な海面を拒否"), Refined->ConstrainGradients(Gradients, std::numeric_limits<double>::quiet_NaN()).has_value());
		TestFalse(TEXT("下流配列の欠落を拒否"), Refined->ConstrainDrainageGradients(Gradients, 0, {}).has_value());
		TestFalse(TEXT("下流制限でも中断を受理"), Refined->ConstrainDrainageGradients(Gradients, 0, Required, Cancelled.get_token()).has_value());
		std::vector<std::uint32_t> Invalid = Required;
		Invalid[0] = static_cast<std::uint32_t>(Required.size());
		TestFalse(TEXT("範囲外の下流を拒否"), Refined->ConstrainDrainageGradients(Gradients, 0, Invalid).has_value());
		const std::span<const double> Elevations = Refined->GetElevations();
		const std::uint32_t Highest =
		static_cast<std::uint32_t>(std::max_element(Elevations.begin(), Elevations.end()) - Elevations.begin());
		const std::uint32_t Lowest =
		static_cast<std::uint32_t>(std::min_element(Elevations.begin(), Elevations.end()) - Elevations.begin());
		Invalid = Required;
		Invalid[Highest] = Lowest;
		TestFalse(TEXT("高低差だけ正しい非隣接辺を拒否"), Refined->ConstrainDrainageGradients(Gradients, 0, Invalid).has_value());
		Invalid = Required;
		Invalid[Lowest] = Highest;
		TestFalse(TEXT("上り返す下流を拒否"), Refined->ConstrainDrainageGradients(Gradients, 0, Invalid).has_value());
		Gradients[0].X = std::numeric_limits<double>::quiet_NaN();
		TestFalse(TEXT("非有限な勾配を拒否"), Refined->ConstrainGradients(Gradients, 0).has_value());
	}
	TestTrue(TEXT("陸面の全同次二次係数を非負に保つ"), MaximumNegativeCoefficient < 1E-6);
	TestTrue(TEXT("辺全区間の逆勾配を防ぐ"), MaximumReverseDerivative < 1E-6 && MaximumOutsideHeight < 1E-8);
	TestTrue(TEXT("不規則な網でも C1 接続を保持"), MaximumJoin < 1E-6 && MaximumGradientJoin < 1E-6);
	TestTrue(TEXT("勾配を増幅せず球面接線を保持"), MaximumGrowth < 1E-8 && MaximumRadial < 1E-8);
	TestTrue(TEXT("全点零への退化でなく元勾配と海岸を検査"), Retained > 0 && CoastVertices > 0);
	AddInfo(
		FString::Printf(
			TEXT("Spherical constrained gradient: intervals=%llu retained=%llu coast=%llu reverse=%.12g ")
			TEXT("outside=%.12g join=%.12g gradientJoin=%.12g growth=%.12g radial=%.12g negativeCoefficient=%.12g"),
			static_cast<unsigned long long>(Intervals),
			static_cast<unsigned long long>(Retained),
			static_cast<unsigned long long>(CoastVertices),
			MaximumReverseDerivative,
			MaximumOutsideHeight,
			MaximumJoin,
			MaximumGradientJoin,
			MaximumGrowth,
			MaximumRadial,
			MaximumNegativeCoefficient
		)
	);
	// 全区間の微分符号と共有接線の検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalClosestGradient, "PlanetGenerator.SphericalSpline.ClosestGradient",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 最近点が解析的に既知の接線制約を，異なる球面方向・尺度・退化境界で検査する
 * @param Parameters 未使用の UE 引数
 * @return 独立な最近点・方向・入力拒否の検査に成功した場合に真
 */
bool FPlanetSphericalClosestGradient::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::ProjectSphericalGradient;
	using PlanetCore::SphericalGradientConstraint;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	double MaximumError = 0, MaximumRadial = 0, MaximumGrowth = 0;
	unsigned Cases = 0;
	for(const Vec3 Direction : { Vec3{ 0, 0, 1 }, Vec3{ 1, 2, -3 }, Vec3{ -5, 0.3, 2 } }) {
		const UnitDirection Point = *UnitDirection::FromVector(Direction);
		const Vec3 P = Point.Vector(), Axis = std::abs(P.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
		const Vec3 E = UnitDirection::FromVector(P.Cross(Axis))->Vector(), N = P.Cross(E);
		for(const double Scale : { 1E-4, 1.0, 1E6 }) {
			const std::array<SphericalGradientConstraint, 2> Box { { { E, Scale }, { N, 2 * Scale } } };
			const auto Check =
			[&](const Vec3 Original, const Vec3 Expected, const std::span<const SphericalGradientConstraint> Constraints) -> void {
				const std::optional<Vec3> Result = ProjectSphericalGradient(Point, Original, Constraints);
				// 正規入力に対する射影を準備出来ない場合の返戻
				if(!TestTrue(TEXT("正規の最近点を準備"), Result.has_value())) return;
				MaximumError = std::max(MaximumError, (*Result - Expected).Length() / Scale);
				MaximumRadial = std::max(MaximumRadial, std::abs(Result->Dot(P)) / Scale);
				MaximumGrowth = std::max(MaximumGrowth, (Result->Length() - Original.Length()) / Scale);
				++Cases;
				// 終了
				return;
			};
			Check((E * 2 + N * 3) * Scale, (E + N * 2) * Scale, Box);
			Check((E * -2 + N * 3) * Scale, N * (2 * Scale), Box);
			Check((E * 0.5 + N) * Scale, (E * 0.5 + N) * Scale, Box);
			Check((E * 2 + N * 3 + P * 5) * Scale, (E + N * 2) * Scale, Box);
			const std::array<SphericalGradientConstraint, 1> Diagonal { { { (E + N) * std::sqrt(0.5), Scale * std::sqrt(0.5) } } };
			Check((E + N) * (2 * Scale), (E + N) * (0.5 * Scale), Diagonal);
			const std::array<SphericalGradientConstraint, 2> Opposite { { { E, 2 * Scale }, { E * -1, 2 * Scale } } };
			Check((E * 2 + N) * Scale, N * Scale, Opposite);
			const std::array<SphericalGradientConstraint, 1> Zero { { { E, 0 } } };
			Check((E * 2 + N) * Scale, N * Scale, Zero);
			Check((E * 2 + N * 3) * Scale, (E * 2 + N * 3) * Scale, {});
			const std::array<SphericalGradientConstraint, 1> Lower { { { E, 3 * Scale, -Scale } } };
			Check((E * -2 + N * 3) * Scale, (E * -1 + N * 3) * Scale, Lower);
		}
		const std::array<SphericalGradientConstraint, 1> Negative { { { E, -1 } } }, Radial { { { P, 1 } } };
		const std::array<SphericalGradientConstraint, 1> Nonunit { { { E * 2, 1 } } }, PositiveLower { { { E, 2, 1 } } };
		TestFalse(TEXT("原点を含まない下限を拒否"), ProjectSphericalGradient(Point, E, PositiveLower).has_value());
		TestFalse(TEXT("負の上限を拒否"), ProjectSphericalGradient(Point, E, Negative).has_value());
		TestFalse(TEXT("径方向の制約を拒否"), ProjectSphericalGradient(Point, E, Radial).has_value());
		TestFalse(TEXT("非単位の制約を拒否"), ProjectSphericalGradient(Point, E, Nonunit).has_value());
		TestFalse(
			TEXT("非有限な元勾配を拒否"),
			ProjectSphericalGradient(Point, { std::numeric_limits<double>::infinity(), 0, 0 }, {}).has_value()
		);
	}
	TestEqual(TEXT("独立な最近点条件を全件検査"), Cases, 81U);
	TestTrue(TEXT("解析解へ一致し接線を保持"), MaximumError < 1E-10 && MaximumRadial < 1E-10);
	TestTrue(TEXT("原点を含む凸制約で勾配を増幅しない"), MaximumGrowth < 1E-10);
	AddInfo(
		FString::Printf(
			TEXT("Spherical closest gradient: cases=%u error=%.12g radial=%.12g growth=%.12g"),
			Cases,
			MaximumError,
			MaximumRadial,
			MaximumGrowth
		)
	);
	// 独立な解析解との比較結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalMonotoneQuadratic, "PlanetGenerator.SphericalSpline.MonotoneQuadratic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 既に単調な球面二次斜面を，下流制限が不要に平坦化しない事を検査する
 * @param Parameters 未使用の UE 引数
 * @return 元の解析接線と全球二次式を保持した場合に真
 */
bool FPlanetSphericalMonotoneQuadratic::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(2);
	// 解析斜面の網を用意出来ない場合の返戻
	if(!TestTrue(TEXT("単調二次式の網を準備"), Mesh.has_value())) return false;
	const auto Source = [](const UnitDirection Direction) -> PlanetCore::TerrainSample {
		const Vec3 P = Direction.Vector(), Ambient { 500 * P.Y, 500 * P.X, 400 * P.Z };
		// 全域が海面より上に在る，同次二次式と正確な解析接線の返戻
		return { 4000 + 500 * P.X * P.Y + 200 * P.Z * P.Z, Ambient - P * P.Dot(Ambient) };
	};
	std::vector<double> Heights;
	std::vector<Vec3> Gradients;
	std::vector<std::uint32_t> Routes;
	for(const UnitDirection Direction : Mesh->Directions) {
		const PlanetCore::TerrainSample Sample = Source(Direction);
		Routes.push_back(static_cast<std::uint32_t>(Routes.size()));
		Heights.push_back(Sample.ElevationMeters);
		Gradients.push_back(Sample.TangentialGradientPerRadian);
	}
	std::uint32_t SelectedEdges = 0;
	for(const std::array<std::uint32_t, 3> &Face : Mesh->Triangles) {
		for(unsigned Corner = 0; Corner < 3; ++Corner) for(unsigned Offset = 1; Offset < 3; ++Offset) {
			const std::uint32_t A = Face[Corner], B = Face[(Corner + Offset) % 3];
			if(Heights[B] >= Heights[Routes[A]]) continue;
			const Vec3 Start = Mesh->Directions[A].Vector(), End = Mesh->Directions[B].Vector();
			const double Cosine = Start.Dot(End), Sine = Start.Cross(End).Length();
			const Vec3 First = (End - Start * Cosine) * (1 / Sine), Last = (End * Cosine - Start) * (1 / Sine);
			// 既に両端で下降する二次式の辺だけを指定し，新たな地形制約を混入しない
			if(Gradients[A].Dot(First) >= -1E-8 || Gradients[B].Dot(Last) >= -1E-8) continue;
			Routes[A] = B;
		}
	}
	for(std::uint32_t Index = 0; Index < Routes.size(); ++Index) if(Routes[Index] != Index) ++SelectedEdges;
	PlanetCore::SphericalReliefSettings Settings;
	Settings.MaximumAddedVertices = 0;
	const std::optional<PlanetCore::PlanetSphericalRefinement> Scaffold = PlanetCore::PlanetSphericalRefinement::Create(
		Mesh->Directions,
		Mesh->Triangles,
		Heights,
		[&Source](const UnitDirection Direction) -> double {
			// 同じ解析式の標高だけの返戻
			return Source(Direction).ElevationMeters;
		},
		Settings
	);
	// 同じ値と節点を制限へ渡せない場合の返戻
	if(!TestTrue(TEXT("二次式の制約を準備"), Scaffold.has_value() && SelectedEdges > 100)) return false;
	const std::optional<std::vector<Vec3>> Limited = Scaffold->ConstrainDrainageGradients(Gradients, 0, Routes);
	// 選んだ単調辺を扱えない場合の返戻
	if(!TestTrue(TEXT("既に下降する解析接線を制限"), Limited.has_value())) return false;
	double MaximumGradientChange = 0, MaximumValueError = 0;
	for(std::size_t Index = 0; Index < Gradients.size(); ++Index) {
		MaximumGradientChange = std::max(MaximumGradientChange, ((*Limited)[Index] - Gradients[Index]).Length());
	}
	const std::optional<PlanetCore::SphericalSplineField> Field =
	PlanetCore::SphericalSplineField::CreateOnMeshWithGradients(2, Mesh->Directions, Mesh->Triangles, Heights, *Limited);
	// 制限後に補間出来ない場合の返戻
	if(!TestTrue(TEXT("制限後の二次式を補間"), Field.has_value())) return false;
	for(unsigned Index = 0; Index < 1024; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / 1024, Radius = std::sqrt(1 - Z * Z), Angle = Index * 2.399963229728653;
		const UnitDirection At = *UnitDirection::FromVector({ Radius * std::cos(Angle), Radius * std::sin(Angle), Z });
		MaximumValueError = std::max(MaximumValueError, std::abs(Field->Evaluate(At).ElevationMeters - Source(At).ElevationMeters));
	}
	TestTrue(TEXT("既に単調な解析接線を保持"), MaximumGradientChange < 1E-8);
	TestTrue(TEXT("二次斜面に機械的なうねりを作らない"), MaximumValueError < 1E-8);
	AddInfo(
		FString::Printf(
			TEXT("Monotone quadratic: edges=%u gradientChange=%.12g valueError=%.12g"),
			SelectedEdges,
			MaximumGradientChange,
			MaximumValueError
		)
	);
	// 解析解に対する形状保持の検査結果の返戻
	return !HasAnyErrors();
}

#endif
