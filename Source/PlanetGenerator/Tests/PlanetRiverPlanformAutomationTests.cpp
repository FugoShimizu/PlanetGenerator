#include "Core/PlanetDrainage.h"
#include "Core/PlanetRiverPlanform.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetRiverPlanformContracts,
	"PlanetGenerator.RiverPlanform.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 河道曲線の接続・再現性・解析微分・海岸探索を独立に検査する
 *
 * @param Parameters 未使用の Automation 引数
 * @return 全ての曲線条件が合格した場合に真
 */
bool FPlanetRiverPlanformContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector({ 0.3, -0.7, 0.4 });
	const PlanetCore::UnitDirection Along = *PlanetCore::UnitDirection::FromVector(Start.Vector().Cross({ 0.4, 0.1, 0.8 }));
	std::size_t Samples = 0, Shores = 0, CoverageCases = 0;
	/**
	 * 独立な同次平面から三角形頂点の線形水深を定める
	 * @param Triangle 正向きの球面三角形
	 * @param Plane 水深の符号を定める平面
	 * @return 頂点の水深，正の領域は同じ弦面で凸
	 */
	const auto DepthsFor =
	[](const std::array<PlanetCore::UnitDirection, 3> &Triangle, const PlanetCore::Vec3 Plane) -> std::array<double, 3> {
		std::array<double, 3> Values;
		for(unsigned K = 0; K < 3; ++K) Values[K] = Triangle[K].Vector().Dot(Plane);
		// 独立な線形水域の三頂点水深の返戻
		return Values;
	};
	double MaximumDerivativeError = 0, MaximumCurvatureError = 0, MaximumOffsetMeters = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) for(const double Angle : { 0.014, 0.05, 0.2 }) {
		const PlanetCore::UnitDirection End =
		*PlanetCore::UnitDirection::FromVector(Start.Vector() * std::cos(Angle) + Along.Vector() * std::sin(Angle));
		for(const double Width : { 0.8, 15.0, 120.0 }) {
			const std::optional<PlanetCore::RiverPlanform> Curve = PlanetCore::RiverPlanform::Create(Start, End, Radius, Width, 42);
			const std::optional<PlanetCore::RiverPlanform> Repeated = PlanetCore::RiverPlanform::Create(Start, End, Radius, Width, 42);
			const std::optional<PlanetCore::RiverPlanform> Other = PlanetCore::RiverPlanform::Create(Start, End, Radius, Width, 7);
			// 準備失敗を後続の有限検査へ持込まない返戻
			const bool Accepted1 = TestTrue(TEXT("Valid curved planforms prepared"), Curve && Repeated && Other);
			// 検査条件の不合格を保持する返戻
			if(!Accepted1) return false;
			const PlanetCore::RiverPlanformSample First = Curve->Evaluate(0), Last = Curve->Evaluate(1);
			// 端点と二階迄の大円接続を独立に確認する
			const bool Accepted2 = TestTrue(
				TEXT("Endpoints and C2 joins retained"),
				(First.Direction.Vector() - Start.Vector()).Length() < 1E-14 && (Last.Direction.Vector() - End.Vector()).Length() < 1E-14 &&
				(First.DerivativePerFraction - Along.Vector() * Angle).Length() < 1E-14 &&
				(First.SecondDerivativePerFractionSquared + Start.Vector() * (Angle * Angle)).Length() < 1E-14 &&
				(Last.SecondDerivativePerFractionSquared + End.Vector() * (Angle * Angle)).Length() < 1E-14
			);
			// 検査条件の不合格を保持する返戻
			if(!Accepted2) return false;
			double PreviousAngle = -1, DifferentSeedDistance = 0;
			for(unsigned I = 0; I < 257; ++I) {
				const double T = I / 256.0;
				const PlanetCore::RiverPlanformSample Point = Curve->Evaluate(T), Again = Repeated->Evaluate(T);
				const PlanetCore::Vec3 D = Point.Direction.Vector(), Tangent = Point.DerivativePerFraction;
				const double RecoveredAngle = std::atan2(D.Dot(Along.Vector()), D.Dot(Start.Vector())), Stretch = Tangent.Length() / Angle;
				// 位置と微分の有限性・公称座標の前進・実接線長上限を確認する
				const bool Accepted3 = TestTrue(
					TEXT("Unique forward parameter and bounded nonzero tangent"),
					std::isfinite(Point.SecondDerivativePerFractionSquared.Length()) && RecoveredAngle > PreviousAngle &&
					std::abs(RecoveredAngle - T * Angle) < 1E-13 && std::abs(D.Dot(Tangent)) < 1E-13 && Stretch > 0 &&
					Stretch <= Curve->MaximumTangentStretch() + 1E-12
				);
				// 検査条件の不合格を保持する返戻
				if(!Accepted3) return false;
				PreviousAngle = RecoveredAngle;
				const bool Accepted4 = TestTrue(
					TEXT("Same seed repeats every value"),
					D.X == Again.Direction.Vector().X && D.Y == Again.Direction.Vector().Y && D.Z == Again.Direction.Vector().Z
				);
				// 検査条件の不合格を保持する返戻
				if(!Accepted4) return false;
				DifferentSeedDistance = std::max(DifferentSeedDistance, (D - Other->Evaluate(T).Direction.Vector()).Length());
				const PlanetCore::Vec3 Straight = Start.Vector() * std::cos(T * Angle) + Along.Vector() * std::sin(T * Angle);
				MaximumOffsetMeters = std::max(MaximumOffsetMeters, (D - Straight).Length() * Radius);
				const double Offset = (D - Straight).Length(), Bound = Curve->MaximumDeviationRadians() + 1E-14;
				// 公称大円からの離隔を独立な位置差で確認する
				const bool Accepted5 = TestTrue(TEXT("Lateral bound covers the curve"), Offset <= Bound);
				// 検査条件の不合格を保持する返戻
				if(!Accepted5) return false;
				if(I && I < 256) {
					constexpr double H = 2E-5;
					const PlanetCore::Vec3 Coarse =
					(Curve->Evaluate(T + H).Direction.Vector() - Curve->Evaluate(T - H).Direction.Vector()) * (0.5 / H);
					const PlanetCore::Vec3 Fine =
					(Curve->Evaluate(T + H * 0.5).Direction.Vector() - Curve->Evaluate(T - H * 0.5).Direction.Vector()) * (1 / H);
					const PlanetCore::Vec3 Independent = (Fine * 4 - Coarse) * (1.0 / 3);
					const double DerivativeError = (Independent - Tangent).Length() / std::max(1E-5, Tangent.Length());
					const PlanetCore::Vec3 CoarseSecond =
					(Curve->Evaluate(T + H).DerivativePerFraction - Curve->Evaluate(T - H).DerivativePerFraction) * (0.5 / H);
					const PlanetCore::Vec3 FineSecond =
					(Curve->Evaluate(T + H * 0.5).DerivativePerFraction - Curve->Evaluate(T - H * 0.5).DerivativePerFraction) * (1 / H);
					const PlanetCore::Vec3 IndependentSecond = (FineSecond * 4 - CoarseSecond) * (1.0 / 3);
					const double CurvatureError = (IndependentSecond - Point.SecondDerivativePerFractionSquared).Length() /
					std::max(1E-5, Point.SecondDerivativePerFractionSquared.Length());
					MaximumDerivativeError = std::max(MaximumDerivativeError, DerivativeError);
					MaximumCurvatureError = std::max(MaximumCurvatureError, CurvatureError);
					const bool Accepted6 = TestTrue(
						TEXT("Analytic velocity and curvature match independent differences"),
						DerivativeError < 5E-8 && CurvatureError < 5E-8
					);
					// 検査条件の不合格を保持する返戻
					if(!Accepted6) return false;
				}
				++Samples;
			}
			const bool Accepted7 = TestTrue(TEXT("Different seed changes the interior"), DifferentSeedDistance > 1E-13);
			// 検査条件の不合格を保持する返戻
			if(!Accepted7) return false;
			const PlanetCore::Vec3 Center = Start.Vector() * std::cos(Angle * 0.5) + Along.Vector() * std::sin(Angle * 0.5);
			const PlanetCore::Vec3 Forward = Along.Vector() * std::cos(Angle * 0.5) - Start.Vector() * std::sin(Angle * 0.5);
			const PlanetCore::Vec3 Across = Start.Vector().Cross(Along.Vector());
			const std::array<PlanetCore::UnitDirection, 3> Triangle {
				*PlanetCore::UnitDirection::FromVector(Center - Forward * (2 * Angle) - Across * (2 * Angle)),
				*PlanetCore::UnitDirection::FromVector(Center + Forward * (2 * Angle) - Across * (2 * Angle)),
				*PlanetCore::UnitDirection::FromVector(Center + Across * (3 * Angle))
			};
			const bool WholeWater = TestTrue(
				TEXT("Whole wet triangle covers every curved intersection"),
				Curve->IsCoveredByLinearWater(Curve->ArcRadians(), Triangle, { 1, 1, 1 })
			);
			// 凸な水域の独立検査が不合格の場合の返戻
			if(!WholeWater) return false;
			const bool DryCenter = TestFalse(
				TEXT("Curved center in a dry triangle still requests discovery"),
				Curve->IsCoveredByLinearWater(Curve->ArcRadians(), Triangle, { -1, -1, -1 })
			);
			// 乾いた区間の検査が不合格の場合の返戻
			if(!DryCenter) return false;
			const PlanetCore::Vec3 WetPlane = Center * (2 * Curve->MaximumDeviationRadians() + 0.001) + Across;
			const bool BoundedWet = TestTrue(
				TEXT("Mixed triangle continuously covers the bounded curved path"),
				Curve->IsCoveredByLinearWater(Curve->ArcRadians(), Triangle, DepthsFor(Triangle, WetPlane))
			);
			// 中間の曲線を含む被覆検査が不合格の場合の返戻
			if(!BoundedWet) return false;
			const double MidBend = Curve->Evaluate(0.5).Direction.Vector().Dot(Across);
			const PlanetCore::Vec3 DipPlane = Center * (0.5 * std::abs(MidBend)) - Across * (MidBend > 0 ? 1 : -1);
			const bool Dip = TestFalse(
				TEXT("Wet endpoints do not hide a dry curved interior"),
				Curve->IsCoveredByLinearWater(Curve->ArcRadians(), Triangle, DepthsFor(Triangle, DipPlane))
			);
			// 端点だけで被覆と誤認した場合の返戻
			if(!Dip) return false;
			const PlanetCore::Vec3 CutPlane = Start.Vector() * std::sin(0.3 * Angle) - Along.Vector() * std::cos(0.3 * Angle);
			const std::array<double, 3> CutDepths = DepthsFor(Triangle, CutPlane);
			const bool Clipped = TestTrue(
				TEXT("Only the clipped wet prefix can be certified"),
				Curve->IsCoveredByLinearWater(Curve->ArcRadians() * 0.2, Triangle, CutDepths) &&
				!Curve->IsCoveredByLinearWater(Curve->ArcRadians(), Triangle, CutDepths)
			);
			// 切詰めた範囲の検査が不合格の場合の返戻
			if(!Clipped) return false;
			const std::array<PlanetCore::UnitDirection, 3> Disjoint {
				*PlanetCore::UnitDirection::FromVector(Center - Forward * (0.1 * Angle) + Across * (0.9 * Angle)),
				*PlanetCore::UnitDirection::FromVector(Center + Forward * (0.1 * Angle) + Across * (0.9 * Angle)),
				*PlanetCore::UnitDirection::FromVector(Center + Across * (1.1 * Angle))
			};
			const bool Outside = TestTrue(
				TEXT("Dry disjoint triangles do not force river width refinement"),
				Curve->IsCoveredByLinearWater(Curve->ArcRadians(), Disjoint, { -1, -1, -1 })
			);
			// 三角形外の区間の検査が不合格の場合の返戻
			if(!Outside) return false;
			const bool InvalidCoverage = TestFalse(
				TEXT("Nonfinite coverage depth is never certified"),
				Curve->IsCoveredByLinearWater(Curve->ArcRadians(), Triangle, { 1, std::numeric_limits<double>::quiet_NaN(), 1 })
			);
			// 不正な入力の検査が不合格の場合の返戻
			if(!InvalidCoverage) return false;
			CoverageCases += 7;
			constexpr double KnownFraction = 0.437;
			const auto Path = [&](const double T) -> PlanetCore::UnitDirection {
				return Curve->Evaluate(T).Direction;
			};
			const auto Height = [&](const PlanetCore::UnitDirection D) -> double {
				// 公称角度から独立に定めた解析海岸の返戻
				return (KnownFraction * Angle - std::atan2(D.Vector().Dot(Along.Vector()), D.Vector().Dot(Start.Vector()))) * Radius;
			};
			const std::optional<PlanetCore::ShoreCrossing> Shore =
			PlanetCore::FindShoreCrossingOnCurve(Path, Angle * Radius, Curve->MaximumTangentStretch(), Radius, 0, Height);
			const PlanetCore::Vec3 Expected = Curve->Evaluate(KnownFraction).Direction.Vector();
			const double ShoreError = Shore ? (Shore->Direction - Expected).Length() * Radius : std::numeric_limits<double>::infinity();
			const double Bracket = Shore ? Shore->BracketWidthMeters : std::numeric_limits<double>::infinity();
			const double Residual = Shore ? Shore->TerrainResidualMeters : std::numeric_limits<double>::infinity();
			const double Tolerance = std::max(1E-5, Radius * 1E-12);
			const bool Accepted8 = TestTrue(
				TEXT("Shore search follows the same curve with the original physical tolerance"),
				Shore && ShoreError <= Tolerance && Bracket <= Tolerance && Residual <= 0
			);
			// 検査条件の不合格を保持する返戻
			if(!Accepted8) return false;
			++Shores;
		}
	}
	const bool Accepted9 =
	TestFalse(TEXT("Degenerate arc rejected"), PlanetCore::RiverPlanform::Create(Start, Start, 1000, 1, 42).has_value());
	// 検査条件の不合格を保持する返戻
	if(!Accepted9) return false;
	const bool Accepted10 =
	TestFalse(TEXT("Invalid width rejected"), PlanetCore::RiverPlanform::Create(Start, Along, 1000, 0, 42).has_value());
	// 検査条件の不合格を保持する返戻
	if(!Accepted10) return false;
	const auto ConstantPath = [&](const double) -> PlanetCore::UnitDirection {
		return Start;
	};
	const auto Height = [&](const PlanetCore::UnitDirection) -> double {
		return 1.0;
	};
	const bool Accepted11 = TestFalse(
		TEXT("Invalid curve stretch rejected"),
		PlanetCore::FindShoreCrossingOnCurve(ConstantPath, 100, 0.5, 1000, 0, Height).has_value()
	);
	// 検査条件の不合格を保持する返戻
	if(!Accepted11) return false;
	AddInfo(
		FString::Printf(
			TEXT("River planforms: samples=%llu shores=%llu coverage=%llu derivative=%.12g curvature=%.12g maximumOffsetMeters=%.12g"),
			static_cast<unsigned long long>(Samples),
			static_cast<unsigned long long>(Shores),
			static_cast<unsigned long long>(CoverageCases),
			MaximumDerivativeError,
			MaximumCurvatureError,
			MaximumOffsetMeters
		)
	);
	// 全構成条件と独立差分の検査結果の返戻
	return true;
}

#endif
