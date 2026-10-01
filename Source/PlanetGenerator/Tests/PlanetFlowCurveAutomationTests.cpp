#include "Core/PlanetFlowChannel.h"
#include "Core/PlanetFlowCurve.h"
#include "Core/PlanetFlowRoute.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetFlowCurveContracts, "PlanetGenerator.FlowCurve.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 球面上の位置・解析微分・共有接線と，既知の曲がる下降線を独立に検査する
 * @param Parameters 未使用の UE 引数
 * @return 解析形状・接合・不正入力の条件が成立する場合に真
 */
bool FPlanetFlowCurveContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::TerrainFlowSegment;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const Vec3 Up = UnitDirection::FromVector({ 0.37, -0.21, 0.903 })->Vector();
	const Vec3 East = UnitDirection::FromVector(Vec3{ 0, 1, 0 }.Cross(Up))->Vector(), North = Up.Cross(East);
	double MaximumPlaneError = 0, MaximumDerivativeError = 0, MaximumCurveResidual = 0, MaximumJoinError = 0;
	double MaximumCurvatureJoinError = 0, MaximumSecondDerivativeError = 0, MaximumFootpointError = 0;
	double MaximumFootpointDerivativeError = 0, MaximumDistanceDerivativeError = 0, MaximumBedDerivativeError = 0;
	double MaximumWaterDerivativeError = 0;
	std::uint32_t Samples = 0, FootpointSamples = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		// 大円の位置・接線は補間式と独立に三角関数から用意する
		const auto Circle = [&](const double Distance) -> std::pair<UnitDirection, Vec3> {
			const double Angle = Distance / Radius;
			// 既知の大円点と進行方向の返戻
			return {
				*UnitDirection::FromVector(Up * std::cos(Angle) + East * std::sin(Angle)),
				East * std::cos(Angle) - Up * std::sin(Angle)
			};
		};
		const std::pair<UnitDirection, Vec3> A = Circle(0), B = Circle(20), C = Circle(55);
		const std::optional<TerrainFlowSegment> First = TerrainFlowSegment::Create(A.first, B.first, A.second, B.second, Radius);
		const std::optional<TerrainFlowSegment> Next = TerrainFlowSegment::Create(B.first, C.first, B.second, C.second, Radius);
		// 正規な入力が通らない場合の返戻
		if(!TestTrue(TEXT("異なる長さの大円区間を構築出来る"), First && Next)) return false;
		const PlanetCore::TerrainFlowCurveSample Left = *First->Evaluate(1), Right = *Next->Evaluate(0);
		const double JoinError = Radius * (Left.DirectionDerivativePerMeter - Right.DirectionDerivativePerMeter).Length();
		TestEqual(TEXT("共有位置が正確に一致"), (Left.Direction.Vector() - Right.Direction.Vector()).Length(), 0.0);
		TestTrue(TEXT("共有接線は区間長に依存しない"), JoinError < 1E-12);
		MaximumJoinError = std::max(MaximumJoinError, JoinError);
		for(unsigned i = 1; i < 100; ++i) {
			const double T = i / 100.0;
			constexpr double Step = 0.001;
			const PlanetCore::TerrainFlowCurveSample Value = *First->Evaluate(T);
			const Vec3 Low = First->Evaluate(T - Step)->Direction.Vector(), High = First->Evaluate(T + Step)->Direction.Vector();
			const Vec3 Difference = (High - Low) * (1 / (2 * Step * First->GetParameterLengthMeters()));
			const double PlaneError = Radius * std::abs(Value.Direction.Vector().Dot(North));
			const double DerivativeError = Radius * (Difference - Value.DirectionDerivativePerMeter).Length();
			TestTrue(TEXT("大円面から離れない"), PlaneError < 1E-6);
			TestTrue(TEXT("解析微分と独立な中央差分が一致"), DerivativeError < 1E-5);
			TestTrue(TEXT("球面の法線方向へ速度が漏れない"), Radius * std::abs(Value.Direction.Vector().Dot(Value.DirectionDerivativePerMeter)) < 1E-12);
			MaximumPlaneError = std::max(MaximumPlaneError, PlaneError);
			MaximumDerivativeError = std::max(MaximumDerivativeError, DerivativeError);
			++Samples;
		}
		// 立体射影上の異方的な二次場の下降線 V = U² / 160
		const auto Parabola = [&](const double U) -> std::pair<UnitDirection, Vec3> {
			const double V = U * U / 160, DV = U / 80, Square = (U * U + V * V) / (4 * Radius * Radius);
			const Vec3 Position = Up * (1 - Square) + (East * U + North * V) * (1 / Radius);
			const UnitDirection Point = *UnitDirection::FromVector(Position);
			const Vec3 Derivative = Up * (-(U + V * DV) / (2 * Radius * Radius)) + (East + North * DV) * (1 / Radius);
			const Vec3 Tangent = Derivative - Point.Vector() * Derivative.Dot(Point.Vector());
			// U が減少する既知の下降曲線の返戻
			return { Point, Tangent * -1 };
		};
		for(unsigned Segment = 0; Segment < 8; ++Segment) {
			const double U = 120 - Segment * 10.0;
			const std::pair<UnitDirection, Vec3> Start = Parabola(U), End = Parabola(U - 10);
			const std::optional<TerrainFlowSegment> Curve =
			TerrainFlowSegment::Create(Start.first, End.first, Start.second, End.second, Radius);
			// 正規な曲線を構築出来ない場合の返戻
			if(!TestTrue(TEXT("解析的な下降線の区間を構築出来る"), Curve.has_value())) return false;
			for(unsigned i = 0; i < 101; ++i) {
				const PlanetCore::TerrainFlowCurveSample Sample = *Curve->Evaluate(i / 100.0);
				const Vec3 P = Sample.Direction.Vector(), D = Sample.DirectionDerivativePerMeter;
				const double Denominator = 1 + P.Dot(Up), X = 2 * Radius * P.Dot(East) / Denominator;
				const double Y = 2 * Radius * P.Dot(North) / Denominator;
				const double DX = 2 * Radius * (D.Dot(East) * Denominator - P.Dot(East) * D.Dot(Up)) / (Denominator * Denominator);
				const double DY = 2 * Radius * (D.Dot(North) * Denominator - P.Dot(North) * D.Dot(Up)) / (Denominator * Denominator);
				const double Residual = std::abs(Y - X * X / 160), HeightDerivative = 0.002 * X * DX + 0.004 * Y * DY;
				TestTrue(TEXT("閉形式の曲線から１ cm 以内"), Residual < 0.01);
				TestTrue(TEXT("曲線化した後も元場を下降"), HeightDerivative < 0);
				MaximumCurveResidual = std::max(MaximumCurveResidual, Residual);
				++Samples;
			}
		}
		// 曲率が既知の小円で，長さが異なる隣接区間の二次微分を独立に調べる
		const double SmallAngle = 80 / Radius, SmallRadius = Radius * std::sin(SmallAngle);
		struct CircleJet {
			UnitDirection Position;
			Vec3 Tangent;
			Vec3 Curvature;
		};
		const auto SmallCircle = [&](const double Angle) -> CircleJet {
			const Vec3 Radial = East * std::cos(Angle) + North * std::sin(Angle);
			// 三角関数に依る球面位置・単位接線・実弧長当たりの曲率の返戻
			return {
				*UnitDirection::FromVector(Up * std::cos(SmallAngle) + Radial * std::sin(SmallAngle)),
				North * std::cos(Angle) - East * std::sin(Angle),
				Radial * (-1 / SmallRadius)
			};
		};
		const CircleJet J0 = SmallCircle(0), J1 = SmallCircle(0.16), J2 = SmallCircle(0.45);
		const std::optional<TerrainFlowSegment> Curved =
		TerrainFlowSegment::CreateWithCurvature(J0.Position, J1.Position, J0.Tangent, J1.Tangent, J0.Curvature, J1.Curvature, Radius);
		const std::optional<TerrainFlowSegment> Following =
		TerrainFlowSegment::CreateWithCurvature(J1.Position, J2.Position, J1.Tangent, J2.Tangent, J1.Curvature, J2.Curvature, Radius);
		// 正常な曲率を持つ小円を構成出来ない場合の返戻
		if(!TestTrue(TEXT("共有曲率を持つ球面曲線を構成出来る"), Curved && Following)) return false;
		const PlanetCore::TerrainFlowCurveSample Before = *Curved->Evaluate(1), After = *Following->Evaluate(0);
		const double CurvatureJoin =
		Radius * (Before.DirectionSecondDerivativePerMeterSquared - After.DirectionSecondDerivativePerMeterSquared).Length();
		TestTrue(TEXT("異なる区間長の間でも二次微分が一致"), CurvatureJoin < 1E-8);
		TestTrue(
			TEXT("共有曲率の実値が独立な小円の曲率と一致"),
			(Before.DirectionSecondDerivativePerMeterSquared * Radius - J1.Curvature).Length() < 1E-8
		);
		MaximumCurvatureJoinError = std::max(MaximumCurvatureJoinError, CurvatureJoin);
		for(unsigned i = 1; i < 100; ++i) {
			const double T = i / 100.0;
			constexpr double Step = 0.001;
			const PlanetCore::TerrainFlowCurveSample Value = *Curved->Evaluate(T);
			const Vec3 Difference =
			(Curved->Evaluate(T + Step)->DirectionDerivativePerMeter - Curved->Evaluate(T - Step)->DirectionDerivativePerMeter) *
			(1 / (2 * Step * Curved->GetParameterLengthMeters()));
			const double Error = Radius * (Difference - Value.DirectionSecondDerivativePerMeterSquared).Length();
			const Vec3 Position = Value.Direction.Vector();
			const double RadialDistance = Radius * std::hypot(Position.Dot(East), Position.Dot(North));
			TestTrue(TEXT("解析二次微分と一次微分の独立差分が一致"), Error < 1E-8);
			TestTrue(TEXT("曲率を共有する曲線が既知の小円から離れない"), std::abs(RadialDistance - SmallRadius) < 0.002);
			TestTrue(
				TEXT("球面上の二次微分の半径方向成分が一致"),
				std::abs(
					Position.Dot(Value.DirectionSecondDerivativePerMeterSquared) +
					Value.DirectionDerivativePerMeter.Dot(Value.DirectionDerivativePerMeter)
				) * Radius < 1E-12
			);
			MaximumSecondDerivativeError = std::max(MaximumSecondDerivativeError, Error);
			++Samples;
		}
		// 多区間の索引を，既知の小円と全区間走査の両方へ比較する
		std::vector<PlanetCore::FlowRouteKnot> Knots;
		for(unsigned i = 0; i < 31; ++i) {
			const double Angle = i * 0.08;
			const CircleJet Point = SmallCircle(Angle);
			Knots.push_back({ Point.Position, Point.Tangent, SmallRadius * Angle });
		}
		const std::optional<PlanetCore::TerrainFlowRoute> Route = PlanetCore::TerrainFlowRoute::Create(Knots, Radius);
		// 有効な経路を準備出来ない場合の返戻
		if(!TestTrue(TEXT("複数区間の索引を準備出来る"), Route.has_value())) return false;
		for(const double Along : { 0.13, 0.53, 1.23, 2.07 }) for(const double Across : { -3.0, 0.0, 3.0 }) {
			const CircleJet Expected = SmallCircle(Along);
			const Vec3 Side = Expected.Position.Vector().Cross(Expected.Tangent);
			const UnitDirection Query =
			*UnitDirection::FromVector(Expected.Position.Vector() * std::cos(Across / Radius) + Side * std::sin(Across / Radius));
			std::uint32_t Candidates = 0;
			const std::optional<PlanetCore::FlowRouteProjection> Indexed = Route->FindFootpoint(Query, 6, &Candidates);
			// 通常の近傍を索引から照会出来ない場合の返戻
			if(!TestTrue(TEXT("経路の近傍を索引から照会出来る"), Indexed.has_value())) return false;
			double BestDistance = std::numeric_limits<double>::infinity();
			for(const PlanetCore::TerrainFlowSegment &Segment : Route->GetSegments()) {
				const std::optional<PlanetCore::TerrainFlowFootpoint> Foot = Segment.FindInteriorFootpoint(Query);
				if(Foot) BestDistance = std::min(BestDistance, Foot->SquaredChordDistance);
			}
			TestTrue(TEXT("索引が全区間走査と同じ最小距離を返す"), std::abs(Indexed->Footpoint.SquaredChordDistance - BestDistance) * Radius * Radius < 1E-8);
			TestTrue(TEXT("索引で候補区間を減らせる"), Candidates < Route->GetSegments().size());
			TestTrue(
				TEXT("経路の投影点が既知の小円から１㎝以内"),
				(Indexed->Footpoint.Curve.Direction.Vector() - Expected.Position.Vector()).Length() * Radius < 0.01
			);
			const std::optional<PlanetCore::TerrainFlowCurveSample> Recovered = Route->Sample(Indexed->DistanceMeters);
			TestTrue(
				TEXT("公称距離から同じ経路位置を回復出来る"),
				Recovered && (Recovered->Direction.Vector() - Indexed->Footpoint.Curve.Direction.Vector()).Length() * Radius < 1E-6
			);
		}
		std::uint32_t FarCandidates = 99;
		TestFalse(TEXT("反対の半球に候補を作らない"), Route->FindFootpoint(*UnitDirection::FromVector(Up * -1), 6, &FarCandidates).has_value());
		TestEqual(TEXT("遠方照会では直交点を解かない"), FarCandidates, 0U);
		TestFalse(TEXT("範囲外の経路距離を拒否"), Route->Sample(-1).has_value() || Route->Sample(Route->GetLengthMeters() + 1).has_value());
		TestFalse(TEXT("無効な検索幅を拒否"), Route->FindFootpoint(J0.Position, 0).has_value());
		Knots[1].DistanceMeters = Knots[0].DistanceMeters;
		TestFalse(TEXT("増加しない基準距離を拒否"), PlanetCore::TerrainFlowRoute::Create(Knots, Radius).has_value());
		PlanetCore::FlowChannelShape Shape;
		Shape.RadiusMeters = Radius;
		const Vec3 FieldVector = (East * 0.03 + North * 0.07) * Radius;
		const auto Source = [&](const UnitDirection Direction) -> PlanetCore::TerrainSample {
			const Vec3 D = Direction.Vector();
			// 補正処理から独立した傾斜場の返戻
			return { 1000 + FieldVector.Dot(D), FieldVector - D * FieldVector.Dot(D) };
		};
		const auto WaterAtFoot = [&](const PlanetCore::TerrainFlowFootpoint &Foot) -> PlanetCore::FlowChannelWaterLevel {
			const PlanetCore::TerrainSample Center = Source(Foot.Curve.Direction);
			// 独立な元場を水位候補にした返戻
			return { Center.ElevationMeters, Center.TangentialGradientPerRadian.Dot(Foot.Curve.DirectionDerivativePerMeter) };
		};
		// 小円の法線へずらした照会点で，独立な距離と陰関数微分を検査する
		for(const double Along : { 0.03, 0.08, 0.13 }) for(const double Across : { -9.0, -6.0, -3.0, 0.0, 3.0, 6.0, 9.0 }) {
			const CircleJet Expected = SmallCircle(Along);
			const Vec3 Side = Expected.Position.Vector().Cross(Expected.Tangent);
			const UnitDirection Query =
			*UnitDirection::FromVector(Expected.Position.Vector() * std::cos(Across / Radius) + Side * std::sin(Across / Radius));
			const std::optional<PlanetCore::TerrainFlowFootpoint> Foot = Curved->FindInteriorFootpoint(Query);
			// 正常な管状近傍の投影を取得出来ない場合の返戻
			if(!TestTrue(TEXT("小円の近傍で直交点を取得出来る"), Foot.has_value())) return false;
			const PlanetCore::TerrainSample Here = Source(Query);
			const std::optional<PlanetCore::FlowChannelSection> Section =
			PlanetCore::SampleFlowChannel(*Foot, Here, WaterAtFoot(*Foot), Shape);
			// 有効な投影から断面を取得出来ない場合の返戻
			if(!TestTrue(TEXT("正常な投影から有限幅の断面を取得出来る"), Section.has_value())) return false;
			TestTrue(
				TEXT("開削は元地形を上げず指定深さを超えない"),
				Section->Terrain.ElevationMeters <= Here.ElevationMeters &&
				Here.ElevationMeters - Section->Terrain.ElevationMeters <= Shape.DepthMeters + 1E-10
			);
			if(!Across) {
				TestTrue(TEXT("河床中心で指定深さになる"), std::abs(Here.ElevationMeters - Section->Terrain.ElevationMeters - Shape.DepthMeters) < 1E-6);
			}
			if(std::abs(Across) > Shape.HalfWidthMeters) {
				TestEqual(TEXT("支持領域外では元標高を保持"), Section->Terrain.ElevationMeters, Here.ElevationMeters);
				TestEqual(
					TEXT("支持領域外では元勾配を保持"),
					(Section->Terrain.TangentialGradientPerRadian - Here.TangentialGradientPerRadian).Length(),
					0.0
				);
			}
			const double PositionError = Radius * (Foot->Curve.Direction.Vector() - Expected.Position.Vector()).Length();
			const double DistanceError =
			std::abs(Radius * std::sqrt(Foot->SquaredChordDistance) - 2 * Radius * std::sin(std::abs(Across) / (2 * Radius)));
			TestTrue(TEXT("既知の小円上の直交点から２㎜以内"), PositionError < 0.002);
			TestTrue(TEXT("既知の距離から２㎜以内"), DistanceError < 0.002);
			MaximumFootpointError = std::max(MaximumFootpointError, PositionError);
			const Vec3 Q = Query.Vector();
			const Vec3 Axis = UnitDirection::FromVector(Expected.Tangent - Q * Expected.Tangent.Dot(Q))->Vector();
			for(const Vec3 Direction : { Axis, Q.Cross(Axis) }) {
				constexpr double Step = 0.01;
				const UnitDirection Low = *UnitDirection::FromVector(Q * std::cos(Step / Radius) - Direction * std::sin(Step / Radius));
				const UnitDirection High = *UnitDirection::FromVector(Q * std::cos(Step / Radius) + Direction * std::sin(Step / Radius));
				const std::optional<PlanetCore::TerrainFlowFootpoint> L = Curved->FindInteriorFootpoint(Low);
				const std::optional<PlanetCore::TerrainFlowFootpoint> H = Curved->FindInteriorFootpoint(High);
				// 独立差分の両端を取得出来ない場合の返戻
				if(!TestTrue(TEXT("両接線方向の差分用投影を取得出来る"), L && H)) return false;
				const std::optional<PlanetCore::FlowChannelSection> SL = PlanetCore::SampleFlowChannel(*L, Source(Low), WaterAtFoot(*L), Shape);
				const std::optional<PlanetCore::FlowChannelSection> SH =
				PlanetCore::SampleFlowChannel(*H, Source(High), WaterAtFoot(*H), Shape);
				// 元場を再評価した両端の断面を取得出来ない場合の返戻
				if(!TestTrue(TEXT("独立差分の両端で断面を取得出来る"), SL && SH)) return false;
				const double BedDifference = (SH->Terrain.ElevationMeters - SL->Terrain.ElevationMeters) / (2 * Step);
				const double WaterDifference = (SH->Water.ElevationMeters - SL->Water.ElevationMeters) / (2 * Step);
				const double BedError = std::abs(BedDifference - Section->Terrain.TangentialGradientPerRadian.Dot(Direction) / Radius);
				const double WaterError = std::abs(WaterDifference - Section->Water.TangentialGradientPerRadian.Dot(Direction) / Radius);
				TestTrue(TEXT("河床の解析勾配が独立差分と一致"), BedError < 0.001);
				TestTrue(TEXT("水位候補の解析勾配が独立差分と一致"), WaterError < 0.001);
				MaximumBedDerivativeError = std::max(MaximumBedDerivativeError, BedError);
				MaximumWaterDerivativeError = std::max(MaximumWaterDerivativeError, WaterError);
				const double PositionDifference = (H->Fraction - L->Fraction) * Curved->GetParameterLengthMeters() / (2 * Step);
				const double DistanceDifference = (H->SquaredChordDistance - L->SquaredChordDistance) * Radius * Radius / (2 * Step);
				const double PositionDerivative = Foot->ParameterGradientMetersPerRadian.Dot(Direction) / Radius;
				const double DistanceDerivative = Foot->SquaredChordGradientPerRadian.Dot(Direction) * Radius;
				const double PositionResidual = std::abs(PositionDifference - PositionDerivative);
				const double DistanceResidual = std::abs(DistanceDifference - DistanceDerivative);
				TestTrue(TEXT("公称位置の陰関数微分が独立差分と一致"), PositionResidual < 0.001);
				TestTrue(TEXT("距離二乗の勾配が独立差分と一致"), DistanceResidual < 0.001);
				MaximumFootpointDerivativeError = std::max(MaximumFootpointDerivativeError, PositionResidual);
				MaximumDistanceDerivativeError = std::max(MaximumDistanceDerivativeError, DistanceResidual);
				++FootpointSamples;
			}
		}
		for(const double Across : { -3.0, 3.0 }) {
			const Vec3 Side = J1.Position.Vector().Cross(J1.Tangent);
			const UnitDirection Query =
			*UnitDirection::FromVector(J1.Position.Vector() * std::cos(Across / Radius) + Side * std::sin(Across / Radius));
			const std::optional<PlanetCore::TerrainFlowFootpoint> L = Curved->FindInteriorFootpoint(Query);
			const std::optional<PlanetCore::TerrainFlowFootpoint> H = Following->FindInteriorFootpoint(Query);
			// 接合部の両側で同じ直交点を取得出来ない場合の返戻
			if(!TestTrue(TEXT("接合部の岸側を両区間から取得出来る"), L && H)) return false;
			const PlanetCore::FlowChannelSection SL = *PlanetCore::SampleFlowChannel(*L, Source(Query), WaterAtFoot(*L), Shape);
			const PlanetCore::FlowChannelSection SH = *PlanetCore::SampleFlowChannel(*H, Source(Query), WaterAtFoot(*H), Shape);
			TestTrue(
				TEXT("接合部で河床の標高と勾配が一致"),
				std::abs(SL.Terrain.ElevationMeters - SH.Terrain.ElevationMeters) < 1E-10 &&
				(SL.Terrain.TangentialGradientPerRadian - SH.Terrain.TangentialGradientPerRadian).Length() / Radius < 1E-8
			);
			TestTrue(
				TEXT("接合部で水位と其の勾配が一致"),
				std::abs(SL.Water.ElevationMeters - SH.Water.ElevationMeters) < 1E-10 &&
				(SL.Water.TangentialGradientPerRadian - SH.Water.TangentialGradientPerRadian).Length() / Radius < 1E-8
			);
			PlanetCore::FlowChannelShape Invalid = Shape;
			Invalid.DepthMeters = -1;
			TestFalse(TEXT("負の深さの断面を拒否"), PlanetCore::SampleFlowChannel(*L, Source(Query), WaterAtFoot(*L), Invalid).has_value());
			Invalid = Shape;
			Invalid.HalfWidthMeters = 1E-300;
			TestFalse(TEXT("表現範囲を超える断面を拒否"), PlanetCore::SampleFlowChannel(*L, Source(Query), WaterAtFoot(*L), Invalid).has_value());
			TestTrue(
				TEXT("接合部で公称位置の勾配が一致"),
				(L->ParameterGradientMetersPerRadian - H->ParameterGradientMetersPerRadian).Length() / Radius < 1E-8
			);
			TestTrue(
				TEXT("接合部で距離二乗の勾配が一致"),
				(L->SquaredChordGradientPerRadian - H->SquaredChordGradientPerRadian).Length() * Radius < 1E-8
			);
		}
		TestFalse(TEXT("区間の始点より手前の投影を押し込まない"), Curved->FindInteriorFootpoint(SmallCircle(-0.1).Position).has_value());
		TestFalse(TEXT("区間の終点より先の投影を押し込まない"), Curved->FindInteriorFootpoint(SmallCircle(0.3).Position).has_value());
		TestFalse(
			TEXT("非有限の曲率を拒否"),
			TerrainFlowSegment::CreateWithCurvature(
				J0.Position,
				J1.Position,
				J0.Tangent,
				J1.Tangent,
				{ std::numeric_limits<double>::quiet_NaN(), 0, 0 },
				J1.Curvature,
				Radius
			).has_value()
		);
		TestFalse(
			TEXT("区間の尺度に対して過大な曲率を拒否"),
			TerrainFlowSegment::CreateWithCurvature(
				J0.Position,
				J1.Position,
				J0.Tangent,
				J1.Tangent,
				J0.Curvature * 100,
				J1.Curvature,
				Radius
			).has_value()
		);
		// 許可経路とは別に，不正な位置と接線の拒否を確認する
		TestFalse(TEXT("範囲外の照会を拒否"), First->Evaluate(-0.01).has_value() || First->Evaluate(1.01).has_value());
		TestFalse(TEXT("非有限の照会を拒否"), First->Evaluate(std::numeric_limits<double>::quiet_NaN()).has_value());
		TestFalse(TEXT("零長の区間を拒否"), TerrainFlowSegment::Create(A.first, A.first, A.second, A.second, Radius).has_value());
		TestFalse(TEXT("逆向きの接線を拒否"), TerrainFlowSegment::Create(A.first, B.first, A.second * -1, B.second, Radius).has_value());
		TestFalse(TEXT("零接線を拒否"), TerrainFlowSegment::Create(A.first, B.first, {}, B.second, Radius).has_value());
		TestFalse(TEXT("不正半径を拒否"), TerrainFlowSegment::Create(A.first, B.first, A.second, B.second, 999).has_value());
		const std::pair<UnitDirection, Vec3> Far = Circle(Radius * 0.2);
		TestFalse(TEXT("長過ぎる単一区間を拒否"), TerrainFlowSegment::Create(A.first, Far.first, A.second, Far.second, Radius).has_value());
	}
	// 既知の下降放物線で，上流の静水接続・余裕高・下流の静水接続を通して調べる
	PlanetCore::FlowChannelWaterSettings WaterSettings;
	WaterSettings.DownstreamWaterMeters = 1;
	const auto Profile = [&](const double S) -> std::optional<PlanetCore::FlowChannelWaterLevel> {
		// 解析的な下降縦断から作る水位候補の返戻
		return PlanetCore::SampleFlowWaterLevel({ 10 - 0.001 * S * S, -0.002 * S }, S, WaterSettings);
	};
	double PreviousWater = 10, MaximumProfileDerivativeError = 0;
	for(unsigned i = 0; i < 2401; ++i) {
		const double S = i * 0.05;
		const std::optional<PlanetCore::FlowChannelWaterLevel> Level = Profile(S);
		// 正常な下降縦断を受理しない場合の返戻
		if(!TestTrue(TEXT("正常な下降縦断から連続水位を構成出来る"), Level.has_value())) return false;
		TestTrue(TEXT("水位と其の微分が下流へ非増加"), Level->ElevationMeters <= PreviousWater + 1E-12 && !(Level->DerivativePerMeter > 0));
		TestTrue(TEXT("下流の静水位を割らない"), Level->ElevationMeters > 1 - 1E-12);
		PreviousWater = Level->ElevationMeters;
		if(i && i < 2400) {
			constexpr double Step = 1E-4;
			const double Difference = (Profile(S + Step)->ElevationMeters - Profile(S - Step)->ElevationMeters) / (2 * Step);
			const double Error = std::abs(Difference - Level->DerivativePerMeter);
			TestTrue(TEXT("水位の縦断微分が独立差分と一致"), Error < 1E-6);
			MaximumProfileDerivativeError = std::max(MaximumProfileDerivativeError, Error);
		}
	}
	TestEqual(TEXT("上流の接続水位を保持"), Profile(0)->ElevationMeters, 10.0);
	TestEqual(TEXT("上流の静水面と接線が一致"), Profile(0)->DerivativePerMeter, 0.0);
	TestEqual(TEXT("下流の静水位へ一致"), Profile(120)->ElevationMeters, 1.0);
	TestEqual(TEXT("下流の静水面と接線が一致"), Profile(120)->DerivativePerMeter, 0.0);
	TestFalse(TEXT("上昇する元縦断を拒否"), PlanetCore::SampleFlowWaterLevel({ 10, 0.1 }, 0, WaterSettings).has_value());
	WaterSettings.RampLengthMeters = 0;
	TestFalse(TEXT("零長の水位遷移を拒否"), PlanetCore::SampleFlowWaterLevel({ 10, -0.1 }, 0, WaterSettings).has_value());
	AddInfo(
		FString::Printf(TEXT("Flow water profile contracts: samples=2401 derivativeError=%.12g"), MaximumProfileDerivativeError)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow curve contracts: samples=%u planeError=%.12g derivativeError=%.12g curveResidual=%.12g joinError=%.12g"),
			Samples,
			MaximumPlaneError,
			MaximumDerivativeError,
			MaximumCurveResidual,
			MaximumJoinError
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow curvature contracts: joinError=%.12g secondDerivativeError=%.12g"),
			MaximumCurvatureJoinError,
			MaximumSecondDerivativeError
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow footpoint contracts: samples=%u positionError=%.12g parameterDerivativeError=%.12g distanceDerivativeError=%.12g"),
			FootpointSamples,
			MaximumFootpointError,
			MaximumFootpointDerivativeError,
			MaximumDistanceDerivativeError
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow channel contracts: bedDerivativeError=%.12g waterDerivativeError=%.12g"),
			MaximumBedDerivativeError,
			MaximumWaterDerivativeError
		)
	);
	// 独立な幾何・微分・入力条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
