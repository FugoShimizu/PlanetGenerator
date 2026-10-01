#include "PlanetFlowCurve.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace PlanetCore {
	/**
	 * 検査済の球面 Hermite 区間を不変な係数として保持する
	 * @param First 始点
	 * @param Last 終点
	 * @param FirstVelocity 正規化前の始点微分
	 * @param LastVelocity 正規化前の終点微分
	 * @param Length 端点間大円距離による公称の区間長
	 */
	TerrainFlowSegment::TerrainFlowSegment(
		const UnitDirection First,
		const UnitDirection Last,
		const Vec3 FirstVelocity,
		const Vec3 LastVelocity,
		const double Length
	) : Start(First), End(Last), StartVelocity(FirstVelocity), EndVelocity(LastVelocity), ParameterLengthMeters(Length) {
		// 終了
		return;
	}

	/**
	 * 端点の下降接線を保持した正規化三次 Hermite 曲線を作る
	 * 計算量 O(1)，準備後の照会は元場の再評価と割当を必要としない
	 * @param First 球面上の始点
	 * @param Last 球面上の終点
	 * @param FirstTangent 始点の下流方向，長さは任意だが有限且つ非零
	 * @param LastTangent 終点の下流方向，長さは任意だが有限且つ非零
	 * @param Radius 1000～1000000000 m の惑星半径
	 * @return 短い正向きの区間；不正入力・逆向き・零長・半径の１割以上の長さは空
	 */
	std::optional<TerrainFlowSegment> TerrainFlowSegment::Create(
		const UnitDirection First,
		const UnitDirection Last,
		const Vec3 FirstTangent,
		const Vec3 LastTangent,
		const double Radius
	) {
		// 有効な物理尺度以外への返戻
		if(!std::isfinite(Radius) || Radius < 1000 || Radius > 1E9) return {};
		const Vec3 A = First.Vector(), B = Last.Vector(), Delta = B - A;
		const double Angle = std::atan2(A.Cross(B).Length(), A.Dot(B)), Length = Angle * Radius;
		// 退化区間又は単一の局所曲線の範囲外への返戻
		if(!(Length > 1E-6) || !(Angle < 0.1)) return {};
		const std::optional<UnitDirection> T0 = UnitDirection::FromVector(FirstTangent - A * FirstTangent.Dot(A));
		const std::optional<UnitDirection> T1 = UnitDirection::FromVector(LastTangent - B * LastTangent.Dot(B));
		// 零・非有限・端点間の進行に逆らう接線への返戻
		if(!T0 || !T1 || !(T0->Vector().Dot(Delta) > 0) || !(T1->Vector().Dot(Delta) > 0)) return {};
		// 公称距離に対する端点微分を共有する区間の返戻
		return TerrainFlowSegment(First, Last, T0->Vector() * Angle, T1->Vector() * Angle, Length);
	}

	/**
	 * 両端の測地曲率も共有する正規化五次曲線を作る
	 * @param First 始点
	 * @param Last 終点
	 * @param FirstTangent 始点の下流接線
	 * @param LastTangent 終点の下流接線
	 * @param FirstCurvature 始点の単位接線のメートル当たり変化，球面内の直交成分を用いる
	 * @param LastCurvature 終点の単位接線のメートル当たり変化
	 * @param Radius 惑星半径
	 * @return 位置・物理接線・曲率を共有する区間；非有限又は端点曲率と区間長の積が１以上なら空
	 */
	std::optional<TerrainFlowSegment> TerrainFlowSegment::CreateWithCurvature(
		const UnitDirection First,
		const UnitDirection Last,
		const Vec3 FirstTangent,
		const Vec3 LastTangent,
		const Vec3 FirstCurvature,
		const Vec3 LastCurvature,
		const double Radius
	) {
		std::optional<TerrainFlowSegment> Result = Create(First, Last, FirstTangent, LastTangent, Radius);
		// 基本の位置と接線が不正な場合の返戻
		if(!Result) return {};
		const std::array<Vec3, 2> Normals { First.Vector(), Last.Vector() }, Velocities { Result->StartVelocity, Result->EndVelocity };
		const std::array<Vec3, 2> Curvatures { FirstCurvature, LastCurvature };
		std::array<Vec3, 2> Accelerations;
		const double Length = Result->ParameterLengthMeters, Angle = Length / Radius;
		for(unsigned i = 0; i < 2; ++i) {
			// 有限でない曲率を射影で隠さない返戻
			if(!std::isfinite(Curvatures[i].Length())) return {};
			const Vec3 Tangent = UnitDirection::FromVector(Velocities[i])->Vector();
			const Vec3 Geodesic = Curvatures[i] - Normals[i] * Curvatures[i].Dot(Normals[i]) - Tangent * Curvatures[i].Dot(Tangent);
			// 区間長に対して過大な曲率を許可しない返戻
			if(!(Geodesic.Length() * Length < 1)) return {};
			Accelerations[i] = (Geodesic * Length - Normals[i] * Angle) * Angle;
		}
		Result->StartAcceleration = Accelerations[0];
		Result->EndAcceleration = Accelerations[1];
		Result->HasSharedCurvature = true;
		// 正規化の半径方向の加速度も保持した曲線の返戻
		return Result;
	}

	/**
	 * 球面上の連続曲線と其の解析微分を評価する
	 * @param Fraction 区間の正規化位置，０～１
	 * @return 球面方向と公称メートルに対する一次・二次微分；範囲外又は非有限の引数は空
	 */
	std::optional<TerrainFlowCurveSample> TerrainFlowSegment::Evaluate(const double Fraction) const {
		// 範囲外の位置を暗黙に丸めない返戻
		if(!std::isfinite(Fraction) || Fraction < 0 || Fraction > 1) return {};
		// 始点からの小さい変位として評価する多項式の係数
		const Vec3 Delta = End.Vector() - Start.Vector();
		Vec3 C2 = Delta * 3 - StartVelocity * 2 - EndVelocity;
		Vec3 C3 = Delta * -2 + StartVelocity + EndVelocity, C4 {}, C5 {};
		if(HasSharedCurvature) {
			C2 = StartAcceleration * 0.5;
			C3 = Delta * 10 - StartVelocity * 6 - EndVelocity * 4 - StartAcceleration * 1.5 + EndAcceleration * 0.5;
			C4 = Delta * -15 + StartVelocity * 8 + EndVelocity * 7 + StartAcceleration * 1.5 - EndAcceleration;
			C5 = Delta * 6 - StartVelocity * 3 - EndVelocity * 3 - StartAcceleration * 0.5 + EndAcceleration * 0.5;
		}
		const double T = Fraction;
		const Vec3 Position = Start.Vector() + (StartVelocity + (C2 + (C3 + (C4 + C5 * T) * T) * T) * T) * T;
		const Vec3 Velocity = StartVelocity + (C2 * 2 + (C3 * 3 + (C4 * 4 + C5 * (5 * T)) * T) * T) * T;
		const Vec3 Acceleration = C2 * 2 + (C3 * 6 + (C4 * 12 + C5 * (20 * T)) * T) * T;
		const UnitDirection Direction = !T ? Start : T == 1 ? End : *UnitDirection::FromVector(Position);
		const Vec3 Normal = Direction.Vector();
		const double Norm = Position.Length(), RadialSpeed = Normal.Dot(Velocity);
		const Vec3 First = (Velocity - Normal * RadialSpeed) * (1 / Norm);
		const Vec3 Second =
		(Acceleration - Normal * Normal.Dot(Acceleration) - First * (2 * RadialSpeed)) * (1 / Norm) - Normal * First.Dot(First);
		const double InverseLength = 1 / ParameterLengthMeters;
		// 正規化写像の一次・二次微分を物理単位へ戻した返戻
		return TerrainFlowCurveSample{ Direction, First * InverseLength, Second * (InverseLength * InverseLength) };
	}

	/**
	 * 短い区間内で直交条件を満たす局所的な距離の極小を探す
	 * 自己接近する複数区間の最近点選択は呼出側の責務
	 * @param Query 照会する球面方向
	 * @return 区間内の直交点と距離・公称位置の勾配；範囲外・特異点・未収束は空
	 */
	std::optional<TerrainFlowFootpoint> TerrainFlowSegment::FindInteriorFootpoint(const UnitDirection Query) const {
		const Vec3 D = Query.Vector();
		const double Radius = ParameterLengthMeters / StartVelocity.Length();
		const double Tolerance = std::max(1E-6, 32 * std::numeric_limits<double>::epsilon() * Radius);
		const auto Residual = [&](const TerrainFlowCurveSample &Sample) -> double {
			// 惑星規模の内積の相殺を避けた直交条件の返戻
			return (Sample.Direction.Vector() - D).Dot(Sample.DirectionDerivativePerMeter);
		};
		const TerrainFlowCurveSample A = *Evaluate(0), B = *Evaluate(1);
		const double GA = Residual(A), GB = Residual(B), AScale = A.DirectionDerivativePerMeter.Dot(A.DirectionDerivativePerMeter);
		const double BScale = B.DirectionDerivativePerMeter.Dot(B.DirectionDerivativePerMeter);
		// 区間の外側に在る投影を端点へ押し込まない返戻
		if(GA > Tolerance * AScale || GB < -Tolerance * BScale) return {};
		double Low = 0, High = 1, T = std::abs(GA) <= Tolerance * AScale ? 0 : std::abs(GB) <= Tolerance * BScale ? 1 : 0.5;
		for(unsigned Iteration = 0; Iteration < 64; ++Iteration) {
			const TerrainFlowCurveSample Value = *Evaluate(T);
			const Vec3 Offset = Value.Direction.Vector() - D, Velocity = Value.DirectionDerivativePerMeter;
			const double SpeedSquared = Velocity.Dot(Velocity), G = Offset.Dot(Velocity);
			const double Hessian = SpeedSquared + Offset.Dot(Value.DirectionSecondDerivativePerMeterSquared);
			// 距離の極大又は焦点に近い特異な逆写像を採用しない返戻
			if(!(Hessian > SpeedSquared * 1E-6)) return {};
			if(std::abs(G) <= Tolerance * SpeedSquared) {
				const Vec3 Difference = D - Value.Direction.Vector();
				const Vec3 DistanceGradient = (Difference - D * Difference.Dot(D)) * 2;
				const Vec3 ParameterGradient = (Velocity - D * Velocity.Dot(D)) * (1 / Hessian);
				// 陰関数の微分を含む局所座標の返戻
				return TerrainFlowFootpoint{ Value, T, Difference.Dot(Difference), DistanceGradient, ParameterGradient };
			}
			if(G < 0) Low = T;
			else High = T;
			const double Newton = T - G / (Hessian * ParameterLengthMeters);
			T = Newton > Low && Newton < High ? Newton : (Low + High) * 0.5;
		}
		// 反復予算内で直交条件を満たせなかった返戻
		return {};
	}

	/**
	 * 端点間の大円距離で定義した公称区間長を返す
	 * @return メートル単位のパラメータ長；湾曲した経路の厳密な弧長ではない
	 */
	double TerrainFlowSegment::GetParameterLengthMeters() const {
		// 公称の区間長の返戻
		return ParameterLengthMeters;
	}
}
