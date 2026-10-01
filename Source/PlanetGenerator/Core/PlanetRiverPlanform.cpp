#include "PlanetRiverPlanform.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>

namespace PlanetCore {
	namespace {
		/**
		 * 位相用の固定幅乱数語を混合する
		 * @param Value Seed と地点番号から構成した入力
		 * @return SplitMix64 の最終化に依る値
		 */
		std::uint64_t Mix(std::uint64_t Value) {
			Value += UINT64_C(0X9E3779B97F4A7C15);
			Value = (Value ^ (Value >> 30)) * UINT64_C(0XBF58476D1CE4E5B9);
			Value = (Value ^ (Value >> 27)) * UINT64_C(0X94D049BB133111EB);
			// 決定論的な混合語の返戻
			return Value ^ Value >> 31;
		}

		/**
		 * 混合した５３ビットから位相を得る
		 * @param Value 位相を定める入力語
		 * @return ０以上２ π 未満の位相
		 */
		double PhaseAngle(const std::uint64_t Value) {
			// 半開区間の位相の返戻
			return static_cast<double>(Mix(Value) >> 11) * (2 * std::numbers::pi_v<double> / 9007199254740992.0);
		}
	}

	/**
	 * 物理川幅と原弧長から，端点を動かさない有限振幅の蛇行を準備する
	 * @param First 上流の接続地点
	 * @param Last 下流の接続地点
	 * @param RadiusMeters 惑星の半径
	 * @param BedHalfWidthMeters 河床の半幅
	 * @param Seed 河道の位相を定める固定語
	 * @return 有効な曲線，退化した弧又は非有限の条件では空
	 */
	std::optional<RiverPlanform> RiverPlanform::Create(
		const UnitDirection First,
		const UnitDirection Last,
		const double RadiusMeters,
		const double BedHalfWidthMeters,
		const std::uint64_t Seed
	) {
		const bool Valid = std::isfinite(RadiusMeters) && RadiusMeters >= 1000 && (RadiusMeters < 1E9 || RadiusMeters == 1E9) &&
		std::isfinite(BedHalfWidthMeters) && BedHalfWidthMeters > 0;
		// 対応範囲を持たない入力の返戻
		if(!Valid) return {};
		RiverPlanform Result;
		Result.Start = First.Vector();
		const Vec3 End = Last.Vector();
		Result.Arc = std::atan2(Result.Start.Cross(End).Length(), Result.Start.Dot(End));
		const std::optional<UnitDirection> Tangent =
		UnitDirection::FromVector(Result.Start.Cross(End - Result.Start).Cross(Result.Start));
		// 退化又は対向半球に跨る弧の返戻
		if(!Tangent || !Result.Arc || Result.Arc >= std::numbers::pi_v<double> * 0.5) return {};
		Result.Along = Tangent->Vector();
		Result.Across = Result.Start.Cross(Result.Along);
		Result.Amplitude = std::min(2 * BedHalfWidthMeters / RadiusMeters, 0.0125 * Result.Arc);
		Result.Frequency =
		2 * std::numbers::pi_v<double> * std::clamp(Result.Arc * RadiusMeters / std::max(150.0, 30 * BedHalfWidthMeters), 1.0, 32.0);
		Result.Phase = PhaseAngle(Seed);
		Result.SecondaryPhase = PhaseAngle(Seed ^ UINT64_C(0XD1B54A32D192ED03));
		// 位置と二階微分迄を端点へ接続する曲線の返戻
		return Result;
	}

	/**
	 * 両端で二階迄が零となる横方向の変位を求める
	 * @param Fraction 元の弧で正規化した位置
	 * @return 変位と，正規化位置に対する一階・二階微分
	 */
	std::array<double, 3> RiverPlanform::Offset(const double Fraction) const {
		// 端点と区間外の無変位の返戻
		if(Fraction <= 0 || Fraction >= 1) return {};
		const double T = Fraction, U = 1 - T, TU = T * U, Envelope = 64 * TU * TU * TU, EnvelopeSlope = 192 * TU * TU * (1 - 2 * T);
		const double EnvelopeCurvature = 384 * TU * ((1 - 2 * T) * (1 - 2 * T) - TU), A = Frequency * T + Phase;
		const double B = 1.73 * Frequency * T + SecondaryPhase, Wave = std::sin(A) + 0.27 * std::sin(B);
		const double Slope = Frequency * (std::cos(A) + 0.4671 * std::cos(B));
		const double Curvature = -Frequency * Frequency * (std::sin(A) + 0.808083 * std::sin(B)), Scale = Amplitude / 1.27;
		// 厳密な積の微分を含む横変位の返戻
		return {
			Scale * Envelope * Wave,
			Scale * (EnvelopeSlope * Wave + Envelope * Slope),
			Scale * (EnvelopeCurvature * Wave + 2 * EnvelopeSlope * Slope + Envelope * Curvature)
		};
	}

	/**
	 * 同じ蛇行を球面の位置と接線微分へ写す
	 * @param Fraction 元の弧の正規化位置，区間外は大円弧の延長
	 * @return 単位方向と，正規化位置に対する一階・二階微分
	 */
	RiverPlanformSample RiverPlanform::Evaluate(const double Fraction) const {
		const double Angle = Fraction * Arc;
		const std::array<double, 3> Bend = Offset(Fraction);
		const double Cosine = std::cos(Angle), Sine = std::sin(Angle), C = std::cos(Bend[0]), S = std::sin(Bend[0]);
		const Vec3 Base = Start * Cosine + Along * Sine, BaseSlope = Along * Cosine - Start * Sine;
		const Vec3 Point = Base * C + Across * S, Lateral = Across * C - Base * S;
		const Vec3 Velocity = BaseSlope * (C * Arc) + Lateral * Bend[1];
		const Vec3 Acceleration =
		Base * (-C * Arc * Arc) - BaseSlope * (2 * S * Arc * Bend[1]) + Lateral * Bend[2] - Point * (Bend[1] * Bend[1]);
		// 正規化で除いた微小な半径成分も一階微分へ反映する
		const UnitDirection Direction = *UnitDirection::FromVector(Point);
		const Vec3 D = Direction.Vector();
		// 同じ曲線の位置と接線微分の返戻
		return { Direction, (Velocity - D * D.Dot(Velocity)) * (1 / Point.Length()), Acceleration };
	}

	/**
	 * 横断評価と同じ弧角から曲線を評価する
	 * @param Angle 原弧の上流からの角度
	 * @return 同じ正規化位置の曲線標本
	 */
	RiverPlanformSample RiverPlanform::EvaluateAngle(const double Angle) const {
		// 同じ角度座標で評価した曲線の返戻
		return Evaluate(Angle / Arc);
	}

	/**
	 * 縦断の公称座標に使う原弧角を取得する
	 * @return 正の有限な弧角
	 */
	double RiverPlanform::ArcRadians() const {
		return Arc;
	}

	/**
	 * 元の大円弧からの最大離隔を取得する
	 * @return 球面角の保守的な振幅上限
	 */
	double RiverPlanform::MaximumDeviationRadians() const {
		return Amplitude;
	}

	/**
	 * 角度座標から曲線への微分長の保守的な上限を得る
	 * @return 曲線区間の物理的な挟込み幅を覆う Lipschitz 上限
	 */
	double RiverPlanform::MaximumTangentStretch() const {
		// 包絡微分と二つの周波数の振幅上限を合成した返戻
		return 1 + Amplitude * (12 + 1.4671 * Frequency) / (1.27 * Arc);
	}

	/**
	 * 曲線と三角形の交差区間が，現在の線形水域に覆われる事を保守的に確認する
	 * @param EndAngle 海岸で切詰めた公称終端角度
	 * @param Triangle 正向きで同じ半球に在る三角形
	 * @param Depths 共通参照球面へ射影した頂点水深
	 * @return 交差しない又は全交差区間が水域内の場合に真，未確定は偽
	 * @note 元の連続地形の全水域を証明する条件ではない
	 */
	bool RiverPlanform::IsCoveredByLinearWater(
		const double EndAngle,
		const std::array<UnitDirection, 3> &Triangle,
		const std::array<double, 3> &Depths
	) const {
		const bool Invalid = !std::isfinite(EndAngle) || EndAngle <= 0 || EndAngle > Arc || !std::all_of(
			Depths.begin(),
			Depths.end(),
			[](const double D) -> bool {
				return std::isfinite(D);
			}
		);
		// 範囲外の曲線又は非有限な水深を被覆済みとしない返戻
		if(Invalid) return false;
		const Vec3 A = Triangle[0].Vector(), B = Triangle[1].Vector(), C = Triangle[2].Vector();
		const Vec3 U = B - A, V = C - A, Cross = U.Cross(V);
		const double Area = Cross.Length();
		// 退化した三角形を被覆済みとしない返戻
		if(!std::isfinite(Area) || Area <= 0) return false;
		const Vec3 N = Cross * (1 / Area);
		const double PlaneRadius = N.Dot(A);
		// 逆向き又は球心を跨ぐ射影を被覆済みとしない返戻
		if(PlaneRadius <= 0 || !std::isfinite(PlaneRadius)) return false;
		const bool WholeWet = std::all_of(
			Depths.begin(),
			Depths.end(),
			[](const double D) -> bool {
				return D > 0;
			}
		);
		// 凸な線形水域が三角形全体を覆う場合の返戻
		if(WholeWet) return true;
		std::array<Vec3, 3> Edges;
		for(unsigned K = 0; K < 3; ++K) {
			const Vec3 First = Triangle[K].Vector(), Last = Triangle[(K + 1) % 3].Vector();
			const std::optional<UnitDirection> Edge = UnitDirection::FromVector(First.Cross(Last - First));
			// 非退化な内向きの半空間を持たない三角形の返戻
			if(!Edge) return false;
			Edges[K] = Edge->Vector();
		}
		// 弦面への中心射影で，水深の符号を表す同次平面を構成する
		const Vec3 Gradient = (V.Cross(N) * (Depths[1] - Depths[0]) + N.Cross(U) * (Depths[2] - Depths[0])) * (1 / Area);
		const double Intercept = Depths[0] - Gradient.Dot(A - N * PlaneRadius);
		const std::optional<UnitDirection> Wet = UnitDirection::FromVector(Gradient + N * (Intercept / PlaneRadius));
		// 零又は非有限な水深平面を，正の被覆としない返戻
		if(!Wet) return false;
		constexpr double Roundoff = 64 * std::numeric_limits<double>::epsilon();
		double Low = 0, High = EndAngle / Arc;
		bool PositiveHemisphere = true;
		double Minimum = std::numeric_limits<double>::infinity(), Maximum = -Minimum;
		for(const UnitDirection D : Triangle) {
			const double X = D.Vector().Dot(Start), Y = D.Vector().Dot(Along);
			if(X <= 0) PositiveHemisphere = false;
			const double Angle = std::atan2(Y, X);
			Minimum = std::min(Minimum, Angle);
			Maximum = std::max(Maximum, Angle);
		}
		// 正の半球では三角形内の公称角度が頂点の角度範囲に入る
		if(PositiveHemisphere) {
			Low = std::max(Low, (Minimum - Roundoff) / Arc);
			High = std::min(High, (Maximum + Roundoff) / Arc);
		}
		// 公称角度が交差しない曲線の返戻
		if(High < Low) return true;
		struct Interval {
			double Low;
			double High;
			unsigned Depth;
		};
		constexpr unsigned MaximumDepth = 32, MaximumVisits = 512;
		std::array<Interval, MaximumDepth + 1> Pending;
		unsigned Count = 1, Visits = 0;
		Pending[0] = { Low, High, 0 };
		const double Stretch = Arc * MaximumTangentStretch();
		while(Count) {
			// 有界な確認で証明出来ない区間は発見用の細分化へ残す返戻
			if(Visits++ >= MaximumVisits) return false;
			const Interval I = Pending[--Count];
			const double Mid = (I.Low + I.High) * 0.5;
			const Vec3 P = Evaluate(Mid).Direction.Vector();
			const double Bound = Stretch * (I.High - I.Low) * 0.5 + Roundoff;
			bool Outside = false, Inside = true;
			for(const Vec3 Edge : Edges) {
				const double S = Edge.Dot(P);
				if(S + Bound < -Roundoff) Outside = true;
				if(S < Roundoff) Inside = false;
			}
			if(Outside) continue;
			const double Water = Wet->Vector().Dot(P);
			if(Water - Bound > Roundoff) continue;
			// 実際の曲線中点が三角形内の乾いた部分へ入る場合の返戻
			if(Inside && Water <= 0) return false;
			// 区間又は深度がこれ以上表現出来ない場合の返戻
			if(I.Depth == MaximumDepth || Mid == I.Low || Mid == I.High || Count + 2 > Pending.size()) return false;
			Pending[Count++] = { Mid, I.High, I.Depth + 1 };
			Pending[Count++] = { I.Low, Mid, I.Depth + 1 };
		}
		// 全区間が外側又は正の線形水域へ入る被覆の返戻
		return true;
	}
}
