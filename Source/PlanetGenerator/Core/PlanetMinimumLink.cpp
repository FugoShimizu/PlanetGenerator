#include "PlanetMinimumLink.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <optional>
#include <span>

namespace PlanetCore {
	/**
	 * 停留点直前の標高差の桁落ちを避け，解析勾配で短区間の下降を照合する
	 * @param Last 下降追跡の最終標本
	 * @param Minimum 同じ元場で精査した局所極小
	 * @param Source 元の標高と解析接線勾配
	 * @param Stop 中断要求
	 * @return 公開端点の標高と，２刻み幅の局所積分が下降を示す場合に真
	 */
	bool DescendsToTerrainMinimum(
		const TerrainDescentSample &Last,
		const TerrainCriticalPoint &Minimum,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::stop_token Stop
	) {
		// 局所極小との接続でない依存条件への返戻
		if(!Source || Minimum.Kind != TerrainCriticalKind::Minimum || Stop.stop_requested()) return false;
		// 元標高を比較出来ない非有限値への返戻
		if(!std::isfinite(Minimum.ElevationMeters) || !std::isfinite(Last.ElevationMeters)) return false;
		// 公開する標高の上昇を，微分による検査で隠さない返戻
		if(Minimum.ElevationMeters > Last.ElevationMeters) return false;
		const Vec3 A = Last.Direction.Vector(), B = Minimum.Direction.Vector(), Delta = B - A;
		// 局所区間の正規化不能な対蹠点への返戻
		if(A.Dot(B) <= 0.0) return false;
		std::array<double, 17> Derivatives;
		for(unsigned I = 0; I < Derivatives.size(); ++I) {
			// 評価途中の中断への返戻
			if(Stop.stop_requested()) return false;
			const double Fraction = I / 16.0;
			const Vec3 P = A * (1 - Fraction) + B * Fraction;
			const UnitDirection D = !I ? Last.Direction : I == 16 ? Minimum.Direction : *UnitDirection::FromVector(P);
			const TerrainSample Value = Source(D);
			// 差分の代替検査でも元場の欠落を受理しない返戻
			if(!std::isfinite(Value.ElevationMeters) || !std::isfinite(Value.TangentialGradientPerRadian.Length())) return false;
			const Vec3 Unit = D.Vector();
			// 正規化補間の連鎖律に依る，区間パラメータに対する標高微分
			Derivatives[I] = Value.TangentialGradientPerRadian.Dot(Delta - Unit * Unit.Dot(Delta)) / P.Length();
			// 極小端点以外で測定した実際の上りを拒否する返戻
			if(!std::isfinite(Derivatives[I]) || I < 16 && Derivatives[I] > 0) return false;
		}
		for(unsigned I = 0; I < 16; I += 4) {
			// 同じ４分割区間で，Simpson 積分の刻みを半減して符号を照合
			const double Coarse = (Derivatives[I] + 4 * Derivatives[I + 2] + Derivatives[I + 4]) / 24;
			const double Fine =
			(Derivatives[I] + 4 * Derivatives[I + 1] + 2 * Derivatives[I + 2] + 4 * Derivatives[I + 3] + Derivatives[I + 4]) / 48;
			// 刻み幅の差で符号が変わる区間を下降として受理しない返戻
			if(Fine + std::abs(Fine - Coarse) > 0) return false;
		}
		// 元の標高を変更しない局所接続の検査結果の返戻
		return !Stop.stop_requested();
	}

	/**
	 * 確認済極小へ繋ぐ末尾だけを，同じ位置精度内で元場から再準備する
	 * 計算量：最大２４回の局所位置照合と，二つの１７点微分検査；追加空間 O(1)
	 * @param Path 解析勾配で追跡した元の下降標本，少なくとも一つ必要
	 * @param Minimum 同じ元場の局所精査で確認した極小
	 * @param RadiusMeters 元の球面の物理半径
	 * @param PositionToleranceMeters 終端の既存の位置精度；拡大しない
	 * @param Source 同じ元標高と解析接線勾配
	 * @param Stop 中断要求
	 * @return 保持する前半の個数と実標高を持つ末尾；下降未確認では空で入力も不変
	 */
	std::optional<TerrainMinimumLink> PrepareTerrainMinimumLink(
		const std::span<const TerrainDescentSample> Path,
		const TerrainCriticalPoint &Minimum,
		const double RadiusMeters,
		const double PositionToleranceMeters,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::stop_token Stop
	) {
		const bool HasInput =
		Source && !Path.empty() && Path.size() < 65537 && Minimum.Kind == TerrainCriticalKind::Minimum && std::isfinite(RadiusMeters) &&
		RadiusMeters >= 1000 && RadiusMeters <= 1E9 && std::isfinite(PositionToleranceMeters) && PositionToleranceMeters >= 0.001 &&
		PositionToleranceMeters <= 0.1;
		// 確認範囲を変更する入力又は開始前の中断への返戻
		if(!HasInput || Stop.stop_requested()) return {};
		const auto Distance = [&](const UnitDirection A, const UnitDirection B) -> double {
			const Vec3 U = A.Vector(), V = B.Vector();
			// 同じ球面上の実弧長の返戻
			return RadiusMeters * std::atan2(U.Cross(V).Length(), U.Dot(V));
		};
		const TerrainDescentSample &Last = Path.back();
		// 既存標本の非有限な累積距離を新しい接続へ持越さない返戻
		if(!std::isfinite(Last.DistanceMeters) || Last.DistanceMeters < 0) return {};
		// 位置精度外の極小を，末尾の差替で近い終端へ読み替えない返戻
		if(Distance(Last.Direction, Minimum.Direction) > PositionToleranceMeters) return {};
		// 通常の下降接続は実標本も保持数も変更しない返戻
		if(DescendsToTerrainMinimum(Last, Minimum, Source, Stop)) return TerrainMinimumLink{ Last, Path.size() - 1 };
		// 直前の有効な辺を持たない経路への返戻
		if(Path.size() < 2 || Stop.stop_requested()) return {};
		const TerrainDescentSample &Previous = Path[Path.size() - 2];
		const bool HasPrevious =
		Previous.DistanceMeters < Last.DistanceMeters && Previous.Direction.Vector().Dot(Last.Direction.Vector()) > 0.0;
		// 累積距離の不整合又は局所補間不能な辺への返戻
		if(!HasPrevious) return {};
		UnitDirection Anchor = Previous.Direction;
		if(Distance(Anchor, Minimum.Direction) > PositionToleranceMeters) {
			// 元の末尾辺で，確認済極小の位置精度内に入る最初の点を絞る
			double Outside = 0, Inside = 1;
			for(unsigned Iteration = 0; Iteration < 24; ++Iteration) {
				// 位置だけの照合中も中断を保持する返戻
				if(Stop.stop_requested()) return {};
				const double Fraction = std::midpoint(Outside, Inside);
				const UnitDirection Candidate =
				*UnitDirection::FromVector(Previous.Direction.Vector() * (1 - Fraction) + Last.Direction.Vector() * Fraction);
				if(Distance(Candidate, Minimum.Direction) <= PositionToleranceMeters) {
					Inside = Fraction;
					Anchor = Candidate;
				} else Outside = Fraction;
			}
		}
		// 絞込みが有効な実位置を得なかった場合の返戻
		if(Distance(Anchor, Minimum.Direction) > PositionToleranceMeters) return {};
		const TerrainSample Value = Source(Anchor);
		const TerrainDescentSample Point {
			Anchor,
			Value.ElevationMeters,
			Previous.DistanceMeters + Distance(Previous.Direction, Anchor)
		};
		const bool HasOrderedHeights =
		std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length()) &&
		Minimum.ElevationMeters <= Point.ElevationMeters && Point.ElevationMeters <= Previous.ElevationMeters;
		// 末尾の実標高を修正せず，公開する前後の標高が下降する場合だけ採用
		if(!HasOrderedHeights || Stop.stop_requested() || !DescendsToTerrainMinimum(Point, Minimum, Source, Stop)) return {};
		if(Distance(Previous.Direction, Anchor) > 0) {
			// 元辺の切詰めで新しい登坂区間を作らない；終端も含めた解析微分を確認
			const Vec3 A = Previous.Direction.Vector(), B = Anchor.Vector(), Delta = B - A;
			for(unsigned I = 0; I < 17; ++I) {
				// 元辺の短縮を確認する途中の中断への返戻
				if(Stop.stop_requested()) return {};
				const double Fraction = I / 16.0;
				const Vec3 P = A * (1 - Fraction) + B * Fraction;
				const UnitDirection D = !I ? Previous.Direction : I == 16 ? Anchor : *UnitDirection::FromVector(P);
				const TerrainSample Probe = Source(D);
				const Vec3 U = D.Vector();
				const double Derivative = Probe.TangentialGradientPerRadian.Dot(Delta - U * U.Dot(Delta)) / P.Length();
				// 実標高又は解析微分の欠落，実際の上りを受理しない返戻
				if(!std::isfinite(Probe.ElevationMeters) || !std::isfinite(Derivative) || Derivative > 0) return {};
			}
		}
		// 元の末尾だけを交換し，確認した極小と元標高を保持する返戻
		return TerrainMinimumLink{ Point, Distance(Previous.Direction, Anchor) > 0 ? Path.size() - 1 : Path.size() - 2 };
	}
}
