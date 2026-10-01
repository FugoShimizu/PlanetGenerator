#include "PlanetSpillString.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		/**
		 * 球面折線の点を等しい弧長へ配置し，両端を厳密に保持する
		 * 計算量：時間・追加空間 O(M+N)，M は入力点数，N は出力点数
		 * @param Points 順序付きの短い大円弧
		 * @param Count 再配置する点の個数
		 * @param Radius 物理的な惑星半径
		 * @return 退化又は反対側を跨ぐ入力は空
		 */
		std::optional<std::vector<UnitDirection>> RedistributeSpillString(
			const std::span<const UnitDirection> Points,
			const std::uint32_t Count,
			const double Radius
		) {
			// 単調な累積弧長の準備
			std::vector<double> Lengths(Points.size(), 0);
			for(std::size_t I = 1; I < Points.size(); ++I) {
				const Vec3 A = Points[I - 1].Vector(), B = Points[I].Vector();
				// 大円の選択が曖昧な区間への返戻
				if(!(A.Dot(B) > 0)) return {};
				Lengths[I] = Lengths[I - 1] + Radius * std::atan2(A.Cross(B).Length(), A.Dot(B));
			}
			// 全体が一点へ潰れた入力への返戻
			if(!std::isfinite(Lengths.back()) || !(Lengths.back() > 0)) return {};
			std::vector<UnitDirection> Result;
			Result.reserve(Count);
			Result.push_back(Points.front());
			std::size_t Edge = 0;
			for(std::uint32_t I = 1; I + 1 < Count; ++I) {
				const double Distance = Lengths.back() * I / (Count - 1);
				while(Edge + 1 < Points.size() && Lengths[Edge + 1] <= Distance) ++Edge;
				// 正の長さの補間区間を失った場合の返戻
				if(Edge + 1 >= Points.size() || Lengths[Edge + 1] <= Lengths[Edge]) return {};
				const double Fraction = (Distance - Lengths[Edge]) / (Lengths[Edge + 1] - Lengths[Edge]);
				const Vec3 A = Points[Edge].Vector(), B = Points[Edge + 1].Vector();
				const double Angle = std::atan2(A.Cross(B).Length(), A.Dot(B));
				const std::optional<UnitDirection> Tangent = UnitDirection::FromVector(B - A * A.Dot(B));
				// 表現出来ない短い弧を同一点として通さない返戻
				if(!Tangent) return {};
				Result.push_back(*UnitDirection::FromVector(A * std::cos(Angle * Fraction) + Tangent->Vector() * std::sin(Angle * Fraction)));
			}
			Result.push_back(Points.back());
			// 両端の元方向を其の儘保持した再配置の返戻
			return Result;
		}
	}

	/**
	 * 元場の下降と球面弧長の再配置で，格子経路由来の候補を緩和する
	 * 計算量：時間 O(M+KN+A)，追加空間 O(M+N)，M は入力点数，N は配置点数，K は反復数
	 * 元場照会１回の費用を一定とし，A は外部受理関数の累計費用
	 * E・Ren・Vanden-Eijnden（２００７）の二段階法を参考にした有界な候補準備
	 * 三次補間でなく大円弧の補間を使い，原論文の四次精度又は収束保証を主張しない
	 * @param Initial 元場で探索した有限の経路
	 * @param Source 不変な元地形の標高と解析接線勾配
	 * @param Settings 物理尺度と点数・反復・評価の上限
	 * @param Stop 中断要求
	 * @param Accept 有界な途中経路を同じ元場で評価し，外部の実接続条件を満たした時だけ真を返す関数；省略可
	 * @return 実鞍点の別途確認を要する経路候補；無効・評価不足・中断時は空
	 * @note 外部受理関数の追加の元場照会は，呼出側で全準備の共通予算へ計上する
	 */
	std::optional<SpillString> RelaxSpillString(
		const std::span<const UnitDirection> Initial,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillStringSettings &Settings,
		const std::stop_token Stop,
		const SpillStringAcceptance &Accept
	) {
		// 対応する物理尺度と，確保前の全予算の検査
		const double Radius = Settings.RadiusMeters;
		const bool Valid =
		Source && Initial.size() > 1 && Initial.size() < 65537 && std::isfinite(Radius) && !(Radius < 1000) && Radius <= 1E9 &&
		std::isfinite(Settings.SpacingMeters) && Settings.SpacingMeters >= 0.01 && !(Settings.SpacingMeters > 50) &&
		std::isfinite(Settings.TimeStepMeters) && Settings.TimeStepMeters > 0 && Settings.TimeStepMeters <= 1E6 &&
		std::isfinite(Settings.MaximumMoveMeters) && Settings.MaximumMoveMeters > 0 && Settings.MaximumMoveMeters < Radius * 0.1 &&
		std::isfinite(Settings.MotionToleranceMeters) && Settings.MotionToleranceMeters > 0 &&
		Settings.MotionToleranceMeters < Settings.SpacingMeters && Settings.MaximumVertices > 2 && Settings.MaximumVertices < 65537 &&
		Settings.MaximumIterations > 0 && Settings.MaximumIterations < 513 && Settings.MaximumEvaluations > 0 &&
		Settings.MaximumEvaluations < 100000001;
		const bool HasAcceptance = !Accept || Settings.AcceptanceStride > 0 && Settings.AcceptanceStride <= Settings.MaximumIterations;
		// 不正設定又は開始前の中断への返戻
		if(!Valid || !HasAcceptance || Stop.stop_requested()) return {};
		double Length = 0;
		for(std::size_t I = 1; I < Initial.size(); ++I) {
			const Vec3 A = Initial[I - 1].Vector(), B = Initial[I].Vector();
			Length += Radius * std::atan2(A.Cross(B).Length(), A.Dot(B));
		}
		const double Required = std::ceil(Length / Settings.SpacingMeters) + 1;
		// 退化又は点数予算を超えた経路への返戻
		if(!std::isfinite(Required) || Required < 3 || Required > Settings.MaximumVertices) return {};
		const std::uint32_t Count = static_cast<std::uint32_t>(Required);
		std::optional<std::vector<UnitDirection>> Resampled = RedistributeSpillString(Initial, Count, Radius);
		// 初期曲線を構成出来ない場合の返戻
		if(!Resampled) return {};
		SpillString Result;
		Result.Path = std::move(*Resampled);
		const auto Sample = [&](const UnitDirection D) -> std::optional<TerrainSample> {
			// 予算後又は中断後に元場を照会しない返戻
			if(Stop.stop_requested() || Result.Evaluations >= Settings.MaximumEvaluations) return {};
			++Result.Evaluations;
			const TerrainSample Value = Source(D);
			// 非有限な元場を候補へ混ぜない返戻
			if(!std::isfinite(Value.ElevationMeters) || !std::isfinite(Value.TangentialGradientPerRadian.Length())) return {};
			// 計数した同じ元場の返戻
			return Value;
		};
		const auto UpdateCandidates = [&]() -> bool {
			Result.PassCandidates.clear();
			Result.MaximumNormalSlope = 0;
			std::vector<TerrainSample> Values;
			Values.reserve(Count);
			for(const UnitDirection D : Result.Path) {
				const std::optional<TerrainSample> Value = Sample(D);
				// 最終検査の評価不足への返戻
				if(!Value) return false;
				Values.push_back(*Value);
			}
			for(std::uint32_t I = 1; I + 1 < Count; ++I) {
				const Vec3 P = Result.Path[I].Vector();
				const Vec3 Delta = Result.Path[I + 1].Vector() - Result.Path[I - 1].Vector();
				const std::optional<UnitDirection> Tangent = UnitDirection::FromVector(Delta - P * Delta.Dot(P));
				// 経路の接線を定義出来ない折返しへの返戻
				if(!Tangent) return false;
				const Vec3 G = Values[I].TangentialGradientPerRadian;
				const Vec3 Gradient = (G - P * G.Dot(P)) * (1 / Radius), Along = Tangent->Vector();
				Result.MaximumNormalSlope = std::max(Result.MaximumNormalSlope, (Gradient - Along * Gradient.Dot(Along)).Length());
				if(Values[I].ElevationMeters > Values[I - 1].ElevationMeters && Values[I].ElevationMeters >= Values[I + 1].ElevationMeters) {
					Result.PassCandidates.push_back(Result.Path[I]);
				}
			}
			// 同じ途中曲線の元場と接線を確認した返戻
			return true;
		};
		for(std::uint32_t Iteration = 0; Iteration < Settings.MaximumIterations; ++Iteration) {
			std::vector<UnitDirection> Evolved = Result.Path;
			for(std::uint32_t I = 1; I + 1 < Count; ++I) {
				const UnitDirection Point = Result.Path[I];
				const Vec3 P = Point.Vector();
				const std::optional<TerrainSample> Value = Sample(Point);
				// 下降段階の場を評価出来ない場合の返戻
				if(!Value) return {};
				const Vec3 G = Value->TangentialGradientPerRadian;
				const Vec3 Gradient = (G - P * G.Dot(P)) * (1 / Radius);
				const double Norm = Gradient.Length();
				if(!Norm) continue;
				const Vec3 Down = Gradient * (-1 / Norm);
				// 隣接点の入替を避ける局所変位の上限
				const Vec3 Before = Result.Path[I - 1].Vector(), After = Result.Path[I + 1].Vector();
				const double PreviousDistance = Radius * std::atan2(P.Cross(Before).Length(), P.Dot(Before));
				const double NextDistance = Radius * std::atan2(P.Cross(After).Length(), P.Dot(After));
				double Move =
				std::min({ Settings.MaximumMoveMeters, Settings.TimeStepMeters * Norm, 0.25 * std::min(PreviousDistance, NextDistance) });
				for(unsigned Trial = 0; Trial < 16; ++Trial) {
					const double Angle = Move / Radius;
					const UnitDirection Next = *UnitDirection::FromVector(P * std::cos(Angle) + Down * std::sin(Angle));
					const std::optional<TerrainSample> Candidate = Sample(Next);
					// 縮小探索でも場又は予算を失った場合の返戻
					if(!Candidate) return {};
					if(Candidate->ElevationMeters < Value->ElevationMeters) {
						Evolved[I] = Next;
						break;
					}
					Move *= 0.5;
				}
			}
			Resampled = RedistributeSpillString(Evolved, Count, Radius);
			// 再配置不能又は途中の中断への返戻
			if(!Resampled || Stop.stop_requested()) return {};
			Result.MaximumMotionMeters = 0;
			for(std::uint32_t I = 0; I < Count; ++I) {
				const Vec3 A = Result.Path[I].Vector(), B = (*Resampled)[I].Vector();
				Result.MaximumMotionMeters = std::max(Result.MaximumMotionMeters, Radius * std::atan2(A.Cross(B).Length(), A.Dot(B)));
			}
			Result.Path = std::move(*Resampled);
			Result.Iterations = Iteration + 1;
			if(Result.MaximumMotionMeters <= Settings.MotionToleranceMeters) break;
			if(Accept && !(Result.Iterations % Settings.AcceptanceStride)) {
				// 実際の途中経路から候補を作成出来ない場合の返戻
				if(!UpdateCandidates() || Stop.stop_requested()) return {};
				if(Accept(Result)) {
					// 外部の確認処理による中断を受理へ置換えない返戻
					if(Stop.stop_requested()) return {};
					Result.AcceptedCheckpoint = true;
					// 変位収束の保証とは異なる，外部の実条件を確認した候補の返戻
					return Result;
				}
			}
		}
		// 最終曲線も同じ元場で再評価する
		// 有効な経路と候補の欠落への返戻
		if(!UpdateCandidates() || Stop.stop_requested()) return {};
		// 収束証明とは区別した有界な候補の返戻
		return Result;
	}

	/**
	 * 谷底を固定し，終点の接線方向の力を反転させて，盆地に隣接する峠候補へ登る
	 * 計算量：時間 O(KN²)，追加空間 O(N)，N は配置点数，K は反復数（元場照会１回の費用は一定）
	 * Ren・Vanden-Eijnden（２０１３）の登坂式と単調制約を球面に適用した有限点の候補準備
	 * 原論文の連続曲線の収束保証を有限標本へ移さず，実接続は後段で確認する
	 * @param Minimum 同じ元場で確認済の非退化な極小
	 * @param InitialTip 初期経路のもう一方の端点
	 * @param Source 不変な元地形の標高と解析接線微分
	 * @param Settings 物理寸法，停止勾配と点数・反復・評価の上限
	 * @param Stop 中断要求
	 * @return 有界な候補と停止理由；無効・評価不足・元場の欠落・中断時は空
	 */
	std::optional<SpillClimb> ClimbSpillString(
		const TerrainCriticalPoint &Minimum,
		const UnitDirection InitialTip,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillClimbSettings &Settings,
		const std::stop_token Stop
	) {
		// 配列確保又は元場の照会に先立つ，全ての外部設定の検査
		const double Radius = Settings.RadiusMeters;
		const bool Valid = Source && Minimum.Kind == TerrainCriticalKind::Minimum && std::isfinite(Minimum.ElevationMeters) &&
		std::isfinite(Minimum.GradientNorm) && Minimum.GradientNorm >= 0 && Minimum.GradientNorm <= 1E-8 && std::isfinite(Radius) &&
		!(Radius < 1000) && Radius <= 1E9 && std::isfinite(Settings.TimeStepMeters) && Settings.TimeStepMeters > 0 &&
		Settings.TimeStepMeters <= 1E6 && std::isfinite(Settings.MaximumMoveMeters) && Settings.MaximumMoveMeters > 0 &&
		Settings.MaximumMoveMeters < Radius * 0.1 && std::isfinite(Settings.MaximumLengthMeters) &&
		Settings.MaximumLengthMeters > 0.01 && Settings.MaximumLengthMeters < Radius * 0.25 &&
		std::isfinite(Settings.GradientTolerance) && Settings.GradientTolerance > 0 && Settings.GradientTolerance < 0.01 &&
		Settings.Vertices > 3 && Settings.Vertices < 130 && Settings.MaximumIterations > 0 && Settings.MaximumIterations < 1025 &&
		Settings.MaximumEvaluations > 0 && Settings.MaximumEvaluations < 10000001;
		// 無効設定又は開始前の中断への返戻
		if(!Valid || Stop.stop_requested()) return {};
		const auto Length = [Radius](const std::span<const UnitDirection> Points) -> double {
			double Total = 0;
			for(std::size_t I = 1; I < Points.size(); ++I) {
				const Vec3 A = Points[I - 1].Vector(), B = Points[I].Vector();
				Total += Radius * std::atan2(A.Cross(B).Length(), A.Dot(B));
			}
			// 元の球面上の物理的な弧長の返戻
			return Total;
		};
		const std::array<UnitDirection, 2> Initial { Minimum.Direction, InitialTip };
		const double InitialLength = Length(Initial);
		// 潰れた初期曲線又は開始時点の範囲超過への返戻
		if(!(InitialLength >= 0.01) || InitialLength > Settings.MaximumLengthMeters) return {};
		std::optional<std::vector<UnitDirection>> Resampled = RedistributeSpillString(Initial, Settings.Vertices, Radius);
		// 弧長で配置出来ない初期方向への返戻
		if(!Resampled) return {};
		SpillClimb Result;
		Result.Path = std::move(*Resampled);
		const auto Sample = [&](const UnitDirection D) -> std::optional<TerrainSample> {
			// 中断後又は予算後に元場を呼出さない返戻
			if(Stop.stop_requested() || Result.Evaluations >= Settings.MaximumEvaluations) return {};
			++Result.Evaluations;
			TerrainSample Value = Source(D);
			const Vec3 P = D.Vector(), G = Value.TangentialGradientPerRadian;
			// 非有限な地形を候補へ混ぜない返戻
			if(Stop.stop_requested() || !std::isfinite(Value.ElevationMeters) || !std::isfinite(G.Length())) return {};
			Value.TangentialGradientPerRadian = (G - P * G.Dot(P)) * (1 / Radius);
			// 同じ元場の物理的な接線勾配の返戻
			return Value;
		};
		const auto Evaluate = [&](const std::span<const UnitDirection> Points) -> std::optional<std::vector<TerrainSample>> {
			std::vector<TerrainSample> Values;
			Values.reserve(Points.size());
			for(const UnitDirection D : Points) {
				const std::optional<TerrainSample> Value = Sample(D);
				// 配列の一部だけを有効にしない返戻
				if(!Value) return {};
				Values.push_back(*Value);
			}
			// 全点を同じ元場で評価した結果の返戻
			return Values;
		};
		std::optional<std::vector<TerrainSample>> Values = Evaluate(Result.Path);
		// 入力極小と再評価した元場の照合
		const bool MatchesMinimum = Values && std::abs(Values->front().ElevationMeters - Minimum.ElevationMeters) <= 1E-6 &&
		Values->front().TangentialGradientPerRadian.Length() <= 1E-8;
		// 別の元場の極小又は初回評価不足への返戻
		if(!MatchesMinimum) return {};
		for(std::uint32_t Iteration = 0; Iteration <= Settings.MaximumIterations; ++Iteration) {
			// 初期経路と弧長補間後も，同じ元場の単調標本を確認してから停止を判定
			for(std::uint32_t Trial = 0; Trial < Settings.Vertices; ++Trial) {
				std::size_t FirstDescent = Result.Path.size();
				for(std::size_t I = 1; I < Result.Path.size(); ++I) if((*Values)[I].ElevationMeters < (*Values)[I - 1].ElevationMeters) {
					FirstDescent = I;
					break;
				}
				if(FirstDescent == Result.Path.size()) break;
				// 谷底だけへの退化又は制約修正の有限予算を超えた場合の返戻
				if(FirstDescent < 2 || Trial + 1 == Settings.Vertices) return {};
				Result.Path.erase(Result.Path.begin() + static_cast<std::ptrdiff_t>(FirstDescent), Result.Path.end());
				++Result.Truncations;
				Resampled = RedistributeSpillString(Result.Path, Settings.Vertices, Radius);
				// 単調制約の修正後に経路を作れない場合の返戻
				if(!Resampled) return {};
				Result.Path = std::move(*Resampled);
				Values = Evaluate(Result.Path);
				// 制約の検査途中で場又は予算を失った場合の返戻
				if(!Values) return {};
			}
			Result.LengthMeters = Length(Result.Path);
			Result.TipGradientNorm = Values->back().TangentialGradientPerRadian.Length();
			Result.MaximumNormalSlope = 0;
			std::vector<Vec3> Forces(Result.Path.size()), Tangents(Result.Path.size());
			for(std::uint32_t I = 1; I < Settings.Vertices; ++I) {
				const Vec3 P = Result.Path[I].Vector();
				// 単調標高の内部には高い側へ片寄せた接線，登坂端点には後退接線を使う
				const Vec3 Delta = I + 1 == Settings.Vertices ? P - Result.Path[I - 1].Vector() : Result.Path[I + 1].Vector() - P;
				const std::optional<UnitDirection> Tangent = UnitDirection::FromVector(Delta - P * Delta.Dot(P));
				// 折返して接線を定義出来ない候補への返戻
				if(!Tangent) return {};
				const Vec3 G = (*Values)[I].TangentialGradientPerRadian, T = Tangent->Vector();
				const Vec3 Normal = G - T * G.Dot(T);
				if(I + 1 < Settings.Vertices) Result.MaximumNormalSlope = std::max(Result.MaximumNormalSlope, Normal.Length());
				// 内部は直交成分だけを下降させ，終点は接線方向だけを反転させる ν ＝２の登坂力
				Tangents[I] = T;
				Forces[I] = I + 1 == Settings.Vertices ? T * (2 * G.Dot(T)) - G : Normal * -1;
			}
			Result.Iterations = Iteration;
			if(Result.TipGradientNorm <= Settings.GradientTolerance && Result.MaximumNormalSlope <= Settings.GradientTolerance) {
				Result.End = SpillClimbEnd::Converged;
				break;
			}
			if(Iteration == Settings.MaximumIterations) break;
			std::vector<UnitDirection> Evolved = Result.Path;
			std::vector<TerrainSample> EvolvedValues = *Values;
			for(std::uint32_t I = 1; I < Settings.Vertices; ++I) {
				const Vec3 P = Result.Path[I].Vector(), F = Forces[I];
				const double Norm = F.Length();
				if(!Norm) continue;
				const Vec3 Previous = Result.Path[I - 1].Vector();
				double Distance = Radius * std::atan2(P.Cross(Previous).Length(), P.Dot(Previous));
				if(I + 1 < Settings.Vertices) {
					const Vec3 Next = Result.Path[I + 1].Vector();
					Distance = std::min(Distance, Radius * std::atan2(P.Cross(Next).Length(), P.Dot(Next)));
				}
				// 接線方向の伝播も隣接点を飛越えない刻みを選び，中央差分由来の振動を作らない
				const double GradientNorm = (*Values)[I].TangentialGradientPerRadian.Length();
				const double StableMove = GradientNorm ? Distance * 0.25 * std::min(1.0, Norm / GradientNorm) : Distance * 0.25;
				double Move = std::min({ Settings.MaximumMoveMeters, Settings.TimeStepMeters * Norm, StableMove });
				const Vec3 Along = Tangents[I];
				for(unsigned Trial = 0; Trial < 16; ++Trial) {
					const double Angle = Move / Radius;
					const UnitDirection Candidate = *UnitDirection::FromVector(P * std::cos(Angle) + F * (std::sin(Angle) / Norm));
					const std::optional<TerrainSample> Value = Sample(Candidate);
					// 縮小探索中の評価不足又は中断への返戻
					if(!Value) return {};
					const Vec3 Q = Candidate.Vector(), G = Value->TangentialGradientPerRadian;
					const Vec3 T = UnitDirection::FromVector(Along - Q * Along.Dot(Q))->Vector();
					const Vec3 NextForce = I + 1 == Settings.Vertices ? T * (2 * G.Dot(T)) - G : T * G.Dot(T) - G;
					// 停留点の往復を避ける刻み縮小；内部点は元標高の下降も確認
					if(F.Dot(NextForce) >= 0 && (I + 1 == Settings.Vertices || Value->ElevationMeters <= (*Values)[I].ElevationMeters)) {
						Evolved[I] = Candidate;
						EvolvedValues[I] = *Value;
						break;
					}
					Move *= 0.5;
				}
			}
			// 標本が初めて下降する手前で切り，離れた盆地へ経路を延ばさない
			for(std::size_t I = 1; I < Evolved.size(); ++I) {
				if(EvolvedValues[I].ElevationMeters >= EvolvedValues[I - 1].ElevationMeters) continue;
				// 残る点が谷底だけになる退化を成功にしない返戻
				if(I < 2) return {};
				Evolved.erase(Evolved.begin() + static_cast<std::ptrdiff_t>(I), Evolved.end());
				++Result.Truncations;
				break;
			}
			if(Length(Evolved) > Settings.MaximumLengthMeters) {
				Result.End = SpillClimbEnd::LengthLimit;
				break;
			}
			Resampled = RedistributeSpillString(Evolved, Settings.Vertices, Radius);
			// 短縮後の曲線を再配置出来ない場合の返戻
			if(!Resampled) return {};
			Result.Path = std::move(*Resampled);
			Values = Evaluate(Result.Path);
			// 再配置後に場又は予算を失った場合の返戻
			if(!Values) return {};
		}
		// 収束・反復上限・距離上限を区別した候補の返戻
		return Result;
	}
}
