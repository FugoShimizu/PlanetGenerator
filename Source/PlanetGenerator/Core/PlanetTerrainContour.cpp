#include "PlanetTerrainContour.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 球面上の有限な物理寸法と追跡予算を検査する
	 * @return 有限且つ有効な精度・予算の場合に真
	 */
	bool TerrainContourSettings::IsValid() const {
		const bool Valid = std::isfinite(RadiusMeters) && !(RadiusMeters < 1000) && RadiusMeters <= 1E9 && std::isfinite(LevelMeters) &&
		std::isfinite(HeightToleranceMeters) && HeightToleranceMeters > 0 && std::isfinite(PositionToleranceMeters) &&
		PositionToleranceMeters > 0 && std::isfinite(ChordToleranceMeters) && ChordToleranceMeters >= PositionToleranceMeters &&
		std::isfinite(GradientTolerance) && GradientTolerance > 0 && GradientTolerance < 1 && std::isfinite(MinimumStepMeters) &&
		MinimumStepMeters > 0 && std::isfinite(MaximumStepMeters) && MaximumStepMeters >= MinimumStepMeters &&
		MaximumStepMeters < RadiusMeters * 0.1 && std::isfinite(MaximumLengthMeters) && MaximumLengthMeters >= MaximumStepMeters &&
		MaximumLengthMeters < RadiusMeters * 8 && MaximumVertices > 3 && MaximumVertices < 65537 && (
			Terminals.empty() || std::isfinite(TerminalRadiusMeters) && TerminalRadiusMeters >= MinimumStepMeters * 4 &&
			TerminalRadiusMeters <= ChordToleranceMeters
		);
		// 無制限な追跡又は数値的に無効な設定を拒否する返戻
		if(!Valid || Terminals.size() > 1024) return false;
		for(std::size_t A = 0; A < Terminals.size(); ++A) for(std::size_t B = A + 1; B < Terminals.size(); ++B) {
			const Vec3 U = Terminals[A].Vector(), V = Terminals[B].Vector();
			const double Separation = RadiusMeters * std::atan2(U.Cross(V).Length(), U.Dot(V));
			// 帰還の識別半径が重なる終点を，入力順で勝手に選択しない返戻
			if(Separation <= TerminalRadiusMeters * 8) return false;
		}
		// 識別可能な終点を持つ有界な追跡設定の返戻
		return true;
	}

	/**
	 * 有界な局所多角形の重複辺・折返・非隣接辺の接触と交差を検査する
	 * 左端順に矩形の候補だけを比較する
	 * 最悪計算量 O(N²)，作業領域 O(N)，最大６５５３６頂点
	 * @param Vertices 閉鎖点を重複させない平面座標，各成分の絶対値は十億以下
	 * @return 非退化で単純な輪郭の場合に真；浮動小数点の符号判定を用いる
	 */
	bool IsSimplePlanarContour(const std::span<const std::array<double, 2>> Vertices) {
		// 入力の規模を超えた場合の返戻
		if(Vertices.size() < 3 || Vertices.size() > 65536) return false;
		// 有限な座標を保証し，向きの積の溢れを防ぐ範囲
		for(const std::array<double, 2> &Point : Vertices) {
			// 範囲外の座標への返戻
			for(const double Component : Point) if(!std::isfinite(Component) || std::abs(Component) > 1E9) return false;
		}
		// 三点の符号付面積
		const auto Orient =
		[](const std::array<double, 2> &A, const std::array<double, 2> &B, const std::array<double, 2> &C) -> double {
			return (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]);
		};
		// 隣接辺の退化と折返
		const std::size_t Count = Vertices.size();
		for(std::size_t i = 0; i < Count; ++i) {
			const std::array<double, 2> &Previous = Vertices[(i + Count - 1) % Count], &Point = Vertices[i];
			const std::array<double, 2> &Next = Vertices[(i + 1) % Count];
			// 長さ零の辺への返戻
			if(Point == Next) return false;
			const double Forward = (Point[0] - Previous[0]) * (Next[0] - Point[0]) + (Point[1] - Previous[1]) * (Next[1] - Point[1]);
			// 同一直線上で重なる隣接辺への返戻
			if(!Orient(Previous, Point, Next) && Forward < 0) return false;
		}
		// 左端順の辺番号と，各辺の閉矩形
		std::vector<std::array<double, 4>> Bounds;
		Bounds.reserve(Count);
		for(std::size_t I = 0; I < Count; ++I) {
			const std::array<double, 2> &A = Vertices[I], &B = Vertices[(I + 1) % Count];
			Bounds.push_back({ std::min(A[0], B[0]), std::max(A[0], B[0]), std::min(A[1], B[1]), std::max(A[1], B[1]) });
		}
		std::vector<std::uint32_t> Order(Count);
		std::iota(Order.begin(), Order.end(), 0U);
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::uint32_t A, const std::uint32_t B) -> bool {
				// 同じ左端では元番号を保つ全順序の返戻
				return Bounds[A][0] < Bounds[B][0] || Bounds[A][0] == Bounds[B][0] && A < B;
			}
		);
		// 非隣接辺の端点接触・重複・交差
		for(std::size_t I = 0; I < Count; ++I) for(std::size_t J = I + 1; J < Count; ++J) {
			const std::uint32_t AIndex = Order[I], BIndex = Order[J];
			const std::array<double, 4> &ABounds = Bounds[AIndex], &BBounds = Bounds[BIndex];
			if(BBounds[0] > ABounds[1]) break;
			if(BIndex == (AIndex + 1) % Count || AIndex == (BIndex + 1) % Count) continue;
			if(ABounds[2] > BBounds[3] || BBounds[2] > ABounds[3]) continue;
			const std::array<double, 2> &A = Vertices[AIndex], &B = Vertices[(AIndex + 1) % Count], &C = Vertices[BIndex];
			const std::array<double, 2> &D = Vertices[(BIndex + 1) % Count];
			// 両端が厳密に同じ側なら，矩形が重なっても線分は交差しない
			const double ABC = Orient(A, B, C), ABD = Orient(A, B, D), CDA = Orient(C, D, A), CDB = Orient(C, D, B);
			const bool IsSameSide = ABC > 0 && ABD > 0 || ABC < 0 && ABD < 0 || CDA > 0 && CDB > 0 || CDA < 0 && CDB < 0;
			// 接触又は交差する非隣接辺への返戻
			if(!IsSameSide) return false;
		}
		// 単純な輪郭の返戻
		return true;
	}

	/**
	 * 半球内の閉じた折線の符号付球面面積を，立体角の補償和で求める
	 * 計算量 O(N)，作業領域 O(1)，N は閉鎖点を含む標本数
	 * @param Contour 閉鎖済の等高線；自交差が有る場合は代数的面積である
	 * @param RadiusMeters 球面の物理半径
	 * @return 左側を囲む向きで正の面積；部分経路，非閉鎖又は半球外の経路は空
	 */
	std::optional<double> MeasureTerrainContourArea(const TerrainContour &Contour, const double RadiusMeters) {
		// 有限な半径と閉鎖条件
		const bool HasValidRadius = std::isfinite(RadiusMeters) && !(RadiusMeters < 1000) && RadiusMeters <= 1E9;
		const bool HasClosedCurve =
		Contour.End == TerrainContourEnd::Closed && Contour.Samples.size() > 3 && Contour.Samples.size() < 65537;
		// 計測対象が不正な場合の返戻
		if(!HasValidRadius || !HasClosedCurve) return {};
		// 重複した閉鎖点を除く基準方向
		const std::uint32_t Count = static_cast<std::uint32_t>(Contour.Samples.size()) - 1;
		const Vec3 First = Contour.Samples.front().Direction.Vector(), Last = Contour.Samples.back().Direction.Vector();
		// 先頭と末尾が異なる場合の返戻
		if((First - Last).Length()) return {};
		Vec3 Sum {};
		for(std::uint32_t i = 0; i < Count; ++i) Sum = Sum + Contour.Samples[i].Direction.Vector();
		const std::optional<UnitDirection> Reference = UnitDirection::FromVector(Sum);
		// 基準方向を決められない場合の返戻
		if(!Reference) return {};
		double Area = 0, Compensation = 0;
		for(std::uint32_t i = 0; i < Count; ++i) {
			const Vec3 A = Contour.Samples[i].Direction.Vector(), B = Contour.Samples[i + 1].Direction.Vector();
			const Vec3 Origin = Reference->Vector();
			// 大きい側と小さい側の判定が曖昧な半球外への返戻
			if(!(A.Dot(Origin) > 0)) return {};
			// 近い方向同士の外積の相殺を抑えた立体角と補償加算
			const double Numerator = Origin.Dot((A - Origin).Cross(B - Origin)), Denominator = 1 + Origin.Dot(A) + A.Dot(B) + B.Dot(Origin);
			const double Contribution = 2 * std::atan2(Numerator, Denominator) - Compensation, Total = Area + Contribution;
			Compensation = Total - Area - Contribution;
			Area = Total;
		}
		// 球面の物理的な代数面積の返戻
		return Area * RadiusMeters * RadiusMeters;
	}

	/**
	 * 接線方向の予測と勾配方向の補正で，不変な球面標高の正則な等高線を辿る
	 * 計算量 O(N B I)，N は頂点予算，B は３２回の刻み試行，I は１２回の補正上限
	 * @param Initial 等高線付近の初期点，補正距離は最大刻み以内
	 * @param Source 同一の連続地形の標高と解析接線勾配
	 * @param Settings 水位，半径，精度と有界な予算
	 * @param Stop 中断要求
	 * @return 閉曲線又は理由付の部分経路；不正入力・初期補正失敗・非有限な場・中断時は空
	 */
	std::optional<TerrainContour> TraceTerrainContour(
		const UnitDirection Initial,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainContourSettings &Settings,
		const std::stop_token Stop
	) {
		// 不正な入口又は中断時の返戻
		if(!Source || !Settings.IsValid() || Stop.stop_requested()) return {};
		// 追跡結果と，縮小しても回復しない場の失敗
		TerrainContour Result;
		bool HasInvalidSource = false;
		const auto Sample = [&](const UnitDirection Direction) -> std::optional<TerrainSample> {
			// 中断時の返戻
			if(Stop.stop_requested()) return {};
			// 不変な元場と接平面への勾配射影
			TerrainSample Value = Source(Direction);
			++Result.Evaluations;
			const Vec3 Gradient = Value.TangentialGradientPerRadian;
			if(!std::isfinite(Value.ElevationMeters) || !std::isfinite(Gradient.Length())) HasInvalidSource = true;
			// 非有限な場又は中断時の返戻
			if(HasInvalidSource || Stop.stop_requested()) return {};
			Value.TangentialGradientPerRadian =
			(Gradient - Direction.Vector() * Gradient.Dot(Direction.Vector())) * (1 / Settings.RadiusMeters);
			// メートル当たりの物理勾配と元標高の返戻
			return Value;
		};
		// 弧長と，接線上の物理変位
		const auto Distance = [&](const UnitDirection A, const UnitDirection B) -> double {
			return Settings.RadiusMeters * std::atan2(A.Vector().Cross(B.Vector()).Length(), A.Vector().Dot(B.Vector()));
		};
		const auto Offset = [&](const UnitDirection Point, const Vec3 Displacement) -> UnitDirection {
			return *UnitDirection::FromVector(Point.Vector() + Displacement * (1 / Settings.RadiusMeters));
		};
		// 法線方向の局所補正を終えた点
		struct CorrectedPoint {
			// 等高点と同じ元場の値
			UnitDirection Direction;
			TerrainSample Value;
		};
		// 予測点の近傍だけで行う等高線への補正
		const auto Correct = [&](const UnitDirection Predicted, const double Limit) -> std::optional<CorrectedPoint> {
			// 初期予測の周辺に限定した反復位置
			UnitDirection Point = Predicted;
			for(unsigned Iteration = 0; Iteration < 12; ++Iteration) {
				// 同じ等高線へ戻す為の元場の照会
				const std::optional<TerrainSample> Value = Sample(Point);
				// 場を照会出来ない場合の返戻
				if(!Value) return {};
				// 元高さの残差と局所の物理勾配
				const double Residual = Value->ElevationMeters - Settings.LevelMeters;
				const Vec3 Gradient = Value->TangentialGradientPerRadian;
				const double Norm = Gradient.Length();
				// 残差と推定位置誤差を共に満たす条件
				const bool HasConverged = std::abs(Residual) <= Settings.HeightToleranceMeters &&
				(Norm <= Settings.GradientTolerance || std::abs(Residual) <= Norm * Settings.PositionToleranceMeters);
				// 指定精度の等高点の返戻
				if(HasConverged) return CorrectedPoint{ Point, *Value };
				// 正則でない点の補正を打切る返戻
				if(Norm <= Settings.GradientTolerance) return {};
				// 最大移動を抑えた勾配方向の更新
				const double Travel = -Residual / Norm;
				// 許可した補正距離を超える場合の返戻
				if(std::abs(Travel) > Limit) return {};
				Point = Offset(Point, Gradient * (Travel / Norm));
				// 他の曲線へ飛び移る補正を拒否する返戻
				if(Distance(Predicted, Point) > Limit) return {};
			}
			// 補正の反復予算を超えた場合の返戻
			return {};
		};
		std::vector<double> TerminalHeights;
		double MaximumTerminalResidual = 0;
		for(const UnitDirection Terminal : Settings.Terminals) {
			const std::optional<TerrainSample> Target = Sample(Terminal);
			// 別の水位に在る終点を同じ等高線へ接続しない返戻
			if(!Target || std::abs(Target->ElevationMeters - Settings.LevelMeters) > Settings.HeightToleranceMeters) return {};
			TerminalHeights.push_back(Target->ElevationMeters);
			MaximumTerminalResidual = std::max(MaximumTerminalResidual, std::abs(Target->ElevationMeters - Settings.LevelMeters));
		}
		// 始点の正則性と，同じ水位への初期補正
		const std::optional<CorrectedPoint> Start = Correct(Initial, Settings.MaximumStepMeters);
		// 初期点を確定出来ない場合の返戻
		if(!Start || HasInvalidSource || Stop.stop_requested()) return {};
		CorrectedPoint Current = *Start;
		Result.Samples.push_back({ Current.Direction, Current.Value.ElevationMeters, 0 });
		Result.MaximumHeightResidualMeters =
		std::max(std::abs(Current.Value.ElevationMeters - Settings.LevelMeters), MaximumTerminalResidual);
		// 平坦面又は停留点を閉曲線として公開しない返戻
		if(Current.Value.TangentialGradientPerRadian.Length() <= Settings.GradientTolerance) {
			Result.End = TerrainContourEnd::CriticalPoint;
			// 正則でない始点の停止結果の返戻
			return Result;
		}
		// 低い側を左に保つ初期接線と，次の刻み
		const Vec3 StartTangent =
		UnitDirection::FromVector(Current.Direction.Vector().Cross(Current.Value.TangentialGradientPerRadian))->Vector();
		double Step = Settings.MaximumStepMeters;
		std::vector<bool> HasLeftTerminals(Settings.Terminals.size(), false);
		while(Result.Samples.size() < Settings.MaximumVertices) {
			// 出発点に近い終点からは一度離れ，他の終点には最初の接近で停止
			for(std::uint32_t Index = 0; Index < Settings.Terminals.size(); ++Index) {
				const UnitDirection Terminal = Settings.Terminals[Index];
				const double ToTerminal = Distance(Current.Direction, Terminal);
				if(ToTerminal > Settings.TerminalRadiusMeters * 4) HasLeftTerminals[Index] = true;
				if(HasLeftTerminals[Index] && ToTerminal <= Settings.TerminalRadiusMeters) {
					// 中断済の経路を終点到達として返さない返戻
					if(Stop.stop_requested()) return {};
					const double EndDistance = Result.Samples.back().DistanceMeters + ToTerminal;
					if(EndDistance > Settings.MaximumLengthMeters) {
						Result.End = TerrainContourEnd::LengthLimit;
						// 最後の短い接続も追跡距離の予算へ含める返戻
						return Result;
					}
					Result.Samples.push_back({ Terminal, TerminalHeights[Index], EndDistance });
					Result.End = TerrainContourEnd::Terminal;
					Result.TerminalIndex = Index;
					// 終点近傍の短い接続を含む部分経路の返戻
					return Result;
				}
				// 停留点を一度の予測で飛越し，反対側の等高枝へ移らない
				Step = std::min(Step, std::max(Settings.TerminalRadiusMeters * 0.5, ToTerminal * 0.25));
			}
			// 現在点の正則性
			const Vec3 Gradient = Current.Value.TangentialGradientPerRadian;
			const double Norm = Gradient.Length();
			if(Norm <= Settings.GradientTolerance) {
				Result.End = TerrainContourEnd::CriticalPoint;
				break;
			}
			// 今回の接線と残りの追跡距離
			const Vec3 Tangent = Current.Direction.Vector().Cross(Gradient) * (1 / Norm);
			const double Remaining = Settings.MaximumLengthMeters - Result.Samples.back().DistanceMeters;
			if(Remaining < Settings.MinimumStepMeters) {
				Result.End = TerrainContourEnd::LengthLimit;
				break;
			}
			Step = std::min(Step, Remaining);
			bool HasAccepted = false;
			for(unsigned Attempt = 0; Attempt < 32; ++Attempt) {
				// 接線上の予測と，同じ等高線への局所補正
				std::optional<CorrectedPoint> Next = Correct(Offset(Current.Direction, Tangent * Step), Step * 0.5);
				// 非有限な場又は中断時の返戻
				if(HasInvalidSource || Stop.stop_requested()) return {};
				if(Next) {
					// 始点の接線面を正方向へ横切る局所の閉鎖候補
					const double PreviousSide = (Current.Direction.Vector() - Start->Direction.Vector()).Dot(StartTangent);
					const double NextSide = (Next->Direction.Vector() - Start->Direction.Vector()).Dot(StartTangent);
					const bool CanClose =
					Result.Samples.size() > 7 && PreviousSide < 0 && NextSide >= 0 && Distance(Next->Direction, Start->Direction) <= Step * 1.5 &&
					Next->Value.TangentialGradientPerRadian.Dot(Start->Value.TangentialGradientPerRadian) > 0;
					if(CanClose) Next = Start;
					// 接線の向き，前進量と折線の中点誤差
					const Vec3 NextGradient = Next->Value.TangentialGradientPerRadian;
					const Vec3 Forward = Next->Direction.Vector() - Current.Direction.Vector();
					const UnitDirection Midpoint = *UnitDirection::FromVector(Current.Direction.Vector() + Next->Direction.Vector());
					const std::optional<CorrectedPoint> Middle = Correct(Midpoint, Step * 0.5);
					// 非有限な場又は中断時の返戻
					if(HasInvalidSource || Stop.stop_requested()) return {};
					const double Length = Distance(Current.Direction, Next->Direction);
					const bool HasForwardTangent = NextGradient.Dot(Gradient) > 0 && Forward.Dot(Tangent) > 0;
					if(Middle && HasForwardTangent && Length > 0 && Length <= Step * 1.5 && Length <= Remaining) {
						// 中点に現れない湾曲も検出する四分点の補正
						double Sag = Distance(Midpoint, Middle->Direction);
						bool HasResolvedQuarters = true;
						for(const double Fraction : { 0.25, 0.75 }) {
							const UnitDirection Probe =
							*UnitDirection::FromVector(Current.Direction.Vector() * (1 - Fraction) + Next->Direction.Vector() * Fraction);
							const std::optional<CorrectedPoint> Projected = Correct(Probe, Step * 0.5);
							if(!Projected) {
								HasResolvedQuarters = false;
								break;
							}
							Sag = std::max(Sag, Distance(Probe, Projected->Direction));
						}
						// 照会中の中断又は非有限な場を公開しない返戻
						if(HasInvalidSource || Stop.stop_requested()) return {};
						if(HasResolvedQuarters && Sag <= Settings.ChordToleranceMeters) {
							Current = *Next;
							Result.Samples.push_back({ Current.Direction, Current.Value.ElevationMeters, Result.Samples.back().DistanceMeters + Length });
							Result.MaximumHeightResidualMeters =
							std::max(Result.MaximumHeightResidualMeters, std::abs(Current.Value.ElevationMeters - Settings.LevelMeters));
							Result.MaximumChordCorrectionMeters = std::max(Result.MaximumChordCorrectionMeters, Sag);
							HasAccepted = true;
							if(CanClose) {
								Result.End = TerrainContourEnd::Closed;
								// 始点と一致する正則な閉曲線の返戻
								return Result;
							}
							if(Sag < Settings.ChordToleranceMeters * 0.2) Step = std::min(Settings.MaximumStepMeters, Step * 1.5);
							break;
						}
					}
				}
				++Result.RejectedSteps;
				Step *= 0.5;
				if(Step < Settings.MinimumStepMeters) break;
			}
			if(!HasAccepted) {
				Result.End = TerrainContourEnd::StepLimit;
				break;
			}
		}
		// 非有限な場又は中断時の返戻
		if(HasInvalidSource || Stop.stop_requested()) return {};
		// 停止理由を持つ部分経路の返戻
		return Result;
	}
}
