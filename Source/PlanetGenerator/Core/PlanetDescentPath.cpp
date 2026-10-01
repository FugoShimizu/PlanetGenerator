#include "PlanetDescentPath.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	/**
	 * 物理尺度，停止条件と有限の計算予算を検査する
	 * @return 全設定が対応範囲内の場合に真
	 */
	bool TerrainDescentSettings::IsValid() const {
		// 不正な尺度又は無制限な計算を拒否する返戻
		return std::isfinite(RadiusMeters) && !(RadiusMeters < 1000) && RadiusMeters <= 1E9 && std::isfinite(StopElevationMeters) &&
		std::isfinite(GradientTolerance) && GradientTolerance > 0 && GradientTolerance < 0.01 && std::isfinite(MinimumStepMeters) &&
		MinimumStepMeters > 0 && std::isfinite(MaximumStepMeters) && MaximumStepMeters >= MinimumStepMeters &&
		MaximumStepMeters < RadiusMeters * 0.1 && std::isfinite(PositionToleranceMeters) && PositionToleranceMeters > 0 &&
		PositionToleranceMeters < MaximumStepMeters && std::isfinite(MaximumLengthMeters) && MaximumLengthMeters >= MaximumStepMeters &&
		MaximumLengthMeters < RadiusMeters && MaximumVertices > 1 && MaximumVertices < 65537;
	}

	/**
	 * 元場の解析勾配に沿う曲線を適応刻みの４次 Runge–Kutta 法で追跡する
	 * 地形を変更せず，試料間の連続な排水保証又は極小への到達証明とは区別する
	 * 計算量：O(V) 回の有界な場評価，記憶量：O(V)，V は頂点予算
	 * @param Initial 球面上の初期方向
	 * @param Source 同じ連続場の標高と解析接線勾配
	 * @param Settings 物理尺度，精度，停止条件と計算予算
	 * @param Stop 中断要求
	 * @param StopDomainDistance 高度終端の所属領域までの符号付物理距離；内側は非負，省略時は高度だけ
	 * @return 停止理由を持つ経路，入力不正・場の非有限値・中断時は空
	 */
	std::optional<TerrainDescentPath> TraceTerrainDescent(
		const UnitDirection Initial,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainDescentSettings &Settings,
		const std::stop_token Stop,
		const std::function<double(UnitDirection)> &StopDomainDistance
	) {
		// 不正な入口又は中断時の返戻
		if(!Source || !Settings.IsValid() || Stop.stop_requested()) return {};
		// 元場の有限値と物理勾配の取得
		TerrainDescentPath Path;
		const auto Sample = [&](const UnitDirection Direction) -> std::optional<TerrainSample> {
			// 中断時の返戻
			if(Stop.stop_requested()) return {};
			// 現在方向での元場の照会
			TerrainSample Value = Source(Direction);
			++Path.Evaluations;
			// 元場の有限性と中断状態
			const bool HasFiniteValue = std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
			// 中断又は非有限な元場を採用しない返戻
			if(Stop.stop_requested() || !HasFiniteValue) return {};
			Value.TangentialGradientPerRadian =
			Value.TangentialGradientPerRadian - Direction.Vector() * Value.TangentialGradientPerRadian.Dot(Direction.Vector());
			// 球面接線の解析勾配の返戻
			return Value;
		};
		const auto Offset = [&](const UnitDirection Position, const Vec3 Delta) -> UnitDirection {
			// 物理変位を同じ球面へ射影した方向の返戻
			return *UnitDirection::FromVector(Position.Vector() + Delta * (1 / Settings.RadiusMeters));
		};
		const auto Velocity = [&](const UnitDirection Position) -> std::optional<Vec3> {
			// 弧長を独立変数にする下降方向
			const std::optional<TerrainSample> Value = Sample(Position);
			// 場を照会出来ない場合の返戻
			if(!Value) return {};
			// 速度の正規化に使う勾配長
			const double Length = Value->TangentialGradientPerRadian.Length();
			// 停留位置の零速度又は単位下降方向の返戻
			return Length ? Value->TangentialGradientPerRadian * (-1 / Length) : Vec3{};
		};
		const auto Integrate = [&](const UnitDirection Position, const double Step, bool &HasTurning) -> std::optional<UnitDirection> {
			// ４次積分の段階速度は各段階の同じ球面上で照会
			const std::optional<Vec3> A = Velocity(Position);
			// 最初の速度を取得出来ない場合の返戻
			if(!A) return {};
			// 中間段階の最初の速度
			const std::optional<Vec3> B = Velocity(Offset(Position, *A * (Step * 0.5)));
			// 第２段の速度を取得出来ない場合の返戻
			if(!B) return {};
			// 中間段階の更新速度
			const std::optional<Vec3> C = Velocity(Offset(Position, *B * (Step * 0.5)));
			// 第３段の速度を取得出来ない場合の返戻
			if(!C) return {};
			// 終点段階の速度
			const std::optional<Vec3> D = Velocity(Offset(Position, *C * Step));
			// 最後の速度を取得出来ない場合の返戻
			if(!D) return {};
			// 正規化勾配の特異点を跨ぐ段階反転の検出
			HasTurning = HasTurning || A->Dot(*B) < 0 || A->Dot(*C) < 0 || A->Dot(*D) < 0;
			// 球面上の積分候補の返戻
			return Offset(Position, (*A + *B * 2 + *C * 2 + *D) * (Step / 6));
		};
		const auto Distance = [&](const UnitDirection A, const UnitDirection B) -> double {
			// 小角度でも精度を保つ球面弧長の返戻
			return Settings.RadiusMeters * std::atan2(A.Vector().Cross(B.Vector()).Length(), A.Vector().Dot(B.Vector()));
		};
		// 現在の標高と適応刻み
		std::optional<TerrainSample> Current = Sample(Initial);
		// 初期値を取得出来ない場合の返戻
		if(!Current) return {};
		Path.Samples.reserve(std::min(Settings.MaximumVertices, std::uint32_t{ 1024 }));
		Path.Samples.push_back({ Initial, Current->ElevationMeters, 0 });
		// 初期刻みと１頂点当りの最大試行回数
		double Step = Settings.MaximumStepMeters;
		constexpr std::uint32_t MaximumAttemptsPerVertex = 32;
		for(std::uint32_t Attempt = 0; Attempt < Settings.MaximumVertices * MaximumAttemptsPerVertex; ++Attempt) {
			// 採用済の末尾と残りの計算範囲
			const TerrainDescentSample Previous = Path.Samples.back();
			const double Remaining = Settings.MaximumLengthMeters - Previous.DistanceMeters;
			// 元高度と実際の受入領域を両方満たして初めて停止する
			const bool HasReachedHeight = Current->ElevationMeters <= Settings.StopElevationMeters;
			const double DomainDistance = HasReachedHeight && StopDomainDistance ? StopDomainDistance(Previous.Direction) : 0;
			// 所属の照会不能又は照会中の中断を，高度到達の成功へ読み替えない返戻
			if(!std::isfinite(DomainDistance) || Stop.stop_requested()) return {};
			if(HasReachedHeight && DomainDistance >= 0) {
				Path.End = TerrainDescentEnd::HeightLimit;
				break;
			}
			// 岸付近で元の下降を継続し，短い受入域を大きな歩幅で飛越す候補を抑える
			if(HasReachedHeight) Step = std::min(Step, std::max(Settings.MinimumStepMeters, -2 * DomainDistance));
			if(Current->TangentialGradientPerRadian.Length() / Settings.RadiusMeters <= Settings.GradientTolerance) {
				Path.End = TerrainDescentEnd::GradientLimit;
				break;
			}
			if(Remaining < Settings.MinimumStepMeters) {
				Path.End = TerrainDescentEnd::LengthLimit;
				break;
			}
			if(Path.Samples.size() >= Settings.MaximumVertices) break;
			Step = std::min(Step, Remaining);
			// 全幅と半幅２回の積分による独立な刻み誤差推定
			bool HasTurning = false;
			const std::optional<UnitDirection> Full = Integrate(Previous.Direction, Step, HasTurning);
			const std::optional<UnitDirection> Half = Integrate(Previous.Direction, Step * 0.5, HasTurning);
			// 積分途中で場を失った場合の返戻
			if(!Full || !Half) return {};
			// 半幅の後半の積分
			const std::optional<UnitDirection> Fine = Integrate(*Half, Step * 0.5, HasTurning);
			// 半幅の後半を積分出来ない場合の返戻
			if(!Fine) return {};
			// 更新候補の元場
			const std::optional<TerrainSample> Next = Sample(*Fine);
			// 更新先を照会出来ない場合の返戻
			if(!Next) return {};
			// ４次積分の差から半幅側の誤差を推定：2^4 - 1 = 15
			const double Error = Distance(*Full, *Fine) / 15, Length = Distance(Previous.Direction, *Fine);
			bool IsDescending = Next->ElevationMeters <= Current->ElevationMeters;
			double LastHeight = Current->ElevationMeters;
			for(unsigned Probe = 1; Probe < 5 && IsDescending; ++Probe) {
				// 積分後の短い弧の内部で同じ元場の高さを確認
				const double Fraction = Probe * 0.25;
				const UnitDirection Direction =
				*UnitDirection::FromVector(Previous.Direction.Vector() * (1 - Fraction) + Fine->Vector() * Fraction);
				const std::optional<TerrainSample> Value = Sample(Direction);
				// 内部の元場を照会出来ない場合の返戻
				if(!Value) return {};
				IsDescending = Value->ElevationMeters <= LastHeight;
				LastHeight = Value->ElevationMeters;
			}
			if(HasTurning || !Length || Error > Settings.PositionToleranceMeters || !IsDescending) {
				++Path.RejectedSteps;
				// 最小刻み自体を試さずに停止すると，確認可能な極小より遠くで経路が途切れる
				if(Step <= Settings.MinimumStepMeters) {
					Path.End = TerrainDescentEnd::StepLimit;
					break;
				}
				Step = std::max(Settings.MinimumStepMeters, Step * 0.5);
				continue;
			}
			// 合格した位置だけを公開経路へ追加
			Path.MaximumEstimatedErrorMeters = std::max(Path.MaximumEstimatedErrorMeters, Error);
			Path.Samples.push_back({ *Fine, Next->ElevationMeters, Previous.DistanceMeters + Length });
			Current = Next;
			// ５次の局所誤差に対して余裕が有る時だけ刻みを倍増：2^5 = 32
			if(Error < Settings.PositionToleranceMeters / 32) Step = std::min(Settings.MaximumStepMeters, Step * 2);
		}
		// 停止理由を保持した下降経路の返戻
		return Path;
	}
}
