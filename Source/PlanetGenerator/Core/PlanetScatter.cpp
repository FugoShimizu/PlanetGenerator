#include "PlanetScatter.h"
#include <numbers>

namespace PlanetCore {
	namespace {
		/**
		 * 符号無整数の固定演算で乱数の状態を混合する
		 * @param Value 入力の状態
		 * @return 混合後の状態
		 */
		std::uint64_t Mix(std::uint64_t Value) {
			Value += 0x9e3779b97f4a7c15ULL;
			Value = (Value ^ (Value >> 30)) * 0xbf58476d1ce4e5b9ULL;
			Value = (Value ^ (Value >> 27)) * 0x94d049bb133111ebULL;
			// 混合した状態の返戻
			return Value ^ (Value >> 31);
		}

		/**
		 * 端点を含まない一様乱数を生成する
		 * @param State 更新する固定幅の状態
		 * @return ０より大きく１より小さい値
		 */
		double Random(std::uint64_t &State) {
			State = Mix(State);
			// ５２ビットを使い，丸めで１に到達しない中点値の返戻
			return (static_cast<double>(State >> 12) + 0.5) / 4503599627370496.0;
		}

		/**
		 * 立方体の面の座標を三次元へ対応させる
		 * @param Face 面番号
		 * @param U 横座標
		 * @param V 縦座標
		 * @return 正規化前の位置
		 */
		Vec3 FacePoint(const std::uint8_t Face, const double U, const double V) {
			const double Sign = Face % 2 ? -1 : 1;
			// 各面の座標の返戻
			if(Face < 2) return { Sign, U, V };
			// 各面の座標の返戻
			if(Face < 4) return { U, Sign, V };
			// 各面の座標の返戻
			return { U, V, Sign };
		}

		/**
		 * 球面上の球形範囲を包含する投影座標の区間を求める
		 * @param Component 接線方向の中心成分
		 * @param Normal 面法線方向の中心成分
		 * @param Chord 球形範囲の弦半径
		 * @return 立方体面へ切り詰めた座標の下限・上限
		 */
		std::array<double, 2> ProjectBounds(const double Component, const double Normal, const double Chord) {
			const double Low = std::max(1 / std::sqrt(3.0), Normal - Chord), High = std::min(1.0, Normal + Chord);
			const std::array<double, 4> Ratios{
				(Component - Chord) / Low, (Component - Chord) / High,
				(Component + Chord) / Low, (Component + Chord) / High
			};
			// 保守的な面内区間の返戻
			return { std::max(-1.0, *std::min_element(Ratios.begin(), Ratios.end())),
				std::min(1.0, *std::max_element(Ratios.begin(), Ratios.end())) };
		}
	}

	/**
	 * 配置定義を検証し，全球で固定する分割数を決める
	 * @param InSettings Seed・層・半径・候補間隔
	 * @return 有効な配置器，不正な定義では空
	 */
	std::optional<PlanetScatter> PlanetScatter::Create(const ScatterSettings &InSettings) {
		// 不正な物理寸法の返戻
		if(!std::isfinite(InSettings.RadiusMeters) || InSettings.RadiusMeters < 1000 ||
			!std::isfinite(InSettings.SpacingMeters) || InSettings.SpacingMeters <= 0) return {};
		const double Resolution = std::ceil(2 * InSettings.RadiusMeters / InSettings.SpacingMeters);
		// 倍精度でセル内の変化を保持出来る範囲を越えた場合の返戻
		if(!std::isfinite(Resolution) || Resolution < 1 || Resolution > 1099511627776.0) return {};
		PlanetScatter Result;
		Result.Settings = InSettings;
		Result.CellsPerSide = static_cast<std::uint64_t>(Resolution);
		// 検証済み配置器の返戻
		return Result;
	}

	/**
	 * セルに固定した候補を生成し，球面への射影面積に応じて密度を補正する
	 * @param Cell 面とセル番号
	 * @return 面積補正で残る候補，範囲外又は間引くセルでは空
	 */
	std::optional<ScatterCandidate> PlanetScatter::Candidate(const ScatterCell Cell) const {
		// 不正なセルの返戻
		if(Cell.Face >= 6 || Cell.U >= CellsPerSide || Cell.V >= CellsPerSide) return {};
		std::uint64_t State = Mix(Settings.Seed ^ Mix(Settings.Layer) ^ 0x7d32a68bf409e151ULL);
		State = Mix(State ^ Cell.Face);
		State = Mix(State ^ Cell.U);
		State = Mix(State ^ Cell.V);
		const double U = 2 * (Cell.U + 0.001 + 0.998 * Random(State)) / CellsPerSide - 1;
		const double V = 2 * (Cell.V + 0.001 + 0.998 * Random(State)) / CellsPerSide - 1;
		const double SquaredLength = 1 + U * U + V * V;
		// 正射影したセルの面積が小さい所を間引き，期待密度を面間で揃える返戻
		if(Random(State) > 1 / (SquaredLength * std::sqrt(SquaredLength))) return {};
		const UnitDirection Direction = *UnitDirection::FromVector(FacePoint(Cell.Face, U, V));
		const double Selection = Random(State), Rotation = Random(State), Size = 0.65 + 0.7 * Random(State);
		// 地形や季節から独立した候補の返戻
		return ScatterCandidate{ Cell, Direction, Selection, Rotation, Size };
	}

	/**
	 * 地表の円形範囲を検索し，完了した候補だけを返す
	 * @param Center 地表範囲の中心
	 * @param ArcRadiusMeters 球面に沿った検索半径
	 * @param MaximumExaminedCells 調べてよいセル数の上限
	 * @param Stop 中断要求
	 * @return 検索結果，範囲不正・予算超過・中断の場合は部分結果を返さず空
	 */
	std::optional<ScatterQueryResult> PlanetScatter::Query(
		const UnitDirection Center, const double ArcRadiusMeters, const std::uint64_t MaximumExaminedCells,
		const std::stop_token Stop
	) const {
		// 不正な範囲又は中断時の返戻
		if(!std::isfinite(ArcRadiusMeters) || ArcRadiusMeters < 0 || Stop.stop_requested()) return {};
		const double Angle = std::min(std::numbers::pi, ArcRadiusMeters / Settings.RadiusMeters);
		const double Chord = 2 * std::sin(Angle / 2), ChordSquared = Chord * Chord;
		const Vec3 Position = Center.Vector();
		ScatterQueryResult Result;
		for(std::uint8_t Face = 0; Face < 6; ++Face) {
			const double Sign = Face % 2 ? -1 : 1;
			const double Normal = Sign * (Face < 2 ? Position.X : Face < 4 ? Position.Y : Position.Z);
			if(Normal + Chord < 1 / std::sqrt(3.0)) continue;
			const double U = Face < 2 ? Position.Y : Position.X, V = Face < 4 ? Position.Z : Position.Y;
			const std::array<double, 2> BoundsU = ProjectBounds(U, Normal, Chord), BoundsV = ProjectBounds(V, Normal, Chord);
			if(BoundsU[0] > BoundsU[1] || BoundsV[0] > BoundsV[1]) continue;
			// 境界の丸めを含むセルを一つ余分に調べ，最終の距離判定で切り詰める
			const std::uint64_t BeginU = static_cast<std::uint64_t>(std::max(0.0, (BoundsU[0] + 1) * CellsPerSide / 2 - 1));
			const std::uint64_t BeginV = static_cast<std::uint64_t>(std::max(0.0, (BoundsV[0] + 1) * CellsPerSide / 2 - 1));
			const std::uint64_t EndU = static_cast<std::uint64_t>(std::min<double>(CellsPerSide, (BoundsU[1] + 1) * CellsPerSide / 2 + 2));
			const std::uint64_t EndV = static_cast<std::uint64_t>(std::min<double>(CellsPerSide, (BoundsV[1] + 1) * CellsPerSide / 2 + 2));
			const std::uint64_t Width = EndU - BeginU, Height = EndV - BeginV;
			// 積を溢れさせず，走査前にセル数の上限を検査する返戻
			if(Height && Width > (MaximumExaminedCells - Result.ExaminedCells) / Height) return {};
			for(std::uint64_t x = BeginU; x < EndU; ++x) {
				// 行ごとに中断を反映する返戻
				if(Stop.stop_requested()) return {};
				for(std::uint64_t y = BeginV; y < EndV; ++y) {
					++Result.ExaminedCells;
					if(const std::optional<ScatterCandidate> Point = Candidate({ x, y, Face })) {
						const Vec3 Difference = Point->Direction.Vector() - Position;
						if(Difference.Dot(Difference) <= ChordSquared) Result.Candidates.push_back(*Point);
					}
				}
			}
		}
		// 完了した範囲の返戻
		return Result;
	}

	/**
	 * 全球で固定する面ごとのセル数を読む
	 * @return 一辺の分割数
	 */
	std::uint64_t PlanetScatter::Resolution() const {
		// 分割数の返戻
		return CellsPerSide;
	}
}
