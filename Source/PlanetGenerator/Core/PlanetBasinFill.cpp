#include "PlanetBasinFill.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		constexpr double PositiveHalfWidthMeters = 1;

		struct BasinCapacity {
			double Value = 0;
			double Derivative = 0;
		};

		/**
		 * 二つの面の高い方を下回らず，元地形と補間面の勾配を非負の重みで接続する
		 * @param Difference 補間面から元地形を引いた標高差
		 * @return 元地形へ加える非負の標高と，標高差に対する０～１の微分
		 */
		BasinCapacity PositiveRaise(const double Difference) {
			// 元地形が十分に高い領域を変更しない返戻
			if(Difference <= -PositiveHalfWidthMeters) return {};
			// 十分に高い補間面を其の儘使う返戻
			if(Difference >= PositiveHalfWidthMeters) return { Difference, 1 };
			const double T = (Difference + PositiveHalfWidthMeters) / (2 * PositiveHalfWidthMeters), Square = T * T;
			// 平滑な段階関数を積分し，二階迄接続した上包絡の返戻
			return { PositiveHalfWidthMeters * Square * T * (2 - T), Square * (3 - 2 * T) };
		}

		/**
		 * 必要な非負盛土を，同じ単調な平滑化の前の標高差へ戻す
		 * @param Raise 海岸合成前に必要な盛土量
		 * @return 標本拘束を下回らない上側の逆像
		 */
		double InvertPositiveRaise(const double Raise) {
			// 盛土０の正規入力に対する遷移開始位置の返戻
			if(Raise <= 0.0) return -PositiveHalfWidthMeters;
			// 線形域は其の儘使う返戻
			if(Raise >= PositiveHalfWidthMeters) return Raise;
			double Low = -PositiveHalfWidthMeters, High = PositiveHalfWidthMeters;
			for(unsigned Step = 0; Step < 64; ++Step) {
				const double Mid = Low + 0.5 * (High - Low);
				if(PositiveRaise(Mid).Value < Raise) Low = Mid;
				else High = Mid;
			}
			// 下限側へ丸めない平滑化の逆像の返戻
			return High;
		}

		/**
		 * 海面で二階迄零となる盛土容量と，元標高に対する微分を求める
		 * @param Height 海面より高い正の標高
		 * @param Band 正の遷移尺度
		 * @param Ratio 正の相対上限
		 * @return 容量と其の微分，微分の上限は 9 Ratio / 8
		 */
		BasinCapacity CoastalCapacity(const double Height, const double Band, const double Ratio) {
			const double T = Height / std::hypot(Height, Band), Square = T * T;
			// h³ / (h² ＋ b²) を二乗の溢れを避けて計算する返戻
			return { Ratio * Height * Square, Ratio * Square * (3 - 2 * Square) };
		}

		/**
		 * 容量の半分迄は元の盛土を保ち，1.5 倍で容量に達する C2 の単調な飽和を求める
		 * @param X 盛土量を正の容量で割った値
		 * @return 正規化した盛土量と X に対する微分
		 */
		BasinCapacity SaturatedRaise(const double X) {
			// 小さな盛土を変えない返戻
			if(X <= 0.5) return { X, 1 };
			// 容量より大きくしない返戻
			if(X >= 1.5) return { 1, 0 };
			const double T = X - 0.5, Square = T * T;
			// 値・一階・二階を両端で接続する返戻
			return { 0.5 + T - Square * T + 0.5 * Square * Square, 1 - 3 * Square + 2 * Square * T };
		}

		/**
		 * 海面で値・勾配・二階微分が零となる盛土の支持率を求める
		 * @param Height 海面からの標高
		 * @param Band 海岸遷移の標高幅，正の有限値
		 * @return 陸上で０～１となる支持率
		 */
		double CoastMask(const double Height, const double Band) {
			// 大洋では盛土を加えない返戻
			if(Height <= 0) return 0;
			// 遷移域より高い地点の完全寄与の返戻
			if(Height >= Band) return 1;
			const double T = Height / Band;
			// 海岸と内陸の両端で二階迄接続する支持率の返戻
			return T * T * T * (10 + T * (-15 + 6 * T));
		}

		/**
		 * 必要な実盛土を，単調な海岸合成の前に置く補間値へ逆変換する
		 * @param Original 元標高
		 * @param Minimum 必要な最低標高
		 * @param Sea 固定海面
		 * @param Band 正の海岸尺度
		 * @param Ratio 相対容量，０なら既存の固定帯比較式
		 * @return 補間すべき値，海底の埋立又は容量を越える要求では空
		 */
		std::optional<double> RequiredFloor(
			const double Original,
			const double Minimum,
			const double Sea,
			const double Band,
			const double Ratio
		) {
			// 盛土０を保つ為，接続域の補間値だけを其の開始位置迄下げる返戻
			if(Minimum <= Original) return std::min(Minimum, Original - PositiveHalfWidthMeters);
			const double Height = Original - Sea, Raise = Minimum - Original;
			// 固定した大洋を埋立てない返戻
			if(Height <= 0) return {};
			if(!Ratio) {
				const double Mask = CoastMask(Height, Band);
				// 有限な補間値で表せない固定帯の要求の返戻
				if(Mask <= 0) return {};
				// 固定帯と平滑化の両方を逆変換した標高の返戻
				return Original + InvertPositiveRaise(Raise / Mask);
			}
			const double Capacity = CoastalCapacity(Height, Band, Ratio).Value;
			// 実際の合成で表せない容量を使用しない返戻
			if(!std::isfinite(Capacity) || Capacity <= 0) return {};
			const double Ceiling = Original + Capacity;
			// 引算の丸めで容量丁度の正規入力を拒否せず，実際の上限と比較する返戻
			if(Minimum > Ceiling) return {};
			// 上限丁度なら飽和域の始点を使用する返戻
			if(Minimum == Ceiling) return Original + InvertPositiveRaise(1.5 * Capacity);
			// 海岸側の線形部分では正値化だけを逆変換する返戻
			if(Raise <= 0.5 * Capacity) return Original + InvertPositiveRaise(Raise);
			const double Target = Raise / Capacity;
			double Low = 0.5, High = 1.5;
			for(unsigned Step = 0; Step < 64; ++Step) {
				const double Mid = Low + 0.5 * (High - Low);
				if(SaturatedRaise(Mid).Value < Target) Low = Mid;
				else High = Mid;
			}
			// 上側の逆像を使い，下限を下回らない補間値の返戻
			return Original + InvertPositiveRaise(High * Capacity);
		}
	}

	/**
	 * 補間面と元地形の高い方を C2 の上包絡で接続し，海岸で滑らかに消える盛土を加える
	 * @tparam Derivatives 解析勾配を加える場合に真
	 * @param Direction 元地形と同じ評価方向
	 * @param Original 同じ生成定義の元地形の値と勾配
	 * @return 盛土後の地形，海面以下の元地形は値と勾配を其の儘保持する
	 */
	template<bool Derivatives> TerrainSample PlanetBasinFill::ApplyImpl(
		const UnitDirection Direction,
		const TerrainSample Original
	) const {
		const double Height = Original.ElevationMeters - SeaLevelMeters;
		// 大洋の形状と接線を厳密に保持する返戻
		if(Height <= 0) return Original;
		TerrainSample Floor;
		if constexpr(Derivatives) Floor = Surface.Evaluate(Direction);
		else Floor.ElevationMeters = Surface.EvaluateElevation(Direction);
		// 元地形との差を評価し，標本間の窪地にも同じ盆地底を与える
		const double Difference = Floor.ElevationMeters - Original.ElevationMeters;
		// 平滑化の支持より十分に高い元地形の返戻
		if(Difference <= -PositiveHalfWidthMeters) return Original;
		const BasinCapacity Positive = PositiveRaise(Difference);
		const double Raise = Positive.Value, PositiveDerivative = Positive.Derivative;
		double Applied = 0, ByRaise = 0, ByHeight = 0;
		if(RelativeRaiseLimit > 0) {
			const BasinCapacity Capacity = CoastalCapacity(Height, CoastBandMeters, RelativeRaiseLimit);
			// 海面極近傍で容量が表現下限へ丸められた場合の返戻
			if(Capacity.Value <= 0.0) return Original;
			const double X = Raise / Capacity.Value;
			const BasinCapacity Blend = SaturatedRaise(X);
			Applied = Capacity.Value * Blend.Value;
			ByRaise = Blend.Derivative;
			ByHeight = Capacity.Derivative * (Blend.Value - (Blend.Derivative ? X * Blend.Derivative : 0));
		} else {
			const double Mask = CoastMask(Height, CoastBandMeters), T = std::min(1.0, Height / CoastBandMeters);
			Applied = Raise * Mask;
			ByRaise = Mask;
			ByHeight = Raise * (30 * T * T * (1 - T) * (1 - T) / CoastBandMeters);
		}
		TerrainSample Result { Original.ElevationMeters + Applied, {} };
		if constexpr(Derivatives) {
			Result.TangentialGradientPerRadian = Original.TangentialGradientPerRadian * (1 + ByHeight - ByRaise * PositiveDerivative) +
			Floor.TangentialGradientPerRadian * (ByRaise * PositiveDerivative);
		}
		// 陸だけを上げ，大洋の集合を変えない地形の返戻
		return Result;
	}

	/**
	 * 元地形との上包絡へ入る前の補間値と解析勾配を診断する
	 * @param Direction 球面上の評価方向
	 * @return 海岸保持・盛土の正値化を適用していない補間面
	 */
	TerrainSample PlanetBasinFill::EvaluateReconstruction(const UnitDirection Direction) const {
		// 元の地形・水位と区別した補間面の返戻
		return Surface.Evaluate(Direction);
	}

	/**
	 * 元地形へ盛土と解析勾配を加える
	 * @param Direction 元地形を取得した方向
	 * @param Original 同じ生成定義の値と勾配
	 * @return 海岸と大洋を保持する盛土後の地形
	 */
	TerrainSample PlanetBasinFill::Apply(const UnitDirection Direction, const TerrainSample Original) const {
		// 勾配を含めた同じ盛土式の返戻
		return ApplyImpl<true>(Direction, Original);
	}

	/**
	 * 元地形へ同じ盛土の値だけを加える
	 * @param Direction 元地形を取得した方向
	 * @param OriginalElevation 同じ生成定義の標高
	 * @return 同じ式に依る盛土後の標高
	 */
	double PlanetBasinFill::ApplyElevation(const UnitDirection Direction, const double OriginalElevation) const {
		// 微分計算を省いた同じ盛土式の返戻
		return ApplyImpl<false>(Direction, { OriginalElevation, {} }).ElevationMeters;
	}

	/**
	 * 平滑化に依り二つの面の高い方を越える標高の保守的な上限を返す
	 * @return 最大の超過は両面が等しい位置の 3 B / 16，海岸合成は此れを増やさない
	 */
	double PlanetBasinFill::SmoothingOvershootBoundMeters() {
		// 実際の平滑化と同じ尺度から求めた超過の上限の返戻
		return 3 * PositiveHalfWidthMeters / 16;
	}

	/**
	 * 元に為る有限支持補間の検査値を参照する
	 * @return 再構成の残差・反復数・規模
	 */
	const BasinReconstructionStatistics &PlanetBasinFill::Statistics() const {
		// 補間準備の統計の返戻
		return Surface.Statistics();
	}

	/**
	 * 検証済みの補間と海岸条件を保持する
	 * @param Reconstruction 不変な有限支持補間
	 * @param Sea 固定海面
	 * @param CoastBand 海岸の遷移標高幅
	 * @param RelativeLimit 元標高に対する容量，０なら固定帯の比較用
	 */
	PlanetBasinFill::PlanetBasinFill(
		PlanetBasinReconstruction Reconstruction,
		const double Sea,
		const double CoastBand,
		const double RelativeLimit
	) : Surface(std::move(Reconstruction)), SeaLevelMeters(Sea), CoastBandMeters(CoastBand), RelativeRaiseLimit(RelativeLimit) {}

	/**
	 * 海域を変えず，指定した盆地底より低い陸上の元地形を面状に盛土する準備を行う
	 * 計算量：O(n log n ＋ k m)，記憶量：O(n ＋ m)，n は標本数，m は支持内の結合数，k は反復数
	 * @param Level 全球を覆う固定球面網の細分段階，０～６
	 * @param OriginalElevations 元地形の同地点の標高
	 * @param MinimumElevations 標本で必要な地面の最低標高，元地形を下回る値も盆地底の形状として用いる
	 * @param Sea 固定海面
	 * @param CoastBand 海岸遷移の正の標高幅
	 * @param RelativeLimit 相対容量，０なら固定帯比較式
	 * @param Stop 中断要求
	 * @return 大洋の形を保つ非負の盛土，入力不正・標本での必要盛土を表現出来ない場合・中断では空
	 */
	std::optional<PlanetBasinFill> PlanetBasinFill::CreateImpl(
		const unsigned Level,
		const std::span<const double> OriginalElevations,
		const std::span<const double> MinimumElevations,
		const double Sea,
		const double CoastBand,
		const double RelativeLimit,
		const std::stop_token Stop
	) {
		const bool InvalidDefinition =
		Level > 6 || OriginalElevations.empty() || OriginalElevations.size() != MinimumElevations.size() || !std::isfinite(Sea) ||
		!std::isfinite(CoastBand) || CoastBand <= 0 || !std::isfinite(RelativeLimit) || RelativeLimit < 0 || Stop.stop_requested();
		// 不正な標本又は固定境界の返戻
		if(InvalidDefinition) return {};
		const std::optional<SphereMesh> Mesh = BuildUniformSphere(static_cast<std::uint8_t>(Level));
		// 元地形の標本と固定網が一致しない場合の返戻
		if(!Mesh || Mesh->Directions.size() != OriginalElevations.size()) return {};
		const std::span<const UnitDirection> Directions = Mesh->Directions;
		std::vector<double> Targets;
		Targets.reserve(Directions.size());
		for(std::size_t Index = 0; Index < Directions.size(); ++Index) {
			const double Original = OriginalElevations[Index], Minimum = MinimumElevations[Index];
			// 不正な標高又は中断された入力の返戻
			if(!std::isfinite(Original) || !std::isfinite(Minimum) || Stop.stop_requested()) return {};
			const std::optional<double> Target = RequiredFloor(Original, Minimum, Sea, CoastBand, RelativeLimit);
			// 海底の埋立・容量超過・非有限な必要盛土を拒否する返戻
			if(!Target || !std::isfinite(*Target)) return {};
			Targets.push_back(*Target);
		}
		std::optional<PlanetBasinReconstruction> Reconstruction = PlanetBasinReconstruction::CreateGlobal(Level, Targets, Stop);
		// 未収束又は中断した面を公開しない返戻
		if(!Reconstruction) return {};
		PlanetBasinFill Result(std::move(*Reconstruction), Sea, CoastBand, RelativeLimit);
		for(std::size_t Index = 0; Index < Directions.size(); ++Index) {
			// 公開前の中断時の返戻
			if(Stop.stop_requested()) return {};
			const double Actual = Result.ApplyElevation(Directions[Index], OriginalElevations[Index]);
			const double Expected = std::max(OriginalElevations[Index], MinimumElevations[Index]);
			// 正値化と海岸遷移を含めた実際の標本誤差を確認する返戻
			if(!std::isfinite(Actual) || std::abs(Actual - Expected) > 2E-6) return {};
		}
		// 大洋の海底・海岸を維持する不変な盛土場の返戻
		return Result;
	}

	/**
	 * 固定幅で海岸を消す従来の比較用の盛土を準備する
	 * @param Level 固定網の段階
	 * @param OriginalElevations 元標高
	 * @param MinimumElevations 最低標高
	 * @param Sea 海面
	 * @param CoastBand 固定海岸幅
	 * @param Stop 中断要求
	 * @return 従来式の比較用場，不正・準備失敗・中断では空
	 */
	std::optional<PlanetBasinFill> PlanetBasinFill::Create(
		const unsigned Level,
		const std::span<const double> OriginalElevations,
		const std::span<const double> MinimumElevations,
		const double Sea,
		const double CoastBand,
		const std::stop_token Stop
	) {
		// 以前の失敗も再現出来る固定帯の比較経路の返戻
		return CreateImpl(Level, OriginalElevations, MinimumElevations, Sea, CoastBand, 0, Stop);
	}

	/**
	 * 海岸で盛土量に比例する勾配増幅を避ける，容量付きの合成を準備する
	 * @param Level 固定網の段階
	 * @param OriginalElevations 元標高
	 * @param MinimumElevations 容量内で満たす最低標高
	 * @param Sea 固定海面
	 * @param CoastBand 容量が海岸で消える正の尺度
	 * @param RelativeLimit 正の相対容量
	 * @param Stop 中断要求
	 * @return 下限と容量を共に保持する場，不正・容量超過・準備失敗・中断では空
	 */
	std::optional<PlanetBasinFill> PlanetBasinFill::CreateWithCoastalLimit(
		const unsigned Level,
		const std::span<const double> OriginalElevations,
		const std::span<const double> MinimumElevations,
		const double Sea,
		const double CoastBand,
		const double RelativeLimit,
		const std::stop_token Stop
	) {
		// 固定帯比較式への意図しない切替を拒否する返戻
		if(RelativeLimit <= 0) return {};
		// 水位の準備と同じ容量を用いる実合成の返戻
		return CreateImpl(Level, OriginalElevations, MinimumElevations, Sea, CoastBand, RelativeLimit, Stop);
	}

	/**
	 * 水位を先に合わせる為，同じ海岸容量を公開する
	 * @param HeightAboveSea 元標高から固定海面を引いた値
	 * @param CoastBand 正の尺度
	 * @param RelativeLimit 正の相対容量
	 * @return 非負の盛土容量，不正な定義では NaN
	 */
	double PlanetBasinFill::CoastalRaiseCapacity(const double HeightAboveSea, const double CoastBand, const double RelativeLimit) {
		const bool InvalidDefinition =
		!std::isfinite(HeightAboveSea) || !std::isfinite(CoastBand) || !std::isfinite(RelativeLimit) || CoastBand <= 0 ||
		RelativeLimit <= 0;
		// 不正な上限を有効な水位に渡さない返戻
		if(InvalidDefinition) return std::numeric_limits<double>::quiet_NaN();
		// 大洋へ盛土しない返戻
		if(HeightAboveSea <= 0) return 0;
		const double Capacity = CoastalCapacity(HeightAboveSea, CoastBand, RelativeLimit).Value;
		// 表現出来る有限な容量だけを水位の上限へ渡す返戻
		return std::isfinite(Capacity) ? Capacity : std::numeric_limits<double>::quiet_NaN();
	}

	/**
	 * 河道内部や両岸の最低標高を，既存の盆地と同じ広がりを持つ面へ加える
	 * 計算量：O(q k log k ＋ n)，記憶量：O(q ＋ n ＋ k)，q は追加地点数，n は元標本数，k は索引候補数
	 * @param Directions 追加地点の方向，順序を固定する
	 * @param OriginalElevations 同じ生成定義の元標高
	 * @param MinimumElevations 必要な最低標高，既存面との等値一致は要求しない
	 * @param Stop 中断要求
	 * @return 元の面を下げず海底を保持する盛土，入力不正・海底の埋立要求・数値失敗・中断では空
	 */
	std::optional<PlanetBasinFill> PlanetBasinFill::WithMinimumElevations(
		const std::span<const UnitDirection> Directions,
		const std::span<const double> OriginalElevations,
		const std::span<const double> MinimumElevations,
		const std::stop_token Stop
	) const {
		const bool InvalidInputs = Directions.size() > 1000000 || Directions.size() != OriginalElevations.size() ||
		Directions.size() != MinimumElevations.size() || Stop.stop_requested();
		// 配列の対応を失った指定や中断の返戻
		if(InvalidInputs) return {};
		std::vector<UnitDirection> Locations;
		std::vector<double> Targets;
		for(std::size_t Index = 0; Index < Directions.size(); ++Index) {
			const double Original = OriginalElevations[Index], Minimum = MinimumElevations[Index];
			// 不正な入力又は準備中の中断の返戻
			if(!std::isfinite(Original) || !std::isfinite(Minimum) || Stop.stop_requested()) return {};
			if(Minimum <= Original) continue;
			const std::optional<double> Target = RequiredFloor(Original, Minimum, SeaLevelMeters, CoastBandMeters, RelativeRaiseLimit);
			// 海底の埋立・容量超過・非有限な必要盛土を拒否する返戻
			if(!Target || !std::isfinite(*Target)) return {};
			Locations.push_back(Directions[Index]);
			Targets.push_back(*Target);
		}
		std::optional<PlanetBasinReconstruction> Reconstruction = Surface.WithLowerBounds(Locations, Targets, Stop);
		// 全追加地点の下限を満たせなかった場合の返戻
		if(!Reconstruction) return {};
		PlanetBasinFill Result(std::move(*Reconstruction), SeaLevelMeters, CoastBandMeters, RelativeRaiseLimit);
		for(std::size_t Index = 0; Index < Directions.size(); ++Index) {
			const double Actual = Result.ApplyElevation(Directions[Index], OriginalElevations[Index]);
			// 正値化と海岸の遷移を含む最終評価を満たさない場合の返戻
			if(Stop.stop_requested() || !std::isfinite(Actual) || MinimumElevations[Index] - Actual > 2E-6) return {};
		}
		// 新しい下限を満たし，先の下限も下げない不変な盛土場の返戻
		return Result;
	}
}
