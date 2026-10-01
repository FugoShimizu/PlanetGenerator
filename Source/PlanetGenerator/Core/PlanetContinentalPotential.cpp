#include "PlanetContinentalPotential.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 有限で，閉じた単位円板内に在る照会位置を判定する
	 * @param X 第１座標
	 * @param Y 第２座標
	 * @return 有効な位置の場合に真
	 */
	bool ContinentalPotential::IsValidPoint(const double X, const double Y) {
		const double SquaredMagnitude = X * X + Y * Y;
		// 球面へ写像する前の正規座標の判定の返戻
		return std::isfinite(X) && std::isfinite(Y) && SquaredMagnitude <= 1;
	}

	/**
	 * 円板内に完全に収まる正の源と，数値的に表現可能な境界を準備する
	 * @param Settings 正の背景高さと有限源
	 * @param Stop 中断要求
	 * @return 不変な候補，不正・中断・表現範囲外の場合は空
	 */
	std::optional<ContinentalPotential> ContinentalPotential::Create(
		const ContinentalPotentialSettings &Settings,
		const std::stop_token Stop
	) {
		const double Background = Settings.BackgroundMeters;
		const bool IsInvalid =
		!std::isfinite(Background) || Background <= 0 || !std::isfinite(4 * Background) || Settings.Sources.size() > MaximumSources ||
		Stop.stop_requested();
		// 厳密な負の背景 Laplacian 又は有界な資源を準備出来ない入力の返戻
		if(IsInvalid) return {};
		ContinentalPotential Result;
		Result.BackgroundMeters = Background;
		Result.GradientMagnitudeBound = 2 * Background;
		Result.AbsoluteHeightBoundMeters = Background;
		Result.LaplacianMagnitudeBound = 4 * Background;
		Result.Hash = UINT64_C(1469598103934665603) ^ ContinentalPotentialVersion;
		const auto AppendHash = [&](const double Value) -> void {
			Result.Hash = (Result.Hash ^ std::bit_cast<std::uint64_t>(Value)) * UINT64_C(1099511628211);
			return;
		};
		AppendHash(Background);
		Result.Sources.reserve(Settings.Sources.size());
		for(const ContinentalUpliftSource &Input : Settings.Sources) {
			const double Magnitude = std::hypot(Input.X, Input.Y), Radius = Input.Radius, Weight = Input.WeightMeters;
			const bool IsInvalidSource =
			!std::isfinite(Magnitude) || !std::isfinite(Radius) || !std::isfinite(Weight) || Radius <= 0 || Weight <= 0 ||
			!(Magnitude < 1) || !(Radius < 1 - Magnitude);
			// 海岸を跨ぐ源，非有限な源，又は中断された準備の返戻
			if(IsInvalidSource || Stop.stop_requested()) return {};
			const double SquaredRadius = Radius * Radius, InverseSquaredRadius = 1 / SquaredRadius;
			const double CurvatureBound = 16 * InverseSquaredRadius, WeightedCurvatureBound = Weight * CurvatureBound;
			const double HeightBound = Weight * (std::log((1 + Magnitude) / Radius) + 11.0 / 12);
			const double GradientBound = Weight * (3 / Radius + Magnitude / (1 - Magnitude));
			const bool IsUnrepresentable =
			!std::isfinite(CurvatureBound) || !std::isfinite(WeightedCurvatureBound) || !std::isfinite(HeightBound) ||
			!std::isfinite(Result.AbsoluteHeightBoundMeters + HeightBound) ||
			!std::isfinite(Result.GradientMagnitudeBound + GradientBound) ||
			!std::isfinite(Result.LaplacianMagnitudeBound + 6 * Weight * InverseSquaredRadius);
			// 中間微分又は合成値の表現範囲を超える源の返戻
			if(IsUnrepresentable) return {};
			Result.Sources.push_back(
				{ Input.X, Input.Y, Input.X * Input.X + Input.Y * Input.Y, SquaredRadius, InverseSquaredRadius, Weight, Radius, Magnitude }
			);
			Result.AbsoluteHeightBoundMeters += HeightBound;
			Result.GradientMagnitudeBound += GradientBound;
			Result.LaplacianMagnitudeBound += 6 * Weight * InverseSquaredRadius;
			AppendHash(Input.X);
			AppendHash(Input.Y);
			AppendHash(Radius);
			AppendHash(Weight);
		}
		// 別の源の頂点を同じ地点で同時に仮定しない，全域の保守的な高さ上界
		Result.TightenHeightBound();
		// 上界の準備中に届いた中断を公開しない返戻
		if(Stop.stop_requested()) return {};
		// 海岸と有限源の拘束を満たした比較用高さ場の返戻
		return Result;
	}

	/**
	 * 複数の正の超調和場から，海岸を固定する滑らかな最小を構成する
	 * @param LayerSettings 各層の背景隆起と有限源
	 * @param TemperatureMeters 切替えの物理的な標高尺度
	 * @param Stop 準備の中断要求
	 * @return 正の内部・零の海岸と負の Laplacian を持つ不変場
	 */
	std::optional<ContinentalPotential> ContinentalPotential::CreateBlend(
		const std::vector<ContinentalPotentialSettings> &LayerSettings,
		const double TemperatureMeters,
		const std::stop_token Stop
	) {
		const bool IsInvalid = LayerSettings.size() < 2 || LayerSettings.size() > MaximumLayers || !std::isfinite(TemperatureMeters) ||
		TemperatureMeters <= 0 || Stop.stop_requested();
		// 有限な層と温度を持たない合成の返戻
		if(IsInvalid) return {};
		ContinentalPotential Result;
		Result.BlendTemperatureMeters = TemperatureMeters;
		Result.Hash = UINT64_C(1469598103934665603) ^ ContinentalPotentialVersion ^ UINT64_C(0X5F7BACE19D42E683);
		Result.Layers.reserve(LayerSettings.size());
		const auto AppendHash = [&](const std::uint64_t Bits) -> void {
			Result.Hash = (Result.Hash ^ Bits) * UINT64_C(1099511628211);
			return;
		};
		AppendHash(std::bit_cast<std::uint64_t>(TemperatureMeters));
		AppendHash(static_cast<std::uint64_t>(LayerSettings.size()));
		for(const ContinentalPotentialSettings &Settings : LayerSettings) {
			std::optional<ContinentalPotential> Layer = Create(Settings, Stop);
			// 不正な層又は中断を不完全な合成として公開しない返戻
			if(!Layer) return {};
			Result.AbsoluteHeightBoundMeters = std::max(Result.AbsoluteHeightBoundMeters, Layer->AbsoluteHeightBoundMeters);
			Result.GradientMagnitudeBound = std::max(Result.GradientMagnitudeBound, Layer->GradientMagnitudeBound);
			Result.LaplacianMagnitudeBound = std::max(Result.LaplacianMagnitudeBound, Layer->LaplacianMagnitudeBound);
			AppendHash(Layer->Hash);
			Result.Layers.push_back(std::move(*Layer));
		}
		const double AdditionalLaplacian = Result.GradientMagnitudeBound * Result.GradientMagnitudeBound / TemperatureMeters;
		// 共分散と背景の全域上界を表現出来ない合成の返戻
		if(!std::isfinite(AdditionalLaplacian) || !std::isfinite(Result.LaplacianMagnitudeBound + AdditionalLaplacian)) return {};
		Result.LaplacianMagnitudeBound += AdditionalLaplacian;
		// 合成後の不変な源と境界の返戻
		return Stop.stop_requested() ? std::nullopt : std::optional<ContinentalPotential>{ std::move(Result) };
	}

	/**
	 * 最寄りの源の領域毎に，他の源迄の距離下界から全域の高さ上界を狭める
	 * 高さ・勾配の照会式と源の密度は変更しない
	 */
	void ContinentalPotential::TightenHeightBound() {
		// 源無しの背景高さは既に正確な上界
		if(Sources.empty()) return;
		double Bound = 0;
		constexpr double CoordinateMargin = 32 * std::numeric_limits<double>::epsilon();
		for(const PreparedSource &Nearest : Sources) {
			double Candidate = BackgroundMeters;
			for(const PreparedSource &Source : Sources) {
				// 最寄りが j なら，d_i は源 i と j の距離の半分以上
				const double Separation = std::hypot(Source.X - Nearest.X, Source.Y - Nearest.Y);
				const double LowerDistance = std::max(0.0, 0.5 * Separation - CoordinateMargin);
				const double RadiusFloor = std::max(Source.Radius, LowerDistance), Core = LowerDistance < Source.Radius ? 11.0 / 12 : 0;
				Candidate += Source.WeightMeters * (std::log((1 + Source.Magnitude) / RadiusFloor) + Core);
			}
			Bound = std::max(Bound, Candidate);
		}
		// 正の和と距離の丸めに余裕を持たせ，既存の安全上界も保持する
		const double Inflation = 1 + 64 * std::numeric_limits<double>::epsilon() * (static_cast<double>(Sources.size()) + 1);
		const double Rounded = std::nextafter(Bound * Inflation, std::numeric_limits<double>::infinity());
		if(std::isfinite(Rounded) && Rounded > 0) AbsoluteHeightBoundMeters = std::min(AbsoluteHeightBoundMeters, Rounded);
		return;
	}

	/**
	 * 調和的な外部解と有限密度の内部解を，３階迄一致させて評価する
	 * @param X 検査済みの第１座標
	 * @param Y 検査済みの第２座標
	 * @return 高さ，又は高さと解析微分，座標は円板の尺度
	 */
	template<bool Derivatives> ContinentalPotentialSample ContinentalPotential::EvaluateInside(
		const double X,
		const double Y
	) const {
		const double SquaredMagnitude = X * X + Y * Y, Margin = 1 - SquaredMagnitude;
		if(!Layers.empty()) {
			std::array<ContinentalPotentialSample, MaximumLayers> Values {};
			double Minimum = std::numeric_limits<double>::infinity();
			for(std::size_t i = 0; i < Layers.size(); ++i) {
				Values[i] = Layers[i].EvaluateInside<Derivatives>(X, Y);
				Minimum = std::min(Minimum, Values[i].HeightMeters);
			}
			std::array<double, MaximumLayers> Weights {};
			double Sum = 0;
			for(std::size_t i = 0; i < Layers.size(); ++i) {
				Weights[i] = std::exp((Minimum - Values[i].HeightMeters) / BlendTemperatureMeters);
				Sum += Weights[i];
			}
			ContinentalPotentialSample Blend;
			Blend.HeightMeters = Minimum - BlendTemperatureMeters * std::log(Sum / static_cast<double>(Layers.size()));
			if constexpr(Derivatives) {
				double SecondXX = 0, SecondXY = 0, SecondYY = 0;
				for(std::size_t i = 0; i < Layers.size(); ++i) {
					const double Weight = Weights[i] / Sum, GX = Values[i].GradientPerUnit[0], GY = Values[i].GradientPerUnit[1];
					Blend.GradientPerUnit[0] += Weight * GX;
					Blend.GradientPerUnit[1] += Weight * GY;
					for(unsigned Axis = 0; Axis < 3; ++Axis) Blend.HessianPerUnitSquared[Axis] += Weight * Values[i].HessianPerUnitSquared[Axis];
					SecondXX += Weight * GX * GX;
					SecondXY += Weight * GX * GY;
					SecondYY += Weight * GY * GY;
				}
				const double GX = Blend.GradientPerUnit[0], GY = Blend.GradientPerUnit[1];
				Blend.HessianPerUnitSquared[0] -= (SecondXX - GX * GX) / BlendTemperatureMeters;
				Blend.HessianPerUnitSquared[1] -= (SecondXY - GX * GY) / BlendTemperatureMeters;
				Blend.HessianPerUnitSquared[2] -= (SecondYY - GY * GY) / BlendTemperatureMeters;
				Blend.LaplacianPerUnitSquared = Blend.HessianPerUnitSquared[0] + Blend.HessianPerUnitSquared[2];
			}
			// 同じ層から標高と共分散を含む解析微分を返戻
			return Blend;
		}
		ContinentalPotentialSample Result;
		Result.HeightMeters = BackgroundMeters * Margin;
		if constexpr(Derivatives) {
			Result.GradientPerUnit = { -2 * BackgroundMeters * X, -2 * BackgroundMeters * Y };
			Result.HessianPerUnitSquared = { -2 * BackgroundMeters, 0, -2 * BackgroundMeters };
			Result.LaplacianPerUnitSquared = -4 * BackgroundMeters;
		}
		for(const PreparedSource &Source : Sources) {
			// 複素積の成分で計算し，海岸付近の大きな項の差引を避ける
			const double DX = X - Source.X, DY = Y - Source.Y, DistanceSquared = DX * DX + DY * DY, Real = 1 - Source.X * X - Source.Y * Y;
			const double Imaginary = Source.X * Y - Source.Y * X, B = Real * Real + Imaginary * Imaginary;
			const bool IsInsideSource = DistanceSquared < Source.SquaredRadius;
			double Height = 0, RadialCoefficient = 0, RadialHessianCoefficient = 0, QX = 0, QY = 0;
			if(IsInsideSource) {
				const double U = DistanceSquared * Source.InverseSquaredRadius;
				Height = 0.5 * std::log(B * Source.InverseSquaredRadius) + (1 - U) * (11 - 7 * U + 2 * U * U) / 12;
				if constexpr(Derivatives) {
					RadialCoefficient = (-3 + 3 * U - U * U) * Source.InverseSquaredRadius;
					RadialHessianCoefficient = 2 * (3 - 2 * U);
					QX = DX * Source.InverseSquaredRadius;
					QY = DY * Source.InverseSquaredRadius;
					Result.LaplacianPerUnitSquared -= 6 * Source.WeightMeters * Source.InverseSquaredRadius * (1 - U) * (1 - U);
				}
			} else {
				Height = 0.5 * std::log1p(Margin * (1 - Source.SquaredMagnitude) / DistanceSquared);
				if constexpr(Derivatives) {
					const double InverseDistanceSquared = 1 / DistanceSquared;
					RadialCoefficient = -InverseDistanceSquared;
					RadialHessianCoefficient = 2;
					QX = DX * InverseDistanceSquared;
					QY = DY * InverseDistanceSquared;
				}
			}
			Result.HeightMeters += Source.WeightMeters * Height;
			if constexpr(Derivatives) {
				const double GX = -(Source.X * Real + Source.Y * Imaginary) / B, GY = (-Source.Y * Real + Source.X * Imaginary) / B;
				const double Common = Source.SquaredMagnitude / B;
				Result.GradientPerUnit[0] += Source.WeightMeters * (GX + RadialCoefficient * DX);
				Result.GradientPerUnit[1] += Source.WeightMeters * (GY + RadialCoefficient * DY);
				Result.HessianPerUnitSquared[0] +=
				Source.WeightMeters * (Common - 2 * GX * GX + RadialCoefficient + RadialHessianCoefficient * QX * QX);
				Result.HessianPerUnitSquared[1] += Source.WeightMeters * (-2 * GX * GY + RadialHessianCoefficient * QX * QY);
				Result.HessianPerUnitSquared[2] +=
				Source.WeightMeters * (Common - 2 * GY * GY + RadialCoefficient + RadialHessianCoefficient * QY * QY);
			}
		}
		// 同じ源と加算順序を使う高さと，必要な解析微分の返戻
		return Result;
	}

	/**
	 * 任意の生成順で同じ高さを照会する
	 * @param X 第１座標
	 * @param Y 第２座標
	 * @return 有限な高さ，円板外又は非有限な入力・結果は空
	 */
	std::optional<double> ContinentalPotential::EvaluateHeight(const double X, const double Y) const {
		// 基準円板の外又は不正な入力の返戻
		if(!IsValidPoint(X, Y)) return {};
		const double Height = EvaluateInside<false>(X, Y).HeightMeters;
		// 非有限な結果を正常な地形として公開しない返戻
		if(!std::isfinite(Height)) return {};
		// 微分を計算しない同じ高さの返戻
		return Height;
	}

	/**
	 * 高さと独立に検査可能な解析微分を照会する
	 * @param X 第１座標
	 * @param Y 第２座標
	 * @return 有限な高さと微分，円板外又は非有限は空
	 */
	std::optional<ContinentalPotentialSample> ContinentalPotential::Evaluate(const double X, const double Y) const {
		// 基準円板の外又は不正な入力の返戻
		if(!IsValidPoint(X, Y)) return {};
		const ContinentalPotentialSample Result = EvaluateInside<true>(X, Y);
		const bool IsFinite = std::isfinite(Result.HeightMeters) && std::isfinite(Result.LaplacianPerUnitSquared) && std::all_of(
			Result.GradientPerUnit.begin(),
			Result.GradientPerUnit.end(),
			[](const double Value) -> bool {
				return std::isfinite(Value);
			}
		) && std::all_of(
			Result.HessianPerUnitSquared.begin(),
			Result.HessianPerUnitSquared.end(),
			[](const double Value) -> bool {
				return std::isfinite(Value);
			}
		);
		// 表現出来ない微分を持つ地形の返戻
		if(!IsFinite) return {};
		// 円板の座標尺度を明示した解析微分の返戻
		return Result;
	}

	/**
	 * 有限源の内外を跨ぐ円板近傍で，Hessian と其の変化の上界を求める
	 * @param X 近傍の第１座標
	 * @param Y 近傍の第２座標
	 * @param NeighborhoodRadius 正規座標の近傍半径，円板との共通部分だけを対象
	 * @return 有限な保守的上界，座標・尺度の拒否又は表現範囲外の場合は空
	 */
	std::optional<ContinentalPotentialDerivativeBounds> ContinentalPotential::GetDerivativeBounds(
		const double X,
		const double Y,
		const double NeighborhoodRadius
	) const {
		// 円板外の位置又は不正な近傍から上界を作らない返戻
		if(!IsValidPoint(X, Y) || !std::isfinite(NeighborhoodRadius) || NeighborhoodRadius < 0 || NeighborhoodRadius > 2) return {};
		if(!Layers.empty()) {
			double Hessian = 0, Third = 0, Gradient = 0;
			for(const ContinentalPotential &Layer : Layers) {
				const std::optional<ContinentalPotentialDerivativeBounds> Bounds = Layer.GetDerivativeBounds(X, Y, NeighborhoodRadius);
				const std::optional<ContinentalPotentialSample> Center = Layer.Evaluate(X, Y);
				// 各層の近傍上界と中心勾配が不足する場合の返戻
				if(!Bounds || !Center) return {};
				Hessian = std::max(Hessian, Bounds->HessianNormPerUnitSquared);
				Third = std::max(Third, Bounds->ThirdDerivativeNormPerUnitCubed);
				Gradient = std::max(
					Gradient,
					std::hypot(Center->GradientPerUnit[0], Center->GradientPerUnit[1]) + Bounds->HessianNormPerUnitSquared * NeighborhoodRadius
				);
			}
			const double Temperature = BlendTemperatureMeters;
			ContinentalPotentialDerivativeBounds Result;
			Result.HessianNormPerUnitSquared = Hessian + Gradient * Gradient / Temperature;
			Result.ThirdDerivativeNormPerUnitCubed =
			Third + 12 * Hessian * Gradient / Temperature + 16 * Gradient * Gradient * Gradient / (Temperature * Temperature);
			const bool IsFinite = std::isfinite(Result.HessianNormPerUnitSquared) && std::isfinite(Result.ThirdDerivativeNormPerUnitCubed);
			// 合成微分の近傍上界を解像出来ない場合の返戻
			if(!IsFinite) return {};
			const double Margin = 1 + 64 * std::numeric_limits<double>::epsilon() * (static_cast<double>(GetSourceCount()) + 1);
			Result.HessianNormPerUnitSquared =
			std::nextafter(Result.HessianNormPerUnitSquared * Margin, std::numeric_limits<double>::infinity());
			Result.ThirdDerivativeNormPerUnitCubed =
			std::nextafter(Result.ThirdDerivativeNormPerUnitCubed * Margin, std::numeric_limits<double>::infinity());
			// 区間微分の保守的な合成上界の返戻
			return Result;
		}
		const double Epsilon = std::numeric_limits<double>::epsilon(), CoordinateMargin = 32 * Epsilon;
		const double MagnitudeBound = std::min(1.0, std::hypot(X, Y) + NeighborhoodRadius + CoordinateMargin);
		ContinentalPotentialDerivativeBounds Result;
		Result.HessianNormPerUnitSquared = 2 * BackgroundMeters;
		for(const PreparedSource &Source : Sources) {
			const double Distance = std::hypot(X - Source.X, Y - Source.Y);
			const double Lower = std::max(0.0, Distance - NeighborhoodRadius - CoordinateMargin);
			const double Upper = Distance + NeighborhoodRadius + CoordinateMargin;
			const double ImageDistance = 1 - Source.Magnitude * MagnitudeBound - CoordinateMargin;
			// 海岸に近過ぎて鏡像項の下界を数値的に解像出来ない源の返戻
			if(!(ImageDistance > 0)) return {};
			double RadialHessian = 0, RadialThird = 0;
			if(Lower < Source.Radius) {
				// 内部の接線固有値は３／σ² 以下，３階の変化は２６ｄ／σ⁴ 以下
				RadialHessian = 3 * Source.InverseSquaredRadius;
				RadialThird = 26 * std::min(Upper, Source.Radius) * Source.InverseSquaredRadius * Source.InverseSquaredRadius;
			} else {
				// 外部の対数解の作用素ノルムは１／ｄ²と２／ｄ³
				const double Inverse = 1 / Lower;
				RadialHessian = Inverse * Inverse;
				RadialThird = 2 * RadialHessian * Inverse;
			}
			const double ImageRatio = Source.Magnitude / ImageDistance, ImageSquare = ImageRatio * ImageRatio;
			Result.HessianNormPerUnitSquared += Source.WeightMeters * (RadialHessian + ImageSquare);
			Result.ThirdDerivativeNormPerUnitCubed += Source.WeightMeters * (RadialThird + 2 * ImageSquare * ImageRatio);
		}
		// 正の和の加算と初等演算の丸めに保守的な余裕を付与
		const double ArithmeticMargin = 1 + 64 * Epsilon * (static_cast<double>(Sources.size()) + 1);
		Result.HessianNormPerUnitSquared =
		std::nextafter(Result.HessianNormPerUnitSquared * ArithmeticMargin, std::numeric_limits<double>::infinity());
		Result.ThirdDerivativeNormPerUnitCubed =
		std::nextafter(Result.ThirdDerivativeNormPerUnitCubed * ArithmeticMargin, std::numeric_limits<double>::infinity());
		const bool IsFinite = std::isfinite(Result.HessianNormPerUnitSquared) && std::isfinite(Result.ThirdDerivativeNormPerUnitCubed);
		// 解像出来ない上界を正常な証明値として公開しない返戻
		if(!IsFinite) return {};
		// 源の密度境界も含む有限な作用素上界の返戻
		return Result;
	}

	/**
	 * 全入力位置に対する保守的な高さ上界を得る
	 * @return メートル単位の上界
	 */
	double ContinentalPotential::GetAbsoluteHeightBoundMeters() const {
		return AbsoluteHeightBoundMeters;
	}

	/**
	 * 円板座標での Laplacian の絶対上界を得る
	 * @return メートル毎正規座標二乗の上界
	 */
	double ContinentalPotential::GetLaplacianMagnitudeBound() const {
		return LaplacianMagnitudeBound;
	}

	/**
	 * 保持する有限源の数を得る
	 * @return 準備済みの源数
	 */
	std::size_t ContinentalPotential::GetSourceCount() const {
		// 合成の各層に含む有限源の総数
		if(!Layers.empty()) {
			std::size_t Count = 0;
			for(const ContinentalPotential &Layer : Layers) Count += Layer.GetSourceCount();
			// 各層を合計した源数の返戻
			return Count;
		}
		// 単層の有限源の個数の返戻
		return Sources.size();
	}

	/**
	 * 仕様版と全入力の順序を含む識別子を得る
	 * @return 実行時共有の為の識別子，保存データではない
	 */
	std::uint64_t ContinentalPotential::GetDefinitionHash() const {
		return Hash;
	}
}
