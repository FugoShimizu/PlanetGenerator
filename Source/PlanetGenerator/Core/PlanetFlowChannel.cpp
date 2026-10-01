#include "PlanetFlowChannel.h"
#include <algorithm>
#include <cmath>
#include <optional>

namespace PlanetCore {
	namespace {
		/**
		 * 五次接続を近い端から評価し，１の近傍で上限を越える桁落ちを防ぐ
		 * @param T 検査済の０以上１以下の位置
		 * @return 元の五次式と同じ接続値；上限を後から切詰めない
		 */
		double UnitTransition(const double T) {
			const double Near = T < 0.5 ? T : 1 - T, Value = Near * Near * Near * (10 + Near * (-15 + 6 * Near));
			// 対称性に依って両端の表現精度を保った値の返戻
			return T < 0.5 ? Value : 1 - Value;
		}
	}

	/**
	 * 球面の局所断面として有効な尺度を検査する
	 * @return 有限な半径・幅・深さで，幅が半径の５％未満且つ深さが幅未満なら真
	 */
	bool FlowChannelShape::IsValid() const {
		// 局所的な開削尺度の検査結果の返戻
		return std::isfinite(RadiusMeters) && !(RadiusMeters < 1000) && RadiusMeters <= 1E9 && std::isfinite(HalfWidthMeters) &&
		HalfWidthMeters > 0 && HalfWidthMeters < RadiusMeters * 0.05 && std::isfinite(DepthMeters) && DepthMeters > 0 &&
		DepthMeters < HalfWidthMeters && std::isfinite(HalfWidthDerivativePerMeter) && std::isfinite(DepthDerivativePerMeter) &&
		std::isfinite(CutWeight) && !(CutWeight < 0) && !(CutWeight > 1) && std::isfinite(CutWeightDerivativePerMeter);
	}

	/**
	 * 同じ経路の尺度で，重ならない有限幅の淵を準備する
	 * @param Base 端部と淵以外で使う一定の寸法
	 * @param LengthMeters 経路の公称長さ
	 * @param Pools 始点順の淵，経路内で支持区間が重ならない事
	 * @return 再評価時に乱数・割当を使わない寸法場，不正入力は空
	 */
	std::optional<FlowChannelProfile> FlowChannelProfile::Create(
		const FlowChannelShape &Base,
		const double LengthMeters,
		const std::span<const FlowChannelPool> Pools
	) {
		const bool HasValidBase =
		Base.IsValid() && !Base.HalfWidthDerivativePerMeter && !Base.DepthDerivativePerMeter && !Base.CutWeightDerivativePerMeter;
		const bool HasValidLength = std::isfinite(LengthMeters) && LengthMeters > 0;
		// 有限な基準寸法と経路が無い場合の返戻
		if(!HasValidBase || !HasValidLength || Pools.size() > 4096) return {};
		FlowChannelProfile Result;
		Result.Base = Base;
		Result.LengthMeters = LengthMeters;
		Result.MaximumHalfWidthMeters = Base.HalfWidthMeters;
		double PreviousEnd = 0;
		for(const FlowChannelPool &Pool : Pools) {
			const bool HasFiniteExtent =
			std::isfinite(Pool.CenterDistanceMeters) && std::isfinite(Pool.HalfLengthMeters) && Pool.HalfLengthMeters > 0 &&
			Pool.CenterDistanceMeters - Pool.HalfLengthMeters >= PreviousEnd &&
			Pool.CenterDistanceMeters - Pool.HalfLengthMeters < Pool.CenterDistanceMeters &&
			Pool.CenterDistanceMeters + Pool.HalfLengthMeters > Pool.CenterDistanceMeters &&
			Pool.CenterDistanceMeters + Pool.HalfLengthMeters <= LengthMeters;
			const bool HasPositiveAddition =
			std::isfinite(Pool.AddedHalfWidthMeters) && std::isfinite(Pool.AddedDepthMeters) && !(Pool.AddedHalfWidthMeters < 0) &&
			Pool.AddedDepthMeters > 0;
			// 重複・逆順・経路外の淵を黙って削らない返戻
			if(!HasFiniteExtent || !HasPositiveAddition) return {};
			FlowChannelShape Peak = Base;
			Peak.HalfWidthMeters += Pool.AddedHalfWidthMeters;
			Peak.DepthMeters += Pool.AddedDepthMeters;
			Peak.HalfWidthDerivativePerMeter = 6 * Pool.AddedHalfWidthMeters / Pool.HalfLengthMeters;
			Peak.DepthDerivativePerMeter = 6 * Pool.AddedDepthMeters / Pool.HalfLengthMeters;
			// 最大寸法と微分の表現範囲が不正な場合の返戻
			if(!Peak.IsValid()) return {};
			Result.MaximumHalfWidthMeters = std::max(Result.MaximumHalfWidthMeters, Peak.HalfWidthMeters);
			PreviousEnd = Pool.CenterDistanceMeters + Pool.HalfLengthMeters;
		}
		Result.Pools.assign(Pools.begin(), Pools.end());
		// 一度だけ準備した不変な縦断寸法の返戻
		return Result;
	}

	/**
	 * 淵の内部だけで幅と深さを変え，端で二次微分まで元断面へ繋ぐ
	 * @param DistanceMeters 同じ経路の公称距離
	 * @return 寸法と距離微分，無効な距離は空
	 */
	std::optional<FlowChannelShape> FlowChannelProfile::Sample(const double DistanceMeters) const {
		// 公称範囲外を端点へ丸めない返戻
		if(!std::isfinite(DistanceMeters) || DistanceMeters < 0 || DistanceMeters > LengthMeters) return {};
		const std::vector<FlowChannelPool>::const_iterator After = std::upper_bound(
			Pools.begin(),
			Pools.end(),
			DistanceMeters,
			[](const double Distance, const FlowChannelPool &Pool) {
				// 支持区間の始点との比較結果の返戻
				return Distance < Pool.CenterDistanceMeters - Pool.HalfLengthMeters;
			}
		);
		// 最初の淵より前では基準寸法を返す
		if(After == Pools.begin()) return Base;
		const FlowChannelPool &Pool = *(After - 1);
		const double T = (DistanceMeters - Pool.CenterDistanceMeters) / Pool.HalfLengthMeters;
		// 淵の支持区間外では基準寸法を返す
		if(!(std::abs(T) < 1)) return Base;
		const double Inner = 1 - T * T, Weight = Inner * Inner * Inner, Derivative = -6 * T * Inner * Inner / Pool.HalfLengthMeters;
		FlowChannelShape Result = Base;
		Result.HalfWidthMeters += Pool.AddedHalfWidthMeters * Weight;
		Result.DepthMeters += Pool.AddedDepthMeters * Weight;
		Result.HalfWidthDerivativePerMeter = Pool.AddedHalfWidthMeters * Derivative;
		Result.DepthDerivativePerMeter = Pool.AddedDepthMeters * Derivative;
		// 同じ重みと其の解析微分で変化する寸法の返戻
		return Result;
	}

	/**
	 * 経路索引の照会に使う支持幅の上限を取得する
	 * @return 基準断面と全ての淵を覆う半幅
	 */
	double FlowChannelProfile::GetMaximumHalfWidthMeters() const {
		// 幅が広がる地点も候補から落とさない上限の返戻
		return MaximumHalfWidthMeters;
	}

	/**
	 * 河道の両端を，値・二次微分まで元地形へ繋ぐ
	 * @param Shape 寸法と其の距離微分
	 * @param DistanceMeters 経路の始点からの公称距離
	 * @param LengthMeters 経路の公称長さ
	 * @param FadeLengthMeters 各端で使う正の遷移長，経路長の半分以下
	 * @return 端の開削強度と微分を含む寸法，不正入力は空
	 */
	std::optional<FlowChannelShape> FadeFlowChannelEnds(
		const FlowChannelShape &Shape,
		const double DistanceMeters,
		const double LengthMeters,
		const double FadeLengthMeters
	) {
		const bool HasValidDistance =
		std::isfinite(DistanceMeters) && std::isfinite(LengthMeters) && LengthMeters > 0 && !(DistanceMeters < 0) &&
		DistanceMeters <= LengthMeters;
		const bool HasValidFade = std::isfinite(FadeLengthMeters) && FadeLengthMeters > 0 && FadeLengthMeters <= LengthMeters * 0.5;
		// 不正な距離又は強度を端へ丸めない返戻
		if(!Shape.IsValid() || !HasValidDistance || !HasValidFade) return {};
		const double Remaining = LengthMeters - DistanceMeters, Distance = std::min(DistanceMeters, Remaining);
		// 両端の支持区間外では元の寸法を保持した返戻
		if(Distance >= FadeLengthMeters) return Shape;
		const double T = Distance / FadeLengthMeters, Gate = UnitTransition(T), Sign = DistanceMeters < Remaining ? 1.0 : -1.0;
		const double Derivative = Sign * 30 * T * T * (1 - T) * (1 - T) / FadeLengthMeters;
		FlowChannelShape Result = Shape;
		Result.CutWeight = Shape.CutWeight * Gate;
		Result.CutWeightDerivativePerMeter = Shape.CutWeightDerivativePerMeter * Gate + Shape.CutWeight * Derivative;
		// 演算の表現範囲を超えた結果を公開しない返戻
		if(!Result.IsValid()) return {};
		// 両端で零に成る強度と解析微分の返戻
		return Result;
	}

	/**
	 * 上流の湖から徐々に余裕高を取り，下流の一定水位へ滑らかに接続する
	 * @param Source 中心線上の元標高と，下流向きの公称距離微分
	 * @param DistanceMeters 上流接続点からの公称距離
	 * @param Settings 余裕高・遷移長・下流水位
	 * @return 非増加な水位と微分；不正入力又は上昇する元縦断は空
	 */
	std::optional<FlowChannelWaterLevel> SampleFlowWaterLevel(
		const FlowChannelWaterLevel Source,
		const double DistanceMeters,
		const FlowChannelWaterSettings &Settings
	) {
		const bool HasDescendingSource =
		std::isfinite(Source.ElevationMeters) && std::isfinite(Source.DerivativePerMeter) && !(Source.DerivativePerMeter > 0);
		const bool HasValidDistances =
		std::isfinite(DistanceMeters) && !(DistanceMeters < 0) && std::isfinite(Settings.FreeboardMeters) &&
		!(Settings.FreeboardMeters < 0) && std::isfinite(Settings.RampLengthMeters) && Settings.RampLengthMeters > 0;
		const bool HasValidShore =
		std::isfinite(Settings.DownstreamWaterMeters) && std::isfinite(Settings.ShoreBlendMeters) && Settings.ShoreBlendMeters > 0;
		// 下降縦断と正の遷移尺度を満たさない返戻
		if(!HasDescendingSource || !HasValidDistances || !HasValidShore) return {};
		const double T = std::min(1.0, DistanceMeters / Settings.RampLengthMeters), Gate = UnitTransition(T);
		const double GateDerivative = 30 * T * T * (1 - T) * (1 - T) / Settings.RampLengthMeters;
		const double Raw = Source.ElevationMeters - Settings.FreeboardMeters * Gate;
		const double Derivative = Source.DerivativePerMeter - Settings.FreeboardMeters * GateDerivative;
		// 有限入力でも演算の表現範囲を超えた場合の返戻
		if(!std::isfinite(Raw) || !std::isfinite(Derivative)) return {};
		const double Height = Raw - Settings.DownstreamWaterMeters;
		// 下流の水域内では同じ一定水位を返す
		if(Height <= 0) return FlowChannelWaterLevel{ Settings.DownstreamWaterMeters, 0 };
		// 遷移域より上流では余裕高を引いた縦断を返す
		if(Height >= Settings.ShoreBlendMeters) return FlowChannelWaterLevel{ Raw, Derivative };
		const double Q = Height / Settings.ShoreBlendMeters, Blend = Q * Q * Q * (6 + Q * (-8 + 3 * Q));
		const double BlendDerivative = Q * Q * (18 + Q * (-32 + 15 * Q));
		// 正の導関数を持つ接続式で，下流向きの符号を保った返戻
		return FlowChannelWaterLevel{
			Settings.DownstreamWaterMeters + Settings.ShoreBlendMeters * Blend,
			Derivative * BlendDerivative
		};
	}

	/**
	 * 湖の一定水面を，鞍点から下流の下降縦断へ二次微分まで接続する
	 * @param Source 経路上の元標高と公称距離微分
	 * @param SignedDistanceMeters 鞍点を零とする距離，上流は負
	 * @param UpstreamWaterMeters 鞍点の元標高と同じ上流水位
	 * @param Settings 余裕高・遷移長・下流水位
	 * @return 非増加な候補水位，源場又は上下流の整合が不正なら空
	 */
	std::optional<FlowChannelWaterLevel> SampleFlowOutletWaterLevel(
		const FlowChannelWaterLevel Source,
		const double SignedDistanceMeters,
		const double UpstreamWaterMeters,
		const FlowChannelWaterSettings &Settings
	) {
		const bool HasFiniteSource = std::isfinite(Source.ElevationMeters) && std::isfinite(Source.DerivativePerMeter);
		const bool HasValidLevels = std::isfinite(UpstreamWaterMeters) && std::isfinite(Settings.DownstreamWaterMeters) &&
		Settings.DownstreamWaterMeters <= UpstreamWaterMeters;
		const bool HasValidSettings =
		std::isfinite(SignedDistanceMeters) && std::isfinite(Settings.FreeboardMeters) && !(Settings.FreeboardMeters < 0) &&
		std::isfinite(Settings.RampLengthMeters) && Settings.RampLengthMeters > 0 && std::isfinite(Settings.ShoreBlendMeters) &&
		Settings.ShoreBlendMeters > 0;
		// 一定水面を返す分岐でも無効な設定を受理しない返戻
		if(!HasFiniteSource || !HasValidLevels || !HasValidSettings) return {};
		// 湖側では元地形の上昇に依らず一定水位を返す
		if(!(SignedDistanceMeters > 0)) return FlowChannelWaterLevel{ UpstreamWaterMeters, 0 };
		// 鞍点より高い又は下流へ上昇する源場を隠さない返戻
		if(Source.ElevationMeters > UpstreamWaterMeters || Source.DerivativePerMeter > 0) return {};
		const double T = std::min(1.0, SignedDistanceMeters / Settings.RampLengthMeters), Gate = UnitTransition(T);
		const double GateDerivative = 30 * T * T * (1 - T) * (1 - T) / Settings.RampLengthMeters;
		const double Offset = Source.ElevationMeters - UpstreamWaterMeters - Settings.FreeboardMeters;
		const FlowChannelWaterLevel Joined {
			UpstreamWaterMeters + Gate * Offset,
			Gate * Source.DerivativePerMeter + GateDerivative * Offset
		};
		FlowChannelWaterSettings Downstream = Settings;
		Downstream.FreeboardMeters = 0;
		const double Head = UpstreamWaterMeters - Settings.DownstreamWaterMeters;
		// 同水位の許可された静水面では，正の補間幅を捏造しない返戻
		if(!Head) return FlowChannelWaterLevel{ Settings.DownstreamWaterMeters, 0 };
		// 下流への補間が峠の水位へ及ばない様，実際の上下流水位差以内に収める
		Downstream.ShoreBlendMeters = std::min(Downstream.ShoreBlendMeters, Head);
		// 既存の単調な接続式で下流静水面へ繋いだ返戻
		return SampleFlowWaterLevel(Joined, 0, Downstream);
	}

	/**
	 * 元の谷筋へ有限幅の浅い断面を掘り，水位照会を伴わずに地形と解析勾配を返す
	 * 外縁では補正の値・一次・二次微分が零に接続する
	 * @param Footpoint 同じ照会点に対する区間内の直交点
	 * @param Here 照会点での元地形
	 * @param Shape 物理単位の幅・深さ
	 * @return 補正後の地形と被覆；水位は未要求，不正入力は空
	 */
	std::optional<FlowChannelSection> SampleFlowChannelTerrain(
		const TerrainFlowFootpoint &Footpoint,
		const TerrainSample &Here,
		const FlowChannelShape &Shape
	) {
		const bool HasFiniteSamples = std::isfinite(Here.ElevationMeters) && std::isfinite(Here.TangentialGradientPerRadian.Length());
		const bool HasFiniteFootpoint = std::isfinite(Footpoint.SquaredChordDistance) && !(Footpoint.SquaredChordDistance < 0) &&
		std::isfinite(Footpoint.SquaredChordGradientPerRadian.Length()) &&
		std::isfinite(Footpoint.ParameterGradientMetersPerRadian.Length());
		// 不正な尺度・元場・投影結果を通常の地形として返さない返戻
		if(!Shape.IsValid() || !HasFiniteSamples || !HasFiniteFootpoint) return {};
		const double Ratio = Shape.RadiusMeters / Shape.HalfWidthMeters, Scale = Ratio * Ratio;
		// 表現範囲を超える幅を静かに零の地形へ変えない返戻
		if(!std::isfinite(Scale)) return {};
		const double Q = Footpoint.SquaredChordDistance * Scale;
		// 支持領域の外では寸法の変化に依らず元場を正確に保つ返戻
		if(Q >= 1) return FlowChannelSection{ Here, {}, 0, false };
		const double Remaining = 1 - Q, Coverage = Remaining * Remaining * Remaining, Depth = Shape.DepthMeters * Shape.CutWeight;
		const double DepthDerivative =
		Shape.DepthDerivativePerMeter * Shape.CutWeight + Shape.DepthMeters * Shape.CutWeightDerivativePerMeter;
		const double AlongCorrection =
		-6 * Depth * Remaining * Remaining * Q * Shape.HalfWidthDerivativePerMeter / Shape.HalfWidthMeters - DepthDerivative * Coverage;
		const Vec3 CorrectionGradient = Footpoint.SquaredChordGradientPerRadian * (3 * Depth * Remaining * Remaining * Scale) +
		Footpoint.ParameterGradientMetersPerRadian * AlongCorrection;
		const TerrainSample Terrain { Here.ElevationMeters - Depth * Coverage, Here.TangentialGradientPerRadian + CorrectionGradient };
		const bool HasFiniteTerrain =
		std::isfinite(Terrain.ElevationMeters) && std::isfinite(Terrain.TangentialGradientPerRadian.Length());
		// 有限な入力からの演算でも表現範囲を超えた場合は公開しない返戻
		if(!HasFiniteTerrain) return {};
		// 河床中心の深さを保ち，支持領域外では元地形へ戻る断面の返戻
		return FlowChannelSection{ Terrain, {}, Coverage * Shape.CutWeight, false };
	}

	/**
	 * 同じ河床の評価へ，指定された縦断水位と解析勾配を添える
	 * @param Footpoint 同じ照会点に対する経路の直交点
	 * @param Here 照会点の元地形
	 * @param WaterLevel 直交点の水位と距離微分
	 * @param Shape 物理単位の幅・深さ
	 * @return 地形と水位の共通断面；無効な入力・演算範囲超過は空
	 */
	std::optional<FlowChannelSection> SampleFlowChannel(
		const TerrainFlowFootpoint &Footpoint,
		const TerrainSample &Here,
		const FlowChannelWaterLevel WaterLevel,
		const FlowChannelShape &Shape
	) {
		// 要求された水位が不正な場合は，地形だけの成功へ置換えない返戻
		if(!std::isfinite(WaterLevel.ElevationMeters) || !std::isfinite(WaterLevel.DerivativePerMeter)) return {};
		std::optional<FlowChannelSection> Section = SampleFlowChannelTerrain(Footpoint, Here, Shape);
		// 同じ河床が不正な場合の返戻
		if(!Section) return {};
		const TerrainSample Water {
			WaterLevel.ElevationMeters,
			Footpoint.ParameterGradientMetersPerRadian * WaterLevel.DerivativePerMeter
		};
		// 水位勾配の演算範囲を超えた場合の返戻
		if(!std::isfinite(Water.TangentialGradientPerRadian.Length())) return {};
		Section->Water = Water;
		Section->HasWater = true;
		// 同じ河床へ正規の水位だけを加えた返戻
		return Section;
	}
}
