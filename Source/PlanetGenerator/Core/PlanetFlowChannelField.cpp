#include "PlanetFlowChannelField.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace PlanetCore {
	/**
	 * 同じ縦断寸法を構築した物理半径を取得する
	 * @return 入力検査済の半径
	 */
	double FlowChannelField::GetRadiusMeters() const {
		return Definition.Shape.RadiusMeters;
	}

	/**
	 * 必須の不変資源と，同じ水位・尺度・経路範囲を確認して場を構築する
	 * @param Input 同じ元場から準備した定義；元場の依存物は照会期間を通じて有効に保つ
	 * @param Stop 準備中の中断要求
	 * @param WaterDiagnostics 水位近似の誤差と拒否段階，省略可
	 * @return 完成した地点評価，欠落又は異なる幾何定義の混在は空
	 */
	std::optional<FlowChannelField> FlowChannelField::Create(
		FlowChannelFieldDefinition Input,
		const std::stop_token Stop,
		FlowWaterProfileDiagnostics *const WaterDiagnostics
	) {
		if(WaterDiagnostics) *WaterDiagnostics = {};
		const bool HasResources = Input.Source && Input.Route && Input.Lake && Input.ShapeProfile;
		// 未準備の依存を通常の乾いた場へ置換えない返戻
		if(!HasResources || !Input.Shape.IsValid() || Stop.stop_requested()) return {};
		const double Length = Input.Route->GetLengthMeters();
		const bool HasRange =
		std::isfinite(Input.PassDistanceMeters) && Input.PassDistanceMeters >= 0 && Input.PassDistanceMeters <= Length &&
		std::isfinite(Input.WaterQueryHalfWidthMeters) &&
		Input.WaterQueryHalfWidthMeters >= Input.ShapeProfile->GetMaximumHalfWidthMeters();
		const bool HasWater =
		std::isfinite(Input.UpstreamWaterMeters) && Input.UpstreamWaterMeters > Input.WaterSettings.DownstreamWaterMeters &&
		SampleFlowOutletWaterLevel({ Input.UpstreamWaterMeters, 0 }, 0, Input.UpstreamWaterMeters, Input.WaterSettings).has_value();
		// 範囲外又は下降しない水位を河川として準備しない返戻
		if(!HasRange || !HasWater) return {};
		const std::optional<FlowChannelShape> First = Input.ShapeProfile->Sample(0), Last = Input.ShapeProfile->Sample(Length);
		const TerrainContour &Lake = Input.Lake->GetContour();
		const bool SameDefinition =
		First && Last && First->RadiusMeters == Input.Shape.RadiusMeters && Last->RadiusMeters == Input.Shape.RadiusMeters &&
		!Lake.Samples.empty() && Lake.Samples.front().ElevationMeters == Input.UpstreamWaterMeters;
		// 長さ・尺度又は上流湖の水位が異なる資源への返戻
		if(!SameDefinition) return {};
		if(Input.ReceivingLake) {
			const TerrainContour &Receiving = Input.ReceivingLake->GetContour();
			if(Receiving.Samples.empty() || Receiving.Samples.front().ElevationMeters != Input.WaterSettings.DownstreamWaterMeters) {
				// 異なる受入水位を拒否する返戻
				return {};
			}
		}
		// 曲線の接続点と湖面の遷移点を，視点に依存しない近似の初期節点に含める
		std::vector<double> Distances { 0 };
		double Distance = 0;
		for(const TerrainFlowSegment &Segment : Input.Route->GetSegments()) {
			Distance += Segment.GetParameterLengthMeters();
			Distances.push_back(Distance);
		}
		Distances.push_back(Input.PassDistanceMeters);
		const double RampEnd = Input.PassDistanceMeters + Input.WaterSettings.RampLengthMeters;
		if(RampEnd < Length) Distances.push_back(RampEnd);
		std::sort(Distances.begin(), Distances.end());
		Distances.erase(std::unique(Distances.begin(), Distances.end()), Distances.end());
		const auto OriginalWater = [&](const double Along) -> std::optional<FlowChannelWaterLevel> {
			const std::optional<TerrainFlowCurveSample> Point = Input.Route->Sample(Along);
			// 曲線を失った縦断を補間で隠さない返戻
			if(!Point) return {};
			const TerrainSample Value = Input.Source(Point->Direction);
			const FlowChannelWaterLevel Original {
				Value.ElevationMeters,
				Value.TangentialGradientPerRadian.Dot(Point->DirectionDerivativePerMeter)
			};
			// 既存の湖面接続と下降検査を通した元水位の返戻
			return SampleFlowOutletWaterLevel(Original, Along - Input.PassDistanceMeters, Input.UpstreamWaterMeters, Input.WaterSettings);
		};
		std::optional<FlowWaterProfile> PreparedWater = FlowWaterProfile::Create(OriginalWater, Distances, {}, Stop, WaterDiagnostics);
		// 精度又は下降条件を満たさない水位を，未検証の直接評価へ差替えない返戻
		if(!PreparedWater) return {};
		FlowChannelField Field;
		Field.WaterProfile = std::move(PreparedWater);
		Field.Definition = std::move(Input);
		// 元の組立変数から独立した不変場の返戻
		return Field;
	}

	/**
	 * 細い水域を頂点標本だけで見落さない為の，不変な経路と岸を取得する
	 * @return 所有する場の寿命内で有効な読取専用の幾何
	 */
	FlowWaterGeometry FlowChannelField::GetWaterGeometry() const {
		// 地点評価と同じ定義だけを参照する返戻
		return {
			*Definition.Route,
			*Definition.Lake,
			Definition.ReceivingLake.get(),
			Definition.Shape.HalfWidthMeters,
			Definition.ShapeProfile->GetMaximumHalfWidthMeters()
		};
	}

	/**
	 * 単独河道の幾何を，共通の描画準備へ渡す
	 * @return この場が所有する一つの河道と両湖岸
	 */
	std::vector<FlowWaterGeometry> FlowChannelField::GetWaterGeometries() const {
		return { GetWaterGeometry() };
	}

	/**
	 * 同じ元場の標高を，利用可能なら微分無しで照会する
	 * @param Direction 球面方向
	 * @return 元の標高
	 */
	double FlowChannelField::EvaluateSourceHeight(const UnitDirection Direction) const {
		// 省略した標高専用の照会も，正規の全微分評価へ接続する
		return Definition.SourceHeight ? Definition.SourceHeight(Direction) : Definition.Source(Direction).ElevationMeters;
	}

	/**
	 * 同じ元場の一点評価を共有して，河道の補正と水位を計算する
	 * @param Direction 元場と同じ照会方向
	 * @param Here 同じ所有元から取得した標高と，指定時の解析勾配
	 * @param NeedsWater 水位を計算する指定
	 * @param HasSourceGradient Here の勾配を水位の接続にも使える場合に真
	 * @return 通常の地点評価と同じ断面；不正な縦断又は寸法では空
	 */
	std::optional<FlowChannelSection> FlowChannelField::EvaluateOnSource(
		const UnitDirection Direction,
		const TerrainSample Here,
		const bool NeedsWater,
		const bool HasSourceGradient
	) const {
		const double QueryWidth =
		NeedsWater ? Definition.WaterQueryHalfWidthMeters : Definition.ShapeProfile->GetMaximumHalfWidthMeters() + 0.02;
		const std::optional<FlowRouteProjection> Projection = Definition.Route->FindFootpoint(Direction, QueryWidth, nullptr, true);
		// 管状支持領域の外では元地形を保持した返戻
		if(!Projection) {
			if(NeedsWater && Definition.Lake->Contains(Direction)) {
				// 上流湖の一定面の返戻
				return FlowChannelSection{ Here, { Definition.UpstreamWaterMeters, {} }, 0, true };
			}
			if(NeedsWater && Definition.ReceivingLake) {
				// 受入側の一定面の返戻
				return FlowChannelSection{ Here, { Definition.WaterSettings.DownstreamWaterMeters, {} }, 0, true };
			}
			// 河道補正の外では元地形を返す
			return FlowChannelSection{ Here, {}, 0 };
		}
		const TerrainFlowFootpoint &Foot = Projection->Footpoint;
		const std::optional<FlowChannelShape> LocalShape = Definition.ShapeProfile->Sample(Projection->DistanceMeters);
		// 経路と縦断寸法が対応しない場合の返戻
		if(!LocalShape) return {};
		const std::optional<FlowChannelShape> Faded = FadeFlowChannelEnds(
			*LocalShape,
			Projection->DistanceMeters,
			Definition.Route->GetLengthMeters(),
			std::min(25.0, Definition.Route->GetLengthMeters() * 0.25)
		);
		// 経路端の開削強度を構成出来ない場合の返戻
		if(!Faded) return {};
		// 河床の式は水位に依存しない為，地面だけの照会で中心の元場を再評価しない返戻
		if(!NeedsWater) return SampleFlowChannelTerrain(Foot, Here, *Faded);
		// 生成時の不変な縦断から照会し，中心の元地形を移動毎に再評価しない
		const std::optional<FlowChannelWaterLevel> Water = WaterProfile->Sample(Projection->DistanceMeters);
		// 不正な水位を通常の河床へ読み替えない返戻
		if(!Water) return {};
		// 表示と照会が共有する断面の返戻
		std::optional<FlowChannelSection> Section = SampleFlowChannel(Foot, Here, *Water, *Faded);
		if(Section && NeedsWater && Definition.ReceivingLake && !Definition.Lake->Contains(Direction)) {
			// 湖岸での横方向の水位段差を消し，両端の値と微分を保つ
			const double Excess = Section->Water.ElevationMeters - Definition.WaterSettings.DownstreamWaterMeters;
			const double Height = Here.ElevationMeters - Definition.WaterSettings.DownstreamWaterMeters;
			if(Excess > 0 && Height < Excess) {
				if(Height <= 0) Section->Water = { Definition.WaterSettings.DownstreamWaterMeters, {} };
				else {
					const double Fraction = Height / Excess, Blend = Fraction * Fraction * Fraction * (10 + Fraction * (-15 + 6 * Fraction));
					const double Derivative = 30 * Fraction * Fraction * (1 - Fraction) * (1 - Fraction);
					const Vec3 Gradient =
					HasSourceGradient ? Here.TangentialGradientPerRadian : Definition.Source(Direction).TangentialGradientPerRadian;
					Section->Water.ElevationMeters = Definition.WaterSettings.DownstreamWaterMeters + Excess * Blend;
					Section->Water.TangentialGradientPerRadian =
					Section->Water.TangentialGradientPerRadian * (Blend - Fraction * Derivative) + Gradient * Derivative;
				}
				Section->HasWater = true;
			}
		}
		// 受入湖へ滑らかにつながる水面と，元の河道補正を返す
		return Section;
	}

	/**
	 * 同じ元標高を共有して，河道と両湖岸の支持域を求める
	 * @param Direction 元場と同じ照会方向
	 * @param ElevationMeters 同じ所有元の標高；受入湖無しの場合は未使用
	 * @return 通常の所属照会と同じ支持値，不正な縦断では空
	 */
	std::optional<double> FlowChannelField::WaterSupportOnHeight(
		const UnitDirection Direction,
		const double ElevationMeters
	) const {
		const double SourceMargin = Definition.Lake->SignedPlanarDistanceMeters(Direction);
		const double Limit = Definition.ShapeProfile->GetMaximumHalfWidthMeters();
		double LakeMargin = SourceMargin;
		if(Definition.ReceivingLake) {
			const double Depth = Definition.WaterSettings.DownstreamWaterMeters - ElevationMeters;
			// 受入水深が既存支持値以下なら，最小値との合成結果を変えない
			if(Depth > std::max(SourceMargin, -Limit)) {
				LakeMargin = std::max(SourceMargin, std::min(Definition.ReceivingLake->SignedPlanarDistanceMeters(Direction), Depth));
			}
		}
		// 河道の支持値は最大半幅以下なので，深い湖内では直交点を求めても最大値を変えない返戻
		if(LakeMargin >= Limit) return LakeMargin;
		const double QueryWidth = Limit * 2;
		const std::optional<FlowRouteProjection> Projection = Definition.Route->FindFootpoint(Direction, QueryWidth, nullptr, true);
		// 負側だけを有限距離で飽和させ，索引の外でも同じ零等値線と連続性を保つ
		if(!Projection) return std::max(LakeMargin, -Limit);
		const std::optional<FlowChannelShape> LocalShape = Definition.ShapeProfile->Sample(Projection->DistanceMeters);
		// 不整合な縦断を乾いた領域で置換えない返戻
		if(!LocalShape) return {};
		const TerrainFlowFootpoint &Foot = Projection->Footpoint;
		const double Across = LocalShape->HalfWidthMeters - Definition.Shape.RadiusMeters * std::sqrt(Foot.SquaredChordDistance);
		double Along = std::min(Projection->DistanceMeters, Definition.Route->GetLengthMeters() - Projection->DistanceMeters);
		if(!Projection->DistanceMeters || Projection->DistanceMeters == Definition.Route->GetLengthMeters()) {
			const Vec3 Tangent = UnitDirection::FromVector(Foot.Curve.DirectionDerivativePerMeter)->Vector();
			const double Offset = Definition.Shape.RadiusMeters * (Direction.Vector() - Foot.Curve.Direction.Vector()).Dot(Tangent);
			Along = Projection->DistanceMeters ? -Offset : Offset;
		}
		const double RiverMargin = std::max(-Limit, std::min(Across, Along));
		// 湖と開削支持域の和集合の返戻
		return std::max(LakeMargin, RiverMargin);
	}

	/**
	 * 元の湖岸と有限幅の河道を合わせ，視点に依存しない水面の所属を返す
	 * @param Direction 同じ惑星の照会点
	 * @return 内側が正の支持距離；不整合な縦断寸法は空
	 */
	std::optional<double> FlowChannelField::WaterSupport(const UnitDirection Direction) const {
		const double Height = Definition.ReceivingLake ? EvaluateSourceHeight(Direction) : 0;
		// 単独河道の所属も同じ評価本体へ渡す返戻
		return WaterSupportOnHeight(Direction, Height);
	}

	/**
	 * 元地形と経路索引を使い，同じ式から河床と水位を照会する
	 * @param Direction 球面の照会方向
	 * @param NeedsWater 湖側の広い水位候補も照会する場合に真
	 * @param NeedsTerrainGradient 地面の勾配を使う場合に真；偽では地面の標高だけが有効
	 * @return 局所断面又は元地形；縦断・断面の不整合は空
	 */
	std::optional<FlowChannelSection> FlowChannelField::Evaluate(
		const UnitDirection Direction,
		const bool NeedsWater,
		const bool NeedsTerrainGradient
	) const {
		// 水位の勾配は常に保持し，要求された地面の微分だけを省略する
		const TerrainSample Here =
		NeedsTerrainGradient ? Definition.Source(Direction) : TerrainSample{ EvaluateSourceHeight(Direction), {} };
		// 単独河道も同じ評価本体を通す返戻
		return EvaluateOnSource(Direction, Here, NeedsWater, NeedsTerrainGradient);
	}

	/**
	 * 同じ元標高を河床・水位・所属へ共有し，重複する標高照会を省く
	 * @param Direction 元場と同じ球面方向
	 * @param NeedsTerrainGradient 地面の接線勾配を使う場合に真；水位の勾配は常に有効
	 * @return 同じ断面と支持距離；不整合な元場・水位・支持域は空
	 */
	std::optional<FlowWaterSample> FlowChannelField::EvaluateWithSupport(
		const UnitDirection Direction,
		const bool NeedsTerrainGradient
	) const {
		const TerrainSample Here =
		NeedsTerrainGradient ? Definition.Source(Direction) : TerrainSample{ EvaluateSourceHeight(Direction), {} };
		// 不正な標高を所属の外側へ読み替えない返戻
		if(!std::isfinite(Here.ElevationMeters)) return {};
		const std::optional<FlowChannelSection> Section = EvaluateOnSource(Direction, Here, true, NeedsTerrainGradient);
		// 断面が欠落した水域を受理しない返戻
		if(!Section) return {};
		const std::optional<double> Support = WaterSupportOnHeight(Direction, Here.ElevationMeters);
		// 未定義又は非有限な所属を有効値として渡さない返戻
		if(!Support || !std::isfinite(*Support)) return {};
		// 共通の元標高から評価した組の返戻
		return FlowWaterSample{ *Section, *Support };
	}
}
