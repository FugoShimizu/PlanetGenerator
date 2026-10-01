#include "PlanetFlowView.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stop_token>
#include <unordered_map>
#include <utility>

namespace PlanetCore {
	namespace {
		struct FlowVertexSample {
			// 同じ不変場から得る，身体の局所座標に依存しない描画属性
			double Height = 0;
			Vec3 Normal;
			SurfaceMaterialWeights Material;
			std::array<double, 2> WaterCoordinates {};
		};

		/**
		 * 河床・解析法線・湿潤材質を，同じ元場の方向から求める
		 * @param Field 不変な地形と水域
		 * @param Direction 量子化していない照会方向
		 * @return 有効な頂点属性，不整合な水域又は非有限な面は空
		 */
		std::optional<FlowVertexSample> SampleFlowVertex(const FlowSurfaceField &Field, const UnitDirection Direction) {
			// 同じ元場の一回の評価から，地面・水位・所属と其の解析微分を取得
			const std::optional<FlowWaterSample> Value = Field.EvaluateWithSupport(Direction);
			// 不正な支持距離又は元場を乾いた面で置換えない返戻
			if(!Value || !std::isfinite(Value->SupportMeters)) return {};
			const FlowChannelSection &Sample = Value->Section;
			const double Height = Sample.Terrain.ElevationMeters, Radius = Field.GetRadiusMeters();
			const std::optional<UnitDirection> Normal =
			UnitDirection::FromVector(Direction.Vector() * (Radius + Height) - Sample.Terrain.TangentialGradientPerRadian);
			// 反転又は非有限な面への返戻
			if(!Normal || !std::isfinite(Height) || !(Radius + Height > 0)) return {};
			const double Depth = Sample.HasWater ? Sample.Water.ElevationMeters - Height : -1;
			// 非有限な水深を頂点属性へ運ばない返戻
			if(!std::isfinite(Depth) || Value->SupportMeters > 0 && !Sample.HasWater) return {};
			RiverSurfaceSample MaterialSample;
			MaterialSample.Terrain = Sample.Terrain;
			if(Value->SupportMeters > 0) {
				MaterialSample.ChannelCoverage = std::clamp(Value->SupportMeters * 10, 0.0, 1.0);
				MaterialSample.WaterElevationMeters = Sample.Water.ElevationMeters;
				MaterialSample.WaterSupport = 1;
			}
			// 比較用の従来被覆と，画素で被覆を決める未飽和の座標を共に保持した返戻
			return FlowVertexSample{
				Height,
				Normal->Vector(),
				EvaluateSurfaceMaterial(Direction, Radius, 0, MaterialSample),
				{ Depth, Value->SupportMeters }
			};
		}
	}

	/**
	 * 同じ不変場を捕捉し，半径に対応する標高の再利用を準備する
	 * @param Input 有効な河床場
	 */
	FlowTerrainView::FlowTerrainView(std::shared_ptr<const FlowSurfaceField> Input) : Field(std::move(Input)) {
		Cache = std::make_unique<LodSurfaceCache>(
			Field->GetRadiusMeters(),
			[Owned = Field](const UnitDirection Direction) -> double {
				const std::optional<FlowChannelSection> Sample = Owned->Evaluate(Direction, false, false);
				// 欠落を元地形又は零標高で代用しない返戻
				return Sample ? Sample->Terrain.ElevationMeters : std::numeric_limits<double>::quiet_NaN();
			}
		);
		// 終了
		return;
	}

	/**
	 * 同じ生成定義だけに束縛された描画準備を作る
	 * @param Field 有効期間を共有する不変な河床
	 * @return 一件のワーカーで使う準備状態；元場が無い場合は空
	 */
	std::unique_ptr<FlowTerrainView> FlowTerrainView::Create(std::shared_ptr<const FlowSurfaceField> Field) {
		// 未準備の元場を拒否する返戻
		if(!Field) return {};
		// 私有コンストラクターで束縛した状態の返戻
		return std::unique_ptr<FlowTerrainView>(new FlowTerrainView(std::move(Field)));
	}

	/**
	 * 再利用が同じ元場に対する物か確認する
	 * @param Candidate 使用予定の場
	 * @return 所有する不変場が同じ場合に真
	 */
	bool FlowTerrainView::Uses(const std::shared_ptr<const FlowSurfaceField> &Candidate) const {
		// 同じ不変場への束縛を返す
		return Field == Candidate;
	}

	/**
	 * 同じ不変場を指定された完全分割へ載せ，実際の近景の精度を検査する
	 * @param Field 元の河床と所属を所有する定義
	 * @param View 描画視点と身体の精細範囲
	 * @param Sphere 完全な分割の頂点と接合三角形
	 * @param Tree 接合の所有葉を持つ同じ分割木
	 * @param Stop 中断要求
	 * @param Report 既存の選択計測に加える出力精度と拒否理由
	 * @param Previous 同じ場で完成した前回の面，省略時は全頂点を評価
	 * @return 同じ座標・材質・法線の候補；不整合又は近景の精度未達は空
	 * @note 前回頂点数 P，今回頂点数 V，三角形数 T に対して期待時間 O(P log(P+1)+V log(P+2)+T)，元場の照会費用を除く；追加保持量 O(P+T)
	 */
	std::optional<FlowTerrainMesh> BuildFlowTerrainPartition(
		std::shared_ptr<const FlowSurfaceField> Field,
		const LodView &View,
		AdaptiveSphereMesh Sphere,
		LodTree Tree,
		const std::stop_token Stop,
		FlowViewDiagnostics &Report,
		const FlowTerrainMesh *const Previous
	) {
		// 候補の部分公開を避ける拒否経路の準備
		const auto Reject = [&](const FlowViewFailure Failure) -> std::optional<FlowTerrainMesh> {
			Report.Failure = Stop.stop_requested() ? FlowViewFailure::Cancelled : Failure;
			// 同じ元場と精度を満たさない候補への返戻
			return {};
		};
		if(!Field || !View.IsValid() || View.RadiusMeters != Field->GetRadiusMeters() || Stop.stop_requested()) {
			// 未準備又は異なる入力への返戻
			return Reject(FlowViewFailure::InvalidView);
		}
		if(Sphere.TriangleOwners.size() != Sphere.Mesh.Triangles.size() || Sphere.Mesh.Directions.empty()) {
			// 所有葉との対応が無い分割への返戻
			return Reject(FlowViewFailure::Topology);
		}
		// 同じ分割木に属さない面への返戻
		for(const TileId Owner : Sphere.TriangleOwners) if(!Tree.IsLeaf(Owner)) return Reject(FlowViewFailure::Topology);
		for(const std::array<std::uint32_t, 3> Triangle : Sphere.Mesh.Triangles) {
			// 有効な頂点を参照しない面への返戻
			for(const std::uint32_t Vertex : Triangle) if(Vertex >= Sphere.Mesh.Directions.size()) return Reject(FlowViewFailure::Topology);
		}
		// 同じ元場で完成した前回の頂点だけを，正確な方向で索引化
		std::map<std::array<double, 3>, std::uint32_t> PreviousVertices;
		if(Previous) {
			// 別の定義の値を混ぜない返戻
			if(Previous->SourceField != Field) return Reject(FlowViewFailure::InvalidView);
			const std::size_t Count = Previous->Sphere.Mesh.Directions.size();
			const bool Complete =
			Previous->Heights.size() == Count && Previous->PlanetNormals.size() == Count && Previous->Materials.size() == Count &&
			Previous->WaterCoordinates.size() == Count;
			// 部分的な頂点属性を再利用しない返戻
			if(!Complete) return Reject(FlowViewFailure::Topology);
			for(std::uint32_t I = 0; I < Count; ++I) {
				// 索引準備中の中断への返戻
				if(Stop.stop_requested()) return Reject(FlowViewFailure::Cancelled);
				const Vec3 D = Previous->Sphere.Mesh.Directions[I].Vector();
				PreviousVertices.emplace(std::array<double, 3>{ D.X, D.Y, D.Z }, I);
			}
		}
		// 局所座標と身体周辺の精細範囲の準備
		const Vec3 Focus = View.SurfaceObserverMeters.value_or(View.ObserverMeters);
		const UnitDirection Anchor = *UnitDirection::FromVector(Focus);
		const std::optional<FlowChannelSection> AnchorSample = Field->Evaluate(Anchor, false, false);
		// 無効な原点標高への返戻
		if(!AnchorSample) return Reject(FlowViewFailure::Sample);
		const std::optional<GroundFrame> Frame = GroundFrame::Create(Anchor, View.RadiusMeters, AnchorSample->Terrain.ElevationMeters);
		// 有効な局所座標を作れない場合の返戻
		if(!Frame) return Reject(FlowViewFailure::Sample);
		const std::optional<SurfaceCorridor> Region =
		SurfaceCorridor::Create(Focus, View.SurfacePathEndMeters.value_or(Focus), View.RadiusMeters, View.SurfaceRadiusMeters);
		// 精細範囲が有る場合の不正な通路への返戻
		if(View.SurfaceRadiusMeters > 0 && !Region) return Reject(FlowViewFailure::InvalidView);
		FlowTerrainMesh Result;
		Result.SourceField = Field;
		Result.Frame = *Frame;
		Result.Sphere = std::move(Sphere);
		Result.Tree = std::move(Tree);
		Report.Leaves = Result.Tree.Leaves().size();
		Result.Heights.reserve(Result.Sphere.Mesh.Directions.size());
		Result.Positions.reserve(Result.Sphere.Mesh.Directions.size());
		Result.Normals.reserve(Result.Sphere.Mesh.Directions.size());
		Result.PlanetNormals.reserve(Result.Sphere.Mesh.Directions.size());
		Result.Materials.reserve(Result.Sphere.Mesh.Directions.size());
		Result.WaterCoordinates.reserve(Result.Sphere.Mesh.Directions.size());
		const std::optional<SurfaceTextureFrame> Texture =
		SurfaceTextureFrame::Create(Anchor, View.RadiusMeters, Frame->AnchorElevationMeters);
		// 材質も同じ物理尺度の座標で評価する返戻
		if(!Texture) return Reject(FlowViewFailure::Sample);
		Result.TextureOrigin = Texture->Phase;
		// エンジンへ渡すセンチメートルの単精度を再現し，実際の近景位置を検査
		const auto Quantize = [](const Vec3 P) -> Vec3 {
			// 表示頂点と同じ丸めを通したメートル座標の返戻
			return {
				static_cast<double>(static_cast<float>(P.X * 100)) * 0.01,
				static_cast<double>(static_cast<float>(P.Y * 100)) * 0.01,
				static_cast<double>(static_cast<float>(P.Z * 100)) * 0.01
			};
		};
		for(const UnitDirection Direction : Result.Sphere.Mesh.Directions) {
			// 頂点間の中断を伝える返戻
			if(Stop.stop_requested()) return Reject(FlowViewFailure::Cancelled);
			const Vec3 D = Direction.Vector();
			const std::map<std::array<double, 3>, std::uint32_t>::const_iterator Found = PreviousVertices.find({ D.X, D.Y, D.Z });
			FlowVertexSample Vertex;
			if(Found != PreviousVertices.end()) {
				const std::uint32_t I = Found->second;
				Vertex = { Previous->Heights[I], Previous->PlanetNormals[I], Previous->Materials[I], Previous->WaterCoordinates[I] };
				++Report.VertexSampleReuses;
			} else {
				const std::optional<FlowVertexSample> Sample = SampleFlowVertex(*Field, Direction);
				++Report.VertexSampleEvaluations;
				// 不整合な元場で頂点を補わない返戻
				if(!Sample) return Reject(FlowViewFailure::Sample);
				Vertex = *Sample;
			}
			const double Height = Vertex.Height;
			const Vec3 Position = Frame->SurfaceToLocal(Direction, Height), FloatPosition = Quantize(Position);
			// 遠景も単精度で非有限になる候補は拒否する返戻
			if(!std::isfinite(FloatPosition.Length())) return Reject(FlowViewFailure::FloatPrecision);
			if(View.SurfaceRadiusMeters > 0 && Region->Intersects(Direction.Vector(), 8)) {
				Report.MaximumFloatErrorMeters = std::max(Report.MaximumFloatErrorMeters, (Position - FloatPosition).Length());
				// 近景の頂点誤差は既存の一ミリメートル以内に保つ返戻
				if(Report.MaximumFloatErrorMeters > 0.001) return Reject(FlowViewFailure::FloatPrecision);
			}
			Result.Heights.push_back(Height);
			Result.Positions.push_back(Position);
			Result.Normals.push_back(Frame->VectorToLocal(Vertex.Normal));
			Result.PlanetNormals.push_back(Vertex.Normal);
			Result.Materials.push_back(Vertex.Material);
			Result.WaterCoordinates.push_back(Vertex.WaterCoordinates);
		}
		const double R0 = Frame->RadiusMeters + Frame->AnchorElevationMeters;
		const UnitDirection End = *UnitDirection::FromVector(View.SurfacePathEndMeters.value_or(Focus));
		const std::optional<FlowChannelSection> EndSample = Field->Evaluate(End, false, false);
		// 通路の終端を同じ元場で評価出来ない場合の返戻
		if(!EndSample) return Reject(FlowViewFailure::Sample);
		const Vec3 PathSpan = Frame->SurfaceToLocal(End, EndSample->Terrain.ElevationMeters);
		const double PathLengthSquared = PathSpan.Dot(PathSpan);
		// 同じ候補内の共有辺は，同じ単精度端点の中点を一度だけ元場へ照会する
		std::unordered_map<std::uint64_t, double> ValidationHeights;
		const auto ExpectedHeight =
		[&](const UnitDirection Direction, const std::optional<std::uint64_t> Edge) -> std::optional<double> {
			if(Edge) {
				const std::unordered_map<std::uint64_t, double>::const_iterator Found = ValidationHeights.find(*Edge);
				if(Found != ValidationHeights.end()) {
					++Report.ValidationReuses;
					// 同じ候補・端点対・元場に限った中点標高の返戻
					return Found->second;
				}
			}
			const std::optional<FlowChannelSection> Sample = Field->Evaluate(Direction, false, false);
			// 無効な標本を保持しない返戻
			if(!Sample) return {};
			++Report.ValidationEvaluations;
			if(Edge) ValidationHeights.emplace(*Edge, Sample->Terrain.ElevationMeters);
			// 中心点又は初めて現れた共有辺の標高の返戻
			return Sample->Terrain.ElevationMeters;
		};
		// 実際に出力する接合三角形を，GPU と同じ単精度頂点から独立に再検査
		if(View.SurfaceRadiusMeters > 0) for(const std::array<std::uint32_t, 3> Triangle : Result.Sphere.Mesh.Triangles) {
			// 三角形間の中断への返戻
			if(Stop.stop_requested()) return Reject(FlowViewFailure::Cancelled);
			const Vec3 A = Quantize(Result.Positions[Triangle[0]]), B = Quantize(Result.Positions[Triangle[1]]);
			const Vec3 C = Quantize(Result.Positions[Triangle[2]]), Center = (A + B + C) * (1.0 / 3);
			const double Cap = std::max({ (A - Center).Length(), (B - Center).Length(), (C - Center).Length() });
			const double Fraction = PathLengthSquared > 0 ? std::clamp(Center.Dot(PathSpan) / PathLengthSquared, 0.0, 1.0) : 0;
			// 実出力面の包囲球で除外し，粗い面の中心だけを球面へ投影しない
			if((Center - PathSpan * Fraction).Length() > Region->TubeRadiusMeters + Region->SagittaMeters + Cap + 8) continue;
			const std::array<Vec3, 4> Probes { (A + B) * 0.5, (B + C) * 0.5, (C + A) * 0.5, Center };
			for(std::size_t I = 0; I < Probes.size(); ++I) {
				const Vec3 Probe = Probes[I];
				const std::optional<UnitDirection> Direction = UnitDirection::FromVector(Frame->LocalToPlanet(Probe));
				// 参照球の中心を通る不正な面への返戻
				if(!Direction) return Reject(FlowViewFailure::Topology);
				if(!Region->Contains(Direction->Vector())) continue;
				const double Increment = 2 * R0 * Probe.Z + Probe.Dot(Probe);
				const double Interpolated = Frame->AnchorElevationMeters + Increment / (std::sqrt(R0 * R0 + Increment) + R0);
				std::optional<std::uint64_t> Edge;
				if(I < 3) {
					const std::uint32_t First = Triangle[I], Second = Triangle[(I + 1) % 3];
					Edge = static_cast<std::uint64_t>(std::min(First, Second)) << 32 | std::max(First, Second);
				}
				const std::optional<double> Expected = ExpectedHeight(*Direction, Edge);
				// 誤差を定義出来ない候補の返戻
				if(!Expected) return Reject(FlowViewFailure::Sample);
				const double Error = std::abs(Interpolated - *Expected);
				++Report.ValidationSamples;
				Report.MaximumSurfaceErrorMeters = std::max(Report.MaximumSurfaceErrorMeters, Error);
				// 既存の二センチメートル条件を超える候補は公開しない返戻
				if(!std::isfinite(Error) || Error > 0.02) return Reject(FlowViewFailure::Interpolation);
			}
		}
		// 同じ不変場を持つ完全な全球面の返戻
		return Result;
	}

	/**
	 * 全球を覆う共有分割に同じ河床を載せ，近景の単精度描画を検査する
	 * @param View 観測位置・身体の精細範囲・有限な分割予算
	 * @param Stop 生成を中断する要求
	 * @param Diagnostics 拒否理由と地点評価・精度の計測先
	 * @return 完全な候補面；予算・精度・中断が未達なら空，古い公開面の所有は呼出側
	 */
	std::optional<FlowTerrainMesh> FlowTerrainView::Build(
		const LodView &View,
		const std::stop_token Stop,
		FlowViewDiagnostics *const Diagnostics
	) {
		FlowViewDiagnostics Local;
		FlowViewDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const FlowViewFailure Failure) -> std::optional<FlowTerrainMesh> {
			Report.Failure = Stop.stop_requested() ? FlowViewFailure::Cancelled : Failure;
			// 部分的な球面又は未検査の候補を公開しない返戻
			return {};
		};
		if(!View.IsValid() || View.RadiusMeters != Field->GetRadiusMeters() || Stop.stop_requested()) {
			// 異なる尺度の再利用又は中断済の要求を拒否する返戻
			return Reject(FlowViewFailure::InvalidView);
		}
		std::optional<LodSelection> Selection = SelectLodForView(View, Stop, {}, Cache.get());
		// 分割途中の中断又は不成立への返戻
		if(!Selection) return Reject(FlowViewFailure::Selection);
		Report.Leaves = Selection->Tree.Leaves().size();
		Report.HeightEvaluations = Selection->SurfaceHeightEvaluations;
		Report.ProbeCacheHits = Selection->SurfaceProbeCacheHits;
		Report.MaximumSelectionErrorMeters = std::max(Selection->MaximumSurfaceErrorMeters, Selection->MaximumStitchErrorMeters);
		// 予算内でも精度未達の面を拒否する返戻
		if(!Selection->SurfaceResolutionSatisfied || !Selection->FoliageResolutionSatisfied) return Reject(FlowViewFailure::Resolution);
		std::optional<AdaptiveSphereMesh> Sphere = BuildAdaptiveSphere(Selection->Tree);
		// 全球を覆う整合した接合が無い場合の返戻
		if(!Sphere || Stop.stop_requested()) return Reject(FlowViewFailure::Topology);
		// 地面と水面で再利用する共通の実頂点評価への返戻
		return BuildFlowTerrainPartition(Field, View, std::move(*Sphere), std::move(Selection->Tree), Stop, Report);
	}
}
