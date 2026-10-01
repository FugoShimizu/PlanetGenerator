#include "PlanetSphericalDrainage.h"
#include "PlanetDrainage.h"
#include "PlanetDrainageSampling.h"
#include "PlanetOcean.h"
#include "PlanetSphericalGradient.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 元の球面節点と追加河口を，同じ下流関係を保つ補間へ渡す
	 * @param Network 段階６の元網から得た，海岸出口付の平衡縦断
	 * @param InOcean 元の起伏を保持する海域場
	 * @param Refinement 陸上の内部細分化設定，空なら従来の網を保持する
	 * @param RefinementStats 実際の細分化量
	 * @param Stop 中断要求
	 * @param InitialMesh 排水前に標本化した共有網，空なら従来の段階６
	 * @param Mode 補間へ渡す接線の準備方法
	 * @return 節点の位置・水位・共有辺を保つ場，異なる網や不正入力では空
	 */
	std::optional<SphericalSplineField> PlanetSphericalDrainage::PrepareSurface(
		const DrainageNetwork &Network,
		const OceanConnectivityField &InOcean,
		const SphericalReliefSettings *const Refinement,
		SphericalReliefStatistics &RefinementStats,
		const std::stop_token Stop,
		const SphericalReliefSamplingMesh *const InitialMesh,
		const SphericalGradientMode Mode
	) {
		const std::optional<SphereMesh> Base = InitialMesh ? std::optional<SphereMesh>() : BuildUniformSphere(6);
		const std::span<const DrainageNode> Nodes = Network.Nodes();
		// 別の網や未準備の縦断を既存節点と誤対応させない返戻
		if(!InitialMesh && !Base || !Network.RequiresBalancedTerrain() || Stop.stop_requested()) return {};
		const std::span<const UnitDirection> Directions =
		InitialMesh ? std::span<const UnitDirection>(InitialMesh->Directions) : std::span<const UnitDirection>(Base->Directions);
		const std::span<const std::array<std::uint32_t, 3>> Triangles = InitialMesh ?
		std::span<const std::array<std::uint32_t, 3>>(InitialMesh->Triangles) :
		std::span<const std::array<std::uint32_t, 3>>(Base->Triangles);
		// 追加した沿岸出口以外の節点を暗黙に捨てない返戻
		if(Nodes.size() != Directions.size() + Network.Statistics().CoastalOutletNodes) return {};
		std::vector<UnitDirection> Outlets;
		Outlets.reserve(Network.Statistics().CoastalOutletNodes);
		for(std::size_t Index = Directions.size(); Index < Nodes.size(); ++Index) {
			const DrainageNode &Node = Nodes[Index];
			const std::optional<UnitDirection> Direction = UnitDirection::FromVector(Node.Direction);
			// 元網の外へ追加した地点が正規の沿岸出口でない場合の返戻
			if(!Node.Ocean || Node.Downstream != Index || !Direction || Stop.stop_requested()) return {};
			Outlets.push_back(*Direction);
		}
		const std::optional<DrainageSamplingMesh> Mesh = BuildDrainageSamplingMeshOnMesh(Directions, Triangles, Outlets, Stop);
		// 位置を省略した近似網で代用しない返戻
		if(!Mesh) return {};
		const double Sea = Network.Settings().SeaLevelMeters, Freeboard = Network.Settings().BankFreeboardMeters;
		std::vector<double> Heights(Mesh->Directions.size(), std::numeric_limits<double>::quiet_NaN());
		for(std::size_t Index = 0; Index < Nodes.size(); ++Index) {
			const DrainageNode &Node = Nodes[Index];
			const std::uint32_t Vertex =
			Index < Directions.size() ? static_cast<std::uint32_t>(Index) : Mesh->SampleNodes[Index - Directions.size()];
			const bool HasValidHeight = std::isfinite(Node.WaterMeters) && std::isfinite(Node.DepthMeters) && Node.DepthMeters > 0;
			// 有効な下流と有限な河床を持たない地点の返戻
			if(!HasValidHeight || Node.Downstream >= Nodes.size() || Stop.stop_requested()) return {};
			// 元の番号を異なる方向へ対応させない返戻
			if((Node.Direction - Mesh->Directions[Vertex].Vector()).Length() > 1E-12) return {};
			if(!Node.Ocean) {
				const DrainageNode &Downstream = Nodes[Node.Downstream];
				const bool HasDescendingRoute =
				Node.WaterMeters > Downstream.WaterMeters && Node.DistanceToOceanMeters > Downstream.DistanceToOceanMeters &&
				Node.WaterMeters > Sea;
				// 海へ向かう下降が成立しない節点を補間へ渡さない返戻
				if(!HasDescendingRoute) return {};
			}
			const double Head = std::max(0.0, Node.WaterMeters - Sea);
			const double Height = Node.Ocean ?
			Index < Directions.size() ? Node.OriginalElevationMeters : Node.WaterMeters - Node.DepthMeters :
			Node.WaterMeters + Freeboard * Head / (Head + Freeboard);
			// 重複した沿岸出口に矛盾する値を与えない返戻
			if(std::isfinite(Heights[Vertex]) && Heights[Vertex] != Height) return {};
			Heights[Vertex] = Height;
		}
		if(Refinement) {
			std::vector<std::uint32_t> OriginalRoutes(Mesh->Directions.size());
			for(std::uint32_t Index = 0; Index < OriginalRoutes.size(); ++Index) OriginalRoutes[Index] = Index;
			for(std::size_t Index = 0; Index < Directions.size(); ++Index) {
				if(Nodes[Index].Ocean) continue;
				const std::uint32_t Parent = Nodes[Index].Downstream;
				OriginalRoutes[Index] = Parent < Directions.size() ? Parent : Mesh->SampleNodes[Parent - Directions.size()];
			}
			const auto Source = [&InOcean](const UnitDirection Direction) -> double {
				// 海岸と同じ元地形から得た起伏の返戻
				return InOcean.EvaluateElevation(Direction);
			};
			const std::optional<PlanetSphericalRefinement> Refined = Refinement->ShouldRefineEdges ?
			PlanetSphericalRefinement::CreateWithRefinedEdges(
				Mesh->Directions,
				Mesh->Triangles,
				Heights,
				OriginalRoutes,
				Source,
				*Refinement,
				Stop
			) :
			PlanetSphericalRefinement::Create(Mesh->Directions, Mesh->Triangles, Heights, Source, *Refinement, Stop);
			// 下降辺を保持した内部細分化が成立しない場合の返戻
			if(!Refined) return {};
			RefinementStats = Refined->GetStatistics();
			if(Refinement->ShouldPreserveVertexSlopes) {
				std::vector<Vec3> Gradients;
				if(Mode == SphericalGradientMode::Sampled) {
					std::optional<SphericalGradientEstimate> Estimated =
					EstimateSphericalGradients(Refined->GetDirections(), Refined->GetTriangles(), Refined->GetElevations(), Stop);
					// 同じ調整済標高から接線を準備出来ない場合の返戻
					if(!Estimated) return {};
					Gradients = std::move(Estimated->Gradients);
				} else {
					Gradients.reserve(Refined->GetDirections().size());
					for(const UnitDirection Direction : Refined->GetDirections()) {
						// 高価な元場の勾配準備中も中断を受理する返戻
						if(Stop.stop_requested()) return {};
						Gradients.push_back(InOcean.Evaluate(Direction).TangentialGradientPerRadian);
					}
				}
				std::vector<std::uint32_t> Required;
				if(Refinement->ShouldLimitToDrainageEdges) {
					const std::span<const std::uint32_t> AddedRoutes = Refined->GetDownstreamVertices();
					Required.assign(AddedRoutes.begin(), AddedRoutes.end());
					if(!Refinement->ShouldRefineEdges) std::copy(OriginalRoutes.begin(), OriginalRoutes.end(), Required.begin());
				}
				const std::optional<std::vector<Vec3>> Limited = Refinement->ShouldLimitToDrainageEdges ?
				Refined->ConstrainDrainageGradients(Gradients, Sea, Required, Stop) :
				Refined->ConstrainGradients(Gradients, Sea, Stop);
				// 有効な制限済微分を準備出来ない場合の返戻
				if(!Limited) return {};
				for(const Vec3 Gradient : *Limited) if(Gradient.Length() > 0) ++RefinementStats.PreservedSlopeVertices;
				// 元辺の単調性を保つ接線を共有した比較面の返戻
				return SphericalSplineField::CreateOnMeshWithGradients(
					6,
					Refined->GetDirections(),
					Refined->GetTriangles(),
					Refined->GetElevations(),
					*Limited,
					Stop
				);
			}
			// 元辺を保ち，追加した内部点も同じ C1 面へ渡す返戻
			return SphericalSplineField::CreateOnMesh(6, Refined->GetDirections(), Refined->GetTriangles(), Refined->GetElevations(), Stop);
		}
		// 有限幅の河口底を持つ同じ不変場の返戻
		return SphericalSplineField::CreateOnMesh(6, Mesh->Directions, Mesh->Triangles, Heights, Stop);
	}

	/**
	 * 海底だけを復元する合成場を保持する
	 * @param InSurface 節点間も連続な基盤
	 * @param InOcean 元の海底を持つ共有場
	 * @param InDrainage 水位と水深を定義した共有網
	 */
	PlanetSphericalDrainage::PlanetSphericalDrainage(
		SphericalSplineField InSurface,
		std::shared_ptr<const OceanConnectivityField> InOcean,
		std::shared_ptr<const DrainageNetwork> InDrainage
	) : Surface(std::move(InSurface)), Ocean(std::move(InOcean)), Drainage(std::move(InDrainage)) {
		Stats.Nodes = Surface.GetStatistics().Vertices;
		Stats.CoastalOutlets = Drainage->Statistics().CoastalOutletNodes;
		Stats.SplineResidentBytes = Surface.GetStatistics().ResidentBytes;
		// 元地質と同じ標高尺度を使い，最小惑星でも大陸棚の復元を有効にする
		const double Radius = Ocean->Base().Settings().RadiusMeters, Scale = Radius / (Radius + 250000);
		double DeepestOutlet = 0;
		const std::span<const DrainageNode> Nodes = Drainage->Nodes();
		for(std::size_t Index = Nodes.size() - Stats.CoastalOutlets; Index < Nodes.size(); ++Index) {
			DeepestOutlet = std::max(DeepestOutlet, Nodes[Index].DepthMeters);
		}
		// 陸から河口へ下る補間辺の最低値は河口底なので，其の水深迄は基盤を保持する
		Stats.SeabedBlendStartMeters =
		std::max(UnscaledSeabedBlendStartMeters * Scale, DeepestOutlet + Drainage->Settings().BankFreeboardMeters);
		Stats.SeabedMinimumBlendWidthMeters = std::max(UnscaledSeabedMinimumBlendWidthMeters * Scale, Stats.SeabedBlendStartMeters);
		// 終了
		return;
	}

	/**
	 * 同じ生成定義の海底・平衡縦断・任意の内部細分化から C1 場を準備する
	 * @param InOcean 元地質と接続海域の共有場
	 * @param InDrainage 同じ海域から準備した平衡縦断
	 * @param Refinement 同じ半径・海面の内部細分化，空なら追加しない
	 * @param Stop 中断要求
	 * @param InitialMesh 排水前に標本化した共有網，空なら従来の段階６
	 * @param Mode 補間へ渡す接線の準備方法
	 * @return 入力と生成定義が一致する比較場，不正又は中断では空
	 */
	std::optional<PlanetSphericalDrainage> PlanetSphericalDrainage::CreateImpl(
		std::shared_ptr<const OceanConnectivityField> InOcean,
		std::shared_ptr<const DrainageNetwork> InDrainage,
		const SphericalReliefSettings *const Refinement,
		const std::stop_token Stop,
		const SphericalReliefSamplingMesh *const InitialMesh,
		const SphericalGradientMode Mode
	) {
		// 未準備の場や中断要求に対する返戻
		if(!InOcean || !InDrainage || Stop.stop_requested()) return {};
		// 異なる地形定義を混合しない返戻
		if(InOcean->DefinitionHash() != InDrainage->SourceDefinitionHash()) return {};
		if(Refinement) {
			const GeologySettings &Settings = InOcean->Base().Settings();
			const bool HasMatchingScale =
			Refinement->RadiusMeters == Settings.RadiusMeters && Refinement->SeaLevelMeters == Settings.SeaLevelMeters;
			// 異なる物理尺度で追加点の下降を判定しない返戻
			if(!Refinement->IsValid() || !HasMatchingScale) return {};
		}
		SphericalReliefStatistics RefinementStats;
		std::optional<SphericalSplineField> Prepared =
		PrepareSurface(*InDrainage, *InOcean, Refinement, RefinementStats, Stop, InitialMesh, Mode);
		// 未成立の補間を公開しない返戻
		if(!Prepared) return {};
		PlanetSphericalDrainage Result(std::move(*Prepared), std::move(InOcean), std::move(InDrainage));
		Result.Stats.Relief = RefinementStats;
		Result.Stats.GradientMode = Mode;
		// 元の水位と海岸を共有する不変場の返戻
		return Result;
	}

	/**
	 * 元の排水網と河口から，追加細分化の無い比較場を準備する
	 * @param InOcean 元地質と接続海域
	 * @param InDrainage 同じ海域の平衡縦断
	 * @param Stop 中断要求
	 * @return 有効な比較場，不正又は中断では空
	 */
	std::optional<PlanetSphericalDrainage> PlanetSphericalDrainage::Create(
		std::shared_ptr<const OceanConnectivityField> InOcean,
		std::shared_ptr<const DrainageNetwork> InDrainage,
		const std::stop_token Stop
	) {
		// 従来の補間経路の返戻
		return CreateImpl(std::move(InOcean), std::move(InDrainage), nullptr, Stop);
	}

	/**
	 * 既存排水辺を保持した内部細分化を含む比較場を準備する
	 * @param InOcean 保持したい元起伏と接続海域
	 * @param InDrainage 同じ海域の平衡縦断
	 * @param Refinement 同じ半径・海面に於ける誤差と準備予算
	 * @param Stop 中断要求
	 * @return 追加点にも下降辺を持つ比較場，不正又は中断では空
	 */
	std::optional<PlanetSphericalDrainage> PlanetSphericalDrainage::CreateWithRefinedRelief(
		std::shared_ptr<const OceanConnectivityField> InOcean,
		std::shared_ptr<const DrainageNetwork> InDrainage,
		const SphericalReliefSettings &Refinement,
		const std::stop_token Stop
	) {
		// 既存の排水辺を保持する細分化経路の返戻
		return CreateImpl(std::move(InOcean), std::move(InDrainage), &Refinement, Stop);
	}

	/**
	 * 両方の場が海面より深い範囲だけで元海底へ連続に戻す
	 * 重みが正なら両標高は海面未満なので，合成前後で水没領域は厳密に同じとなる
	 * @tparam HasDerivatives 解析勾配も計算する場合に真
	 * @param Direction 球面方向
	 * @return 基盤と同じ海岸・海域接続を持つ標高と，必要に応じた接線勾配
	 */
	template<bool HasDerivatives> TerrainSample PlanetSphericalDrainage::EvaluateImpl(const UnitDirection Direction) const {
		const TerrainSample Base = Surface.Evaluate(Direction);
		const double Sea = Drainage->Settings().SeaLevelMeters, BaseDepth = Sea - Base.ElevationMeters;
		// 陸と浅い河口は基盤を厳密に保持する返戻
		if(BaseDepth <= Stats.SeabedBlendStartMeters) return Base;
		TerrainSample Source;
		if constexpr(HasDerivatives) Source = Ocean->Evaluate(Direction);
		else Source.ElevationMeters = Ocean->EvaluateElevation(Direction);
		const double SourceDepth = Sea - Source.ElevationMeters;
		// 元地形が陸又は浅海の場所へ，海底の形を無条件に貼付けない返戻
		if(SourceDepth <= Stats.SeabedBlendStartMeters) return Base;
		// 標高差より狭い固定帯で混合すると，重みの微分が人工的な断崖を作る
		const double Difference = Source.ElevationMeters - Base.ElevationMeters;
		const double Width = std::hypot(Stats.SeabedMinimumBlendWidthMeters, Difference), End = Stats.SeabedBlendStartMeters + Width;
		// 高低差も含めた遷移帯を越えれば，元海底の値と勾配を其の儘保持した返戻
		if(BaseDepth >= End && SourceDepth >= End) return Source;
		const double A = std::clamp((BaseDepth - Stats.SeabedBlendStartMeters) / Width, 0.0, 1.0);
		const double B = std::clamp((SourceDepth - Stats.SeabedBlendStartMeters) / Width, 0.0, 1.0), WA = A * A * (3 - 2 * A);
		const double WB = B * B * (3 - 2 * B), Weight = WA * WB;
		TerrainSample Result;
		Result.ElevationMeters = std::lerp(Base.ElevationMeters, Source.ElevationMeters, Weight);
		if constexpr(HasDerivatives) {
			const double DA = -6 * A * (1 - A) / Width, DB = -6 * B * (1 - B) / Width;
			const Vec3 GradientWidth = (Source.TangentialGradientPerRadian - Base.TangentialGradientPerRadian) * (Difference / Width);
			const Vec3 GradientWeight = (Base.TangentialGradientPerRadian + GradientWidth * A) * (WB * DA) +
			(Source.TangentialGradientPerRadian + GradientWidth * B) * (WA * DB);
			Result.TangentialGradientPerRadian =
			Base.TangentialGradientPerRadian * (1 - Weight) + Source.TangentialGradientPerRadian * Weight + GradientWeight * Difference;
		}
		// 水没領域を変えない凸結合と其の連鎖微分の返戻
		return Result;
	}

	/**
	 * 気候の準備にも，比較元と同じ不変な海陸の所有権を渡す
	 * @return 元の地形評価と同じ海域場
	 */
	std::shared_ptr<const OceanConnectivityField> PlanetSphericalDrainage::GetOriginalOcean() const {
		return Ocean;
	}

	/**
	 * 比較面の準備で共有した元地形を照会する
	 * @param Direction 同じ惑星の単位方向
	 * @return 元の地質・造山・起伏・海域接続を含む値と勾配
	 */
	TerrainSample PlanetSphericalDrainage::EvaluateOriginal(const UnitDirection Direction) const {
		// 追加の近似を挟まない元場の返戻
		return Ocean->Evaluate(Direction);
	}

	/**
	 * 比較元の同じ標高を，解析勾配を組立てずに照会する
	 * @param Direction 同じ惑星の単位方向
	 * @return 元の海域・地質・起伏を保持した標高
	 */
	double PlanetSphericalDrainage::EvaluateOriginalElevation(const UnitDirection Direction) const {
		return Ocean->EvaluateElevation(Direction);
	}

	/**
	 * 海底復元を含む場を評価する
	 * @param Direction 球面方向
	 * @return 標高と解析的な球面勾配
	 */
	TerrainSample PlanetSphericalDrainage::Evaluate(const UnitDirection Direction) const {
		// 微分付の点照会の返戻
		return EvaluateImpl<true>(Direction);
	}

	/**
	 * 地形準備用に標高だけを評価する
	 * @param Direction 球面方向
	 * @return 同じ場の標高
	 */
	double PlanetSphericalDrainage::EvaluateElevation(const UnitDirection Direction) const {
		// 微分計算を省いた海底評価の返戻
		return EvaluateImpl<false>(Direction).ElevationMeters;
	}

	/**
	 * 補間の節点数と保持量を参照する
	 * @return 共有元の場を二重計上しない補間単独の保持量
	 */
	const SphericalDrainageStatistics &PlanetSphericalDrainage::GetStatistics() const {
		// 準備済統計の返戻
		return Stats;
	}
}
