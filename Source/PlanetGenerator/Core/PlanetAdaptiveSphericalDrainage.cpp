#include "PlanetSphericalDrainage.h"
#include "PlanetDrainage.h"
#include "PlanetOcean.h"
#include "PlanetSphericalGradient.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace {
	using PlanetCore::DrainageNetwork;
	using PlanetCore::OceanConnectivityField;
	using PlanetCore::PlanetSphericalRefinement;
	using PlanetCore::SphericalReliefSamplingMesh;
	using PlanetCore::SphericalGradientMode;
	using PlanetCore::SphericalReliefSettings;
	using PlanetCore::SphericalSplineField;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;

	/**
	 * 未調整の元場を，海陸符号を保つ共有接線で補間する
	 * @param Mesh 排水前の標本化と海域再接続を終えた網
	 * @param Ocean 同じ網で接続した元地形
	 * @param Sampling 物理尺度と誤差の設定
	 * @param Mode 元勾配又は標本尺度の推定勾配
	 * @param Stop 中断要求
	 * @return 河口と縦断へ共用する元場，不正又は中断では空
	 */
	std::optional<SphericalSplineField> PrepareSourceSurface(
		const SphericalReliefSamplingMesh &Mesh,
		const OceanConnectivityField &Ocean,
		const SphericalReliefSettings &Sampling,
		const SphericalGradientMode Mode,
		const std::stop_token Stop
	) {
		// 節点を再追加せず，同じ隣接関係だけを接線制限へ使う
		SphericalReliefSettings Settings = Sampling;
		Settings.MaximumAddedVertices = 0;
		const auto Height = [&Ocean](const UnitDirection Direction) -> double {
			// 海域再接続を含む元標高の返戻
			return Ocean.EvaluateElevation(Direction);
		};
		const std::optional<PlanetSphericalRefinement> Scaffold =
		PlanetSphericalRefinement::Create(Mesh.Directions, Mesh.Triangles, Mesh.ElevationsMeters, Height, Settings, Stop);
		// 同じ節点を保持出来ない場合の返戻
		if(!Scaffold) return {};
		std::vector<Vec3> Gradients;
		std::vector<std::uint32_t> Unconstrained(Mesh.Directions.size());
		std::iota(Unconstrained.begin(), Unconstrained.end(), 0U);
		if(Mode == SphericalGradientMode::Sampled) {
			std::optional<PlanetCore::SphericalGradientEstimate> Estimated =
			PlanetCore::EstimateSphericalGradients(Mesh.Directions, Mesh.Triangles, Mesh.ElevationsMeters, Stop);
			// 同じ元標高の尺度から接線を得られない場合の返戻
			if(!Estimated) return {};
			Gradients = std::move(Estimated->Gradients);
		} else {
			Gradients.reserve(Mesh.Directions.size());
			for(const UnitDirection Direction : Mesh.Directions) {
				// 高価な元場の照会途中でも中断を受理する返戻
				if(Stop.stop_requested()) return {};
				Gradients.push_back(Ocean.Evaluate(Direction).TangentialGradientPerRadian);
			}
		}
		// 未確定の流路に接線を従わせず，海陸符号の制限だけを与える
		const std::optional<std::vector<Vec3>> Limited =
		Scaffold->ConstrainDrainageGradients(Gradients, Settings.SeaLevelMeters, Unconstrained, Stop);
		// 有効な接線を準備出来ない場合の返戻
		if(!Limited) return {};
		// 全段階で共有する連続元場の返戻
		return SphericalSplineField::CreateOnMeshWithGradients(
			6,
			Mesh.Directions,
			Mesh.Triangles,
			Mesh.ElevationsMeters,
			*Limited,
			Stop
		);
	}
}

namespace PlanetCore {
	/**
	 * 元起伏の標本化，海域接続，排水，縦断，河口を同じ適応網で準備する
	 * @param InOcean 造山と排水前起伏を含む元地形
	 * @param Sampling 同じ物理尺度の標本化誤差と有限予算
	 * @param Mode 元勾配又は標本尺度の推定勾配
	 * @param Stop 中断要求
	 * @return 適応網の C1 比較場，不正・容量不足・中断では空
	 */
	std::optional<PlanetSphericalDrainage> PlanetSphericalDrainage::CreateWithAdaptiveDrainage(
		std::shared_ptr<const OceanConnectivityField> InOcean,
		const SphericalReliefSettings &Sampling,
		const SphericalGradientMode Mode,
		const std::stop_token Stop
	) {
		// 未準備又は中断済の入力を評価しない返戻
		if(!InOcean || !Sampling.IsValid() || Stop.stop_requested()) return {};
		// 未定義の推定方式を既定値へ丸めない返戻
		if(Mode != SphericalGradientMode::Analytic && Mode != SphericalGradientMode::Sampled) return {};
		const GeologySettings &Definition = InOcean->Base().Settings();
		const bool HasSharedScale =
		Sampling.RadiusMeters == Definition.RadiusMeters && Sampling.SeaLevelMeters == Definition.SeaLevelMeters &&
		InOcean->Statistics().InputGraphNodes == 40962;
		// 異なる尺度や元網を暗黙に混合しない返戻
		if(!HasSharedScale) return {};
		const std::optional<SphereMesh> Base = BuildUniformSphere(6);
		// 元網を準備出来ない場合の返戻
		if(!Base) return {};
		const auto Original = [&InOcean](const UnitDirection Direction) -> double {
			// 既存流路で補正する前の元起伏の返戻
			return InOcean->EvaluateElevation(Direction);
		};
		std::vector<double> Heights;
		Heights.reserve(Base->Directions.size());
		for(const UnitDirection Direction : Base->Directions) {
			// 標本化の準備中も中断を受理する返戻
			if(Stop.stop_requested()) return {};
			Heights.push_back(Original(Direction));
		}
		std::optional<SphericalReliefSamplingMesh> Mesh =
		PlanetSphericalRefinement::CreateSamplingMesh(Base->Directions, Base->Triangles, Heights, Original, Sampling, Stop);
		// 既存流路へ地形を押付けずに標本化出来ない場合の返戻
		if(!Mesh) return {};
		std::optional<OceanConnectivityField> Connected = InOcean->WithSamplingMesh(Mesh->Directions, Mesh->Triangles, Stop);
		// 新たに見付かった水没点を孤立した儘で排水へ渡さない返戻
		if(!Connected) return {};
		for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			// 連続補正の再評価中も中断を受理する返戻
			if(Stop.stop_requested()) return {};
			Mesh->ElevationsMeters[Index] = Connected->EvaluateElevation(Mesh->Directions[Index]);
		}
		std::optional<SphericalSplineField> Source = PrepareSourceSurface(*Mesh, *Connected, Sampling, Mode, Stop);
		// 元地形を連続化出来ない場合の返戻
		if(!Source) return {};
		std::optional<DrainageNetwork> Network =
		DrainageNetwork::CreateForSphericalSurfaceOnMesh(*Connected, *Source, Mesh->Directions, Mesh->Triangles, Stop);
		// 実際の適応網から大洋へ流れる縦断を得られない場合の返戻
		if(!Network) return {};
		Network = Network->WithBalancedTerrainProfiles(
			[&Source](const UnitDirection Direction) -> double {
				// 海岸と縦断を共有する元場の返戻
				return Source->Evaluate(Direction).ElevationMeters;
			},
			Stop
		);
		// 合流点を共有する縦断を準備出来ない場合の返戻
		if(!Network) return {};
		// 最終面を作る前に，準備専用の大きな補間係数を解放する
		Source.reset();
		SphericalReliefSettings Reconstruction = Sampling;
		Reconstruction.MaximumAddedVertices = 0;
		Reconstruction.MinimumSlope = Network->Settings().MinimumSlope;
		Reconstruction.ShouldRefineEdges = true;
		Reconstruction.ShouldLimitToDrainageEdges = true;
		Reconstruction.ShouldRelaxAddedHeights = false;
		std::optional<PlanetSphericalDrainage> Result = CreateImpl(
			std::make_shared<const OceanConnectivityField>(std::move(*Connected)),
			std::make_shared<const DrainageNetwork>(std::move(*Network)),
			&Reconstruction,
			Stop,
			&*Mesh,
			Mode
		);
		// 同じ排水辺と河口を保持する最終面を作れない場合の返戻
		if(!Result) return {};
		// 標本化の統計を保持し，実際の節点調整量は別に計測する
		const std::uint32_t PreservedSlopes = Result->Stats.Relief.PreservedSlopeVertices;
		Result->Stats.Relief = Mesh->Statistics;
		Result->Stats.Relief.PreservedSlopeVertices = PreservedSlopes;
		for(const DrainageNode &Node : Result->Drainage->Nodes()) {
			if(Node.Ocean) continue;
			const double Sea = Sampling.SeaLevelMeters, Head = Node.WaterMeters - Sea;
			const double Freeboard = Result->Drainage->Settings().BankFreeboardMeters;
			const double Delta = Node.WaterMeters + Freeboard * Head / (Head + Freeboard) - Node.OriginalElevationMeters;
			if(Delta > Result->Stats.Relief.MaximumConditioningRaiseMeters) {
				Result->Stats.Relief.MaximumConditioningRaiseMeters = Delta;
				Result->Stats.Relief.MaximumRaiseDirection = Node.Direction;
			}
			if(-Delta > Result->Stats.Relief.MaximumConditioningCutMeters) {
				Result->Stats.Relief.MaximumConditioningCutMeters = -Delta;
				Result->Stats.Relief.MaximumCutDirection = Node.Direction;
			}
		}
		// 視点移動では再準備しない共有比較場の返戻
		return Result;
	}

	/**
	 * 従来の解析接線を使う適応排水の公開入口を保持する
	 * @param InOcean 元地形
	 * @param Sampling 物理尺度と標本化予算
	 * @param Stop 中断要求
	 * @return 従来の比較条件と同じ場，不正又は中断では空
	 */
	std::optional<PlanetSphericalDrainage> PlanetSphericalDrainage::CreateWithAdaptiveDrainage(
		std::shared_ptr<const OceanConnectivityField> InOcean,
		const SphericalReliefSettings &Sampling,
		const std::stop_token Stop
	) {
		// 従来の解析接線を明示した経路の返戻
		return CreateWithAdaptiveDrainage(std::move(InOcean), Sampling, SphericalGradientMode::Analytic, Stop);
	}

	/**
	 * 比較場と同じ河口・共有水位を保持する排水網を照会する
	 * @return 不変な元排水網への参照
	 */
	const DrainageNetwork &PlanetSphericalDrainage::GetDrainageNetwork() const {
		// 不変場の寿命内で共有する排水網の返戻
		return *Drainage;
	}
}
