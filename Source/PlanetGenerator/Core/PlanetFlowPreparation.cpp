#include "PlanetFlowPreparation.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 同じ元地形の鞍点・下降枝・湖岸・縦断寸法を準備し，不変な地点評価へまとめる
	 * @param Pass 元の流出口候補
	 * @param Source 所有する不変場を捕捉した標高と解析勾配の照会
	 * @param SourceHeight 同じ元場の標高専用照会，省略時は Source を使用
	 * @param Settings 尺度・水位・向きと生成時の乱数条件
	 * @param ReceivingLake 確認済の受入湖，無しでは比較用の一定下流水位
	 * @param Stop 下位の追跡・岸・淵配置へ伝播する中断要求
	 * @param Diagnostics 失敗段階の出力先，省略可
	 * @return 完全に準備した共有場，未達・不正入力・中断では空
	 */
	std::optional<PreparedFlowChannel> PrepareFlowChannel(
		const UnitDirection Pass,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::function<double(UnitDirection)> &SourceHeight,
		const FlowPreparationSettings &Settings,
		std::shared_ptr<const SpillLakeShore> ReceivingLake,
		const std::stop_token Stop,
		FlowPreparationDiagnostics *const Diagnostics
	) {
		FlowPreparationDiagnostics Local;
		FlowPreparationDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const FlowPreparationFailure Failure) -> std::optional<PreparedFlowChannel> {
			Report.Failure = Stop.stop_requested() ? FlowPreparationFailure::Cancelled : Failure;
			// 部分的な準備を使用可能な場へ変換しない返戻
			return {};
		};
		// 未設定の照会と中断済の要求を元場へ渡さない返戻
		if(!Source || Stop.stop_requested()) return Reject(FlowPreparationFailure::InvalidInput);
		// 下流の水位が違う既存岸を，長い追跡後の別の停止理由へ読み替えない
		if(ReceivingLake) {
			const TerrainContour &Receiving = ReceivingLake->GetContour();
			if(Receiving.Samples.empty() || Receiving.Samples.front().ElevationMeters != Settings.Outlet.DownstreamWaterMeters) {
				// 異なる水位の受入岸への返戻
				return Reject(FlowPreparationFailure::Field);
			}
		}
		std::function<double(UnitDirection)> StopDomainDistance;
		if(ReceivingLake) StopDomainDistance = [ReceivingLake](const UnitDirection Direction) -> double {
			// 高度だけで岸の折線の外側を受入済にしない返戻
			return ReceivingLake->SignedPlanarDistanceMeters(Direction);
		};
		std::optional<FlowOutletRoute> Outlet =
		PrepareFlowOutletRoute(Pass, Source, Settings.Outlet, Stop, &Report.Outlet, StopDomainDistance);
		// 正しい下流水位へ到達しない準備の返戻
		if(!Outlet) return Reject(FlowPreparationFailure::Outlet);
		const double Radius = Settings.Outlet.RadiusMeters;
		TerrainContourSettings ShoreSettings;
		ShoreSettings.RadiusMeters = Radius;
		// 同じ精度の実岸が旧３２７６８点を越えて帰還する為，検証済の有限保持量へ揃える
		// 全水系の岸・河道・水位の合算予算は，呼出側で引続き検査する
		ShoreSettings.MaximumVertices = 65536;
		ShoreSettings.MaximumLengthMeters = std::min(3000000.0, Radius * 4);
		std::optional<SpillLakeShore> Lake =
		PrepareSpillLakeShore(Outlet->Pass, !Settings.Outlet.IsPositiveBranch, Source, ShoreSettings, Stop, &Report.Shore);
		// 元の湖岸が閉じない準備の返戻
		if(!Lake) return Reject(FlowPreparationFailure::Shore);
		// ワーカー終了後も元場を所有する，描画と独立した定義
		FlowChannelFieldDefinition Definition;
		Definition.Lake = std::make_shared<const SpillLakeShore>(std::move(*Lake));
		Definition.ReceivingLake = std::move(ReceivingLake);
		Definition.Source = Source;
		Definition.SourceHeight = SourceHeight;
		Definition.PassDistanceMeters = Outlet->PassDistanceMeters;
		Definition.UpstreamWaterMeters = Outlet->Pass.ElevationMeters;
		Definition.WaterQueryHalfWidthMeters = std::min(500.0, Radius * 0.025);
		Definition.Route = std::make_shared<const TerrainFlowRoute>(std::move(Outlet->Route));
		Definition.Shape.RadiusMeters = Radius;
		Definition.WaterSettings.DownstreamWaterMeters = Settings.Outlet.DownstreamWaterMeters;
		// P17 の急流の淵を局所形状として比較する為の生成時の配置
		// 係数は既存の比較候補を保持し，流量・岩質から較正した水理則とはしない
		std::uint64_t PoolState = Settings.Seed ^ UINT64_C(0X504F4F4C50524F46);
		const auto Random = [&]() -> double {
			PoolState += UINT64_C(0X9E3779B97F4A7C15);
			std::uint64_t Value = PoolState;
			Value = (Value ^ (Value >> 30)) * UINT64_C(0XBF58476D1CE4E5B9);
			Value = (Value ^ (Value >> 27)) * UINT64_C(0X94D049BB133111EB);
			Value ^= Value >> 31;
			// 生成時だけの再現可能な標本の返戻
			return static_cast<double>(Value >> 11) / 9007199254740992.0;
		};
		std::vector<FlowChannelPool> Pools;
		const double RouteLength = Definition.Route->GetLengthMeters();
		double Cursor = Definition.PassDistanceMeters + 25;
		while(Cursor + 100 < RouteLength) {
			// 生成途中の中断を不完全な縦断として保持しない返戻
			if(Stop.stop_requested()) return Reject(FlowPreparationFailure::Cancelled);
			const TerrainFlowCurveSample Point = *Definition.Route->Sample(Cursor);
			const double Slope = -Source(Point.Direction).TangentialGradientPerRadian.Dot(Point.DirectionDerivativePerMeter);
			const double Gap = Definition.Shape.HalfWidthMeters * (8 + 6 * Random()) / (1 + std::min(1.0, 10 * std::max(0.0, Slope)));
			const double HalfLength = Definition.Shape.HalfWidthMeters * (2 + 2 * Random()), CenterDistance = Cursor + Gap + HalfLength;
			Cursor = CenterDistance + HalfLength;
			if(Cursor + 25 > RouteLength) break;
			if(Slope > 0.02) Pools.push_back({ CenterDistance, HalfLength, 1 + 2 * Random(), 0.5 + 0.8 * Random() });
		}
		std::optional<FlowChannelProfile> Profile = FlowChannelProfile::Create(Definition.Shape, RouteLength, Pools);
		// 縦断寸法の不整合への返戻
		if(!Profile) return Reject(FlowPreparationFailure::Profile);
		Definition.ShapeProfile = std::make_shared<const FlowChannelProfile>(std::move(*Profile));
		const std::shared_ptr<const TerrainFlowRoute> Route = Definition.Route;
		std::optional<FlowChannelField> Field = FlowChannelField::Create(std::move(Definition), Stop, &Report.Water);
		// 場の整合性又は中断を最終公開前に確認する返戻
		if(!Field || Stop.stop_requested()) return Reject(FlowPreparationFailure::Field);
		// 完成した同じ所有定義と観察距離の対応を返す
		return PreparedFlowChannel{
			std::make_shared<const FlowChannelField>(std::move(*Field)),
			Route,
			Outlet->PassDistanceMeters,
			Outlet->Pass.ElevationMeters,
			static_cast<std::uint32_t>(Pools.size())
		};
	}
}
