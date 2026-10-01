#include "Core/PlanetFlowChannelField.h"
#include "Core/PlanetFlowOutlet.h"
#include "Core/PlanetFlowPreparation.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowFieldContracts,
	"PlanetGenerator.FlowChannel.Field",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * エンジン非依存の共有場を，解析的な二重窪地と正規・拒否の両経路で検査する
 * @param Parameters 未使用の実行引数
 * @return 尺度・水位・所有する定義と，湖外までの地点評価を確認した場合に真
 */
bool FPlanetFlowFieldContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Origin = *PlanetCore::UnitDirection::FromVector({ 0, 0, 1 });
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto DirectionAt = [Radius](const double X, const double Y) -> PlanetCore::UnitDirection {
			// 同じ物理座標の球面方向を返す
			return *PlanetCore::UnitDirection::FromVector({ X / Radius, Y / Radius, 1 });
		};
		const auto Source = [Radius](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double U = Radius * D.X / D.Z / 40, V = Radius * D.Y / D.Z / 20, DX = (4 * U * U * U - 4 * U) / 40, DY = 2 * V / 20;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 元の二重窪地と独立な解析勾配の返戻
			return { U * U * U * U - 2 * U * U + V * V + 1, Gradient };
		};
		PlanetCore::FlowOutletSettings OutletSettings;
		OutletSettings.RadiusMeters = Radius;
		OutletSettings.DownstreamWaterMeters = 0.25;
		PlanetCore::FlowOutletDiagnostics Report;
		std::optional<PlanetCore::FlowOutletRoute> Outlet =
		PlanetCore::PrepareFlowOutletRoute(Origin, Source, OutletSettings, {}, &Report);
		AddInfo(
			FString::Printf(
				TEXT("Analytic outlet R=%.0f failure=%u upEnd=%u downEnd=%u upVertices=%u upDistance=%.12g upHeight=%.12g ")
				TEXT("upDirection=(%.17g,%.17g,%.17g) axis=(%.17g,%.17g,%.17g)"),
				Radius,
				static_cast<unsigned>(Report.Failure),
				static_cast<unsigned>(Report.UpstreamEnd),
				static_cast<unsigned>(Report.DownstreamEnd),
				Report.UpstreamVertices,
				Report.UpstreamDistanceMeters,
				Report.UpstreamHeightMeters,
				Report.UpstreamDirection.X,
				Report.UpstreamDirection.Y,
				Report.UpstreamDirection.Z,
				Report.DownstreamAxis.X,
				Report.DownstreamAxis.Y,
				Report.DownstreamAxis.Z
			)
		);
		// 解析場の正規の準備経路を確認する返戻
		if(!TestTrue(TEXT("二重窪地の流出口と下降枝を準備"), Outlet.has_value())) return false;
		if(Report.UpstreamEnd == PlanetCore::TerrainDescentEnd::StepLimit) {
			TestTrue(TEXT("最小歩幅の停止は同じ元場の極小を確認"), Report.HasUpstreamMinimum);
			TestTrue(TEXT("極小は最小歩幅の１ｍｍ以内"), Report.UpstreamMinimumMoveMeters <= 0.001);
		}
		AddInfo(
			FString::Printf(
				TEXT("Upstream minimum confirmation: verified=%d distance=%.12g"),
				static_cast<int>(Report.HasUpstreamMinimum),
				Report.UpstreamMinimumMoveMeters
			)
		);
		PlanetCore::TerrainContourSettings ShoreSettings;
		ShoreSettings.RadiusMeters = Radius;
		ShoreSettings.MaximumLengthMeters = std::min(ShoreSettings.MaximumLengthMeters, Radius * 4);
		PlanetCore::SpillLakeDiagnostics ShoreReport;
		std::optional<PlanetCore::SpillLakeShore> Lake =
		PlanetCore::PrepareSpillLakeShore(Outlet->Pass, false, Source, ShoreSettings, {}, &ShoreReport);
		AddInfo(
			FString::Printf(
				TEXT("Analytic lake R=%.0f failure=%u end=%u vertices=%u length=%.12g heightError=%.12g gap=%.12g area=%.12g ")
				TEXT("last=(%.17g,%.17g,%.17g) curvature=(%.17g,%.17g)"),
				Radius,
				static_cast<unsigned>(ShoreReport.Failure),
				static_cast<unsigned>(ShoreReport.End),
				ShoreReport.Vertices,
				ShoreReport.LengthMeters,
				ShoreReport.HeightResidualMeters,
				ShoreReport.FirstGapMeters,
				ShoreReport.AreaMetersSquared,
				ShoreReport.LastDirection.X,
				ShoreReport.LastDirection.Y,
				ShoreReport.LastDirection.Z,
				Outlet->Pass.MinimumCurvaturePerMeter,
				Outlet->Pass.MaximumCurvaturePerMeter
			)
		);
		// 正しい岸の欠落を試験除外にしない返戻
		if(!TestTrue(TEXT("上流の湖岸を準備"), Lake.has_value())) return false;
		PlanetCore::FlowChannelFieldDefinition Definition;
		Definition.Source = Source;
		Definition.SourceHeight = [Source](const PlanetCore::UnitDirection Direction) -> double {
			// 同じ元場を標高専用の呼出形で渡す
			return Source(Direction).ElevationMeters;
		};
		Definition.Route = std::make_shared<const PlanetCore::TerrainFlowRoute>(std::move(Outlet->Route));
		Definition.Lake = std::make_shared<const PlanetCore::SpillLakeShore>(std::move(*Lake));
		Definition.Shape.RadiusMeters = Radius;
		Definition.PassDistanceMeters = Outlet->PassDistanceMeters;
		Definition.UpstreamWaterMeters = Outlet->Pass.ElevationMeters;
		Definition.WaterSettings.DownstreamWaterMeters = OutletSettings.DownstreamWaterMeters;
		Definition.WaterQueryHalfWidthMeters = 8;
		std::optional<PlanetCore::FlowChannelProfile> Profile =
		PlanetCore::FlowChannelProfile::Create(Definition.Shape, Definition.Route->GetLengthMeters(), {});
		// 構成する縦断寸法の失敗への返戻
		if(!TestTrue(TEXT("同じ尺度の縦断寸法を準備"), Profile.has_value())) return false;
		Definition.ShapeProfile = std::make_shared<const PlanetCore::FlowChannelProfile>(std::move(*Profile));
		const std::optional<PlanetCore::FlowChannelField> Field = PlanetCore::FlowChannelField::Create(Definition);
		// 正規の入力を誤って拒否しない返戻
		if(!TestTrue(TEXT("確認済資源から共有場を準備"), Field.has_value())) return false;
		PlanetCore::FlowChannelFieldDefinition FullDefinition = Definition;
		FullDefinition.SourceHeight = {};
		const std::optional<PlanetCore::FlowChannelField> Full = PlanetCore::FlowChannelField::Create(FullDefinition);
		// 省略可能な照会が無い場合の正規経路への返戻
		if(!TestTrue(TEXT("標高専用の照会を省略出来る"), Full.has_value())) return false;
		// 同じ解析場を独立な手順で組立てた基準と，描画に依存しない生成入口を照合
		PlanetCore::FlowPreparationSettings PreparationSettings;
		PreparationSettings.Outlet = OutletSettings;
		PlanetCore::FlowPreparationDiagnostics PreparationReport;
		const std::optional<PlanetCore::PreparedFlowChannel> Prepared =
		PlanetCore::PrepareFlowChannel(Origin, Source, Definition.SourceHeight, PreparationSettings, {}, {}, &PreparationReport);
		// 正規の準備を拒否した場合の返戻
		if(!TestTrue(TEXT("描画無しで不変な河道を準備"), Prepared.has_value())) return false;
		TestEqual(TEXT("独立準備と同じ鞍点距離"), Prepared->PassDistanceMeters, Definition.PassDistanceMeters);
		TestEqual(TEXT("独立準備と同じ上流水位"), Prepared->UpstreamWaterMeters, Definition.UpstreamWaterMeters);
		TestEqual(TEXT("短い解析経路では淵を追加しない"), Prepared->PoolCount, 0U);
		PlanetCore::FlowChannelFieldDefinition ComparisonDefinition = Definition;
		ComparisonDefinition.WaterQueryHalfWidthMeters = std::min(500.0, Radius * 0.025);
		const std::optional<PlanetCore::FlowChannelField> Comparison = PlanetCore::FlowChannelField::Create(ComparisonDefinition);
		// 比較条件を同じ水位照会幅へ揃えられない場合の返戻
		if(!TestTrue(TEXT("独立した組立基準"), Comparison.has_value())) return false;
		unsigned Samples = 0;
		for(unsigned I = 0; I < 43; ++I) for(const double Y : { -12.0, 0.0, 12.0, 30.0 }) {
			const PlanetCore::UnitDirection Query = DirectionAt(-50 + 2.0 * I, Y);
			const std::optional<PlanetCore::FlowChannelSection> A = Field->Evaluate(Query), B = Field->Evaluate(Query, true, false);
			const std::optional<PlanetCore::FlowChannelSection> C = Full->Evaluate(Query, true, false);
			// 有効な照会の欠落を乾いた点で置換えない返戻
			if(!TestTrue(TEXT("地面と水位の両方式が定義"), A && B && C)) return false;
			TestEqual(TEXT("元の微分省略でも同じ地形"), A->Terrain.ElevationMeters, B->Terrain.ElevationMeters);
			TestEqual(TEXT("標高専用が無くても同じ地形"), A->Terrain.ElevationMeters, C->Terrain.ElevationMeters);
			TestEqual(TEXT("元の微分省略でも同じ水位"), A->Water.ElevationMeters, B->Water.ElevationMeters);
			TestEqual(TEXT("標高専用が無くても同じ水位"), A->Water.ElevationMeters, C->Water.ElevationMeters);
			const std::optional<double> Support = Field->WaterSupport(Query), FullSupport = Full->WaterSupport(Query);
			// 水域判定の欠落を無視しない返戻
			if(!TestTrue(TEXT("水域の所属を両方式で評価"), Support && FullSupport)) return false;
			TestEqual(TEXT("標高の照会方法で所属を変えない"), *Support, *FullSupport);
			// 一括照会は独立した旧操作口と全属性を厳密に照合
			for(const bool NeedsGradient : { false, true }) {
				const std::optional<PlanetCore::FlowWaterSample> Joined = Field->EvaluateWithSupport(Query, NeedsGradient);
				const std::optional<PlanetCore::FlowWaterSample> JoinedFull = Full->EvaluateWithSupport(Query, NeedsGradient);
				// 省略可能な標高照会の有無を共に検査する返戻
				if(!TestTrue(TEXT("一括照会が両方式で存在"), Joined && JoinedFull)) return false;
				for(const PlanetCore::FlowWaterSample &Result : { *Joined, *JoinedFull }) {
					TestEqual(TEXT("一括照会の河床を保持"), Result.Section.Terrain.ElevationMeters, A->Terrain.ElevationMeters);
					TestEqual(TEXT("一括照会の水位を保持"), Result.Section.Water.ElevationMeters, A->Water.ElevationMeters);
					TestEqual(TEXT("一括照会の所属を保持"), Result.SupportMeters, *Support);
					TestEqual(TEXT("一括照会の水面の有無"), Result.Section.HasWater, A->HasWater);
					TestEqual(TEXT("一括照会の開削被覆"), Result.Section.BedCoverage, A->BedCoverage);
					const PlanetCore::Vec3 WaterError = Result.Section.Water.TangentialGradientPerRadian - A->Water.TangentialGradientPerRadian;
					TestEqual(TEXT("地面の微分を省いても水面微分を保持"), WaterError.Length(), 0.0);
					if(NeedsGradient) {
						const PlanetCore::Vec3 Error = Result.Section.Terrain.TangentialGradientPerRadian - A->Terrain.TangentialGradientPerRadian;
						TestEqual(TEXT("一括照会の地面微分を保持"), Error.Length(), 0.0);
					}
				}
			}
			const std::optional<PlanetCore::FlowChannelSection> Ready = Prepared->Field->Evaluate(Query);
			const std::optional<PlanetCore::FlowChannelSection> Expected = Comparison->Evaluate(Query);
			// 独立に組立てた全地形・水位・所属を比較する返戻
			if(!TestTrue(TEXT("生成入口と独立組立の全照会が存在"), Ready && Expected)) return false;
			TestEqual(TEXT("生成入口も同じ河床"), Ready->Terrain.ElevationMeters, Expected->Terrain.ElevationMeters);
			TestEqual(TEXT("生成入口も同じ河床微分Ｘ"), Ready->Terrain.TangentialGradientPerRadian.X, Expected->Terrain.TangentialGradientPerRadian.X);
			TestEqual(TEXT("生成入口も同じ河床微分Ｙ"), Ready->Terrain.TangentialGradientPerRadian.Y, Expected->Terrain.TangentialGradientPerRadian.Y);
			TestEqual(TEXT("生成入口も同じ河床微分Ｚ"), Ready->Terrain.TangentialGradientPerRadian.Z, Expected->Terrain.TangentialGradientPerRadian.Z);
			TestEqual(TEXT("生成入口も同じ水位"), Ready->Water.ElevationMeters, Expected->Water.ElevationMeters);
			TestEqual(TEXT("生成入口も同じ水面の有無"), Ready->HasWater, Expected->HasWater);
			TestTrue(TEXT("生成入口も同じ支持距離"), Prepared->Field->WaterSupport(Query) == Comparison->WaterSupport(Query));
			++Samples;
		}
		// 不正入力と中断で元場を不必要に評価せず，部分場を返さない事の確認
		unsigned SourceCalls = 0;
		std::stop_source Cancel;
		const auto CountedSource = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++SourceCalls;
			if(SourceCalls == 3) Cancel.request_stop();
			// 中断を要求した後も元の標本自体を変更しない返戻
			return Source(Direction);
		};
		TestFalse(
			TEXT("元場欠落を準備前に拒否"),
			PlanetCore::PrepareFlowChannel(Origin, {}, {}, PreparationSettings, {}, {}, &PreparationReport).has_value()
		);
		TestTrue(TEXT("元場欠落の理由を区別"), PreparationReport.Failure == PlanetCore::FlowPreparationFailure::InvalidInput);
		Cancel.request_stop();
		TestFalse(
			TEXT("中断済の準備を拒否"),
			PlanetCore::PrepareFlowChannel(Origin, CountedSource, {}, PreparationSettings, {}, Cancel.get_token(), &PreparationReport)
			.has_value()
		);
		TestEqual(TEXT("中断済なら元場を照会しない"), SourceCalls, 0U);
		TestTrue(TEXT("中断済の理由を保持"), PreparationReport.Failure == PlanetCore::FlowPreparationFailure::Cancelled);
		Cancel = std::stop_source();
		TestFalse(
			TEXT("準備中の中断でも部分場を返さない"),
			PlanetCore::PrepareFlowChannel(Origin, CountedSource, {}, PreparationSettings, {}, Cancel.get_token(), &PreparationReport)
			.has_value()
		);
		TestTrue(TEXT("下位処理へ中断を伝播"), SourceCalls > 2 && PreparationReport.Failure == PlanetCore::FlowPreparationFailure::Cancelled);
		TestFalse(
			TEXT("異なる受入水位の湖を準備で拒否"),
			PlanetCore::PrepareFlowChannel(Origin, Source, {}, PreparationSettings, Definition.Lake, {}, &PreparationReport).has_value()
		);
		TestTrue(TEXT("受入湖の不整合を区別"), PreparationReport.Failure == PlanetCore::FlowPreparationFailure::Field);
		const double UpstreamSide = Outlet->Pass.MinimumCurvatureDirection.X > 0 ? -1.0 : 1.0;
		const PlanetCore::UnitDirection WideLake = DirectionAt(UpstreamSide * 40, 12), Dry = DirectionAt(UpstreamSide * 40, 30);
		const std::optional<PlanetCore::FlowChannelSection> Wet = Field->Evaluate(WideLake), Land = Field->Evaluate(Dry);
		// 狭い河道照会の外でも，上流湖と乾いた元場を区別する返戻
		if(!TestTrue(TEXT("河道幅外を評価"), Wet && Land)) return false;
		TestTrue(TEXT("河道幅外の上流湖も水面を保持"), Wet->HasWater);
		TestEqual(TEXT("河道幅外の湖で上流の水位を保持"), Wet->Water.ElevationMeters, Definition.UpstreamWaterMeters);
		TestFalse(TEXT("湖外の元地形に水面を新設しない"), Land->HasWater);
		TestEqual(TEXT("補正外の元地形を保持"), Land->Terrain.ElevationMeters, Source(Dry).ElevationMeters);
		Definition.WaterSettings.DownstreamWaterMeters = 100;
		TestEqual(TEXT("呼出側の定義変更は準備済水位へ影響しない"), Field->Evaluate(WideLake)->Water.ElevationMeters, Wet->Water.ElevationMeters);
		TestFalse(TEXT("下流が高い定義を拒否"), PlanetCore::FlowChannelField::Create(Definition).has_value());
		Definition = FullDefinition;
		Definition.Source = {};
		TestFalse(TEXT("元場欠落を拒否"), PlanetCore::FlowChannelField::Create(Definition).has_value());
		Definition = FullDefinition;
		Definition.WaterQueryHalfWidthMeters = 1;
		TestFalse(TEXT("河道幅を覆わない水位照会を拒否"), PlanetCore::FlowChannelField::Create(Definition).has_value());
		Definition = FullDefinition;
		Definition.PassDistanceMeters = Definition.Route->GetLengthMeters() + 1;
		TestFalse(TEXT("経路外の流出口を拒否"), PlanetCore::FlowChannelField::Create(Definition).has_value());
		Definition = FullDefinition;
		Definition.Shape.RadiusMeters *= 2;
		TestFalse(TEXT("異なる尺度の縦断を拒否"), PlanetCore::FlowChannelField::Create(Definition).has_value());
		Definition = FullDefinition;
		Definition.ReceivingLake = Definition.Lake;
		TestFalse(TEXT("受入水位と異なる湖を拒否"), PlanetCore::FlowChannelField::Create(Definition).has_value());
		AddInfo(
			FString::Printf(
				TEXT("Shared flow field: radius=%.0f samples=%u wideLakeWater=%.12g"),
				Radius,
				Samples,
				Wet->Water.ElevationMeters
			)
		);
	}
	// 正規の地点評価と不整合の拒否を併せた検査結果の返戻
	return !HasAnyErrors();
}

#endif
