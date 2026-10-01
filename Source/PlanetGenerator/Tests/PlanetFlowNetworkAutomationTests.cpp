#include "Core/PlanetFlowNetwork.h"
#include "Core/PlanetFlowNetworkField.h"
#include "Core/PlanetFlowWater.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "HAL/PlatformTime.h"
#include "Math/Float16.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stop_token>
#include <tuple>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 実際の三角形内部で，材質と法線の補間を元の地点評価へ照合する
	 * @param Test 検査結果の受取先
	 * @param Mesh 水面との共通細分化を終えた実地面
	 * @param Field 同じ不変な水系
	 * @return 欠落無く独立標本を評価出来た場合に真；景観の合格判定ではない
	 */
	bool InspectFlowAppearance(
		FAutomationTestBase &Test,
		const PlanetCore::FlowTerrainMesh &Mesh,
		const PlanetCore::FlowSurfaceField &Field
	) {
		const std::size_t Count = Mesh.Positions.size();
		// 未公開の部分配列を参照しない返戻
		if(!Test.TestTrue(TEXT("地形被覆の属性数が一致"), Mesh.Heights.size() == Count && Mesh.WaterCoordinates.size() == Count)) return false;
		std::uint32_t Samples = 0, DryWetErrors = 0;
		double MaximumNormal = 0, MaximumWetness = 0, MaximumLayer = 0, MaximumHeight = 0, MaximumDrySpan = 0, MaximumRawWetness = 0;
		double SumWetness = 0, SumRawWetness = 0, MaxRasterOld = 0, MaxRasterNew = 0, SumRasterOld = 0, SumRasterNew = 0;
		std::array<double, 8> WorstRaw {};
		std::uint32_t RawDryWetErrors = 0;
		std::map<std::uint32_t, std::array<double, 3>> Hydrology;
		PlanetCore::Vec3 WorstNormal, WorstWetness;
		const double Radius = Field.GetRadiusMeters(), R0 = Radius + Mesh.Frame.AnchorElevationMeters;
		const auto Quantize = [](const double Value) -> double {
			// 描画頂点色と同じ８ビットへの最近傍量子化の返戻
			return std::floor(std::clamp(Value, 0.0, 1.0) * 255 + 0.5) / 255;
		};
		const auto Smooth = [](const double A, const double B, const double X) -> double {
			const double T = std::clamp((X - A) / (B - A), 0.0, 1.0);
			// 同じ物理的な被覆の五次遷移を，補間後の画素で評価した値の返戻
			return T * T * T * (10 + T * (-15 + 6 * T));
		};
		for(const std::array<std::uint32_t, 3> Triangle : Mesh.Sphere.Mesh.Triangles) {
			const PlanetCore::Vec3 A = Mesh.Positions[Triangle[0]], B = Mesh.Positions[Triangle[1]], C = Mesh.Positions[Triangle[2]];
			const PlanetCore::Vec3 Center = (A + B + C) * (1.0 / 3);
			const double Cap = std::max({ (A - Center).Length(), (B - Center).Length(), (C - Center).Length() });
			if(Center.Length() > 500 + Cap) continue;
			for(
				const std::array<double, 3> Weights :
				{ std::array<double, 3>{ 0.2, 0.3, 0.5 }, std::array<double, 3>{ 0.6, 0.1, 0.3 }, std::array<double, 3>{ 0.15, 0.7, 0.15 } }
			) {
				const PlanetCore::Vec3 P = A * Weights[0] + B * Weights[1] + C * Weights[2];
				if(P.Length() > 500) continue;
				const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(Mesh.Frame.LocalToPlanet(P));
				const std::optional<PlanetCore::FlowWaterSample> Sample = Field.EvaluateWithSupport(D);
				// 欠落した水域を乾いた材質で置換えない返戻
				if(!Test.TestTrue(TEXT("材質検査の独立地点が定義"), Sample.has_value())) return false;
				const PlanetCore::TerrainSample &Terrain = Sample->Section.Terrain;
				PlanetCore::Vec3 Interpolated;
				std::array<double, 4> Actual {};
				std::array<double, 3> Raw {};
				for(unsigned I = 0; I < 3; ++I) {
					const std::uint32_t Vertex = Triangle[I];
					if(!Hydrology.contains(Vertex)) {
						const std::optional<PlanetCore::FlowWaterSample> V = Field.EvaluateWithSupport(Mesh.Sphere.Mesh.Directions[Vertex]);
						// 表示頂点で未定義の水位を補間しない返戻
						if(!Test.TestTrue(TEXT("未飽和の描画属性を評価"), V.has_value())) return false;
						const double H = V->Section.Terrain.ElevationMeters, Depth = V->Section.HasWater ? V->Section.Water.ElevationMeters - H : -1;
						const std::array<double, 3> Values { Mesh.Heights[Vertex], Mesh.WaterCoordinates[Vertex][0], Mesh.WaterCoordinates[Vertex][1] };
						// 再評価した同じ地点から，全ての搬送属性が作られた事を照合
						if(!Test.TestTrue(TEXT("未飽和の描画属性が独立評価と一致"), Values == std::array<double, 3>{ H, Depth, V->SupportMeters })) return false;
						std::array<double, 3> Packed {};
						for(unsigned K = 0; K < Values.size(); ++K) {
							const FFloat16 Half(static_cast<float>(std::clamp(Values[K], -32768.0, 32768.0)));
							Packed[K] = Half.GetFloat();
						}
						Hydrology.emplace(Vertex, Packed);
					}
					for(unsigned K = 0; K < Raw.size(); ++K) Raw[K] += Hydrology.at(Vertex)[K] * Weights[I];
					Interpolated = Interpolated + Mesh.PlanetNormals[Vertex] * Weights[I];
					const PlanetCore::SurfaceMaterialWeights &M = Mesh.Materials[Vertex];
					const std::array<double, 4> Values { M.Rock, M.Sand, M.Soil, M.Wetness };
					for(unsigned K = 0; K < Values.size(); ++K) Actual[K] += Quantize(Values[K]) * Weights[I];
				}
				const PlanetCore::Vec3 Normal =
				PlanetCore::UnitDirection::FromVector(D.Vector() * (Radius + Terrain.ElevationMeters) - Terrain.TangentialGradientPerRadian)
				->Vector();
				const double Angle =
				std::acos(std::clamp(Normal.Dot(PlanetCore::UnitDirection::FromVector(Interpolated)->Vector()), -1.0, 1.0));
				if(Angle > MaximumNormal) {
					MaximumNormal = Angle;
					WorstNormal = P;
				}
				PlanetCore::RiverSurfaceSample Material;
				Material.Terrain = Terrain;
				if(Sample->SupportMeters > 0 && Sample->Section.HasWater) {
					Material.ChannelCoverage = std::clamp(Sample->SupportMeters * 10, 0.0, 1.0);
					Material.WaterElevationMeters = Sample->Section.Water.ElevationMeters;
					Material.WaterSupport = 1;
				}
				const PlanetCore::SurfaceMaterialWeights Expected = PlanetCore::EvaluateSurfaceMaterial(D, Radius, 0, Material);
				const double WetError = std::abs(Actual[3] - Expected.Wetness);
				if(WetError > MaximumWetness) {
					MaximumWetness = WetError;
					WorstWetness = P;
				}
				MaximumLayer = std::max(
					{ MaximumLayer, std::abs(Actual[0] - Expected.Rock), std::abs(Actual[1] - Expected.Sand), std::abs(Actual[2] - Expected.Soil) }
				);
				if(Expected.Wetness < 0.01 && Actual[3] > 0.1) {
					++DryWetErrors;
					MaximumDrySpan = std::max(MaximumDrySpan, std::max({ (A - B).Length(), (B - C).Length(), (C - A).Length() }));
				}
				const double Increment = 2 * R0 * P.Z + P.Dot(P);
				const double Height = Mesh.Frame.AnchorElevationMeters + Increment / (std::sqrt(R0 * R0 + Increment) + R0);
				const double Marine = Smooth(-0.15, 0.1, -Raw[0]), River = std::clamp(Raw[2] * 10, 0.0, 1.0) * Smooth(-0.05, 0.15, Raw[1]);
				const double RawWet = 1 - (1 - Marine) * (1 - River), RawError = std::abs(RawWet - Expected.Wetness);
				if(RawError > MaximumRawWetness) {
					MaximumRawWetness = RawError;
					WorstRaw = {
						P.X,
						P.Y,
						P.Z,
						Raw[1],
						Raw[2],
						Sample->Section.HasWater ? Sample->Section.Water.ElevationMeters - Terrain.ElevationMeters : -1,
						Sample->SupportMeters,
						Height - Terrain.ElevationMeters
					};
				}
				PlanetCore::RiverSurfaceSample RasterMaterial = Material;
				RasterMaterial.Terrain.ElevationMeters = Height;
				const double RasterWet = PlanetCore::EvaluateSurfaceMaterial(D, Radius, 0, RasterMaterial).Wetness;
				const double OldRasterError = std::abs(Actual[3] - RasterWet), NewRasterError = std::abs(RawWet - RasterWet);
				MaxRasterOld = std::max(MaxRasterOld, OldRasterError);
				MaxRasterNew = std::max(MaxRasterNew, NewRasterError);
				SumRasterOld += OldRasterError;
				SumRasterNew += NewRasterError;
				SumWetness += WetError;
				SumRawWetness += RawError;
				if(Expected.Wetness < 0.01 && RawWet > 0.1) ++RawDryWetErrors;
				MaximumHeight = std::max(MaximumHeight, std::abs(Height - Terrain.ElevationMeters));
				++Samples;
			}
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Flow appearance: samples=%u normal=%.12g wetness=%.12g layer=%.12g height=%.12g dryWet=%u drySpan=%.12g ")
				TEXT("normalAt=(%.12g,%.12g,%.12g) wetAt=(%.12g,%.12g,%.12g)"),
				Samples,
				MaximumNormal,
				MaximumWetness,
				MaximumLayer,
				MaximumHeight,
				DryWetErrors,
				MaximumDrySpan,
				WorstNormal.X,
				WorstNormal.Y,
				WorstNormal.Z,
				WorstWetness.X,
				WorstWetness.Y,
				WorstWetness.Z
			)
		);
		Test.AddInfo(
			FString::Printf(
				TEXT("Flow raw-mask trial: sourceVertices=%u halfPrecision=true samples=%u maximum=%.12g ")
				TEXT("oldMean=%.12g newMean=%.12g oldDryWet=%u newDryWet=%u"),
				static_cast<unsigned>(Hydrology.size()),
				Samples,
				MaximumRawWetness,
				SumWetness / std::max(1U, Samples),
				SumRawWetness / std::max(1U, Samples),
				DryWetErrors,
				RawDryWetErrors
			)
		);
		Test.AddInfo(
			FString::Printf(
				TEXT("Flow raw-mask worst: point=(%.12g,%.12g,%.12g) rawDepth=%.12g rawSupport=%.12g ")
				TEXT("trueDepth=%.12g trueSupport=%.12g groundError=%.12g rasterOldMax=%.12g rasterNewMax=%.12g ")
				TEXT("rasterOldMean=%.12g rasterNewMean=%.12g"),
				WorstRaw[0],
				WorstRaw[1],
				WorstRaw[2],
				WorstRaw[3],
				WorstRaw[4],
				WorstRaw[5],
				WorstRaw[6],
				WorstRaw[7],
				MaxRasterOld,
				MaxRasterNew,
				SumRasterOld / std::max(1U, Samples),
				SumRasterNew / std::max(1U, Samples)
			)
		);
		// 現象の切分用であり，有限な値だけで自然な景観の完成を主張しない返戻
		return Test.TestTrue(TEXT("実地面の材質を千標本以上照合"), Samples > 1000) &&
		Test.TestTrue(TEXT("描画面の水深に対する被覆誤差が一割以内"), MaxRasterNew < 0.1) &&
		Test.TestTrue(TEXT("補間後の被覆判定で平均誤差が減少"), SumRasterNew < SumRasterOld);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowNetworkContracts,
	"PlanetGenerator.FlowChannel.Network",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ実地形の有限水系で，全流出口の準備と受入湖岸の共有を検証する
 * @param Parameters 未使用の自動試験引数
 * @return 全流出口・大洋通路・水位・所有権・拒否条件が成立した場合に真
 */
bool FPlanetFlowNetworkContracts::RunTest(const FString &Parameters) {
	// 不変な元場を所有して，準備関数の終了後にも安全な地点評価を保持
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実造山場を準備"), Ridges.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 海域の元場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実海域を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	const auto SourceCalls = std::make_shared<std::array<std::uint64_t, 2>>();
	const auto Source = [Ocean, SourceCalls](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		++(*SourceCalls)[0];
		// 計測した元場の値と勾配の返戻
		return Ocean->Evaluate(D);
	};
	const auto Height = [Ocean, SourceCalls](const PlanetCore::UnitDirection D) -> double {
		++(*SourceCalls)[1];
		// 計測した元標高の返戻
		return Ocean->EvaluateElevation(D);
	};
	const std::optional<PlanetCore::SphereMesh> Sphere = PlanetCore::BuildUniformSphere(6);
	// 共通の海域標本を得られない場合の返戻
	if(!TestTrue(TEXT("海域標本を準備"), Sphere.has_value())) return false;
	std::vector<PlanetCore::UnitDirection> Anchors;
	std::set<std::tuple<double, double, double>> Keys;
	for(const PlanetCore::UnitDirection D : Sphere->Directions) {
		if(!(Height(D) < -1)) continue;
		Anchors.push_back(D);
		const PlanetCore::Vec3 P = D.Vector();
		Keys.emplace(P.X, P.Y, P.Z);
	}
	const auto IsOcean = [&](const PlanetCore::UnitDirection D) -> bool {
		const PlanetCore::Vec3 P = D.Vector();
		// 元場の接続済海域で確認した標本への返戻
		return Ocean->Statistics().ComponentsAfter == 1 && Keys.contains({ P.X, P.Y, P.Z });
	};
	const PlanetCore::UnitDirection Pass =
	*PlanetCore::UnitDirection::FromVector({ 0.54180127453542282, 0.81192128360203597, -0.21733662403243018 });
	PlanetCore::SpillNetworkDiagnostics Discovery;
	const std::optional<PlanetCore::SpillNetwork> Network =
	PlanetCore::PrepareSpillNetwork(Pass, Source, Anchors, IsOcean, {}, {}, &Discovery);
	AddInfo(
		FString::Printf(
			TEXT("Flow network discovery accepted=%u failure=%u steps=%u evaluations=%llu"),
			static_cast<unsigned>(Network.has_value()),
			static_cast<unsigned>(Discovery.Failure),
			Discovery.Steps,
			static_cast<unsigned long long>(Discovery.Evaluations)
		)
	);
	// 有限網の探索失敗を河道の合格としない返戻
	if(!TestTrue(TEXT("有限な実水系を準備"), Network.has_value())) return false;
	PlanetCore::FlowNetworkSettings Settings;
	PlanetCore::FlowNetworkDiagnostics Report;
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(
		TEXT("中断済では元場の準備を開始しない"),
		PlanetCore::PrepareFlowNetwork(*Network, Source, Height, Settings, Cancel.get_token(), &Report).has_value()
	);
	TestTrue(TEXT("中断理由を保持"), Report.Failure == PlanetCore::FlowNetworkFailure::Cancelled);
	Settings.MaximumChannels = 1;
	TestFalse(
		TEXT("保持上限を超える部分水系を返さない"),
		PlanetCore::PrepareFlowNetwork(*Network, Source, Height, Settings, {}, &Report).has_value()
	);
	TestTrue(TEXT("全流出口の保持上限を識別"), Report.Failure == PlanetCore::FlowNetworkFailure::ChannelBudget);
	Settings.MaximumChannels = 64;
	Settings.MaximumGeometrySamples = 0;
	std::uint64_t InvalidCalls = 0;
	const auto CountedSource = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
		++InvalidCalls;
		// 不正な保持量の時に呼出された実回数を確認する元場の返戻
		return Source(Direction);
	};
	TestFalse(
		TEXT("零の合算保持量を元場の準備前に拒否"),
		PlanetCore::PrepareFlowNetwork(*Network, CountedSource, Height, Settings, {}, &Report).has_value()
	);
	TestTrue(TEXT("不正な合算保持量の理由"), Report.Failure == PlanetCore::FlowNetworkFailure::InvalidInput);
	TestEqual(TEXT("不正な合算保持量で元場を呼出さない"), InvalidCalls, UINT64_C(0));
	Settings.MaximumGeometrySamples = 1;
	TestFalse(
		TEXT("実形状の合算上限を超える部分水系を返さない"),
		PlanetCore::PrepareFlowNetwork(*Network, Source, Height, Settings, {}, &Report).has_value()
	);
	TestTrue(TEXT("実形状の合算上限を識別"), Report.Failure == PlanetCore::FlowNetworkFailure::GeometryBudget);
	TestEqual(TEXT("拒否した形状を保持量へ加算しない"), Report.RetainedGeometrySamples, 0U);
	Settings.MaximumGeometrySamples = 262144;
	const double Started = FPlatformTime::Seconds();
	std::optional<PlanetCore::PreparedFlowNetwork> Prepared =
	PlanetCore::PrepareFlowNetwork(*Network, Source, Height, Settings, {}, &Report);
	AddInfo(
		FString::Printf(
			TEXT("Flow network channels accepted=%u failure=%u basin=%u prepared=%u coastal=%u ")
			TEXT("channel=%u outlet=%u upstream=%u downstream=%u shore=%u shoreEnd=%u seconds=%.12g"),
			static_cast<unsigned>(Prepared.has_value()),
			static_cast<unsigned>(Report.Failure),
			Report.Basin,
			Report.PreparedChannels,
			Report.CoastalChannels,
			static_cast<unsigned>(Report.Channel.Failure),
			static_cast<unsigned>(Report.Channel.Outlet.Failure),
			static_cast<unsigned>(Report.Channel.Outlet.UpstreamEnd),
			static_cast<unsigned>(Report.Channel.Outlet.DownstreamEnd),
			static_cast<unsigned>(Report.Channel.Shore.Failure),
			static_cast<unsigned>(Report.Channel.Shore.End),
			FPlatformTime::Seconds() - Started
		)
	);
	if(!Prepared && Report.Basin < Network->Graph.Hierarchy.Nodes.size()) {
		// 拒否した実流出口と両下降枝を，元の有限網の情報へ照合
		const PlanetCore::DepressionHierarchyNode &Node = Network->Graph.Hierarchy.Nodes[Report.Basin];
		const std::optional<PlanetCore::SpillNetworkOutlet> Outlet = PlanetCore::GetSpillNetworkOutlet(*Network, Node.OutletSite);
		if(Outlet) {
			const PlanetCore::SpillConnection &Connection = Network->Connections[Outlet->Connection];
			const PlanetCore::Vec3 D = Connection.Pass.Direction.Vector();
			AddInfo(
				FString::Printf(
					TEXT("Flow rejected outlet basin=%u receiver=%u connection=%u positive=%u W=%.17g receivingW=%.17g ")
					TEXT("axisDot=%.17g pass=(%.17g,%.17g,%.17g)"),
					Outlet->Basin,
					Outlet->ReceivingBasin,
					Outlet->Connection,
					static_cast<unsigned>(Outlet->IsPositiveDownstream),
					Outlet->UpstreamWaterMeters,
					Outlet->DownstreamWaterMeters,
					Connection.Pass.MinimumCurvatureDirection.Dot(Report.Channel.Outlet.DownstreamAxis),
					D.X,
					D.Y,
					D.Z
				)
			);
			for(std::size_t Side = 0; Side < 2; ++Side) {
				const PlanetCore::SpillConnectionBranch &Branch = Connection.Branches[Side];
				AddInfo(
					FString::Printf(
						TEXT("Flow rejected branch side=%u kind=%u end=%u terminalH=%.17g minimumH=%.17g samples=%u"),
						static_cast<unsigned>(Side),
						static_cast<unsigned>(Branch.Kind),
						static_cast<unsigned>(Branch.Path.End),
						Branch.Path.Samples.back().ElevationMeters,
						Branch.Minimum ? Branch.Minimum->ElevationMeters : 0,
						static_cast<unsigned>(Branch.Path.Samples.size())
					)
				);
			}
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Flow network shore length=%.12g vertices=%u heightResidual=%.12g returnGap=%.12g"),
			Report.Channel.Shore.LengthMeters,
			Report.Channel.Shore.Vertices,
			Report.Channel.Shore.HeightResidualMeters,
			Report.Channel.Shore.FirstGapMeters
		)
	);
	// 準備出来ない流出口を無視して合格しない返戻
	AddInfo(
		FString::Printf(
			TEXT("Network water preparation: failure=%u knots=%u passes=%u source=%u heightError=%.12g slopeError=%.12g"),
			static_cast<unsigned>(Report.Channel.Water.Failure),
			Report.Channel.Water.Knots,
			Report.Channel.Water.Passes,
			Report.Channel.Water.Evaluations,
			Report.Channel.Water.HeightErrorMeters,
			Report.Channel.Water.SlopeError
		)
	);
	// 水位近似を含む全河道の失敗を保持する返戻
	if(!TestTrue(TEXT("全流出口の実河道と共通湖岸を準備"), Prepared.has_value())) return false;
	// 河道の準備と異なる元場を，合成時に再指定する入口を持たせない
	PlanetCore::PreparedFlowNetwork MissingSource = *Prepared;
	MissingSource.Source = {};
	TestFalse(TEXT("元場の所有権を失った有限網を拒否"), PlanetCore::FlowNetworkField::Create(MissingSource).has_value());
	MissingSource = {};
	std::optional<PlanetCore::FlowNetworkField> Combined = PlanetCore::FlowNetworkField::Create(*Prepared);
	// 単一の場を作れない結果を複数描画で代用しない返戻
	if(!TestTrue(TEXT("全河道を一つの地点評価へ統合"), Combined.has_value())) return false;
	const std::shared_ptr<const PlanetCore::FlowNetworkField> Field =
	std::make_shared<const PlanetCore::FlowNetworkField>(std::move(*Combined));
	const std::unique_ptr<PlanetCore::FlowWaterView> WaterView = PlanetCore::FlowWaterView::Create(Field);
	// 全河道の発見用索引を準備出来ない場合の返戻
	if(!TestTrue(TEXT("同じ水系の全河道と岸を索引化"), static_cast<bool>(WaterView))) return false;
	double MaximumCompositionError = 0, MaximumGradientError = 0, MaximumAdditionalCut = 0, MaximumCombinedWaterError = 0;
	std::uint32_t GradientSamples = 0;
	std::set<std::uint32_t> Expected;
	for(const std::uint32_t Root : Network->Graph.Hierarchy.SiteRoots) if(Root) Expected.insert(Root);
	TestEqual(TEXT("全包含根を一度ずつ保持"), Prepared->Channels.size(), Expected.size());
	std::map<std::uint32_t, std::shared_ptr<const PlanetCore::FlowChannelField>> Fields;
	double MaximumRise = 0, MaximumPositiveSlope = 0, MinimumDepth = std::numeric_limits<double>::infinity();
	double MaximumShoreLength = 0, MaximumEndpointWaterError = 0, MaximumProfileHeightError = 0, MaximumProfileSlopeError = 0;
	std::uint32_t ProfileSamples = 0;
	for(const PlanetCore::FlowBasinChannel &Entry : Prepared->Channels) {
		const PlanetCore::FlowWaterGeometry Geometry = Entry.Channel.Field->GetWaterGeometry();
		TestTrue(TEXT("上流から受入水域へ水位が下がる"), Entry.Outlet.DownstreamWaterMeters < Entry.Outlet.UpstreamWaterMeters);
		if(Entry.Outlet.ReceivingBasin) {
			const std::map<std::uint32_t, std::shared_ptr<const PlanetCore::FlowChannelField>>::const_iterator Found =
			Fields.find(Entry.Outlet.ReceivingBasin);
			// 大きさが正しくても下流が先に存在しない場合の返戻
			if(!TestTrue(TEXT("受入側を先に準備"), Found != Fields.end())) return false;
			TestTrue(TEXT("受入湖の岸を同じ所有定義で共有"), Geometry.Receiving == &Found->second->GetWaterGeometry().Upstream);
		}
		MaximumShoreLength = std::max(MaximumShoreLength, Geometry.Upstream.GetContour().Samples.back().DistanceMeters);
		double PreviousWater = Entry.Outlet.UpstreamWaterMeters;
		for(unsigned Index = 0; Index < 1025; ++Index) {
			const double Distance = Entry.Channel.Route->GetLengthMeters() * Index / 1024;
			const PlanetCore::TerrainFlowCurveSample Point = *Entry.Channel.Route->Sample(Distance);
			const std::uint64_t BeforeWaterQuery = (*SourceCalls)[0];
			const std::optional<PlanetCore::FlowChannelSection> Value = Entry.Channel.Field->Evaluate(Point.Direction);
			TestEqual(TEXT("準備後の水位は中心の元場を再照会しない"), (*SourceCalls)[0] - BeforeWaterQuery, std::uint64_t{ 1 });
			// 経路内の水切れや未定義値を標本数から除外しない返戻
			if(!TestTrue(TEXT("全河道中心で地面と水位を評価"), Value && Value->HasWater)) return false;
			const double Rise = Value->Water.ElevationMeters - PreviousWater;
			const double Slope = Value->Water.TangentialGradientPerRadian.Dot(Point.DirectionDerivativePerMeter);
			const PlanetCore::TerrainSample OriginalSource = Source(Point.Direction);
			const PlanetCore::FlowChannelWaterLevel OriginalLevel {
				OriginalSource.ElevationMeters,
				OriginalSource.TangentialGradientPerRadian.Dot(Point.DirectionDerivativePerMeter)
			};
			PlanetCore::FlowChannelWaterSettings WaterSettings;
			WaterSettings.DownstreamWaterMeters = Entry.Outlet.DownstreamWaterMeters;
			const std::optional<PlanetCore::FlowChannelWaterLevel> Exact = PlanetCore::SampleFlowOutletWaterLevel(
				OriginalLevel,
				Distance - Entry.Channel.PassDistanceMeters,
				Entry.Outlet.UpstreamWaterMeters,
				WaterSettings
			);
			// 元の水位式を独立な参照として評価出来ない場合の返戻
			if(!TestTrue(TEXT("元の実水位を直接評価"), Exact.has_value())) return false;
			MaximumProfileHeightError =
			std::max(MaximumProfileHeightError, std::abs(Value->Water.ElevationMeters - Exact->ElevationMeters));
			MaximumProfileSlopeError = std::max(MaximumProfileSlopeError, std::abs(Slope - Exact->DerivativePerMeter));
			MaximumRise = std::max(MaximumRise, Rise);
			MaximumPositiveSlope = std::max(MaximumPositiveSlope, Slope);
			MinimumDepth = std::min(MinimumDepth, Value->Water.ElevationMeters - Value->Terrain.ElevationMeters);
			PreviousWater = Value->Water.ElevationMeters;
			if(Index == 1024) {
				MaximumEndpointWaterError =
				std::max(MaximumEndpointWaterError, std::abs(Value->Water.ElevationMeters - Entry.Outlet.DownstreamWaterMeters));
			}
			++ProfileSamples;
			const std::optional<PlanetCore::FlowChannelSection> Merged = Field->Evaluate(Point.Direction);
			// 合成した地点評価の欠落への返戻
			if(!TestTrue(TEXT("全河道中心で合成場を評価"), Merged && Merged->HasWater)) return false;
			// 同じ地面だけの評価では，河道中心の水位用照会を省く
			for(const bool NeedsGradient : { false, true }) {
				const std::array<std::uint64_t, 2> BeforeGround = *SourceCalls;
				const std::optional<PlanetCore::FlowChannelSection> Ground = Field->Evaluate(Point.Direction, false, NeedsGradient);
				// 地面だけの正規経路を失う場合の返戻
				if(!TestTrue(TEXT("全河道で水位を要求しない地形を評価"), Ground.has_value())) return false;
				TestEqual(TEXT("地面の元場照会は要求された属性で１回"), (*SourceCalls)[NeedsGradient ? 0 : 1] - BeforeGround[NeedsGradient ? 0 : 1], UINT64_C(1));
				TestEqual(TEXT("未要求の元場属性は計算しない"), (*SourceCalls)[NeedsGradient ? 1 : 0] - BeforeGround[NeedsGradient ? 1 : 0], UINT64_C(0));
				TestEqual(TEXT("水位無しでも同じ標高"), Ground->Terrain.ElevationMeters, Merged->Terrain.ElevationMeters);
				TestEqual(TEXT("水位無しでも同じ開削被覆"), Ground->BedCoverage, Merged->BedCoverage);
				TestFalse(TEXT("未要求の水位を有効としない"), Ground->HasWater);
				if(NeedsGradient) {
					const PlanetCore::Vec3 Error = Ground->Terrain.TangentialGradientPerRadian - Merged->Terrain.TangentialGradientPerRadian;
					TestEqual(TEXT("水位無しでも同じ解析勾配"), Error.Length(), 0.0);
				}
			}
			// 水位の選択と所属の一括評価は，従来の独立した照会に一致
			const std::optional<double> SeparateSupport = Field->WaterSupport(Point.Direction);
			for(const bool NeedsGradient : { false, true }) {
				const std::optional<PlanetCore::FlowWaterSample> Joined = Field->EvaluateWithSupport(Point.Direction, NeedsGradient);
				// どちらの照会の欠落も成功として数えない返戻
				if(!TestTrue(TEXT("全河道中心で一括水域を評価"), Joined && SeparateSupport)) return false;
				TestEqual(TEXT("一括水域も同じ所属"), Joined->SupportMeters, *SeparateSupport);
				TestEqual(TEXT("一括水域も同じ河床"), Joined->Section.Terrain.ElevationMeters, Merged->Terrain.ElevationMeters);
				TestEqual(TEXT("一括水域も同じ水位"), Joined->Section.Water.ElevationMeters, Merged->Water.ElevationMeters);
				TestEqual(TEXT("一括水域も同じ開削被覆"), Joined->Section.BedCoverage, Merged->BedCoverage);
				const PlanetCore::Vec3 WaterError =
				Joined->Section.Water.TangentialGradientPerRadian - Merged->Water.TangentialGradientPerRadian;
				TestEqual(TEXT("地面の微分を省いても合流水面の微分を保持"), WaterError.Length(), 0.0);
				if(NeedsGradient) {
					const PlanetCore::Vec3 Error =
					Joined->Section.Terrain.TangentialGradientPerRadian - Merged->Terrain.TangentialGradientPerRadian;
					TestEqual(TEXT("一括水域も同じ河床微分"), Error.Length(), 0.0);
				}
			}
			const double BaseHeight = Height(Point.Direction);
			double SquaredCuts = 0, MaximumCut = 0;
			for(const PlanetCore::FlowBasinChannel &Other : Prepared->Channels) {
				const std::optional<PlanetCore::FlowChannelSection> Part = Other.Channel.Field->Evaluate(Point.Direction, false, false);
				// 包囲箱を使わない独立な全河道評価を確認する返戻
				if(!TestTrue(TEXT("合成の独立参照を評価"), Part.has_value())) return false;
				const double Cut = BaseHeight - Part->Terrain.ElevationMeters;
				SquaredCuts += Cut * Cut;
				MaximumCut = std::max(MaximumCut, Cut);
			}
			MaximumCompositionError =
			std::max(MaximumCompositionError, std::abs(Merged->Terrain.ElevationMeters - (BaseHeight - std::sqrt(SquaredCuts))));
			MaximumAdditionalCut = std::max(MaximumAdditionalCut, BaseHeight - Merged->Terrain.ElevationMeters - MaximumCut);
			MaximumCombinedWaterError =
			std::max(MaximumCombinedWaterError, std::abs(Merged->Water.ElevationMeters - Value->Water.ElevationMeters));
			if(Index % 16) continue;
			const PlanetCore::Vec3 Tangent = PlanetCore::UnitDirection::FromVector(Point.DirectionDerivativePerMeter)->Vector();
			for(const PlanetCore::Vec3 Axis : { Tangent, Point.Direction.Vector().Cross(Tangent) }) {
				constexpr double Step = 0.001;
				const double Angle = Step / Definition.RadiusMeters;
				const PlanetCore::UnitDirection Lower =
				*PlanetCore::UnitDirection::FromVector(Point.Direction.Vector() * std::cos(Angle) - Axis * std::sin(Angle));
				const PlanetCore::UnitDirection Upper =
				*PlanetCore::UnitDirection::FromVector(Point.Direction.Vector() * std::cos(Angle) + Axis * std::sin(Angle));
				const std::optional<PlanetCore::FlowChannelSection> A = Field->Evaluate(Lower, false, false);
				const std::optional<PlanetCore::FlowChannelSection> B = Field->Evaluate(Upper, false, false);
				// 独立な差分の欠落への返戻
				if(!TestTrue(TEXT("合成場の独立差分を評価"), A && B)) return false;
				const double Difference = (B->Terrain.ElevationMeters - A->Terrain.ElevationMeters) / (2 * Step);
				const double Analytic = Merged->Terrain.TangentialGradientPerRadian.Dot(Axis) / Definition.RadiusMeters;
				MaximumGradientError = std::max(MaximumGradientError, std::abs(Difference - Analytic));
				++GradientSamples;
			}
		}
		Fields.emplace(Entry.Outlet.Basin, Entry.Channel.Field);
		Expected.erase(Entry.Outlet.Basin);
	}
	TestTrue(TEXT("全包含根に実河道が存在"), Expected.empty());
	// 河道以外の海陸も独立な全球標本で照合し，未定義水面を増やさない
	std::uint32_t GlobalSamples = 0;
	for(std::size_t I = 0; I < Sphere->Directions.size(); I += 127) {
		const PlanetCore::UnitDirection D = Sphere->Directions[I];
		const std::optional<PlanetCore::FlowChannelSection> Reference = Field->Evaluate(D);
		const std::optional<double> Support = Field->WaterSupport(D);
		const std::optional<PlanetCore::FlowWaterSample> Sample = Field->EvaluateWithSupport(D, false);
		// 乾いた照会の欠落も検査から除外しない返戻
		if(!TestTrue(TEXT("全球の一括水域照会が存在"), Reference && Support && Sample)) return false;
		TestEqual(TEXT("全球の一括所属は独立照会と一致"), Sample->SupportMeters, *Support);
		TestEqual(TEXT("全球の一括河床を保持"), Sample->Section.Terrain.ElevationMeters, Reference->Terrain.ElevationMeters);
		TestEqual(TEXT("全球の一括水位を保持"), Sample->Section.Water.ElevationMeters, Reference->Water.ElevationMeters);
		TestEqual(TEXT("全球の水面の有無を保持"), Sample->Section.HasWater, Reference->HasWater);
		const PlanetCore::Vec3 Error = Sample->Section.Water.TangentialGradientPerRadian - Reference->Water.TangentialGradientPerRadian;
		TestEqual(TEXT("全球の水面微分を保持"), Error.Length(), 0.0);
		++GlobalSamples;
	}
	AddInfo(
		FString::Printf(TEXT("Flow coupled queries: river=%u global=%u exactAttributes=true"), ProfileSamples * 2, GlobalSamples)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow network combined heightError=%.12g gradientSamples=%u gradientError=%.12g ")
			TEXT("additionalCut=%.12g waterError=%.12g guides=%llu"),
			MaximumCompositionError,
			GradientSamples,
			MaximumGradientError,
			MaximumAdditionalCut,
			MaximumCombinedWaterError,
			static_cast<unsigned long long>(WaterView->GuideCount())
		)
	);
	TestTrue(TEXT("包囲箱付き合成が全河道評価と一致"), MaximumCompositionError < 1E-9);
	TestTrue(TEXT("合成地形の解析勾配が独立差分と一致"), MaximumGradientError < 0.0005);
	TestTrue(TEXT("合成後も各河道中心の水位を保持"), MaximumCombinedWaterError < 1E-6);
	// 同じ地点で複数河道が水面を持つ時，別々の高さを重ねて隠さない
	double MaximumOverlapWaterError = 0;
	std::uint32_t WetOverlaps = 0;
	for(std::size_t I = 0; I < Prepared->Channels.size(); ++I) {
		const PlanetCore::PreparedFlowChannel &Channel = Prepared->Channels[I].Channel;
		for(unsigned Index = 0; Index < 257; ++Index) {
			const PlanetCore::TerrainFlowCurveSample Point = *Channel.Route->Sample(Channel.Route->GetLengthMeters() * Index / 256);
			const PlanetCore::Vec3 Along = PlanetCore::UnitDirection::FromVector(Point.DirectionDerivativePerMeter)->Vector();
			const PlanetCore::Vec3 Across = Point.Direction.Vector().Cross(Along);
			for(const double Offset : { -4.0, 0.0, 4.0 }) {
				const double Angle = Offset / Definition.RadiusMeters;
				const PlanetCore::UnitDirection D =
				*PlanetCore::UnitDirection::FromVector(Point.Direction.Vector() * std::cos(Angle) + Across * std::sin(Angle));
				const std::optional<PlanetCore::FlowChannelSection> A = Channel.Field->Evaluate(D);
				const std::optional<double> SupportA = Channel.Field->WaterSupport(D);
				if(!A || !SupportA || *SupportA < 0 || !A->HasWater || !(A->Water.ElevationMeters > A->Terrain.ElevationMeters)) continue;
				for(std::size_t J = 0; J < Prepared->Channels.size(); ++J) {
					if(I == J) continue;
					const PlanetCore::FlowChannelField &Other = *Prepared->Channels[J].Channel.Field;
					const std::optional<PlanetCore::FlowChannelSection> B = Other.Evaluate(D);
					const std::optional<double> SupportB = Other.WaterSupport(D);
					if(!B || !SupportB || *SupportB < 0 || !B->HasWater || !(B->Water.ElevationMeters > B->Terrain.ElevationMeters)) continue;
					MaximumOverlapWaterError = std::max(MaximumOverlapWaterError, std::abs(A->Water.ElevationMeters - B->Water.ElevationMeters));
					++WetOverlaps;
				}
			}
		}
	}
	AddInfo(
		FString::Printf(TEXT("Flow network water overlap samples=%u maxWaterError=%.12g"), WetOverlaps, MaximumOverlapWaterError)
	);
	TestTrue(TEXT("共有する受入水面の標本が存在"), WetOverlaps > 0);
	TestTrue(TEXT("異なる高さの水面を重ねない"), MaximumOverlapWaterError < 1E-6);
	AddInfo(
		FString::Printf(
			TEXT("Flow network profiles channels=%u samples=%u maxRise=%.12g maxPositiveSlope=%.12g ")
			TEXT("minDepth=%.12g endpointWaterError=%.12g maxShoreLength=%.12g"),
			static_cast<unsigned>(Prepared->Channels.size()),
			ProfileSamples,
			MaximumRise,
			MaximumPositiveSlope,
			MinimumDepth,
			MaximumEndpointWaterError,
			MaximumShoreLength
		)
	);
	TestTrue(TEXT("全河道の途中で水位が逆転しない"), MaximumRise < 1E-8 && MaximumPositiveSlope < 1E-8);
	TestTrue(TEXT("全河道中心に正の水深が有る"), MinimumDepth > 0);
	TestTrue(TEXT("各終端で受入水位へ一致"), MaximumEndpointWaterError < 1E-8);
	AddInfo(
		FString::Printf(
			TEXT("Prepared real water: samples=%u heightError=%.12g slopeError=%.12g"),
			ProfileSamples,
			MaximumProfileHeightError,
			MaximumProfileSlopeError
		)
	);
	TestTrue(TEXT("実水位近似の独立な高さ誤差"), MaximumProfileHeightError < 1E-5);
	TestTrue(TEXT("実水位近似の独立な微分誤差"), MaximumProfileSlopeError < 1E-6);
	// 単独河道と同じ生成経路を，水系全体の不変場で実行
	const PlanetCore::TerrainFlowRoute &ObservedRoute = *Prepared->Channels.back().Channel.Route;
	const PlanetCore::UnitDirection Anchor =
	ObservedRoute.Sample(std::min(1500.0, ObservedRoute.GetLengthMeters() * 0.5))->Direction;
	const std::optional<PlanetCore::FlowChannelSection> AnchorValue = Field->Evaluate(Anchor);
	// 表示位置を構成出来ない場合の返戻
	if(!TestTrue(TEXT("合成場の表示地点を評価"), AnchorValue.has_value())) return false;
	PlanetCore::LodView View;
	View.RadiusMeters = Definition.RadiusMeters;
	View.ObserverMeters = Anchor.Vector() * (View.RadiusMeters + AnchorValue->Terrain.ElevationMeters + 1.8);
	View.SurfaceObserverMeters = Anchor.Vector() * (View.RadiusMeters + AnchorValue->Terrain.ElevationMeters + 1);
	View.SurfaceRadiusMeters = 24;
	View.LeafBudget = 32768;
	View.PixelLeafBudget = 2048;
	View.TargetEdgePixels = 64;
	const std::unique_ptr<PlanetCore::FlowTerrainView> TerrainBuilder = PlanetCore::FlowTerrainView::Create(Field);
	PlanetCore::FlowViewDiagnostics GroundReport;
	const double RenderStarted = FPlatformTime::Seconds();
	std::optional<PlanetCore::FlowTerrainMesh> Terrain = TerrainBuilder->Build(View, {}, &GroundReport);
	AddInfo(
		FString::Printf(
			TEXT("Flow network ground accepted=%u failure=%u leaves=%llu error=%.12g"),
			static_cast<unsigned>(Terrain.has_value()),
			static_cast<unsigned>(GroundReport.Failure),
			static_cast<unsigned long long>(GroundReport.Leaves),
			GroundReport.MaximumSurfaceErrorMeters
		)
	);
	// 別の地面を重ねて近景の失敗を隠さない返戻
	if(!TestTrue(TEXT("合成地形の同じ全球分割を準備"), Terrain.has_value())) return false;
	PlanetCore::WaterRefinementDiagnostics WaterReport;
	const std::array<std::uint64_t, 2> BeforeWaterCalls = *SourceCalls;
	const std::optional<PlanetCore::FlowWaterMesh> Water = WaterView->Build(*Terrain, View, WaterReport, {}, &GroundReport);
	const PlanetCore::FlowWaterReuseStatistics Costs = WaterView->GetReuseStatistics();
	AddInfo(
		FString::Printf(
			TEXT("Flow network costs: water=%.9g discovery=%.9g bank=%.9g normals=%.9g ground=%.9g ms ")
			TEXT("source=%llu height=%llu waterSamples=%llu supports=%llu"),
			Costs.StageMilliseconds[0],
			Costs.StageMilliseconds[1],
			Costs.StageMilliseconds[2],
			Costs.StageMilliseconds[3],
			Costs.StageMilliseconds[4],
			static_cast<unsigned long long>((*SourceCalls)[0] - BeforeWaterCalls[0]),
			static_cast<unsigned long long>((*SourceCalls)[1] - BeforeWaterCalls[1]),
			static_cast<unsigned long long>(Costs.WaterSampleEvaluations),
			static_cast<unsigned long long>(Costs.WaterSupportEvaluations)
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow network water accepted=%u pass=%u failed=%llu feature=%llu source=%llu waterError=%.12g ")
			TEXT("shoreError=%.12g edgePixels=%.12g groundFailure=%u seconds=%.12g"),
			static_cast<unsigned>(Water.has_value()),
			WaterReport.Pass,
			static_cast<unsigned long long>(WaterReport.FailedTriangles),
			static_cast<unsigned long long>(WaterReport.FeatureTriangles),
			static_cast<unsigned long long>(WaterReport.SourceTriangles),
			WaterReport.WaterErrorMeters,
			WaterReport.ShoreErrorMeters,
			WaterReport.MaximumShoreEdgePixels,
			static_cast<unsigned>(GroundReport.Failure),
			FPlatformTime::Seconds() - RenderStarted
		)
	);
	// 水域の部分欠落を成功と扱わない返戻
	if(!TestTrue(TEXT("全水系を共有地面と水面へ変換"), Water.has_value())) return false;
	TestTrue(TEXT("水系の水面が非空"), !Water->Surface.Triangles.empty());
	TestEqual(TEXT("同じ水面頂点と法線"), Water->Surface.Vertices.size(), Water->Normals.size());
	// 身体が移動して局所原点が変わる場合の再利用費用を，同じ生成経路で計測
	const PlanetCore::UnitDirection MovedAnchor =
	*PlanetCore::UnitDirection::FromVector(Anchor.Vector() + Terrain->Frame.East * (6.5 / View.RadiusMeters));
	const std::optional<PlanetCore::FlowChannelSection> MovedValue = Field->Evaluate(MovedAnchor);
	// 移動先の失敗を古い面での成功にしない返戻
	if(!TestTrue(TEXT("移動先の同じ水系を評価"), MovedValue.has_value())) return false;
	PlanetCore::LodView MovedView = View;
	MovedView.ObserverMeters = MovedAnchor.Vector() * (View.RadiusMeters + MovedValue->Terrain.ElevationMeters + 1.8);
	MovedView.SurfaceObserverMeters = MovedAnchor.Vector() * (View.RadiusMeters + MovedValue->Terrain.ElevationMeters + 1);
	std::optional<PlanetCore::FlowTerrainMesh> MovedTerrain = TerrainBuilder->Build(MovedView, {}, &GroundReport);
	// 新しい身体の近景を準備出来ない場合の返戻
	if(!TestTrue(TEXT("六・五メートル移動した近景を準備"), MovedTerrain.has_value())) return false;
	const std::array<std::uint64_t, 2> BeforeMovedCalls = *SourceCalls;
	const double MovedStarted = FPlatformTime::Seconds();
	const std::optional<PlanetCore::FlowWaterMesh> MovedWater =
	WaterView->Build(*MovedTerrain, MovedView, WaterReport, {}, &GroundReport);
	const double MovedMilliseconds = (FPlatformTime::Seconds() - MovedStarted) * 1000;
	const PlanetCore::FlowWaterReuseStatistics MovedCosts = WaterView->GetReuseStatistics();
	AddInfo(
		FString::Printf(
			TEXT("Flow network moved costs: total=%.9g water=%.9g discovery=%.9g bank=%.9g normals=%.9g ground=%.9g ms ")
			TEXT("source=%llu height=%llu waterSamples=%llu supports=%llu cacheHits=%llu bankChecks=%llu bankHits=%llu"),
			MovedMilliseconds,
			MovedCosts.StageMilliseconds[0] - Costs.StageMilliseconds[0],
			MovedCosts.StageMilliseconds[1] - Costs.StageMilliseconds[1],
			MovedCosts.StageMilliseconds[2] - Costs.StageMilliseconds[2],
			MovedCosts.StageMilliseconds[3] - Costs.StageMilliseconds[3],
			MovedCosts.StageMilliseconds[4] - Costs.StageMilliseconds[4],
			static_cast<unsigned long long>((*SourceCalls)[0] - BeforeMovedCalls[0]),
			static_cast<unsigned long long>((*SourceCalls)[1] - BeforeMovedCalls[1]),
			static_cast<unsigned long long>(MovedCosts.WaterSampleEvaluations - Costs.WaterSampleEvaluations),
			static_cast<unsigned long long>(MovedCosts.WaterSupportEvaluations - Costs.WaterSupportEvaluations),
			static_cast<unsigned long long>(MovedCosts.WaterSampleHits - Costs.WaterSampleHits),
			static_cast<unsigned long long>(MovedCosts.BankProbeEvaluations - Costs.BankProbeEvaluations),
			static_cast<unsigned long long>(MovedCosts.BankProbeHits - Costs.BankProbeHits)
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Flow bank retention: initial=%llu moved=%llu evictions=%llu movedEvictions=%llu"),
			static_cast<unsigned long long>(Costs.BankChecks),
			static_cast<unsigned long long>(MovedCosts.BankChecks),
			static_cast<unsigned long long>(MovedCosts.BankCheckEvictions),
			static_cast<unsigned long long>(MovedCosts.BankCheckEvictions - Costs.BankCheckEvictions)
		)
	);
	TestTrue(TEXT("移動後も河岸の保持数が有限の予算内"), MovedCosts.BankChecks < 131073);
	// 水位の再検査を省かず，同じ精度で新しい座標へ公開する
	if(!TestTrue(TEXT("移動後も同じ地面と水面を準備"), MovedWater.has_value())) return false;
	TestTrue(TEXT("移動後も非空の水面"), !MovedWater->Surface.Vertices.empty());
	double MaximumNormalError = 0;
	for(
		std::size_t I = 0;
		I < MovedWater->Surface.Vertices.size();
		I += std::max<std::size_t>(1, MovedWater->Surface.Vertices.size() / 128)
	) {
		const PlanetCore::UnitDirection D =
		*PlanetCore::UnitDirection::FromVector(MovedWater->Frame.LocalToPlanet(MovedWater->Surface.Vertices[I].Position));
		const std::optional<PlanetCore::FlowChannelSection> ReferenceSample = Field->Evaluate(D, true, false);
		// 実際の表示地点を独立に評価出来ない場合の返戻
		if(!TestTrue(TEXT("移動後の実水面頂点を独立評価"), ReferenceSample && ReferenceSample->HasWater)) return false;
		const PlanetCore::UnitDirection Normal = *PlanetCore::UnitDirection::FromVector(
			D.Vector() * (View.RadiusMeters + ReferenceSample->Water.ElevationMeters) - ReferenceSample->Water.TangentialGradientPerRadian
		);
		MaximumNormalError =
		std::max(MaximumNormalError, (MovedWater->Normals[I] - MovedWater->Frame.VectorToLocal(Normal.Vector())).Length());
	}
	TestTrue(TEXT("原点変更後も独立な解析法線と一致"), MaximumNormalError < 1E-10);
	AddInfo(
		FString::Printf(
			TEXT("Flow moved normal error=%.12g vertices=%llu"),
			MaximumNormalError,
			static_cast<unsigned long long>(MovedWater->Surface.Vertices.size())
		)
	);
	// 実画面で角張って見えた乾いた岸から，身体を保持して俯瞰の共通面を生成
	const PlanetCore::UnitDirection Dry =
	*PlanetCore::UnitDirection::FromVector({ 0.54178538420982891, 0.81195543427398209, -0.21724863684165494 });
	const std::optional<PlanetCore::FlowChannelSection> DrySample = Field->Evaluate(Dry, false, false);
	// 実画面と同じ地面の欠落を別地点で代用しない返戻
	if(!TestTrue(TEXT("景観比較の乾いた岸が存在"), DrySample.has_value())) return false;
	PlanetCore::LodView Overview = View;
	Overview.SurfaceObserverMeters = Dry.Vector() * (View.RadiusMeters + DrySample->Terrain.ElevationMeters + 1);
	Overview.ObserverMeters = Dry.Vector() * (View.RadiusMeters + DrySample->Terrain.ElevationMeters + 500);
	std::optional<PlanetCore::FlowTerrainMesh> OverviewTerrain = TerrainBuilder->Build(Overview, {}, &GroundReport);
	// 俯瞰の生成拒否を以前の面で隠さない返戻
	if(!TestTrue(TEXT("実画面地点の俯瞰地面を準備"), OverviewTerrain.has_value())) return false;
	const std::optional<PlanetCore::FlowWaterMesh> OverviewWater =
	WaterView->Build(*OverviewTerrain, Overview, WaterReport, {}, &GroundReport);
	// 水面を省略した比較へ縮小しない返戻
	if(!TestTrue(TEXT("俯瞰の共有地面と水面を準備"), OverviewWater.has_value())) return false;
	// 独立な材質照会が欠落した場合の返戻
	if(!InspectFlowAppearance(*this, *OverviewTerrain, *Field)) return false;
	// 準備用コンテナを解放しても，上流の所有参照が下流湖岸を保持
	const std::shared_ptr<const PlanetCore::FlowChannelField> Survivor = Prepared->Channels.back().Channel.Field;
	const PlanetCore::UnitDirection Query =
	Prepared->Channels.back().Channel.Route->Sample(Prepared->Channels.back().Channel.Route->GetLengthMeters())->Direction;
	const std::optional<PlanetCore::FlowChannelSection> Before = Survivor->Evaluate(Query);
	Prepared.reset();
	Fields.clear();
	const std::optional<PlanetCore::FlowChannelSection> After = Survivor->Evaluate(Query);
	TestTrue(
		TEXT("準備コンテナ解放後も水位と河床を保持"),
		Before && After && Before->Water.ElevationMeters == After->Water.ElevationMeters &&
		Before->Terrain.ElevationMeters == After->Terrain.ElevationMeters
	);
	// 有限網の全検査の返戻；全惑星の水系列挙や最終合成の完成とはしない
	return !HasAnyErrors();
}

#endif
