#include "Core/PlanetDrainage.h"
#include "Core/PlanetDrainageSampling.h"
#include "Core/PlanetSphericalGradient.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSphericalDrainage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FPlanetAdaptiveSurfaceContracts, "PlanetGenerator.SphericalRelief.AdaptiveSurface",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 既定の最小半径と地形比較用の半径を列挙する
 * @param Names 検査表示名
 * @param Commands 実行引数
 */
void FPlanetAdaptiveSurfaceContracts::GetTests(TArray<FString> &Names, TArray<FString> &Commands) const {
	Names.Add(TEXT("MinimumRadius"));
	Commands.Add(TEXT("MinimumRadius"));
	Names.Add(TEXT("ReferenceRadius"));
	Commands.Add(TEXT("ReferenceRadius"));
	Names.Add(TEXT("SampledMinimumRadius"));
	Commands.Add(TEXT("SampledMinimumRadius"));
	Names.Add(TEXT("SampledReferenceRadius"));
	Commands.Add(TEXT("SampledReferenceRadius"));
	Names.Add(TEXT("SampledReferenceStages"));
	Commands.Add(TEXT("SampledReferenceStages"));
	Names.Add(TEXT("ReferenceStages"));
	Commands.Add(TEXT("ReferenceStages"));
	// 終了
	return;
}

/**
 * 排水前の適応標本化から，河口・連続標高・解析接線までの結合を検査する
 * @param Parameters 検査半径
 * @return 有効な接続と標本点での下降を確認した場合に真，景観の合格は別
 */
bool FPlanetAdaptiveSurfaceContracts::RunTest(const FString &Parameters) {
	using PlanetCore::DrainageNode;
	using PlanetCore::OceanConnectivityField;
	using PlanetCore::PlanetSphericalDrainage;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = Parameters.EndsWith(TEXT("MinimumRadius")) ? 1000 : 1000000;
	const PlanetCore::SphericalGradientMode Mode = Parameters.StartsWith(TEXT("Sampled")) ?
	PlanetCore::SphericalGradientMode::Sampled :
	PlanetCore::SphericalGradientMode::Analytic;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元地質が無い場合の返戻
	if(!TestTrue(TEXT("適応面の地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 元山稜が無い場合の返戻
	if(!TestTrue(TEXT("適応面の山稜を準備"), Ridges.has_value())) return false;
	std::optional<OceanConnectivityField> PreparedOcean = OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 元海域が無い場合の返戻
	if(!TestTrue(TEXT("適応面の元海域を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const OceanConnectivityField> Ocean =
	std::make_shared<const OceanConnectivityField>(std::move(*PreparedOcean));
	PlanetCore::SphericalReliefSettings Settings;
	Settings.RadiusMeters = Definition.RadiusMeters;
	Settings.TargetErrorMeters *= Settings.RadiusMeters / (Settings.RadiusMeters + 250000);
	Settings.MaximumAddedVertices = 16384;
	Settings.ShouldPreserveVertexSlopes = true;
	// 同じ公開入口に於ける不正入力と中断の検査
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断済の適応面を拒否"),
		PlanetSphericalDrainage::CreateWithAdaptiveDrainage(Ocean, Settings, Mode, Cancelled.get_token()).has_value()
	);
	TestFalse(TEXT("元場の無い適応面を拒否"), PlanetSphericalDrainage::CreateWithAdaptiveDrainage(nullptr, Settings, Mode).has_value());
	PlanetCore::SphericalReliefSettings Different = Settings;
	Different.RadiusMeters *= 2;
	TestFalse(TEXT("異なる物理尺度を拒否"), PlanetSphericalDrainage::CreateWithAdaptiveDrainage(Ocean, Different, Mode).has_value());
	TestFalse(
		TEXT("未定義の接線方式を拒否"),
		PlanetSphericalDrainage::CreateWithAdaptiveDrainage(Ocean, Settings, static_cast<PlanetCore::SphericalGradientMode>(255))
		.has_value()
	);
	const std::optional<PlanetSphericalDrainage> Field = PlanetSphericalDrainage::CreateWithAdaptiveDrainage(Ocean, Settings, Mode);
	// 最終面の失敗を排水準備だけの成功へ置換えない返戻
	if(!TestTrue(TEXT("適応網から河口を保持した連続地形まで準備"), Field.has_value())) return false;
	const PlanetCore::DrainageNetwork &Network = Field->GetDrainageNetwork();
	const std::span<const DrainageNode> Nodes = Network.Nodes();
	const PlanetCore::SphericalDrainageStatistics &Stats = Field->GetStatistics();
	TestTrue(TEXT("実際の接線方式を保持"), Stats.GradientMode == Mode);
	TestTrue(TEXT("追加した元起伏の標本を最終面でも保持"), Stats.Relief.AddedVertices > 0 && Stats.Nodes > 40962 && Stats.Nodes < 65537);
	TestTrue(TEXT("実際の河口を共有"), Stats.CoastalOutlets == Network.Statistics().CoastalOutletNodes && Stats.CoastalOutlets > 0);
	// 全陸上節点と実下流辺の独立標本検査
	std::uint32_t LandNodes = 0, BrokenRoutes = 0, UphillSamples = 0, EdgeSamples = 0, GradientChecks = 0;
	// 差分の刻み幅依存と独立な収束判定
	double MaximumNodeError = 0, MaximumUphill = 0, MaximumGradientError = 0, MaximumCoarseError = 0, MaximumFineError = 0;
	double MaximumRichardsonChange = 0;
	std::uint32_t UnconvergedGradients = 0;
	Vec3 WorstGradientDirection;
	std::array<double, 3> WorstDifferences {};
	double WorstAnalytic = 0;
	for(std::uint32_t Index = 0; Index < Nodes.size(); ++Index) {
		const DrainageNode &Node = Nodes[Index];
		if(Node.Ocean) continue;
		++LandNodes;
		const UnitDirection Direction = *UnitDirection::FromVector(Node.Direction);
		const double Freeboard = Network.Settings().BankFreeboardMeters, Head = Node.WaterMeters - Settings.SeaLevelMeters;
		const double Expected = Node.WaterMeters + Freeboard * Head / (Head + Freeboard);
		MaximumNodeError = std::max(MaximumNodeError, std::abs(Field->EvaluateElevation(Direction) - Expected));
		if(Node.Downstream >= Nodes.size() || Node.Outlet >= Nodes.size() || !Nodes[Node.Outlet].Ocean) {
			++BrokenRoutes;
			continue;
		}
		const DrainageNode &Parent = Nodes[Node.Downstream];
		if(!(Node.WaterMeters > Parent.WaterMeters && Node.DistanceToOceanMeters > Parent.DistanceToOceanMeters)) ++BrokenRoutes;
		const double Cosine = std::clamp(Node.Direction.Dot(Parent.Direction), -1.0, 1.0), Angle = std::acos(Cosine);
		const Vec3 Tangent = (Parent.Direction - Node.Direction * Cosine) * (1 / std::sin(Angle));
		for(unsigned Step = 1; Step < 12; ++Step) {
			const double Arc = Angle * Step / 12;
			const UnitDirection At = *UnitDirection::FromVector(Node.Direction * std::cos(Arc) + Tangent * std::sin(Arc));
			const Vec3 Downhill = Tangent * std::cos(Arc) - Node.Direction * std::sin(Arc);
			const PlanetCore::TerrainSample Sample = Field->Evaluate(At);
			const double Slope = Sample.TangentialGradientPerRadian.Dot(Downhill) / Settings.RadiusMeters;
			MaximumUphill = std::max(MaximumUphill, Slope);
			if(!std::isfinite(Sample.ElevationMeters) || !std::isfinite(Slope) || Slope > 1E-8) ++UphillSamples;
			++EdgeSamples;
		}
		// 流路と異なる接線方向に於ける解析微分と中央差分の照合
		if(Index % 31) continue;
		const Vec3 Side = Node.Direction.Cross(Tangent);
		constexpr double Step = 1E-7;
		std::array<double, 3> Differences {};
		double LargestHeight = 0;
		for(unsigned Level = 0; Level < Differences.size(); ++Level) {
			const double H = std::ldexp(Step, -static_cast<int>(Level));
			const UnitDirection Plus = *UnitDirection::FromVector(Node.Direction * std::cos(H) + Side * std::sin(H));
			const UnitDirection Minus = *UnitDirection::FromVector(Node.Direction * std::cos(H) - Side * std::sin(H));
			const double Positive = Field->EvaluateElevation(Plus), Negative = Field->EvaluateElevation(Minus);
			Differences[Level] = (Positive - Negative) / (2 * H);
			LargestHeight = std::max({ LargestHeight, std::abs(Positive), std::abs(Negative) });
		}
		// C1 接続点では二次微分が一致しない為，中央差分の先頭誤差は刻み幅の一次となる
		const double Analytic = Field->Evaluate(Direction).TangentialGradientPerRadian.Dot(Side);
		const double CoarseError = std::abs(Differences[0] - Analytic) / Settings.RadiusMeters;
		if(CoarseError > MaximumCoarseError) {
			MaximumCoarseError = CoarseError;
			WorstGradientDirection = Node.Direction;
			WorstDifferences = Differences;
			WorstAnalytic = Analytic;
		}
		MaximumFineError = std::max(MaximumFineError, std::abs(Differences[2] - Analytic) / Settings.RadiusMeters);
		const double First = 2 * Differences[1] - Differences[0], Second = 2 * Differences[2] - Differences[1];
		MaximumGradientError = std::max(MaximumGradientError, std::abs(Second - Analytic) / Settings.RadiusMeters);
		MaximumRichardsonChange = std::max(MaximumRichardsonChange, std::abs(Second - First) / Settings.RadiusMeters);
		const double Roundoff = 256 * std::numeric_limits<double>::epsilon() * std::max(1.0, LargestHeight) / (Step * 0.25);
		if(std::abs(Differences[2] - Differences[1]) > 0.75 * std::abs(Differences[1] - Differences[0]) + Roundoff) {
			++UnconvergedGradients;
		}
		++GradientChecks;
	}
	TestTrue(TEXT("共有節点の標高を保持"), LandNodes > 0 && MaximumNodeError < 1E-6);
	TestTrue(TEXT("全陸上節点の下流と大洋出口を保持"), BrokenRoutes == 0);
	TestTrue(TEXT("指定下流辺の地形が独立標本でも下降"), EdgeSamples > 0 && UphillSamples == 0);
	TestTrue(TEXT("解析接線と独立差分が一致"), GradientChecks > 0 && MaximumGradientError < 1E-5);
	TestTrue(TEXT("差分と一次外挿の収束を確認"), UnconvergedGradients == 0 && MaximumRichardsonChange < 1E-5);
	AddInfo(
		FString::Printf(
			TEXT("Adaptive derivative convergence: radius=%.0f coarse=%.12g fine=%.12g extrapolated=%.12g change=%.12g")
			TEXT(" unconverged=%u direction=(%.17g,%.17g,%.17g) differences=(%.17g,%.17g,%.17g) analytic=%.17g"),
			Settings.RadiusMeters,
			MaximumCoarseError,
			MaximumFineError,
			MaximumGradientError,
			MaximumRichardsonChange,
			UnconvergedGradients,
			WorstGradientDirection.X,
			WorstGradientDirection.Y,
			WorstGradientDirection.Z,
			WorstDifferences[0],
			WorstDifferences[1],
			WorstDifferences[2],
			WorstAnalytic
		)
	);
	// 既知の不自然な地点での元地形との差分計測（景観合格とは別）
	const UnitDirection Anchor = *UnitDirection::FromVector({ 0.24923960651576593, 0.8616223148088015, 0.442138671875 });
	const Vec3 East = UnitDirection::FromVector(Vec3{ 0, 0, 1 }.Cross(Anchor.Vector()))->Vector();
	const Vec3 North = Anchor.Vector().Cross(East);
	const double Extent = 12000 * Settings.RadiusMeters / 1000000;
	double Squared = 0, MaximumDifference = 0;
	for(unsigned Y = 0; Y < 33; ++Y) for(unsigned X = 0; X < 33; ++X) {
		const UnitDirection At = *UnitDirection::FromVector(
			Anchor.Vector() + East * ((X / 16.0 - 1) * Extent / Settings.RadiusMeters) +
			North * ((Y / 16.0 - 1) * Extent / Settings.RadiusMeters)
		);
		const double Difference = Field->EvaluateElevation(At) - Field->EvaluateOriginal(At).ElevationMeters;
		Squared += Difference * Difference;
		MaximumDifference = std::max(MaximumDifference, std::abs(Difference));
	}
	// 同じ最終網で，標高調整と接線制限を独立に外した反実仮想を計測する
	if(Parameters.Contains(TEXT("Stages"))) {
		const std::optional<PlanetCore::SphereMesh> Uniform = PlanetCore::BuildUniformSphere(6);
		// 元の標本化を再現出来ない場合の返戻
		if(!TestTrue(TEXT("切分けの元網を準備"), Uniform.has_value())) return false;
		std::vector<double> OriginalHeights;
		for(const UnitDirection Direction : Uniform->Directions) OriginalHeights.push_back(Ocean->EvaluateElevation(Direction));
		const auto Original = [&Ocean](const UnitDirection Direction) -> double {
			// 適応標本化前の同じ海域場の返戻
			return Ocean->EvaluateElevation(Direction);
		};
		const std::optional<PlanetCore::SphericalReliefSamplingMesh> Sampling =
		PlanetCore::PlanetSphericalRefinement::CreateSamplingMesh(
			Uniform->Directions,
			Uniform->Triangles,
			OriginalHeights,
			Original,
			Settings
		);
		// 元場と同じ適応配置を用意出来ない場合の返戻
		if(!TestTrue(TEXT("切分けの適応網を再現"), Sampling.has_value())) return false;
		const std::size_t InitialCount = Sampling->Directions.size();
		// 河口だけが追加された事を確認する返戻
		if(!TestTrue(TEXT("同じ元節点と河口数"), InitialCount + Stats.CoastalOutlets == Nodes.size())) return false;
		std::vector<UnitDirection> Outlets;
		for(std::size_t Index = InitialCount; Index < Nodes.size(); ++Index) {
			Outlets.push_back(*UnitDirection::FromVector(Nodes[Index].Direction));
		}
		const std::optional<PlanetCore::DrainageSamplingMesh> Mesh =
		PlanetCore::BuildDrainageSamplingMeshOnMesh(Sampling->Directions, Sampling->Triangles, Outlets);
		// 比較で位相を変えていない事を確認する返戻
		if(!TestTrue(TEXT("同じ河口を含む最終網を再現"), Mesh.has_value() && Mesh->Directions.size() == Stats.Nodes)) return false;
		std::vector<double> SourceHeights, AdjustedHeights(Mesh->Directions.size());
		std::vector<std::uint32_t> Routes(Mesh->Directions.size()), Identity(Mesh->Directions.size());
		std::iota(Routes.begin(), Routes.end(), 0U);
		std::iota(Identity.begin(), Identity.end(), 0U);
		for(const UnitDirection Direction : Mesh->Directions) {
			SourceHeights.push_back(Field->EvaluateOriginal(Direction).ElevationMeters);
		}
		const auto Vertex = [&](const std::size_t Index) -> std::uint32_t {
			// 元節点又は追加河口の頂点番号の返戻
			return Index < InitialCount ? static_cast<std::uint32_t>(Index) : Mesh->SampleNodes[Index - InitialCount];
		};
		double NodeSquared = 0, NodeRaise = 0, NodeCut = 0;
		std::uint32_t LocalNodes = 0;
		for(std::size_t Index = 0; Index < Nodes.size(); ++Index) {
			const DrainageNode &Node = Nodes[Index];
			const double Freeboard = Network.Settings().BankFreeboardMeters;
			const double Head = std::max(0.0, Node.WaterMeters - Settings.SeaLevelMeters);
			const double Value = Node.Ocean ?
			Index < InitialCount ? Node.OriginalElevationMeters : Node.WaterMeters - Node.DepthMeters :
			Node.WaterMeters + Freeboard * Head / (Head + Freeboard);
			const std::uint32_t At = Vertex(Index);
			AdjustedHeights[At] = Value;
			if(Node.Ocean) continue;
			Routes[At] = Vertex(Node.Downstream);
			const double Denominator = Node.Direction.Dot(Anchor.Vector());
			if(Denominator <= 0) continue;
			const double X = Node.Direction.Dot(East) / Denominator, Y = Node.Direction.Dot(North) / Denominator;
			if(std::abs(X) > 0.012 || std::abs(Y) > 0.012) continue;
			const double Delta = Value - SourceHeights[At];
			NodeSquared += Delta * Delta;
			NodeRaise = std::max(NodeRaise, Delta);
			NodeCut = std::max(NodeCut, -Delta);
			++LocalNodes;
			AddInfo(
				FString::Printf(
					TEXT("Staged local node: index=%llu x=%.9g y=%.9g source=%.12g adjusted=%.12g delta=%.12g"),
					static_cast<unsigned long long>(Index),
					X,
					Y,
					SourceHeights[At],
					Value,
					Delta
				)
			);
		}
		TestTrue(TEXT("問題地点内の節点が存在"), LocalNodes > 0);
		AddInfo(
			FString::Printf(
				TEXT("Staged nodal conditioning: count=%u rms=%.12g raise=%.12g cut=%.12g"),
				LocalNodes,
				std::sqrt(NodeSquared / std::max(1U, LocalNodes)),
				NodeRaise,
				NodeCut
			)
		);
		PlanetCore::SphericalReliefSettings Unrefined = Settings;
		Unrefined.MaximumAddedVertices = 0;
		const auto SameSource = [&Field](const UnitDirection Direction) -> double {
			// 海域再接続を含む同じ元標高の返戻
			return Field->EvaluateOriginal(Direction).ElevationMeters;
		};
		// 残差復元の比較では，同じ内陸流路の方向を独立に保持する
		std::vector<UnitDirection> EdgeDirections;
		std::vector<Vec3> EdgeTangents;
		std::vector<double> SourceEdgeDerivatives, SourcePatchHeights;
		for(std::uint32_t Index = 0; Index < Nodes.size(); Index += 17) {
			const DrainageNode &Node = Nodes[Index];
			if(Node.Ocean) continue;
			const Vec3 End = Nodes[Node.Downstream].Direction;
			const double Cosine = std::clamp(Node.Direction.Dot(End), -1.0, 1.0), Angle = std::acos(Cosine);
			const Vec3 Tangent = (End - Node.Direction * Cosine) * (1 / std::sin(Angle));
			for(unsigned Step = 1; Step < 12; ++Step) {
				const double Arc = Angle * Step / 12;
				EdgeDirections.push_back(*UnitDirection::FromVector(Node.Direction * std::cos(Arc) + Tangent * std::sin(Arc)));
				EdgeTangents.push_back(Tangent * std::cos(Arc) - Node.Direction * std::sin(Arc));
			}
		}
		for(unsigned Adjusted = 0; Adjusted < 2; ++Adjusted) {
			const std::vector<double> &Heights = Adjusted ? AdjustedHeights : SourceHeights;
			std::optional<PlanetCore::SphericalGradientEstimate> Estimated;
			if(Mode == PlanetCore::SphericalGradientMode::Sampled) {
				Estimated = PlanetCore::EstimateSphericalGradients(Mesh->Directions, Mesh->Triangles, Heights);
			} else {
				Estimated.emplace();
				for(const UnitDirection Direction : Mesh->Directions) {
					Estimated->Gradients.push_back(Field->EvaluateOriginal(Direction).TangentialGradientPerRadian);
				}
			}
			const std::optional<PlanetCore::PlanetSphericalRefinement> Scaffold =
			PlanetCore::PlanetSphericalRefinement::Create(Mesh->Directions, Mesh->Triangles, Heights, SameSource, Unrefined);
			// 同じ値と配置で接線を準備出来ない場合の返戻
			if(!TestTrue(TEXT("切分けの接線と制約を準備"), Estimated.has_value() && Scaffold.has_value())) return false;
			for(unsigned Constraint = 0; Constraint < (Adjusted ? 3U : 2U); ++Constraint) {
				std::optional<std::vector<Vec3>> Gradients = Constraint ?
				Scaffold->ConstrainDrainageGradients(Estimated->Gradients, Settings.SeaLevelMeters, Constraint == 2 ? Routes : Identity) :
				std::optional<std::vector<Vec3>>(Estimated->Gradients);
				// 制約を除く以外の準備失敗を無視しない返戻
				if(!TestTrue(TEXT("切分けの制約を解く"), Gradients.has_value())) return false;
				const std::optional<PlanetCore::SphericalSplineField> Surface =
				PlanetCore::SphericalSplineField::CreateOnMeshWithGradients(6, Mesh->Directions, Mesh->Triangles, Heights, *Gradients);
				// 同じ補間式が成立しない場合の返戻
				if(!TestTrue(TEXT("切分けの補間を準備"), Surface.has_value())) return false;
				double StageSquared = 0, LargestDifference = 0, FinalDifference = 0, RestoredSquared = 0, RestoredMaximum = 0;
				for(unsigned Y = 0; Y < 33; ++Y) for(unsigned X = 0; X < 33; ++X) {
					const UnitDirection At =
					*UnitDirection::FromVector(Anchor.Vector() + East * ((X / 16.0 - 1) * 0.012) + North * ((Y / 16.0 - 1) * 0.012));
					const double Value = Surface->Evaluate(At).ElevationMeters, Delta = Value - SameSource(At);
					if(!Adjusted && Constraint == 1) SourcePatchHeights.push_back(Value);
					if(Adjusted && Constraint == 2) {
						const double Correction = Value - SourcePatchHeights[33 * Y + X];
						RestoredSquared += Correction * Correction;
						RestoredMaximum = std::max(RestoredMaximum, std::abs(Correction));
					}
					StageSquared += Delta * Delta;
					LargestDifference = std::max(LargestDifference, std::abs(Delta));
					FinalDifference = std::max(FinalDifference, std::abs(Value - Field->EvaluateElevation(At)));
				}
				AddInfo(
					FString::Printf(
						TEXT("Staged surface: adjusted=%u constraint=%u nodes=%llu rms=%.12g maximum=%.12g finalDelta=%.12g"),
						Adjusted,
						Constraint,
						static_cast<unsigned long long>(Heights.size()),
						std::sqrt(StageSquared / 1089),
						LargestDifference,
						FinalDifference
					)
				);
				if(!Adjusted && Constraint == 1) for(std::size_t Index = 0; Index < EdgeDirections.size(); ++Index) {
					SourceEdgeDerivatives.push_back(Surface->Evaluate(EdgeDirections[Index]).TangentialGradientPerRadian.Dot(EdgeTangents[Index]));
				}
				if(Adjusted && Constraint == 2) {
					TestTrue(TEXT("全段階を戻すと実際の公開面に一致"), FinalDifference < 1E-6);
					double MaximumRestoredSlope = 0, MaximumReferenceSlope = 0;
					std::uint32_t RestoredUphill = 0, ReferenceUphill = 0, InlandUphill = 0;
					for(std::size_t Index = 0; Index < EdgeDirections.size(); ++Index) {
						const PlanetCore::TerrainSample BaseSample = Field->EvaluateOriginal(EdgeDirections[Index]);
						const PlanetCore::TerrainSample FinalSample = Field->Evaluate(EdgeDirections[Index]);
						const double OriginalSlope = BaseSample.TangentialGradientPerRadian.Dot(EdgeTangents[Index]);
						const double ReferenceSlope = FinalSample.TangentialGradientPerRadian.Dot(EdgeTangents[Index]);
						const double RestoredSlope = (OriginalSlope + ReferenceSlope - SourceEdgeDerivatives[Index]) / Settings.RadiusMeters;
						MaximumRestoredSlope = std::max(MaximumRestoredSlope, RestoredSlope);
						MaximumReferenceSlope = std::max(MaximumReferenceSlope, ReferenceSlope / Settings.RadiusMeters);
						if(ReferenceSlope / Settings.RadiusMeters > 1E-8) ++ReferenceUphill;
						if(RestoredSlope > 1E-8) {
							++RestoredUphill;
							if(BaseSample.ElevationMeters > 100) ++InlandUphill;
						}
					}
					TestTrue(TEXT("復元前の同じ比較辺は下降"), ReferenceUphill == 0 && !EdgeDirections.empty());
					AddInfo(
						FString::Printf(
							TEXT("Source restoration: mode=%u rms=%.12g maximum=%.12g edgeSamples=%llu uphill=%u inlandUphill=%u")
							TEXT(" maximumSlope=%.12g referenceUphill=%u referenceMaximumSlope=%.12g"),
							static_cast<unsigned>(Mode),
							std::sqrt(RestoredSquared / 1089),
							RestoredMaximum,
							static_cast<unsigned long long>(EdgeDirections.size()),
							RestoredUphill,
							InlandUphill,
							MaximumRestoredSlope,
							ReferenceUphill,
							MaximumReferenceSlope
						)
					);
				}
			}
		}
	}
	const double Milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(
		FString::Printf(
			TEXT("Adaptive surface: mode=%u radius=%.0f nodes=%u added=%u mouths=%u land=%u nodeError=%.12g edges=%u uphill=%u")
			TEXT(" maximumSlope=%.12g gradients=%u gradientError=%.12g siteRms=%.12g siteMax=%.12g")
			TEXT(" raise=%.12g cut=%.12g resident=%llu milliseconds=%.6g"),
			static_cast<unsigned>(Mode),
			Settings.RadiusMeters,
			Stats.Nodes,
			Stats.Relief.AddedVertices,
			Stats.CoastalOutlets,
			LandNodes,
			MaximumNodeError,
			EdgeSamples,
			UphillSamples,
			MaximumUphill,
			GradientChecks,
			MaximumGradientError,
			std::sqrt(Squared / 1089),
			MaximumDifference,
			Stats.Relief.MaximumConditioningRaiseMeters,
			Stats.Relief.MaximumConditioningCutMeters,
			static_cast<unsigned long long>(Stats.SplineResidentBytes),
			Milliseconds
		)
	);
	// 全検査結果の返戻
	return !HasAnyErrors();
}

#endif
