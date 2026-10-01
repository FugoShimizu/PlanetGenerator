#include "Core/PlanetDrainage.h"
#include "Core/PlanetErosionPreparation.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOceanMinimaDiagnostics.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetRiverTerrain.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPlanetPredrainageOrogenyContracts, "PlanetGenerator.Geology.PredrainageOrogenyContracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 造山を排水・侵食より前に共有し，異なる生成定義の水系を拒否する
 * @param Parameters 未使用の自動検査引数
 * @return 経路・微分・定義整合が成立する場合に真
 */
bool FPlanetPredrainageOrogenyContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	const std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Settings);
	// 地質の準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("地質の準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(*PreparedGeology);
	std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山場の準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("造山場の準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*PreparedRidges));
	TestFalse(TEXT("造山場の欠落を拒否"), PlanetCore::OceanConnectivityField::CreateWithOrogeny(Geology, {}).has_value());
	PlanetCore::GeologySettings OtherSettings = Settings;
	OtherSettings.Seed = 7;
	const std::optional<PlanetCore::GeologyField> OtherGeology = PlanetCore::GeologyField::Create(OtherSettings);
	// 別惑星の地質の準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("別惑星の地質の準備"), OtherGeology.has_value())) return false;
	const std::optional<PlanetCore::PlanetOrogenicRidges> OtherRidges = PlanetCore::PlanetOrogenicRidges::Create(*OtherGeology);
	// 別惑星の造山場の準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("別惑星の造山場の準備"), OtherRidges.has_value())) return false;
	TestFalse(
		TEXT("異なる生成定義の造山場を拒否"),
		PlanetCore::OceanConnectivityField::CreateWithOrogeny(
			Geology,
			std::make_shared<const PlanetCore::PlanetOrogenicRidges>(*OtherRidges)
		).has_value()
	);
	std::stop_source Cancelled;
	Cancelled.request_stop();
	TestFalse(
		TEXT("中断済みの海域準備を拒否"),
		PlanetCore::OceanConnectivityField::CreateWithOrogeny(Geology, Ridges, Cancelled.get_token()).has_value()
	);
	const std::optional<PlanetCore::OceanConnectivityField> LegacyPrepared = PlanetCore::OceanConnectivityField::Create(Geology);
	const std::optional<PlanetCore::OceanConnectivityField> OrogenicPrepared =
	PlanetCore::OceanConnectivityField::CreateWithOrogeny(Geology, Ridges);
	// 従来と造山前置の海域場を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("従来と造山前置の海域場を準備"), LegacyPrepared.has_value() && OrogenicPrepared.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Legacy =
	std::make_shared<const PlanetCore::OceanConnectivityField>(*LegacyPrepared);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Orogenic =
	std::make_shared<const PlanetCore::OceanConnectivityField>(*OrogenicPrepared);
	TestTrue(TEXT("海域の算法版を含む定義を識別"), Legacy->DefinitionHash() != Settings.IdentityHash());
	TestTrue(TEXT("造山前置を別の定義として識別"), Orogenic->DefinitionHash() != Legacy->DefinitionHash());
	TestTrue(TEXT("造山後の固定網で大洋が接続"), Orogenic->Statistics().ComponentsAfter == 1);
	double MaximumSourceDifference = 0;
	std::uint32_t TestedLand = 0;
	for(std::uint32_t Index = 0; Index < 1024; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / 1024, Azimuth = Index * 2.399963229728653, Width = std::sqrt(1 - Z * Z);
		const PlanetCore::UnitDirection Direction =
		*PlanetCore::UnitDirection::FromVector({ Width * std::cos(Azimuth), Width * std::sin(Azimuth), Z });
		const double Original = Legacy->EvaluateElevation(Direction), Altered = Orogenic->EvaluateElevation(Direction);
		if(Original > Settings.SeaLevelMeters + 500 && Altered > Settings.SeaLevelMeters) {
			MaximumSourceDifference = std::max(MaximumSourceDifference, Altered - Original);
			++TestedLand;
		}
		const PlanetCore::TerrainSample Analytic = Orogenic->Evaluate(Direction);
		const bool FailedFiniteFieldTest = !TestTrue(
			TEXT("造山後の海陸値と勾配が有限"),
			std::isfinite(Analytic.ElevationMeters) && std::isfinite(Analytic.TangentialGradientPerRadian.Length())
		);
		// 造山後の海陸値と勾配が有限の失敗時に検査を続けない返戻
		if(FailedFiniteFieldTest) return false;
		// 値専用と解析微分の標高が一致の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("値専用と解析微分の標高が一致"), std::abs(Analytic.ElevationMeters - Altered) <= 1E-8)) return false;
	}
	TestTrue(TEXT("陸上の造山差分を実際に評価"), TestedLand > 0 && MaximumSourceDifference > 1);
	const std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Orogenic);
	// 造山を含む海陸から排水を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("造山を含む海陸から排水を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(*PreparedDrainage);
	TestTrue(TEXT("排水が造山定義を保持"), Drainage->SourceDefinitionHash() == Orogenic->DefinitionHash());
	TestFalse(TEXT("異なる海陸と排水の混合を拒否"), PlanetCore::RiverTerrainField::Create(Legacy, Drainage).has_value());
	const std::optional<PlanetCore::RiverTerrainField> Final = PlanetCore::RiverTerrainField::CreateWithErosion(Orogenic, Drainage);
	// 造山・排水・侵食・河川を一体で準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("造山・排水・侵食・河川を一体で準備"), Final.has_value())) return false;
	TestTrue(TEXT("最終河道の定義を保持"), Final->Drainage().SourceDefinitionHash() == Orogenic->DefinitionHash());
	const PlanetCore::UnitDirection Probe = *PlanetCore::UnitDirection::FromVector({ 1, 0.3, 0.1 });
	const PlanetCore::TerrainSample Sample = Final->Evaluate(Probe);
	TestTrue(
		TEXT("表示と衝突で共有する最終Hが有限"),
		std::isfinite(Sample.ElevationMeters) && std::isfinite(Sample.TangentialGradientPerRadian.Length())
	);
	const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
	// 最終標高と同じ固定球面網を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("最終標高と同じ固定球面網を準備"), Mesh.has_value())) return false;
	std::array<std::vector<double>, 3> Heights;
	for(std::vector<double> &Values : Heights) Values.resize(Mesh->Directions.size());
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		const PlanetCore::UnitDirection Direction = Mesh->Directions[Index];
		const PlanetCore::RiverElevationStages Stage = Final->EvaluateElevationStages(Direction);
		Heights[0][Index] = Orogenic->EvaluateElevation(Direction);
		Heights[1][Index] = Stage.AfterFineReliefMeters;
		Heights[2][Index] = Stage.FinalMeters;
	}
	std::array<std::uint32_t, 3> Sinks {}, DeepSinks {}, RouteRises {};
	for(std::size_t Stage = 0; Stage < Heights.size(); ++Stage) {
		std::vector<double> Minimum(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::size_t Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
			Minimum[A] = std::min(Minimum[A], Heights[Stage][B]);
			Minimum[B] = std::min(Minimum[B], Heights[Stage][A]);
		}
		for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			const PlanetCore::DrainageNode &Node = Final->Drainage().Nodes()[Index];
			if(Node.Ocean || Heights[Stage][Index] <= Settings.SeaLevelMeters + 0.01) continue;
			const double Depth = Minimum[Index] - Heights[Stage][Index];
			if(Depth > 0.01) {
				++Sinks[Stage];
				if(Depth >= 20) ++DeepSinks[Stage];
			}
			if(
				Node.Downstream < Mesh->Directions.size() && Node.Downstream != Index &&
				Heights[Stage][Node.Downstream] - Heights[Stage][Index] > 0.01
			) ++RouteRises[Stage];
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage sampled graph: source/fine/final sinks=%u/%u/%u ")
			TEXT("deepSinks=%u/%u/%u routeRises=%u/%u/%u. Fixed graph only."),
			Sinks[0],
			Sinks[1],
			Sinks[2],
			DeepSinks[0],
			DeepSinks[1],
			DeepSinks[2],
			RouteRises[0],
			RouteRises[1],
			RouteRises[2]
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Predrainage orogeny: landSamples=%u maxOceanFieldDifference=%.6fm ")
			TEXT("oceanFraction=%.6f originalReaches=%u finalReaches=%u. Drainage quality remains unproven."),
			TestedLand,
			MaximumSourceDifference,
			Orogenic->Statistics().IndependentOceanFractionEstimate,
			Drainage->Statistics().Reaches,
			Final->Drainage().Statistics().Reaches
		)
	);
	TestFalse(TEXT("中規模起伏前置の造山場欠落を拒否"), PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(Geology, {}).has_value());
	const std::optional<PlanetCore::OceanConnectivityField> PreparedRelief =
	PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(Geology, Ridges);
	if(!PreparedRelief) {
		const PlanetCore::OceanMinimaPreparationDiagnostics Diagnostic = PlanetCore::InspectOceanMinimaPreparation();
		AddInfo(
			FString::Printf(
				TEXT("Predrainage ocean rejection: stage=%u evaluations=%llu criticals=%u witnesses=%llu sourceRejected=%u complete=%u"),
				static_cast<unsigned>(Diagnostic.Stage),
				static_cast<unsigned long long>(Diagnostic.Evaluations),
				Diagnostic.CriticalCandidates,
				static_cast<unsigned long long>(Diagnostic.WetWitnesses),
				Diagnostic.SourceRejected ? 1U : 0U,
				Diagnostic.Completed ? 1U : 0U
			)
		);
	}
	// 造山と中規模起伏を排水前へ準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("造山と中規模起伏を排水前へ準備"), PreparedRelief.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> ReliefOcean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(*PreparedRelief);
	TestTrue(TEXT("排水前の中規模起伏を識別"), ReliefOcean->HasPredrainageMesoRelief());
	TestTrue(TEXT("中規模起伏の配置順を定義ハッシュで区別"), ReliefOcean->DefinitionHash() != Orogenic->DefinitionHash());
	TestTrue(TEXT("中規模起伏前置後も大洋を接続"), ReliefOcean->Statistics().ComponentsAfter == 1);
	const std::optional<PlanetCore::DrainageNetwork> PreparedReliefDrainage = PlanetCore::DrainageNetwork::Create(*ReliefOcean);
	// 中規模起伏を含む排水を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("中規模起伏を含む排水を準備"), PreparedReliefDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> ReliefDrainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(*PreparedReliefDrainage);
	TestFalse(TEXT("配置順の異なる海陸と排水を混ぜない"), PlanetCore::RiverTerrainField::Create(Orogenic, ReliefDrainage).has_value());
	const std::optional<PlanetCore::RiverTerrainField> ReliefFinal =
	PlanetCore::RiverTerrainField::CreateWithErosion(ReliefOcean, ReliefDrainage);
	// 排水前起伏を侵食と河川へ接続の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("排水前起伏を侵食と河川へ接続"), ReliefFinal.has_value())) return false;
	TestTrue(TEXT("侵食後も同じ配置順を保持"), ReliefFinal->Drainage().SourceDefinitionHash() == ReliefOcean->DefinitionHash());
	std::array<std::vector<double>, 3> ReliefHeights;
	for(std::vector<double> &Values : ReliefHeights) Values.resize(Mesh->Directions.size());
	std::uint32_t DuplicateMesoSamples = 0;
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		const PlanetCore::UnitDirection Direction = Mesh->Directions[Index];
		const PlanetCore::RiverElevationStages Stages = ReliefFinal->EvaluateElevationStages(Direction);
		ReliefHeights[0][Index] = ReliefOcean->EvaluateElevation(Direction);
		ReliefHeights[1][Index] = Stages.AfterValleyMeters;
		ReliefHeights[2][Index] = Stages.FinalMeters;
		if(Stages.FinalMeters != Stages.AfterFineReliefMeters) ++DuplicateMesoSamples;
	}
	TestTrue(TEXT("河川後段に中規模起伏を二重加算しない"), DuplicateMesoSamples == 0);
	std::array<std::uint32_t, 3> ReliefSinks {}, ReliefDeepSinks {}, ReliefRouteRises {};
	std::array<double, 3> Deepest {};
	std::vector<std::uint8_t> SourceSink(Mesh->Directions.size()), ValleySink(Mesh->Directions.size());
	std::vector<std::uint32_t> FinalSinkIndices;
	for(std::size_t Stage = 0; Stage < ReliefHeights.size(); ++Stage) {
		std::vector<double> Minimum(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::size_t Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
			Minimum[A] = std::min(Minimum[A], ReliefHeights[Stage][B]);
			Minimum[B] = std::min(Minimum[B], ReliefHeights[Stage][A]);
		}
		for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			const PlanetCore::DrainageNode &Node = ReliefFinal->Drainage().Nodes()[Index];
			if(Node.Ocean || ReliefHeights[Stage][Index] <= Settings.SeaLevelMeters + 0.01) continue;
			const double Depth = Minimum[Index] - ReliefHeights[Stage][Index];
			if(Depth > 0.01) {
				++ReliefSinks[Stage];
				Deepest[Stage] = std::max(Deepest[Stage], Depth);
				if(Depth >= 20) ++ReliefDeepSinks[Stage];
				if(!Stage) SourceSink[Index] = 1;
				if(Stage == 1) ValleySink[Index] = 1;
				if(Stage == 2) FinalSinkIndices.push_back(static_cast<std::uint32_t>(Index));
			}
			if(
				Node.Downstream < Mesh->Directions.size() && Node.Downstream != Index &&
				ReliefHeights[Stage][Node.Downstream] - ReliefHeights[Stage][Index] > 0.01
			) ++ReliefRouteRises[Stage];
		}
	}
	std::vector<std::uint8_t> SelectedUpstream(Mesh->Directions.size());
	for(const PlanetCore::RiverReach &Reach : ReliefFinal->Drainage().Reaches()) SelectedUpstream[Reach.Upstream] = 1;
	std::array<std::uint32_t, 3> SinkRiverProximity {};
	std::uint32_t NewAfterFine = 0, SourceSurvivors = 0;
	double MaximumSinkHeightAboveWater = 0;
	for(const std::uint32_t Index : FinalSinkIndices) {
		const PlanetCore::DrainageNode &Node = ReliefFinal->Drainage().Nodes()[Index];
		if(!ValleySink[Index]) ++NewAfterFine;
		if(SourceSink[Index]) ++SourceSurvivors;
		if(SelectedUpstream[Index]) ++SinkRiverProximity[0];
		else if(Node.Downstream < SelectedUpstream.size() && SelectedUpstream[Node.Downstream]) ++SinkRiverProximity[1];
		else ++SinkRiverProximity[2];
		MaximumSinkHeightAboveWater = std::max(MaximumSinkHeightAboveWater, ReliefHeights[2][Index] - Node.WaterMeters);
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage sink sources: newAfterFine=%u sourceSurvivors=%u ")
			TEXT("selected/adjacent/unselected=%u/%u/%u maxHeightAboveWater=%.3fm."),
			NewAfterFine,
			SourceSurvivors,
			SinkRiverProximity[0],
			SinkRiverProximity[1],
			SinkRiverProximity[2],
			MaximumSinkHeightAboveWater
		)
	);
	std::vector<std::uint8_t> MissingPathEdge(Mesh->Directions.size());
	std::uint32_t MissingPathEdges = 0, LongestMissingPath = 0;
	for(const std::uint32_t Start : FinalSinkIndices) {
		std::uint32_t Current = Start, Steps = 0;
		while(!ReliefFinal->Drainage().Nodes()[Current].Ocean && !SelectedUpstream[Current]) {
			if(!MissingPathEdge[Current]) {
				MissingPathEdge[Current] = 1;
				++MissingPathEdges;
			}
			Current = ReliefFinal->Drainage().Nodes()[Current].Downstream;
			// 下流の親が循環して大洋へ辿れない場合の返戻
			if(++Steps > Mesh->Directions.size()) return false;
		}
		LongestMissingPath = std::max(LongestMissingPath, Steps);
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage missing tributaries: uniqueEdges=%u ") TEXT("longestPath=%u fromFinalSinks=%u."),
			MissingPathEdges,
			LongestMissingPath,
			static_cast<std::uint32_t>(FinalSinkIndices.size())
		)
	);
	std::vector<std::uint32_t> PromotedEdges;
	for(std::uint32_t Index = 0; Index < MissingPathEdge.size(); ++Index) if(MissingPathEdge[Index]) PromotedEdges.push_back(Index);
	const std::chrono::steady_clock::time_point PromotionStart = std::chrono::steady_clock::now();
	const std::optional<PlanetCore::DrainageNetwork> Promoted =
	ReliefDrainage->WithPromotedTributaries(PromotedEdges, *ReliefOcean);
	// 必要な小支流を同じ排水木へ追加の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("必要な小支流を同じ排水木へ追加"), Promoted.has_value())) return false;
	const double PromotionMilliseconds =
	std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - PromotionStart).count();
	const std::array<std::uint32_t, 1> InvalidEdge { static_cast<std::uint32_t>(Promoted->Nodes().size()) };
	TestFalse(TEXT("排水網の範囲外の地点を拒否"), ReliefDrainage->WithPromotedTributaries(InvalidEdge, *ReliefOcean).has_value());
	TestFalse(TEXT("異なる生成定義の海陸場を拒否"), ReliefDrainage->WithPromotedTributaries(PromotedEdges, *Orogenic).has_value());
	TestTrue(TEXT("選択した辺だけを河道へ昇格"), Promoted->Reaches().size() == ReliefDrainage->Reaches().size() + MissingPathEdges);
	const std::shared_ptr<const PlanetCore::DrainageNetwork> PromotedNetwork =
	std::make_shared<const PlanetCore::DrainageNetwork>(*Promoted);
	const std::chrono::steady_clock::time_point ReconstructionStart = std::chrono::steady_clock::now();
	const std::optional<PlanetCore::RiverTerrainField> PromotedFinal =
	PlanetCore::RiverTerrainField::CreateWithErosion(ReliefOcean, PromotedNetwork);
	// 小支流追加後に侵食・水位・河床を再準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("小支流追加後に侵食・水位・河床を再準備"), PromotedFinal.has_value())) return false;
	const double ReconstructionMilliseconds =
	std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ReconstructionStart).count();
	double MaximumLowering = 0, MaximumRaising = 0, MeanAbsoluteChange = 0;
	std::uint32_t MaximumLoweringIndex = 0;
	std::vector<double> PromotedHeights(Mesh->Directions.size());
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		PromotedHeights[Index] = PromotedFinal->EvaluateElevation(Mesh->Directions[Index]);
		const double Delta = PromotedHeights[Index] - ReliefHeights[2][Index];
		if(-Delta > MaximumLowering) {
			MaximumLowering = -Delta;
			MaximumLoweringIndex = static_cast<std::uint32_t>(Index);
		}
		MaximumRaising = std::max(MaximumRaising, Delta);
		MeanAbsoluteChange += std::abs(Delta);
	}
	MeanAbsoluteChange /= Mesh->Directions.size();
	std::vector<double> PromotedNeighborMinimum(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::size_t Corner = 0; Corner < 3; ++Corner) {
		const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
		PromotedNeighborMinimum[A] = std::min(PromotedNeighborMinimum[A], PromotedHeights[B]);
		PromotedNeighborMinimum[B] = std::min(PromotedNeighborMinimum[B], PromotedHeights[A]);
	}
	std::uint32_t PromotedSinks = 0, PromotedDeepSinks = 0, PromotedRouteRises = 0;
	double PromotedDeepest = 0;
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		const PlanetCore::DrainageNode &Node = PromotedFinal->Drainage().Nodes()[Index];
		if(Node.Ocean || PromotedHeights[Index] <= Settings.SeaLevelMeters + 0.01) continue;
		const double Depth = PromotedNeighborMinimum[Index] - PromotedHeights[Index];
		if(Depth > 0.01) {
			++PromotedSinks;
			if(Depth >= 20) ++PromotedDeepSinks;
			PromotedDeepest = std::max(PromotedDeepest, Depth);
		}
		if(
			Node.Downstream < Mesh->Directions.size() && Node.Downstream != Index &&
			PromotedHeights[Node.Downstream] - PromotedHeights[Index] > 0.01
		) ++PromotedRouteRises;
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage promoted tributaries: reaches=%u ") TEXT("sinks=%u deepSinks=%u routeRises=%u deepest=%.3fm."),
			static_cast<std::uint32_t>(PromotedFinal->Drainage().Reaches().size()),
			PromotedSinks,
			PromotedDeepSinks,
			PromotedRouteRises,
			PromotedDeepest
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Predrainage promotion cost: preparation=%.3fms ")
			TEXT("reconstruction=%.3fms lowering=%.3fm raising=%.3fm meanAbs=%.3fm ")
			TEXT("maxValleyHalfWidth=%.3fm maxPreparedCut=%.3fm."),
			PromotionMilliseconds,
			ReconstructionMilliseconds,
			MaximumLowering,
			MaximumRaising,
			MeanAbsoluteChange,
			PromotedFinal->Statistics().MaximumValleyHalfWidthMeters,
			PromotedFinal->Statistics().MaximumPreparedCutMeters
		)
	);
	TestTrue(TEXT("同条件で小支流追加が１km級の過大開削を再発させない"), MaximumLowering < 200.0);
	std::vector<std::vector<std::uint32_t>> Adjacency(Mesh->Directions.size());
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::size_t Corner = 0; Corner < 3; ++Corner) {
		const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
		Adjacency[A].push_back(B);
		Adjacency[B].push_back(A);
	}
	struct Counts {
		std::uint32_t Land = 0;
		std::uint32_t NoLowerNeighbor = 0;
		std::uint32_t Trapped = 0;
		std::uint32_t SubmergedLand = 0;
	};
	// 最も低い隣接点を辿った大洋出口の集計
	const auto CountStrictEscape = [&Adjacency, &ReliefDrainage, &Settings](const std::vector<double> &Heights) {
		Counts Result;
		std::vector<std::uint32_t> HeightOrder(Heights.size());
		for(std::uint32_t Index = 0; Index < HeightOrder.size(); ++Index) HeightOrder[Index] = Index;
		std::sort(
			HeightOrder.begin(),
			HeightOrder.end(),
			[&Heights](const std::uint32_t A, const std::uint32_t B) {
				return Heights[A] < Heights[B] || Heights[A] == Heights[B] && A < B;
			}
		);
		std::vector<std::uint8_t> ReachesOcean(Heights.size());
		for(const std::uint32_t Index : HeightOrder) {
			if(ReliefDrainage->Nodes()[Index].Ocean) {
				ReachesOcean[Index] = 1;
				continue;
			}
			++Result.Land;
			if(Heights[Index] <= Settings.SeaLevelMeters) ++Result.SubmergedLand;
			std::uint32_t Lowest = static_cast<std::uint32_t>(Heights.size());
			for(const std::uint32_t Neighbor : Adjacency[Index]) {
				if(Heights[Neighbor] >= Heights[Index]) continue;
				if(Lowest == Heights.size() || Heights[Neighbor] < Heights[Lowest]) Lowest = Neighbor;
			}
			if(Lowest == Heights.size()) ++Result.NoLowerNeighbor;
			else ReachesOcean[Index] = ReachesOcean[Lowest];
			if(!ReachesOcean[Index]) ++Result.Trapped;
		}
		// 大洋への下降経路を持たない陸地点の集計の返戻
		return Result;
	};
	const Counts PromotedEscape = CountStrictEscape(PromotedHeights);
	AddInfo(
		FString::Printf(
			TEXT("Predrainage strict descent promoted: land=%u ")
			TEXT("noLowerNeighbor=%u trapped=%u submergedSourceLand=%u. Fixed graph only."),
			PromotedEscape.Land,
			PromotedEscape.NoLowerNeighbor,
			PromotedEscape.Trapped,
			PromotedEscape.SubmergedLand
		)
	);
	TestTrue(TEXT("小支流追加後の固定網で全陸地点が下降して大洋へ達する"), PromotedEscape.Trapped == 0);
	using SpillEntry = std::pair<double, std::uint32_t>;
	std::priority_queue<SpillEntry, std::vector<SpillEntry>, std::greater<SpillEntry>> Frontier;
	std::vector<double> LeastSpill(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
	for(std::uint32_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		if(!ReliefFinal->Drainage().Nodes()[Index].Ocean) continue;
		LeastSpill[Index] = Settings.SeaLevelMeters;
		Frontier.emplace(Settings.SeaLevelMeters, Index);
	}
	while(!Frontier.empty()) {
		const SpillEntry Current = Frontier.top();
		Frontier.pop();
		if(Current.first != LeastSpill[Current.second]) continue;
		for(const std::uint32_t Neighbor : Adjacency[Current.second]) {
			const double Candidate = std::max(Current.first, ReliefHeights[2][Neighbor]);
			if(Candidate >= LeastSpill[Neighbor]) continue;
			LeastSpill[Neighbor] = Candidate;
			Frontier.emplace(Candidate, Neighbor);
		}
	}
	double MaximumSinkSpill = 0, SumSinkSpill = 0;
	std::array<std::uint32_t, 3> SinkSpillBands {};
	for(const std::uint32_t Index : FinalSinkIndices) {
		const double Spill = LeastSpill[Index] - ReliefHeights[2][Index];
		// 窪地から大洋へ離散経路が存在の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("窪地から大洋へ離散経路が存在"), std::isfinite(Spill))) return false;
		MaximumSinkSpill = std::max(MaximumSinkSpill, Spill);
		SumSinkSpill += Spill;
		++SinkSpillBands[Spill < 20 ? 0 : Spill < 100 ? 1 : 2];
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage ocean escape: localSinks=%u ")
			TEXT("spillBelow20/20to100/atLeast100=%u/%u/%u max=%.3fm mean=%.3fm ")
			TEXT("worstPromotionNodeSpill=%.3fm. Discrete minimax lower bound only."),
			static_cast<std::uint32_t>(FinalSinkIndices.size()),
			SinkSpillBands[0],
			SinkSpillBands[1],
			SinkSpillBands[2],
			MaximumSinkSpill,
			FinalSinkIndices.empty() ? 0 : SumSinkSpill / FinalSinkIndices.size(),
			LeastSpill[MaximumLoweringIndex] - ReliefHeights[2][MaximumLoweringIndex]
		)
	);
	std::vector<PlanetCore::DrainageSite> RerouteSites(Mesh->Directions.size());
	std::vector<std::array<std::uint32_t, 2>> RerouteEdges;
	RerouteEdges.reserve(Mesh->Triangles.size() * 3);
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		RerouteSites[Index].Direction = Mesh->Directions[Index].Vector();
		RerouteSites[Index].ElevationMeters = ReliefHeights[2][Index];
		RerouteSites[Index].Ocean = ReliefFinal->Drainage().Nodes()[Index].Ocean;
	}
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
		const PlanetCore::Vec3 A = RerouteSites[Triangle[0]].Direction;
		const PlanetCore::Vec3 B = RerouteSites[Triangle[1]].Direction;
		const PlanetCore::Vec3 C = RerouteSites[Triangle[2]].Direction;
		const double Share =
		2 * std::atan2(std::abs(A.Dot((B - A).Cross(C - A))), 1 + A.Dot(B) + B.Dot(C) + C.Dot(A)) * Settings.RadiusMeters *
		Settings.RadiusMeters / 3;
		for(std::size_t Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t From = Triangle[Corner], To = Triangle[(Corner + 1) % 3];
			RerouteSites[From].AreaSquareMeters += Share;
			RerouteEdges.push_back({ std::min(From, To), std::max(From, To) });
		}
	}
	std::sort(RerouteEdges.begin(), RerouteEdges.end());
	RerouteEdges.erase(std::unique(RerouteEdges.begin(), RerouteEdges.end()), RerouteEdges.end());
	const std::optional<PlanetCore::DrainageNetwork> Rerouted =
	PlanetCore::DrainageNetwork::Solve(RerouteSites, RerouteEdges, ReliefDrainage->Settings());
	// 最終 H を基準に排水木を再計算の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("最終Hを基準に排水木を再計算"), Rerouted.has_value())) return false;
	std::uint32_t ChangedRoutes = 0;
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		if(Rerouted->Nodes()[Index].Downstream != ReliefDrainage->Nodes()[Index].Downstream) ++ChangedRoutes;
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage route recomputation: changedEdges=%u ")
			TEXT("rawMaxWaterCut=%.3fm rawMaxBankRaise=%.3fm worstPromotionNodeWater=%.3fm."),
			ChangedRoutes,
			Rerouted->Statistics().MaximumRequiredWaterCutMeters,
			Rerouted->Statistics().MaximumRequiredBankRaiseMeters,
			Rerouted->Nodes()[MaximumLoweringIndex].WaterMeters
		)
	);
	/** 更新済み地表の排水先だけを選ぶ地点標高 */
	// 再経路化の基準となる最終標高の共有
	const auto FinalHeight = [&ReliefFinal](const PlanetCore::UnitDirection Direction) {
		return ReliefFinal->EvaluateElevation(Direction);
	};
	TestFalse(TEXT("標高関数を欠く再経路化を拒否"), PlanetCore::DrainageNetwork::CreateForHeight(*ReliefOcean, {}).has_value());
	const std::chrono::steady_clock::time_point ReroutedPreparationStart = std::chrono::steady_clock::now();
	const std::optional<PlanetCore::DrainageNetwork> PreparedRerouted =
	PlanetCore::DrainageNetwork::CreateForHeight(*ReliefOcean, FinalHeight);
	// 最終 H から経路を選び元の海陸から縦断を再準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("最終Hから経路を選び元の海陸から縦断を再準備"), PreparedRerouted.has_value())) return false;
	const double ReroutedPreparationMilliseconds =
	std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ReroutedPreparationStart).count();
	TestTrue(TEXT("再経路化でも海陸定義を保持"), PreparedRerouted->SourceDefinitionHash() == ReliefOcean->DefinitionHash());
	std::uint32_t SubmergedLand = 0, ExposedOcean = 0;
	double LowestLand = std::numeric_limits<double>::infinity();
	for(const PlanetCore::DrainageNode &Node : PreparedRerouted->Nodes()) {
		if(Node.Ocean) {
			if(Node.OriginalElevationMeters > Settings.SeaLevelMeters) ++ExposedOcean;
		} else {
			LowestLand = std::min(LowestLand, Node.OriginalElevationMeters);
			if(Node.OriginalElevationMeters <= Settings.SeaLevelMeters) ++SubmergedLand;
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage reroute sea mismatch: submergedLand=%u ") TEXT("exposedOcean=%u lowestLand=%.6fm."),
			SubmergedLand,
			ExposedOcean,
			LowestLand
		)
	);
	TestEqual(TEXT("再経路化した侵食基準に海面下の陸域を渡さない"), SubmergedLand, 0U);
	TestEqual(TEXT("再経路化で海域の侵食基準を露出させない"), ExposedOcean, 0U);
	const std::shared_ptr<const PlanetCore::DrainageNetwork> ReroutedNetwork =
	std::make_shared<const PlanetCore::DrainageNetwork>(*PreparedRerouted);
	const std::chrono::steady_clock::time_point ReroutedReconstructionStart = std::chrono::steady_clock::now();
	const std::optional<PlanetCore::RiverTerrainField> ReroutedFinal =
	PlanetCore::RiverTerrainField::CreateWithErosion(ReliefOcean, ReroutedNetwork);
	if(!ReroutedFinal) {
		PlanetCore::ErosionPreparationStatistics Failure;
		const std::optional<PlanetCore::PlanetErosionPreparation> PreparationProbe =
		PlanetCore::PlanetErosionPreparation::Create(ReliefOcean, ReroutedNetwork, {}, &Failure);
		AddInfo(
			FString::Printf(
				TEXT("Predrainage reroute failure: stage=%u preparation=%d ")
				TEXT("climate=%.3fms runoff=%.3fms discrete=%.3fms reconstruction=%.3fms ")
				TEXT("aeolian=%.3fms profiles=%.3fms total=%.3fms."),
				static_cast<unsigned>(Failure.Failure),
				PreparationProbe.has_value() ? 1 : 0,
				Failure.ClimateMilliseconds,
				Failure.RunoffMilliseconds,
				Failure.DiscreteMilliseconds,
				Failure.ReconstructionMilliseconds,
				Failure.AeolianMilliseconds,
				Failure.ProfileMilliseconds,
				Failure.TotalMilliseconds
			)
		);
	}
	// 再経路化後の侵食・河床を再構築の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("再経路化後の侵食・河床を再構築"), ReroutedFinal.has_value())) return false;
	const double ReroutedReconstructionMilliseconds =
	std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ReroutedReconstructionStart).count();
	std::vector<double> ReroutedHeights(Mesh->Directions.size());
	double ReroutedMaximumLowering = 0, ReroutedMeanAbsoluteChange = 0;
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		ReroutedHeights[Index] = ReroutedFinal->EvaluateElevation(Mesh->Directions[Index]);
		const double Delta = ReroutedHeights[Index] - ReliefHeights[2][Index];
		ReroutedMaximumLowering = std::max(ReroutedMaximumLowering, -Delta);
		ReroutedMeanAbsoluteChange += std::abs(Delta);
	}
	ReroutedMeanAbsoluteChange /= Mesh->Directions.size();
	std::vector<double> ReroutedNeighborMinimum(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::size_t Corner = 0; Corner < 3; ++Corner) {
		const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
		ReroutedNeighborMinimum[A] = std::min(ReroutedNeighborMinimum[A], ReroutedHeights[B]);
		ReroutedNeighborMinimum[B] = std::min(ReroutedNeighborMinimum[B], ReroutedHeights[A]);
	}
	std::uint32_t ReroutedSinks = 0, ReroutedDeepSinks = 0, ReroutedRouteRises = 0;
	std::vector<std::uint32_t> ReroutedSinkIndices;
	for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
		const PlanetCore::DrainageNode &Node = ReroutedFinal->Drainage().Nodes()[Index];
		if(Node.Ocean || ReroutedHeights[Index] <= Settings.SeaLevelMeters + 0.01) continue;
		const double Depth = ReroutedNeighborMinimum[Index] - ReroutedHeights[Index];
		if(Depth > 0.01) {
			++ReroutedSinks;
			ReroutedSinkIndices.push_back(static_cast<std::uint32_t>(Index));
			if(Depth >= 20) ++ReroutedDeepSinks;
		}
		if(
			Node.Downstream < Mesh->Directions.size() && Node.Downstream != Index &&
			ReroutedHeights[Node.Downstream] - ReroutedHeights[Index] > 0.01
		) ++ReroutedRouteRises;
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage reconstructed reroute: reaches=%u ")
			TEXT("sinks=%u deepSinks=%u routeRises=%u maxLowering=%.3fm meanAbs=%.3fm ")
			TEXT("worstOldCutNode=%.3fm preparation=%.3fms reconstruction=%.3fms."),
			static_cast<std::uint32_t>(ReroutedFinal->Drainage().Reaches().size()),
			ReroutedSinks,
			ReroutedDeepSinks,
			ReroutedRouteRises,
			ReroutedMaximumLowering,
			ReroutedMeanAbsoluteChange,
			ReroutedHeights[MaximumLoweringIndex],
			ReroutedPreparationMilliseconds,
			ReroutedReconstructionMilliseconds
		)
	);
	std::vector<std::uint8_t> SelectedRerouted(Mesh->Directions.size()), TributaryNeeded(Mesh->Directions.size());
	for(const PlanetCore::RiverReach &Reach : ReroutedNetwork->Reaches()) SelectedRerouted[Reach.Upstream] = 1;
	std::uint32_t SelectedReroutedSinks = 0, ReroutedMissingEdges = 0;
	for(const std::uint32_t Start : ReroutedSinkIndices) {
		if(SelectedRerouted[Start]) ++SelectedReroutedSinks;
		std::uint32_t Current = Start, Steps = 0;
		while(!ReroutedNetwork->Nodes()[Current].Ocean && !SelectedRerouted[Current]) {
			if(!TributaryNeeded[Current]) {
				TributaryNeeded[Current] = 1;
				++ReroutedMissingEdges;
			}
			Current = ReroutedNetwork->Nodes()[Current].Downstream;
			// 下流の親が循環して大洋へ辿れない場合の返戻
			if(++Steps > Mesh->Directions.size()) return false;
		}
	}
	std::vector<std::uint32_t> ReroutedPromotionEdges;
	for(std::uint32_t Index = 0; Index < TributaryNeeded.size(); ++Index) {
		if(TributaryNeeded[Index]) ReroutedPromotionEdges.push_back(Index);
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage rerouted tributary paths: sinks=%u ") TEXT("selectedSinkNodes=%u missingEdges=%u."),
			ReroutedSinks,
			SelectedReroutedSinks,
			ReroutedMissingEdges
		)
	);
	const std::optional<PlanetCore::DrainageNetwork> ReroutedPromotedNetwork =
	ReroutedNetwork->WithPromotedTributaries(ReroutedPromotionEdges, *ReliefOcean);
	// 再経路化した排水木へ必要支流を接続の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("再経路化した排水木へ必要支流を接続"), ReroutedPromotedNetwork.has_value())) return false;
	const std::optional<PlanetCore::RiverTerrainField> ReroutedPromotedFinal = PlanetCore::RiverTerrainField::CreateWithErosion(
		ReliefOcean,
		std::make_shared<const PlanetCore::DrainageNetwork>(*ReroutedPromotedNetwork)
	);
	// 再経路化と支流接続から地形を再構築の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("再経路化と支流接続から地形を再構築"), ReroutedPromotedFinal.has_value())) return false;
	std::vector<double> ReroutedPromotedHeights(Mesh->Directions.size());
	std::vector<double> ReroutedPromotedNeighborMinimum(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
	double ReroutedPromotedMaximumLowering = 0, ReroutedPromotedMeanAbsoluteChange = 0;
	std::uint32_t ReroutedPromotedWorstIndex = 0;
	for(std::size_t Index = 0; Index < ReroutedPromotedHeights.size(); ++Index) {
		ReroutedPromotedHeights[Index] = ReroutedPromotedFinal->EvaluateElevation(Mesh->Directions[Index]);
		const double Delta = ReroutedPromotedHeights[Index] - ReliefHeights[2][Index];
		if(-Delta > ReroutedPromotedMaximumLowering) {
			ReroutedPromotedMaximumLowering = -Delta;
			ReroutedPromotedWorstIndex = static_cast<std::uint32_t>(Index);
		}
		ReroutedPromotedMeanAbsoluteChange += std::abs(Delta);
	}
	ReroutedPromotedMeanAbsoluteChange /= ReroutedPromotedHeights.size();
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::size_t Corner = 0; Corner < 3; ++Corner) {
		const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
		ReroutedPromotedNeighborMinimum[A] = std::min(ReroutedPromotedNeighborMinimum[A], ReroutedPromotedHeights[B]);
		ReroutedPromotedNeighborMinimum[B] = std::min(ReroutedPromotedNeighborMinimum[B], ReroutedPromotedHeights[A]);
	}
	std::uint32_t ReroutedPromotedSinks = 0, ReroutedPromotedRouteRises = 0;
	for(std::size_t Index = 0; Index < ReroutedPromotedHeights.size(); ++Index) {
		const PlanetCore::DrainageNode &Node = ReroutedPromotedFinal->Drainage().Nodes()[Index];
		if(Node.Ocean || ReroutedPromotedHeights[Index] <= Settings.SeaLevelMeters + 0.01) continue;
		if(ReroutedPromotedNeighborMinimum[Index] - ReroutedPromotedHeights[Index] > 0.01) ++ReroutedPromotedSinks;
		if(
			Node.Downstream < ReroutedPromotedHeights.size() && Node.Downstream != Index &&
			ReroutedPromotedHeights[Node.Downstream] - ReroutedPromotedHeights[Index] > 0.01
		) ++ReroutedPromotedRouteRises;
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage reroute plus tributaries: reaches=%u sinks=%u ")
			TEXT("routeRises=%u maxLowering=%.3fm meanAbs=%.3fm maxValleyHalfWidth=%.3fm."),
			static_cast<std::uint32_t>(ReroutedPromotedFinal->Drainage().Reaches().size()),
			ReroutedPromotedSinks,
			ReroutedPromotedRouteRises,
			ReroutedPromotedMaximumLowering,
			ReroutedPromotedMeanAbsoluteChange,
			ReroutedPromotedFinal->Statistics().MaximumValleyHalfWidthMeters
		)
	);
	const Counts ReroutedPromotedEscape = CountStrictEscape(ReroutedPromotedHeights);
	AddInfo(
		FString::Printf(
			TEXT("Predrainage strict descent rerouted: land=%u ")
			TEXT("noLowerNeighbor=%u trapped=%u submergedSourceLand=%u. Fixed graph only."),
			ReroutedPromotedEscape.Land,
			ReroutedPromotedEscape.NoLowerNeighbor,
			ReroutedPromotedEscape.Trapped,
			ReroutedPromotedEscape.SubmergedLand
		)
	);
	TestTrue(TEXT("再経路化と支流追加後の固定網で全陸地点が下降して大洋へ達する"), ReroutedPromotedEscape.Trapped == 0);
	TestTrue(TEXT("同条件で再経路化後の支流も１km級の開削を防ぐ"), ReroutedPromotedMaximumLowering < 350.0);
	const PlanetCore::DrainageNode &RerouteWorstBefore = ReroutedNetwork->Nodes()[ReroutedPromotedWorstIndex];
	const PlanetCore::DrainageNode &RerouteWorstAfter = ReroutedPromotedFinal->Drainage().Nodes()[ReroutedPromotedWorstIndex];
	AddInfo(
		FString::Printf(
			TEXT("Predrainage reroute promotion worst: node=%u source=%.3fm ")
			TEXT("baseline=%.3fm final=%.3fm originalWater=%.3fm ")
			TEXT("reroutedWater=%.3fm promotedWater=%.3fm profileLowering=%.3fm ")
			TEXT("preparedCut=%.3fm candidates=%u."),
			ReroutedPromotedWorstIndex,
			ReliefOcean->EvaluateElevation(Mesh->Directions[ReroutedPromotedWorstIndex]),
			ReliefHeights[2][ReroutedPromotedWorstIndex],
			ReroutedPromotedHeights[ReroutedPromotedWorstIndex],
			ReliefDrainage->Nodes()[ReroutedPromotedWorstIndex].WaterMeters,
			RerouteWorstBefore.WaterMeters,
			RerouteWorstAfter.WaterMeters,
			ReroutedPromotedFinal->Drainage().Statistics().MaximumProfileWaterLoweringMeters,
			ReroutedPromotedFinal->Statistics().MaximumPreparedCutMeters,
			static_cast<std::uint32_t>(ReroutedPromotedFinal->CandidateCount(Mesh->Directions[ReroutedPromotedWorstIndex]))
		)
	);
	std::uint32_t InteriorOceanReaches = 0, DirectNewDonorsAtWorst = 0;
	double LowestNewReachInterior = std::numeric_limits<double>::infinity();
	double LowestWorstDonorInterior = std::numeric_limits<double>::infinity();
	for(
		std::size_t ReachIndex = ReroutedNetwork->Reaches().size();
		ReachIndex < ReroutedPromotedNetwork->Reaches().size();
		++ReachIndex
	) {
		const PlanetCore::RiverReach &Reach = ReroutedPromotedNetwork->Reaches()[ReachIndex];
		if(ReroutedPromotedNetwork->Nodes()[Reach.Downstream].Ocean) continue;
		const bool DirectDonor = Reach.Downstream == ReroutedPromotedWorstIndex;
		if(DirectDonor) ++DirectNewDonorsAtWorst;
		bool WetInterior = false;
		for(unsigned Step = 1; Step < 64; ++Step) {
			const std::optional<PlanetCore::RiverProfileSample> ReachSample = ReroutedPromotedNetwork->SampleReach(ReachIndex, Step / 64.0);
			// 昇格した辺の内側方向を評価の失敗時に検査を続けない返戻
			if(!TestTrue(TEXT("昇格した辺の内側方向を評価"), ReachSample.has_value())) return false;
			const double Height = ReliefOcean->EvaluateElevation(*PlanetCore::UnitDirection::FromVector(ReachSample->Direction));
			LowestNewReachInterior = std::min(LowestNewReachInterior, Height);
			if(DirectDonor) LowestWorstDonorInterior = std::min(LowestWorstDonorInterior, Height);
			if(Height <= Settings.SeaLevelMeters) WetInterior = true;
		}
		if(WetInterior) ++InteriorOceanReaches;
	}
	AddInfo(
		FString::Printf(
			TEXT("Predrainage promoted edge interiors: wetLandLandReaches=%u ")
			TEXT("lowestHeight=%.3fm directNewDonorsAtWorst=%u lowestDirectDonor=%.3fm."),
			InteriorOceanReaches,
			LowestNewReachInterior,
			DirectNewDonorsAtWorst,
			LowestWorstDonorInterior
		)
	);
	std::shared_ptr<const PlanetCore::RiverTerrainField> Previous =
	std::make_shared<const PlanetCore::RiverTerrainField>(*ReroutedFinal);
	for(unsigned Pass = 2; Pass < 4; ++Pass) {
		/** 直前の候補地形から次の排水経路を選ぶ地点標高 */
		// 次の比較反復へ渡す前回の最終標高
		const auto LatestHeight = [&Previous](const PlanetCore::UnitDirection Direction) {
			return Previous->EvaluateElevation(Direction);
		};
		const std::optional<PlanetCore::DrainageNetwork> NextNetwork =
		PlanetCore::DrainageNetwork::CreateForHeight(*ReliefOcean, LatestHeight);
		// 反復する排水経路を再選択の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("反復する排水経路を再選択"), NextNetwork.has_value())) return false;
		std::uint32_t Changed = 0;
		for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			if(NextNetwork->Nodes()[Index].Downstream != Previous->Drainage().Nodes()[Index].Downstream) ++Changed;
		}
		const std::optional<PlanetCore::RiverTerrainField> NextFinal = PlanetCore::RiverTerrainField::CreateWithErosion(
			ReliefOcean,
			std::make_shared<const PlanetCore::DrainageNetwork>(*NextNetwork)
		);
		// 反復後の侵食と地形を再構築の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("反復後の侵食と地形を再構築"), NextFinal.has_value())) return false;
		std::vector<double> IterationHeights(Mesh->Directions.size());
		std::vector<double> NeighborMinimum(Mesh->Directions.size(), std::numeric_limits<double>::infinity());
		double IterationMaximumLowering = 0, IterationMeanAbsoluteChange = 0;
		for(std::size_t Index = 0; Index < IterationHeights.size(); ++Index) {
			IterationHeights[Index] = NextFinal->EvaluateElevation(Mesh->Directions[Index]);
			const double Delta = IterationHeights[Index] - ReliefHeights[2][Index];
			IterationMaximumLowering = std::max(IterationMaximumLowering, -Delta);
			IterationMeanAbsoluteChange += std::abs(Delta);
		}
		IterationMeanAbsoluteChange /= IterationHeights.size();
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::size_t Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
			NeighborMinimum[A] = std::min(NeighborMinimum[A], IterationHeights[B]);
			NeighborMinimum[B] = std::min(NeighborMinimum[B], IterationHeights[A]);
		}
		std::uint32_t IterationSinks = 0, IterationDeepSinks = 0, IterationRouteRises = 0;
		for(std::size_t Index = 0; Index < IterationHeights.size(); ++Index) {
			const PlanetCore::DrainageNode &Node = NextFinal->Drainage().Nodes()[Index];
			if(Node.Ocean || IterationHeights[Index] <= Settings.SeaLevelMeters + 0.01) continue;
			const double Depth = NeighborMinimum[Index] - IterationHeights[Index];
			if(Depth > 0.01) {
				++IterationSinks;
				if(Depth >= 20) ++IterationDeepSinks;
			}
			if(
				Node.Downstream < IterationHeights.size() && Node.Downstream != Index &&
				IterationHeights[Node.Downstream] - IterationHeights[Index] > 0.01
			) ++IterationRouteRises;
		}
		AddInfo(
			FString::Printf(
				TEXT("Predrainage reroute iteration=%u changedEdges=%u reaches=%u ")
				TEXT("sinks=%u deepSinks=%u routeRises=%u maxLowering=%.3fm meanAbs=%.3fm."),
				Pass,
				Changed,
				static_cast<std::uint32_t>(NextFinal->Drainage().Reaches().size()),
				IterationSinks,
				IterationDeepSinks,
				IterationRouteRises,
				IterationMaximumLowering,
				IterationMeanAbsoluteChange
			)
		);
		Previous = std::make_shared<const PlanetCore::RiverTerrainField>(*NextFinal);
	}
	const PlanetCore::UnitDirection WorstDirection = Mesh->Directions[MaximumLoweringIndex];
	AddInfo(
		FString::Printf(
			TEXT("Predrainage promotion worst cut: node=%u original=%.3fm ")
			TEXT("baseline=%.3fm promoted=%.3fm baselineWater=%.3fm promotedWater=%.3fm ")
			TEXT("candidates=%u/%u baselineValleyHalfWidth=%.3fm baselinePreparedCut=%.3fm."),
			MaximumLoweringIndex,
			ReliefOcean->EvaluateElevation(WorstDirection),
			ReliefHeights[2][MaximumLoweringIndex],
			PromotedHeights[MaximumLoweringIndex],
			ReliefFinal->Drainage().Nodes()[MaximumLoweringIndex].WaterMeters,
			PromotedFinal->Drainage().Nodes()[MaximumLoweringIndex].WaterMeters,
			static_cast<std::uint32_t>(ReliefFinal->CandidateCount(WorstDirection)),
			static_cast<std::uint32_t>(PromotedFinal->CandidateCount(WorstDirection)),
			ReliefFinal->Statistics().MaximumValleyHalfWidthMeters,
			ReliefFinal->Statistics().MaximumPreparedCutMeters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Predrainage meso source/valley/final sinks=%u/%u/%u ")
			TEXT("deepSinks=%u/%u/%u routeRises=%u/%u/%u deepest=%.3f/%.3f/%.3fm."),
			ReliefSinks[0],
			ReliefSinks[1],
			ReliefSinks[2],
			ReliefDeepSinks[0],
			ReliefDeepSinks[1],
			ReliefDeepSinks[2],
			ReliefRouteRises[0],
			ReliefRouteRises[1],
			ReliefRouteRises[2],
			Deepest[0],
			Deepest[1],
			Deepest[2]
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Predrainage meso integration: originalReaches=%u finalReaches=%u ")
			TEXT("oceanFraction=%.6f duplicateMesoSamples=%u. Fixed graph only."),
			ReliefDrainage->Statistics().Reaches,
			ReliefFinal->Drainage().Statistics().Reaches,
			ReliefOcean->Statistics().IndependentOceanFractionEstimate,
			DuplicateMesoSamples
		)
	);
	// 要求した全比較の実行を終えた結果の返戻；地形品質の合格とは区別する
	return true;
}

#endif
