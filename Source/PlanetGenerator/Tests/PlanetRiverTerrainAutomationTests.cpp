#include "Core/PlanetEvaluationCache.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetView.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetRiverTerrainContracts, "PlanetGenerator.RiverTerrain.Contracts",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 河床・法線・近景分割・足場を本番の評価関数で検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を完遂した場合に真，失敗は検査器へ記録
 */
bool FPlanetRiverTerrainContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::AdaptiveSphereMesh;
	using PlanetCore::BuildAdaptiveSphere;
	using PlanetCore::BuildGroundPatch;
	using PlanetCore::DrainageNetwork;
	using PlanetCore::EvaluationCacheStats;
	using PlanetCore::GeologyField;
	using PlanetCore::GeologySettings;
	using PlanetCore::GroundFrame;
	using PlanetCore::GroundPatch;
	using PlanetCore::GroundPatchSettings;
	using PlanetCore::LodSelection;
	using PlanetCore::LodSurfaceCache;
	using PlanetCore::LodSurfacePreparation;
	using PlanetCore::LodView;
	using PlanetCore::OceanConnectivityField;
	using PlanetCore::RiverProfileSample;
	using PlanetCore::RiverSurfaceSample;
	using PlanetCore::RiverTerrainChannel;
	using PlanetCore::RiverTerrainField;
	using PlanetCore::RiverWaterHeights;
	using PlanetCore::SelectLodForView;
	using PlanetCore::SelectRenderFrame;
	using PlanetCore::SurfaceHeightEvaluator;
	using PlanetCore::SurfacePreparationProgress;
	using PlanetCore::TerrainEvaluationCache;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	const double Begin = FPlatformTime::Seconds();
	std::uint64_t Centers = 0, Normals = 0, IndexChecks = 0, ScalarChecks = 0, ReliefChecks = 0;
	TestFalse(TEXT("Missing fields rejected"), RiverTerrainField::Create({}, {}).has_value());
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		GeologySettings S;
		S.RadiusMeters = Radius;
		const double AdditionalReliefBound = 335 * Radius / (Radius + 250000);
		std::optional<GeologyField> Geo = GeologyField::Create(S);
		const bool Rejected1 = !Geo;
		// 不合格又は準備失敗を保持する返戻
		if(Rejected1) return false;
		std::optional<OceanConnectivityField> Ocean =
		OceanConnectivityField::Create(std::make_shared<const GeologyField>(std::move(*Geo)));
		const bool Rejected2 = !Ocean;
		// 不合格又は準備失敗を保持する返戻
		if(Rejected2) return false;
		const std::shared_ptr<const OceanConnectivityField> Base = std::make_shared<const OceanConnectivityField>(std::move(*Ocean));
		std::optional<DrainageNetwork> Drain = DrainageNetwork::Create(*Base);
		const bool Rejected3 = !Drain;
		// 不合格又は準備失敗を保持する返戻
		if(Rejected3) return false;
		const std::shared_ptr<const DrainageNetwork> Network = std::make_shared<const DrainageNetwork>(std::move(*Drain));
		std::optional<RiverTerrainField> PreparedField = RiverTerrainField::Create(Base, Network);
		const bool Rejected4 = !TestTrue(TEXT("River terrain prepared"), PreparedField.has_value());
		// 不合格又は準備失敗を保持する返戻
		if(Rejected4) return false;
		const std::shared_ptr<const RiverTerrainField> Field = std::make_shared<const RiverTerrainField>(std::move(*PreparedField));
		for(unsigned k = 0; k < 32; ++k) {
			const RiverTerrainChannel &Reference = Field->Channels()[k * Field->Channels().size() / 32];
			const Vec3 Padding { k % 3 * 0.02, k % 3 * 0.02, k % 3 * 0.02 };
			const Vec3 Min = Reference.Center - Padding, Max = Reference.Center + Padding;
			std::vector<std::uint32_t> Actual, Expected;
			TestTrue(TEXT("河道の包囲箱照会が成功"), Field->GatherChannelCandidates(Min, Max, Actual));
			for(std::uint32_t i = 0; i < Field->Channels().size(); ++i) {
				const RiverTerrainChannel &Channel = Field->Channels()[i];
				if(
					Min.X <= Channel.BoundsMax.X && Max.X >= Channel.BoundsMin.X && Min.Y <= Channel.BoundsMax.Y && Max.Y >= Channel.BoundsMin.Y &&
					Min.Z <= Channel.BoundsMax.Z && Max.Z >= Channel.BoundsMin.Z
				) Expected.push_back(i);
			}
			std::sort(Actual.begin(), Actual.end());
			TestTrue(TEXT("包囲箱索引が全河道走査と完全一致"), Actual == Expected);
		}
		double MaximumCorridorNormalError = 0, MaximumCorridorGrade = 0;
		for(unsigned i = 0; i < 8; ++i) {
			const RiverTerrainChannel &Channel = Field->Channels()[i * Field->Channels().size() / 8];
			const bool RejectedDetail1 = !TestTrue(
				TEXT("Meso corridor extends beyond the river valley"),
				Channel.MesoInnerMeters > Channel.OuterHalfWidthMeters && Channel.MesoOuterMeters > Channel.MesoInnerMeters
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail1) return false;
			const std::optional<PlanetCore::RiverPlanformSample> Curve = Channel.Planform ?
			std::optional<PlanetCore::RiverPlanformSample>{ Channel.Planform->EvaluateAngle(0.5 * Channel.ArcAngle) } :
			std::nullopt;
			const Vec3 Along = Curve ?
			Curve->Direction.Vector() :
			Channel.Start * std::cos(0.5 * Channel.ArcAngle) + Channel.Tangent * std::sin(0.5 * Channel.ArcAngle);
			const Vec3 Across =
			UnitDirection::FromVector(Curve ? Along.Cross(Curve->DerivativePerFraction) : Channel.Start.Cross(Channel.Tangent))->Vector();
			for(const double Fraction : { 0.0, 0.5, 1.0 }) {
				const double Distance = Channel.MesoInnerMeters + Fraction * (Channel.MesoOuterMeters - Channel.MesoInnerMeters);
				const double Angle = Distance / Radius;
				const UnitDirection Direction = *UnitDirection::FromVector(Along * std::cos(Angle) + Across * std::sin(Angle));
				const TerrainSample Indexed = Field->Evaluate(Direction), Reference = Field->EvaluateReference(Direction);
				const bool RejectedDetail2 = !TestTrue(
					TEXT("Meso corridor indexed and full-scan heights agree"),
					Indexed.ElevationMeters == Reference.ElevationMeters &&
					(Indexed.TangentialGradientPerRadian - Reference.TangentialGradientPerRadian).Length() == 0
				);
				// 不合格の独立検査を保持する返戻
				if(RejectedDetail2) return false;
				if(Fraction != 0.5) continue;
				const double Step = std::min(1E-6, 0.25 / Radius);
				const UnitDirection Before = *UnitDirection::FromVector(Direction.Vector() * std::cos(Step) - Across * std::sin(Step));
				const UnitDirection After = *UnitDirection::FromVector(Direction.Vector() * std::cos(Step) + Across * std::sin(Step));
				const double FiniteDifference = (Field->EvaluateElevation(After) - Field->EvaluateElevation(Before)) / (2 * Step);
				const double Analytic = Indexed.TangentialGradientPerRadian.Dot(Across);
				const double NormalError = std::abs(std::atan(FiniteDifference / Radius) - std::atan(Analytic / Radius));
				MaximumCorridorNormalError = std::max(MaximumCorridorNormalError, NormalError);
				MaximumCorridorGrade = std::max(MaximumCorridorGrade, std::abs(Analytic / Radius));
				const bool RejectedDetail3 =
				!TestTrue(TEXT("Meso corridor analytical normal follows independent height differences"), NormalError < 1E-5);
				// 不合格の独立検査を保持する返戻
				if(RejectedDetail3) return false;
			}
		}
		AddInfo(
			FString::Printf(
				TEXT("Meso corridor R=%.0f normalError=%.9g maximumGrade=%.9g"),
				Radius,
				MaximumCorridorNormalError,
				MaximumCorridorGrade
			)
		);
		std::vector<std::uint32_t> InvalidCandidates { 123 };
		TestFalse(TEXT("逆転した照会範囲を拒否"), Field->GatherChannelCandidates({ 1, 1, 1 }, { -1, -1, -1 }, InvalidCandidates));
		TestTrue(TEXT("不正照会に古い候補を残さない"), InvalidCandidates.empty());
		/**
		 * 値だけの評価と完全評価の標高を照合する
		 * @param D 評価方向
		 * @param Expected 完全評価の標高
		 * @return 倍精度のビット列が一致する場合に真
		 */
		const auto SameHeight = [&](const UnitDirection D, const double Expected) -> bool {
			++ScalarChecks;
			const RiverWaterHeights Water = Field->EvaluateWater(D);
			const RiverSurfaceSample Reference = Field->EvaluateSurface(D);
			// 独立検査に必要な値又は不合格の返戻
			return std::bit_cast<std::uint64_t>(Field->EvaluateElevation(D)) == std::bit_cast<std::uint64_t>(Expected) &&
			std::bit_cast<std::uint64_t>(Water.TerrainElevationMeters) == std::bit_cast<std::uint64_t>(Expected) &&
			std::bit_cast<std::uint64_t>(Water.WaterElevationMeters) == std::bit_cast<std::uint64_t>(Reference.WaterElevationMeters) &&
			std::bit_cast<std::uint64_t>(Water.WaterSupport) == std::bit_cast<std::uint64_t>(Reference.WaterSupport);
		};
		// 河道・河岸・合流点・河口・支持境界に加え，独立な全球地点でも評価する
		std::vector<UnitDirection> PerformancePoints;
		PerformancePoints.reserve(512);
		for(unsigned I = 0; I < 512; ++I) {
			const double Z = 1 - 2 * (I + 0.5) / 512, A = I * 2.399963229728653, R = std::sqrt(1 - Z * Z);
			const UnitDirection D = *UnitDirection::FromVector({ R * std::cos(A), R * std::sin(A), Z });
			const bool RejectedDetail4 =
			!TestTrue(TEXT("Global scalar river matches full evaluation"), SameHeight(D, Field->Evaluate(D).ElevationMeters));
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail4) return false;
			PerformancePoints.push_back(D);
			if(Radius == 1E6 && !ReliefChecks && Base->Evaluate(D).ElevationMeters > 20 && !Field->CandidateCount(D)) {
				const TerrainSample Ground = Field->Evaluate(D);
				const double Lift = Ground.ElevationMeters - Base->Evaluate(D).ElevationMeters;
				Vec3 Axis = D.Vector().Cross({ 0, 0, 1 });
				if(Axis.Length() < 0.1) Axis = D.Vector().Cross({ 0, 1, 0 });
				const UnitDirection Tangent = *UnitDirection::FromVector(Axis);
				const double Step = 1 / Radius;
				const UnitDirection Plus = *UnitDirection::FromVector(D.Vector() * std::cos(Step) + Tangent.Vector() * std::sin(Step));
				const UnitDirection Minus = *UnitDirection::FromVector(D.Vector() * std::cos(Step) - Tangent.Vector() * std::sin(Step));
				const double Difference = (Field->EvaluateElevation(Plus) - Field->EvaluateElevation(Minus)) / (2 * Radius * Step);
				const double Analytic = Ground.TangentialGradientPerRadian.Dot(Tangent.Vector()) / Radius;
				const bool RejectedDetail5 = !TestTrue(
					TEXT("Land relief is bounded and its normal follows the same height field"),
					std::abs(Lift) > 0.01 && std::abs(Lift) <= 5 + AdditionalReliefBound && std::abs(Difference - Analytic) < 0.005
				);
				// 不合格の独立検査を保持する返戻
				if(RejectedDetail5) return false;
				++ReliefChecks;
			}
		}
		const std::optional<RiverProfileSample> NearReach = Network->SampleReach(Radius == 1000 ? 842 : 915, 0.5);
		const bool Rejected5 = !TestTrue(TEXT("Performance sample reach exists"), NearReach.has_value());
		// 不合格又は準備失敗を保持する返戻
		if(Rejected5) return false;
		const std::optional<GroundFrame> NearFrame = GroundFrame::Create(*UnitDirection::FromVector(NearReach->Direction), Radius, 0);
		for(unsigned I = 0; I < 512; ++I) {
			const UnitDirection D = NearFrame->DirectionAt((static_cast<int>(I % 32) - 16) * 1.5, (static_cast<int>(I / 32) - 8) * 3.0);
			const bool RejectedDetail6 =
			!TestTrue(TEXT("Near scalar river matches full evaluation"), SameHeight(D, Field->Evaluate(D).ElevationMeters));
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail6) return false;
			PerformancePoints.push_back(D);
		}
		{
			TerrainEvaluationCache Exact(Field), Evicting(Field, 17, 23), Uncached(Field, 0, 0);
			Exact.GrowWaterSamples(1024);
			Evicting.GrowWaterSamples(19);
			const auto Bits = [](const double A, const double B) {
				// 独立検査に必要な値又は不合格の返戻
				return std::bit_cast<std::uint64_t>(A) == std::bit_cast<std::uint64_t>(B);
			};
			for(unsigned Pass = 0; Pass < 3; ++Pass) for(std::size_t I = 0; I < PerformancePoints.size(); ++I) {
				const UnitDirection D = PerformancePoints[Pass == 2 ? PerformancePoints.size() - 1 - I : I];
				const RiverSurfaceSample ReferenceSurface = Field->EvaluateSurface(D);
				const TerrainSample Reference = ReferenceSurface.Terrain;
				const double ReferenceH = Field->EvaluateElevation(D);
				for(TerrainEvaluationCache *const C : { &Exact, &Evicting, &Uncached }) {
					const TerrainSample V = C->Evaluate(D);
					const RiverWaterHeights Water = C->EvaluateWater(D);
					// 不合格又は準備失敗を後続へ持込まない返戻
					const bool RejectedDetail7 = !TestTrue(
						TEXT("Scalar water cache preserves height, water and support bits"),
						Bits(Water.TerrainElevationMeters, Reference.ElevationMeters) &&
						Bits(Water.WaterElevationMeters, ReferenceSurface.WaterElevationMeters) &&
						Bits(Water.WaterSupport, ReferenceSurface.WaterSupport)
					);
					// 不合格の独立検査を保持する返戻
					if(RejectedDetail7) return false;
					const double H = C->EvaluateElevation(D);
					// 不合格又は準備失敗を後続へ持込まない返戻
					const bool RejectedDetail8 = !TestTrue(
						TEXT("Cold/warm/evicted/reordered exact cache preserves all sample bits"),
						Bits(V.ElevationMeters, Reference.ElevationMeters) && Bits(H, ReferenceH) &&
						Bits(V.TangentialGradientPerRadian.X, Reference.TangentialGradientPerRadian.X) &&
						Bits(V.TangentialGradientPerRadian.Y, Reference.TangentialGradientPerRadian.Y) &&
						Bits(V.TangentialGradientPerRadian.Z, Reference.TangentialGradientPerRadian.Z)
					);
					// 不合格の独立検査を保持する返戻
					if(RejectedDetail8) return false;
				}
			}
			const EvaluationCacheStats Counts = Exact.Statistics();
			const bool RejectedDetail9 = !TestTrue(
				TEXT("Repeated exact directions avoid new evaluations"),
				Counts.SampleHits > 2047 && Counts.HeightHits > 2047 && Counts.Samples < 1025 && Counts.Heights < 1025 &&
				Counts.WaterHits > 2047 && Counts.WaterSamples < 1025
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail9) return false;
			const bool RejectedDetail10 = !TestTrue(
				TEXT("Evaluation cache capacity remains bounded including bypass"),
				Evicting.SampleCount() == 17 && Evicting.HeightCount() == 23 && Uncached.SampleCount() == 0 && Uncached.HeightCount() == 0 &&
				Evicting.WaterSampleCount() == 19 && Uncached.WaterSampleCount() == 0
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail10) return false;
			TerrainEvaluationCache Growing(Field, 17, 0);
			for(unsigned I = 0; I < 17; ++I) Growing.EvaluateSurface(PerformancePoints[I]);
			const EvaluationCacheStats BeforeGrowth = Growing.Statistics();
			Growing.GrowSamples(37);
			TestEqual(TEXT("保持枠の拡張時に既存の評価値を捨てない"), Growing.SampleCount(), std::size_t{ 17 });
			for(unsigned I = 0; I < 17; ++I) {
				const PlanetCore::RiverSurfaceSample Reference = Field->EvaluateSurface(PerformancePoints[I]);
				const PlanetCore::RiverSurfaceSample Cached = Growing.EvaluateSurface(PerformancePoints[I]);
				TestTrue(
					TEXT("保持枠の拡張後も水位・水域・地形のビットが一致"),
					Bits(Reference.Terrain.ElevationMeters, Cached.Terrain.ElevationMeters) &&
					Bits(Reference.WaterElevationMeters, Cached.WaterElevationMeters) && Bits(Reference.WaterSupport, Cached.WaterSupport)
				);
			}
			TestEqual(TEXT("拡張後の既存標本は再計算しない"), Growing.Statistics().Samples, BeforeGrowth.Samples);
			for(unsigned I = 0; I < 64; ++I) Growing.EvaluateSurface(PerformancePoints[I]);
			TestEqual(TEXT("拡張後も指定した保持件数で置換"), Growing.SampleCount(), std::size_t{ 37 });
			Growing.GrowSamples(TerrainEvaluationCache::MaximumSamples + 1);
			TestEqual(TEXT("拡張要求も固定の最大件数で制限"), Growing.SampleCapacity(), TerrainEvaluationCache::MaximumSamples);
			Growing.GrowSamples(0);
			TestEqual(TEXT("小さい拡張要求で既存枠を縮小しない"), Growing.SampleCapacity(), TerrainEvaluationCache::MaximumSamples);
			TestEqual(TEXT("通常の既定保持枠は維持"), Exact.SampleCapacity(), TerrainEvaluationCache::DefaultSamples);
			const UnitDirection Anchor = PerformancePoints.back();
			const double H = Field->EvaluateElevation(Anchor);
			const std::optional<GroundFrame> Frame = SelectRenderFrame(Anchor, Radius, H);
			const bool Rejected6 = !TestTrue(TEXT("Initial rendering origin exists"), Frame.has_value());
			// 不合格又は準備失敗を保持する返戻
			if(Rejected6) return false;
			const UnitDirection Nearby = Frame->DirectionAt(64, 0);
			const std::optional<GroundFrame> Retained = SelectRenderFrame(Nearby, Radius, H, Frame);
			const UnitDirection Distant = Frame->DirectionAt(131, 0);
			const std::optional<GroundFrame> Rebased = SelectRenderFrame(Distant, Radius, H, Frame);
			const std::optional<GroundFrame> Raised = SelectRenderFrame(Anchor, Radius, H + 129, Frame);
			const bool RejectedDetail11 = !TestTrue(
				TEXT("Origin is retained nearby and rebased by distance or altitude"),
				Retained && Rebased && Raised && Bits(Retained->Up.X, Frame->Up.X) && Bits(Retained->Up.Y, Frame->Up.Y) &&
				Bits(Retained->Up.Z, Frame->Up.Z) && (Rebased->Up - Frame->Up).Length() > 0 && Raised->AnchorElevationMeters == H + 129
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail11) return false;
			TestFalse(TEXT("Invalid render frame radius rejected"), SelectRenderFrame(Anchor, 0, H, Frame).has_value());
			AddInfo(
				FString::Printf(
					TEXT(
						"Exact evaluation cache R=%.9g samples=%llu hits=%llu heights=%llu hits=%llu; eviction/reordering/bypass "
						"bits "
						"match; 64m retention and 131m rebase checked"
					),
					Radius,
					static_cast<unsigned long long>(Counts.Samples),
					static_cast<unsigned long long>(Counts.SampleHits),
					static_cast<unsigned long long>(Counts.Heights),
					static_cast<unsigned long long>(Counts.HeightHits)
				)
			);
		}
		volatile double Checksum = 0;
		std::array<double, 2> Times {};
		for(unsigned Pass = 0; Pass < 2; ++Pass) {
			const double Start = FPlatformTime::Seconds();
			for(const UnitDirection D : PerformancePoints) {
				Checksum = Checksum + (!Pass ? Field->Evaluate(D).ElevationMeters : Field->EvaluateElevation(D));
			}
			Times[Pass] = (FPlatformTime::Seconds() - Start) * 1000;
		}
		AddInfo(
			FString::Printf(
				TEXT("Scalar terrain R=%.9g full=%.3f ms scalar=%.3f ms checksum=%.9g"),
				Radius,
				Times[0],
				Times[1],
				static_cast<double>(Checksum)
			)
		);
		std::stop_source Stop;
		Stop.request_stop();
		TestFalse(TEXT("Cancelled river preparation rejected"), RiverTerrainField::Create(Base, Network, Stop.get_token()).has_value());
		if(Radius == 1000) {
			// 旧版で 53.8° の河岸に停止した身体位置と向きから，合成地形の近傍を検査する
			const Vec3 Stopped { 823.7846000179857, 507.7657612806537, -264.3599423770205 };
			const UnitDirection StoppedUp = *UnitDirection::FromVector(Stopped);
			const Vec3 Heading { 4.940781568763943, -7.50949595608013, 0.4428859620514302 };
			const UnitDirection Forward = *UnitDirection::FromVector(Heading - StoppedUp.Vector() * Heading.Dot(StoppedUp.Vector()));
			const Vec3 Across = StoppedUp.Vector().Cross(Forward.Vector());
			double MaximumBankTilt = 0;
			for(int Along = -8; Along < 33; ++Along) for(int Side = -4; Side < 5; ++Side) {
				const UnitDirection D = *UnitDirection::FromVector(Stopped + Forward.Vector() * (Along * 0.125) + Across * (Side * 0.125));
				const TerrainSample Sample = Field->Evaluate(D);
				const double Tilt = std::atan(Sample.TangentialGradientPerRadian.Length() / (Radius + Sample.ElevationMeters));
				MaximumBankTilt = std::max(MaximumBankTilt, Tilt);
				const bool RejectedDetail12 = !TestTrue(
					TEXT("Captured shallow bank is finite and below 40 degrees"),
					std::isfinite(Tilt) && Tilt < 40 * 3.14159265358979323846 / 180
				);
				// 不合格の独立検査を保持する返戻
				if(RejectedDetail12) return false;
			}
			AddInfo(
				FString::Printf(
					TEXT("Captured shallow bank: 369 composed samples, maximum slope %.9g degrees"),
					MaximumBankTilt * 180 / 3.14159265358979323846
				)
			);
			const std::optional<RiverProfileSample> River = Network->SampleReach(842, 0.5);
			const bool Rejected7 = !TestTrue(TEXT("Small-radius rendering regression reach exists"), River.has_value());
			// 不合格又は準備失敗を保持する返戻
			if(Rejected7) return false;
			const UnitDirection Anchor = *UnitDirection::FromVector(River->Direction * (Radius + River->WaterMeters));
			const double H = Field->Evaluate(Anchor).ElevationMeters;
			const GroundFrame Frame = *GroundFrame::Create(Anchor, Radius, H);
			LodView View;
			View.RadiusMeters = Radius;
			View.ReliefBoundMeters = Field->AbsoluteReliefBoundMeters();
			View.ObserverMeters = Anchor.Vector() * (Radius + H + 1.6);
			View.SurfaceRadiusMeters = 24;
			View.LeafBudget = LodView::MaximumViewLeaves;
			const SurfaceHeightEvaluator Height = [&](const UnitDirection D) {
				return Field->EvaluateElevation(D);
			};
			LodSurfaceCache Cache(Radius, Height);
			const std::optional<LodSelection> Selection = SelectLodForView(View, {}, {}, &Cache);
			if(Selection) {
				AddInfo(
					FString::Printf(
						TEXT("Small river selection: leaves=%llu edge=%.9g raw_error=%.9g stitched_error=%.9g splits=%u passes=%u"),
						static_cast<unsigned long long>(Selection->Tree.Leaves().size()),
						Selection->MaximumSurfaceEdgeMeters,
						Selection->MaximumSurfaceErrorMeters,
						Selection->MaximumStitchErrorMeters,
						Selection->StitchRefinedParents,
						Selection->StitchRefinementPasses
					)
				);
			}
			const bool RejectedDetail13 =
			!TestTrue(TEXT("Actual small river reaches stitched near accuracy"), Selection && Selection->SurfaceResolutionSatisfied);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail13) return false;
			LodView EagerView = View;
			EagerView.DeferCoarseSurfaceProbes = false;
			LodSurfaceCache EagerCache(Radius, Height);
			const std::optional<LodSelection> Eager = SelectLodForView(EagerView, {}, {}, &EagerCache);
			const bool RejectedDetail14 = !TestTrue(
				TEXT("Deferring forced splits preserves real river final partition and accuracy"),
				Eager && Eager->SurfaceResolutionSatisfied && Eager->Tree.Leaves() == Selection->Tree.Leaves() &&
				Eager->MaximumSurfaceErrorMeters == Selection->MaximumSurfaceErrorMeters &&
				Eager->MaximumStitchErrorMeters == Selection->MaximumStitchErrorMeters &&
				Selection->SurfaceHeightEvaluations < Eager->SurfaceHeightEvaluations
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail14) return false;
			AddInfo(
				FString::Printf(
					TEXT("Deferred coarse river probes: evaluations=%llu eager=%llu identical final leaves=%llu"),
					static_cast<unsigned long long>(Selection->SurfaceHeightEvaluations),
					static_cast<unsigned long long>(Eager->SurfaceHeightEvaluations),
					static_cast<unsigned long long>(Selection->Tree.Leaves().size())
				)
			);
			const std::optional<AdaptiveSphereMesh> Mesh = BuildAdaptiveSphere(Selection->Tree);
			const bool Rejected8 = !TestTrue(TEXT("Actual small river has a shared mesh"), Mesh.has_value());
			// 不合格又は準備失敗を保持する返戻
			if(Rejected8) return false;
			const UnitDirection Shifted = Frame.DirectionAt(64, 0);
			const GroundFrame DisplacedFrame = *GroundFrame::Create(Shifted, Radius, Height(Shifted));
			TerrainEvaluationCache Exact(Field);
			for(const GroundFrame &RenderFrame : { Frame, DisplacedFrame }) {
				const Vec3 FocusLocal = RenderFrame.SurfaceToLocal(Anchor, H);
				std::vector<Vec3> Positions;
				for(const UnitDirection D : Mesh->Mesh.Directions) {
					const Vec3 P = RenderFrame.SurfaceToLocal(D, Exact.Evaluate(D).ElevationMeters);
					Positions.push_back(
						{
							static_cast<double>(static_cast<float>(P.X * 100)) / 100,
							static_cast<double>(static_cast<float>(P.Y * 100)) / 100,
							static_cast<double>(static_cast<float>(P.Z * 100)) / 100
						}
					);
				}
				double Maximum = 0, IndependentMaximum = 0;
				std::uint64_t Checked = 0;
				const double R0 = Radius + RenderFrame.AnchorElevationMeters;
				for(const std::array<std::uint32_t, 3> T : Mesh->Mesh.Triangles) {
					const Vec3 A = Positions[T[0]], B = Positions[T[1]], C = Positions[T[2]], Center = (A + B + C) * (1.0 / 3);
					const double Cap = std::max({ (A - Center).Length(), (B - Center).Length(), (C - Center).Length() });
					if((Center - FocusLocal).Length() > 32 + Cap) continue;
					unsigned K = 0;
					for(
						const Vec3 Q :
						{ (A + B) * 0.5, (B + C) * 0.5, (C + A) * 0.5, Center, A * 0.2 + B * 0.3 + C * 0.5, A * 0.6 + B * 0.1 + C * 0.3 }
					) {
						const bool bIndependent = K++ > 3;
						const UnitDirection D = *UnitDirection::FromVector(RenderFrame.LocalToPlanet(Q));
						if((D.Vector() - Anchor.Vector()).Length() * Radius > 24) continue;
						const double Increment = 2 * R0 * Q.Z + Q.Dot(Q);
						const double Approximate = RenderFrame.AnchorElevationMeters + Increment / (std::sqrt(R0 * R0 + Increment) + R0);
						const double ReferenceHeight = Height(D), Cached = Exact.EvaluateElevation(D), Warm = Exact.EvaluateElevation(D);
						// 不合格又は準備失敗を後続へ持込まない返戻
						const bool RejectedDetail15 = !TestTrue(
							TEXT("Actual float probes retain exact heights across render origins"),
							std::bit_cast<std::uint64_t>(ReferenceHeight) == std::bit_cast<std::uint64_t>(Cached) &&
							std::bit_cast<std::uint64_t>(ReferenceHeight) == std::bit_cast<std::uint64_t>(Warm)
						);
						// 不合格の独立検査を保持する返戻
						if(RejectedDetail15) return false;
						const double Error = std::abs(Approximate - Cached);
						++Checked;
						if(bIndependent) IndependentMaximum = std::max(IndependentMaximum, Error);
						else Maximum = std::max(Maximum, Error);
					}
				}
				AddInfo(
					FString::Printf(
						TEXT(
							"Small river render regression: frame_offset=%.9g m leaves=%llu "
							"stitched_splits=%u passes=%u probes=%llu "
							"render_error=%.9g m independent_error=%.9g m"
						),
						FocusLocal.Length(),
						static_cast<unsigned long long>(Selection->Tree.Leaves().size()),
						Selection->StitchRefinedParents,
						Selection->StitchRefinementPasses,
						static_cast<unsigned long long>(Checked),
						Maximum,
						IndependentMaximum
					)
				);
				const bool RejectedDetail16 =
				!TestTrue(TEXT("Final float river mesh meets unchanged 2cm probes"), Maximum <= 0.02 && IndependentMaximum <= 0.02);
				// 不合格の独立検査を保持する返戻
				if(RejectedDetail16) return false;
			}
			// 必須条件は修復回数でなく最終描画の独立誤差とする
			// 190 m の移動で遅れた実地点を，同じ分割予算の前景処理で比較する
			LodView Heavy = View;
			Heavy.ObserverMeters = { 815.2935697050591, 522.9983743524768, -265.26553782749573 };
			Heavy.SurfaceObserverMeters = Heavy.ObserverMeters;
			Heavy.FocalLengthPixels = 482.5;
			LodSurfaceCache HeavyCold(Radius, Height);
			const double ColdStart = FPlatformTime::Seconds();
			const std::optional<LodSelection> ColdHeavy = SelectLodForView(Heavy, {}, {}, &HeavyCold);
			const double ColdMs = (FPlatformTime::Seconds() - ColdStart) * 1000;
			const std::shared_ptr<LodSurfaceCache> HeavyCache = std::make_shared<LodSurfaceCache>(Radius, Height);
			const std::shared_ptr<LodSurfacePreparation> Preparation = LodSurfacePreparation::Create(Heavy, HeavyCache);
			const bool Rejected9 = !TestTrue(TEXT("Heavy-region preparation created"), Preparation != nullptr);
			// 不合格又は準備失敗を保持する返戻
			if(Rejected9) return false;
			const double PrepareStart = FPlatformTime::Seconds();
			SurfacePreparationProgress Progress;
			do Progress = Preparation->Advance(127);
			while(!Progress.Complete);
			const double PrepareMs = (FPlatformTime::Seconds() - PrepareStart) * 1000, WarmStart = FPlatformTime::Seconds();
			const std::optional<LodSelection> PreparedHeavy = SelectLodForView(Heavy, {}, {}, HeavyCache.get());
			const double WarmMs = (FPlatformTime::Seconds() - WarmStart) * 1000;
			AddInfo(
				FString::Printf(
					TEXT("Heavy curved river diagnostic: valid=%d exhausted=%d visits=%llu cold=%d prepared=%d ")
					TEXT("cold_satisfied=%d prepared_satisfied=%d leaves_equal=%d cold_leaves=%llu prepared_leaves=%llu ")
					TEXT("cold_error=%.17g prepared_error=%.17g cold_stitch=%.17g prepared_stitch=%.17g ")
					TEXT("cold_H=%llu prepared_H=%llu cold_reclaimed=%u prepared_reclaimed=%u"),
					Progress.Valid,
					Progress.BudgetExhausted,
					static_cast<unsigned long long>(Progress.VisitedTiles),
					ColdHeavy.has_value(),
					PreparedHeavy.has_value(),
					ColdHeavy && ColdHeavy->SurfaceResolutionSatisfied,
					PreparedHeavy && PreparedHeavy->SurfaceResolutionSatisfied,
					ColdHeavy && PreparedHeavy && ColdHeavy->Tree.Leaves() == PreparedHeavy->Tree.Leaves(),
					static_cast<unsigned long long>(ColdHeavy ? ColdHeavy->Tree.Leaves().size() : 0),
					static_cast<unsigned long long>(PreparedHeavy ? PreparedHeavy->Tree.Leaves().size() : 0),
					ColdHeavy ? ColdHeavy->MaximumSurfaceErrorMeters : -1,
					PreparedHeavy ? PreparedHeavy->MaximumSurfaceErrorMeters : -1,
					ColdHeavy ? ColdHeavy->MaximumStitchErrorMeters : -1,
					PreparedHeavy ? PreparedHeavy->MaximumStitchErrorMeters : -1,
					static_cast<unsigned long long>(ColdHeavy ? ColdHeavy->SurfaceHeightEvaluations : 0),
					static_cast<unsigned long long>(PreparedHeavy ? PreparedHeavy->SurfaceHeightEvaluations : 0),
					ColdHeavy ? ColdHeavy->ReclaimedDistantParents : 0,
					PreparedHeavy ? PreparedHeavy->ReclaimedDistantParents : 0
				)
			);
			const bool Reclaimed = TestTrue(
				TEXT("Captured curved bank reclaims distant partitions within the original total budget"),
				ColdHeavy && PreparedHeavy && ColdHeavy->ReclaimedDistantParents > 0 &&
				ColdHeavy->ReclaimedDistantParents == PreparedHeavy->ReclaimedDistantParents &&
				ColdHeavy->Tree.Leaves().size() <= Heavy.LeafBudget && PreparedHeavy->Tree.Leaves().size() <= Heavy.LeafBudget
			);
			// 遠景返却又は予算の独立検査が不合格の場合の返戻
			if(!Reclaimed) return false;
			const bool RejectedDetail17 = !TestTrue(
				TEXT("Actual expensive region stays identical after advance preparation"),
				Progress.Valid && !Progress.BudgetExhausted && ColdHeavy && PreparedHeavy && ColdHeavy->SurfaceResolutionSatisfied &&
				PreparedHeavy->SurfaceResolutionSatisfied && ColdHeavy->Tree.Leaves() == PreparedHeavy->Tree.Leaves() &&
				ColdHeavy->MaximumSurfaceErrorMeters == PreparedHeavy->MaximumSurfaceErrorMeters &&
				ColdHeavy->MaximumStitchErrorMeters == PreparedHeavy->MaximumStitchErrorMeters &&
				PreparedHeavy->SurfaceHeightEvaluations < ColdHeavy->SurfaceHeightEvaluations
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail17) return false;
			AddInfo(
				FString::Printf(
					TEXT(
						"Heavy river preparation: leaves=%llu visits=%llu preparation_H=%llu foreground_H=%llu cold_H=%llu "
						"preparation_ms=%.3f foreground_ms=%.3f cold_ms=%.3f"
					),
					static_cast<unsigned long long>(PreparedHeavy->Tree.Leaves().size()),
					static_cast<unsigned long long>(Progress.VisitedTiles),
					static_cast<unsigned long long>(Progress.HeightEvaluations),
					static_cast<unsigned long long>(PreparedHeavy->SurfaceHeightEvaluations),
					static_cast<unsigned long long>(ColdHeavy->SurfaceHeightEvaluations),
					PrepareMs,
					WarmMs,
					ColdMs
				)
			);
		}
		double MaxExtraCut = 0, MaxGradientError = 0, MaxCut = 0, MaxOriginalGap = 0, MaxInducedGap = 0, MaxNormalError = 0;
		std::uint64_t CandidateSum = 0, OceanPoints = 0, MaxGapReach = 0;
		double MaxGapFraction = 0;
		for(std::size_t I = 0; I < Network->Reaches().size(); ++I) {
			for(unsigned K = 0; K < 9; ++K) {
				const std::optional<RiverProfileSample> P = Network->SampleReach(I, K / 8.0);
				const UnitDirection D = *UnitDirection::FromVector(P->Direction);
				const TerrainSample H = Field->Evaluate(D), Original = Base->Evaluate(D);
				const RiverSurfaceSample Detail = Field->EvaluateSurface(D);
				const double ChannelCoverage = Detail.ChannelCoverage;
				const bool RejectedDetail18 = !TestTrue(
					TEXT("Material metadata leaves terrain bits unchanged and water attributes finite"),
					std::bit_cast<std::uint64_t>(Detail.Terrain.ElevationMeters) == std::bit_cast<std::uint64_t>(H.ElevationMeters) &&
					Detail.Terrain.TangentialGradientPerRadian.X == H.TangentialGradientPerRadian.X &&
					Detail.Terrain.TangentialGradientPerRadian.Y == H.TangentialGradientPerRadian.Y &&
					Detail.Terrain.TangentialGradientPerRadian.Z == H.TangentialGradientPerRadian.Z && ChannelCoverage >= 0 &&
					ChannelCoverage <= 1 && std::isfinite(Detail.WaterElevationMeters)
				);
				// 不合格の独立検査を保持する返戻
				if(RejectedDetail18) return false;
				const bool RejectedDetail19 =
				!TestTrue(TEXT("Channel centre scalar matches full evaluation"), SameHeight(D, H.ElevationMeters));
				// 不合格の独立検査を保持する返戻
				if(RejectedDetail19) return false;
				const bool RejectedDetail20 = !TestTrue(
					TEXT("Actual channel bed is covered by descending water"),
					std::isfinite(H.ElevationMeters) && H.ElevationMeters <= P->BedMeters + 1E-5 &&
					H.ElevationMeters <= Original.ElevationMeters + 1E-9 && std::isfinite(H.TangentialGradientPerRadian.Length())
				);
				// 不合格の独立検査を保持する返戻
				if(RejectedDetail20) return false;
				if(P->BedMeters - H.ElevationMeters > MaxExtraCut) {
					MaxExtraCut = P->BedMeters - H.ElevationMeters;
					MaxGapReach = I;
					MaxGapFraction = K / 8.0;
				}
				MaxOriginalGap = std::max(MaxOriginalGap, P->BedMeters - Original.ElevationMeters);
				MaxInducedGap = std::max(MaxInducedGap, std::min(P->BedMeters, Original.ElevationMeters) - H.ElevationMeters);
				++Centers;
			}
			if(I % 23) continue;
			const RiverTerrainChannel &C = Field->Channels()[I];
			for(const double T : { -0.05, 0.0, 0.5, 1.0, 1.05 }) for(const double F : { 0.0, 0.35, 0.8, 1.0, 1.05 }) {
				const std::optional<PlanetCore::RiverPlanformSample> Curve =
				C.Planform ? std::optional<PlanetCore::RiverPlanformSample>{ C.Planform->EvaluateAngle(T * C.ArcAngle) } : std::nullopt;
				const UnitDirection Along = Curve ?
				Curve->Direction :
				*UnitDirection::FromVector(C.Start * std::cos(T * C.ArcAngle) + C.Tangent * std::sin(T * C.ArcAngle));
				const UnitDirection Across =
				*UnitDirection::FromVector(Curve ? Along.Vector().Cross(Curve->DerivativePerFraction) : C.Start.Cross(C.Tangent));
				const double A = F * C.OuterHalfWidthMeters / Radius;
				const UnitDirection D = *UnitDirection::FromVector(Along.Vector() * std::cos(A) + Across.Vector() * std::sin(A));
				const TerrainSample H = Field->Evaluate(D);
				const bool Rejected10 = !TestTrue(TEXT("Bank scalar matches full evaluation"), SameHeight(D, H.ElevationMeters));
				// 不合格又は準備失敗を保持する返戻
				if(Rejected10) return false;
				// 合成後の標高を差分評価する，大半径でほぼ等しい標高を引く丸め誤差を避ける
				const double Step = std::max(1E-8, std::min(1E-6, C.OuterHalfWidthMeters / Radius * 1E-3));
				const UnitDirection AcrossDirection =
				*UnitDirection::FromVector(Across.Vector() - D.Vector() * Across.Vector().Dot(D.Vector()));
				for(const UnitDirection Tangent : { AcrossDirection, *UnitDirection::FromVector(D.Vector().Cross(AcrossDirection.Vector())) }) {
					/**
					 * 接線方向の中央差分を求める
					 * @param E 球面角の刻み幅
					 * @return 合成地形の差分微分
					 */
					const auto Difference = [&](const double E) -> double {
						const UnitDirection Plus = *UnitDirection::FromVector(D.Vector() * std::cos(E) + Tangent.Vector() * std::sin(E));
						const UnitDirection Minus = *UnitDirection::FromVector(D.Vector() * std::cos(E) - Tangent.Vector() * std::sin(E));
						// 独立検査に必要な値又は不合格の返戻
						return (Field->Evaluate(Plus).ElevationMeters - Field->Evaluate(Minus).ElevationMeters) / (2 * E);
					};
					// 河道端は C1 で曲率が切替わる為，中央差分の O(h) 項を先に消す
					// 其の後で O(h²) 項を外挿し，滑らかな点と端点の両方で同じ収束条件を使う
					double E = Step, HalfCentral = Difference(E / 2);
					const double Central = Difference(E), Coarse = 2 * Central - Difference(2 * E);
					double Fine = 2 * HalfCentral - Central, FD = (4 * Fine - Coarse) / 3;
					const double SurfaceRadius = Radius + H.ElevationMeters;
					bool bConverged = false;
					for(unsigned Refinement = 0; Refinement < 6; ++Refinement) {
						const double QuarterCentral = Difference(E / 4), Finer = 2 * QuarterCentral - HalfCentral, Next = (4 * Finer - Fine) / 3;
						const double Change = std::abs(std::atan(Next / SurfaceRadius) - std::atan(FD / SurfaceRadius));
						FD = Next;
						// 粗い２組だけの偶然の一致で打切らず，６段階の最後で収束を判定する
						bConverged = Change < 2E-6;
						E *= 0.5;
						HalfCentral = QuarterCentral;
						Fine = Finer;
					}
					const double Exact = H.TangentialGradientPerRadian.Dot(Tangent.Vector());
					const double Relative = std::abs(FD - Exact) / std::max(1.0, std::abs(Exact));
					// 半径に依存する相対微分誤差でなく，実際の法線の傾斜角を比較する
					const double NormalError = std::abs(std::atan(FD / SurfaceRadius) - std::atan(Exact / SurfaceRadius));
					MaxNormalError = std::max(MaxNormalError, NormalError);
					MaxGradientError = std::max(MaxGradientError, Relative);
					++Normals;
					// 不合格又は準備失敗を後続へ持込まない返戻
					const bool RejectedDetail21 = !TestTrue(
						FString::Printf(
							TEXT("Converged valley normal R=%.0f reach=%llu T=%.2g F=%.2g FD=%.9g " "analytic=%.9g angle_error=%.6g rad converged=%d"),
							Radius,
							static_cast<unsigned long long>(I),
							T,
							F,
							FD,
							Exact,
							NormalError,
							bConverged ? 1 : 0
						),
						bConverged && NormalError < 1E-5
					);
					// 不合格の独立検査を保持する返戻
					if(RejectedDetail21) return false;
				}
			}
		}
		for(unsigned I = 0; I < 256; ++I) {
			const double Z = 1 - 2 * (I + 0.5) / 256, P = I * 2.399963229728653, Q = std::sqrt(1 - Z * Z);
			const UnitDirection D = *UnitDirection::FromVector({ Q * std::cos(P), Q * std::sin(P), Z });
			const TerrainSample A = Field->Evaluate(D), B = Field->EvaluateReference(D), Original = Base->Evaluate(D);
			const bool RejectedDetail22 = !TestTrue(
				TEXT("BVH matches independent full-channel evaluation"),
				A.ElevationMeters == B.ElevationMeters && A.TangentialGradientPerRadian.X == B.TangentialGradientPerRadian.X &&
				A.TangentialGradientPerRadian.Y == B.TangentialGradientPerRadian.Y &&
				A.TangentialGradientPerRadian.Z == B.TangentialGradientPerRadian.Z
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail22) return false;
			const double MaximumUplift = Original.ElevationMeters > S.SeaLevelMeters ? 5.00000001 + AdditionalReliefBound : 1E-9;
			const bool RejectedDetail23 = !TestTrue(
				TEXT("Added land relief stays bounded and ocean height is preserved"),
				A.ElevationMeters <= Original.ElevationMeters + MaximumUplift &&
				std::abs(A.ElevationMeters) <= Field->AbsoluteReliefBoundMeters()
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail23) return false;
			MaxCut = std::max(MaxCut, Original.ElevationMeters - A.ElevationMeters);
			CandidateSum += Field->CandidateCount(D);
			OceanPoints += A.ElevationMeters < S.SeaLevelMeters;
			++IndexChecks;
		}
		const std::optional<RiverProfileSample> Center = Network->SampleReach(Network->Reaches().size() / 2, 0.5);
		GroundPatchSettings GroundSettings;
		std::optional<GroundPatch> Patch;
		while(true) {
			Patch = BuildGroundPatch(
				*UnitDirection::FromVector(Center->Direction),
				Radius,
				GroundSettings,
				[&](const UnitDirection D) {
					return Field->Evaluate(D);
				}
			);
			AddInfo(
				FString::Printf(
					TEXT("River ground R=%.0f cells=%u accepted=%d"),
					Radius,
					GroundSettings.CellsPerSide,
					Patch.has_value() ? 1 : 0
				)
			);
			if(Patch) {
				AddInfo(
					FString::Printf(
						TEXT("Adaptive river ground R=%.0f vertices=%llu triangles=%llu evaluations=%llu max_error=%.9g m"),
						Radius,
						static_cast<unsigned long long>(Patch->Positions.size()),
						static_cast<unsigned long long>(Patch->Triangles.size()),
						static_cast<unsigned long long>(Patch->Evaluations),
						Patch->MaximumProbeErrorMeters
					)
				);
			}
			if(Patch || GroundSettings.CellsPerSide > 127) break;
			GroundSettings.CellsPerSide *= 2;
		}
		AddInfo(
			FString::Printf(
				TEXT(
					"River terrain R=%.0f channels=%u max_width=%.6g m max_prepared_cut=%.6g m max_extra_bed_cut=%.6g m "
					"gradient_relative=%.6g mean_candidates=%.6g ocean_256=%.6g sampled_cut=%.6g m original_bed_gap=%.6g m "
					"induced_bed_gap=%.6g m normal_angle_error=%.6g rad max_gap_reach=%llu max_gap_fraction=%.6g"
				),
				Radius,
				Field->Statistics().Channels,
				Field->Statistics().MaximumValleyHalfWidthMeters,
				Field->Statistics().MaximumPreparedCutMeters,
				MaxExtraCut,
				MaxGradientError,
				CandidateSum / 256.0,
				OceanPoints / 256.0,
				MaxCut,
				MaxOriginalGap,
				MaxInducedGap,
				MaxNormalError,
				static_cast<unsigned long long>(MaxGapReach),
				MaxGapFraction
			)
		);
		const bool RejectedDetail24 =
		!TestTrue(TEXT("River terrain supplies local ground within the production refinement limit"), Patch.has_value());
		// 不合格の独立検査を保持する返戻
		if(RejectedDetail24) return false;
		const std::optional<GroundPatch> ScalarPatch = BuildGroundPatch(
			*UnitDirection::FromVector(Center->Direction),
			Radius,
			GroundSettings,
			[&](const UnitDirection D) {
				return Field->Evaluate(D);
			},
			{},
			[&](const UnitDirection D) {
				return Field->EvaluateElevation(D);
			}
		);
		const bool RejectedDetail25 = !TestTrue(
			TEXT("Scalar probes preserve ground topology and error"),
			ScalarPatch && ScalarPatch->Triangles == Patch->Triangles && ScalarPatch->Positions.size() == Patch->Positions.size() &&
			ScalarPatch->MaximumProbeErrorMeters == Patch->MaximumProbeErrorMeters && ScalarPatch->Evaluations == Patch->Evaluations
		);
		// 不合格の独立検査を保持する返戻
		if(RejectedDetail25) return false;
		for(std::size_t I = 0; I < Patch->Positions.size(); ++I) {
			const Vec3 A = Patch->Positions[I], B = ScalarPatch->Positions[I], N = Patch->Normals[I], M = ScalarPatch->Normals[I];
			const auto Bits = [](const double X, const double Y) -> bool {
				// 独立検査に必要な値又は不合格の返戻
				return std::bit_cast<std::uint64_t>(X) == std::bit_cast<std::uint64_t>(Y);
			};
			const bool RejectedDetail26 = !TestTrue(
				TEXT("Scalar probes preserve ground positions and normals bitwise"),
				Bits(A.X, B.X) && Bits(A.Y, B.Y) && Bits(A.Z, B.Z) && Bits(N.X, M.X) && Bits(N.Y, M.Y) && Bits(N.Z, M.Z)
			);
			// 不合格の独立検査を保持する返戻
			if(RejectedDetail26) return false;
		}
	}
	TestTrue(TEXT("An inland relief sample was checked"), ReliefChecks > 0);
	AddInfo(
		FString::Printf(
			TEXT("River terrain contracts: centers=%llu gradients=%llu index=%llu seconds=%.3f"),
			static_cast<unsigned long long>(Centers),
			static_cast<unsigned long long>(Normals),
			static_cast<unsigned long long>(IndexChecks),
			FPlatformTime::Seconds() - Begin
		)
	);
	AddInfo(
		FString::Printf(TEXT("Scalar height/water/support bit comparisons: %llu"), static_cast<unsigned long long>(ScalarChecks))
	);
	// 全拘束の検査完遂の返戻
	return true;
}

#endif
