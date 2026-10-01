#include "Core/PlanetErosionPreparation.h"
#include "Core/PlanetRiverSurface.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetRiverWaterContracts, "PlanetGenerator.RiverWater.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 描画へ渡す地点水位を縦断拘束と照合し，合流に依る逆勾配と河口の不一致を検出する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，水位の不一致は UE に記録
 */
bool FPlanetRiverWaterContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::array<std::pair<std::uint64_t, double>, 4> Planets { { { 42, 1000 }, { 42, 1E6 }, { 42, 1E9 }, { 7, 6371000 } } };
	for(const std::pair<std::uint64_t, double> Definition : Planets) {
		const double Radius = Definition.second;
		PlanetCore::GeologySettings Settings;
		Settings.Seed = Definition.first;
		Settings.RadiusMeters = Radius;
		std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
		// 基盤を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("水面検査の地質を準備"), Geology.has_value())) return false;
		std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
		// 海陸を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("水面検査の海陸を準備"), PreparedOcean.has_value())) return false;
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
		std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
		std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
		// 排水を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("水面検査の排水を準備"), PreparedDrainage.has_value())) return false;
		const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
		std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
		const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
		// 本番と同じ地形を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("水面検査に本番の侵食地形を使用"), Field.has_value())) return false;
		double MaximumProfileError = 0, MaximumRise = 0, MaximumMouthError = 0, MaximumExposedBed = 0;
		double MaximumWaterAboveOriginalTerrain = 0, MaximumMinimumGradeAboveTerrain = 0, MaximumAvoidableOverflow = 0;
		double MaximumAvoidableBankOverflow = 0, MaximumBankGradeExcess = 0, MaximumInteriorSeaDepth = 0;
		std::uint32_t OceanSamples = 0;
		std::size_t WorstReach = 0;
		std::uint32_t WorstStep = 0;
		std::uint64_t Samples = 0;
		bool bHasFiniteSamples = true;
		for(std::size_t i = 0; i < Field->Drainage().Reaches().size(); ++i) {
			double PreviousWater = 0;
			const double BankAngle = Field->Channels()[i].InnerHalfWidthMeters / Radius, BankCosine = std::cos(BankAngle);
			const double BankSine = std::sin(BankAngle);
			for(std::uint32_t j = 0; j < 258; ++j) {
				const std::optional<PlanetCore::RiverProfileSample> Profile = Field->Drainage().SampleReach(i, j / 257.0);
				// 定義済みの河道を評価出来なかった場合の返戻
				if(!Profile) return false;
				const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Profile->Direction);
				const PlanetCore::Vec3 Across =
				PlanetCore::UnitDirection::FromVector(Profile->Direction.Cross(Profile->DownstreamTangent))->Vector();
				const PlanetCore::RiverSurfaceSample Surface = Field->EvaluateSurface(Direction);
				const double OriginalHeight = Field->GetErosionPreparation()->GetAeolianField()->EvaluateElevation(Direction);
				MaximumWaterAboveOriginalTerrain = std::max(MaximumWaterAboveOriginalTerrain, Profile->WaterMeters - OriginalHeight);
				// 端点より手前で大洋を横断する河道を見逃さない
				if(j < 257) {
					MaximumInteriorSeaDepth = std::max(MaximumInteriorSeaDepth, Field->Drainage().Settings().SeaLevelMeters - OriginalHeight);
				}
				const PlanetCore::RiverLongitudinalProfile &Curve = Field->Drainage().Profiles()[i];
				MaximumMinimumGradeAboveTerrain =
				std::max(MaximumMinimumGradeAboveTerrain, std::lerp(Curve.GradeStart, Curve.GradeEnd, j / 257.0) - OriginalHeight);
				MaximumAvoidableOverflow = std::max(
					MaximumAvoidableOverflow,
					Profile->WaterMeters - std::max(OriginalHeight, std::lerp(Curve.GradeStart, Curve.GradeEnd, j / 257.0))
				);
				for(const double Sign : { -1.0, 1.0 }) {
					const PlanetCore::UnitDirection Bank =
					*PlanetCore::UnitDirection::FromVector(Direction.Vector() * BankCosine + Across * (Sign * BankSine));
					const double BankHeight = Field->GetErosionPreparation()->GetAeolianField()->EvaluateElevation(Bank);
					const double Grade = std::lerp(Curve.GradeStart, Curve.GradeEnd, j / 257.0);
					MaximumAvoidableBankOverflow = std::max(MaximumAvoidableBankOverflow, Profile->WaterMeters - std::max(BankHeight, Grade));
					MaximumBankGradeExcess = std::max(MaximumBankGradeExcess, Grade - BankHeight);
				}
				const double WaterSupport = Surface.WaterSupport;
				bHasFiniteSamples &=
				std::isfinite(Surface.WaterElevationMeters) && Surface.ChannelCoverage > 0 && WaterSupport >= Surface.ChannelCoverage &&
				WaterSupport <= 1;
				MaximumProfileError = std::max(MaximumProfileError, std::abs(Surface.WaterElevationMeters - Profile->WaterMeters));
				MaximumExposedBed = std::max(MaximumExposedBed, Surface.Terrain.ElevationMeters - Surface.WaterElevationMeters);
				if(j && Surface.WaterElevationMeters - PreviousWater > MaximumRise) {
					MaximumRise = Surface.WaterElevationMeters - PreviousWater;
					WorstReach = i;
					WorstStep = j;
				}
				PreviousWater = Surface.WaterElevationMeters;
				++Samples;
			}
		}
		// 河道から離れた海面も同じ一定水位で定義され，負の川底を海と混同しない事を確認する
		for(std::uint32_t i = 0; i < 1024; ++i) {
			const double Z = 1 - 2 * (i + 0.5) / 1024, Angle = i * 2.399963229728653, EquatorRadius = std::sqrt(1 - Z * Z);
			const PlanetCore::UnitDirection Direction =
			*PlanetCore::UnitDirection::FromVector({ EquatorRadius * std::cos(Angle), EquatorRadius * std::sin(Angle), Z });
			if(Ocean->Evaluate(Direction).ElevationMeters > Ocean->Base().Settings().SeaLevelMeters) continue;
			const PlanetCore::RiverSurfaceSample Sample = Field->EvaluateSurface(Direction);
			TestEqual(TEXT("元の大洋の水位が一定海面に厳密に一致"), Sample.WaterElevationMeters, Field->Drainage().Settings().SeaLevelMeters);
			TestEqual(TEXT("元の大洋には常に水位の支持が有る"), Sample.WaterSupport, 1.0);
			++OceanSamples;
		}
		TestTrue(TEXT("河川の支持域外を含む大洋の地点を検査"), OceanSamples > 100);
		for(const PlanetCore::RiverMouth &Mouth : Field->Drainage().Mouths()) {
			const PlanetCore::Vec3 End = Field->Drainage().SampleReach(Mouth.Reach, 1)->Direction;
			const PlanetCore::RiverSurfaceSample Surface = Field->EvaluateSurface(*PlanetCore::UnitDirection::FromVector(End));
			MaximumMouthError =
			std::max(MaximumMouthError, std::abs(Surface.WaterElevationMeters - Field->Drainage().Settings().SeaLevelMeters));
		}
		AddInfo(
			FString::Printf(
				TEXT("River water R=%.0f samples=%llu profileError=%.9g rise=%.9g " "reach=%llu step=%u mouthError=%.9g exposedBed=%.9g"),
				Radius,
				static_cast<unsigned long long>(Samples),
				MaximumProfileError,
				MaximumRise,
				static_cast<unsigned long long>(WorstReach),
				WorstStep,
				MaximumMouthError,
				MaximumExposedBed
			)
		);
		if(MaximumRise > 1E-8) {
			const PlanetCore::RiverReach &Reach = Field->Drainage().Reaches()[WorstReach];
			for(std::uint32_t Step = WorstStep > 1 ? WorstStep - 2 : 0; Step <= std::min(257U, WorstStep + 2); ++Step) {
				const double Fraction = Step / 257.0;
				const PlanetCore::RiverProfileSample Profile = *Field->Drainage().SampleReach(WorstReach, Fraction);
				const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Profile.Direction);
				const PlanetCore::RiverSurfaceSample Sample = Field->EvaluateSurface(Direction);
				const double Original = Field->GetErosionPreparation()->GetAeolianField()->EvaluateElevation(Direction);
				const PlanetCore::RiverLongitudinalProfile &Curve = Field->Drainage().Profiles()[WorstReach];
				AddInfo(
					FString::Printf(
						TEXT(
							"Water ceiling regression seed=%llu R=%.0f reach=%llu step=%u "
							"W=%.17g planned=%.17g original=%.17g grade=%.17g bed=%.17g downstreamOcean=%d"
						),
						static_cast<unsigned long long>(Definition.first),
						Radius,
						static_cast<unsigned long long>(WorstReach),
						Step,
						Sample.WaterElevationMeters,
						Profile.WaterMeters,
						Original,
						std::lerp(Curve.GradeStart, Curve.GradeEnd, Fraction),
						Sample.Terrain.ElevationMeters,
						Field->Drainage().Nodes()[Reach.Downstream].Ocean ? 1 : 0
					)
				);
			}
		}
		std::uint32_t Sections = 0, Banks = 0, Connections = 0, OceanEdges = 0;
		double MaximumBankBracket = 0, MaximumArcError = 0, MaximumBankDepth = 0;
		TestFalse(TEXT("範囲外の河道断面を拒否"), PlanetCore::SampleRiverWaterSection(*Field, Field->Channels().size(), 0.5).has_value());
		TestFalse(TEXT("範囲外の縦断位置を拒否"), PlanetCore::SampleRiverWaterSection(*Field, 0, -0.1).has_value());
		std::stop_source Stop;
		Stop.request_stop();
		TestFalse(TEXT("中断した断面を公開しない"), PlanetCore::SampleRiverWaterSection(*Field, 0, 0.5, Stop.get_token()).has_value());
		for(std::size_t i = 0; i < Field->Channels().size(); i += 17) for(const double Fraction : { 0.371, 1.0 }) {
			const std::optional<PlanetCore::RiverWaterSection> Section = PlanetCore::SampleRiverWaterSection(*Field, i, Fraction);
			// 有効な河川断面を構築出来なかった場合の返戻
			if(!TestTrue(TEXT("最終河床に対する河川断面を準備"), Section.has_value())) return false;
			const PlanetCore::Vec3 Across =
			PlanetCore::UnitDirection::FromVector(Section->Center.Direction.Cross(Section->DownstreamTangent))->Vector();
			++Sections;
			for(const PlanetCore::RiverWaterEdge &Edge : { Section->Left, Section->Right }) {
				TestTrue(TEXT("断面の水際が中心から正の幅を持つ"), Edge.DistanceMeters > 0);
				TestTrue(TEXT("水際を地面の下に置かない"), Edge.WaterMeters >= Edge.GroundMeters);
				const double Arc =
				Radius * std::atan2(Section->Center.Direction.Cross(Edge.Direction).Length(), Section->Center.Direction.Dot(Edge.Direction));
				MaximumArcError = std::max(MaximumArcError, std::abs(Arc - Edge.DistanceMeters));
				if(Edge.Boundary != PlanetCore::RiverWaterBoundary::DryBank) {
					if(Edge.Boundary == PlanetCore::RiverWaterBoundary::Ocean) {
						++OceanEdges;
						TestEqual(TEXT("大洋への接続水面は一定海面に一致"), Edge.WaterMeters, Field->Drainage().Settings().SeaLevelMeters);
						TestTrue(TEXT("大洋への接続地点は海面下に有る"), Edge.GroundMeters < Field->Drainage().Settings().SeaLevelMeters);
					}
					if(Edge.Boundary == PlanetCore::RiverWaterBoundary::OpenWater) {
						const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Edge.Direction);
						const PlanetCore::RiverSurfaceSample Surface = Field->EvaluateSurface(Direction);
						const double SeaGround = Ocean->Evaluate(Direction).ElevationMeters;
						AddInfo(
							FString::Printf(
								TEXT(
									"Open river edge R=%.0f reach=%llu fraction=%.9g side=%s "
									"distance=%.9g outer=%.9g W=%.17g H=%.9g originalH=%.9g support=%.9g extension=%d"
								),
								Radius,
								static_cast<unsigned long long>(i),
								Fraction,
								(Edge.Direction - Section->Center.Direction).Dot(Across) > 0 ? TEXT("left") : TEXT("right"),
								Edge.DistanceMeters,
								Field->Channels()[i].OuterHalfWidthMeters,
								Edge.WaterMeters,
								Edge.GroundMeters,
								SeaGround,
								Surface.WaterSupport,
								Edge.bUsesProfileExtension ? 1 : 0
							)
						);
					}
					++Connections;
					continue;
				}
				++Banks;
				MaximumBankDepth = std::max(MaximumBankDepth, Edge.WaterMeters - Edge.GroundMeters);
				MaximumBankBracket = std::max(MaximumBankBracket, Edge.BracketWidthMeters);
				const double Sign = (Edge.Direction - Section->Center.Direction).Dot(Across) > 0 ? 1 : -1;
				const double Angle = Sign * (Edge.DistanceMeters + Edge.BracketWidthMeters) / Radius;
				const PlanetCore::UnitDirection Outside =
				*PlanetCore::UnitDirection::FromVector(Section->Center.Direction * std::cos(Angle) + Across * std::sin(Angle));
				const PlanetCore::RiverSurfaceSample Surface = Field->EvaluateSurface(Outside);
				const double ProfileWater = Field->Drainage().SampleReach(i, Fraction)->WaterMeters;
				const double Water =
				std::max(Field->Drainage().Settings().SeaLevelMeters, Surface.WaterSupport > 0 ? Surface.WaterElevationMeters : ProfileWater);
				if(Surface.Terrain.ElevationMeters < Water - 1E-8) {
					const PlanetCore::RiverProfileSample Original = *Field->Drainage().SampleReach(i, Fraction);
					const PlanetCore::Vec3 OriginalAcross =
					PlanetCore::UnitDirection::FromVector(Original.Direction.Cross(Original.DownstreamTangent))->Vector();
					const PlanetCore::UnitDirection OriginalOutside =
					*PlanetCore::UnitDirection::FromVector(Original.Direction * std::cos(Angle) + OriginalAcross * std::sin(Angle));
					const PlanetCore::RiverSurfaceSample OriginalSurface = Field->EvaluateSurface(OriginalOutside);
					const double OriginalWater = std::max(
						Field->Drainage().Settings().SeaLevelMeters,
						OriginalSurface.WaterSupport > 0 ? OriginalSurface.WaterElevationMeters : Original.WaterMeters
					);
					AddInfo(
						FString::Printf(
							TEXT(
								"Bank reconstruction R=%.0f reach=%llu fraction=%.9g sign=%.0f "
								"distance=%.17g bracket=%.17g reconstructedGap=%.17g originalGap=%.17g "
								"directionDeltaMeters=%.17g centerDeltaMeters=%.17g"
							),
							Radius,
							static_cast<unsigned long long>(i),
							Fraction,
							Sign,
							Edge.DistanceMeters,
							Edge.BracketWidthMeters,
							Surface.Terrain.ElevationMeters - Water,
							OriginalSurface.Terrain.ElevationMeters - OriginalWater,
							(OriginalOutside.Vector() - Outside.Vector()).Length() * Radius,
							(Original.Direction - Section->Center.Direction).Length() * Radius
						)
					);
				}
				TestTrue(TEXT("岸の挟込みの外側が独立評価でも陸になる"), Surface.Terrain.ElevationMeters >= Water - 1E-8);
			}
			TestTrue(TEXT("左岸の向きが下流と惑星外向きから決まる"), (Section->Left.Direction - Section->Center.Direction).Dot(Across) > 0);
			TestTrue(TEXT("右岸の向きが下流と惑星外向きから決まる"), (Section->Right.Direction - Section->Center.Direction).Dot(Across) < 0);
		}
		if(Radius == 1E9) {
			const std::optional<PlanetCore::RiverWaterSection> Regression = PlanetCore::SampleRiverWaterSection(*Field, 289, 0.371);
			// 再現地点の断面が欠落した場合の返戻
			if(!TestTrue(TEXT("被覆終端の回帰断面を準備"), Regression.has_value())) return false;
			// 第５版では水深 5.18 m の開いた端だった地点を，新しい縦断でも実際の水面接触で判定する
			TestTrue(
				TEXT("河道の被覆終端を水没した陸岸に化けさせない"),
				Regression->Left.Boundary != PlanetCore::RiverWaterBoundary::DryBank ||
				Regression->Left.WaterMeters - Regression->Left.GroundMeters <= 0.02
			);
			TestTrue(TEXT("回帰地点で水際の水位が地面を下回らない"), Regression->Left.WaterMeters >= Regression->Left.GroundMeters);
			AddInfo(
				FString::Printf(
					TEXT("Captured river edge reach=289 distance=%.9g depth=%.9g extension=%d"),
					Regression->Left.DistanceMeters,
					Regression->Left.WaterMeters - Regression->Left.GroundMeters,
					Regression->Left.bUsesProfileExtension ? 1 : 0
				)
			);
		}
		TestTrue(TEXT("陸岸は実際の水面と２ cm 以内で接触する"), MaximumBankDepth <= 0.02);
		TestTrue(TEXT("独立な縦断試料で最低勾配に強制されない溢水が２ cm 以下"), MaximumAvoidableOverflow <= 0.02);
		TestTrue(TEXT("左右の予定岸で最低勾配に強制されない溢水が２ cm 以下"), MaximumAvoidableBankOverflow <= 0.02);
		AddInfo(
			FString::Printf(
				TEXT("Transverse ceiling R=%.0f avoidableOverflow=%.9g minimumGradeExcess=%.9g"),
				Radius,
				MaximumAvoidableBankOverflow,
				MaximumBankGradeExcess
			)
		);
		AddInfo(
			FString::Printf(
				TEXT(
					"River bank ceiling R=%.0f knotWaterAboveTerrain=%.9g "
					"independentWaterAboveTerrain=%.9g bankContactDepth=%.9g "
					"minimumGradeAboveTerrain=%.9g avoidableOverflow=%.9g evaluations=%llu"
				),
				Radius,
				Field->Drainage().Statistics().MaximumSampledWaterAboveTerrainMeters,
				MaximumWaterAboveOriginalTerrain,
				MaximumBankDepth,
				MaximumMinimumGradeAboveTerrain,
				MaximumAvoidableOverflow,
				static_cast<unsigned long long>(Field->Drainage().Statistics().ProfileTerrainEvaluations)
			)
		);
		TestTrue(TEXT("岸の挟込み幅が 0.1 mm 以下"), MaximumBankBracket <= 1E-4);
		TestTrue(TEXT("断面の距離が球面の実弧長と一致"), MaximumArcError < 1E-6);
		AddInfo(
			FString::Printf(
				TEXT("River sections R=%.0f sections=%u banks=%u connections=%u bracket=%.9g arcError=%.9g oceanEdges=%u openEdges=%u"),
				Radius,
				Sections,
				Banks,
				Connections,
				MaximumBankBracket,
				MaximumArcError,
				OceanEdges,
				Connections - OceanEdges
			)
		);
		TestTrue(TEXT("独立な河道試料で河口より手前の大洋横断が無い"), MaximumInteriorSeaDepth <= 1E-8);
		AddInfo(
			FString::Printf(
				TEXT("Coastal water R=%.0f extraOutlets=%u interiorSeaDepth=%.9g oceanSamples=%u"),
				Radius,
				Field->Drainage().Statistics().CoastalOutletNodes,
				MaximumInteriorSeaDepth,
				OceanSamples
			)
		);
		TestTrue(TEXT("河道上の地点水位が有限で被覆を持つ"), bHasFiniteSamples);
		TestTrue(TEXT("地点水位が縦断拘束に２ cm 以内で一致"), MaximumProfileError <= 0.02);
		TestTrue(TEXT("地点水位に下流への上り返しが無い"), MaximumRise <= 1E-8);
		TestTrue(TEXT("地点水位の河口が海面に２ cm 以内で一致"), MaximumMouthError <= 0.02);
		TestTrue(TEXT("河道中心の地面が水面より上に出ない"), MaximumExposedBed <= 1E-8);
	}
	// 個別の拘束違反は検査器へ記録した上で，検査完遂の返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetRiverEndpointNormals, "PlanetGenerator.RiverTerrain.EndpointNormals",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 曲率が切替わる河道端で，両側差分の収束を刻み幅毎に調べる
 * @param Parameters UE 自動検査の引数
 * @return 有効な地形を準備し，両接線の収束を検査出来た場合に真
 */
bool FPlanetRiverEndpointNormals::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Settings;
	Settings.RadiusMeters = 1000;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
	// 地質を準備出来なかった場合の返戻
	if(!Geology) return false;
	std::optional<PlanetCore::OceanConnectivityField> Ocean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 海陸を準備出来なかった場合の返戻
	if(!Ocean) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Base =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*Ocean));
	std::optional<PlanetCore::DrainageNetwork> Drainage = PlanetCore::DrainageNetwork::Create(*Base);
	// 排水を準備出来なかった場合の返戻
	if(!Drainage) return false;
	const std::optional<PlanetCore::RiverTerrainField> Field =
	PlanetCore::RiverTerrainField::Create(Base, std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*Drainage)));
	// 本番の合成場を準備出来なかった場合の返戻
	if(!Field) return false;
	// 再現対象の河道が無い場合の返戻
	if(!TestTrue(TEXT("再現対象の河道が存在"), Field->Channels().size() > 1656)) return false;
	const PlanetCore::RiverTerrainChannel &Channel = Field->Channels()[1656];
	const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Channel.Start);
	const PlanetCore::Vec3 Across = PlanetCore::UnitDirection::FromVector(Channel.Start.Cross(Channel.Tangent))->Vector();
	const PlanetCore::TerrainSample Height = Field->Evaluate(Direction);
	const double Radius = Settings.RadiusMeters + Height.ElevationMeters;
	unsigned Axis = 0;
	for(
		const PlanetCore::Vec3 Tangent :
		{ Across, PlanetCore::UnitDirection::FromVector(Direction.Vector().Cross(Across))->Vector() }
	) {
		const double Exact = Height.TangentialGradientPerRadian.Dot(Tangent);
		double Previous = 0, PreviousFirstOrder = 0, LastError = 0, LastChange = 0;
		for(unsigned Step = 0; Step < 14; ++Step) {
			const double E =
			std::ldexp(std::clamp(Channel.OuterHalfWidthMeters / Settings.RadiusMeters * 1E-3, 1E-8, 1E-6), -static_cast<int>(Step));
			const PlanetCore::UnitDirection Plus =
			*PlanetCore::UnitDirection::FromVector(Direction.Vector() * std::cos(E) + Tangent * std::sin(E));
			const PlanetCore::UnitDirection Minus =
			*PlanetCore::UnitDirection::FromVector(Direction.Vector() * std::cos(E) - Tangent * std::sin(E));
			const double Central = (Field->EvaluateElevation(Plus) - Field->EvaluateElevation(Minus)) / (2 * E);
			const double FirstOrder = 2 * Central - Previous, SecondOrder = (4 * FirstOrder - PreviousFirstOrder) / 3;
			LastError = std::abs(std::atan(SecondOrder / Radius) - std::atan(Exact / Radius));
			LastChange = std::abs(std::atan(FirstOrder / Radius) - std::atan(PreviousFirstOrder / Radius));
			AddInfo(
				FString::Printf(
					TEXT("Endpoint axis=%u step=%u h=%.9g central=%.9g first=%.9g second=%.9g exact=%.9g error=%.9g change=%.9g"),
					Axis,
					Step,
					E,
					Central,
					FirstOrder,
					SecondOrder,
					Exact,
					LastError,
					LastChange
				)
			);
			Previous = Central;
			PreviousFirstOrder = FirstOrder;
		}
		TestTrue(TEXT("河道端の片側曲率を含む差分が収束"), LastChange < 2E-6);
		TestTrue(TEXT("河道端の独立差分が解析法線と一致"), LastError < 1E-5);
		++Axis;
	}
	// 記録した収束列と検査結果の返戻
	return true;
}

#endif
