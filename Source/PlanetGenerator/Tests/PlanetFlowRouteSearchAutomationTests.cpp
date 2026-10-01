#include "Core/PlanetFlowRoute.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 索引を使わず全区間と制約付き端点から最小距離を選ぶ
	 * @param Route 検査する不変な経路
	 * @param Query 照会方向
	 * @param Radius 惑星半径
	 * @param Width 検索の物理幅
	 * @param Endpoints 端点も比較する場合に真
	 * @return 元と同じ同距離規則で選んだ候補
	 */
	std::optional<PlanetCore::FlowRouteProjection> ExhaustiveFootpoint(
		const PlanetCore::TerrainFlowRoute &Route,
		const PlanetCore::UnitDirection Query,
		const double Radius,
		const double Width,
		const bool Endpoints
	) {
		const double Chord = 2 * std::sin(Width / (2 * Radius)), Limit = Chord * Chord;
		std::optional<PlanetCore::FlowRouteProjection> Best;
		// 制約付き経路端の比較
		if(Endpoints) for(unsigned Side = 0; Side < 2; ++Side) {
			const double Distance = Side ? Route.GetLengthMeters() : 0.0;
			const PlanetCore::TerrainFlowCurveSample Point = *Route.Sample(Distance);
			const PlanetCore::Vec3 Delta = Query.Vector() - Point.Direction.Vector();
			const double Squared = Delta.Dot(Delta), Along = Delta.Dot(Point.DirectionDerivativePerMeter);
			if(Side ? Along < 0 : Along > 0) continue;
			if(Squared > Limit || Best && !(Squared < Best->Footpoint.SquaredChordDistance)) continue;
			const PlanetCore::Vec3 Gradient = (Delta - Query.Vector() * Delta.Dot(Query.Vector())) * 2;
			const PlanetCore::TerrainFlowFootpoint Foot { Point, Side ? 1.0 : 0.0, Squared, Gradient, {} };
			Best = PlanetCore::FlowRouteProjection{ Foot, Distance, Side ? static_cast<std::uint32_t>(Route.GetSegments().size() - 1) : 0 };
		}
		// 保守境界を使わない全区間の独立比較
		double Start = 0;
		std::uint32_t Index = 0;
		for(const PlanetCore::TerrainFlowSegment &Segment : Route.GetSegments()) {
			const std::optional<PlanetCore::TerrainFlowFootpoint> Foot = Segment.FindInteriorFootpoint(Query);
			if(
				Foot && !(Foot->SquaredChordDistance > Limit) && (
					!Best || Foot->SquaredChordDistance < Best->Footpoint.SquaredChordDistance ||
					Foot->SquaredChordDistance == Best->Footpoint.SquaredChordDistance && Index <= Best->SegmentIndex
				)
			) Best = PlanetCore::FlowRouteProjection{ *Foot, Start + Foot->Fraction * Segment.GetParameterLengthMeters(), Index };
			Start += Segment.GetParameterLengthMeters();
			++Index;
		}
		// 全区間の最小候補の返戻
		return Best;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowRouteSearchContracts,
	"PlanetGenerator.FlowRoute.Search",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 幅広い照会の探索費用と全区間比較との一致を物理尺度毎に測る
 * @param Parameters 未使用の実行引数
 * @return 端点・内部・範囲外で索引が同じ候補を返す場合に真
 */
bool FPlanetFlowRouteSearchContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		// 滑らかな曲線を持つ多区間の基準経路
		const double Length = std::min(4000.0, Radius * 0.4), Amplitude = Length * 0.01;
		std::vector<PlanetCore::FlowRouteKnot> Knots;
		for(unsigned i = 0; i < 513; ++i) {
			const double S = Length * i / 512, X = S / Radius, Y = Amplitude * std::sin(S * 12 / Length) / Radius;
			const double DY = Amplitude * 12 / Length * std::cos(S * 12 / Length);
			const PlanetCore::Vec3 Position { std::cos(X) * std::cos(Y), std::sin(X) * std::cos(Y), std::sin(Y) };
			const PlanetCore::Vec3 Tangent {
				-std::sin(X) * std::cos(Y) - std::cos(X) * std::sin(Y) * DY,
				std::cos(X) * std::cos(Y) - std::sin(X) * std::sin(Y) * DY,
				std::cos(Y) * DY
			};
			Knots.push_back({ *PlanetCore::UnitDirection::FromVector(Position), Tangent, S });
		}
		const std::optional<PlanetCore::TerrainFlowRoute> Route = PlanetCore::TerrainFlowRoute::Create(Knots, Radius);
		// 準備出来ない経路を計測対象から除外しない返戻
		if(!TestTrue(TEXT("検索比較用の曲線を準備"), Route.has_value())) return false;
		const double Width = std::min(500.0, Radius * 0.02);
		std::vector<PlanetCore::UnitDirection> Queries;
		for(unsigned i = 0; i < 1024; ++i) {
			const double Along = Route->GetLengthMeters() * i / 1023;
			const PlanetCore::TerrainFlowCurveSample Point = *Route->Sample(Along);
			const PlanetCore::Vec3 Tangent = PlanetCore::UnitDirection::FromVector(Point.DirectionDerivativePerMeter)->Vector();
			const PlanetCore::Vec3 Across = Point.Direction.Vector().Cross(Tangent);
			const double Offset = Width * (static_cast<int>(i % 13) - 6) / 8;
			Queries.push_back(
				*PlanetCore::UnitDirection::FromVector(
					Point.Direction.Vector() * std::cos(Offset / Radius) + Across * std::sin(Offset / Radius)
				)
			);
		}
		// 端の外と対蹠点の照会
		for(unsigned Side = 0; Side < 2; ++Side) {
			const PlanetCore::TerrainFlowCurveSample Point = *Route->Sample(Side ? Route->GetLengthMeters() : 0.0);
			const PlanetCore::Vec3 Tangent = PlanetCore::UnitDirection::FromVector(Point.DirectionDerivativePerMeter)->Vector();
			const double Offset = (Side ? 1.0 : -1.0) * Width * 0.5;
			Queries.push_back(
				*PlanetCore::UnitDirection::FromVector(
					Point.Direction.Vector() * std::cos(Offset / Radius) + Tangent * std::sin(Offset / Radius)
				)
			);
		}
		Queries.push_back(*PlanetCore::UnitDirection::FromVector(Knots[0].Direction.Vector() * -1));
		std::uint64_t Candidates = 0, ResultHash = UINT64_C(0XCBF29CE484222325);
		const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
		// 広い候補範囲での実際の直交点解法の呼出数
		for(const PlanetCore::UnitDirection Query : Queries) {
			std::uint32_t Count = 0;
			const std::optional<PlanetCore::FlowRouteProjection> Found = Route->FindFootpoint(Query, Width, &Count, true);
			Candidates += Count;
			if(Found) {
				ResultHash = (ResultHash ^ Found->SegmentIndex) * UINT64_C(0X100000001B3);
				ResultHash = (ResultHash ^ std::bit_cast<std::uint64_t>(Found->Footpoint.Fraction)) * UINT64_C(0X100000001B3);
				ResultHash = (ResultHash ^ std::bit_cast<std::uint64_t>(Found->Footpoint.SquaredChordDistance)) * UINT64_C(0X100000001B3);
			}
		}
		const double Milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count();
		std::uint32_t Comparisons = 0;
		// 計測と分けた全区間比較，端点を含む場合と含まない場合
		for(std::size_t i = 0; i < Queries.size(); ++i) for(const bool Endpoints : { false, true }) {
			const std::optional<PlanetCore::FlowRouteProjection> Expected =
			ExhaustiveFootpoint(*Route, Queries[i], Radius, Width, Endpoints);
			const std::optional<PlanetCore::FlowRouteProjection> Found = Route->FindFootpoint(Queries[i], Width, nullptr, Endpoints);
			TestEqual(TEXT("全区間比較と候補の有無が一致"), Found.has_value(), Expected.has_value());
			if(Found && Expected) {
				TestEqual(TEXT("全区間比較と選択区間が一致"), Found->SegmentIndex, Expected->SegmentIndex);
				TestEqual(TEXT("全区間比較と曲線の座標が一致"), Found->Footpoint.Fraction, Expected->Footpoint.Fraction);
				TestEqual(TEXT("全区間比較と距離二乗が一致"), Found->Footpoint.SquaredChordDistance, Expected->Footpoint.SquaredChordDistance);
			}
			++Comparisons;
		}
		AddInfo(
			FString::Printf(
				TEXT("Route search R=%.0f queries=%llu candidates=%llu comparisons=%u hash=%llu ms=%.9g"),
				Radius,
				static_cast<unsigned long long>(Queries.size()),
				static_cast<unsigned long long>(Candidates),
				Comparisons,
				static_cast<unsigned long long>(ResultHash),
				Milliseconds
			)
		);
	}
	// 独立な比較結果の返戻
	return !HasAnyErrors();
}

#endif
