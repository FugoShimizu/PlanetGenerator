#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillString.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillStringContracts,
	"PlanetGenerator.SpillString.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 谷から外れた初期経路を球面上で緩和し，解析的な峠と固定端点を確認する
 * @param Parameters 未使用の実行引数
 * @return 元場の実鞍点・３半径・有限予算・中断を確認した場合に真
 */
bool FPlanetSpillStringContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 P = D.Vector();
			const double Horizontal = P.X * P.X + P.Y * P.Y, X = Radius * std::atan2(P.Y, P.X), Y = Radius * std::asin(P.Z);
			const double Q = X * X / 400 - 1, DX = Q * X / 100, DY = Y / 200;
			const PlanetCore::Vec3 G {
				-Radius * DX * P.Y / Horizontal,
				Radius * DX * P.X / Horizontal,
				Radius * DY / std::sqrt(1 - P.Z * P.Z)
			};
			// 不変な二重窪地の元標高と解析接線微分の返戻
			return { Q * Q + Y * Y / 400, G - P * G.Dot(P) };
		};
		std::vector<PlanetCore::UnitDirection> Initial;
		for(unsigned I = 0; I < 33; ++I) {
			const double Fraction = I / 32.0, X = -20 + 40 * Fraction, Y = !I || I == 32 ? 0 : 10 * std::sin(std::acos(-1.0) * Fraction);
			const double Latitude = Y / Radius, Longitude = X / Radius;
			Initial.push_back(
				*PlanetCore::UnitDirection::FromVector(
					{ std::cos(Latitude) * std::cos(Longitude), std::cos(Latitude) * std::sin(Longitude), std::sin(Latitude) }
				)
			);
		}
		PlanetCore::SpillStringSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SpacingMeters = 1;
		Settings.MaximumMoveMeters = 0.5;
		Settings.MaximumIterations = 256;
		const std::optional<PlanetCore::SpillString> Relaxed = PlanetCore::RelaxSpillString(Initial, Source, Settings);
		// 完成していない反復を有効な候補へ読み替えない返戻
		if(!TestTrue(TEXT("球面上の候補経路を準備"), Relaxed.has_value())) return false;
		for(const std::size_t Index : { std::size_t{ 0 }, Relaxed->Path.size() - 1 }) {
			const PlanetCore::Vec3 A = Relaxed->Path[Index].Vector(), B = Index ? Initial.back().Vector() : Initial.front().Vector();
			TestTrue(TEXT("端点の３成分を厳密に維持"), A.X == B.X && A.Y == B.Y && A.Z == B.Z);
		}
		double MaximumY = 0, MaximumElevation = -1;
		for(const PlanetCore::UnitDirection D : Relaxed->Path) {
			MaximumY = std::max(MaximumY, std::abs(Radius * std::asin(D.Vector().Z)));
			MaximumElevation = std::max(MaximumElevation, Source(D).ElevationMeters);
			TestTrue(TEXT("候補も単位球面に存在"), std::abs(D.Vector().Length() - 1) < 1E-14);
		}
		TestTrue(TEXT("解析的な谷へ経路が接近"), MaximumY < 0.05);
		TestTrue(TEXT("反復変位も設定した収束閾値以下"), Relaxed->MaximumMotionMeters <= Settings.MotionToleranceMeters);
		TestTrue(TEXT("元の経路の人工的な障壁を低減"), MaximumElevation <= 1.0001);
		TestTrue(TEXT("横方向の物理勾配を低減"), Relaxed->MaximumNormalSlope < 0.001);
		TestTrue(TEXT("実鞍点の検査に渡す候補が存在"), !Relaxed->PassCandidates.empty());
		bool HasSaddle = false;
		for(const PlanetCore::UnitDirection Candidate : Relaxed->PassCandidates) {
			PlanetCore::SpillConnectionSettings ConnectionSettings;
			ConnectionSettings.RadiusMeters = Radius;
			ConnectionSettings.SeaLevelMeters = -100;
			const std::optional<PlanetCore::SpillConnection> Connection =
			PlanetCore::PrepareSpillConnection(Candidate, Source, ConnectionSettings);
			if(Connection && std::abs(Connection->Pass.ElevationMeters - 1) < 1E-8) HasSaddle = true;
		}
		TestTrue(TEXT("緩和候補を元場の実鞍点と両極小で検証"), HasSaddle);
		AddInfo(
			FString::Printf(
				TEXT("Spill string R=%.0f nodes=%llu iterations=%u evaluations=%llu maxY=%.12g crest=%.12g normalSlope=%.12g motion=%.12g"),
				Radius,
				static_cast<unsigned long long>(Relaxed->Path.size()),
				Relaxed->Iterations,
				static_cast<unsigned long long>(Relaxed->Evaluations),
				MaximumY,
				MaximumElevation,
				Relaxed->MaximumNormalSlope,
				Relaxed->MaximumMotionMeters
			)
		);
		// 評価上限と開始前の中断
		Settings.MaximumEvaluations = 100;
		Calls = 0;
		TestFalse(TEXT("評価途中の予算超過を拒否"), PlanetCore::RelaxSpillString(Initial, Source, Settings).has_value());
		TestEqual(TEXT("元場を予算以上に呼出さない"), Calls, UINT64_C(100));
		std::stop_source Stop;
		Stop.request_stop();
		Calls = 0;
		TestFalse(TEXT("開始前の中断を拒否"), PlanetCore::RelaxSpillString(Initial, Source, Settings, Stop.get_token()).has_value());
		TestEqual(TEXT("中断時は元場を呼出さない"), Calls, UINT64_C(0));
		Settings.MaximumEvaluations = 2000000;
		Settings.MaximumVertices = 3;
		TestFalse(TEXT("点数予算を超えた経路を粗くして受理しない"), PlanetCore::RelaxSpillString(Initial, Source, Settings).has_value());
		TestEqual(TEXT("点数予算の検査前に元場を評価しない"), Calls, UINT64_C(0));
	}
	// 有限の候補準備と元場の確認結果の返戻
	return !HasAnyErrors();
}

#endif
