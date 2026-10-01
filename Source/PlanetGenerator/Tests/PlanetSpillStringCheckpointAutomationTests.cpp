#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillString.h"
#include "Misc/AutomationTest.h"
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	struct StringCheckpointFixture {
		double Radius;
		std::uint64_t Calls = 0;
		PlanetCore::TerrainSample Sample(const PlanetCore::UnitDirection Direction);
		PlanetCore::UnitDirection Direction(const double X, const double Y) const;
		std::vector<PlanetCore::UnitDirection> Initial() const;
		static std::vector<std::uint64_t> Values(const PlanetCore::SpillString &String);
	};

	/**
	 * 独立な二重窪地の標高と全接線勾配を照会し，実回数を記録する
	 * @param Direction 球面上の地点
	 * @return 峠が標高１ｍの解析標高と勾配
	 */
	PlanetCore::TerrainSample StringCheckpointFixture::Sample(const PlanetCore::UnitDirection Direction) {
		++Calls;
		const PlanetCore::Vec3 P = Direction.Vector();
		const double X = Radius * std::atan2(P.Y, P.X), Y = Radius * std::asin(P.Z), Horizontal = P.X * P.X + P.Y * P.Y;
		const double Q = X * X / 400 - 1, DX = Q * X / 100, DY = Y / 200;
		const PlanetCore::Vec3 G {
			-Radius * DX * P.Y / Horizontal,
			Radius * DX * P.X / Horizontal,
			Radius * DY / std::sqrt(1 - P.Z * P.Z)
		};
		// 同じ解析式の物理値の返戻
		return { Q * Q + Y * Y / 400, G - P * G.Dot(P) };
	}

	/**
	 * 解析場の物理座標を同じ球上へ写す
	 * @param X 経度方向の距離
	 * @param Y 緯度方向の距離
	 * @return 同じ物理半径上の正規方向
	 */
	PlanetCore::UnitDirection StringCheckpointFixture::Direction(const double X, const double Y) const {
		const double Longitude = X / Radius, Latitude = Y / Radius;
		// 球面上の位置の返戻
		return *PlanetCore::UnitDirection::FromVector(
			{ std::cos(Latitude) * std::cos(Longitude), std::cos(Latitude) * std::sin(Longitude), std::sin(Latitude) }
		);
	}

	/**
	 * 独立な両極小を固定し，谷から外れた有限の初期経路を作成する
	 * @return 形状を変える同じ入力経路
	 */
	std::vector<PlanetCore::UnitDirection> StringCheckpointFixture::Initial() const {
		std::vector<PlanetCore::UnitDirection> Result;
		for(unsigned I = 0; I < 33; ++I) {
			const double T = I / 32.0, X = -20 + 40 * T, Y = !I || I == 32 ? 0 : 10 * std::sin(std::acos(-1.0) * T);
			Result.push_back(Direction(X, Y));
		}
		// 端点が厳密に解析極小である経路の返戻
		return Result;
	}

	/**
	 * 実照会数を除き，経路と候補及び最終形状の全成分を型の詰物無で採集する
	 * 計算量：時間・追加空間 O(P)，P は経路と候補の総点数
	 * @param String 同じ設定の実準備結果
	 * @return 数値成分の全ビット
	 */
	std::vector<std::uint64_t> StringCheckpointFixture::Values(const PlanetCore::SpillString &String) {
		std::vector<std::uint64_t> Result;
		Result.push_back(String.Path.size());
		Result.push_back(String.PassCandidates.size());
		Result.push_back(String.Iterations);
		Result.push_back(String.AcceptedCheckpoint);
		Result.push_back(std::bit_cast<std::uint64_t>(String.MaximumMotionMeters));
		Result.push_back(std::bit_cast<std::uint64_t>(String.MaximumNormalSlope));
		for(const std::vector<PlanetCore::UnitDirection> *const Points : { &String.Path, &String.PassCandidates }) {
			for(const PlanetCore::UnitDirection D : *Points) {
				const PlanetCore::Vec3 V = D.Vector();
				for(const double Component : { V.X, V.Y, V.Z }) Result.push_back(std::bit_cast<std::uint64_t>(Component));
			}
		}
		// 検査対象の物理的な成分列の返戻
		return Result;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetStringCheckpointAcceptance,
	"PlanetGenerator.SpillString.CheckpointAcceptance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 外部の実接続確認が成立した途中経路だけを返し，従来経路と中断・予算を維持する
 * @param Parameters 未使用の実行引数
 * @return 三半径・偽条件時のビット一致・実峠による早期受理・失敗経路の確認が成立した場合に真
 */
bool FPlanetStringCheckpointAcceptance::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		StringCheckpointFixture Fixture { Radius };
		const std::vector<PlanetCore::UnitDirection> Initial = Fixture.Initial();
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			// 同じ所有場の値と費用の返戻
			return Fixture.Sample(D);
		};
		PlanetCore::SpillStringSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SpacingMeters = 1;
		Settings.MaximumMoveMeters = 0.5;
		Settings.MaximumIterations = 256;
		const std::optional<PlanetCore::SpillString> Cold = PlanetCore::RelaxSpillString(Initial, Source, Settings);
		// 基準の正規経路を準備出来ない場合の返戻
		if(!TestTrue(TEXT("受理関数無しの従来経路"), Cold.has_value())) return false;
		TestEqual(TEXT("従来経路の実照会数"), Fixture.Calls, Cold->Evaluations);
		Fixture.Calls = 0;
		unsigned Checks = 0;
		const auto NeverAccept = [&](const PlanetCore::SpillString &Checkpoint) -> bool {
			++Checks;
			TestEqual(TEXT("指定した間隔でだけ外部条件を確認"), Checkpoint.Iterations % Settings.AcceptanceStride, 0U);
			for(const std::size_t Index : { std::size_t(0), Checkpoint.Path.size() - 1 }) {
				const PlanetCore::Vec3 A = Checkpoint.Path[Index].Vector(), B = Index ? Initial.back().Vector() : Initial.front().Vector();
				TestTrue(TEXT("途中でも固定した端点の全成分を保持"), A.X == B.X && A.Y == B.Y && A.Z == B.Z);
			}
			// 未成立の外部条件の返戻
			return false;
		};
		const std::optional<PlanetCore::SpillString> Continued =
		PlanetCore::RelaxSpillString(Initial, Source, Settings, {}, NeverAccept);
		// 偽の外部条件で正規経路を失った場合の返戻
		if(!TestTrue(TEXT("未確認の場合は従来の反復を維持"), Continued.has_value())) return false;
		TestTrue(TEXT("途中確認が実際に発生"), Checks > 0);
		TestTrue(TEXT("受理しない経路の物理値はビット一致"), StringCheckpointFixture::Values(*Cold) == StringCheckpointFixture::Values(*Continued));
		TestEqual(TEXT("途中の評価も予算と費用に計上"), Fixture.Calls, Continued->Evaluations);
		Fixture.Calls = 0;
		unsigned EarlyChecks = 0;
		const auto HasActualPass = [&](const PlanetCore::SpillString &Checkpoint) -> bool {
			++EarlyChecks;
			PlanetCore::SpillConnectionSettings ConnectionSettings;
			ConnectionSettings.RadiusMeters = Radius;
			ConnectionSettings.SeaLevelMeters = -100;
			for(const PlanetCore::UnitDirection D : Checkpoint.PassCandidates) {
				const std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(D, Source, ConnectionSettings);
				const bool HasResolved = Connection && Connection->Branches[0].Minimum && Connection->Branches[1].Minimum &&
				std::abs(Connection->Pass.ElevationMeters - 1) < 1E-8;
				// 経路の形だけでなく，独立な実鞍点と両極小を確認した返戻
				if(HasResolved) return true;
			}
			// 実接続を確認出来ない候補の返戻
			return false;
		};
		const std::optional<PlanetCore::SpillString> Early = PlanetCore::RelaxSpillString(Initial, Source, Settings, {}, HasActualPass);
		// 早期の実峠確認を出来ない場合の返戻
		if(!TestTrue(TEXT("外部の実鞍点確認による受理"), Early.has_value())) return false;
		TestTrue(TEXT("早期受理を変位収束と区別"), Early->AcceptedCheckpoint);
		TestTrue(TEXT("従来の収束前に実条件を確認"), Early->Iterations < Cold->Iterations);
		TestTrue(TEXT("外部の実鞍点照会を含めても実費用を削減"), Fixture.Calls < Cold->Evaluations);
		AddInfo(
			FString::Printf(
				TEXT("String checkpoints R=%.0f continuedChecks=%u cold=%u/%llu early=%u/%llu actualCalls=%llu earlyChecks=%u"),
				Radius,
				Checks,
				Cold->Iterations,
				Cold->Evaluations,
				Early->Iterations,
				Early->Evaluations,
				Fixture.Calls,
				EarlyChecks
			)
		);
		Fixture.Calls = 0;
		Settings.AcceptanceStride = 0;
		TestFalse(TEXT("受理関数がある場合の零間隔を拒否"), PlanetCore::RelaxSpillString(Initial, Source, Settings, {}, NeverAccept).has_value());
		TestEqual(TEXT("不正間隔では元場を照会しない"), Fixture.Calls, UINT64_C(0));
		const std::optional<PlanetCore::SpillString> WithoutAcceptance = PlanetCore::RelaxSpillString(Initial, Source, Settings);
		TestTrue(
			TEXT("未使用の間隔は従来の正規経路を変えない"),
			WithoutAcceptance && StringCheckpointFixture::Values(*WithoutAcceptance) == StringCheckpointFixture::Values(*Cold)
		);
		Settings.AcceptanceStride = 16;
		Settings.MaximumEvaluations = 100;
		Fixture.Calls = 0;
		TestFalse(TEXT("外部条件があっても評価不足を拒否"), PlanetCore::RelaxSpillString(Initial, Source, Settings, {}, NeverAccept).has_value());
		TestEqual(TEXT("途中確認でも元場を予算以上に呼ばない"), Fixture.Calls, UINT64_C(100));
		Settings.MaximumEvaluations = 2000000;
		Settings.AcceptanceStride = 1;
		std::stop_source Stop;
		Fixture.Calls = 0;
		std::uint64_t CancelCalls = 0;
		const auto Cancel = [&](const PlanetCore::SpillString &Checkpoint) -> bool {
			static_cast<void>(Checkpoint);
			CancelCalls = Fixture.Calls;
			Stop.request_stop();
			// 真の外部条件と同時の中断を，成功へ置換えない返戻
			return true;
		};
		TestFalse(TEXT("外部の確認による中断を拒否"), PlanetCore::RelaxSpillString(Initial, Source, Settings, Stop.get_token(), Cancel).has_value());
		TestTrue(TEXT("実際の途中確認で中断"), CancelCalls > 0);
		TestEqual(TEXT("外部中断後の元場照会無し"), Fixture.Calls, CancelCalls);
		Fixture.Calls = 0;
		Checks = 0;
		TestFalse(
			TEXT("中断済の場合は確認を起動しない"),
			PlanetCore::RelaxSpillString(Initial, Source, Settings, Stop.get_token(), NeverAccept).has_value()
		);
		TestEqual(TEXT("開始前の中断で元場照会無し"), Fixture.Calls, UINT64_C(0));
		TestEqual(TEXT("開始前の中断で外部確認無し"), Checks, 0U);
	}
	// 実接続と正規経路及び失敗経路の検証結果の返戻
	return !HasAnyErrors();
}

#endif
