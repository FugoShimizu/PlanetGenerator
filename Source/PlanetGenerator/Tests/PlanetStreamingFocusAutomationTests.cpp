#include "Core/PlanetStreamingFocus.h"
#include "Core/PlanetSurfaceCorridor.h"
#include "Misc/AutomationTest.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetStreamingFocusContracts, "PlanetGenerator.StreamingFocus.Contracts",
								 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 身体の全方位と前方通路，逆走・加速・静止時の被覆を独立に検査する
 * @param Parameters 未使用の検査引数
 * @return 全標本と不正入力が期待通りの場合に真
 */
bool FPlanetStreamingFocusContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	using PlanetCore::PredictStreamingFocus;
	using PlanetCore::StreamingFocus;
	using PlanetCore::SurfaceCorridor;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	std::uint32_t Checked = 0, IntersectionMismatches = 0;
	for(const double Radius : {1000.0, 1000000.0, 1000000000.0}) {
		for(const Vec3 Axis : {Vec3{1, 2, 3}, Vec3{0, 0, 1}, Vec3{-1, 0, 0}}) {
			const Vec3 Up = UnitDirection::FromVector(Axis)->Vector();
			const Vec3 East =
				UnitDirection::FromVector(Up.Cross(std::abs(Up.Z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}))->Vector();
			const Vec3 North = Up.Cross(East);
			const Vec3 Position = Up * (Radius + 2);
			const double Tolerance = 256 * std::numeric_limits<double>::epsilon() * Radius;
			for(const double Speed : {-20.0, -9.0, 0.0, 4.5, 9.0, 20.0}) {
				for(const double Horizon : {0.0, 0.25, 1.5, 3.0, 100.0}) {
					const std::optional<StreamingFocus> Focus =
						PredictStreamingFocus(Position, East * Speed, East, Radius, Horizon, 20);
					if(!TestTrue(TEXT("先読み通路を生成"), Focus.has_value())) {
						// 有効な通路が無ければ幾何検査を続行出来ない．
						return false;
					}
					const std::optional<SurfaceCorridor> Region =
						SurfaceCorridor::Create(Focus->StartMeters, Focus->EndMeters, Radius, Focus->RadiusMeters);
					if(!TestTrue(TEXT("短い球面通路を生成"), Region.has_value())) {
						// 有効な通路が無ければ幾何検査を続行出来ない．
						return false;
					}
					TestEqual(TEXT("全方位半径を維持"), Focus->RadiusMeters, 24.0);
					TestTrue(TEXT("始点の高度を保持"), std::abs(Focus->StartMeters.Length() - Position.Length()) <= Tolerance);
					TestTrue(TEXT("終点の高度を保持"), std::abs(Focus->EndMeters.Length() - Position.Length()) <= Tolerance);
					TestTrue(TEXT("先読み長を制限"), Region->Span.Length() * Radius <= 36 + Tolerance);
					for(std::uint32_t Step = 0; Step <= 4; ++Step) {
						const Vec3 OnPath =
							UnitDirection::FromVector(Region->StartDirection * (4 - Step) + Region->EndDirection * Step)
								->Vector();
						TestTrue(TEXT("前方中心線を被覆"), Region->Contains(OnPath));
					}
					for(std::uint32_t Index = 0; Index < 64; ++Index) {
						const double Angle = 2 * std::numbers::pi * Index / 64;
						const Vec3 Around = East * std::cos(Angle) + North * std::sin(Angle);
						const double Chord = 23.9 / Radius;
						const Vec3 Edge = Up * (1 - Chord * Chord / 2) + Around * (Chord * std::sqrt(1 - Chord * Chord / 4));
						TestTrue(TEXT("身体周囲23.9mを全方位被覆"), Region->Contains(Edge));
						const double OuterChord = 60 / Radius;
						const Vec3 Outside = Up * (1 - OuterChord * OuterChord / 2) +
											 Around * (OuterChord * std::sqrt(1 - OuterChord * OuterChord / 4));
						const double Bound = Region->TubeRadiusMeters + Region->SagittaMeters + 3 +
											 64 * std::numeric_limits<double>::epsilon() * Radius;
						if(Region->Intersects(Edge, 3) != (Region->DistanceMeters(Edge) <= Bound) ||
						   Region->Intersects(Outside, 3) != (Region->DistanceMeters(Outside) <= Bound)) {
							++IntersectionMismatches;
						}
						++Checked;
					}
				}
			}
			const std::optional<StreamingFocus> Accelerated =
				PredictStreamingFocus(Position, East * 4.5, East, Radius, 0.5, 20);
			const std::optional<StreamingFocus> Idle = PredictStreamingFocus(Position, {}, East, Radius, 0.5, 20);
			const std::optional<StreamingFocus> Reverse = PredictStreamingFocus(Position, East * -9, East, Radius, 0.5, 20);
			if(!TestTrue(TEXT("加速・停止・逆走の通路を生成"), Accelerated && Idle && Reverse)) {
				// 有効な通路が無ければ方向検査を続行出来ない．
				return false;
			}
			TestTrue(TEXT("加速前に10m先読み"), (Accelerated->EndMeters - Position).Dot(East) >= 9.9);
			TestTrue(TEXT("静止時も前方を先読み"), (Idle->EndMeters - Position).Dot(East) >= 9.9);
			TestTrue(TEXT("逆走方向を優先"), (Reverse->EndMeters - Position).Dot(East) <= -9.9);
		}
	}
	const double Nan = std::numeric_limits<double>::quiet_NaN();
	if(!TestFalse(TEXT("原点を拒否"), PredictStreamingFocus({}, {}, {}, 1000, 1, 20).has_value()) ||
	   !TestFalse(TEXT("過小半径を拒否"), PredictStreamingFocus({1000, 0, 0}, {}, {}, 999, 1, 20).has_value()) ||
	   !TestFalse(TEXT("負の時間を拒否"), PredictStreamingFocus({1000, 0, 0}, {}, {}, 1000, -1, 20).has_value()) ||
	   !TestFalse(TEXT("非有限速度を拒否"), PredictStreamingFocus({1000, 0, 0}, {Nan, 0, 0}, {}, 1000, 1, 20).has_value()) ||
	   !TestFalse(TEXT("非有限方向を拒否"), PredictStreamingFocus({1000, 0, 0}, {}, {Nan, 0, 0}, 1000, 1, 20).has_value()) ||
	   !TestFalse(TEXT("非有限時間を拒否"), PredictStreamingFocus({1000, 0, 0}, {}, {}, 1000, Nan, 20).has_value()) ||
	   !TestFalse(TEXT("負の許容速度を拒否"), PredictStreamingFocus({1000, 0, 0}, {}, {}, 1000, 1, -1).has_value()) ||
	   !TestFalse(TEXT("過大な許容速度を拒否"), PredictStreamingFocus({1000, 0, 0}, {}, {}, 1000, 1, 21).has_value()) ||
	   !TestFalse(TEXT("非有限の許容速度を拒否"), PredictStreamingFocus({1000, 0, 0}, {}, {}, 1000, 1, Nan).has_value())) {
		// 不正入力の受理時は検査を失敗とする．
		return false;
	}
	TestEqual(TEXT("タイル交差判定は元の距離式と一致"), IntersectionMismatches, 0U);
	AddInfo(FString::Printf(TEXT("Streaming corridor: %u body-boundary checks."), Checked));
	// 全条件の成否を返す．
	return !HasAnyErrors();
}

#endif
