#include "Core/PlanetSpillLake.h"
#include "Core/PlanetTerrainContour.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetContourArcContracts,
	"PlanetGenerator.TerrainContour.SaddleArcs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ水位の複数鞍点を結ぶ岸の弧を解析式と比較し，帰還と別終点への到達を区別する
 * @param Parameters 未使用の自動試験引数
 * @return 三つの半径・回転・両側の弧・終点順序・停止条件が成立した場合に真
 */
bool FPlanetContourArcContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::TerrainContour;
	using PlanetCore::TerrainContourEnd;
	using PlanetCore::TerrainContourSettings;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	constexpr double Length = 80, Amplitude = 20, InitialOffset = 0.005;
	std::uint32_t Cases = 0, IndependentPoints = 0;
	double MaximumDeviation = 0, MaximumLengthError = 0;
	for(const double Radius : { 1000.0, 1E6, 1E9 }) for(const bool Rotated : { false, true }) {
		const Vec3 Up = Rotated ? UnitDirection::FromVector({ -0.2, 0.3, 0.8 })->Vector() : Vec3{ 0, 0, 1 };
		const Vec3 East = UnitDirection::FromVector(Vec3{ 0, 1, 0 }.Cross(Up))->Vector(), North = Up.Cross(East);
		const auto At = [&](const double X, const double Y) -> UnitDirection {
			// 独立な接平面上の解析位置の返戻
			return *UnitDirection::FromVector(Up + East * (X / Radius) + North * (Y / Radius));
		};
		const auto Source = [&](const UnitDirection Direction) -> TerrainSample {
			const Vec3 D = Direction.Vector();
			const double Z = D.Dot(Up), U = D.Dot(East), V = D.Dot(North), X = Radius * U / Z, Y = Radius * V / Z;
			const double Angle = std::numbers::pi * X / Length, Sine = std::sin(Angle), Cosine = std::cos(Angle);
			const Vec3 DX = (East * Z - Up * U) * (Radius / (Z * Z)), DY = (North * Z - Up * V) * (Radius / (Z * Z));
			// 幅８０メートル毎に同水位の鞍点を持つ不変な解析場の返戻
			return {
				Y * Y / (Amplitude * Amplitude) - Sine * Sine,
				DX * (-2 * Sine * Cosine * std::numbers::pi / Length) + DY * (2 * Y / (Amplitude * Amplitude))
			};
		};
		TerrainContourSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.LevelMeters = 0;
		Settings.ChordToleranceMeters = Settings.TerminalRadiusMeters = 0.02;
		Settings.MaximumStepMeters = 20;
		Settings.MaximumLengthMeters = 400;
		Settings.Terminals = { At(0, 0), At(Length, 0), At(2 * Length, 0) };
		// 解析的な球面弧長を，生成側と異なる固定刻みの Simpson 積分で求める
		const auto Speed = [&](const double X) -> double {
			const double Y = Amplitude * std::sin(std::numbers::pi * X / Length);
			const double DY = Amplitude * std::numbers::pi / Length * std::cos(std::numbers::pi * X / Length);
			const double Scale = 1 + (X * X + Y * Y) / (Radius * Radius), Product = (X + Y * DY) / Radius;
			// 球面へ正規化した曲線の物理速度の返戻
			return std::sqrt((1 + DY * DY) * Scale - Product * Product) / Scale;
		};
		constexpr unsigned Intervals = 4096;
		const double Step = (Length - InitialOffset) / Intervals;
		double ExactLength = Speed(InitialOffset) + Speed(Length);
		for(unsigned Index = 1; Index < Intervals; ++Index) ExactLength += Speed(InitialOffset + Step * Index) * (Index % 2 ? 4 : 2);
		ExactLength *= Step / 3;
		for(const bool Upper : { false, true }) {
			const double InitialX = Upper ? Length - InitialOffset : InitialOffset;
			const double InitialY = (Upper ? 1.0 : -1.0) * Amplitude * std::sin(std::numbers::pi * InitialX / Length);
			const UnitDirection Initial = At(InitialX, InitialY);
			const std::uint32_t Expected = Upper ? 0 : 1;
			const std::optional<TerrainContour> Arc = PlanetCore::TraceTerrainContour(Initial, Source, Settings);
			// 解析上は存在する弧を準備出来ない場合の返戻
			if(!TestTrue(TEXT("同水位の別鞍点へ向かう弧を準備"), Arc.has_value())) return false;
			// 弧の上側と下側では曲率による長さの差が有るので，積分の範囲も実際の始点へ合わせる
			double ArcLength = ExactLength;
			if(Upper) {
				ArcLength = Speed(0) + Speed(Length - InitialOffset);
				for(unsigned Index = 1; Index < Intervals; ++Index) ArcLength += Speed(Step * Index) * (Index % 2 ? 4 : 2);
				ArcLength *= Step / 3;
			}
			AddInfo(
				FString::Printf(
					TEXT("Saddle arc R=%.0f rotated=%u upper=%u end=%u terminal=%d vertices=%u length=%.12g exact=%.12g"),
					Radius,
					static_cast<unsigned>(Rotated),
					static_cast<unsigned>(Upper),
					static_cast<unsigned>(Arc->End),
					Arc->TerminalIndex ? static_cast<int>(*Arc->TerminalIndex) : -1,
					static_cast<unsigned>(Arc->Samples.size()),
					Arc->Samples.back().DistanceMeters,
					ArcLength
				)
			);
			// 不完全な追跡を終点接続の合格として検査しない返戻
			if(!TestTrue(TEXT("出発鞍点以外の終点へ到達"), Arc->End == TerrainContourEnd::Terminal && Arc->TerminalIndex == Expected)) return false;
			TestTrue(
				TEXT("終点の位置を入力の鞍点と同一に保持"),
				(Arc->Samples.back().Direction.Vector() - Settings.Terminals[Expected].Vector()).Length() == 0
			);
			MaximumLengthError = std::max(MaximumLengthError, std::abs(Arc->Samples.back().DistanceMeters - ArcLength));
			for(std::size_t Index = 1; Index < Arc->Samples.size(); ++Index) for(unsigned Sample = 0; Sample < 17; ++Sample) {
				const double Fraction = Sample / 16.0;
				const Vec3 A = Arc->Samples[Index - 1].Direction.Vector(), B = Arc->Samples[Index].Direction.Vector();
				const Vec3 D = UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction)->Vector();
				const double Z = D.Dot(Up), X = Radius * D.Dot(East) / Z, Y = Radius * D.Dot(North) / Z;
				const double ExpectedY = (Upper ? 1.0 : -1.0) * Amplitude * std::sin(std::numbers::pi * X / Length);
				MaximumDeviation = std::max(MaximumDeviation, std::abs(Y - ExpectedY));
				TestTrue(TEXT("隣の窪地へ等高枝を飛越さない"), X >= -1E-5 && X <= Length + 1E-5);
				++IndependentPoints;
			}
			TerrainContourSettings Reversed = Settings;
			std::reverse(Reversed.Terminals.begin(), Reversed.Terminals.end());
			const std::optional<TerrainContour> Reordered = PlanetCore::TraceTerrainContour(Initial, Source, Reversed);
			// 候補の並換えで同じ弧を失う場合の返戻
			if(!TestTrue(TEXT("終点の並換えでも到達"), Reordered && Reordered->End == TerrainContourEnd::Terminal)) return false;
			TestTrue(TEXT("終点番号は並換後の入力へ対応"), Reordered->TerminalIndex == 2 - Expected);
			TestEqual(TEXT("終点順序で追跡の長さを変えない"), Reordered->Samples.back().DistanceMeters, Arc->Samples.back().DistanceMeters);
			// 実際の鞍点の主曲率から同じ弧を開始し，両端を実鞍点まで含める
			const double PassX = Upper ? Length : 0, Scale = std::sqrt(1 + PassX * PassX / (Radius * Radius));
			PlanetCore::TerrainCriticalPoint Pass { At(PassX, 0) };
			Pass.ElevationMeters = 0;
			Pass.MinimumCurvatureDirection = East - Pass.Direction.Vector() * East.Dot(Pass.Direction.Vector());
			Pass.MinimumCurvaturePerMeter = -2 * std::numbers::pi * std::numbers::pi / (Length * Length) * std::pow(Scale, 4);
			Pass.MaximumCurvaturePerMeter = 2 / (Amplitude * Amplitude) * Scale * Scale;
			PlanetCore::SpillLakeDiagnostics Report;
			const std::optional<TerrainContour> Started = PlanetCore::TraceSpillLakeArc(Pass, !Upper, Source, Settings, {}, &Report);
			// 主曲率からの正規の開始経路を拒否した場合の返戻
			if(!TestTrue(TEXT("主曲率から別鞍点まで岸の弧を準備"), Started.has_value())) return false;
			TestTrue(TEXT("弧の到達鞍点を保持"), Started->End == TerrainContourEnd::Terminal && Started->TerminalIndex == Expected);
			TestEqual(TEXT("開始鞍点の累積距離は零"), Started->Samples.front().DistanceMeters, 0.0);
			TestTrue(TEXT("開始の実鞍点を保持"), (Started->Samples.front().Direction.Vector() - Pass.Direction.Vector()).Length() == 0);
			TestEqual(TEXT("診断の長さは開始の接続を含む"), Report.LengthMeters, Started->Samples.back().DistanceMeters);
			TestEqual(TEXT("診断の頂点数は開始の鞍点を含む"), Report.Vertices, static_cast<std::uint32_t>(Started->Samples.size()));
			TerrainContourSettings ExactBudget = Settings;
			ExactBudget.MaximumVertices = static_cast<std::uint32_t>(Started->Samples.size());
			const std::optional<TerrainContour> Fits = PlanetCore::TraceSpillLakeArc(Pass, !Upper, Source, ExactBudget, {}, &Report);
			TestTrue(TEXT("開始鞍点を含む正確な頂点予算は受理"), Fits && Fits->Samples.size() <= ExactBudget.MaximumVertices);
			--ExactBudget.MaximumVertices;
			TestFalse(TEXT("開始鞍点の追加で頂点予算を越えない"), PlanetCore::TraceSpillLakeArc(Pass, !Upper, Source, ExactBudget, {}, &Report).has_value());
			++Cases;
		}
		// 最初の鞍点だけを指定すると，中間の同水位鞍点を越えた偽の帰還にはしない
		const UnitDirection Initial = At(InitialOffset, -Amplitude * std::sin(std::numbers::pi * InitialOffset / Length));
		TerrainContourSettings Missing = Settings;
		Missing.Terminals.erase(Missing.Terminals.begin() + 1, Missing.Terminals.end());
		const std::optional<TerrainContour> Incomplete = PlanetCore::TraceTerrainContour(Initial, Source, Missing);
		TestTrue(
			TEXT("未登録の鞍点で止まる旧来の経路を保持"),
			Incomplete && Incomplete->End != TerrainContourEnd::Terminal && Incomplete->End != TerrainContourEnd::Closed &&
			!Incomplete->TerminalIndex
		);
		TerrainContourSettings Limited = Settings;
		Limited.MaximumLengthMeters = ExactLength * 0.5;
		const std::optional<TerrainContour> Short = PlanetCore::TraceTerrainContour(Initial, Source, Limited);
		TestTrue(
			TEXT("距離予算に達した部分経路は終点到達にしない"),
			Short && Short->End == TerrainContourEnd::LengthLimit && !Short->TerminalIndex &&
			Short->Samples.back().DistanceMeters <= Limited.MaximumLengthMeters
		);
		Limited = Settings;
		Limited.MaximumVertices = 8;
		const std::optional<TerrainContour> Few = PlanetCore::TraceTerrainContour(Initial, Source, Limited);
		TestTrue(TEXT("頂点予算を超える弧を閉鎖しない"), Few && Few->End == TerrainContourEnd::VertexLimit && !Few->TerminalIndex);
		Limited = Settings;
		Limited.Terminals.push_back(At(Length, 0));
		TestFalse(TEXT("重複した終点は番号を勝手に選ばず拒否"), Limited.IsValid());
		Limited = Settings;
		Limited.Terminals.push_back(At(Length + 0.1, 0));
		TestFalse(TEXT("識別半径が重なる終点は拒否"), Limited.IsValid());
		Limited = Settings;
		Limited.Terminals.assign(1025, At(0, 0));
		TestFalse(TEXT("終点の保持上限を超える入力は拒否"), Limited.IsValid());
		Limited = Settings;
		Limited.Terminals.push_back(At(Length * 0.25, 0));
		TestFalse(TEXT("別水位の終点を岸へ接続しない"), PlanetCore::TraceTerrainContour(Initial, Source, Limited).has_value());
		std::stop_source Cancel;
		std::uint32_t Calls = 0;
		const auto Cancelling = [&](const UnitDirection Direction) -> TerrainSample {
			if(++Calls == 2) Cancel.request_stop();
			// 終点の照会中に中断した元場の返戻
			return Source(Direction);
		};
		TestFalse(TEXT("終点の準備中の中断を保持"), PlanetCore::TraceTerrainContour(Initial, Cancelling, Settings, Cancel.get_token()).has_value());
		TestEqual(TEXT("中断後に残りの終点を照会しない"), Calls, std::uint32_t{ 2 });
	}
	TestTrue(TEXT("独立な折線標本は解析曲線から３センチ以内"), MaximumDeviation < 0.03);
	TestTrue(TEXT("独立な積分弧長と５センチ以内で一致"), MaximumLengthError < 0.05);
	AddInfo(
		FString::Printf(
			TEXT("Saddle arc contracts: cases=%u independentPoints=%u maxDeviation=%.12g maxLengthError=%.12g"),
			Cases,
			IndependentPoints,
			MaximumDeviation,
			MaximumLengthError
		)
	);
	// 弧の精度と停止条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
