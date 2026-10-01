#include "Core/PlanetLakeBoundary.h"
#include "Core/PlanetSpillLake.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLakeBoundaryContracts,
	"PlanetGenerator.LakeTopology.BoundaryLoops",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同水位の三鞍点間の四つの弧を二つの輪郭へし，解析面積・所属範囲・閉鎖を独立に検査する
 * @param Parameters 未使用の自動試験引数
 * @return 弧の欠落・重複・不整合・予算・中断を拒否し，正規の全弧が一度ずつ閉鎖する場合に真
 */
bool FPlanetLakeBoundaryContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::LakeBoundaryLoop;
	using PlanetCore::TerrainContour;
	using PlanetCore::TerrainCriticalPoint;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	constexpr double Length = 80, Amplitude = 20;
	std::uint32_t Cases = 0, Vertices = 0;
	double MaximumAreaError = 0;
	for(const double Radius : { 1000.0, 1E6, 1E9 }) for(const bool Rotated : { false, true }) {
		const Vec3 Up = Rotated ? UnitDirection::FromVector({ -0.2, 0.3, 0.8 })->Vector() : Vec3{ 0, 0, 1 };
		const Vec3 East = UnitDirection::FromVector(Vec3{ 0, 1, 0 }.Cross(Up))->Vector(), North = Up.Cross(East);
		const auto At = [&](const double X, const double Y) -> UnitDirection {
			// 共通の物理座標から球面へ写した方向の返戻
			return *UnitDirection::FromVector(Up + East * (X / Radius) + North * (Y / Radius));
		};
		const auto Source = [&](const UnitDirection Direction) -> TerrainSample {
			const Vec3 D = Direction.Vector();
			const double Z = D.Dot(Up), U = D.Dot(East), V = D.Dot(North), X = Radius * U / Z, Y = Radius * V / Z;
			const double Angle = std::numbers::pi * X / Length, Sine = std::sin(Angle), Cosine = std::cos(Angle);
			const Vec3 DX = (East * Z - Up * U) * (Radius / (Z * Z)), DY = (North * Z - Up * V) * (Radius / (Z * Z));
			// 等間隔の同水位鞍点と二つの既知の湖を持つ場の返戻
			return {
				Y * Y / (Amplitude * Amplitude) - Sine * Sine,
				DX * (-2 * Sine * Cosine * std::numbers::pi / Length) + DY * (2 * Y / (Amplitude * Amplitude))
			};
		};
		PlanetCore::TerrainContourSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.ChordToleranceMeters = Settings.TerminalRadiusMeters = 0.02;
		Settings.MaximumStepMeters = 20;
		Settings.MaximumLengthMeters = 1000;
		std::vector<TerrainCriticalPoint> Passes;
		for(unsigned Index = 0; Index < 3; ++Index) {
			const double X = Length * Index, Scale = std::sqrt(1 + X * X / (Radius * Radius));
			TerrainCriticalPoint Pass { At(X, 0) };
			Pass.ElevationMeters = 0;
			Pass.MinimumCurvatureDirection = East - Pass.Direction.Vector() * East.Dot(Pass.Direction.Vector());
			Pass.MinimumCurvaturePerMeter = -2 * std::numbers::pi * std::numbers::pi / (Length * Length) * std::pow(Scale, 4);
			Pass.MaximumCurvaturePerMeter = 2 / (Amplitude * Amplitude) * Scale * Scale;
			Settings.Terminals.push_back(Pass.Direction);
			Passes.push_back(Pass);
		}
		std::vector<TerrainContour> Arcs;
		std::uint32_t SourceVertices = 0;
		for(const std::pair<unsigned, bool> Arm : { std::pair{ 0U, true }, { 1U, false }, { 1U, true }, { 2U, false } }) {
			std::optional<TerrainContour> Arc = PlanetCore::TraceSpillLakeArc(Passes[Arm.first], Arm.second, Source, Settings);
			// 既知の湖岸の弧が未準備なら輪郭の合格にしない返戻
			if(!TestTrue(TEXT("二つの湖の四つの岸の弧を準備"), Arc.has_value())) return false;
			SourceVertices += static_cast<std::uint32_t>(Arc->Samples.size());
			Arcs.push_back(std::move(*Arc));
		}
		Settings.MaximumVertices = SourceVertices;
		const std::optional<std::vector<LakeBoundaryLoop>> Loops = PlanetCore::AssembleLakeBoundary(Passes, Arcs, Settings);
		// 正規の接続を拒否した場合の返戻
		if(!TestTrue(TEXT("同水位で接する二つの輪郭を組立"), Loops.has_value())) return false;
		TestEqual(TEXT("二つの湖の輪郭を保持"), Loops->size(), std::size_t{ 2 });
		std::array<unsigned, 4> Uses {};
		for(const LakeBoundaryLoop &Loop : *Loops) {
			TestTrue(TEXT("閉鎖を終点到達と混同しない"), Loop.Contour.End == PlanetCore::TerrainContourEnd::Closed && !Loop.Contour.TerminalIndex);
			TestTrue(
				TEXT("閉鎖の両端は同一座標"),
				(Loop.Contour.Samples.front().Direction.Vector() - Loop.Contour.Samples.back().Direction.Vector()).Length() == 0
			);
			TestEqual(TEXT("二本の有向弧で一周"), Loop.Arcs.size(), std::size_t{ 2 });
			for(const std::uint32_t Arc : Loop.Arcs) {
				// 組立結果が入力を失う場合の返戻
				if(!TestTrue(TEXT("輪郭が参照する入力弧は範囲内"), Arc < Uses.size())) return false;
				++Uses[Arc];
			}
			TestTrue(TEXT("低い湖側を囲む正の球面面積"), Loop.SignedAreaMetersSquared > 0);
			double TwiceArea = 0, Previous = -1;
			const double LowerX = Loop.Arcs.front() < 2 ? 0.0 : Length;
			for(std::size_t Point = 0; Point < Loop.Contour.Samples.size(); ++Point) {
				const PlanetCore::TerrainContourSample &Sample = Loop.Contour.Samples[Point];
				TestTrue(TEXT("接続点で弧長が逆転せず厳密に進む"), Sample.DistanceMeters > Previous);
				Previous = Sample.DistanceMeters;
				const Vec3 A = Sample.Direction.Vector();
				const Vec3 B = Loop.Contour.Samples[(Point + 1) % Loop.Contour.Samples.size()].Direction.Vector();
				const double AX = Radius * A.Dot(East) / A.Dot(Up), AY = Radius * A.Dot(North) / A.Dot(Up);
				const double BX = Radius * B.Dot(East) / B.Dot(Up), BY = Radius * B.Dot(North) / B.Dot(Up);
				TwiceArea += AX * BY - BX * AY;
				TestTrue(TEXT("隣の湖の境界へ枝を繋ぎ替えない"), AX >= LowerX - 1E-5 && AX <= LowerX + Length + 1E-5);
				++Vertices;
			}
			const double ExactArea = 4 * Amplitude * Length / std::numbers::pi;
			MaximumAreaError = std::max(MaximumAreaError, std::abs(TwiceArea * 0.5 - ExactArea));
		}
		for(const unsigned Count : Uses) TestEqual(TEXT("指定した各弧を一度ずつ使う"), Count, 1U);
		std::vector<TerrainContour> Reordered(Arcs.rbegin(), Arcs.rend());
		const std::optional<std::vector<LakeBoundaryLoop>> Reverse = PlanetCore::AssembleLakeBoundary(Passes, Reordered, Settings);
		TestTrue(TEXT("弧の入力順を反転しても二つの輪郭"), Reverse && Reverse->size() == 2);
		std::vector<TerrainContour> Broken = Arcs;
		Broken.erase(Broken.begin());
		TestFalse(TEXT("欠けた弧を線分で隠さない"), PlanetCore::AssembleLakeBoundary(Passes, Broken, Settings).has_value());
		Broken = Arcs;
		Broken[1] = Broken[0];
		TestFalse(TEXT("重複した出発枝を拒否"), PlanetCore::AssembleLakeBoundary(Passes, Broken, Settings).has_value());
		Broken = Arcs;
		Broken[0].TerminalIndex = 2;
		TestFalse(TEXT("実終点と異なる鞍点番号を拒否"), PlanetCore::AssembleLakeBoundary(Passes, Broken, Settings).has_value());
		Broken = Arcs;
		++Broken[0].Samples[3].ElevationMeters;
		TestFalse(TEXT("岸の途中の別水位を拒否"), PlanetCore::AssembleLakeBoundary(Passes, Broken, Settings).has_value());
		Broken = Arcs;
		std::swap(Broken[0].Samples[20].Direction, Broken[0].Samples[40].Direction);
		TestFalse(TEXT("輪郭内の自交差を拒否"), PlanetCore::AssembleLakeBoundary(Passes, Broken, Settings).has_value());
		PlanetCore::TerrainContourSettings Budget = Settings;
		--Budget.MaximumVertices;
		TestFalse(TEXT("全入力の保持予算を一つでも超えれば拒否"), PlanetCore::AssembleLakeBoundary(Passes, Arcs, Budget).has_value());
		Budget = Settings;
		Budget.MaximumLengthMeters = 100;
		TestFalse(TEXT("各弧が短くても合計の長さ超過を拒否"), PlanetCore::AssembleLakeBoundary(Passes, Arcs, Budget).has_value());
		std::stop_source Cancel;
		Cancel.request_stop();
		TestFalse(TEXT("中断済の組立を拒否"), PlanetCore::AssembleLakeBoundary(Passes, Arcs, Settings, Cancel.get_token()).has_value());
		++Cases;
	}
	TestTrue(TEXT("独立な解析面積との差は２平方メートル以内"), MaximumAreaError < 2);
	AddInfo(
		FString::Printf(
			TEXT("Lake boundary loops: cases=%u loops=%u vertices=%u maxPlanarAreaError=%.12g"),
			Cases,
			Cases * 2,
			Vertices,
			MaximumAreaError
		)
	);
	// 指定した弧の接続と個別閉輪郭の検査結果の返戻
	return !HasAnyErrors();
}

#endif
