#include "Core/PlanetMinimumLink.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetMinimumLinkReconstruction,
	"PlanetGenerator.SpillConnection.MinimumLinkReconstruction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 桁落ちで端点高度が逆転する末尾を，元標高と既存の位置精度で再構成する
 * @param Parameters 未使用の実行引数
 * @return 元の失敗を再現し，解析下降・端点・拒否条件・中断を確認した場合に真
 */
bool FPlanetMinimumLinkReconstruction::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z;
			// 数学的に相殺する式の丸めを，最適化で省かず検査入力として保持する
			const volatile double Raised = 1000 + X, Difference = Raised - 1000;
			const double Residual = Difference - X, DX = 1E-7 * X, DY = 1E-7 * Y;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 元の意図は高さ１００の非退化極小を持つ解析的な二次関数
			return { 100 + Residual + 0.5E-7 * (X * X + Y * Y), Gradient };
		};
		PlanetCore::TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Radius;
		Critical.MaximumDisplacementMeters = std::min(Critical.MaximumDisplacementMeters, Radius * 0.125);
		const PlanetCore::UnitDirection Origin = *PlanetCore::UnitDirection::FromVector({ 0, 0, 1 });
		const std::optional<PlanetCore::TerrainCriticalPoint> Minimum =
		PlanetCore::RefineTerrainCriticalPoint(Origin, Source, Critical);
		// 検査自体で確認出来ない極小への返戻
		if(!TestTrue(TEXT("同じ元場の極小と曲率を精査"), Minimum && Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum)) return false;
		const PlanetCore::UnitDirection Previous = *PlanetCore::UnitDirection::FromVector({ -0.05 / Radius, 0, 1 });
		std::array<PlanetCore::TerrainDescentSample, 2> Path {
			PlanetCore::TerrainDescentSample{ Previous, Source(Previous).ElevationMeters, 0 },
			PlanetCore::TerrainDescentSample{ Origin, 100, 0.05 }
		};
		bool HasReversal = false;
		for(unsigned Probe = 0; Probe < 128; ++Probe) {
			const double X = -(0.0005 + Probe * 0.00001);
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector({ X / Radius, 0, 1 });
			const double H = Source(D).ElevationMeters;
			if(!(H < Minimum->ElevationMeters)) continue;
			const PlanetCore::Vec3 A = Previous.Vector(), B = D.Vector();
			Path[1] = { D, H, Radius * std::atan2(A.Cross(B).Length(), A.Dot(B)) };
			HasReversal = true;
			break;
		}
		// 元の逆転を再現しない条件を修正成功としない返戻
		if(!TestTrue(TEXT("丸めによる同じ端点逆転を再現"), HasReversal)) return false;
		TestFalse(TEXT("以前の短区間だけでは拒否"), PlanetCore::DescendsToTerrainMinimum(Path.back(), *Minimum, Source));
		const std::array<PlanetCore::TerrainDescentSample, 2> Original = Path;
		const std::optional<PlanetCore::TerrainMinimumLink> Link =
		PlanetCore::PrepareTerrainMinimumLink(Path, *Minimum, Radius, 0.01, Source);
		// 実測出来ない末尾を準備済としない返戻
		if(!TestTrue(TEXT("同じ元場で下降する末尾を準備"), Link.has_value())) return false;
		TestEqual(TEXT("元の前半標本を保持"), Link->RetainedSamples, std::size_t{ 1 });
		TestEqual(TEXT("新しい末尾も元場の実標高"), Link->Anchor.ElevationMeters, Source(Link->Anchor.Direction).ElevationMeters);
		TestTrue(TEXT("公開する両端の標高は下降"), Minimum->ElevationMeters <= Link->Anchor.ElevationMeters);
		TestTrue(TEXT("直前の元標本からも下降"), Link->Anchor.ElevationMeters <= Original.front().ElevationMeters);
		const PlanetCore::Vec3 A = Link->Anchor.Direction.Vector(), B = Minimum->Direction.Vector();
		const double Move = Radius * std::atan2(A.Cross(B).Length(), A.Dot(B));
		TestTrue(TEXT("既存の位置精度を拡げない"), Move <= 0.01);
		TestTrue(TEXT("接続開始点は元辺の途中"), Link->Anchor.DistanceMeters > 0 && Link->Anchor.DistanceMeters < Original.back().DistanceMeters);
		// 実装の積分標本とは異なる１３点の解析微分を独立に確認する
		for(unsigned Probe = 0; Probe < 13; ++Probe) {
			const double Fraction = (Probe + 0.3) / 13;
			const PlanetCore::Vec3 P = A * (1 - Fraction) + B * Fraction, Delta = B - A;
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(P);
			const PlanetCore::Vec3 U = D.Vector();
			TestTrue(TEXT("元場の区間内解析微分は下降"), Source(D).TangentialGradientPerRadian.Dot(Delta - U * U.Dot(Delta)) < 0);
		}
		for(unsigned I = 0; I < Path.size(); ++I) {
			TestEqual(TEXT("入力標本の位置を変更しない"), (Path[I].Direction.Vector() - Original[I].Direction.Vector()).Length(), 0.0);
			TestEqual(TEXT("入力標本の標高を変更しない"), Path[I].ElevationMeters, Original[I].ElevationMeters);
		}
		AddInfo(
			FString::Printf(
				TEXT("Minimum link R=%.0f originalHeightDelta=%.17g anchorH=%.17g minimumH=%.17g move=%.17g calls=%llu"),
				Radius,
				Minimum->ElevationMeters - Original.back().ElevationMeters,
				Link->Anchor.ElevationMeters,
				Minimum->ElevationMeters,
				Move,
				static_cast<unsigned long long>(Calls)
			)
		);
		Calls = 0;
		TestFalse(TEXT("位置精度の無断拡大を拒否"), PlanetCore::PrepareTerrainMinimumLink(Path, *Minimum, Radius, 0.2, Source).has_value());
		TestEqual(TEXT("不正な精度では照会しない"), Calls, UINT64_C(0));
		std::stop_source Stop;
		Stop.request_stop();
		TestFalse(
			TEXT("中断済では末尾を準備しない"),
			PlanetCore::PrepareTerrainMinimumLink(Path, *Minimum, Radius, 0.01, Source, Stop.get_token()).has_value()
		);
		TestEqual(TEXT("中断後の元場照会は零"), Calls, UINT64_C(0));
		const auto Invalid = [](const PlanetCore::UnitDirection) -> PlanetCore::TerrainSample {
			// 欠落した元場を数値修正で補わない返戻
			return { std::numeric_limits<double>::quiet_NaN(), {} };
		};
		TestFalse(TEXT("非有限な元場を受理しない"), PlanetCore::PrepareTerrainMinimumLink(Path, *Minimum, Radius, 0.01, Invalid).has_value());
		PlanetCore::TerrainCriticalPoint Wrong = *Minimum;
		Wrong.ElevationMeters = Original.front().ElevationMeters + 1;
		TestFalse(TEXT("実際に高い端点を下降として採用しない"), PlanetCore::PrepareTerrainMinimumLink(Path, Wrong, Radius, 0.01, Source).has_value());
	}
	// 元標高・解析微分・入力保持・拒否条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
