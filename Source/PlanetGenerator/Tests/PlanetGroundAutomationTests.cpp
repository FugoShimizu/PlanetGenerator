#include "Misc/AutomationTest.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetOcean.h"
#include "HAL/PlatformTime.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <map>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/** 内部辺の共有数と向きの独立集計 */
	struct EdgeUse {
		unsigned Count = 0;
		int Winding = 0;
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetGroundContracts,"PlanetGenerator.Ground.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 局所座標，細分化及び共有辺の再利用を独立な球面式と実地形で検査する
 * @param Parameters 未使用の実行引数
 * @return 記録した全検査が合格なら真
 */
bool FPlanetGroundContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const double Start = FPlatformTime::Seconds();
	std::uint64_t Checked = 0;
	double MaximumFloatError = 0;
	const PlanetCore::GroundEvaluator Sphere = [](const PlanetCore::UnitDirection) -> PlanetCore::TerrainSample {
		// 標高零の球面の返戻
		return {};
	};
	for(const double Radius : { 1000.0, 1000000.0, 6371000.0, 1E9 }) {
		for(
			const PlanetCore::Vec3 Anchor :
			{
				PlanetCore::Vec3{ 0, 0, 1 },
				PlanetCore::Vec3{ 0, 0, -1 },
				PlanetCore::Vec3{ 1, 0, 0 },
				PlanetCore::Vec3{ -1, 0, 0 },
				PlanetCore::Vec3{ 1, 2, 3 },
				PlanetCore::Vec3{ -3, 2, -1 }
			}
		) {
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(Anchor);
			const std::optional<PlanetCore::Vec3> G = PlanetCore::SurfaceGravity(D.Vector() * Radius);
			if(!TestTrue(TEXT("Constant inward gravity"), G && std::abs(G->Length() - 9.80665) < 1E-12 && G->Dot(D.Vector()) < 0)) {
				// 半径と方向に依らない重力の不成立への返戻
				return false;
			}
			PlanetCore::GroundPatchSettings S;
			S.CellsPerSide = 8;
			S.HalfExtentMeters = 8;
			const std::optional<PlanetCore::GroundPatch> Patch = PlanetCore::BuildGroundPatch(D, Radius, S, Sphere);
			// 基準球面を生成出来ない場合の返戻
			if(!TestTrue(TEXT("Local sphere patch built"), Patch.has_value())) return false;
			const PlanetCore::GroundFrame &F = Patch->Frame;
			// 局所基底が右手直交系でない場合の返戻
			if(!TestTrue(TEXT("Orthonormal right-handed frame"), std::abs(F.East.Cross(F.North).Dot(F.Up) - 1) < 1E-12)) return false;
			for(std::size_t I = 0; I < Patch->Positions.size(); ++I) {
				const PlanetCore::Vec3 P = Patch->Positions[I];
				const PlanetCore::Vec3 FloatP {
					static_cast<double>(static_cast<float>(P.X * 100)) / 100,
					static_cast<double>(static_cast<float>(P.Y * 100)) / 100,
					static_cast<double>(static_cast<float>(P.Z * 100)) / 100
				};
				const double Error = (P - FloatP).Length();
				MaximumFloatError = std::max(MaximumFloatError, Error);
				const PlanetCore::Vec3 World = F.LocalToPlanet(P);
				// 描画精度・球面再構成・法線の検査
				const bool CoordinateFailure = !TestTrue(TEXT("Local float retains sub-millimeter accuracy"), Error < 0.001) ||
				!TestTrue(TEXT("Reconstructed point lies on sphere"), std::abs(World.Length() - Radius) < 2E-6) ||
				!TestTrue(TEXT("Outward finite normal"), Patch->Normals[I].Z > 0);
				// 対応する契約を満たさない場合の返戻
				if(CoordinateFailure) return false;
				++Checked;
			}
			// 大半径でも保つ局所曲率 z = − (x²+y²)/(R+ √ (R² − x² − y²)) を独立式で調べる
			const PlanetCore::Vec3 P = Patch->Positions.front();
			const double XY = P.X * P.X + P.Y * P.Y, Expected = -XY / (Radius + std::sqrt(Radius * Radius - XY));
			// 独立な曲率式と一致しない場合の返戻
			if(!TestTrue(TEXT("Local curvature survives large radius"), std::abs(P.Z - Expected) < 1E-11)) return false;
			std::stop_source Cancel;
			Cancel.request_stop();
			TestFalse(
				TEXT("Cancellation rejected before evaluating"),
				PlanetCore::BuildGroundPatch(D, Radius, S, Sphere, Cancel.get_token()).has_value()
			);
		}
	}
	const PlanetCore::UnitDirection North = *PlanetCore::UnitDirection::FromVector({ 0, 0, 1 });
	PlanetCore::GroundPatchSettings Tight;
	Tight.MaximumProbeErrorMeters = 1E-9;
	TestFalse(
		TEXT("Inaccurate collision candidate rejected"),
		PlanetCore::BuildGroundPatch(North, 1000, Tight, Sphere).has_value()
	);
	const PlanetCore::GroundEvaluator NarrowRidge = [](const PlanetCore::UnitDirection Q) -> PlanetCore::TerrainSample {
		const PlanetCore::Vec3 D = Q.Vector();
		const double X = 1000 * std::atan2(D.X, D.Z), H = 0.8 * std::exp(-X * X / 0.25);
		const PlanetCore::Vec3 Gradient = PlanetCore::Vec3{ D.Z, 0, -D.X } * (1000 / (D.X * D.X + D.Z * D.Z));
		// 狭い隆起と解析勾配の返戻
		return PlanetCore::TerrainSample{ H, Gradient * (-2 * X / 0.25 * H) };
	};
	PlanetCore::GroundPatchSettings Adaptive;
	Adaptive.CellsPerSide = 16;
	Adaptive.HalfExtentMeters = 8;
	const std::optional<PlanetCore::GroundPatch> Ridge = PlanetCore::BuildGroundPatch(North, 1000, Adaptive, NarrowRidge);
	// 狭い起伏への局所細分化の検査
	const bool RefinementFailure = !TestTrue(
		TEXT("Narrow continuous ridge receives local refinement within the original error limit"),
		Ridge && Ridge->Positions.size() > 17 * 17 && Ridge->Positions.size() < 129 * 129
	);
	// 対応する契約を満たさない場合の返戻
	if(RefinementFailure) return false;
	// 評価点を増やしても，共有辺の再利用が形状を変えず，実際に省いた評価数と一致する事を調べる
	// 全評価数には共有出来ない内部点も含む為，削減率２５％という旧条件は適用しない
	PlanetCore::GroundPatchSettings ReferenceSettings = Adaptive;
	ReferenceSettings.ReuseSharedEdgeProbes = false;
	const std::optional<PlanetCore::GroundPatch> Reference =
	PlanetCore::BuildGroundPatch(North, 1000, ReferenceSettings, NarrowRidge);
	// 共有辺の再利用前後の同一性と評価数の検査
	const bool ReuseFailure = !TestTrue(
		TEXT("Shared edge reuse preserves the complete patch"),
		Reference && Reference->Triangles == Ridge->Triangles && Reference->Positions.size() == Ridge->Positions.size() &&
		Reference->MaximumProbeErrorMeters == Ridge->MaximumProbeErrorMeters &&
		Reference->Evaluations == Ridge->Evaluations + Ridge->SharedEdgeProbeHits && Ridge->SharedEdgeProbeHits > 0
	);
	// 対応する契約を満たさない場合の返戻
	if(ReuseFailure) return false;
	const auto Bits = [](const double A, const double B) -> bool {
		// ビット列の厳密な比較結果の返戻
		return std::bit_cast<std::uint64_t>(A) == std::bit_cast<std::uint64_t>(B);
	};
	for(std::size_t I = 0; I < Ridge->Positions.size(); ++I) {
		const PlanetCore::Vec3 A = Ridge->Positions[I], B = Reference->Positions[I], N = Ridge->Normals[I], M = Reference->Normals[I];
		// 位置と法線の厳密な再現性の検査
		const bool BitIdentityFailure = !TestTrue(
			TEXT("Shared edge reuse is bit identical"),
			Bits(A.X, B.X) && Bits(A.Y, B.Y) && Bits(A.Z, B.Z) && Bits(N.X, M.X) && Bits(N.Y, M.Y) && Bits(N.Z, M.Z)
		);
		// 対応する契約を満たさない場合の返戻
		if(BitIdentityFailure) return false;
	}
	AddInfo(
		FString::Printf(
			TEXT("Ground shared edges: evaluations=%llu reference=%llu hits=%llu"),
			static_cast<unsigned long long>(Ridge->Evaluations),
			static_cast<unsigned long long>(Reference->Evaluations),
			static_cast<unsigned long long>(Ridge->SharedEdgeProbeHits)
		)
	);
	std::map<std::pair<std::uint32_t, std::uint32_t>, EdgeUse> Uses;
	for(const std::array<std::uint32_t, 3> T : Ridge->Triangles) for(unsigned E = 0; E < 3; ++E) {
		const std::uint32_t A = T[E], B = T[(E + 1) % 3];
		EdgeUse &Use = Uses[{ std::min(A, B), std::max(A, B) }];
		++Use.Count;
		Use.Winding += A < B ? 1 : -1;
	}
	const auto Project = [&](const std::uint32_t I) -> PlanetCore::Vec3 {
		const PlanetCore::Vec3 P = Ridge->Frame.LocalToPlanet(Ridge->Positions[I]);
		const double Z = P.Dot(Ridge->Frame.Up);
		// 頂点の接平面座標の返戻
		return PlanetCore::Vec3{ 1000 * P.Dot(Ridge->Frame.East) / Z, 1000 * P.Dot(Ridge->Frame.North) / Z, 0 };
	};
	for(const std::pair<const std::pair<std::uint32_t, std::uint32_t>, EdgeUse> &Item : Uses) {
		const std::pair<std::uint32_t, std::uint32_t> &Key = Item.first;
		const EdgeUse &Use = Item.second;
		if(Use.Count == 2) {
			// 内部辺の向きが打消し合わない場合の返戻
			if(!TestTrue(TEXT("Adaptive interior edges have opposite winding"), Use.Winding == 0)) return false;
		} else {
			const PlanetCore::Vec3 A = Project(Key.first), B = Project(Key.second);
			const bool Boundary = std::abs(std::abs(A.X) - 8) < 1E-9 && std::abs(A.X - B.X) < 1E-9 ||
			std::abs(std::abs(A.Y) - 8) < 1E-9 && std::abs(A.Y - B.Y) < 1E-9;
			// 外周以外へ不対辺が残る場合の返戻
			if(!TestTrue(TEXT("No unmatched edge or T-junction inside adaptive ground"), Use.Count == 1 && Boundary)) return false;
		}
	}
	// 円板の位相を保持する検査
	const bool TopologyFailure =
	!TestTrue(TEXT("Adaptive ground retains disk topology"), Ridge->Positions.size() + Ridge->Triangles.size() == Uses.size() + 1);
	// 対応する契約を満たさない場合の返戻
	if(TopologyFailure) return false;
	AddInfo(
		FString::Printf(
			TEXT("Adaptive narrow ridge vertices=%llu triangles=%llu evaluations=%llu max_probe_error=%.9g m"),
			static_cast<unsigned long long>(Ridge->Positions.size()),
			static_cast<unsigned long long>(Ridge->Triangles.size()),
			static_cast<unsigned long long>(Ridge->Evaluations),
			Ridge->MaximumProbeErrorMeters
		)
	);
	TestFalse(TEXT("No gravity at invalid center"), PlanetCore::SurfaceGravity({ 0, 0, 0 }).has_value());
	PlanetCore::GeologySettings Definition;
	Definition.RadiusMeters = 1E6;
	Definition.Seed = 42;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Definition);
	// 地質を準備出来ない場合の返戻
	if(!TestTrue(TEXT("Geology prepared"), Geology.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 海域を準備出来ない場合の返戻
	if(!TestTrue(TEXT("Ocean field prepared"), Ocean.has_value())) return false;
	const PlanetCore::GroundEvaluator Terrain = [&](const PlanetCore::UnitDirection Q) -> PlanetCore::TerrainSample {
		// 同じ実地形の返戻
		return Ocean->Evaluate(Q);
	};
	for(const PlanetCore::Vec3 Anchor : { PlanetCore::Vec3{ 1, 2, 3 }, PlanetCore::Vec3{ -3, -1, 2 } }) {
		const PlanetCore::UnitDirection Q = *PlanetCore::UnitDirection::FromVector(Anchor);
		PlanetCore::GroundPatchSettings S;
		S.CellsPerSide = 16;
		S.HalfExtentMeters = 8;
		const std::optional<PlanetCore::GroundPatch> A = PlanetCore::BuildGroundPatch(Q, Definition.RadiusMeters, S, Terrain);
		const std::optional<PlanetCore::GroundPatch> B = PlanetCore::BuildGroundPatch(Q, Definition.RadiusMeters, S, Terrain);
		// 実地形の二つの候補を準備出来ない場合の返戻
		if(!TestTrue(TEXT("Actual terrain patches accepted"), A && B)) return false;
		for(std::size_t I = 0; I < A->Positions.size(); ++I) {
			// 同じ定義から異なる位置が出た場合の返戻
			if(!TestTrue(TEXT("Repeated local terrain mesh exact"), (A->Positions[I] - B->Positions[I]).Length() == 0)) return false;
			++Checked;
		}
	}
	AddInfo(
		FString::Printf(
			TEXT("Ground contracts: %llu vertices, 24 radius/orientation cases, 2 actual terrain cases; ")
			TEXT("max float error %.12g m; elapsed %.3f s. Collision cooking and walking require separate tests."),
			static_cast<unsigned long long>(Checked),
			MaximumFloatError,
			FPlatformTime::Seconds() - Start
		)
	);
	// 記録した全検査の結果の返戻
	return !HasAnyErrors();
}

#endif
