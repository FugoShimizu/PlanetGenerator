#include "PlanetDrainage.h"
#include "PlanetSphericalSpline.h"
#include "PlanetSphericalMesh.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 同じ球面二次場から，接続を再検査した海域・排水・河口・縦断を準備する
	 * @param Ocean 同じ元網の節点で海域接続を準備した地質場
	 * @param Surface 元網の標高を補間した，未調整の球面二次場
	 * @param Stop 中断要求
	 * @return 全段階で同じ標高を用いた排水，不一致・非連結・中断では空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::CreateForSphericalSurface(
		const OceanConnectivityField &Ocean,
		const SphericalSplineField &Surface,
		const std::stop_token Stop
	) {
		const bool HasSharedMesh = Ocean.Statistics().InputGraphNodes == 40962 && Surface.GetStatistics().Vertices == 40962 &&
		Surface.GetStatistics().Triangles == 81920;
		// 元網以外の場を既存の節点番号として受取らない返戻
		if(Stop.stop_requested() || !HasSharedMesh) return {};
		const std::optional<SphereMesh> Mesh = BuildUniformSphere(6);
		// 元網を準備出来ない場合の返戻
		if(!Mesh) return {};
		// 従来の元網も同じ公開経路へ渡す返戻
		return CreateForSphericalSurfaceOnMesh(Ocean, Surface, Mesh->Directions, Mesh->Triangles, Stop);
	}

	/**
	 * 適応した共有網から，実際の海域接続を検査して排水と縦断を準備する
	 * @param Ocean 同じ網で海域を接続した元地形
	 * @param Surface 元標高を補間する球面場
	 * @param Directions 元場と海域判定が共有する方向
	 * @param Triangles 同じ球面場の閉じた隣接関係
	 * @param Stop 中断要求
	 * @return 同じ地形から準備した排水，不一致・非連結・中断では空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::CreateForSphericalSurfaceOnMesh(
		const OceanConnectivityField &Ocean,
		const SphericalSplineField &Surface,
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::stop_token Stop
	) {
		const bool HasSharedMesh =
		Ocean.Statistics().InputGraphNodes == Directions.size() && Surface.GetStatistics().Vertices == Directions.size() &&
		Surface.GetStatistics().Triangles == Triangles.size() && Directions.size() > 40961 &&
		IsClosedSphericalSamplingMesh(Directions, Triangles, Stop);
		// 別の網や開いた辺を海域接続済として受取らない返戻
		if(Stop.stop_requested() || !HasSharedMesh) return {};
		DrainageSettings Settings;
		Settings.RadiusMeters = Ocean.Base().Settings().RadiusMeters;
		Settings.SeaLevelMeters = Ocean.Base().Settings().SeaLevelMeters;
		Settings.PreferBreaching = true;
		Settings.MinimumSlope = 1E-7 * std::min(1.0, 1E6 / Settings.RadiusMeters);
		std::vector<DrainageSite> Sites(Directions.size());
		std::vector<std::array<std::uint32_t, 2>> Edges;
		Edges.reserve(Triangles.size() * 3);
		for(std::size_t Index = 0; Index < Sites.size(); ++Index) {
			// 中断済みの準備の返戻
			if(!(Index & 255) && Stop.stop_requested()) return {};
			const UnitDirection Direction = Directions[Index];
			const double Height = Surface.Evaluate(Direction).ElevationMeters, Original = Ocean.EvaluateElevation(Direction);
			// 別の元地形を識別値だけ同じにして渡さない返戻
			if(!std::isfinite(Height) || !std::isfinite(Original) || std::abs(Height - Original) > 1E-7) return {};
			Sites[Index].Direction = Direction.Vector();
			Sites[Index].ElevationMeters = Height;
			Sites[Index].Ocean = Height < Settings.SeaLevelMeters;
		}
		for(const std::array<std::uint32_t, 3> &Triangle : Triangles) {
			const Vec3 A = Sites[Triangle[0]].Direction, B = Sites[Triangle[1]].Direction, C = Sites[Triangle[2]].Direction;
			const double Share =
			2 * std::atan2(std::abs(A.Dot((B - A).Cross(C - A))), 1 + A.Dot(B) + B.Dot(C) + C.Dot(A)) * Settings.RadiusMeters *
			Settings.RadiusMeters / 3;
			for(unsigned Side = 0; Side < 3; ++Side) {
				Sites[Triangle[Side]].AreaSquareMeters += Share;
				const std::uint32_t First = Triangle[Side], Second = Triangle[(Side + 1) % 3];
				Edges.push_back({ std::min(First, Second), std::max(First, Second) });
			}
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		// 元地質の統計で代用せず，実際に補間した節点と辺の海域接続を解法内で検査する
		std::optional<DrainageNetwork> Network = Solve(Sites, Edges, Settings, Stop);
		// 実海域が連結でない場合の返戻
		if(!Network) return {};
		Network->SourceHash = Ocean.DefinitionHash();
		const auto Height = [&Surface](const UnitDirection Direction) -> double {
			// 分類・下流・河口・縦断で共有する同じ標高の返戻
			return Surface.Evaluate(Direction).ElevationMeters;
		};
		Network = Network->WithCoastalOutlets(Height, Stop);
		// 同じ海岸へ接続出来ない場合の返戻
		if(!Network) return {};
		// 海岸と同じ二次場から準備した縦断の返戻
		return Network->WithTerrainProfiles(Height, Stop);
	}
}
