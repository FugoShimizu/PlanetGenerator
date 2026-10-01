"""通常の探索 Actor が公開した流量と，照会時の再準備の有無を調べる"""
import math
import unreal

WORLD = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
if WORLD is None:
	raise RuntimeError("探索を開始して下さい")
MODE = unreal.GameplayStatics.get_game_mode(WORLD)
if not isinstance(MODE, unreal.PlanetGameMode) or MODE.explorer is None:
	raise RuntimeError("通常の探索準備の完了を待って下さい")
PLANET = MODE.planet
if not PLANET.runoff_ready:
	raise RuntimeError("流量が未準備です：" + PLANET.runoff_message)
BEFORE = (PLANET.runoff_preparation_count, PLANET.climate_preparation_count, PLANET.published_revision)
SAMPLES = []
for index in range(0, PLANET.river_reach_count, max(1, PLANET.river_reach_count // 32)):
	VALUES = []
	for phase in (0, 0.25, 0.5, 0.75, 1):
		VALUE = PLANET.evaluate_river_discharge(index, phase)
		if VALUE is None or len(VALUE) != 2 or not all(math.isfinite(item) and item >= 0 for item in VALUE):
			raise RuntimeError("有効な河道の流量が不正です：" + str((index, phase, VALUE)))
		VALUES.append(list(VALUE))
	if VALUES[0] != VALUES[-1] or any(item[1] != VALUES[0][1] for item in VALUES):
		raise RuntimeError("年境界又は年平均流量が位相で変化しました")
	SAMPLES.append({"reach": index, "periods": VALUES})
for index, phase in ((-1, 0), (PLANET.river_reach_count, 0), (0, float("inf"))):
	if PLANET.evaluate_river_discharge(index, phase) is not None:
		raise RuntimeError("不正な河道又は位相が拒否されませんでした")
AFTER = (PLANET.runoff_preparation_count, PLANET.climate_preparation_count, PLANET.published_revision)
if BEFORE != AFTER:
	raise RuntimeError("流量の照会で地形又は気候を再準備しました")
RESULT = {
	"radiusMeters": PLANET.radius_meters, "seed": PLANET.seed, "reachCount": PLANET.river_reach_count,
	"runoffPreparations": PLANET.runoff_preparation_count, "climatePreparations": PLANET.climate_preparation_count,
	"runoffMilliseconds": PLANET.last_runoff_preparation_milliseconds,
	"combinedClimateMilliseconds": PLANET.last_climate_preparation_milliseconds,
	"annualOceanCubicMeters": PLANET.annual_ocean_runoff_cubic_meters,
	"relativeWaterBudgetResidual": PLANET.runoff_water_budget_relative_residual,
	"samePreparationAndGeometryRevision": BEFORE == AFTER, "samples": SAMPLES,
	"waterRenderingVerified": False, "erosionQualityVerified": False
}
