"""探索開始候補の起伏を，公開済みの地形関数で比較する"""
import math
import unreal


def unit(vector: unreal.Vector) -> unreal.Vector:
	"""有限な非零ベクトルを単位化する"""
	return vector / vector.length()


def terrain_probe(planet: unreal.PlanetWorld, direction: unreal.Vector, radius: float, span: float, center: float) -> dict:
	"""接線４方向の標高幅と歩行可能性を返す"""
	first = unit(unreal.MathLibrary.cross_vector_vector(direction, unreal.Vector(0, 0, 1)))
	second = unit(unreal.MathLibrary.cross_vector_vector(direction, first))
	heights = []
	accessible = True
	for axis in (first, first * -1, second, second * -1):
		probe_direction = unit(direction * radius + axis * span)
		height, normal = planet.evaluate_direction(probe_direction)
		heights.append(height)
		if height < 2 or unreal.MathLibrary.dot_vector_vector(probe_direction, normal) < 0.95:
			accessible = False
	return {"reliefMeters": max(max(heights), center) - min(min(heights), center),
		"heightsMeters": heights, "accessible": accessible}


world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
if world is None:
	raise RuntimeError("探索中のゲーム世界が有りません")
mode = unreal.GameplayStatics.get_game_mode(world)
if not isinstance(mode, unreal.PlanetGameMode) or mode.explorer is None:
	raise RuntimeError("探索の開始が完了していません")
planet = mode.planet
previous_utc = planet.simulation_utc
clock_reset = bool(ARGS.get("resetToInitialUtc", False))
if clock_reset and not planet.set_simulation_utc(planet.initial_utc):
	raise RuntimeError("Could not reset the PIE clock to InitialUtc")
sun_local = unreal.MathLibrary.transform_direction(
	unreal.MathLibrary.invert_transform(planet.get_actor_transform()), planet.sun_direction_world
)
radius = planet.radius_meters
player = unreal.MathLibrary.inverse_transform_location(
	planet.get_actor_transform(), mode.explorer.get_actor_location()
)
player_direction = unit(player)
near_span = min(100, radius * 0.01)
far_span = min(500, radius * 0.05)
mid_span = min(1000, radius * 0.02)
horizon_span = min(5000, radius * 0.1)
candidate_count = int(ARGS.get("candidateCount", 512))
if candidate_count < 16 or candidate_count > 8192:
	raise ValueError("candidateCount must be between 16 and 8192")
candidates = []
for index in range(candidate_count):
	z = -1 + 2 * (index + 0.5) / candidate_count
	angle = index * 2.399963229728653
	width = math.sqrt(1 - z * z)
	direction = unreal.Vector(width * math.cos(angle), width * math.sin(angle), z)
	elevation, normal = planet.evaluate_direction(direction)
	if elevation < 2 or unreal.MathLibrary.dot_vector_vector(direction, normal) < 0.95:
		continue
	coverage, factors = planet.evaluate_seasonal_surface(direction, planet.solar_year_phase)
	base = 1 + coverage.r + 0.5 * factors.a - coverage.a - factors.r - 0.2 * factors.b
	near = terrain_probe(planet, direction, radius, near_span, elevation)
	far = terrain_probe(planet, direction, radius, far_span, elevation)
	mid = terrain_probe(planet, direction, radius, mid_span, elevation)
	horizon = terrain_probe(planet, direction, radius, horizon_span, elevation)
	if not near["accessible"]:
		continue
	near_relief = near["reliefMeters"]
	near_scenic = max(0, min(1, (near_relief - 1.5) / 3.5)) * max(0, min(1, (12 - near_relief) / 4))
	far_relief = far["reliefMeters"]
	far_scenic = max(0, min(1, (far_relief - 8) / 18)) * max(0, min(1, (55 - far_relief) / 15))
	mid_length = math.sqrt(radius * radius + mid_span * mid_span)
	mid_sagitta = radius * mid_span * mid_span / (mid_length * (mid_length + radius))
	mid_skyline = max(0, max(mid["heightsMeters"]) * radius / mid_length - elevation - mid_sagitta - 1.65)
	horizon_rise = max(horizon["heightsMeters"]) - elevation
	sagitta = radius * (1 - radius / math.sqrt(radius * radius + horizon_span * horizon_span))
	visible_rise = max(0, max(horizon["heightsMeters"]) * radius /
		math.sqrt(radius * radius + horizon_span * horizon_span) - elevation - sagitta - 1.65)
	candidates.append({
		"index": index, "elevationMeters": elevation, "baseScore": base,
		"sunAltitudeDegrees": math.degrees(math.asin(max(-1, min(1,
			unreal.MathLibrary.dot_vector_vector(direction, sun_local))))),
		"coverage": [coverage.r, coverage.g, coverage.b, coverage.a],
		"factors": [factors.r, factors.g, factors.b, factors.a],
		"nearReliefMeters": near_relief, "farReliefMeters": far_relief, "midSkylineMeters": mid_skyline,
		"nearAccessible": near["accessible"], "farAccessible": far["accessible"],
		"horizonReliefMeters": horizon["reliefMeters"], "horizonAccessible": horizon["accessible"],
		"horizonRiseMeters": horizon_rise, "horizonSkylineMeters": visible_rise,
		"horizonFacingIndex": horizon["heightsMeters"].index(max(horizon["heightsMeters"])),
		"horizonFallMeters": elevation - min(horizon["heightsMeters"]),
		"currentScore": base + 0.12 * near_scenic,
		"farScore": base + 0.12 * near_scenic + 0.25 * far_scenic,
		"visibilityScore": base + 0.12 * near_scenic + 0.5 * max(0, min(1, factors.a)) *
			max(0, min(1, visible_rise / 150)),
		"playerDirectionDot": unreal.MathLibrary.dot_vector_vector(direction, player_direction)
	})

if clock_reset and not planet.set_simulation_utc(previous_utc):
	raise RuntimeError("Could not restore the PIE clock after the survey")

RESULT = {
	"radiusMeters": radius, "candidateCount": candidate_count, "landCandidates": len(candidates),
	"clockResetToInitialUtc": clock_reset,
	"nearSpanMeters": near_span, "farSpanMeters": far_span, "midSpanMeters": mid_span,
	"horizonSpanMeters": horizon_span,
	"closestToPlayer": max(candidates, key=lambda value: value["playerDirectionDot"]),
	"topByCurrentScore": sorted(candidates, key=lambda value: value["currentScore"], reverse=True)[:12],
	"topByFarScore": sorted(candidates, key=lambda value: value["farScore"], reverse=True)[:12],
	"topByVisibilityScore": sorted(candidates, key=lambda value: value["visibilityScore"], reverse=True)[:12],
	"topDaylightByVisibilityScore": sorted(
		(value for value in candidates if value["sunAltitudeDegrees"] >= 10),
		key=lambda value: value["visibilityScore"], reverse=True
	)[:12],
	"topDaylightForestByMidSkyline": sorted(
		(value for value in candidates if value["sunAltitudeDegrees"] >= 10 and value["factors"][3] > 0.65
			and value["nearAccessible"]),
		key=lambda value: value["midSkylineMeters"], reverse=True
	)[:24],
	"topDaylightForestBySkyline": sorted(
		(value for value in candidates if value["sunAltitudeDegrees"] >= 10 and value["factors"][3] > 0.65
			and value["nearAccessible"]),
		key=lambda value: value["horizonSkylineMeters"], reverse=True
	)[:24],
	"topDaylightOpenBySkyline": sorted(
		(value for value in candidates if value["sunAltitudeDegrees"] >= 10 and value["factors"][3] < 0.45
			and value["nearAccessible"]),
		key=lambda value: value["horizonSkylineMeters"], reverse=True
	)[:24],
	"topForestByHorizon": sorted(
		(value for value in candidates if value["factors"][3] > 0.65),
		key=lambda value: value["horizonReliefMeters"], reverse=True
	)[:12]
}
