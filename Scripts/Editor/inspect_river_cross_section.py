"""公開された河床断面と独立な差分の収束を比較する"""

import math
import unreal

args: dict[str, object] = globals().get("ARGS", {})
editor: unreal.UnrealEditorSubsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
found: list[unreal.PlanetTerrain] = []

for actor in unreal.GameplayStatics.get_all_actors_of_class(editor.get_editor_world(), unreal.PlanetTerrain):
	if actor.get_actor_label() == "Planet_SurfaceRenderProbe":
		found.append(actor)

if len(found) != 1 or found[0].published_revision <= 0:
	raise RuntimeError("A published surface render probe is required")

terrain: unreal.PlanetTerrain = found[0]
index: int = int(args.get("river_reach_index", 0))
fraction: float = float(args.get("river_fraction", 0.5))
profile: tuple[unreal.Vector, float, float, unreal.Vector]|None = terrain.evaluate_river_profile(index, fraction)

if profile is None:
	raise RuntimeError("Invalid river profile")

position, water, bed, tangent = profile
up: unreal.Vector = position / position.length()
across: unreal.Vector = unreal.MathLibrary.cross_vector_vector(up, tangent)
across = across / across.length()
distances: list[float] = [0.0]

for sign in -1, 1:
	for i in range(37):
		distances.append(sign * (0.25 * 1.3 ** i))

distances = sorted(set(distances))
samples: list[dict[str, object]] = []

for distance in distances:
	angle: float = distance / terrain.radius_meters
	direction: unreal.Vector = up * math.cos(angle) + across * math.sin(angle)
	h, normal = terrain.evaluate_direction(direction)
	radial: float = unreal.MathLibrary.dot_vector_vector(direction, normal)
	lateral: float = unreal.MathLibrary.dot_vector_vector(across * math.cos(angle) - up * math.sin(angle), normal)
	slope: float = -lateral / radial
	if not math.isfinite(h) or not math.isfinite(slope) or radial <= 0:
		raise RuntimeError("Invalid cross-section normal")
	def difference(step: float) -> float:
		"""接線方向の中央差分を半径の伸長率で換算する"""
		heights: list[float] = []
		for side in -1, 1:
			a: float = (distance + side * step) / terrain.radius_meters
			heights.append(terrain.evaluate_direction(up * math.cos(a) + across * math.sin(a))[0])
		# 描画法線と同じ接線長当たりの標高微分の返戻
		return (heights[1] - heights[0]) / (2 * step) * terrain.radius_meters / (terrain.radius_meters + h)
	# 元の三つの刻みを最初に保ち，打切り誤差が残る地点だけを半分にする
	coarse_step: float = 0.1
	coarse: float = difference(coarse_step)
	fine: float = difference(coarse_step * 0.5)
	finer: float = difference(coarse_step * 0.25)
	attempts: list[dict[str, object]] = []
	is_converged: bool = False
	angle_error: float = math.inf
	for attempt in range(12):
		first: float = (4 * fine - coarse) / 3
		second: float = (4 * finer - fine) / 3
		angle_error = abs(math.atan(second) - math.atan(slope))
		convergence_error: float = abs(math.atan(second) - math.atan(first))
		is_converged = convergence_error < 2E-6
		attempts.append(
			{
				"coarse_step_m": coarse_step,
				"coarse_slope": coarse,
				"fine_slope": fine,
				"finer_slope": finer,
				"extrapolated_slope": second,
				"convergence_angle_error_rad": convergence_error,
				"normal_angle_error_rad": angle_error,
				"converged": is_converged
			}
		)
		if is_converged:
			break
		coarse_step *= 0.5
		coarse, fine = fine, finer
		finer = difference(coarse_step * 0.25)
	if not is_converged or angle_error > 1E-5:
		details: str = f"error={angle_error}, converged={is_converged}, attempts={attempts}"
		raise RuntimeError(f"Cross-section derivative mismatch at {distance}m: " + details)
	samples.append(
		{
			"offset_m": distance,
			"height_m": h,
			"cross_slope": slope,
			"normal_angle_error_rad": angle_error,
			"difference_attempts": attempts
		}
	)

RESULT = {
	"river_reach_index": index,
	"river_fraction": fraction,
	"radius_meters": terrain.radius_meters,
	"water_at_center_m": water,
	"intended_bed_at_center_m": bed,
	"samples": samples,
	"maximum_sampled_cross_slope": max(abs(s["cross_slope"]) for s in samples),
	"maximum_normal_angle_error_rad": max(s["normal_angle_error_rad"] for s in samples),
	"maximum_difference_attempts": max(len(s["difference_attempts"]) for s in samples),
	"convergence_angle_tolerance_rad": 2E-6,
	"normal_angle_tolerance_rad": 1E-5,
	"scope": "Finite analytic cross-section; constant reference line is not a rendered water width or a global slope bound."
}
