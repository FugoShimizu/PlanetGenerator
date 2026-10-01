"""実際の C++ 探索モード・身体・表示・足場の状態を読む"""
import unreal

editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = editor.get_game_world()
if world is None:
	raise RuntimeError("探索レベルで PIE を開始して下さい")
mode = unreal.GameplayStatics.get_game_mode(world)
if not isinstance(mode, unreal.PlanetGameMode):
	raise RuntimeError("C++ 探索モードが動いていません")
planet = mode.planet
body = mode.explorer
controller = unreal.GameplayStatics.get_player_controller(world, 0)
RESULT = {
	"stage": str(mode.exploration_stage), "message": mode.preparation_message,
	"hasBody": body is not None,
	"possessed": body is not None and controller is not None and controller.get_controlled_pawn() == body
}
if planet is not None:
	RESULT["planet"] = {
		"seed": planet.seed, "radiusMeters": planet.radius_meters, "busy": planet.generation_busy,
		"publishedRevision": planet.published_revision, "requestedRevision": planet.requested_revision,
		"climateReady": planet.climate_ready, "climatePreparations": planet.climate_preparation_count,
		"workerMilliseconds": planet.last_worker_milliseconds, "nearRadiusMeters": planet.visible_surface_radius_meters,
		"terrainMessage": planet.generation_message
	}
if body is not None:
	position = body.get_actor_location()
	velocity = body.get_velocity()
	RESULT["body"] = {
		"position": [position.x, position.y, position.z], "speedMetersPerSecond": velocity.length() / 100,
		"viewMode": str(body.view_mode), "coverageMissFrames": body.coverage_miss_frames,
		"groundMarginMeters": body.ground_margin_meters, "groundRefreshRequests": body.ground_refresh_requests,
		"traversalSeconds": body.traversal_seconds
	}
if mode.ground is not None:
	RESULT["ground"] = {
		"busy": mode.ground.busy, "publishedRevision": mode.ground.published_revision,
		"maximumProbeErrorMeters": mode.ground.maximum_probe_error_meters, "message": mode.ground.status
	}
