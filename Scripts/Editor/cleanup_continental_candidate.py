"""此の操作が所有する比較表示だけを破棄し，元カメラを読戻して確認する．"""

from __future__ import annotations
import builtins
import math
from typing import cast
import unreal

# 所有資源と開始前の画面状態の取得
RESULT: dict[str, object]
STATE_KEY = "_planet_continental_candidate_preview_state"
state: dict[str, object]|None = cast(dict[str, object] | None, builtins.__dict__.get(STATE_KEY))

if state is None:
	RESULT = { "cleaned": True, "wasPresent": False }
else:
	actors: unreal.EditorActorSubsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
	editor: unreal.UnrealEditorSubsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
	levels: unreal.LevelEditorSubsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
	if levels.is_in_play_in_editor():
		raise RuntimeError("Stop PIE before restoring the editor comparison state.")
	actor: unreal.Actor = cast(unreal.Actor, state["actor"])
	camera: tuple[unreal.Vector, unreal.Rotator] = cast(tuple[unreal.Vector, unreal.Rotator], state["camera"])
	if not actors.destroy_actor(actor):
		raise RuntimeError("The owned transient comparison actor could not be destroyed.")
	unreal.EditorLevelLibrary.set_level_viewport_camera_info(camera[0], camera[1])
	actual: tuple[unreal.Vector, unreal.Rotator] = unreal.EditorLevelLibrary.get_level_viewport_camera_info()
	delta: unreal.Vector = actual[0] - camera[0]
	position_error: float = math.sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z)
	rotation_error: float = max(
		abs(actual[1].pitch - camera[1].pitch),
		abs(actual[1].yaw - camera[1].yaw),
		abs(actual[1].roll - camera[1].roll)
	)
	current_count: int = len(unreal.GameplayStatics.get_all_actors_of_class(editor.get_editor_world(), unreal.Actor))
	initial_count: int = int(cast(int, state["initialActorCount"]))
	if position_error > 1E-6 or rotation_error > 1E-6 or current_count != initial_count:
		raise RuntimeError("The original camera or actor count was not restored.")
	# 所有参照を解放し，画像又は生成地形を残さない
	builtins.__dict__.pop(STATE_KEY)
	state.clear()
	levels.editor_invalidate_viewports()
	RESULT = {
		"cleaned": True,
		"wasPresent": True,
		"positionErrorCentimeters": position_error,
		"rotationErrorDegrees": rotation_error,
		"actorCount": current_count
	}
