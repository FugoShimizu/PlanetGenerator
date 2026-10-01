"""河川水面検査のコールバックを解放する，共有アクターは地表検査の後始末で破棄する"""
import builtins
import unreal

state = getattr(builtins, "_planet_river_render_probe", None)
RESULT = {"cleaned": state is not None, "shared_actors_removed": False}
if state is not None:
	if state.get("handle") is not None:
		unreal.unregister_slate_post_tick_callback(state["handle"])
		state["handle"] = None
	RESULT["previous_status"] = state["status"]
	state.pop("actor", None)
	del builtins._planet_river_render_probe
