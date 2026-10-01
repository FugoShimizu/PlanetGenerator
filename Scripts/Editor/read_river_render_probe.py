"""河川水面の描画検査を待機せずに読取る"""
import builtins
import time

state = getattr(builtins, "_planet_river_render_probe", None)
if state is None:
	raise RuntimeError("No river water render probe")
if state["status"] == "running":
	actor = state["actor"]
	RESULT = {
		"status": "running", "seconds": time.monotonic() - state["start"],
		"busy": actor.generation_busy, "message": actor.generation_message
	}
else:
	RESULT = state["result"]
