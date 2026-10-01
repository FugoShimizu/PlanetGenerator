"""水面の並行更新と定義変更の検査状態を読む"""
import builtins
import time

state = getattr(builtins, "_planet_water_lifecycle_probe", None)
if state is None:
	raise RuntimeError("水面更新検査を開始して下さい")
RESULT = {
	"status": state["status"], "phase": state["phase"], "seconds": time.monotonic() - state["start"],
	"samples": state["samples"], "parallel_frames": state["parallel_frames"]
} if state["status"] == "running" else {key: value for key, value in state.items() if key != "handle"}
