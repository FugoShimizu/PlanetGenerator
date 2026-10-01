"""Read the current asynchronous probe without waiting on the editor thread."""
import builtins
import time
state=getattr(builtins,"_planet_stream_probe",None)
if not state:
    raise RuntimeError("No streaming probe has been started")
if state["status"]=="running":
    actor=state["actor"]
    RESULT={"status":"running","phase":state["phase"],"elapsed_seconds":time.monotonic()-state["start"],
            "requested_revision":actor.requested_revision,"published_revision":actor.published_revision,
            "busy":actor.generation_busy,"message":actor.generation_message}
else:
    RESULT=state["result"]
