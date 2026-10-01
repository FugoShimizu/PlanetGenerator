import builtins
import time
state=getattr(builtins,"_planet_ground_probe",None)
if not state:
    raise RuntimeError("No ground probe")
RESULT=state["result"] if state["status"]!="running" else {"status":"running","phase":state["phase"],
    "seconds":time.monotonic()-state["start"],"ground_message":state["ground"].status}
