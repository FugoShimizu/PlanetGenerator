import builtins
import time
state=getattr(builtins,"_planet_surface_probe",None)
if not state:
    raise RuntimeError("No surface render probe")
RESULT=state["result"] if state["status"]!="running" else {"status":"running","phase":state["phase"],"seconds":time.monotonic()-state["start"],"message":state["terrain"].generation_message}
