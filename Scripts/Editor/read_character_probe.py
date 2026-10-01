import builtins
import time
state=getattr(builtins,"_planet_character_probe",None)
if not state:
    raise RuntimeError("No character probe")
RESULT=state["result"] if state["status"]!="running" else {"status":"running","phase":state["phase"],"seconds":time.monotonic()-state["start"],"samples":state["samples"],"travel_m":state.get("travel_m",0),
    "ocean_publications":state.get("ocean_publication_checks",0),
    "reused_ocean_publications":sum("Ocean topology reused=1" in row.get("details","") for row in state.get("render_worker_samples",[])),
    "retained_ocean_publications":sum("previous ocean partition=1" in row.get("details","") for row in state.get("render_worker_samples",[])),
    "last_ocean_build_ms":state["render_worker_samples"][-1].get("ocean_build_ms") if state.get("render_worker_samples") else None}
