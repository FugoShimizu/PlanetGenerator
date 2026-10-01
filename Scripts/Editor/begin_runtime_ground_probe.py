from pathlib import Path
import runpy
RESULT=runpy.run_path(str(Path(__file__).with_name("begin_ground_probe.py")),init_globals={"ARGS":{"game_world":True}})["RESULT"]
