"""Display and probe the geological base with initial ocean corridors enabled."""
from pathlib import Path
import runpy

RESULT = runpy.run_path(str(Path(__file__).with_name("preview_planet_geology.py")),
                        init_globals={"ARGS": {"connect_oceans": True, "view_offset_radii": (-2.1, 2.1, 0.9)}})["RESULT"]
