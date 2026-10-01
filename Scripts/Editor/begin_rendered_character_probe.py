import runpy
import unreal
from pathlib import Path
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
if world is None:
    raise RuntimeError("Start PIE before rendered traversal")
# Remove only copies of our completed editor probes, which would otherwise
# place a second collision patch at the exact same test location.
for cls,label in ((unreal.PlanetTerrain,"Planet_SurfaceRenderProbe"),(unreal.PlanetGroundPatch,"Planet_SurfaceRenderGround")):
    for actor in unreal.GameplayStatics.get_all_actors_of_class(world,cls):
        if actor.get_actor_label()==label:
            actor.destroy_actor()
RESULT=runpy.run_path(str(Path(unreal.Paths.project_dir())/"Scripts/Editor/begin_character_probe.py"),init_globals={"ARGS":dict(globals().get("ARGS",{}),rendered=True)})["RESULT"]
