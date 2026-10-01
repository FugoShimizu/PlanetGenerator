"""Close only after checking that no user asset or level changes would be lost."""
import unreal

if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
    raise RuntimeError("Stop PIE before closing the editor")
dirty_maps = list(unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages())
dirty_assets = list(unreal.EditorLoadingAndSavingUtils.get_dirty_content_packages())
if dirty_maps or dirty_assets:
    raise RuntimeError("Unsaved changes: " + ", ".join(p.get_name() for p in dirty_maps + dirty_assets))
RESULT = {"quit_requested":True}
unreal.SystemLibrary.quit_editor()
