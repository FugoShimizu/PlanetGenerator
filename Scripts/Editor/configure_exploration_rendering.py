"""昼の照明を保持出来る露出範囲を，プロジェクトの設定値へ揃える"""
import unreal

editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = editor.get_game_world() or editor.get_editor_world()
previous = unreal.SystemLibrary.get_console_variable_float_value("r.EyeAdaptation.CachedLightingPreExposure")
unreal.SystemLibrary.execute_console_command(world, "r.EyeAdaptation.CachedLightingPreExposure 8")
current = unreal.SystemLibrary.get_console_variable_float_value("r.EyeAdaptation.CachedLightingPreExposure")
if current != 8:
	raise RuntimeError("照明キャッシュの露出範囲を変更出来ませんでした")
RESULT = {"previousEv": previous, "currentEv": current, "safeExposureEvRange": [-4, 16], "warningDisabled": False}
