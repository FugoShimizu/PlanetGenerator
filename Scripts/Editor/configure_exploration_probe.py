"""探索の受け入れ検査に使う初期設定を一時変更し，終了後に復元する"""
import builtins
import unreal

args = globals().get("ARGS", {})
key = "_planet_exploration_settings_probe"
editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
if editor.get_game_world() is not None:
	raise RuntimeError("初期設定の変更前に PIE を停止して下さい")
world = editor.get_editor_world()
planets = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.PlanetWorld)
if len(planets) != 1:
	raise RuntimeError("探索用の惑星が一つのレベルを開いて下さい")
planet = planets[0]
state = getattr(builtins, key, None)
if args.get("operation", "restore") == "set":
	if state is None:
		if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
			raise RuntimeError("未保存のレベル変更が有る為，検査設定へ変更しません")
		state = {"planet": planet, "radiusMeters": planet.radius_meters, "seed": planet.seed}
		setattr(builtins, key, state)
	if state["planet"] != planet:
		raise RuntimeError("開始時と異なる惑星は変更しません")
	planet.set_editor_property("radius_meters", float(args["radiusMeters"]))
	if "seed" in args:
		planet.set_editor_property("seed", int(args["seed"]))
else:
	if state is None or state["planet"] != planet:
		raise RuntimeError("復元する設定が有りません")
	planet.set_editor_property("radius_meters", state["radiusMeters"])
	planet.set_editor_property("seed", state["seed"])
	delattr(builtins, key)
RESULT = {"radiusMeters": planet.radius_meters, "seed": planet.seed, "saved": False}
