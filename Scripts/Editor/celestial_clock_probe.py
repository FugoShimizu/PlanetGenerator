"""実際の日時・天体照明・季節材質の接続と，日時変更時の地形不変を確認する"""
import builtins
import hashlib
import math
import struct
import time
import unreal

key = "_planet_celestial_clock_probe"
args = globals().get("ARGS", {})
state = getattr(builtins, key, None)
dates = [(2026, 6, 21, hour) for hour in (0, 6, 12, 18)] + [(2026, 12, 21, 6)]


def mesh_hash(planet: unreal.PlanetWorld) -> str:
	"""公開された表示形状の識別値を返す

	Args:
		planet: 探索中の惑星
	Returns:
		頂点・法線・接続の SHA-256
	"""
	vertices, indices, normals, _, _ = unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(planet.get_visible_mesh(), 0)
	digest = hashlib.sha256()
	for values in (vertices, normals):
		for vector in values:
			digest.update(struct.pack("<ddd", vector.x, vector.y, vector.z))
	for index in indices:
		digest.update(struct.pack("<i", index))
	# 実際の表示形状の識別値の返戻
	return digest.hexdigest()


def set_date(current: dict, index: int) -> None:
	"""検査対象の共通日時を設定する

	Args:
		current: 所有する検査状態
		index: 日時の番号
	Raises:
		RuntimeError: 正規な日時が拒否された場合
	"""
	year, month, day, hour = dates[index]
	utc = current["originalUtc"].copy()
	if not utc.import_text(f"{year:04d}.{month:02d}.{day:02d}-{hour:02d}.00.00"):
		raise RuntimeError("DateTime import failed")
	if not current["planet"].set_simulation_utc(utc):
		raise RuntimeError(current["planet"].clock_message)
	current["nextRead"] = time.monotonic() + 0.3
	# 終了
	return


def finish(current: dict, error: str | None) -> None:
	"""通知を解除し，元の日時と進行設定を復元する

	Args:
		current: 所有する検査状態
		error: 失敗内容，成功時は None
	"""
	if current["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(current["handle"])
	current["handle"] = None
	current["error"] = error
	current["completed"] = True
	planet = current["planet"]
	if not planet.set_simulation_utc(current["originalUtc"]):
		current["error"] = planet.clock_message
	planet.set_editor_property("advance_time", current["originalAdvance"])
	for name in ("planet", "body", "ground", "originalUtc"):
		current.pop(name, None)
	# 終了
	return


def tick(delta_seconds: float) -> None:
	"""描画更新後の実際の照明と材質を比較する

	Args:
		delta_seconds: Slate の更新間隔，待機には単調時計を使う
	"""
	_ = delta_seconds
	current = getattr(builtins, key)
	# 描画更新を待つ終了
	if time.monotonic() < current["nextRead"]:
		return
	try:
		planet = current["planet"]
		if not planet.celestial_clock_active:
			raise RuntimeError(planet.clock_message)
		if planet.published_revision != current["revision"] or mesh_hash(planet) != current["meshHash"]:
			raise RuntimeError("日時の変更で公開済み地形が変わりました")
		if current["ground"].published_revision != current["groundRevision"]:
			raise RuntimeError("静止中の日時変更で足場の版が変わりました")
		if planet.climate_preparation_count != current["climateCount"]:
			raise RuntimeError("日時の変更で気候を再準備しました")
		material = planet.get_visible_mesh().get_material(0)
		phase = material.get_scalar_parameter_value("SolarYearPhase")
		if abs(phase - planet.solar_year_phase) > 2.0e-7:
			raise RuntimeError("季節材質と共通時計の位相が一致しません")
		lights = planet.get_components_by_class(unreal.DirectionalLightComponent)
		if len(lights) != 2:
			raise RuntimeError("惑星に属する天体照明が二灯ではありません")
		for light in lights:
			index = light.get_editor_property("atmosphere_sun_light_index")
			expected = planet.sun_direction_world if index == 0 else planet.moon_direction_world
			expected_shadows = (planet.sun_altitude_degrees >= -4 if index == 0 else
				planet.sun_altitude_degrees < -4 and planet.moon_altitude_degrees > 0)
			if light.get_editor_property("cast_shadows") != expected_shadows:
				raise RuntimeError("地平線と主照明に応じた影の切替が一致しません")
			actual = -light.get_forward_vector()
			if (actual - expected).length() > 1.0e-6:
				raise RuntimeError("天体方向が実照明へ届いていません")
		priorities = [light.get_editor_property("forward_shading_priority") for light in lights]
		if sorted(priorities) != [0, 1]:
			raise RuntimeError("二灯の主照明の優先順位が一意ではありません")
		domes = planet.get_components_by_class(unreal.StaticMeshComponent)
		if len(domes) != 1:
			raise RuntimeError("天球の描画面が一つではありません")
		dome = domes[0]
		camera = unreal.GameplayStatics.get_player_camera_manager(planet.get_world(), 0)
		if (dome.get_world_location() - camera.get_camera_location()).length() > 0.1:
			raise RuntimeError("天球の中心が現在のカメラへ追従していません")
		sky = dome.get_material(0)
		for name, expected in (("MoonDirection", planet.moon_direction_world), ("MoonToSun", planet.moon_to_sun_direction_world)):
			value = sky.get_vector_parameter_value(name)
			if (unreal.Vector(value.r, value.g, value.b) - expected).length() > 1.0e-6:
				raise RuntimeError("天体の方向が天球材質へ届いていません：" + name)
		angle = math.sin(math.radians(planet.moon_angular_diameter_degrees) / 2)
		if abs(sky.get_scalar_parameter_value("MoonSinRadius") - angle) > 1.0e-8:
			raise RuntimeError("月の視直径が天球材質へ届いていません")
		up = current["body"].get_actor_location() - planet.get_actor_location()
		up /= up.length()
		for direction, altitude in ((planet.sun_direction_world, planet.sun_altitude_degrees),
			(planet.moon_direction_world, planet.moon_altitude_degrees)):
			dot = up.x * direction.x + up.y * direction.y + up.z * direction.z
			angle = math.degrees(math.asin(max(-1, min(1, dot))))
			if abs(angle - altitude) > 0.001:
				raise RuntimeError("惑星姿勢への変換で天体高度が変わりました")
		current["samples"].append({
			"dateUtc": dates[current["index"]], "solarYearPhase": planet.solar_year_phase,
			"sunAltitudeDegrees": planet.sun_altitude_degrees, "moonAltitudeDegrees": planet.moon_altitude_degrees,
			"moonIlluminatedFraction": planet.moon_illuminated_fraction,
			"moonAngularDiameterDegrees": planet.moon_angular_diameter_degrees, "forwardLightPriorities": priorities,
			"sunDirection": [planet.sun_direction_world.x, planet.sun_direction_world.y, planet.sun_direction_world.z]
		})
		current["index"] += 1
		if current["index"] == len(dates):
			finish(current, None)
		else:
			set_date(current, current["index"])
	except Exception as e:
		finish(current, str(e))
	# 終了
	return


if args.get("operation", "read") == "begin":
	if state and not state["completed"]:
		raise RuntimeError("前の時計検査が続いています")
	world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
	if world is None:
		raise RuntimeError("探索レベルで PIE を開始して下さい")
	mode = unreal.GameplayStatics.get_game_mode(world)
	if not isinstance(mode, unreal.PlanetGameMode) or mode.explorer is None:
		raise RuntimeError("探索の開始準備が完了していません")
	planet = mode.planet
	if planet.generation_busy or mode.ground.busy:
		raise RuntimeError("初期の表示と足場の公開を待って下さい")
	state = {
		"planet": planet, "body": mode.explorer, "ground": mode.ground,
		"originalUtc": planet.simulation_utc, "originalAdvance": planet.advance_time,
		"revision": planet.published_revision, "groundRevision": mode.ground.published_revision,
		"climateCount": planet.climate_preparation_count, "meshHash": mesh_hash(planet),
		"index": 0, "samples": [], "completed": False, "error": None, "handle": None
	}
	setattr(builtins, key, state)
	planet.set_editor_property("advance_time", False)
	try:
		set_date(state, 0)
		state["handle"] = unreal.register_slate_post_tick_callback(tick)
	except Exception as e:
		finish(state, str(e))
	RESULT = {"started": state["handle"] is not None, "error": state["error"]}
else:
	if not state:
		raise RuntimeError("時計検査を開始して下さい")
	RESULT = {"completed": state["completed"], "error": state["error"], "samples": state["samples"]}
