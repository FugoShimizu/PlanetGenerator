"""Physical locomotion acceptance probe in PIE; not a visual terrain acceptance test."""
import builtins
import math
import time
import struct
import unreal
args=globals().get("ARGS",{})
rendered=bool(args.get("rendered",False))
run_seconds=float(args.get("run_seconds",4))
run_speed=float(args.get("run_speed_meters_per_second",9.0))
if not math.isfinite(run_speed) or not 0<run_speed<=20:
	raise RuntimeError("run_speed_meters_per_second must be between 0 and 20")
if not math.isfinite(run_seconds) or not 4<=run_seconds<=60:
	raise RuntimeError("run_seconds must be between 4 and 60")
run_end=2.6+run_seconds
reverse_end=run_end+2
brake_end=reverse_end+1
old=getattr(builtins,"_planet_character_probe",None)
if old and old.get("status")=="running":
	raise RuntimeError("Character probe already running")
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
if world is None:
	raise RuntimeError("Start PIE before running this probe")
radius=float(args.get("radius_meters",1000000))
rotation=unreal.Rotator(pitch=float(args.get("pitch",0)),yaw=float(args.get("yaw",0)),roll=float(args.get("roll",0)))
terrain=unreal.PlanetGroundPatch.spawn_probe_actor(world,unreal.PlanetTerrain,unreal.Vector(0,0,1e10))
ground=unreal.PlanetGroundPatch.spawn_probe_actor(world,unreal.PlanetGroundPatch,unreal.Vector())
if terrain is None or ground is None:
	raise RuntimeError("Failed to create physical test actors")
terrain.set_actor_label("Planet_CharacterProbeSource")
ground.set_actor_label("Planet_CharacterProbeGround")
terrain.set_editor_property("follow_player_camera",False)
terrain.set_editor_property("enable_surface_preparation",bool(args.get("surface_preparation",True)))
terrain.set_editor_property("leaf_budget",512)
terrain.set_editor_property("seed",42)
terrain.set_editor_property("radius_meters",radius)
ground.set_editor_property("terrain",terrain)
ground.set_editor_property("half_extent_meters",16.0)
ground.set_editor_property("cells_per_side",32)
state={"status":"running","phase":"terrain","start":time.monotonic(),"handle":None,"terrain":terrain,"ground":ground,
	   "samples":0,"old_collision_frames":0,"published_revisions":[],"max_speed_mps":0,"min_margin_m":1e9,
	   "min_clearance_m":1e9,"max_grounded_clearance_m":0,"jump_apex_m":0,"jump_air_samples":0,
	   "jump_landed":False,"travel_m":0,"max_gravity_error_mps2":0,"max_gravity_magnitude_error_mps2":0,
	   "max_up_error":0,"camera_checks":0,"render_samples":0,"render_revisions":[],
	   "max_render_collision_error_m":0,"render_diagnostics":[],"render_worker_samples":[],
	   "ground_worker_samples":[],"maximum_body_render_offset_m":0}
builtins._planet_character_probe=state
if not terrain.request_generation():
	raise RuntimeError(terrain.generation_message)

def unit(v):
	return v/v.length()

def dot(a,b):
	return unreal.MathLibrary.dot_vector_vector(a,b)

def observe_failed_candidate(delta):
	# Diagnostic only: the traversal is already failed and its result frozen.
	# Let its submitted candidate finish, without issuing further view work.
	try:
		elapsed=time.monotonic()-state["failure_observation_start"]
		if terrain.generation_busy and elapsed<30:
			return
		focus=terrain.visible_surface_observer_meters
		state["result"]["failed_candidate_completion"]={"seconds_after_failure":elapsed,
			"busy":terrain.generation_busy,"published_revision":terrain.published_revision,
			"requested_revision":terrain.requested_revision,"worker_ms":terrain.last_worker_milliseconds,
			"message":terrain.generation_message,"published_details":terrain.last_published_generation_details,
			"published_surface_focus_m":[focus.x,focus.y,focus.z],"leaves":terrain.visible_leaf_count,
			"traversal_still_failed":True,"character_and_ground_removed_after_failure":True}
	except Exception as error:
		state["result"]["candidate_observation_error"]=str(error)
	unreal.unregister_slate_post_tick_callback(state["handle"])
	state["handle"]=None
	terrain.destroy_actor()
	state.pop("terrain",None)
	state["status"]="failed"


def finish(error=None):
	state["status"]="failed" if error else "passed"
	if state["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(state["handle"])
		state["handle"]=None
	c=state.get("character")
	state["coverage_miss_frames"]=c.coverage_miss_frames if c else None
	state["refresh_requests"]=c.ground_refresh_requests if c else None
	state["terminal_ground_diagnostic"]=[{"revision":ground.published_revision,
		"requested_revision":ground.requested_revision,"busy":ground.busy,
		"message":ground.status,"worker_ms":ground.last_worker_milliseconds,
		"margin_m":c.ground_margin_meters if c else None}]
	if rendered:
		mesh=terrain.get_visible_mesh()
		body=c.get_actor_location() if c else unreal.Vector()
		local=unreal.MathLibrary.inverse_transform_location(mesh.get_world_transform(),body) if mesh else unreal.Vector()
		state["terminal_render_diagnostic"]=[{"revision":terrain.published_revision,
			"requested_revision":terrain.requested_revision,"busy":terrain.generation_busy,
			"message":terrain.generation_message,"worker_ms":terrain.last_worker_milliseconds,
			"body_mesh_local_m":[local.x/100,local.y/100,local.z/100],
			"body_planet_m":[v/100 for v in (
				unreal.MathLibrary.inverse_transform_location(terrain.get_actor_transform(),body).x,
				unreal.MathLibrary.inverse_transform_location(terrain.get_actor_transform(),body).y,
				unreal.MathLibrary.inverse_transform_location(terrain.get_actor_transform(),body).z)],
			"camera_observer_m":[terrain.observer_meters.x,terrain.observer_meters.y,terrain.observer_meters.z],
			"focal_length_pixels":terrain.focal_length_pixels,"surface_radius_m":terrain.visible_surface_radius_meters}]
	state["surface_preparation"]=[{"enabled":terrain.enable_surface_preparation,"busy":terrain.preparation_busy,
		"slices":terrain.preparation_slices,"interruptions":terrain.interrupted_preparations,
		"tile_visits":terrain.prepared_tile_visits,"height_evaluations":terrain.prepared_height_evaluations,
		"last_burst_ms":terrain.last_preparation_milliseconds}]
	state["result"]={k:v for k,v in state.items() if isinstance(v,(str,int,float,bool,list))}
	state["result"].update(error=error,seconds=time.monotonic()-state["start"],radius_meters=radius,
		run_seconds=run_seconds,river_reach_index=args.get("river_reach_index"),river_fraction=args.get("river_fraction",0.5),
		rotation_degrees=[rotation.pitch,rotation.yaw,rotation.roll],physics_world="PIE",render_collision_match_validated=False,
		full_gameplay_validated=False,saved_terrain_assets=False)
	state["result"].update(rendered_mode=rendered,possessed_character=state.get("possession_verified",False),
		gpu_position_conversion_emulated=rendered,gpu_readback=False,render_collision_match_validated=rendered and not error)
	if state.get("controller") and state.get("previous_pawn"):
		state["controller"].possess(state["previous_pawn"])
	for key in ("controller","previous_pawn","render_bins","render_mesh","render_triangles"):
		state.pop(key,None)
	observe=bool(error and rendered and args.get("observe_failed_candidate",False) and terrain.generation_busy)
	if observe:
		terrain.set_editor_property("follow_player_camera",False)
	for key in (("character","ground") if observe else ("character","ground","terrain")):
		actor=state.pop(key,None)
		if actor:
			actor.destroy_actor()
	if observe:
		state["status"]="running"
		state["phase"]="failed_candidate"
		state["failure_observation_start"]=time.monotonic()
		state["handle"]=unreal.register_slate_post_tick_callback(observe_failed_candidate)

def check_cameras(c):
	position=c.get_actor_location()
	for mode in (unreal.PlanetViewMode.FIRST_PERSON,unreal.PlanetViewMode.THIRD_PERSON,unreal.PlanetViewMode.OVERVIEW):
		c.set_view_mode(mode)
		if (c.get_actor_location()-position).length()>1e-6:
			raise RuntimeError("View change moved the body")
		cameras=c.get_components_by_class(unreal.CameraComponent)
		if sum(1 for camera in cameras if camera.is_active())!=1:
			raise RuntimeError("Expected exactly one active native camera")
		state["camera_checks"]+=1
	c.set_view_mode(unreal.PlanetViewMode.FIRST_PERSON)

def trace_render_triangles(triangles,start,ray):
	for a,b,c in triangles:
		e1=b-a; e2=c-a; p=unreal.MathLibrary.cross_vector_vector(ray,e2); det=dot(e1,p)
		if abs(det)<1e-12:
			continue
		s=start-a; u=dot(s,p)/det
		if u<-1e-9 or u>1+1e-9:
			continue
		q=unreal.MathLibrary.cross_vector_vector(s,e1); v=dot(ray,q)/det
		if v<-1e-9 or u+v>1+1e-9:
			continue
		t=dot(e2,q)/det
		if 0<=t<=1:
			return start+ray*t
	return None


def inspect_render(position,up):
	mesh=terrain.get_visible_mesh()
	if mesh is None or not mesh.is_mesh_section_visible(0):
		raise RuntimeError("Visible ground disappeared during walking")
	if terrain.published_revision not in state["render_revisions"]:
		state["render_revisions"].append(terrain.published_revision)
		ocean=terrain.get_visible_ocean_mesh()
		if ocean is None or not ocean.is_mesh_section_visible(0) or terrain.visible_ocean_triangle_count<=0:
			raise RuntimeError("Ocean missing during a terrain publication")
		water_material=ocean.get_material(0)
		if not isinstance(water_material,unreal.MaterialInstanceDynamic):
			raise RuntimeError("Ocean frame material missing during traversal")
		water_origin=ocean.get_relative_transform().translation/100
		water_phase=water_material.get_vector_parameter_value("TextureOrigin")
		for actual,expected in zip((water_phase.r,water_phase.g,water_phase.b),(water_origin.x,water_origin.y,water_origin.z)):
			error=abs(math.remainder(actual-expected%512,512))
			state["max_ocean_phase_error_m"]=max(state.get("max_ocean_phase_error_m",0),error)
			if error>0.0002: raise RuntimeError("Ocean texture frame mismatch during traversal")
		state["ocean_publication_checks"]=state.get("ocean_publication_checks",0)+1
		state["max_ocean_build_ms"]=max(state.get("max_ocean_build_ms",0),terrain.last_ocean_build_milliseconds)
		material=mesh.get_material(0)
		if not isinstance(material,unreal.MaterialInstanceDynamic):
			raise RuntimeError("Published terrain lacks its own texture frame material")
		relative=mesh.get_relative_transform()
		origin=relative.translation/100
		phase=material.get_vector_parameter_value("TextureOrigin")
		for actual,expected in zip((phase.r,phase.g,phase.b),(origin.x,origin.y,origin.z)):
			error=abs(math.remainder(actual-expected%512,512))
			state["max_material_phase_error_m"]=max(state.get("max_material_phase_error_m",0),error)
			if error>0.0002: raise RuntimeError("Texture phase did not follow published render origin")
		for name,basis in (("TextureAxisX",unreal.Vector(1,0,0)),
			("TextureAxisY",unreal.Vector(0,1,0)),("TextureAxisZ",unreal.Vector(0,0,1))):
			value=material.get_vector_parameter_value(name)
			expected=unreal.MathLibrary.transform_direction(relative,basis)
			if (unreal.Vector(value.r,value.g,value.b)-expected).length()>1e-6:
				raise RuntimeError("Texture axis did not follow published render orientation")
		state["material_frame_checks"]=state.get("material_frame_checks",0)+1
		mesh_origin=unreal.MathLibrary.transform_location(mesh.get_world_transform(),unreal.Vector())
		state["render_worker_samples"].append({"revision":terrain.published_revision,
			"worker_ms":terrain.last_worker_milliseconds,
			"details":terrain.last_published_generation_details,"leaves":terrain.visible_leaf_count,
			"ocean_build_ms":terrain.last_ocean_build_milliseconds,"ocean_triangles":terrain.visible_ocean_triangle_count,
			"preparation_slices":terrain.preparation_slices,"preparation_interruptions":terrain.interrupted_preparations,
			"prepared_height_evaluations":terrain.prepared_height_evaluations,
			"mesh_origin_cm":[mesh_origin.x,mesh_origin.y,mesh_origin.z],
			"surface_focus_m":[terrain.visible_surface_observer_meters.x,
				terrain.visible_surface_observer_meters.y,terrain.visible_surface_observer_meters.z]})
		vertices,indices,_,_,_=unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(mesh,0)
		def f32(x):
			return struct.unpack("<f",struct.pack("<f",x))[0]
		vertices=[unreal.Vector(f32(v.x),f32(v.y),f32(v.z)) for v in vertices]
		# Broad phase follows the published disk, independently of the retained
		# mesh origin. Do not clip away steep terrain with an arbitrary Z slab.
		focus_world=unreal.MathLibrary.transform_location(
			terrain.get_actor_transform(),terrain.visible_surface_observer_meters*100)
		focus_local=unreal.MathLibrary.inverse_transform_location(mesh.get_world_transform(),focus_world)
		path_world=unreal.MathLibrary.transform_location(terrain.get_actor_transform(),
			terrain.visible_surface_path_end_meters*100)
		path_local=unreal.MathLibrary.inverse_transform_location(mesh.get_world_transform(),path_world)
		xmin_limit=min(focus_local.x,path_local.x)-3200
		xmax_limit=max(focus_local.x,path_local.x)+3200
		ymin_limit=min(focus_local.y,path_local.y)-3200
		ymax_limit=max(focus_local.y,path_local.y)+3200
		bins={}
		all_triangles=[]
		for i in range(0,len(indices),3):
			a,b,c=[vertices[int(indices[i+k])] for k in range(3)]
			triangle=(a,b,c); all_triangles.append(triangle)
			xmin=max(xmin_limit,min(a.x,b.x,c.x)); xmax=min(xmax_limit,max(a.x,b.x,c.x))
			ymin=max(ymin_limit,min(a.y,b.y,c.y)); ymax=min(ymax_limit,max(a.y,b.y,c.y))
			if xmin>xmax or ymin>ymax:
				continue
			for x in range(math.floor(xmin/200),math.floor(xmax/200)+1):
				for y in range(math.floor(ymin/200),math.floor(ymax/200)+1):
					bins.setdefault((x,y),[]).append(triangle)
		state["render_bins"]=bins
		state["render_triangles"]=tuple(all_triangles)
	transform=mesh.get_world_transform()
	planet_transform=terrain.get_actor_transform()
	anchor=unit(terrain.visible_surface_observer_meters)
	body_direction=unit(unreal.MathLibrary.inverse_transform_location(planet_transform,position))
	endpoint=unit(terrain.visible_surface_path_end_meters)
	span=endpoint-anchor
	span_squared=span.x*span.x+span.y*span.y+span.z*span.z
	displacement=body_direction-anchor
	fraction=max(0,min(1,(displacement.x*span.x+displacement.y*span.y+
		displacement.z*span.z)/span_squared)) if span_squared>0 else 0
	offset=(displacement-span*fraction).length()*radius
	sagitta=radius*(1-math.sqrt(max(0,1-span_squared/4)))
	state["maximum_body_render_offset_m"]=max(state["maximum_body_render_offset_m"],offset)
	if offset+0.34>terrain.visible_surface_radius_meters+sagitta+1e-6:
		raise RuntimeError(f"Body capsule left mandatory refined corridor: offset={offset} m, "
			f"radius={terrain.visible_surface_radius_meters} m, path={span_squared**0.5*radius} m")
	start=unreal.MathLibrary.inverse_transform_location(transform,position+up*300)
	end=unreal.MathLibrary.inverse_transform_location(transform,position-up*400)
	ray=end-start
	start_candidates=state["render_bins"].get((math.floor(start.x/200),math.floor(start.y/200)),[])
	# A radial ray is oblique in a retained frame. Visit every cell in its
	# projected segment bounds, not only the cell containing its origin.
	candidate_map={}
	for x in range(math.floor(min(start.x,end.x)/200),math.floor(max(start.x,end.x)/200)+1):
		for y in range(math.floor(min(start.y,end.y)/200),math.floor(max(start.y,end.y)/200)+1):
			for triangle in state["render_bins"].get((x,y),[]):
				candidate_map[id(triangle)]=triangle
	candidates=list(candidate_map.values())
	if len(state["render_diagnostics"])<12:
		camera=unreal.GameplayStatics.get_player_camera_manager(world,0).get_camera_location()
		state["render_diagnostics"].append({"revision":terrain.published_revision,
			"surface_radius":terrain.visible_surface_radius_meters,
			"observer":[terrain.observer_meters.x,terrain.observer_meters.y,terrain.observer_meters.z],
			"body":[position.x,position.y,position.z],"camera":[camera.x,camera.y,camera.z],
			"local_ray_start":[start.x,start.y,start.z],"candidates":len(candidates)})
	local_hit=trace_render_triangles(candidates,start,ray)
	if local_hit is None:
		full_hit=trace_render_triangles(state["render_triangles"],start,ray)
		state["full_scan_on_failure"]=[{"hit":full_hit is not None,"triangles":len(state["render_triangles"]),
			"candidates":len(candidates),"start_cm":[start.x,start.y,start.z],"end_cm":[end.x,end.y,end.z]}]
		raise RuntimeError("No displayed triangle beneath moving body; full scan hit="+str(full_hit is not None))
	if trace_render_triangles(start_candidates,start,ray) is None:
		state["start_bin_miss_count"]=state.get("start_bin_miss_count",0)+1
		if "start_bin_miss_proof" not in state:
			full_hit=trace_render_triangles(state["render_triangles"],start,ray)
			if full_hit is None or (full_hit-local_hit).length()>1e-5:
				raise RuntimeError("Segment broad phase differs from independent full triangle scan")
			state["start_bin_miss_proof"]=[{"revision":terrain.published_revision,"travel_m":state["travel_m"],
				"start_candidates":len(start_candidates),"segment_candidates":len(candidates),
				"all_triangles":len(state["render_triangles"]),"full_scan_difference_m":(full_hit-local_hit).length()/100,
				"start_cm":[start.x,start.y,start.z],"end_cm":[end.x,end.y,end.z]}]
	hit=unreal.MathLibrary.transform_location(transform,local_hit)
	physical=ground.trace_ground(position+up*300,position-up*400)
	if physical is None:
		raise RuntimeError("No collision beneath moving rendered body")
	error=(hit-physical[0]).length()/100
	state["max_render_collision_error_m"]=max(state["max_render_collision_error_m"],error)
	state["render_samples"]+=1
	if error>0.04:
		raise RuntimeError(f"Displayed ground disagrees with collision by {error} m")

def tick(delta):
	try:
		if time.monotonic()-state["start"]>180:
			raise RuntimeError("Character probe timed out")
		if state["phase"]=="terrain":
			if terrain.generation_busy or not terrain.published_revision:
				return
			if "river_reach_index" in args:
				profile=terrain.evaluate_river_profile(int(args["river_reach_index"]),float(args.get("river_fraction",0.5)))
				if profile is None:
					raise RuntimeError("Invalid river reach for character probe")
				d=unit(profile[0])
			else:
				d=unit(unreal.Vector(1,2,3))
			h,_=terrain.evaluate_direction(d)
			transform=unreal.Transform(location=unreal.Vector(),rotation=rotation)
			point=unreal.MathLibrary.transform_location(transform,d*((radius+h)*100))
			terrain.set_actor_location_and_rotation(unreal.Vector(0,0,500000)-point,rotation,False,False)
			ground.set_editor_property("anchor_direction",d)
			if not ground.request_patch():
				raise RuntimeError(ground.status)
			state["direction"]=d
			if rendered:
				terrain.set_editor_property("leaf_budget",int(args.get("leaf_budget",32768)))
				terrain.set_editor_property("maximum_level",32)
				terrain.set_editor_property("observer_meters",d*(radius+h+1.6))
				if not terrain.request_generation():
					raise RuntimeError(terrain.generation_message)
			state["phase"]="ground"
			return
		if state["phase"]=="ground":
			if rendered and terrain.generation_busy:
				return
			if rendered and terrain.visible_surface_radius_meters<=0:
				raise RuntimeError("No prepared surface rendering before character spawn")
			if ground.busy:
				return
			if ground.published_revision!=ground.requested_revision:
				raise RuntimeError(ground.status)
			c=unreal.PlanetExplorerCharacter.spawn_on_prepared_ground(ground,state["direction"])
			if c is None:
				raise RuntimeError("Safe spawn on prepared land failed")
			state["character"]=c
			c.set_editor_property("run_speed_meters_per_second",run_speed)
			if rendered:
				controller=unreal.GameplayStatics.get_player_controller(world,0)
				if controller is None:
					raise RuntimeError("No player controller for rendered traversal")
				state["controller"]=controller
				state["previous_pawn"]=controller.get_controlled_pawn()
				controller.possess(c)
				if controller.get_controlled_pawn()!=c:
					raise RuntimeError("Native character was not possessed")
				state["possession_verified"]=True
				terrain.set_editor_property("follow_player_camera",True)
			check_cameras(c)
			state["phase"]="settle"
			state["last_position"]=c.get_actor_location()
			state["last_time"]=c.traversal_seconds
			return
		c=state["character"]
		m=c.get_movement_component()
		t=c.traversal_seconds
		state["ground_half_extent_m"]=ground.half_extent_meters
		pos=c.get_actor_location()
		relative=pos-terrain.get_actor_location()
		up=unit(relative)
		local=unit(unreal.MathLibrary.inverse_transform_location(terrain.get_actor_transform(),pos))
		h,terrain_normal=terrain.evaluate_direction(local)
		clearance=relative.length()/100-radius-h-0.96
		gravity=m.get_planet_gravity_vector()/100
		state["max_gravity_magnitude_error_mps2"]=max(state["max_gravity_magnitude_error_mps2"],abs(gravity.length()-9.80665))
		state["max_gravity_error_mps2"]=max(state["max_gravity_error_mps2"],(gravity+up*9.80665).length())
		state["max_up_error"]=max(state["max_up_error"],(c.get_actor_up_vector()-up).length())
		speed=c.get_velocity().length()/100
		state["max_speed_mps"]=max(state["max_speed_mps"],speed)
		state["min_margin_m"]=min(state["min_margin_m"],c.ground_margin_meters)
		state["min_clearance_m"]=min(state["min_clearance_m"],clearance)
		if m.is_moving_on_ground():
			state["max_grounded_clearance_m"]=max(state["max_grounded_clearance_m"],clearance)
		if ground.busy:
			state["old_collision_frames"]+=1
		if ground.published_revision not in state["published_revisions"]:
			state["published_revisions"].append(ground.published_revision)
			state["ground_worker_samples"].append({"revision":ground.published_revision,
				"worker_ms":ground.last_worker_milliseconds})
		dt=t-state["last_time"]
		distance=(pos-state["last_position"]).length()/100
		velocity=c.get_velocity()/100
		radial_speed=dot(velocity,up)
		tangent_speed=math.sqrt(max(0,speed*speed-radial_speed*radial_speed))
		delta=(pos-state["last_position"])/100
		midpoint_up=unit(up+unit(state["last_position"]-terrain.get_actor_location()))
		tangent_displacement=(delta-midpoint_up*dot(delta,midpoint_up)).length()
		state["max_tangent_speed_mps"]=max(state.get("max_tangent_speed_mps",0),tangent_speed)
		if speed>run_speed+0.2 or (dt>0 and distance>(run_speed+1)*dt+0.1):
			old_rejections=state.setdefault("total_motion_limit_rejections",[])
			if len(old_rejections)<12:
				old_rejections.append({"time_s":t,"total_speed_mps":speed,"tangent_speed_mps":tangent_speed,
					"radial_speed_mps":radial_speed,"displacement_m":distance,"tangent_displacement_m":tangent_displacement,
					"frame_seconds":dt,"falling":m.is_falling()})
		state["terminal_motion"]=[{"velocity_mps":[velocity.x,velocity.y,velocity.z],
			"radial_speed_mps":radial_speed,"tangent_speed_mps":tangent_speed,
			"moving_on_ground":m.is_moving_on_ground(),"falling":m.is_falling(),
			"configured_walk_speed_mps":m.max_walk_speed/100,
			"maintain_horizontal_ground_velocity":m.maintain_horizontal_ground_velocity,
			"frame_displacement_m":distance,"tangent_displacement_m":tangent_displacement,"frame_seconds":dt}]
		state["terminal_motion"][0].update(time_s=t,phase=state["phase"],
			position_planet_m=[local.x*relative.length()/100,local.y*relative.length()/100,local.z*relative.length()/100],
			height_m=h,terrain_slope_degrees=math.degrees(math.acos(max(-1,min(1,dot(unit(terrain_normal),local))))),
			ground_revision=ground.published_revision,render_revision=terrain.published_revision)
		if t>=state.get("next_motion_sample",0):
			history=state.setdefault("motion_history",[])
			history.append(dict(state["terminal_motion"][0]))
			if len(history)>256:
				del history[0]
			state["next_motion_sample"]=t+0.25
		# MaxWalkSpeed controls motion in the plane perpendicular to gravity.
		# Falling and following a ramp add radial motion; they must not be
		# mistaken for horizontal acceleration or a streaming teleport.
		if state["phase"]=="run" and tangent_speed<1 and t>5 and not state.get("obstacle_profile"):
			forward=c.get_actor_forward_vector()
			forward=unit(forward-up*dot(forward,up))
			profile=[]
			for step in range(33):
				offset=step*0.125
				probe=pos+forward*(offset*100)
				probe_local=unit(unreal.MathLibrary.inverse_transform_location(terrain.get_actor_transform(),probe))
				probe_h,probe_n=terrain.evaluate_direction(probe_local)
				profile.append({"forward_m":offset,"height_m":probe_h,
					"slope_degrees":math.degrees(math.acos(max(-1,min(1,dot(unit(probe_n),probe_local)))))})
			sweep=ground.sweep_ground_capsule(pos,pos+forward*100,up,34,96)
			obstacle={"time_s":t,"profile":profile,"capsule_hit":sweep is not None}
			if sweep:
				hit,hit_normal=sweep
				hit_local=unit(unreal.MathLibrary.inverse_transform_location(terrain.get_actor_transform(),hit))
				hit_h,field_normal=terrain.evaluate_direction(hit_local)
				obstacle.update(impact_slope_degrees=math.degrees(math.acos(max(-1,min(1,dot(unit(hit_normal),up))))),
					field_slope_at_impact_degrees=math.degrees(math.acos(max(-1,min(1,dot(unit(field_normal),hit_local))))),
					field_height_at_impact_m=hit_h,
					impact_height_m=(hit-terrain.get_actor_location()).length()/100-radius)
			state["obstacle_profile"]=[obstacle]
		if dt>0 and tangent_displacement>(run_speed+1)*dt+0.1:
			raise RuntimeError(f"Unexpected tangential body displacement {tangent_displacement} m in {dt} s")
		radial_displacement=abs(dot(delta,midpoint_up))
		radial_motion_bound=(abs(h-state.get("last_terrain_height",h))+m.max_step_height/100+0.04+
			max(abs(radial_speed),abs(state.get("last_radial_speed",0)),m.jump_z_velocity/100)*max(0,dt)+
			0.5*9.80665*max(0,dt)**2)
		state["terminal_motion"][0].update(radial_displacement_m=radial_displacement,radial_motion_bound_m=radial_motion_bound)
		if dt>0 and radial_displacement>radial_motion_bound:
			raise RuntimeError(f"Unexpected radial body displacement {radial_displacement} m; bound={radial_motion_bound} m")
		state["last_terrain_height"]=h
		state["last_radial_speed"]=radial_speed
		state["travel_m"]+=distance
		state["last_time"]=t
		state["last_position"]=pos
		state["samples"]+=1
		if rendered:
			inspect_render(pos,up)
		if c.coverage_miss_frames or clearance < -0.03 or tangent_speed>run_speed+0.2:
			raise RuntimeError(f"Locomotion invariant failed: misses={c.coverage_miss_frames}, "
				f"clearance={clearance}, tangent_speed={tangent_speed}, total_speed={speed}")
		if state["max_gravity_magnitude_error_mps2"]>1e-5 or state["max_gravity_error_mps2"]>0.1 or state["max_up_error"]>0.01:
			raise RuntimeError("Radial gravity or body orientation mismatch")
		if t<0.6:
			return
		if t<2.6:
			state["phase"]="walk"
			c.set_move_input(1,0,False)
		elif t<run_end:
			if rendered and state["phase"]!="run":
				c.set_view_mode(unreal.PlanetViewMode.THIRD_PERSON)
			state["phase"]="run"
			c.set_move_input(1,0,True)
		elif t<reverse_end:
			if rendered and state["phase"]!="reverse":
				c.set_view_mode(unreal.PlanetViewMode.FIRST_PERSON)
			state["phase"]="reverse"
			c.set_move_input(-1,0.5,True)
		elif t<brake_end:
			state["phase"]="brake"
			c.set_move_input(0,0,False)
		elif not state.get("jump_requested",False):
			if not m.is_moving_on_ground():
				raise RuntimeError("Not grounded before jump")
			c.jump()
			state["jump_requested"]=True
			state["jump_time"]=t
			state["phase"]="jump"
		elif t<state["jump_time"]+1.6:
			state["jump_apex_m"]=max(state["jump_apex_m"],clearance)
			if not m.is_moving_on_ground():
				state["jump_air_samples"]+=1
			elif state["jump_air_samples"]:
				state["jump_landed"]=True
			if t-state["jump_time"]>0.2:
				c.stop_jumping()
		elif not state.get("overview_start"):
			if not state["jump_landed"] or state["jump_apex_m"]<0.5:
				raise RuntimeError("Jump did not produce an airborne arc and landing")
			check_cameras(c)
			c.set_view_mode(unreal.PlanetViewMode.OVERVIEW)
			c.set_move_input(1,0,True)
			state["overview_start"]=t
			state["overview_position"]=pos
			state["phase"]="overview"
		elif t<state["overview_start"]+0.7:
			if (pos-state["overview_position"]).length()>5:
				raise RuntimeError("Overview movement input moved the body")
		else:
			if (state["travel_m"]<45+7*(run_seconds-4) or
				len(state["published_revisions"])<3 or not state["old_collision_frames"]):
				raise RuntimeError("Insufficient traversal or collision replacement coverage")
			finish()
	except Exception as error:
		finish(str(error))
state["handle"]=unreal.register_slate_post_tick_callback(tick)
RESULT={"status":"started","read_result_with":"Scripts/Editor/read_character_probe.py"}
