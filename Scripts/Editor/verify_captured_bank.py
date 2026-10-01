"""Differentiate the final UE terrain at the previously blocking river bank."""
import math
import unreal
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
found=[a for a in unreal.GameplayStatics.get_all_actors_of_class(editor.get_editor_world(),unreal.PlanetTerrain)
       if a.get_actor_label()=="Planet_SurfaceRenderProbe"]
if len(found)!=1 or found[0].radius_meters!=1000 or found[0].published_revision<=0:
    raise RuntimeError("Prepare the R1000 static surface probe")
terrain=found[0]
def unit(v): return v/v.length()
def dot(a,b): return unreal.MathLibrary.dot_vector_vector(a,b)
origin=unreal.Vector(823.7846000179857,507.7657612806537,-264.3599423770205)
up=unit(origin)
heading=unreal.Vector(4.940781568763943,-7.50949595608013,0.4428859620514302)
forward=unit(heading-up*dot(heading,up))
across=unreal.MathLibrary.cross_vector_vector(up,forward)
maximum_tilt=0
maximum_normal_error=0
count=0
for along in range(-8,33):
    for side in range(-4,5):
        d=unit(origin+forward*(along*0.125)+across*(side*0.125))
        h,n=terrain.evaluate_direction(d)
        maximum_tilt=max(maximum_tilt,math.degrees(math.acos(max(-1,min(1,dot(unit(n),d))))))
        tangent=unit(forward-d*dot(forward,d))
        second=unreal.MathLibrary.cross_vector_vector(d,tangent)
        for axis in (tangent,second):
            eps=1e-6
            def difference(e):
                hp,_=terrain.evaluate_direction(d*math.cos(e)+axis*math.sin(e))
                hm,_=terrain.evaluate_direction(d*math.cos(e)-axis*math.sin(e))
                return (hp-hm)/(2*e)
            derivative=(4*difference(eps/2)-difference(eps))/3
            finite_tilt=math.atan(derivative/(1000+h))
            analytic_tilt=math.atan(-dot(n,axis)/dot(n,d))
            maximum_normal_error=max(maximum_normal_error,abs(finite_tilt-analytic_tilt))
        count+=1
if maximum_tilt>=40 or maximum_normal_error>=1e-5:
    raise RuntimeError(f"Captured bank failed: slope={maximum_tilt}, normal_error={maximum_normal_error}")
RESULT={"status":"passed","samples":count,"maximum_slope_degrees":maximum_tilt,
        "maximum_independent_normal_error_radians":maximum_normal_error,
        "continuous_global_certificate":False,"saved_terrain_assets":False}
