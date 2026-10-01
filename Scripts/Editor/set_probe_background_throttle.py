"""Temporarily permit UE acceptance probes to tick when Windows denies focus."""
import builtins
import unreal
settings=unreal.get_default_object(unreal.load_class(None,"/Script/UnrealEd.EditorPerformanceSettings"))
key="_planet_probe_background_throttle"
restore=bool(globals().get("ARGS",{}).get("restore",False))
if restore:
    if not hasattr(builtins,key):
        raise RuntimeError("No saved background throttle setting")
    value=getattr(builtins,key)
    settings.set_editor_property("bThrottleCPUWhenNotForeground",value)
    if settings.get_editor_property("bThrottleCPUWhenNotForeground")!=value:
        raise RuntimeError("Background throttle restoration failed")
    delattr(builtins,key)
else:
    if not hasattr(builtins,key):
        setattr(builtins,key,settings.get_editor_property("bThrottleCPUWhenNotForeground"))
    settings.set_editor_property("bThrottleCPUWhenNotForeground",False)
RESULT={"restored":restore,"bThrottleCPUWhenNotForeground":settings.get_editor_property("bThrottleCPUWhenNotForeground"),"save_config_called":False}
