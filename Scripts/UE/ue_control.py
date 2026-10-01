"""Control this project via Epic's installed Python remote_execution client.

No external Python packages or global Codex settings are needed. Commands run
with the editor user's privileges. Only use trusted project editor scripts.
"""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import uuid

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "Scripts/Editor"
OUTPUT = ROOT / "Saved/Automation"


def engine_root(override=None):
    if override:
        return Path(override).resolve()
    project = json.loads((ROOT / "PlanetGenerator.uproject").read_text(encoding="utf-8-sig"))
    manifest = Path(os.environ["PROGRAMDATA"]) / "Epic/UnrealEngineLauncher/LauncherInstalled.dat"
    entries = json.loads(manifest.read_text(encoding="utf-8-sig"))["InstallationList"]
    matches = [e for e in entries if e["AppName"] == "UE_" + project["EngineAssociation"]]
    if len(matches) != 1:
        raise RuntimeError("Specify --engine-root for this EngineAssociation.")
    return Path(matches[0]["InstallLocation"]).resolve()


def load_client(engine):
    source = engine / "Engine/Plugins/Experimental/PythonScriptPlugin/Content/Python/remote_execution.py"
    spec = importlib.util.spec_from_file_location("epic_remote_execution", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class EditorSession:
    def __init__(self, engine=None, timeout=30):
        self.engine = engine_root(engine)
        self.api = load_client(self.engine)
        config = self.api.RemoteExecutionConfig()
        config.multicast_bind_address = "127.0.0.1"
        config.multicast_ttl = 0
        config.command_endpoint = ("127.0.0.1", 6776)
        self.remote = self.api.RemoteExecution(config)
        self.timeout = timeout
        self.node = None

    def __enter__(self):
        try:
            self.remote.start()
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                matches = [n for n in self.remote.remote_nodes
                           if n.get("project_root") and Path(n["project_root"]).resolve() == ROOT]
                if len(matches) > 1:
                    raise RuntimeError("Multiple editors for this project; close the duplicate manually.")
                if matches:
                    self.node = matches[0]
                    break
                time.sleep(0.2)
            if not self.node:
                raise TimeoutError("This project was not discovered. Check Start-Editor.ps1 and the latest UE log.")
            self.remote.open_command_connection(self.node["node_id"])
            # Epic 5.8 exposes no public command timeout. Limit its blocking socket.
            self.remote._command_connection._command_channel_socket.settimeout(self.timeout)
            return self
        except BaseException:
            self.remote.stop()
            raise

    def __exit__(self, *_):
        self.remote.stop()

    def run(self, action="status", args=None, script=None):
        request_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8]
        result_path = OUTPUT / (request_id + ".json")
        try:
            return self._run(action, args, script, result_path)
        finally:
            result_path.unlink(missing_ok=True)

    def _run(self, action, args, script, result_path):
        target = (Path(script) if script else SCRIPTS / "automation_actions.py").resolve()
        if not target.is_relative_to(SCRIPTS.resolve()) or target.suffix.lower() != ".py":
            raise ValueError("Editor scripts must be .py files inside Scripts/Editor.")
        if not target.is_file():
            raise FileNotFoundError(target)
        OUTPUT.mkdir(parents=True, exist_ok=True)
        parameters = dict(args or {})
        parameters["action"] = action
        # Python literals are sent directly over Epic's protocol, never through a shell.
        command = (
            "import json, pathlib, runpy, unreal\n"
            f"assert pathlib.Path(unreal.Paths.project_dir()).resolve() == pathlib.Path({str(ROOT)!r})\n"
            f"_pg_result = runpy.run_path({str(target)!r}, init_globals={{'ARGS': {parameters!r}}}).get('RESULT')\n"
            f"pathlib.Path({str(result_path)!r}).write_text(json.dumps(_pg_result, ensure_ascii=False, indent=2), encoding='utf-8')\n"
        )
        if not self.remote.has_command_connection():
            self.remote.open_command_connection(self.node["node_id"])
            self.remote._command_connection._command_channel_socket.settimeout(self.timeout)
        response = self.remote.run_command(command, unattended=True, exec_mode=self.api.MODE_EXEC_FILE)
        # Release the command channel before waiting for subsequent viewport frames.
        self.remote.close_command_connection()
        if not response.get("success"):
            error = {"ok": False, "action": action, "response": response}
            result_path.write_text(json.dumps(error, ensure_ascii=False, indent=2), encoding="utf-8")
            raise RuntimeError(f"UE command failed\n{json.dumps(response, ensure_ascii=False)}")
        if not result_path.is_file():
            raise RuntimeError("UE returned without producing the expected result file.")
        result = json.loads(result_path.read_text(encoding="utf-8"))
        report = {"ok": True, "action": action, "project": str(ROOT), "engine_version": self.node.get("engine_version"),
                  "result": result, "temporary_result_removed_after_read": True, "completed_at": datetime.now(timezone.utc).isoformat()}
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["status", "smoke", "camera", "play", "stop", "script"])
    parser.add_argument("--script", help="Trusted Python file inside Scripts/Editor; set RESULT to JSON-compatible data.")
    parser.add_argument("--args-file", type=Path, help="JSON object exposed to the script as ARGS.")
    parser.add_argument("--engine-root")
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    if args.action == "script" and not args.script:
        parser.error("script requires --script")
    parameters = json.loads(args.args_file.read_text(encoding="utf-8-sig")) if args.args_file else {}
    if not isinstance(parameters, dict):
        parser.error("--args-file must contain a JSON object")
    with EditorSession(args.engine_root, args.timeout) as session:
        result = session.run(args.action, parameters, args.script)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    try:
        main()
    except Exception as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
