"""UE 本体を同じプロセスの一時的な窓と区別して選ぶ"""

import argparse
import ctypes
from ctypes import wintypes
import json

def find_editor_window(process_id: int, project_name: str) -> dict[str, int|str]:
	"""指定プロセスのプロジェクト名に一致する本体を返し，未発見又は重複時は失敗する"""
	if process_id < 1 or not project_name:
		raise ValueError("A positive process identifier and project name are required")
	user32: ctypes.WinDLL = ctypes.WinDLL("user32", use_last_error=True)
	user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
	user32.GetWindowThreadProcessId.restype = wintypes.DWORD
	user32.IsWindowVisible.argtypes = [wintypes.HWND]
	user32.IsWindowVisible.restype = wintypes.BOOL
	user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
	user32.GetWindowTextW.restype = ctypes.c_int
	callback_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
	title: str = project_name + " - Unreal Editor"
	matches: list[int] = []
	@callback_type
	def collect(window: int, parameter: int) -> bool:
		"""同じプロセスで可視の本体だけを列挙し，列挙の継続を返す"""
		del parameter
		owner: wintypes.DWORD = wintypes.DWORD()
		user32.GetWindowThreadProcessId(window, ctypes.byref(owner))
		if owner.value == process_id and user32.IsWindowVisible(window):
			text: ctypes.Array = ctypes.create_unicode_buffer(512)
			user32.GetWindowTextW(window, text, len(text))
			if text.value == title:
				matches.append(window)
		# 列挙を継続する返戻
		return True
	user32.EnumWindows.argtypes = [callback_type, wintypes.LPARAM]
	user32.EnumWindows.restype = wintypes.BOOL
	if not user32.EnumWindows(collect, 0):
		raise ctypes.WinError(ctypes.get_last_error())
	if len(matches) != 1:
		raise RuntimeError(f"Expected one editor window for process {process_id}, found {len(matches)}")
	# 一意に確認した本体のハンドルと題名の返戻
	return { "handle": matches[0], "title": title }

def main() -> None:
	"""検証済の窓を PowerShell の入力・撮影処理へ返す"""
	parser: argparse.ArgumentParser = argparse.ArgumentParser()
	parser.add_argument("--process-id", type=int, required=True)
	parser.add_argument("--project-name", required=True)
	args: argparse.Namespace = parser.parse_args()
	print(json.dumps(find_editor_window(args.process_id, args.project_name), ensure_ascii=False))
	# 終了
	return

if __name__ == "__main__":
	main()
