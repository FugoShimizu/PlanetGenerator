[CmdletBinding()]
param(
	[int]$EditorProcessId = 0,
	[int]$ClickX = -1,
	[int]$ClickY = -1,
	[ValidateSet('None', 'W', 'A', 'S', 'D', 'Space', 'Escape', 'F8', 'F11')][string]$Key = 'None',
	[ValidateRange(1,2000)][int]$HoldMilliseconds = 100
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
if(-not $EditorProcessId) {
	$launch = Get-Content -LiteralPath (Join-Path $projectRoot 'Saved\Automation\last-launch.json') -Raw -Encoding UTF8 |
	ConvertFrom-Json
	$EditorProcessId = $launch.ProcessId
}
$info = Get-CimInstance Win32_Process -Filter "ProcessId = $EditorProcessId"
$projectFile = Join-Path $projectRoot 'PlanetGenerator.uproject'
if(-not $info -or $info.Name -ne 'UnrealEditor.exe' -or -not $info.CommandLine -or
	$info.CommandLine.IndexOf($projectFile, [StringComparison]::OrdinalIgnoreCase) -lt 0) {
	throw 'The target is not this project editor.'
}
if(($ClickX -ge 0) -ne ($ClickY -ge 0)) { throw 'Specify both ClickX and ClickY.' }
$inputTemp = Join-Path $projectRoot 'Saved\Automation\CompilerTemp'
New-Item -ItemType Directory -Path $inputTemp -Force | Out-Null
$previousTemp = $env:TEMP
$previousTmp = $env:TMP
$env:TEMP = $inputTemp
$env:TMP = $inputTemp
try {
	if(-not ('PlanetEditorInput' -as [type])) {
		Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;

public static class PlanetEditorInput {
	[StructLayout(LayoutKind.Sequential)] public struct Rect {
		public int Left;
		public int Top;
		public int Right;
		public int Bottom;
	}

	[StructLayout(LayoutKind.Sequential)] public struct Point {
		public int X;
		public int Y;
	}

	[StructLayout(LayoutKind.Sequential)] struct Mouse {
		public int Dx;
		public int Dy;
		public uint Data;
		public uint Flags;
		public uint Time;
		public UIntPtr Extra;
	}

	[StructLayout(LayoutKind.Sequential)] struct Keyboard {
		public ushort VirtualKey;
		public ushort ScanCode;
		public uint Flags;
		public uint Time;
		public UIntPtr Extra;
	}

	[StructLayout(LayoutKind.Explicit)] struct InputData {
		[FieldOffset(0)] public Mouse Mouse;
		[FieldOffset(0)] public Keyboard Keyboard;
	}

	[StructLayout(LayoutKind.Sequential)] struct Input {
		public uint Type;
		public InputData Data;
	}

	[DllImport("user32.dll")] static extern uint SendInput(uint Count, Input[] Values, int Size);
	[DllImport("user32.dll")] static extern uint MapVirtualKey(uint Code, uint MapType);
	[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr Window, out Rect Bounds);
	[DllImport("user32.dll")] public static extern bool ShowWindowAsync(IntPtr Window, int State);
	[DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr Window);
	[DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
	[DllImport("user32.dll")] public static extern bool SetCursorPos(int X, int Y);
	[DllImport("user32.dll")] public static extern IntPtr WindowFromPoint(Point Position);
	[DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr Window, out uint Process);
	[DllImport("user32.dll")] public static extern bool SetProcessDpiAwarenessContext(IntPtr Context);

	/// <summary>左右の修飾キーを識別する物理コードを添えて，キー状態を送信する</summary>
	/// <param name="Key">Windows の仮想キー番号</param>
	/// <param name="IsDown">押下の場合は真，解放の場合は偽</param>
	/// <returns>送信出来た場合に真</returns>
	public static bool Key(ushort Key, bool IsDown) {
		Input Value = new Input { Type = 1 };
		Value.Data.Keyboard.VirtualKey = Key;
		Value.Data.Keyboard.ScanCode = (ushort)MapVirtualKey(Key, 0);
		Value.Data.Keyboard.Flags = IsDown ? 0U : 2U;
		// 送信結果の返戻 (SendInput)
		return SendInput(1, new[] { Value }, Marshal.SizeOf(typeof(Input))) == 1;
	}

	/// <summary>左ボタンの押下と解放を送信する</summary>
	/// <returns>両方を送信出来た場合に真</returns>
	public static bool Click() {
		Input Down = new Input { Type = 0 }, Up = new Input { Type = 0 };
		Down.Data.Mouse.Flags = 2;
		Up.Data.Mouse.Flags = 4;
		// 両イベントの送信結果の返戻 (SendInput)
		return SendInput(2, new[] { Down, Up }, Marshal.SizeOf(typeof(Input))) == 2;
	}
}
'@
	}
} finally {
	$env:TEMP = $previousTemp
	$env:TMP = $previousTmp
}
[PlanetEditorInput]::SetProcessDpiAwarenessContext([IntPtr](-4)) | Out-Null
$windowOutput = & python -B -X utf8 (Join-Path $PSScriptRoot 'find_editor_window.py') `
	--process-id $EditorProcessId --project-name ([IO.Path]::GetFileNameWithoutExtension($projectFile))
if($LASTEXITCODE -ne 0) { throw 'Could not identify the project editor window.' }
$windowInfo = $windowOutput | ConvertFrom-Json
$window = [IntPtr]([long]$windowInfo.handle)
if($window -eq [IntPtr]::Zero) { throw 'UE has no main window.' }
[PlanetEditorInput]::ShowWindowAsync($window, 9) | Out-Null
[PlanetEditorInput]::SetForegroundWindow($window) | Out-Null
Start-Sleep -Milliseconds 150
if([PlanetEditorInput]::GetForegroundWindow() -ne $window) { throw 'Could not focus UE; no input sent.' }
if($ClickX -ge 0) {
	$rect = New-Object PlanetEditorInput+Rect
	if(-not [PlanetEditorInput]::GetWindowRect($window, [ref]$rect)) { throw 'Could not read UE bounds.' }
	if($ClickX -ge ($rect.Right - $rect.Left) -or $ClickY -ge ($rect.Bottom - $rect.Top)) { throw 'Click is outside UE.' }
	$point = New-Object PlanetEditorInput+Point
	$point.X = $rect.Left + $ClickX
	$point.Y = $rect.Top + $ClickY
	$hitWindow = [PlanetEditorInput]::WindowFromPoint($point)
	[uint32]$hitProcess = 0
	[PlanetEditorInput]::GetWindowThreadProcessId($hitWindow, [ref]$hitProcess) | Out-Null
	if($hitProcess -ne $EditorProcessId) { throw 'Click target is covered by another application; no input sent.' }
	if(-not [PlanetEditorInput]::SetCursorPos($point.X, $point.Y)) { throw 'Cursor move failed.' }
	if(-not [PlanetEditorInput]::Click()) { throw 'Click injection failed.' }
	Start-Sleep -Milliseconds 100
}
if($Key -ne 'None') {
	$keyCodes = @{ W = 0x57; A = 0x41; S = 0x53; D = 0x44; Space = 0x20; Escape = 0x1B; F8 = 0x77; F11 = 0x7A }
	if([PlanetEditorInput]::GetForegroundWindow() -ne $window) { throw 'UE lost focus; key not sent.' }
	try {
		if(-not [PlanetEditorInput]::Key($keyCodes[$Key], $true)) { throw 'Key down failed.' }
		$timer = [Diagnostics.Stopwatch]::StartNew()
		while($timer.ElapsedMilliseconds -lt $HoldMilliseconds) {
			if([PlanetEditorInput]::GetForegroundWindow() -ne $window) { throw 'UE lost focus while holding key.' }
			Start-Sleep -Milliseconds 20
		}
	} finally {
		[PlanetEditorInput]::Key($keyCodes[$Key], $false) | Out-Null
	}
}
[pscustomobject]@{
	ProcessId = $EditorProcessId
	ClickX = $ClickX
	ClickY = $ClickY
	Key = $Key
	HoldMilliseconds = $HoldMilliseconds
	InputSent = $true
} | ConvertTo-Json
