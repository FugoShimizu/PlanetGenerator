[CmdletBinding()]
param([int]$EditorProcessId = 0)
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
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
	throw 'The process is not this project editor.'
}
Add-Type -AssemblyName System.Drawing
if(-not ('PlanetEditorCapture' -as [type])) {
	$captureTemp = Join-Path $projectRoot 'Saved\Automation\CompilerTemp'
	New-Item -ItemType Directory -Path $captureTemp -Force | Out-Null
	$previousTemp = $env:TEMP
	$previousTmp = $env:TMP
	$env:TEMP = $captureTemp
	$env:TMP = $captureTemp
	try {
		Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;

public static class PlanetEditorCapture {
	[StructLayout(LayoutKind.Sequential)] public struct Rect {
		public int Left;
		public int Top;
		public int Right;
		public int Bottom;
	}

	[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr Window, out Rect Bounds);
	[DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr Window, IntPtr DeviceContext, uint Flags);
	[DllImport("user32.dll")] public static extern bool IsIconic(IntPtr Window);
	[DllImport("user32.dll")] public static extern bool SetProcessDpiAwarenessContext(IntPtr Context);
}
'@
	} finally {
		$env:TEMP = $previousTemp
		$env:TMP = $previousTmp
	}
}
[PlanetEditorCapture]::SetProcessDpiAwarenessContext([IntPtr](-4)) | Out-Null
$editorProcess = Get-Process -Id $EditorProcessId
$windowOutput = & python -B -X utf8 (Join-Path $PSScriptRoot 'find_editor_window.py') `
	--process-id $EditorProcessId --project-name ([IO.Path]::GetFileNameWithoutExtension($projectFile))
if($LASTEXITCODE -ne 0) { throw 'Could not identify the project editor window.' }
$windowInfo = $windowOutput | ConvertFrom-Json
$window = [IntPtr]([long]$windowInfo.handle)
if($window -eq [IntPtr]::Zero) { throw 'UE has no main window yet.' }
if ([PlanetEditorCapture]::IsIconic($window)) { throw 'Restore the editor window before capturing it.' }
$rect = New-Object PlanetEditorCapture+Rect
if(-not [PlanetEditorCapture]::GetWindowRect($window, [ref]$rect)) { throw 'Could not read the UE window rectangle.' }
$width = $rect.Right - $rect.Left
$height = $rect.Bottom - $rect.Top
if($width -le 0 -or $height -le 0 -or $width -gt 8192 -or $height -gt 8192) { throw 'Invalid window dimensions.' }
$bitmap = New-Object System.Drawing.Bitmap($width, $height)
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
$deviceContext = $graphics.GetHdc()
try {
	if(-not [PlanetEditorCapture]::PrintWindow($window, $deviceContext, 2)) { throw 'UE window capture failed.' }
} finally {
	$graphics.ReleaseHdc($deviceContext)
	$graphics.Dispose()
}
$preview = $null
$buffer = New-Object IO.MemoryStream
try {
	$previewWidth = [Math]::Min(1280, $width)
	$previewHeight = [int][Math]::Round($height * $previewWidth / $width)
	$preview = New-Object System.Drawing.Bitmap($bitmap, $previewWidth, $previewHeight)
	$preview.Save($buffer, [System.Drawing.Imaging.ImageFormat]::Jpeg)
	[pscustomobject]@{
		ImageBase64=[Convert]::ToBase64String($buffer.ToArray()); MimeType='image/jpeg';
		Width=$previewWidth; Height=$previewHeight; WindowWidth=$width; WindowHeight=$height;
		ProcessId=$EditorProcessId; Title=$windowInfo.title; Method='PrintWindow'; SavedToDisk=$false
	} | ConvertTo-Json -Compress
} finally {
	if($preview) { $preview.Dispose() }
	$buffer.Dispose()
	$bitmap.Dispose()
}
