[CmdletBinding()]
param(
	[ValidateSet('W','A','S','D','One','Two','Three','Space')][string]$Key = 'W',
	[switch]$Run,
	[ValidateRange(1,2000)][int]$HoldMilliseconds = 100
)
$ErrorActionPreference = 'Stop'

# 接続先・最前面・操作窓の検査は既存の Send-EditorInput.ps1
& (Join-Path $PSScriptRoot 'Send-EditorInput.ps1') -Key None | Out-Null
$InputType = 'PlanetEditorInput' -as [type]
if(-not $InputType) { throw 'The verified editor input helper did not load.' }
$ProjectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$Launch = Get-Content -LiteralPath (Join-Path $ProjectRoot 'Saved/Automation/last-launch.json') -Raw -Encoding UTF8 |
	ConvertFrom-Json
$Window = (Get-Process -Id $Launch.ProcessId).MainWindowHandle
if($InputType::GetForegroundWindow() -ne $Window) { throw 'UE lost focus; no input sent.' }
$KeyCodes = @{ W = 0x57; A = 0x41; S = 0x53; D = 0x44; One = 0x31; Two = 0x32; Three = 0x33; Space = 0x20 }
try {
	if($Run -and -not $InputType::Key(0xA0, $true)) { throw 'Shift key down failed.' }
	if(-not $InputType::Key($KeyCodes[$Key], $true)) { throw 'Key down failed.' }
	$Timer = [Diagnostics.Stopwatch]::StartNew()
	while($Timer.ElapsedMilliseconds -lt $HoldMilliseconds) {
		if($InputType::GetForegroundWindow() -ne $Window) { throw 'UE lost focus while holding the key.' }
		Start-Sleep -Milliseconds 20
	}
} finally {
	$InputType::Key($KeyCodes[$Key], $false) | Out-Null
	if($Run) { $InputType::Key(0xA0, $false) | Out-Null }
}
[pscustomobject]@{
	processId = $Launch.ProcessId
	key = $Key
	run = [bool]$Run
	holdMilliseconds = $HoldMilliseconds
	inputSent = $true
} | ConvertTo-Json
