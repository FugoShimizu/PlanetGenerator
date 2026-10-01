[CmdletBinding()]
param(
	[string]$EngineRoot,
	[ValidateSet('LowMemory', 'Project')][string]$Profile = 'LowMemory'
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$projectFile = Join-Path $projectRoot 'PlanetGenerator.uproject'
$activeBuilds = @(Get-CimInstance Win32_Process -Filter "Name = 'dotnet.exe' OR Name = 'UnrealBuildTool.exe'" | Where-Object {
	$_.CommandLine -and $_.CommandLine.IndexOf('UnrealBuildTool', [StringComparison]::OrdinalIgnoreCase) -ge 0 -and
	$_.CommandLine.IndexOf($projectFile, [StringComparison]::OrdinalIgnoreCase) -ge 0
})
if ($activeBuilds.Count -gt 0) {
	throw 'This project is still compiling. Wait for build completion before starting the editor.'
}
$projectData = Get-Content -LiteralPath $projectFile -Raw -Encoding UTF8 | ConvertFrom-Json
if (-not $EngineRoot) {
	$manifestPath = Join-Path $env:ProgramData 'Epic\UnrealEngineLauncher\LauncherInstalled.dat'
	$manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
	$entry = @($manifest.InstallationList | Where-Object AppName -EQ ('UE_' + $projectData.EngineAssociation))
	if ($entry.Count -ne 1) { throw 'Specify -EngineRoot for this EngineAssociation.' }
	$EngineRoot = $entry[0].InstallLocation
}
$editorExe = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
if (-not (Test-Path -LiteralPath $editorExe -PathType Leaf)) { throw "Editor not found: $editorExe" }
$existing = @(Get-CimInstance Win32_Process -Filter "Name = 'UnrealEditor.exe'" | Where-Object {
	$_.CommandLine -and ($_.CommandLine.IndexOf($projectFile, [StringComparison]::OrdinalIgnoreCase) -ge 0)
})
if ($existing.Count -gt 0) {
	[pscustomobject]@{ Status = 'AlreadyRunning'; ProcessIds = @($existing.ProcessId); Project = $projectFile } | ConvertTo-Json
	exit 0
}
$logDir = Join-Path $projectRoot 'Saved\Logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$logPath = Join-Path $logDir ('PlanetGenerator-Automation-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.log')
$editorArguments = @(('"' + $projectFile + '"'), '-EnablePython', ('-abslog="' + $logPath + '"'))
if ($Profile -eq 'LowMemory') {
	$groups = @(
		'ViewDistance', 'AntiAliasing', 'Shadow', 'GlobalIllumination', 'Reflection', 'PostProcess',
		'Texture', 'Effects', 'Foliage', 'Shading', 'Landscape'
	)
	# 素材の細部と控えめな影を保ち，テクスチャの予算を固定する：600 MB
	$quality = @('[ScalabilityGroups]:sg.ResolutionQuality=75') + @($groups | ForEach-Object {
		$level = if ($_ -in @('Texture', 'Shadow')) { 1 } else { 0 }
		'[ScalabilityGroups]:sg.' + $_ + 'Quality=' + $level
	})
	$editorArguments += '-ini:EditorSettings:' + ($quality -join ',')
	$variables = @(
		'[ConsoleVariables]:r.ScreenPercentage=75', '[ConsoleVariables]:t.MaxFPS=30',
		'[ConsoleVariables]:r.Streaming.UsePerTextureBias=1'
	)
	$editorArguments += '-ini:Engine:' + ($variables -join ',')
	# 利用者設定の初期適用後に実行し，起動途中の無制限への上書きを避ける
	$editorArguments += '-ExecCmds="t.MaxFPS 30"'
}
# 利用者が操作する UE 画面を起動する：WindowStyle Normal
$launch = @{ FilePath = $editorExe; ArgumentList = $editorArguments; WorkingDirectory = $projectRoot; WindowStyle = 'Normal' }
$editorProcess = Start-Process @launch -PassThru
$record = [pscustomobject]@{
	Status = 'Started'; ProcessId = $editorProcess.Id; Profile = $Profile; Project = $projectFile
	EngineRoot = $EngineRoot; Log = $logPath; StartedAt = (Get-Date).ToString('o')
}
$outDir = Join-Path $projectRoot 'Saved\Automation'
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
$record | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $outDir 'last-launch.json') -Encoding UTF8
$record | ConvertTo-Json
