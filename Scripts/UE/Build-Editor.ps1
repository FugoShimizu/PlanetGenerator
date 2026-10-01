[CmdletBinding()]
param([string]$EngineRoot)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$projectFile = Join-Path $projectRoot 'PlanetGenerator.uproject'
$running = @(Get-CimInstance Win32_Process -Filter "Name = 'UnrealEditor.exe'" | Where-Object {
    $_.CommandLine -and $_.CommandLine.IndexOf($projectFile, [StringComparison]::OrdinalIgnoreCase) -ge 0
})
if ($running.Count) { throw 'Close this project editor normally before compiling its DLL.' }
if (-not $EngineRoot) {
    $projectData = Get-Content -LiteralPath $projectFile -Raw -Encoding UTF8 | ConvertFrom-Json
    $manifestPath = Join-Path $env:ProgramData 'Epic\UnrealEngineLauncher\LauncherInstalled.dat'
    $manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $matches = @($manifest.InstallationList | Where-Object AppName -EQ ('UE_' + $projectData.EngineAssociation))
    if ($matches.Count -ne 1) { throw 'Specify -EngineRoot.' }
    $EngineRoot = $matches[0].InstallLocation
}
& (Join-Path $EngineRoot 'Engine\Build\BatchFiles\Build.bat') PlanetGeneratorEditor Win64 Development ('-Project=' + $projectFile) -WaitMutex -NoHotReloadFromIDE -NoUBA -UBANoDetour -UBAStoreCapacityGb=1 -MaxParallelActions=1
if ($LASTEXITCODE -ne 0) { throw "Editor build failed: $LASTEXITCODE" }
