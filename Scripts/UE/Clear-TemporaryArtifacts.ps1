[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$automationRoot = Join-Path $projectRoot 'Saved\Automation'
$freedBytes = [long]0
$deletedCount = 0
if (Test-Path -LiteralPath $automationRoot) {
    $files = @(Get-ChildItem -LiteralPath $automationRoot -File | Where-Object {
        $_.Extension -in '.png','.jpg','.jpeg' -or
        $_.Name -match '^20\d{6}T\d{6}Z_[0-9a-f]{8}\.json$' -or
        $_.Name -in 'restore-camera.json','finalize_setup_docs.py'
    })
    foreach ($file in $files) {
        $resolved = (Resolve-Path -LiteralPath $file.FullName).Path
        if (-not $resolved.StartsWith($automationRoot + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Invalid cleanup path.' }
        $freedBytes += $file.Length
        Remove-Item -LiteralPath $resolved -Force
        $deletedCount++
    }
}
$folders = @('Saved\Automation\CompilerTemp','Saved\Automation\SetupBackup','Scripts\UE\__pycache__','Scripts\Editor\__pycache__')
foreach ($relative in $folders) {
    $folder = Join-Path $projectRoot $relative
    if (Test-Path -LiteralPath $folder) {
        $resolved = (Resolve-Path -LiteralPath $folder).Path
        if (-not $resolved.StartsWith($projectRoot + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Invalid cleanup folder.' }
        $freedBytes += [long]((Get-ChildItem -LiteralPath $resolved -File -Recurse | Measure-Object Length -Sum).Sum)
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
}
[pscustomobject]@{ DeletedFiles=$deletedCount; FreedBytes=$freedBytes } | ConvertTo-Json -Compress
