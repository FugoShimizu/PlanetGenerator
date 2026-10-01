[CmdletBinding()]
param([switch]$BuildOnly, [ValidateSet('Core','Lod','Geology','All')][string]$Suite = 'All')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$locator = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vsRoot = & $locator -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsRoot) { throw 'A Visual Studio C++ toolchain is required.' }
Import-Module (Join-Path $vsRoot 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $vsRoot -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64' | Out-Null
$buildRoot = Join-Path $projectRoot 'Saved\Automation\CoreBuild'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
try {
  $suites = if ($Suite -eq 'All') { @('Core','Lod','Geology') } else { @($Suite) }
  foreach ($testSuite in $suites) {
    $executable = Join-Path $buildRoot ('Planet' + $testSuite + 'Tests.exe')
    $arguments = @('/nologo','/std:c++20','/EHsc','/O2','/W4','/WX','/fp:precise','/utf-8',
        ('/I' + (Join-Path $projectRoot 'Source\PlanetGenerator\Core')),
        (Join-Path $projectRoot 'Source\PlanetGenerator\Core\PlanetCore.cpp'),
        (Join-Path $projectRoot 'Source\PlanetGenerator\Core\PlanetLod.cpp'),
        (Join-Path $projectRoot 'Source\PlanetGenerator\Core\PlanetGeology.cpp'),
        (Join-Path $projectRoot ('Tests\Core\Planet' + $testSuite + 'Tests.cpp')),
        ('/Fe:' + $executable), ('/Fo:' + $buildRoot.Replace('\','/') + '/'), '/link','/INCREMENTAL:NO')
    & cl.exe @arguments
    if ($LASTEXITCODE -ne 0) { throw 'Core build failed.' }
    if (-not $BuildOnly) {
        & $executable
        if ($LASTEXITCODE -ne 0) { throw 'Core contract checks failed.' }
    }
  }
} finally {
    if (-not $BuildOnly -and (Test-Path -LiteralPath $buildRoot)) {
        $resolved = (Resolve-Path -LiteralPath $buildRoot).Path
        $expected = Join-Path $projectRoot 'Saved\Automation\CoreBuild'
        if ($resolved -ne $expected) { throw 'Unexpected test build path; cleanup stopped.' }
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
}
