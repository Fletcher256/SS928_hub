param(
  [string]$Scenario = 'sim\firmware_host\scenarios\basic_control.txt',
  [switch]$Quiet
)

$ErrorActionPreference = 'Stop'

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
& powershell -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'build.ps1')

$Exe = Join-Path $RepoRoot 'build\host\ss928_firmware_host.exe'
$ScenarioPath = if ([System.IO.Path]::IsPathRooted($Scenario)) {
  $Scenario
} else {
  Join-Path $RepoRoot $Scenario
}

if ($Quiet) {
  & $Exe $ScenarioPath --quiet
} else {
  & $Exe $ScenarioPath
}

if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}
