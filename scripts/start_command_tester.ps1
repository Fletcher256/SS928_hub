$ErrorActionPreference = 'Stop'

$Root = Resolve-Path (Join-Path $PSScriptRoot '..')
$Tool = Join-Path $Root 'py/stm32_command_tester.py'

Push-Location $Root
try {
  & python $Tool
} finally {
  Pop-Location
}
