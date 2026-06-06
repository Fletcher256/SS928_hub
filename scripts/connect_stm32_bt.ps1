param(
  [string]$Port = '',
  [int]$Baud = 9600
)

$ErrorActionPreference = 'Stop'

$Root = Resolve-Path (Join-Path $PSScriptRoot '..')
$Tool = Join-Path $Root 'py/stm32_command_tester.py'

$Args = @($Tool, '--ping', '--baud', "$Baud")
if ($Port) {
  $Args += @('--port', $Port)
}

Push-Location $Root
try {
  & python @Args
  exit $LASTEXITCODE
} finally {
  Pop-Location
}
