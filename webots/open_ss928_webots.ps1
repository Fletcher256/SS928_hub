$ErrorActionPreference = 'Stop'

$Webots = 'D:\Program Files\Webots\msys64\mingw64\bin\webots.exe'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$World = Join-Path $RepoRoot 'webots\worlds\ss928_car.wbt'

if (!(Test-Path $Webots)) {
  throw "Webots executable not found: $Webots"
}

if (!(Test-Path $World)) {
  throw "World file not found: $World"
}

Start-Process -FilePath $Webots -ArgumentList @('--mode=realtime', "`"$World`"") -WorkingDirectory $RepoRoot
