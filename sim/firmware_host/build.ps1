$ErrorActionPreference = 'Stop'

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$BuildDir = Join-Path $RepoRoot 'build\host'
$ToolsDir = Join-Path $RepoRoot 'tools'
$DownloadsDir = Join-Path $ToolsDir 'downloads'
$TccDir = Join-Path $ToolsDir 'tcc'
$TccExe = Join-Path $TccDir 'tcc\tcc.exe'

New-Item -ItemType Directory -Force -Path $BuildDir, $DownloadsDir | Out-Null

if (!(Test-Path $TccExe)) {
  Write-Host 'Downloading portable TinyCC compiler...'
  $archive = Join-Path $DownloadsDir 'tcc-0.9.27-win64-bin.zip'
  $urls = @(
    'https://mirror.accum.se/mirror/gnu.org/savannah/tinycc/tcc-0.9.27-win64-bin.zip',
    'http://fosszone.csd.auth.gr/nongnu/tinycc/tcc-0.9.27-win64-bin.zip',
    'https://download.savannah.gnu.org/releases/tinycc/tcc-0.9.27-win64-bin.zip'
  )
  if (!(Test-Path $archive)) {
    $curl = Get-Command curl.exe -ErrorAction SilentlyContinue
    foreach ($url in $urls) {
      try {
        if ($null -ne $curl) {
          & $curl.Source -L --fail --retry 3 --output $archive $url
          if ($LASTEXITCODE -ne 0) {
            throw "curl failed with exit code $LASTEXITCODE"
          }
        } else {
          Invoke-WebRequest -Uri $url -OutFile $archive
        }
        break
      } catch {
        Remove-Item -Force $archive -ErrorAction SilentlyContinue
        if ($url -eq $urls[-1]) {
          throw
        }
      }
    }
  }

  if (!(Test-Path $archive) -or (Get-Item $archive).Length -lt 100000) {
    Remove-Item -Force $archive -ErrorAction SilentlyContinue
    throw "TinyCC archive download is incomplete: $archive"
  }

  $extractDir = Join-Path $DownloadsDir 'tcc-extract'
  Remove-Item -Recurse -Force -Path $extractDir -ErrorAction SilentlyContinue
  Expand-Archive -Path $archive -DestinationPath $extractDir -Force
  $inner = Get-ChildItem -Path $extractDir -Directory | Where-Object { Test-Path (Join-Path $_.FullName 'tcc.exe') } | Select-Object -First 1
  if ($null -eq $inner) {
    throw 'TinyCC archive did not contain tcc.exe.'
  }
  Remove-Item -Recurse -Force -Path $TccDir -ErrorAction SilentlyContinue
  New-Item -ItemType Directory -Force -Path $TccDir | Out-Null
  Move-Item -Path $inner.FullName -Destination (Join-Path $TccDir 'tcc')
}

$Output = Join-Path $BuildDir 'ss928_firmware_host.exe'
$Args = @(
  '-DSS928_HOST_SIM',
  '-D__ia64__',
  '-I', $RepoRoot,
  (Join-Path $RepoRoot 'main.c'),
  (Join-Path $RepoRoot 'sim\firmware_host\host_stubs.c'),
  (Join-Path $RepoRoot 'sim\firmware_host\firmware_host_runner.c'),
  '-o', $Output
)

& $TccExe @Args
if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}

Write-Host "Built: $Output"
