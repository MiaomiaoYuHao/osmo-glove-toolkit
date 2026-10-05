param(
    [string]$Destination = (Join-Path (Get-Location) 'build\osmo_tactile_glove'),
    [string]$Upstream = 'https://github.com/jessicayin/osmo_tactile_glove.git',
    [string]$BaseCommit = 'bfc7328'
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$patchPath = Join-Path $projectRoot 'firmware\force\patches\BowieGlove-magnet-stability.patch'

if (Test-Path -LiteralPath $Destination) {
    throw "Destination already exists: $Destination. Choose a new empty path."
}

$parent = Split-Path -Parent $Destination
if (-not (Test-Path -LiteralPath $parent)) {
    New-Item -ItemType Directory -Force -Path $parent | Out-Null
}

git clone $Upstream $Destination
if ($LASTEXITCODE -ne 0) {
    throw 'Failed to clone the upstream repository.'
}
git -C $Destination checkout $BaseCommit
if ($LASTEXITCODE -ne 0) {
    throw "Failed to check out upstream commit $BaseCommit."
}
& (Join-Path $PSScriptRoot 'apply_firmware_patch.ps1') -RepoPath $Destination -Patch $patchPath
Write-Host ''
Write-Host "Reproducible firmware source is ready at:"
Write-Host "  $Destination"
Write-Host 'Open it with STM32CubeIDE or run scripts\build_firmware.ps1.'