param(
    [string]$Destination = (Join-Path (Get-Location) ('build\repro-' + (Get-Date -Format 'yyyyMMdd_HHmmss'))),
    [string]$GccBin = $env:ARM_GCC_BIN,
    [string]$MakeExe = $env:MAKE_EXE
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$releaseDir = Join-Path $projectRoot 'firmware\releases'

if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    throw 'python was not found in PATH. Install Python 3.12+ first.'
}

Write-Host '[1/4] Host self-test'
python -m py_compile (Join-Path $projectRoot 'host\3D力测试上位机.pyw')
python (Join-Path $projectRoot 'host\3D力测试上位机.pyw') --self-test
if ($LASTEXITCODE -ne 0) {
    throw 'Host self-test failed.'
}

Write-Host '[2/4] Bootstrap upstream firmware'
& (Join-Path $PSScriptRoot 'bootstrap_firmware.ps1') -Destination $Destination
if ($LASTEXITCODE -ne 0) {
    throw 'Firmware bootstrap failed.'
}

Write-Host '[3/4] Build firmware'
& (Join-Path $PSScriptRoot 'build_firmware.ps1') -RepoPath $Destination -GccBin $GccBin -MakeExe $MakeExe
if ($LASTEXITCODE -ne 0) {
    throw 'Firmware build failed.'
}

Write-Host '[4/4] Compare with release hashes'
$debugDir = Join-Path $Destination 'firmware\BowieGlove\Debug'
$pairs = @(
    @{ Built = (Join-Path $debugDir 'BowieGlove.bin'); Release = (Join-Path $releaseDir 'BowieGlove_magnet_recovery.bin') },
    @{ Built = (Join-Path $debugDir 'BowieGlove.hex'); Release = (Join-Path $releaseDir 'BowieGlove_magnet_recovery.hex') }
)
foreach ($pair in $pairs) {
    $builtHash = (Get-FileHash -LiteralPath $pair.Built -Algorithm SHA256).Hash
    $releaseHash = (Get-FileHash -LiteralPath $pair.Release -Algorithm SHA256).Hash
    if ($builtHash -ne $releaseHash) {
        throw "Hash mismatch for $(Split-Path -Leaf $pair.Built): $builtHash != $releaseHash"
    }
    Write-Host "  MATCH $builtHash  $(Split-Path -Leaf $pair.Built)"
}

Write-Host ''
Write-Host 'REPRODUCTION_PASS=True'
Write-Host "Fresh source: $Destination"