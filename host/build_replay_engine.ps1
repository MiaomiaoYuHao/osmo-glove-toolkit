$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$fw = Join-Path $root '..\_osmo_glove_research\firmware\BowieGlove_Attitude'
$gcc = (Get-Command gcc -ErrorAction Stop).Source
& $gcc -std=c11 -O2 -Wall -Wextra `
    -I (Join-Path $fw 'Core\Inc\glove') `
    -o (Join-Path $root 'replay_engine.exe') `
    (Join-Path $root 'replay_engine.c') `
    (Join-Path $fw 'Core\Src\glove\mag_fusion.c') `
    -lm
if ($LASTEXITCODE -ne 0) { throw "gcc failed: $LASTEXITCODE" }
Write-Host "BUILT=$(Join-Path $root 'replay_engine.exe')"
