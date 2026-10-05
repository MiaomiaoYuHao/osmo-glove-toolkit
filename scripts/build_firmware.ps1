param(
    [string]$RepoPath = (Join-Path (Get-Location) 'build\osmo_tactile_glove'),
    [string]$GccBin = $env:ARM_GCC_BIN,
    [string]$MakeExe = $env:MAKE_EXE
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path -LiteralPath $RepoPath).Path
if (-not (Test-Path -LiteralPath (Join-Path $repo 'firmware\BowieGlove\Debug'))) {
    throw "Firmware Debug project not found under: $repo"
}

if (-not $GccBin) {
    $GccBin = 'C:\ST\STM32CubeIDE_1.14.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.11.3.rel1.win32_1.1.100.202309141235\tools\bin'
}
if (-not $MakeExe) {
    $MakeExe = 'C:\ST\STM32CubeIDE_1.14.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.1.101.202401061624\tools\bin\make.exe'
}
if (-not (Test-Path -LiteralPath (Join-Path $GccBin 'arm-none-eabi-gcc.exe'))) {
    throw "ARM GCC not found. Set ARM_GCC_BIN to the tools\bin directory."
}
if (-not (Test-Path -LiteralPath $MakeExe)) {
    throw "make.exe not found. Set MAKE_EXE to the GNU make executable."
}

$firmwareDir = Join-Path $repo 'firmware\BowieGlove'
$debugDir = Join-Path $firmwareDir 'Debug'
& (Join-Path $PSScriptRoot 'fix_build_paths.ps1') -RepoPath $repo
$env:Path = "$GccBin;$env:Path"
& $MakeExe -C $debugDir clean
# The upstream clean target may print ignored errors on Windows. A full build below
# is the authoritative success check.
& $MakeExe -C $debugDir -j4 all
if ($LASTEXITCODE -ne 0) {
    throw 'Firmware build failed.'
}
& (Join-Path $GccBin 'arm-none-eabi-objcopy.exe') -O ihex (Join-Path $debugDir 'BowieGlove.elf') (Join-Path $debugDir 'BowieGlove.hex')
if ($LASTEXITCODE -ne 0) {
    throw 'HEX generation failed.'
}
& (Join-Path $GccBin 'arm-none-eabi-objcopy.exe') -O binary (Join-Path $debugDir 'BowieGlove.elf') (Join-Path $debugDir 'BowieGlove.bin')
if ($LASTEXITCODE -ne 0) {
    throw 'BIN generation failed.'
}
Write-Host ''
Write-Host 'Firmware build completed:'
Write-Host "  $(Join-Path $debugDir 'BowieGlove.hex')"
Write-Host "  $(Join-Path $debugDir 'BowieGlove.bin')"