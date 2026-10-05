param(
    [string]$Firmware = (Join-Path $PSScriptRoot '..\firmware\releases\force\BowieGlove_magnet_recovery.bin'),
    [string]$Programmer = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
)

$ErrorActionPreference = 'Stop'
$bin = (Resolve-Path -LiteralPath $Firmware).Path
if (-not (Test-Path -LiteralPath $Programmer)) {
    throw "STM32CubeProgrammer CLI not found: $Programmer"
}
if ([IO.Path]::GetExtension($bin) -ne '.bin') {
    throw 'This helper flashes a BIN image at 0x08000000. Use STM32CubeProgrammer directly for HEX.'
}

& $Programmer -c port=SWD freq=1000 mode=HOTPLUG -w $bin 0x08000000 -v -rst
if ($LASTEXITCODE -ne 0) {
    throw 'Firmware flashing failed.'
}
Write-Host 'Firmware flashed and target reset.'
