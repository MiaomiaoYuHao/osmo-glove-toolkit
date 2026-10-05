param(
    [Parameter(Mandatory = $true)]
    [string]$RepoPath
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path -LiteralPath $RepoPath).Path
$firmwareDir = Join-Path $repo 'firmware\BowieGlove'
$debugDir = Join-Path $firmwareDir 'Debug'
if (-not (Test-Path -LiteralPath $debugDir)) {
    throw "Debug build directory not found: $debugDir"
}

$oldPaths = @(
    'C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove',
    'C:\Users\kundusayantan\Documents\GitHub\GUM\hardware\glove\fw\BowieGlove'
)
$localForward = ((Resolve-Path -LiteralPath $firmwareDir).Path -replace '\\', '/')
$changed = 0
Get-ChildItem -LiteralPath $debugDir -Recurse -File | Where-Object { $_.Name -eq 'makefile' -or $_.Extension -in '.mk', '.list' } | ForEach-Object {
    $text = [IO.File]::ReadAllText($_.FullName)
    $updated = $text
    foreach ($old in $oldPaths) {
        $updated = $updated.Replace($old, $localForward)
    }
    if ($updated -ne $text) {
        [IO.File]::WriteAllText($_.FullName, $updated, [Text.UTF8Encoding]::new($false))
        $changed++
    }
}
Write-Host "Updated portable include paths in $changed makefile(s)."