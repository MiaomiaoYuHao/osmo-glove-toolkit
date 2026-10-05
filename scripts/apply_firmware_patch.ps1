param(
    [Parameter(Mandatory = $true)]
    [string]$RepoPath,

    [string]$Patch = (Join-Path $PSScriptRoot '..\firmware\patches\BowieGlove-magnet-stability.patch')
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path -LiteralPath $RepoPath).Path
$patchPath = (Resolve-Path -LiteralPath $Patch).Path
if (-not (Test-Path -LiteralPath (Join-Path $repo '.git'))) {
    throw "Not a Git repository: $repo"
}

git -C $repo apply --check $patchPath
if ($LASTEXITCODE -ne 0) {
    throw "The patch does not apply cleanly. Check out upstream commit bfc7328 first."
}
git -C $repo apply $patchPath
if ($LASTEXITCODE -ne 0) {
    throw "Failed to apply firmware patch."
}
Write-Host "Firmware patch applied successfully: $patchPath"