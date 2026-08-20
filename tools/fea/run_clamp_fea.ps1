param(
    [double]$Load = 800,
    [double]$MeshSize = 3,
    [ValidateSet("design_d", "legacy_u", "legacy_u_nogusset")]
    [string]$Model = "design_d",
    [ValidateSet("al6061", "petg")]
    [string]$Material = "al6061",
    [double]$Eccentricity = 0,
    [ValidateSet("downward", "upward")]
    [string]$LoadDirection = "downward"
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$scriptPath = Join-Path $repoRoot "models\hoverboard_mount\run_clamp_fea.py"

$pythonCandidates = @(
    (Join-Path $repoRoot ".venv\Scripts\python.exe"),
    "C:\Users\<user>\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
)

$pythonExe = $null
foreach ($candidate in $pythonCandidates) {
    if (-not (Test-Path -LiteralPath $candidate)) {
        continue
    }
    try {
        & $candidate -c "import sys; print(sys.version_info[:2])" *> $null
        if ($LASTEXITCODE -eq 0) {
            $pythonExe = $candidate
            break
        }
    }
    catch {
        continue
    }
}

if (-not $pythonExe) {
    throw "Usable Python 3.12 not found. Recreate .venv as described in SETUP.md."
}

$venvPackages = Join-Path $repoRoot ".venv\Lib\site-packages"
$env:PYTHONPATH = $venvPackages

Write-Host "Python: $pythonExe"
Write-Host "FEA: model=$Model, material=$Material, load=$Load N $LoadDirection, eccentricity=$Eccentricity mm, target mesh=$MeshSize mm"
& $pythonExe $scriptPath --model $Model --material $Material --load $Load --load-direction $LoadDirection --eccentricity $Eccentricity --mesh-size $MeshSize
exit $LASTEXITCODE
