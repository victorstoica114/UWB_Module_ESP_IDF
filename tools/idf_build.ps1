[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$BuildDir = "build",

    [Parameter(Position = 1, ValueFromRemainingArguments = $true)]
    [string[]]$IdfArgs
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
$requiredIdfVersion = "6.0.2"
if (-not $IdfArgs -or $IdfArgs.Count -eq 0) {
    $IdfArgs = @("build")
}

if ([IO.Path]::IsPathRooted($BuildDir)) {
    $resolvedBuildDir = [IO.Path]::GetFullPath($BuildDir)
} else {
    $resolvedBuildDir = [IO.Path]::GetFullPath((Join-Path $repoRoot $BuildDir))
}

function Find-EspIdfPath {
    $candidates = [Collections.Generic.List[string]]::new()

    # Prefer the validated installation.  IDF_PATH is often left behind by an
    # older ESP-IDF shell and must not silently downgrade this project.
    $candidates.Add("C:\esp\v$requiredIdfVersion\esp-idf")

    if ($env:IDF_PATH) {
        $candidates.Add($env:IDF_PATH)
    }

    $descriptionPath = Join-Path $resolvedBuildDir "project_description.json"
    if (Test-Path -LiteralPath $descriptionPath) {
        try {
            $description = Get-Content -LiteralPath $descriptionPath -Raw |
                ConvertFrom-Json
            if ($description.idf_path) {
                $candidates.Add([string]$description.idf_path)
            }
        } catch {
            Write-Warning "Ignoring unreadable $descriptionPath"
        }
    }

    $espRoot = "C:\esp"
    if (Test-Path -LiteralPath $espRoot) {
        Get-ChildItem -LiteralPath $espRoot -Directory -ErrorAction SilentlyContinue |
            Sort-Object Name -Descending |
            ForEach-Object {
                $candidates.Add((Join-Path $_.FullName "esp-idf"))
            }
    }

    foreach ($candidate in $candidates) {
        if (-not $candidate) {
            continue
        }
        $exportScript = Join-Path $candidate "export.ps1"
        $versionScript = Join-Path $candidate "tools\cmake\version.cmake"
        if (-not (Test-Path -LiteralPath $exportScript) -or
            -not (Test-Path -LiteralPath $versionScript)) {
            continue
        }

        $versionText = Get-Content -LiteralPath $versionScript -Raw
        $major = [regex]::Match($versionText,
            'set\(IDF_VERSION_MAJOR\s+(\d+)\)').Groups[1].Value
        $minor = [regex]::Match($versionText,
            'set\(IDF_VERSION_MINOR\s+(\d+)\)').Groups[1].Value
        $patch = [regex]::Match($versionText,
            'set\(IDF_VERSION_PATCH\s+(\d+)\)').Groups[1].Value
        $candidateVersion = "$major.$minor.$patch"
        if ($candidateVersion -eq $requiredIdfVersion) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }

        Write-Warning "Ignoring ESP-IDF $candidateVersion at $candidate; this project requires $requiredIdfVersion."
    }

    throw "ESP-IDF $requiredIdfVersion was not found. Install it below C:\esp or set IDF_PATH to that version."
}

$idfPath = Find-EspIdfPath
$ccacheRoot = Join-Path $repoRoot ".cache\ccache"
$ccacheTemp = Join-Path $repoRoot ".cache\ccache-tmp"
New-Item -ItemType Directory -Force -Path $ccacheRoot, $ccacheTemp |
    Out-Null

$env:CCACHE_DIR = $ccacheRoot
$env:CCACHE_TEMPDIR = $ccacheTemp

# ESP-IDF and its bundled OpenThread checkout are owned by the host user, while
# sandboxed builds may run under a different identity.  Limit the exception to
# this process instead of modifying the user's global Git configuration.
$env:GIT_CONFIG_COUNT = "3"
$env:GIT_CONFIG_KEY_0 = "safe.directory"
$env:GIT_CONFIG_VALUE_0 = $repoRoot
$env:GIT_CONFIG_KEY_1 = "safe.directory"
$env:GIT_CONFIG_VALUE_1 = $idfPath
$env:GIT_CONFIG_KEY_2 = "safe.directory"
$env:GIT_CONFIG_VALUE_2 = Join-Path $idfPath "components\openthread\openthread"

Write-Host "ESP-IDF:  $idfPath"
Write-Host "Build:    $resolvedBuildDir"
Write-Host "ccache:   $ccacheRoot"

# The .cmd launcher starts this script with ExecutionPolicy Bypass.  Calling
# export.ps1 here keeps all environment changes in this build process.
& (Join-Path $idfPath "export.ps1")

Push-Location $repoRoot
try {
    & idf.py -B $resolvedBuildDir @IdfArgs
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}
