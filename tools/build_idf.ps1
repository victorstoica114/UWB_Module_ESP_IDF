[CmdletBinding()]
param(
    [string]$ProjectDir = "",
    [string]$BuildDir = "build",
    [ValidateRange(1, 64)]
    [int]$Jobs = 4
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($ProjectDir)) {
    $ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
}

function Resolve-FullPath([string]$BasePath, [string]$Path) {
    if ([IO.Path]::IsPathRooted($Path)) {
        return [IO.Path]::GetFullPath($Path)
    }
    return [IO.Path]::GetFullPath((Join-Path $BasePath $Path))
}

$projectPath = (Resolve-Path -LiteralPath $ProjectDir).Path
$buildPath = Resolve-FullPath $projectPath $BuildDir
$toolsRoot = "C:\Espressif"
$installerConfigPath = Join-Path $toolsRoot "esp_idf.json"

# Ignored local credentials are not populated by `git worktree add`. Resolve
# the primary worktree from Git's pointer file and seed the local build copy
# only when it is missing. The file remains ignored and is never printed.
$projectSecretsPath = Join-Path $projectPath "secrets.h"
if (-not (Test-Path -LiteralPath $projectSecretsPath)) {
    $gitPointerPath = Join-Path $projectPath ".git"
    $sourceSecretsPath = $null
    if (Test-Path -LiteralPath $gitPointerPath -PathType Leaf) {
        $gitPointer = Get-Content -LiteralPath $gitPointerPath -TotalCount 1
        if ($gitPointer -match '^gitdir:\s*(.+)$') {
            $worktreeGitDir = Resolve-FullPath $projectPath $Matches[1]
            $commonGitDir = Split-Path -Parent (
                Split-Path -Parent $worktreeGitDir)
            $primaryWorktree = Split-Path -Parent $commonGitDir
            $sourceSecretsPath = Join-Path $primaryWorktree "secrets.h"
        }
    }
    if ($null -eq $sourceSecretsPath -or
        -not (Test-Path -LiteralPath $sourceSecretsPath)) {
        throw "secrets.h is missing and no primary-worktree copy was found"
    }
    Copy-Item -LiteralPath $sourceSecretsPath -Destination $projectSecretsPath
    Write-Host "Seeded ignored secrets.h from the primary worktree"
}

if (-not (Test-Path -LiteralPath $installerConfigPath)) {
    throw "ESP-IDF installer configuration not found: $installerConfigPath"
}

$installerConfig = Get-Content -LiteralPath $installerConfigPath -Raw |
    ConvertFrom-Json
$selectedId = [string]$installerConfig.idfSelectedId
if ([string]::IsNullOrWhiteSpace($selectedId)) {
    throw "esp_idf.json does not select an ESP-IDF installation"
}

$selected = $installerConfig.idfInstalled.PSObject.Properties[$selectedId].Value
if ($null -eq $selected) {
    throw "Selected ESP-IDF installation '$selectedId' is missing"
}
if ([string]$selected.version -ne "6.0.2") {
    throw "This project requires ESP-IDF 6.0.2; selected version is '$($selected.version)'"
}

$idfPath = [IO.Path]::GetFullPath([string]$selected.path)
$pythonPath = [IO.Path]::GetFullPath([string]$selected.python)
$idfPy = Join-Path $idfPath "tools\idf.py"
$idfToolsPy = Join-Path $idfPath "tools\idf_tools.py"
foreach ($requiredPath in @($pythonPath, $idfPy, $idfToolsPy)) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "Required ESP-IDF file not found: $requiredPath"
    }
}

# Export the complete environment directly through idf_tools.py. This avoids
# depending on PowerShell's execution policy or on a preconfigured terminal.
$env:IDF_TOOLS_PATH = $toolsRoot
$exportLines = & $pythonPath $idfToolsPy export --format key-value
if ($LASTEXITCODE -ne 0) {
    throw "ESP-IDF environment export failed with exit code $LASTEXITCODE"
}
foreach ($line in $exportLines) {
    if ($line -match '^([^=]+)=(.*)$') {
        $name = $Matches[1]
        $value = $Matches[2]
        if ($name -eq "PATH") {
            $value = $value.Replace("%PATH%", $env:PATH)
        }
        Set-Item -Path "Env:$name" -Value $value
    }
}
$env:IDF_PATH = $idfPath
$env:IDF_PYTHON_ENV_PATH = Split-Path -Parent (Split-Path -Parent $pythonPath)
$env:ESP_IDF_VERSION = "6.0.2"
$env:IDF_BUILD_PARALLEL_LEVEL = [string]$Jobs

# Ninja already provides incremental builds. Keep ccache in pass-through mode
# and its writable state inside the selected build directory.
$env:CCACHE_DISABLE = "1"

# Codex and the interactive user can run under different Windows identities.
# Keep Git ownership exceptions local to this process.
$env:GIT_CONFIG_COUNT = "2"
$env:GIT_CONFIG_KEY_0 = "safe.directory"
$env:GIT_CONFIG_VALUE_0 = ($idfPath -replace '\\', '/')
$env:GIT_CONFIG_KEY_1 = "safe.directory"
$env:GIT_CONFIG_VALUE_1 = ((Join-Path $idfPath "components\openthread\openthread") -replace '\\', '/')

if (-not (Test-Path -LiteralPath $buildPath)) {
    New-Item -ItemType Directory -Path $buildPath | Out-Null
}
$ccachePath = Join-Path $buildPath ".ccache"
$ccacheTempPath = Join-Path $ccachePath "tmp"
foreach ($cacheDirectory in @($ccachePath, $ccacheTempPath)) {
    if (-not (Test-Path -LiteralPath $cacheDirectory)) {
        New-Item -ItemType Directory -Path $cacheDirectory | Out-Null
    }
}
$env:CCACHE_DIR = $ccachePath
$env:CCACHE_TEMPDIR = $ccacheTempPath

# Prevent concurrent wrapper invocations from sharing one Ninja directory.
$lockPath = Join-Path $buildPath ".uwb_build.lock"
$lockStream = $null
try {
    try {
        $lockStream = [IO.File]::Open(
            $lockPath,
            [IO.FileMode]::OpenOrCreate,
            [IO.FileAccess]::ReadWrite,
            [IO.FileShare]::None)
    } catch {
        throw "Build directory is already in use: $buildPath"
    }

    Write-Host "ESP-IDF $($selected.version)"
    Write-Host "Project:   $projectPath"
    Write-Host "Build dir: $buildPath"
    & $pythonPath $idfPy -C $projectPath -B $buildPath build
    if ($LASTEXITCODE -ne 0) {
        throw "ESP-IDF build failed with exit code $LASTEXITCODE"
    }

    $imagePath = Join-Path $buildPath "uwb_esp_idf.bin"
    if (-not (Test-Path -LiteralPath $imagePath)) {
        throw "Build completed without expected image: $imagePath"
    }
    $image = Get-Item -LiteralPath $imagePath
    $hash = Get-FileHash -Algorithm SHA256 -LiteralPath $imagePath
    Write-Host ("Image:     {0} bytes" -f $image.Length)
    Write-Host ("SHA-256:   {0}" -f $hash.Hash.ToLowerInvariant())
} finally {
    if ($null -ne $lockStream) {
        $lockStream.Dispose()
    }
}
