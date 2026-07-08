param(
    [string]$HostIp = "",
    [string[]]$Hosts = @(),
    [string]$TargetList = "",
    [string]$Firmware = "build/uwb_esp_idf.bin",
    [string]$Secrets = "secrets.h",
    [int]$Parallel = 5,
    [int]$TimeoutSec = 120
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")

function Resolve-ProjectPath {
    param([string]$Path)

    if ([System.IO.Path]::IsPathRooted($Path)) {
        return (Resolve-Path -LiteralPath $Path).Path
    }

    return (Resolve-Path -LiteralPath (Join-Path $ProjectRoot $Path)).Path
}

function Normalize-OtaTarget {
    param([string]$Target)

    if ([string]::IsNullOrWhiteSpace($Target)) {
        return $null
    }

    $Clean = ($Target -split '\s+#', 2)[0].Trim()
    if ([string]::IsNullOrWhiteSpace($Clean) -or $Clean.StartsWith("#")) {
        return $null
    }

    return $Clean
}

function Get-OtaUri {
    param([string]$Target)

    if ($Target -match '^https?://') {
        if ($Target.EndsWith("/ota")) {
            return $Target
        }

        return $Target.TrimEnd("/") + "/ota"
    }

    return "http://$Target/ota"
}

function Read-OtaTargets {
    param([string]$Path)

    $ResolvedPath = Resolve-ProjectPath $Path
    Get-Content -LiteralPath $ResolvedPath | ForEach-Object {
        Normalize-OtaTarget $_
    } | Where-Object { $null -ne $_ }
}

$FirmwarePath = Resolve-ProjectPath $Firmware
$SecretsPath = Resolve-ProjectPath $Secrets

$SecretMatch = Select-String -Path $SecretsPath -Pattern '#define\s+APP_OTA_PASSWORD\s+"([^"]*)"' |
    Select-Object -First 1

if ($null -eq $SecretMatch) {
    throw "APP_OTA_PASSWORD was not found in $SecretsPath"
}

$Token = $SecretMatch.Matches[0].Groups[1].Value
if ([string]::IsNullOrWhiteSpace($Token)) {
    throw "APP_OTA_PASSWORD is empty"
}

$Targets = @()
$HasExplicitTargets = $false

if (-not [string]::IsNullOrWhiteSpace($HostIp)) {
    $HasExplicitTargets = $true
    $Targets += $HostIp
}

if ($Hosts.Count -gt 0) {
    $HasExplicitTargets = $true
    $Targets += $Hosts
}

if (-not [string]::IsNullOrWhiteSpace($TargetList)) {
    $HasExplicitTargets = $true
    $Targets += Read-OtaTargets $TargetList
}

if ($Targets.Count -eq 0 -and -not $HasExplicitTargets) {
    $Targets += "192.168.140.143"
}

$Targets = @($Targets | ForEach-Object {
    Normalize-OtaTarget $_
} | Where-Object {
    $null -ne $_
} | Select-Object -Unique)

if ($Targets.Count -eq 0) {
    throw "No OTA targets configured"
}

if ($Parallel -lt 1) {
    $Parallel = 1
}

Write-Host "Uploading $FirmwarePath to $($Targets.Count) target(s), parallel=$Parallel"

$UploadScript = {
    param(
        [string]$Target,
        [string]$FirmwarePath,
        [string]$Token,
        [int]$TimeoutSec
    )

    function Get-JobOtaUri {
        param([string]$Target)

        if ($Target -match '^https?://') {
            if ($Target.EndsWith("/ota")) {
                return $Target
            }

            return $Target.TrimEnd("/") + "/ota"
        }

        return "http://$Target/ota"
    }

    $Uri = Get-JobOtaUri $Target
    Write-Output "Uploading to $Uri"

    $Output = & curl.exe --silent --fail-with-body --show-error `
        --connect-timeout 10 `
        --max-time $TimeoutSec `
        -H "X-OTA-Token: $Token" `
        --data-binary "@$FirmwarePath" `
        $Uri 2>&1

    if ($LASTEXITCODE -ne 0) {
        throw "curl failed for $Target with exit code $LASTEXITCODE`n$Output"
    }

    if ($Output) {
        Write-Output $Output
    }
}

$Failures = @()

if ($Targets.Count -eq 1) {
    $Target = $Targets[0]
    Write-Host "[$Target] $(Get-OtaUri $Target)"
    try {
        & $UploadScript $Target $FirmwarePath $Token $TimeoutSec | ForEach-Object {
            Write-Host "[$Target] $_"
        }
        Write-Host "[$Target] OTA upload completed"
    } catch {
        throw "OTA upload failed for $Target. $($_.Exception.Message)"
    }
    exit 0
}

$Pending = New-Object 'System.Collections.Generic.Queue[string]'
foreach ($Target in $Targets) {
    $Pending.Enqueue($Target)
}

$Running = @()

while ($Pending.Count -gt 0 -or $Running.Count -gt 0) {
    while ($Pending.Count -gt 0 -and $Running.Count -lt $Parallel) {
        $Target = $Pending.Dequeue()
        Write-Host "[$Target] starting $(Get-OtaUri $Target)"
        $Job = Start-Job -ScriptBlock $UploadScript `
            -ArgumentList $Target, $FirmwarePath, $Token, $TimeoutSec
        $Running += [pscustomobject]@{
            Target = $Target
            Job = $Job
        }
    }

    if ($Running.Count -eq 0) {
        continue
    }

    $Jobs = $Running | ForEach-Object { $_.Job }
    $Finished = Wait-Job -Job $Jobs -Any -Timeout 1
    if ($null -eq $Finished) {
        continue
    }

    foreach ($Job in @($Finished)) {
        $Entry = $Running | Where-Object { $_.Job.Id -eq $Job.Id } |
            Select-Object -First 1
        $Target = $Entry.Target
        $JobErrors = @()
        $Output = Receive-Job -Job $Job -ErrorAction SilentlyContinue `
            -ErrorVariable JobErrors

        if ($Job.State -eq "Completed") {
            foreach ($Line in $Output) {
                Write-Host "[$Target] $Line"
            }
            Write-Host "[$Target] OTA upload completed"
        } else {
            $Failures += $Target
            Write-Warning "[$Target] OTA upload failed"
            foreach ($Line in $Output) {
                Write-Warning "[$Target] $Line"
            }
            foreach ($JobError in $JobErrors) {
                Write-Warning "[$Target] $JobError"
            }
        }

        Remove-Job -Job $Job
        $Running = @($Running | Where-Object { $_.Job.Id -ne $Job.Id })
    }
}

if ($Failures.Count -gt 0) {
    throw "OTA failed for: $($Failures -join ', ')"
}

Write-Host "OTA upload completed for all targets"
