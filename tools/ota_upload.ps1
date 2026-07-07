param(
    [string]$HostIp = "192.168.140.143",
    [string]$Firmware = "build/uwb_esp_idf.bin",
    [string]$Secrets = "secrets.h"
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

$Uri = "http://$HostIp/ota"
Write-Host "Uploading $FirmwarePath to $Uri"

& curl.exe --silent --fail-with-body --show-error `
    -H "X-OTA-Token: $Token" `
    --data-binary "@$FirmwarePath" `
    $Uri

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
