param(
    [Parameter(Mandatory = $true)]
    [string]$Port,
    [int]$Baud = 115200,
    [int]$Seconds = 0,
    [switch]$Reset,
    [switch]$Timestamp
)

$ErrorActionPreference = "Stop"

function Write-SerialText {
    param([string]$Text)

    if ([string]::IsNullOrEmpty($Text)) {
        return
    }

    if (-not $Timestamp) {
        Write-Host -NoNewline $Text
        return
    }

    $lines = $Text -split "(`r`n|`n|`r)"
    foreach ($line in $lines) {
        if ($line.Length -eq 0) {
            continue
        }

        Write-Host ("{0:HH:mm:ss.fff} {1}" -f (Get-Date), $line)
    }
}

$serial = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$serial.ReadTimeout = 200
$serial.WriteTimeout = 200

try {
    $serial.Open()
    Write-Host "Serial log opened on $Port at $Baud baud"

    if ($Reset) {
        Write-Host "Resetting target via RTS/DTR"
        $serial.DtrEnable = $false
        $serial.RtsEnable = $true
        Start-Sleep -Milliseconds 120
        $serial.RtsEnable = $false
        Start-Sleep -Milliseconds 120
    }

    $deadline = $null
    if ($Seconds -gt 0) {
        $deadline = (Get-Date).AddSeconds($Seconds)
    }

    while ($true) {
        if ($null -ne $deadline -and (Get-Date) -ge $deadline) {
            break
        }

        try {
            Write-SerialText $serial.ReadExisting()
        } catch {
            Start-Sleep -Milliseconds 50
        }

        Start-Sleep -Milliseconds 50
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
