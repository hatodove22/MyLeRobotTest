$devices = Get-CimInstance Win32_PnPEntity |
    Where-Object { $_.Name -match '\(COM\d+\)' } |
    ForEach-Object {
        [pscustomobject]@{
            Port = [regex]::Match($_.Name, 'COM\d+').Value
            Name = $_.Name
            Status = $_.Status
        }
    } |
    Sort-Object Port

if (-not $devices) {
    Write-Host "No COM ports found."
    exit 1
}

$devices | Format-Table -AutoSize
