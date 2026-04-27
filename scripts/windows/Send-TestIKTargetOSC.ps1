param(
    [string]$HostName = "127.0.0.1",
    [int]$Port = 9000,
    [string]$Mode = "circle",
    [double]$X = 0.0,
    [double]$Y = 0.0,
    [double]$Z = 0.0,
    [double]$Radius = 0.05,
    [double]$Height = 0.0,
    [double]$Fps = 30.0,
    [double]$Duration = 20.0
)

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

uv run python scripts\osc\send_test_ik_target.py `
    --host $HostName `
    --port $Port `
    --mode $Mode `
    --x $X `
    --y $Y `
    --z $Z `
    --radius $Radius `
    --height $Height `
    --fps $Fps `
    --duration $Duration
