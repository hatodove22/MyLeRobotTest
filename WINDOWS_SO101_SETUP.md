# Windows SO-101 setup notes

This workspace is set up for LeRobot on Windows using `uv` and a local `.venv`.

## Current environment

- Repo: `C:\Users\tesul\LeRobot`
- Python: `3.12.12`
- PyTorch: `2.10.0+cpu`
- LeRobot: `0.5.2`
- Installed extras: `core_scripts`, `feetech`

## Activate the environment

From PowerShell in this folder:

```powershell
Set-Location C:\Users\tesul\LeRobot
.\.venv\Scripts\Activate.ps1
```

If PowerShell blocks activation, use:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\.venv\Scripts\Activate.ps1
```

## Windows-specific port checks

List current serial ports:

```powershell
.\scripts\windows\Show-SerialPorts.ps1
```

At setup time, LeRobot detected `COM5` and `COM6`. Treat that as a snapshot, not a fixed mapping.

You can also run the official interactive probe:

```powershell
lerobot-find-port
```

That command asks you to unplug one controller board and press Enter, so it must be run in an interactive shell.

## SO-101 commands on Windows

Replace Linux/macOS paths like `/dev/ttyACM0` with Windows ports like `COM5`.

Follower motor setup:

```powershell
lerobot-setup-motors `
  --robot.type=so101_follower `
  --robot.port=COM5 `
  --robot.id=my_so101_follower
```

Leader motor setup:

```powershell
lerobot-setup-motors `
  --teleop.type=so101_leader `
  --teleop.port=COM6 `
  --teleop.id=my_so101_leader
```

Follower calibration:

```powershell
lerobot-calibrate `
  --robot.type=so101_follower `
  --robot.port=COM5 `
  --robot.id=my_so101_follower
```

Leader calibration:

```powershell
lerobot-calibrate `
  --teleop.type=so101_leader `
  --teleop.port=COM6 `
  --teleop.id=my_so101_leader
```

## Firmware / motor utility on Windows

The Feetech firmware/debug utility is Windows-native. That lines up well with your environment.

Reference:

- LeRobot docs: `docs/source/installation.mdx`
- SO-101 guide: `docs/source/so101.mdx`
- Feetech notes: `docs/source/feetech.mdx`

