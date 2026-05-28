# VMT SETO

Virtual Motion Tracker Setup & Easy Tracker Offset

Current version: v1.0.0

VMT SETO is a Windows tool for Virtual Motion Tracker workflows. It helps set up fixed calibration positions for virtual trackers, then switches back to low-latency live tracking during normal use.

The tool is built for VRChat full-body calibration workflows with VMT and SteamVR/OpenVR.

## Credits

The software concept, direction, and testing were provided by the project owner.
The code implementation was generated with Codex.

## Features

- VMT OSC pose output
- GUI config switching while running
- Calibration lock/unlock from the GUI
- Controller trigger/grip unlock during calibration
- Capture current tracker pose from the GUI or controller trigger
- Multi-tracker capture using checked tracker selection
- Roll offset adjustment for tracker mounting differences
- Position prediction and 1-Euro filtering for lower perceived latency
- Configurable send FPS

## Requirements

- Windows
- SteamVR
- Virtual Motion Tracker
- Booth/release ZIP users do not need to install .NET separately
- .NET SDK 10.0 or newer for building from source

## Download

Booth: Coming soon

## Manual

User manual: [Google Docs](https://docs.google.com/document/d/1EgEW8kCiLtBClyak2-tKwxFNZu0EyZutmvdmoZD_ci4/edit?tab=t.0)

## Build

```powershell
dotnet build .\VMT_SETO.slnx
```

The debug build output is created at:

```text
VMT_SETO\bin\Debug\net10.0-windows\
```

## Run

For release ZIP users, extract the whole ZIP folder before running the exe. Do not run the exe directly from inside the ZIP, and do not copy only the exe by itself.

```powershell
.\start_VMT_SETO.bat
```

Or run the built executable directly:

```text
VMT_SETO\bin\Debug\net10.0-windows\VMT_SETO_v1.0.0.exe
```

## Config

The startup selector is:

```text
VMT_SETO\config.txt
```

By default, VMT SETO starts with the config selected last time:

```text
startupConfigMode=remember
lastConfig=configs/default.txt
```

When a config is applied from the GUI, `lastConfig` is updated automatically.

To always start with the same config, set:

```text
startupConfigMode=fixed
config=configs/default.txt
```

The default config is:

```text
VMT_SETO\configs\default.txt
```

The default config intentionally contains no real tracker serial numbers. Copy it or create another `.txt` file in `VMT_SETO\configs\` for personal use.

Tracker lines use `_+_` as the field separator:

```text
Chest_+_0/0/0/0/0/0_+_0/0/0/0/0/0_+_LHR-XXXXXXXX_+_5
```

Tracker lines must use `_+_` so tracker names or serials can contain `_`.

## Troubleshooting

If VMT SETO exits unexpectedly, check `crash.log` next to `VMT_SETO.exe`.

If the selected config has no real tracker serial/name values, VMT SETO shows a warning and the manual URL. Edit `configs/*.txt` before using Capture.

If SteamVR is not ready, VMT SETO stays open and retries automatically. If OSC sending fails, VMT SETO logs the error and retries instead of exiting immediately.

Tracker status colors:

- `Tracking`: source tracker is found and pose is valid.
- `Not found`: config serial/name does not match any OpenVR device.
- `No pose`: source tracker is found, but SteamVR is not providing a valid pose.
- `Config required`: tracker serial/name is empty or still looks like an example value.

The checked tracker list shows these states as a small color indicator before each tracker name.

## Release Builds

Build artifacts in `bin/` and `obj/` are not committed to the repository. For Booth distribution, run:

```powershell
.\build_booth_package.ps1
```

The versioned package is created under `dist\`.

## License

MIT License.

This project includes ValveSoftware OpenVR files. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
