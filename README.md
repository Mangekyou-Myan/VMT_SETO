# VMT SETO

Virtual Motion Tracker Setup & Easy Tracker Offset

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

```powershell
.\start_VMT_SETO.bat
```

Or run the built executable directly:

```text
VMT_SETO\bin\Debug\net10.0-windows\VMT_SETO.exe
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

## Release Builds

Build artifacts in `bin/` and `obj/` are not committed to the repository. For distribution, package the built files from `VMT_SETO\bin\Debug\net10.0-windows\` as a zip and attach it to a GitHub Release.

## License

MIT License.

This project includes ValveSoftware OpenVR files. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
