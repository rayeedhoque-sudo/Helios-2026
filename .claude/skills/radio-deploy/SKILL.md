---
name: radio-deploy
description: Use when configuring the robot radio (Vivid-Hosting VH-109), imaging the roboRIO, deploying code to the robot, or installing FRC tooling (WPILib, Phoenix Tuner X, REV Hardware Client, Game Tools) — network setup, static IPs, and installer locations for FRC team 9704 "Helios".
---

# Radio, Network & Deploy — FRC 9704 Helios

*(Migrated from `C:\FRC\CLAUDE.md` on 2026-07-11 so it loads only when needed.)*

## Reprogram the radio (2026 = Vivid-Hosting VH-109)
Tether Ethernet into the radio's **DS port**, power the radio (Weidmüller connectors or PoE). Set this laptop's **wired** adapter to a static IP in `192.168.69.0/24` (e.g. `192.168.69.2`, mask `255.255.255.0`), browse to **`http://192.168.69.1/`** (or `http://radio.local/` on firmware > 1.1.0), set **Team Number = 9704**, click Configure.

## Operating IPs once configured (team 9704)
Radio/gateway `10.97.4.1`, roboRIO `10.97.4.2`, Limelight `limelight-knight.local` (~`10.97.4.11` if assigned). USB-B tether to the roboRIO = `172.22.11.2`. (No IP is hardcoded in code — only the team number; the radio IP derives from it.)

## Deploy/operate tools (installed)
FRC Driver Station (`C:\Program Files (x86)\FRC Driver Station\DriverStation.exe`) + roboRIO Imaging Tool (`...\National Instruments\LabVIEW 2025\project\roboRIO Tool\`, 2026 image v1.2). roboRIO already imaged. WPILib 2026 at `C:\Users\Public\wpilib\2026` (bundled JDK at `...\jdk`, "2026 WPILib VS Code" at `...\vscode\Code.exe`). Phoenix Tuner X installed (Microsoft Store). To deploy code the laptop must be **on the robot's network** (USB/Ethernet tether, or Wi-Fi once the radio is configured + in range).

## Bundled tool files: `C:\FRC\tools\`
See `tools\README.md` — FRC Game Tools 2026 installer (reinstalls Driver Station + Imaging Tool), roboRIO 1.0/2.0 images (v1.2), and the PathPlanner 2026.1.2 installer (design autos with no robot).

## Remaining installs (not bundlable)
**REV Hardware Client 2** (SPARK MAX/NEO config — REVLib 2026 needs RHC2; installer was in Downloads). Links in `tools\README.md`. git 2.54 + Java 17 are on PATH.
