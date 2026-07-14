# Helios 2026 — FRC Team 9704

Robot code for Helios (Java 17, WPILib 2026, GradleRIO). Developed by Team 9704 with Claude Code.

## Getting the code

First time — clone the repo:

```
git clone https://github.com/rayeedhoque-sudo/Helios-2026.git
cd Helios-2026
```

Already cloned — pull the latest:

```
git pull
```

Current work lives on the `swerve-fixes` branch (`git checkout swerve-fixes` first if you're not on it).

**Note:** pulling needs internet; deploying needs the robot's network. If the laptop uses Wi-Fi for both, pull *first*, then switch to the robot's Wi-Fi to deploy.

## Deploying code to the robot

1. **Join the robot's network** (any one of these):
   - **Robot Wi-Fi** — connect the laptop to the `9704` network broadcast by the radio. roboRIO is at `10.97.4.2`.
   - **USB-B tether** — plug into the roboRIO's USB-B port. roboRIO is at `172.22.11.2`.
   - **Ethernet tether** — plug into the radio's DS port (robot must be powered).
2. **Deploy** from this folder (`C:\FRC\Helios-2026`):

   ```
   .\gradlew deploy
   ```

   Or in WPILib VS Code: right-click `build.gradle` → **Deploy Robot Code** (Shift+F5). GradleRIO finds the roboRIO automatically from the team number (9704); no IP needed.
3. **Run it** — open the FRC Driver Station, confirm green Communications/Robot Code lights, then enable.

Deploy fails with "Target roboRIO not found"? You're not on the robot's network — check step 1 (Wi-Fi in range and connected, or tether seated) and that the robot is powered on.

Radio reprogramming, roboRIO imaging, and tool installs are documented in the `radio-deploy` skill (`.claude/skills/radio-deploy/`).
