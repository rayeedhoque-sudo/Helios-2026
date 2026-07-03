# Drivetrain (Swerve) Code — Issue Register

**Source:** multi-agent deep review of the swerve code on team `main` (commit `6eb7e8a`
"SlewRateLimiters For Drive Added"), run 2026-07-01. 33 raw findings → 32 survived
adversarial verification → 21 after de-duplication.

**Fixes** (where marked ✅) live on branch **`swerve-fixes`** (commit `300de23`).

**Status legend:** ✅ Fixed · 🔧 Open (code) · 🤖 Needs the robot to resolve/verify

> ⚠️ Team `main` (`6eb7e8a`) **does not compile** because of #1 and #2. The `swerve-fixes`
> branch compiles (`gradlew build` succeeds). None of this is robot-tested yet.

## At a glance

| Status | Findings |
|---|---|
| ✅ Fixed on `swerve-fixes` | #1, #2, #4, #5, #8, #9, #12 |
| 🔧 Open (code fix, no robot needed) | #3, #6, #7, #10, #11, #13, #14, #15, #16, #17, #18 |
| 🤖 Needs the robot | #19, #20, #21 + tuning/verification items below |

---

## Findings (ranked most-severe first)

### ✅ #1 — Build blocker: `SlewRateLimiter` misspelled type + missing import  `[critical/build]`
`RobotContainer.java:42` declared `SlewrRateLimiter` (extra "r"), and `edu.wpi.first.math.filter.SlewRateLimiter` was never imported → two "cannot find symbol" errors that abort the whole build.
**Fixed:** removed the broken slew-limiter field; restored the last-working drive command.

### ✅ #2 — Build blocker: `getLeftC()` + wrong strafe axis  `[critical/build]`
`RobotContainer.java:56` called `joystick2.getLeftC()` (not a real method). The axis mapping was also wrong (`getLeftX()` unused → naive rename would make X and Y read the same axis).
**Fixed:** `withVelocityX(-getLeftY()*…)`, `withVelocityY(-getLeftX()*…)`.

### 🔧 #3 — `getAutonomousCommand()` returns `Commands.none()` — robot does nothing in auto  `[critical/auto]`
`RobotContainer.java:94`. No path runs for the whole ~15 s auto period. PathPlanner autos exist on disk (`Auto PID.auto`, `Drive Test`) but nothing schedules them; there's no `SendableChooser`.
**Fix:** return `drivetrain.getAuton("Auto PID")` or add a chooser. (Fix together with #6, #7.)

### ✅ #4 — Drive Krakens had no supply current limit (brownout risk)  `[high/power]`
`TunerConstants.java` — `driveInitialConfigs` was empty; only `kSlipCurrent=120A` (a stator cap) existed. Four drive motors accelerating together could brown out the roboRIO.
**Fixed:** added `SupplyCurrentLimit = 30A` (conservative start, §7). Stator/traction still from `kSlipCurrent`. TODO tune.

### ✅ #5 — Drive + steer left in Coast (never set to Brake)  `[high/config]`
No neutral mode was configured anywhere → factory default Coast → robot coasts/rolls when disabled and **doesn't decelerate** (this is the "pushes itself along" symptom observed on the robot).
**Fixed:** `NeutralModeValue.Brake` on both drive and steer.

### 🔧 #6 — PathPlanner reset only resets rotation, drops X/Y  `[high/auto]`
`CommandSwerveDrivetrain.java:226` — reset callback is `pose -> seedFieldCentric(pose.getRotation())`, which sets heading only. `Auto PID.auto` has `resetOdom:true`, so the path's start X/Y is silently dropped → lurch off-path at auto start.
**Fix:** `pose -> this.resetPose(pose)`.

### 🔧 #7 — PathPlanner translation `kP = 0.0001` ≈ open-loop  `[high/auto]`
`CommandSwerveDrivetrain.java:230` — a 1 m error yields 0.0001 m/s correction. Path following is essentially feedforward-only; drift/slip never corrected. (Rotation `kP=5` is fine.)
**Fix:** tuned translation `kP ≈ 5.0`, validate on robot.

### ✅ #8 — One shared `SlewRateLimiter` for both X and Y  `[medium/driving]`
`RobotContainer.java` — a single limiter's `.calculate()` was called for both axes each loop, cross-coupling them and gluing strafe to forward. Rotation was unlimited.
**Fixed:** the botched slew limiter was removed entirely (it also fights the crisp-decel goal of #5). If slew limiting is wanted later, add **two** limiters (one per axis) or limit the translation vector magnitude.

### 🔧 #9 → ✅ — `seedFieldCentric` on `povCenter()` re-seeded heading on any D-pad release  `[medium/driving]`
`RobotContainer.java:75` — `povCenter()` is true whenever the D-pad is at rest, so any incidental tap re-zeroed field-centric heading mid-match.
**Fixed:** moved to the **Y** button (deliberate press only).

### 🔧 #10 — No vision pose correction active  `[medium/odometry]`
`CommandSwerveDrivetrain.java:312` — the whole MegaTag2/`addVisionMeasurement` block in `periodic()` is commented out. Pose runs on wheel odometry + Pigeon only → unbounded drift over a match; every absolute-pose consumer (autos, `driveToPose`, AprilTag align) slowly aims wrong. Teleop field-centric still works.
**Fix:** enable MegaTag2 fusion — but fix #15 first (the `getAlliance().get()` in that block will throw).

### 🔧 #11 — `rotateToAngle()` never configures its heading PID  `[medium/auto]`
`CommandSwerveDrivetrain.java:398` — `FieldCentricFacingAngle`'s `HeadingController` gains default to 0, so it outputs zero rotation, and the command `.until()`s on reaching the angle → could hold the drivetrain forever. Latent (its binding is commented out).
**Fix:** `request.HeadingController.setPID(...)` + `enableContinuousInput(-π, π)` before use.

### ✅ #12 — Steer had a stator limit but no supply cap  `[low/power]`
`TunerConstants.java` — steer had 40 A stator, no supply cap.
**Fixed:** added `SupplyCurrentLimit = 20A` (§7). Stator 40 A unchanged.

### 🔧 #13 — `driveToPose(Pose2d)` builds `PathConstraints(0,0,0,0)`  `[low/auto]`
`SubsystemConstants.java:18` — `Vision.PP_MAX_*` are all 0, so pathfinding can't move. Latent (overload unbound today).
**Fix:** populate with real m/s, m/s², rad/s, rad/s² values. TODO tune.

### 🔧 #14 — `RobotConfig.fromGUISettings()` failure silently skips `AutoBuilder.configure`  `[low/auto]`
`CommandSwerveDrivetrain.java:216` — on exception, `config` stays null and AutoBuilder is never configured, with only `printStackTrace()`. A future missing/corrupt `settings.json` would silently disable all path following.
**Fix:** `DriverStation.reportError(...)` / Alert on the failure path.

### 🔧 #15 — Unguarded `getAlliance().get()` in the (commented) vision block  `[low/odometry]`
`CommandSwerveDrivetrain.java:320` — `DriverStation.getAlliance().get()` with no `isPresent()` guard throws before FMS/DS connect. Latent because commented out, **but it's the exact code to re-enable for #10** — guard it first.
**Fix:** `var a = getAlliance(); if (a.isEmpty()) return; …`

### 🔧 #16 — `tuner-project.json` out of sync with `TunerConstants.java`  `[low/config]`
The stale Tuner-X project records **opposite** steer/encoder inversions, different CANcoder offsets, and a swapped FL/BL corner mapping. `TunerConstants.java` (what runs, and matches the data sheet) is authoritative → no runtime effect today.
**Trap:** regenerating from Tuner X would overwrite `TunerConstants.java` with inverted steer polarity + wrong offsets → runaway/oscillating azimuth. **Do not regenerate** until re-synced. Also physically re-zero all four CANcoders on the robot before first enabled drive.

### 🔧 #17 — `DriveRequestType.Velocity` mislabeled "open-loop" in a comment  `[low/config]`
`RobotContainer.java:30` — `Velocity` is closed-loop; the comment is leftover Tuner default text. Documentation only; robot drives.
**Fix:** correct the comment (or switch to `OpenLoopVoltage` if open-loop feel is actually intended).

### 🔧 #18 — Deadband applied after scaling instead of to the raw stick  `[low/driving]`
`RobotContainer.java:29` — the request's 10% deadband applies to the scaled velocity, not the raw joystick, so stick noise is integrated before it's clipped. Minor low-speed imprecision. (The prior "after slew" concern is moot now that the slew limiter is gone.)
**Fix:** `MathUtil.applyDeadband` on the raw axes before scaling.

### 🤖 #19 — `kSpeedAt12Volts = 4.93` vs `tuner-project.json`'s 4.76 (~3.5% off)  `[low/config]`
`TunerConstants.java:78` — 4.93 = X60 non-FOC theoretical; project file records the FOC-loaded figure. Which is right depends on whether a **Phoenix Pro/FOC license** is active (not knowable from code). Affects stick scaling / top-speed ceiling slightly.
**Resolve:** re-derive from a full-speed SysId run on the robot.

### 🤖 #20 — ~24 CAN nodes on the roboRIO CAN 2.0 bus  `[low/canbus]`
`TunerConstants.java:74` — `kCANBus = new CANBus("")` selects the roboRIO onboard CAN 2.0 bus, carrying 19 Phoenix devices + 5 SPARK MAX with FusedCANcoder + 100 Hz odometry. Near the practical CAN 2.0 ceiling; heavy load can drop frames (CAN-not-received faults, motor stutter, odometry jitter). The `.hoot` path is a CANivore feature.
**Resolve:** ideally move the drivetrain to a CANivore CAN FD bus (then bump `odometryUpdateFrequency` to 250); otherwise log `CANBus.getStatus().BusUtilization` (< ~0.7) and trim status-frame rates. Needs on-robot measurement.

### 🤖 #21 — Pigeon2 mount-pose config never applied (`pigeonConfigs = null`)  `[low/config]`
`TunerConstants.java:70` — no MountPose written on boot; yaw reference relies entirely on whatever is flashed in Tuner X. If the Pigeon isn't physically flat / +X-forward or its calibration is stale, field-centric drive (and PathPlanner/vision) inherit a heading offset.
**Resolve:** set the measured MountPose via `withPigeon2Configs(...)`, or verify Tuner-X calibration + heading sign (CCW increases yaw) on the robot.

---

## Not covered above — completeness / verify-on-robot

These came out of the review's completeness pass and are **genuine "needs the robot" unknowns** (Hardware-Data-Sheet §11 items):

- **CANcoder magnet offsets & all motor inversions** — re-zero the 4 CANcoders physically before the first enabled drive; confirm module directions (best done with the robot on blocks). Independent of the `tuner-project.json` mismatch (#16).
- **Phoenix Pro / FOC license status** — decides #19 *and* whether `FusedCANcoder` truly fuses or silently falls back to `RemoteCANcoder`.
- **Module geometry / invert flags** — `kInvertLeftSide`/`kInvertRightSide`, `kSteerMotorInverted`, track width/wheelbase (`k*XPos/YPos`), `kWheelRadius` (2 in), `kDriveGearRatio` (6.48) not independently confirmed against hardware/CAD. A sign error here breaks kinematics.
- **PathPlanner robot mass / MOI** — pulled from the PathPlanner GUI (`settings.json`), not code. If they don't match the real robot, path feedforward is wrong even after #7.
- **Drive/steer PID + FF (`kS/kV/kA`)** — untuned defaults; run SysId.
- **`Robot.java` / `Telemetry.java`** — not deeply reviewed; check scheduler plumbing and NetworkTables/Shuffleboard flooding at 50–100 Hz.
- **No `SendableChooser`** for autos and no fallback if a named auto/path file is missing (compounds #3).

---

## Related work not in this list (mechanism subsystems)

The `swerve-fixes` branch also added conservative-start current limits + neutral modes to
**all non-drivetrain motors** (flywheels ×4, kicker, intake roller, hood, slider, hopper A/B),
and **temporarily disabled** shooter/intake/hopper in `RobotContainer` for a drivetrain-only
test build. Re-enable those three subsystems (and revert this doc's "TEMP" note) before merging
`swerve-fixes` to `main`.
