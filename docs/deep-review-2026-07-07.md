# Deep Code Review — 2026-07-07 (whole codebase, new findings only)

**Source:** multi-agent review (6 domain finders + 1 adversarial verifier per finding) of the
working tree on `swerve-fixes`, run 2026-07-07. 21 raw findings → 14 survived verification →
13 unique (one duplicate merged). Everything already in `drivetrain-issues.md` was excluded.

**Status legend:** ✅ Fixed (this pass, build-verified) · 📋 Accepted as-is (rationale below) ·
🖥️ Needs a desk action · 🤖 Needs the robot

| # | Sev | Status | Finding |
|---|-----|--------|---------|
| D1 | high | ✅ | Disabling the shooter subsystem never stopped the flywheels |
| D2 | high | ✅ | `VelocityVoltage(0)` actively reverse-braked the flywheels |
| D3 | high | ✅ | Hood angle-PID Shuffleboard tuning was dead code |
| D4 | high | ✅ | AprilTag HUB/TRENCH fiducial IDs were scrambled (7/8) |
| D5 | high | ✅ | `driveToPose(Translation2d,…)` ended on a never-terminating Idle |
| D6 | med | ✅ | `driveToPose(Translation2d,…)` measured distance from the field origin |
| D7 | med | ✅ | Shooter vision read the wrong Limelight camera-space axes |
| D8 | med | ✅ | Shooter vision deserialized the full Limelight JSON every 20 ms |
| D9 | med | ✅ | Slew limiters unlimited on the first frame after another command released the drivetrain |
| D10 | med | ✅ | `rotateToAngle(double)` froze the target at binding time (now `DoubleSupplier`) |
| D11 | med | 🖥️ | PathPlanner `.auto`/`.path` files still stamped `"version": "2025.0"` |
| D12 | med | 📋 | Blocking `configurator.apply()` inside `periodic()` on slider change |
| D13 | low | ✅ | `HootAutoReplay.update()` called once in `teleopInit` instead of every loop |

## Details

### ✅ D1 — Disabling the shooter never stopped the flywheels  (`ShooterSubsystem.periodic`)
`periodic()` gated its whole body on `enableSubsystem`. Phoenix 6 `VelocityVoltage` **latches** on
the TalonFX — skipping the loop body leaves the motor executing the last setpoint, so "disable
shooter" while robot-enabled left 4 Krakens spinning at the last commanded speed indefinitely.
**Fixed:** else-branch now sends `CoastOut` to the master (followers mirror it), stops the hood
(`setVoltage(0)`, Brake idle holds the angle), and zeroes `desired_Velocity`.

### ✅ D2 — `VelocityVoltage(0)` reverse-braked the flywheels
Commanding velocity 0 closes the loop on 0 rps: with a spinning wheel, kP saturates the output
negative = hard regen braking of 4 Krakens — exactly what the data sheet's Coast choice (§7) was
meant to avoid (regen current dump + gearbox stress on every post-shot spin-down).
**Fixed:** `motorRps == 0` → `CoastOut`; the wheels now spin down freely.

### ✅ D3 — Hood angle-PID live tuning dead
The re-tune guard compared `shooterAnglePID` against the **static constants it was built from**
(never change → condition never true → `setPID` unreachable). Slider edits silently did nothing;
the speed block was correct. **Fixed:** guard now compares against the Shuffleboard entries,
mirroring the speed block.

### ✅ D4 — AprilTag IDs scrambled  (`SubsystemConstants`)
Old constants: RED_HUB=7, BLUE_HUB=8, RED_TRENCH=7, BLUE_TRENCH=8. Per the official 2026 REBUILT
field (Hardware-Data-Sheet §3): 7 is a RED **TRENCH** tag, 8 a RED **HUB** tag; no BLUE ID was
present at all — on-field vision ranging would essentially never match. **Fixed:** full per-
alliance ID sets (RED HUB 2,3,4,5,8,9,10,11 · BLUE HUB 18,19,20,21,24,25,26,27 · RED TRENCH
1,6,7,12 · BLUE TRENCH 17,22,23,28) + membership check. TODO: filter to own-alliance HUB once the
shooting model (`generate*`) is implemented.

### ✅ D5 / D6 — `driveToPose(Translation2d, Translation2d)` broken twice
(a) Its trailing `applyRequest(Idle)` leg never finishes → parks the drivetrain and locks out the
driver (same trap as `rotateToAngle` had). (b) Drive duration used
`targetTranslation.getNorm()` = distance of the target **from the field origin**, not from the
robot. **Fixed:** wrapped in `Commands.defer` (pose read at schedule time), distance =
`target.minus(currentPose)`, terminating one-shot `runOnce(setControl(idle))` tail.

### ✅ D7 / D8 — Shooter vision path (latent; `enableVision` currently false)
Axes: Limelight camera space is X=right, Y=down, Z=forward; the code used X as distance, Z as
height, and computed the "bearing" in the vertical plane. **Fixed:** distance=Z, height=Y,
bearing=atan2(X, Z) — `// TODO verify signs/axes on the robot with a real tag`. Perf: replaced the
per-loop `getLatestResults()` full-JSON Jackson parse with cheap NT reads
(`getTV`/`getFiducialID`/`getTargetPose3d_CameraSpace`).

### ✅ D9 — Slew limiter first-frame bypass
`SlewRateLimiter` bounds change by `rate × elapsed-since-last-calculate`. When brake/auto/align
held the drivetrain for seconds, the first `calculate()` after release allowed an unlimited jump.
**Fixed:** `.beforeStarting(reset both limiters)` on the default drive command — covers enable,
auto→teleop, and every other release (replaces the narrower disabled-trigger reset).

### ✅ D10 — `rotateToAngle` target frozen at binding time
`drivetrain.rotateToAngle(shoterSS.getDegreesToAlignToTarget())` evaluates the double **once at
boot** (0°). **Fixed:** primary overload now takes a `DoubleSupplier` (re-sampled every loop);
double overload delegates. The commented A-button binding was updated to the `::` supplier form.

### ✅ D11 — PathPlanner files stamped `"version": "2025.0"` — proven compatible
`Auto PID.auto`, `Drive Test.path`, `Rotation Test.path` carry the 2025 schema tag while the lib
is 2026.1.2. **Resolved at the desk:** `src/test/java/frc/robot/PathPlannerFilesTest.java` parses
all three files with the installed 2026.1.2 library — both tests PASS (`gradlew build`), so the
chooser will load them on the robot. `buildAutoChooser` is additionally try/caught (a future bad
file reports to the DS and yields a do-nothing chooser instead of crashing on boot). Re-saving in
the PathPlanner 2026 GUI is now optional hygiene, not a blocker.
*(Environment note: running these tests needed a laptop fix — `~\.gradle\gradle.properties` was
force-pinning ALL Gradle builds to `C:\Program Files\Java\jdk-17` (Oracle), whose old bundled
VC++ runtime hard-crashes WPILib 2026 desktop natives. It now points at the WPILib 2026 JDK;
simulation runs were broken by the same pin.)*

### 📋 D12 — Blocking `configurator.apply()` in `periodic()` — accepted as-is
The speed-PID re-apply is a blocking CAN call (~tens of ms) on the shared scheduler thread, **but
it only fires when someone edits a Shuffleboard slider** — i.e. during tuning sessions, where a
one-loop hitch is harmless; it cannot fire mid-match unattended. Fixing it (async apply /
disabled-only gating) would complicate the team's live-tuning workflow for no match-play benefit.
Revisit only if tuning-time hitches become a problem.

### ✅ D13 — `HootAutoReplay.update()` once in `teleopInit`
Replay advances one frame per `update()` call; a single call = replay never progresses. Moved to
`robotPeriodic()` (before the scheduler run). Inert on the real robot; makes log replay in sim
actually work.

## Refuted in verification (not defects — do not "re-fix")
7 raw findings were killed by the adversarial verifiers as wrong, exaggerated, intentional
behavior, or duplicates of the known registers. Notables: follower-once configuration is correct
(control requests auto-retransmit); the hood gear-ratio constants are internally consistent with
the data sheet; Telemetry's per-odometry-loop NT publishing matches the CTRE template design.
