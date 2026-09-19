# Helios V2 — deep adversarial review, 2026-07-24

Scope: the whole V2 tree (`C:\FRC\Helios-2026-V2`), with emphasis on hardware/current safety,
efficiency, the Limelight, and the reported "robot position on the driver companion app is
extremely off". Includes a research pass over 2025–2026 open-source code from 254, 1678, 3476,
6328, 2910, StuyPulse, 581, 696 and others, plus the Limelight, CTRE, REV and WPILib primary docs.

Everything below is either **fixed and committed**, **needs the team**, or **recommended, not
done**. Nothing tuned on the robot (current limits, hood voltages/anchors, follower alignments,
CANcoder offsets, shot model, keybinds) was changed.

---

## 1. TL;DR

**The pose was wrong for five stacked reasons, not one.** Four are now fixed in code; the
remaining one needs ten minutes with a tape measure and cannot be invented from a dev machine.

| # | Cause | Effect on the drawn robot | Status |
|---|---|---|---|
| 1 | MegaTag2 was solving with the **camera's own IMU**, not the Pigeon | position error ≈ `d·sin(yaw error)` — 0.69 m at 4 m for 10° | **fixed** |
| 2 | The camera's IMU was **never seeded** outside autonomous | worst in teleop-only practice — exactly when it was noticed | **fixed** |
| 3 | The **same camera frame was fused repeatedly** at loop rate | pose snapped to, and jittered around, whatever vision last said | **fixed** |
| 4 | The **camera mount pose is not set** | botpose reports where the *camera* is, not the *robot* | **needs a measurement** |
| 5 | The pose seed **almost never fired**, and it is the only thing that sets X/Y at all | X/Y stuck at the power-on origin; field yaw arbitrary forever | **fixed** |

Two secondary accuracy gaps were also closed: vision measurements are now weighted by tag
distance and count instead of a flat 0.7 m, and off-field solves are rejected on the fusion path
(previously only on the boot seed).

**The driver companion app is not the problem.** Its canvas geometry was checked end to end —
the meters→pixels transform is correct in all three orientations, the module corner offsets match
`TunerConstants` (±13.375 in / ±8.375 in), and it reads the right topic (`/Pose/robotPose`). It
was faithfully drawing a pose the robot had got wrong.

**Nothing about the Limelight's temperature was ever affecting the pose.** Those are unrelated
problems, and the thermal work was being done against the wrong sensor (see §6).

---

## 2. What changed (committed)

Every commit was built and tested green before the next one started.

| Commit | Change |
|---|---|
| `5e088f3` | Fuse each MegaTag2 sample only once |
| `d4a478e` | Weight MegaTag2 by tag distance/count; reject off-field solves |
| `12e59c8` | Publish vision diagnostics that separate the two pose-error causes |
| `d4796d2` | Auto-aim: one pose for distance, bearing and the aim gate |
| `4a40111` | MegaTag2: solve with the Pigeon yaw, never the camera's own IMU |
| `8bc18f0` | Report the camera mount pose; allow pushing it from code |
| `f4fb8b9` | Publish swerve telemetry at 50 Hz instead of every odometry update |
| `d48833f` | Limelight: monitor CPU temp and fps, not just the Hailo module |
| `f4f7aa5` | Publish CAN bus utilization and error counters |
| `bedc3be` | This document |
| `5e601c1` | Fail loudly when a motor config does not reach the controller |
| `d876808` | Hood: one encoder sample per loop for gate, PID and telemetry |
| `cf536fc` | Bound RT's stacked current draw: cap aim rate, hold spin-up until aimed |
| `2aa9c6f` | Only let the sticks drive the robot in teleop |
| `d91dfe8` | Add a stall cutoff to the hood NEO 550 |
| `f805bc3` | Make the shooter PID sliders actually apply |
| `838dbe8` | Keep the Limelight at full resolution while a shot is being aimed |
| `29ddcd5` | Gate the MegaTag1 boot seed on tag ambiguity |
| `c0bb367` | Correct a wrong javadoc; record the remaining verified findings |
| `ac7f26a` | Warn the driver when the tags and the gyro disagree about heading |
| `846748f` | Arm the AprilTag pose seed whenever disabled |

Test count went 17 → 22 (new `VisionFusionTest` pins the measurement-weighting maths).

Six are **team-approved behavior changes** made on 2026-07-24 after the findings were put to the
team — the RT current mitigations, the teleop drive gate, the hood stall cutoff, the live PID
sliders, and the pose-seed arming. Each is called out in its own section below and each needs an
on-robot check.

Findings came from a 47-agent research and adversarial-audit pass. Of the audit's raw findings,
**16 were refuted** on verification and 13 confirmed — the refuted ones are not in this document.

---

## 3. The pose problem, in detail

### 3.1 MegaTag2 was not using your gyro (fixed — `4a40111`)

`updateVisionPose()` set the camera's IMU mode to `isAutonomous() ? 1 : 4`.

Mode 4 is *"use internal IMU with external IMU assisted convergence"*. It makes the **Limelight's
own IMU** the yaw source for the MegaTag2 solve; the Pigeon yaw pushed via `SetRobotOrientation`
enters only as a weak complementary-filter reference at the default assist alpha of **0.001**.

That directly contradicts the rest of the file, which hands every vision measurement a theta
standard deviation of 9999999 *precisely because the Pigeon owns heading*. The code was declaring
"the gyro owns heading" to WPILib while telling the camera "solve with your own gyro instead".

A yaw prior wrong by θ puts the solved position off by roughly `d·sin(θ)`, and it grows with tag
distance:

| yaw error | position error at 2 m | at 4 m | at 6 m |
|---|---|---|---|
| 5° | 0.17 m | 0.35 m | 0.52 m |
| 10° | 0.35 m | 0.69 m | 1.04 m |
| 20° | 0.68 m | 1.37 m | 2.05 m |

**Two independent reasons the camera's yaw was likely wrong on this robot:**

1. **It was never seeded.** Limelight's documented seeding window is **DISABLED**, but the gate
   used `DriverStation.isAutonomous()`, which reads the Driver Station *mode bit* only. On a real
   field the FMS happens to sit in auto mode pre-match, so seeding worked by accident. In the pits
   or in practice with the DS in Teleoperated, mode 1 **never ran** — the camera's internal IMU
   was never given the Pigeon's heading, while mode 4 was already live and trusting it.
   *Testable prediction that matches the report: the map is worst in teleop-only practice, and
   noticeably better right after an auto run.*

2. **The camera is mounted upside down.** `Hardware-Data-Sheet.md` §2 records the Limelight as
   *"physically flipped 180° (upside down)"* (user-stated 2026-07-17). Limelight's MegaTag2 doc
   states verbatim: *"To use the LL4 IMU, currently your LL must be mounted in 'landscape' mode."*
   An inverted mount points the internal IMU's yaw axis **down**. The web-UI rotation setting is
   documented as correcting the **image**; nothing in Limelight's docs says it also rotates the IMU
   reference frame.

**Fix applied:** IMU mode **1** unconditionally — *"use external IMU, seed internal IMU"*.
MegaTag2 now solves with the Pigeon yaw we push, and the camera's IMU is kept seeded from it so it
stays sane. The cost is ~20 ms of NetworkTables latency on the yaw (≈1.1° at 1 rad/s, ≈0.08 m at
4 m) — an order of magnitude below the failure modes above.

> Do **not** move to mode 3/4 unless the camera is remounted landscape *and* the seed is verified
> on a disabled robot. Do **not** raise `SetIMUAssistAlpha` as a workaround — it is a tuning knob,
> not a repair for an unseeded or inverted IMU.

### 3.2 The same camera frame was fused over and over (fixed — `5e088f3`)

`LimelightHelpers.getBotPoseEstimate*` reads botpose with `NetworkTableEntry.getAtomic()`, which
returns the **retained** value — same pose, same timestamp — whenever no new camera frame has
arrived. The robot loop runs at 50 Hz; the camera is slower than that and drops frames. So the
same measurement was handed to the Kalman filter several loops in a row.

That is not extra information, it is over-trust: each application corrects the estimate again, so
a sample nominally weighted at 0.7 m acted far tighter. The fused pose snapped to, and jittered
around, whatever vision last said — and any bias in that solve (see §3.4) got stamped in hard.

**Fix:** track the last fused botpose timestamp; skip anything not strictly newer.

### 3.3 Every solve was weighted the same (fixed — `d4a478e`)

Standard deviations were fixed at `(0.7, 0.7, 9999999)`, so a grainy single tag 6 m away moved the
estimate exactly as hard as a crisp two-tag solve at 2 m. Every comparison repo scales with
distance; WPILib's own guidance is *"scale the vision x and y standard deviation by distance from
the tag"*. The 0.7 in the Limelight sample is a starting default, not a shipped value.

Now `stddev = 0.08 · avgTagDist² / tagCount`, chosen so **one tag at 3 m reproduces the old 0.7
exactly** — mid-range trust is unchanged, close solves count for more, far solves for less:

| solve | old | new |
|---|---|---|
| 1 tag @ 1.5 m | 0.70 | 0.18 |
| 1 tag @ 3 m | 0.70 | 0.72 |
| 1 tag @ 5 m | 0.70 | 2.00 |
| 2 tags @ 4 m | 0.70 | 0.64 |

Also: `isInsideField()` was applied **only to the MegaTag1 boot seed**, never to continuous
MegaTag2 fusion, so a solve placing the robot outside the field still dragged the estimate. Both
layers now use it.

Theta stays at 9999999 (gyro owns heading). That is the universal MegaTag2 pattern — 3476 uses
9999999, 1678 uses 99999, StuyPulse uses 694694 — **keep it**.

> **Expect vision to correct more GENTLY than before, and do not read that as failure.** Two
> changes push the same way: the stale-sample fix cut how often a measurement is applied by
> roughly 2–3× (50 Hz loop vs camera frame rate), and the estimator is no longer over-driven by
> duplicates. The pose should now drift toward vision smoothly instead of snapping to it.
>
> **Tuning direction**, once the mount pose is measured: **lower** `VISION_XY_STDDEV_BASE` to
> trust vision more, **raise** it if the pose is twitchy. Note that 0.08 was chosen to preserve
> the old mid-range weighting, which makes it roughly 4× *more* trusting than the ~0.3 base the
> surveyed top-team code uses — a deliberate choice, but worth knowing before you tune.

### 3.4 The camera mount pose is not set (NEEDS THE TEAM)

**This is the single largest remaining source of a systematically wrong pose, and it is blocked on
a measurement.**

MegaTag reports where the **camera** is. The Limelight converts that to where the **robot** is
using the camera→robot transform. Left at zeros, the "robot position" you get *is the camera's
position* — off by the mount offset, and any unmodelled camera **pitch** becomes an error that
grows linearly with tag distance. No code-side gate can catch this, because the wrong answer is
perfectly self-consistent: MegaTag1 and MegaTag2 will agree with each other and both be wrong.

`setCameraPose_RobotSpace` has existed in the vendored `LimelightHelpers` all along with **zero
callers**. 254 (`VisionIOHardwareLimelight`), 1678 (`LimelightSubsystem`) and StuyPulse
(`LimelightVision`) all push it from code at construction, so it lives in version control and
survives a reflash, a config reset or a spare-camera swap — none of which the web-UI copy survives.

**What was added:** `Vision/cameraPoseRobotSpace` publishes what the camera is *actually* using —
all zeros means never configured, which answers "is it set?" without opening the web UI. Plus six
`LL_CAM_*` constants and a `LL_PUSH_CAMERA_POSE` flag, **default off**, that re-assert the
transform every loop (so it also survives a camera reboot mid-match).

It ships off on purpose: the push overrides the web UI, so enabling it with placeholder zeros
could break a camera that *was* configured correctly.

**How to measure — 10 minutes, robot on the floor, disabled:**

| Value | Meaning |
|---|---|
| `forward` | +X, robot rotational **centre** → camera **lens**, toward the robot front (front = the **shooter side**, per `TunerConstants`) |
| `side` | +Y, centre → lens, toward the robot's **left** |
| `up` | +Z, **floor** → lens |
| `roll` / `pitch` / `yaw` | degrees; **pitch matters most** (camera tilted **up** is positive) |

Measure the pitch with a **phone level against the camera's flat face** — do not eyeball it. A 3°
pitch error is ~0.2 m of range error at 4 m. Enter the six numbers, set `LL_PUSH_CAMERA_POSE =
true`, redeploy, then confirm `Vision/cameraPoseRobotSpace` echoes them back.

### 3.5 Auto-aim mixed two frames (fixed — `d4796d2`, team-approved)

`runVisionTargeting()` took the shot distance and aim bearing from the **raw MegaTag1 botpose**,
while `isAimedAtTarget()` judged "am I lined up" against the **fused** pose. Two different frames.
A single-tag MegaTag1 solve can disagree with the fused estimate by several degrees, so the heading
servo chased a bearing computed in one frame and was graded in another — it could report *aimed*
while pointing off target, or never reach the 2° gate at all and hold the drivetrain for the whole
trigger hold.

All three now read `drivetrain.getState().Pose`. Side benefit: the fused pose is odometry-smoothed
between camera frames, so the aim stops jittering at the camera's frame rate.

Guarded by the new `isVisionFresh()` — the fused pose is only trusted for **range** once vision has
actually corrected it within `VISION_FUSED_FRESH_SEC` (0.5 s, matching `TARGET_HOLD_SEC`).
Otherwise it falls through to the existing camera-space SCORE fallback rather than trusting dead
reckoning from an arbitrary origin.

### 3.6 The new diagnostics — how to tell which fault is present

Published every loop under NT table `Vision`.

> **Where to read them:** *not* the driver companion app. Its `nt.ts` has a fixed `TOPICS` table
> and `graphs.ts` only subscribes to `/CompanionTelemetry/{voltage,supply,stator}`, so nothing
> under `Vision/*` or `Limelight/*` reaches it. Use **Shuffleboard**, **AdvantageScope**,
> **Elastic** or **OutlineViewer** — any of them shows arbitrary NT topics. (Adding a Vision
> readout to the companion is a worthwhile follow-up; it lives in a separate repo at
> `..\Helios-2026\driver-companion` and was out of scope here.)
>
> You do not have to watch these constantly: `warnOnHeadingDisagreement` now raises a
> rate-limited Driver Station warning by itself if the tags and the gyro disagree by more than
> 5° for a sustained second.

Graph the first two while driving a lap:

| Topic | Reads |
|---|---|
| `mt1YawMinusGyroDeg` | **The heading-frame discriminator.** MegaTag1 yaw is solved from tag geometry alone — no IMU of any kind — so its gap from the fused heading *is* the heading error. Should sit near 0 and stay there while rotating. |
| `mt1ToMt2Meters` | Gap between the two solves of the **same image**. MegaTag2 differs from MegaTag1 only by using an external heading, so a large gap points at the heading fed to the camera, not at the optics. |
| `visionToFusedMeters` | How far the pose the robot *acts on* sits from what the camera sees. Large and **steady** = systematic bias (prime suspect: mount pose). Large and **twitchy** = a noisy or rejected feed. |
| `cameraPoseRobotSpace` | What mount transform the camera is using. **All zeros = never configured.** |
| `status` | Why this loop's sample was used or not: `FUSING n TAGS` / `STALE FRAME` / `REJECTED: SPINNING` / `REJECTED: OFF FIELD` / `NO TAGS` / `NO CAMERA DATA` / `MT1 BOOT-SEED`. |
| `mt1Pose`, `mt2Pose`, `avgTagDistMeters`, `maxAmbiguity`, `appliedXyStdDev` | Supporting detail. `maxAmbiguity` high on a single-tag solve is the classic "robot teleports" cause. |

**Reading the result:**
- `mt1YawMinusGyroDeg` ≈ 0 **and** `visionToFusedMeters` large and steady → **mount pose** (§3.4).
- `mt1YawMinusGyroDeg` non-zero, worse while rotating → heading frame; re-check §3.1 landed.
- `mt1ToMt2Meters` large → MegaTag2 is projecting from a bad heading.

### 3.7 The boot seed almost never fires on the bench — and it is the ONLY thing that sets X/Y

This is the finding that best explains why the map looks wrong **in the shop**, and it is not
fixed. It survived adversarial verification, which also rejected the obvious fix.

**The MegaTag1 boot seed is the only absolute-pose initializer in the shipped configuration.**
Nothing else sets X/Y from the field:

- The auto chooser defaults to **"None"**, and the only PathPlanner auto on disk is a test auto —
  so `AutoBuilder`'s `resetPose` normally never runs.
- `driveForwardAuto` does not reset pose.
- **MENU is heading-only.** `seedFieldCentric` sets rotation and silently drops X/Y — the code's
  own comment at the `AutoBuilder.configure` call says exactly that.

So the seed is it. And its window is **only the few seconds between code boot and the first
enable** — during which a tag must be within `VISION_SEED_MAX_TAG_DIST_METERS` (5 m) and in view.
On a bench or a practice field, it usually is not. The seed never fires.

What follows is the reported symptom, exactly:

1. X/Y stays at the power-on origin (0, 0) — the blue corner — no matter where the robot is.
2. Field-frame yaw stays arbitrary, **forever**: vision has a theta std dev of 9999999, so it
   never corrects heading, and MENU only re-zeros to the operator perspective.
3. That wrong yaw is then pushed to the camera every loop, so MegaTag2 solves from a wrong
   heading and lands `d·sin(θ)` off — 0.35 m to 2.05 m per the table in §3.1.
4. Whatever survives the off-field guard gets fused into X/Y.

A power cycle recovers it (the latch clears), which is why it comes and goes and reads as flaky.

**The obvious fix is wrong.** Arming on `isEnabled() && isAutonomous()` would leave the seed armed
for the whole of teleop in any session where auto never runs — i.e. all shop practice. The seed's
action is `resetPose`: X/Y **and** yaw, potentially off a single tag. The pose would teleport
mid-drive and take the blue-frame heading servos with it. That is exactly the in-match auto-reseed
the team removed on 2026-07-21, which should stay removed.

**FIXED (`846748f`, team-approved 2026-07-24).** The seed is now armed on
`DriverStation.isDisabled()` rather than the boot latch. It can only ever fire with the robot
**stationary**, and it re-establishes X/Y any time the robot is disabled in front of a tag. The
team's "MENU is the only in-match re-centre" rule still holds: the robot is enabled for the whole
of a match, so the seed is physically incapable of firing then.

*Known and accepted:* the auto→teleop transition and field timeouts are brief disabled windows
where a re-seed can fire. The robot is stationary in both, so it corrects toward truth. The
quality gates — tag distance, the new ambiguity gate (§3.3), in-field bounds — are what stop a bad
solve being what fires. **Do not weaken them.**

---

## 4. On-robot verification — do these in order

1. **Is the mount pose set?** Read `Vision/cameraPoseRobotSpace`. All zeros → §3.4 is your
   remaining pose bug. Measure and enable it.
2. **Did the IMU fix land?** Enable in teleop, drive a lap, watch `Vision/mt1YawMinusGyroDeg`. It
   should stay small and not grow while turning.
3. **Is vision actually fusing?** `Vision/status` should read `FUSING n TAGS` with tags in view,
   not `STALE FRAME` most of the time. Persistent `STALE FRAME` means the camera frame rate is
   below the loop rate — check `Limelight/fps`.
4. **Auto-aim retest (behavior changed).** RT should still acquire, aim and fire; check the
   reported target distance is sane before trusting a shot.
5. **CAN load.** Run a full-effort practice match and watch `CompanionTelemetry/canBusUtilization`.
   Below ~70% there is nothing to do. `canRxErrors`/`canTxErrors` should be 0.
6. **Limelight health.** `Limelight/cpuTempC` and `Limelight/fps` under match load. This is the
   first honest thermal data you will have (see §6).
7. **Hood stall guard (new).** "Hood Stall Cutoff" on the Shooter tab must stay FALSE through a
   full MIN→MAX→MIN sweep in Test mode (DPAD LEFT/RIGHT). A false trip means the thresholds are
   too tight — see §7.2d.
8. **RT fire rate and belt packing (changed).** Spin-up now follows aiming, so phase 2 opens the
   belts (`() -> true`) while the flywheels are starting from *zero* — the belts pack fuel against
   a closed kicker for the whole spin-up, every shot. That window used to be short because the
   wheels were already near speed by phase 2. Confirm the cadence is acceptable **and** watch for
   fuel jamming or a belt stall at the kicker during the first second of a hold.
9. **Shooter PID sliders are now live.** Confirm an edit on the Shooter tab actually takes effect —
   and brief the drive team that a stray drag now changes gains.
### If RT misbehaves — back it out in this order

Six things on the RT/shooter path changed in one session, which makes a bad retest hard to
bisect. Revert one at a time, in this order; each is a single constant or a small block:

1. **`aimAchieved` latch** (`ShooterSubsystem`) — restores flywheel spin-up during the aim.
   Fixes fire rate and belt packing in one step. Do this first.
2. **`AIM_MAX_ROTATION_RATE_RAD_PER_SEC`** back to `1.5 * Math.PI` — restores the old aim slew.
3. **`AT_SPEED_DEBOUNCE_SEC = 0`** — reverts the kicker gate to the raw signal (V1-identical).
4. Only then question the aim-frame unification (`d4796d2`), which is the change most likely to
   have *helped*.

The vision-side changes (stale-sample guard, std-dev scaling, IMU mode, ambiguity gate) are
independent of the shot path — do not revert them while chasing an RT symptom.

### Then

10. Everything still open from the V1 handoff: shooter C follower direction, hood encoder anchors,
   hopper belt direction, slider extend sign, `TAG_LATERAL_OFFSET_SIGN`, heading-servo PID, steer
   supply 30 A watch, ballistics.

---

## 5. Needs the team — cannot be invented

| # | Item | Why it is blocking |
|---|---|---|
| 1 | **Camera mount pose**, six numbers (§3.4) | Highest-value remaining pose fix. Ten minutes with a tape measure and a level. |
| 2 | **Kicker breaker size** (CAN 17) | The VictorSPX has **no current sensing**, so software cannot limit it — the breaker is the only protection. Still an open TODO. See §7.1. |
| 3 | **Pigeon 2 mount orientation** | `TunerConstants.pigeonConfigs = null` → no mount-pose calibration is applied. If the Pigeon is not mounted flat and square, yaw is wrong — and yaw now drives the entire vision solve. Data-sheet open item #14. |
| 4 | **Is the Limelight still mounted upside down?** | Confirms whether the LL4 internal IMU can ever be used (§3.1). Not needed for the current fix, which avoids the internal IMU entirely. |
| 5 | **Limelight firmware version** | Should be LLOS 2026.1+. Check `limelight-knight.local:5801`. |

---

## 6. Limelight — getting more out of it, safely

### 6.1 The thermal work was measuring the wrong sensor

The ~3.75 W / ~74 °C read from `/status` is the **Hailo M.2 module's own telemetry** — a separate
sensor on a fixed-power accelerator with no idle power gating. That is exactly why it reads 74 °C
two seconds after a cold boot and never moves, whatever you do to the pipeline.

It is **not** the camera's board temperature. So the recorded conclusion that "resolution,
framerate and LED state don't affect temperature" was measured on the wrong sensor — that
experiment has not actually been run yet. And the thermal hysteresis latch was gated on a value
that is constant by design, so it could never fire.

**Fixed (`d48833f`):** the Limelight publishes `hw = [cpu temp °C, cpu usage %, ram usage %, fps]`
over NetworkTables — free, no HTTP, no extra thread. All four are now published under `Limelight/*`,
and the over-temp warning and thermal latch run off **CPU temperature**, the sensor that actually
responds to pipeline load. The Hailo reading stays published: robot code can't influence it, but it
*is* the number that tells you whether a physical fix (thermal pad, mount, airflow) worked.

`fps` is the most useful camera-health number on the robot — it drops when the camera struggles or
is throttled, and zero means vision is dead long before anyone notices the pose drifting. It also
replaces the old HTTP staleness gate: NetworkTables **retains** the last `hw` value forever, so a
camera that died mid-match would otherwise keep reporting its final temperature as if it were live.

### 6.2 What the vendor does and does not publish

Be aware of how thin the ground is here:

- The **only** published power figure is *"12 W maximum power consumption"*. PoE is explicitly
  **removed** on LL4 — wire two 18–20 AWG leads to a PDP/PDH slot on a 5 A or 10 A breaker.
- There is **no published LL4 maximum operating temperature** and **no thermal-shutdown threshold**.
  Your `LL_TEMP_WARN/HOT/COOL` values (80/83/78 °C) remain placeholders — now at least on the right
  sensor.
- The quick-start, hardware-comparison table and product page never mention a **fan or heatsink**.
- There is **no published power or temperature figure** as a function of resolution, framerate or
  downscale. The vendor only ever frames downscale and crop as *framerate* levers, never thermal
  ones.
- The vendor **contradicts itself on input voltage** across three of its own pages (5–26 V /
  3.5–24 V / 3.3–24 V). Treat 12 V from the PDH as the only safe reading.
- The Hailo module *"is inoperable with USB power. It requires a stable 12 V source."*

### 6.3 Features worth using — and ones that would hurt

| Feature | Verdict |
|---|---|
| `SetThrottle` while disabled | **Already doing it.** Vendor: 100–200 while disabled, 0 while enabled. Note it makes pose data stale (~1/(N+1) rate) and *"temperature readings are not accurate while throttling is enabled"* — so never build a temperature-triggered auto-throttle loop. |
| `SetFiducialIDFiltersOverride` | **Helps — but load ALL valid field tag IDs, never a narrow subset.** It stops a stray 36h11 tag (a practice tag in the pits, another team's target) from corrupting botpose. Do *not* follow the docs' "one tag per pipeline" advice for localization. Not implemented — see §9. |
| `setPriorityTagID` | Neutral for pose, useful for aiming: it forces `tx`/`ty` onto a chosen tag. Relevant to a real defect — see §9. |
| Fiducial downscale | Run **1× (no downscale)** on this robot. It helps framerate and **hurts** pose precision. |
| Crop windows | **Hurt** multi-tag localization on a single camera — they physically remove tags from the solution. |
| 3D point-of-interest offset | Neutral for pose, genuinely useful and under-used for aiming at something with no tag on it (e.g. the hub centre rather than a tag face). |
| MegaTag1 vs MegaTag2 | Keep MegaTag2 primary, MegaTag1 logged as a cross-check — which is now exactly what the code does. |
| Static IP | Recommended: give the camera **10.97.4.11** and use the IP for the HTTP status URL; keep `limelight-knight.local` for pit/laptop use. mDNS resolution can fail or stall on the field. |
| Mounting | Use metal fasteners as the thermal path. Community reports describe LL4s in PLA mounts overheating quickly. |

### 6.4 The Hailo either/or

The ~3.75 W Hailo draw is a **fixed floor** no robot-code setting can lower, out of a 12 W envelope.
Either keep the module installed and use Hailo-accelerated AprilTags at 1×, or physically remove
the module and reclaim the power. Robot code cannot split the difference. Software can only warn.

---

## 7. Hardware, current and brownout

The short version: **the current limits in this codebase are in good shape** and match the
Hardware-Data-Sheet §7 tiers. No limit was changed. What follows is what is worth knowing.

### 7.1 The kicker is the one motor software cannot protect (documented per team decision)

CAN 17 is a **VictorSPX driving a CIM**. Phoenix 5, brushed, **no current sensing** — a software
current limit is impossible. The breaker is the only stall protection, and its size is still an
open TODO.

- Holding **B** runs it **ungated at 75 % duty** (`INDEXER_SPEED`). A jammed ball means a CIM stall
  (~131 A at 12 V) until the breaker trips.
- The shot paths (RT/RB) gate it on `isFlywheelAtSpeed()`, so it cannot feed into stopped flywheels
  — that is good and worth keeping.
- `intakeFeedCommand` deliberately leaves the kicker off, which removes the old stopped-flywheel
  stall exposure.

Team decision 2026-07-24: **leave the duty as-is, document the risk.** The mitigation is the
breaker size (§5 item 2). The 0.25 s open-loop ramp already softens inrush.

### 7.2 Worst-case current budget

Supply limits, per the config actually in the tree:

| Group | Burst | Sustained |
|---|---|---|
| 4× drive Kraken X60 | 4 × 40 = **160 A** | folds to 4 × 30 = 120 A after 0.25 s |
| 4× steer Kraken X44 | 4 × 30 = **120 A** | flat |
| 4× flywheel Kraken X60 | 4 × 25 = **100 A** | flat |
| Intake roller X44 | 25 A | — |
| Hopper 2× NEO 2.0 | 40 A | — |
| Intake slider NEO 2.0 | 15 A @ stall → 20 A | tiered |
| Kicker CIM | **unlimited** (stall ~131 A) | breaker only |

Driving worst case ≈ **280 A**, which the `TunerConstants` comments already size against the
~290 A that sags a healthy pack to the RIO2 brownout floor. That is a deliberate, on-robot-tuned
choice and is *more conservative* than CTRE's own suggested starting point (70 A → 40 A after
1.0 s) and than 1678/6328. **Do not undo it.**

**RB is bounded. RT is not — and this is the most serious current finding in the review.**

RB composes `lockDriveAndIntake()` immediately, so the drivetrain sits in `Idle` for the whole
hold and flywheel spin-up cannot stack on drive draw. Good.

**RT does not freeze the drivetrain until *after* the aim completes.** Phase 1 is
`drivetrain.aimUntilAligned(...)`, which actively rotates the robot in place at up to
`MaxAbsRotationalRate` (1.5π rad/s) under a P-only heading servo that commands full rate from a
large error — while `visionShotCommand()` has *already* commanded the flywheels to spin up. The
`Idle` freeze only arrives in phase 2, chained by `.andThen(...)`. So a single RT press overlaps:

| Simultaneous, ~0.2 s window after an RT press | Supply |
|---|---|
| 4× drive Kraken (rotate-in-place accel, burst) | 160 A |
| 4× steer Kraken (full azimuth slew) | 120 A |
| 4× flywheel Kraken (spin-up from zero) | 100 A |
| hopper belts once feeding | 40 A |

That is **~380–420 A**, against a `TunerConstants` budget explicitly sized for **280 A** and only
on a fresh pack. This is the worst-case combination the teleop bindings permit, it is reachable by
one button press, and it is most likely late in a match when the pack is weakest.

Not changed here — every plausible fix (cap the aim slew rate, ramp flywheel spin-up, or move the
freeze before the aim) alters tuned shooting behavior. **This needs a team decision**; see §9.

Separately, **B is the one feed control with no drivetrain lockout at all**: `manualRunCommand()`
does not require the drivetrain, so the driver can drive at full stick while running an unlimited,
unsensed CIM. Team decision 2026-07-24 was to leave the duty as-is and document — recorded here so
the *combination* is on the record too, not just the kicker in isolation.

> **Forward-looking risk:** the RT/RB lockouts are `RobotModeTriggers.teleop()`-gated. Autos are
> currently drive-only, so nothing stacks there today. **The first auto that shoots while driving
> removes that protection entirely.** If a shoot-while-moving auto is planned, re-size the current
> budget first — do not discover this at an event.

### 7.2b What was done about it (team-approved 2026-07-24)

Both mitigations were requested and are in `cf536fc`:

1. **Rotation-rate cap for the RT aim servo only**: 1.5π → π rad/s
   (`FieldConstants.AIM_MAX_ROTATION_RATE_RAD_PER_SEC`). The P-only servo commands its cap
   immediately on any large error, so halving the rate quarters the rotational kinetic energy the
   drives must supply — which is where the burst comes from. `rotateToAngle` and
   `searchAndAlignCommand` keep 1.5π; neither ever runs with the flywheels spinning.
2. **Flywheel spin-up is withheld until the aim has been made once** (`aimAchieved`, latched per
   press). The hood still stages during the aim, so it is in position when the wheels come up.

> **This costs fire rate**, and that is a real trade: spin-up now *follows* aiming instead of
> overlapping it, so the first shot of a press is slower. If that is unacceptable on the robot,
> keep the rate cap — the cheaper half — and revert the latch. Retest RT end to end.

### 7.2c A failed motor config used to be silent (fixed — `5e601c1`)

`MotorConfigs.configureTalonFX` discarded every `StatusCode`. An `apply()` that fails — CAN
timeout, controller unpowered or off the bus — leaves that motor with **no current limit** and the
vendor-default **Coast**, silently. Four unlimited flywheel Krakens is precisely this team's
brownout history, and it defeats the whole point of the factory: it made limits mandatory at the
call site but never checked they arrived.

`HopperSubsystem` already checked its `REVLibError` and reported loudly; the hood and the intake
slider discarded theirs, and the TalonFX helper had no result to discard. All of them now report
which CAN ID failed and what protection is missing. Two cases are worse than they look:

- **Hood**: comes up unlimited (NEO 550, ~100 A stall, almost no thermal margin) *and in Coast*,
  which turns the documented "0 V, brake idle holds the hood" safe state into the hood **dropping**.
- **Slider**: silently loses all three documented pulley-snap protections after a pulley has
  already snapped once.

### 7.2d The hood had no stall protection (fixed — `d91dfe8`, team-approved)

Commanded somewhere it physically cannot reach — a bind, a skipped belt, a wrong anchor — the P
term plus `HOOD_RAISE_FF_VOLTS` kept pushing at up to `HOOD_MAX_UP_VOLTAGE` for the **entire
trigger hold**. The existing hard MIN/MAX travel guards do not help: they fire only when the
*mapped* angle says the hood is at a limit, which is exactly what is untrue when the hood is stuck
short of its target.

Now guarded with the intake slider's proven shape: measured current **and** a stopped encoder,
debounced 0.2 s. Detection latches the **direction** that jammed and refuses only that direction,
so the hood can always be driven back off the jam — a plain cutoff would chatter (cut → current
drops → un-cut → push again) and still cook the motor at high duty.

> **On-robot check:** "Hood Stall Cutoff" on the Shooter tab must stay **FALSE** through a full
> MIN→MAX→MIN sweep (Test mode DPAD LEFT/RIGHT). If it trips during a normal move, raise
> `HOOD_STALL_CURRENT_AMPS` first, then `HOOD_STALL_DEBOUNCE_SEC`. All four thresholds are assumed
> starting values.

### 7.2e The sticks drove the robot in Test mode (fixed — `2aa9c6f`, team-approved)

The drivetrain default command had no mode gate, and a default command runs in **every** enabled
mode when nothing else requires the subsystem. So the robot was fully driveable from the sticks in
**TEST mode** — the mode whose bindings exist precisely so people can run the intake, hopper and
hood by hand, with hands in the mechanisms — and in **autonomous** whenever no auto command held
the drivetrain (chooser on "None", or after a path finished).

It now commands zero outside teleop. Costs nothing: no test binding uses the drivetrain, autos take
the subsystem when they run, and disabled already had its own `Idle` binding. Teleop driving is
byte-identical.

### 7.3 Steer stator is at the data-sheet ceiling

`steerInitialConfigs` uses stator **60 A**, which is the §7 **Cap** column, not the Regular tier
(40 A). This was deliberate — it cured visible module lockup on 2026-07-08 and matches the Tuner X
project value. Flagging it only so it is a known, chosen position at the ceiling rather than an
accident. Supply is 30 A (below the 40 A cap) and is the documented watch item.

### 7.4 Data-sheet drift worth correcting

- §7 lists the **kicker (17) as a Kraken X60** with St 30 / Sup 20. It is a **VictorSPX + CIM** and
  has been since 2026-07-16. Anyone sizing from the sheet would assume protection that does not
  exist.
- §7 lists **Hopper A/B/C (20, 21, ?)** as three motors. `CLAUDE.md` records only two installed
  (confirmed 2026-06-16); the third gearbox position is empty.

### 7.5 Voltage-adaptive load shedding — rare, not recommended now

Across targeted searches of published 2024–2026 code, exactly **one** robot sheds drive current on
brownout (TechnoDogs 3707, 2024). The mainstream practice is precisely what Helios already does:
per-motor stator + supply limits with `SupplyCurrentLowerLimit`/`LowerTime` fold-back. Adding a
load-shed supervisor mid-season to a codebase with a regression history is not a good trade. Noted
and deliberately not built.

`RobotController.setBrownoutVoltage(...)` appears in 2026 code with values from 5.5 to 6.3 V, but
lowering the brownout threshold trades safety margin for uptime — a team decision, not a code
cleanup.

---

## 8. Efficiency

### 8.1 Telemetry was running on the odometry thread at 100 Hz (fixed — `f4fb8b9`)

`registerTelemetry` runs `Telemetry.telemeterize` **on the odometry thread**, once per odometry
update — 100 Hz here. CTRE's own javadoc: *"it is imperative that this function is cheap... if this
takes a long time, it may negatively impact the odometry of this stack."*

Each call did 7 NetworkTables publishes, 6 `SignalLogger` struct writes and 12 `Mechanism2d`
ligament updates — roughly **2500 operations per second on the thread the pose estimate depends
on**, in a codebase whose top complaint is a bad pose.

Nothing downstream consumes it faster than 50 Hz (robot loop 50 Hz, companion field map 30 fps), so
the body is now gated to 50 Hz using the state's own timestamp — no clock call. Odometry still
integrates at the full 100 Hz; only reporting is thinned.

### 8.2 CAN bus is now measured (added — `f4f7aa5`)

18 motor controllers, 4 CANcoders and a Pigeon 2 on **one roboRIO CAN 2.0 bus** at 1 Mbit/s, and
nothing was watching it. A saturated bus delays the swerve's own position/velocity frames, which
degrades odometry — it shows up as a bad pose, not as an obvious CAN error.
`CompanionTelemetry/canBusUtilization` plus REC/TEC/TxFull/BusOff are now published from
`PowerTelemetry`'s existing 10 Hz Notifier.

**This is deliberately a measurement, not an optimization** — see §9 for why the obvious next step
is not safe to do blind.

### 8.3 Keep `PowerTelemetry`'s signal-rate management

`BaseStatusSignal.setUpdateFrequencyForAll(10.0, allSignals)` at `PowerTelemetry.java:119` is a
large existing CAN saving across 52 signals on 13 TalonFXs. **Do not remove it as "unnecessary".**

### 8.4 Known, deliberately unfixed

- **`getState()` vs `getStateCopy()`.** The main loop calls `getState()` 18 times and
  `getStateCopy()` zero times; CTRE documents `getStateCopy()` as the thread-safe accessor for
  exactly this cross-thread read. In practice `Pose2d` is immutable so there is no torn *value* —
  the exposure is field-to-field inconsistency (Pose from update N, Speeds from N+1), which at
  100 Hz is millimetres. An 18-call-site change that allocates per call, for a millimetre-scale
  benefit, is not a good trade against the No-New-Issues rule. Recorded, not done.
- **`SignalLogger.start()`** runs unconditionally and writes `.hoot` files to the roboRIO forever.
  Over a season this can fill the RIO's flash. Check free space periodically, or plug in a USB
  stick so logs land there.
- Redundant per-loop NT writes (`setLEDMode_ForceOff`, and formerly `SetIMUMode`) are real but
  negligible — a handful of small values at 50 Hz. Not worth a change.

---

## 9. Recommended, not done — needs a decision

These are real, but each either changes tuned behavior or optimizes something not yet measured.

1. **Primary-tag-only classification.** `ShooterSubsystem.periodic()` classifies using
   `getFiducialID()`, which is whichever tag the **camera** picked as primary — normally the
   largest in frame. If a FEED tag is bigger than a SCORE tag, `seesScoringTag()` is false and the
   A / DPAD-UP search-align refuses to lock onto a scoring tag that is plainly visible.
   *Minimal fix:* read `getRawFiducials()`, find an own-alliance SCORE tag, and call
   `setPriorityTagID` with it — the camera then reports that tag as primary and `tv`,
   `getFiducialID` and `getTargetPose3d_CameraSpace` all follow, so nothing else has to change.
   Not done because it touches the tuned aiming path and was not explicitly requested.

2. **`SetFiducialIDFiltersOverride` with the full valid tag list.** Safe and useful if driven from
   the WPILib `AprilTagFieldLayout` rather than a hand-typed list (a wrong list blinds the camera).

3. **Slowing SparkMax status frames.** REVLib 2025+ made SPARK status frames on-demand — enabled by
   a getter call at their *default* period — so `PowerTelemetry`'s 10 Hz polling silently enables
   four ~10 ms frames per SPARK. Setting explicit `*PeriodMs(100)` would be a real CAN saving.
   **Not safe to do blind:** the intake slider's stall protection reads `getOutputCurrent()` and
   encoder velocity **at loop rate**, and slowing those to 10 Hz would break the 0.05 s
   current-stall cutoff that exists because a pulley already snapped. Measure `canBusUtilization`
   first; if it is under ~70 %, do nothing.

4. **Camera-loss watchdog on the `hb` heartbeat.** Partly covered already: the stale-sample guard
   stops re-fusing dead frames, `isVisionFresh()` stops the shooter trusting a dead pose, and
   `Limelight/fps` makes it visible. A dedicated `hb` watchdog would be belt-and-braces.

5. **Consensus gate on the driver's MENU re-seed** (1678's pattern: require N consecutive frames
   where vision agrees with odometry before accepting). Today one button press re-zeros heading
   unconditionally. The in-match auto re-seed removed on 2026-07-21 should **stay** removed.

6. **`optimizeBusUtilization()`** on the non-swerve TalonFXs (4 flywheels, intake roller) only —
   never blanket across the swerve motors, CANcoders or Pigeon. Again, only after measuring.

7. **RT/RB strands a deployed intake.** Both shot groups *require* `intakeSS` for the whole hold
   and run `kCancelIncoming`, so pressing RT or RB with the intake out cancels the LT group and
   then **silently rejects** the `stowCommand` that `onFalse` schedules — the slider stays
   extended. Survived adversarial verification. The fix is not a one-liner (appending a retract to
   the shot group changes shot timing), so it needs a deliberate design decision. Workaround
   today: press **X** after the shot to stow.

8. **`kSlipCurrent = 120 A` is probably not the real slip point.** It is copied into the drive
   stator limit by the Phoenix swerve factory. Verified as very likely wrong, but the correct
   value depends on **robot mass**, which is still an open measurement — and changing it naively
   would regress the traction behavior the drives are tuned around. Measure the robot, then
   revisit. Do not guess.

9. **`.withJoystickReplay()`** in `Robot.java` costs ~43 hoot log entries and ~65 HAL calls every
   20 ms loop, most of them for permanently-empty joystick ports. Confirmed, but low severity —
   and deleting it removes a real replay capability. Worth doing only if loop overruns show up.

### Operational notes (no code fix)

- **The MegaTag1 boot seed is latched off by ANY enable — including a pit test.** `m_hasBeenEnabled`
  is set by `DriverStation.isEnabled()`, which is true in teleop, auto **and test**, and is never
  cleared. **Restart robot code (or reboot the roboRIO) before a match** if the robot has been
  enabled in the pit. This is the behavior the team asked for on 2026-07-21.

  **This is bigger than an operational note — see §3.7.**
- **`HopperSubsystem.feedShooterCommand`'s javadoc was wrong** and is now corrected: the kicker
  gate is **speed only** (`isFlywheelAtSpeed`), not `isReadyToShoot`. The kicker will fire with the
  hood wherever it happens to be, including in the pit under test-mode Y. Tighten the call sites
  once the hood feedback is trusted.

### Explicitly rejected

- **A translation-jump gate on vision.** Tempting, and wrong here: if the pose is *already* wrong,
  a jump gate prevents the very correction that would fix it. 254 gates on **yaw** disagreement,
  not translation — and with IMU mode 1 MegaTag2's yaw is our yaw by construction, so even that
  gate is now meaningless.
- **A hard max-tag-distance gate on the MegaTag2 path.** Redundant now that std dev scales with
  distance² — a 6 m solve already gets a 2.9 m std dev and is effectively ignored.
- **Raising `SetIMUAssistAlpha`.** A tuning knob, not a repair.
- **`CANSparkMaxUtil` / `setPeriodicFramePeriod(kStatus3, 65535)`.** Folklore for REVLib 2026 —
  do not port it in.

---

## 10. What the research validated

Worth saying plainly: most of this codebase held up. The survey compared against 254, 1678, 3476,
6328 (2025 and 2026), 2910, StuyPulse, 581 and 696.

- Gyro-owned heading with a huge theta std dev — **universal practice, keep it**.
- Per-motor stator + supply limits with fold-back — **mainstream practice**, and Helios's numbers
  are more conservative than CTRE's own starting point.
- `SupplyCurrentLowerLimit` correctly **below** `SupplyCurrentLimit` — a published repo (696) ships
  it backwards, which silently disables fold-back. Helios has it right.
- Command-factory subsystems, supplier-based cross-subsystem coordination instead of static flags,
  the `MotorConfigs` factory that makes a limit + neutral mode mandatory at the call site — all
  match or exceed what the comparison repos do.
- Rejecting vision when `tagCount == 0` or angular velocity > 360 °/s — matches the vendor example.

The gaps were concentrated in exactly one place: **how vision measurements were weighted, gated,
and what heading they were solved against.** That is now fixed, apart from the mount pose.

---

*Developed by Team 9704 with Claude Code.*
