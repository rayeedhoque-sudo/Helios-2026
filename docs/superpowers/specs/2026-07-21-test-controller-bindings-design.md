# Design — Test-mode mechanism bindings (single controller)

Date: 2026-07-21
Status: implemented, committed (`78b9c71`); amended same day (see below)

> **AMENDMENT (same day, 2026-07-21):** DPAD LEFT/RIGHT changed from a snap-to-MIN/MAX
> press into a hold-to-jog control, 5 deg/sec, team request. The "Implementation shape"
> bullet and table row below describing the original snap behavior are corrected in place
> (not preserved as history — this is a same-day amendment before the design ever shipped
> to drivers, not a post-hoc correction of a shipped decision). See the updated bullet and
> row for the actual (jog) behavior.

## Problem

The driver has two Xbox/XInput pads: a PowerA "Xbox 360 Controller for Windows" (currently
connected, has the match bindings) and a Microsoft "Xbox One Controller" (previously
connected). The team wants per-mechanism test binds — one button per small mechanism
(rollers, slider, individual hopper belts, kicker, etc.), not just one bind per subsystem —
usable during pit/bench checkout, **without any risk of interfering with a real match or
the driver**.

## Decision: one controller, mode-gated

Confirmed via `Robot.java`: `CommandScheduler.getInstance().run()` executes in
`robotPeriodic()`, i.e. every DS mode including Test, and `testInit()` only calls
`cancelAll()` once. The robot is only ever in one DS mode at a time. So a single physical
controller (port 0, the existing `joystick2`) can safely carry two independent binding
sets, gated by `RobotModeTriggers.teleop()` / `RobotModeTriggers.test()` (both confirmed
present on the installed WPILib 2026.2.1 `wpilibNewCommands` jar via `javap`, along with
`Trigger.and(BooleanSupplier)`). Only one gate is ever true at a time, so the same
physical button safely means two different things depending on DS mode — no second
controller required, and no risk of a bench-test button firing during Teleop/a match.

**Rejected alternative**: two controllers (test binds on a second `CommandXboxController`
port 1), which would leave match bindings completely untouched. Rejected by the team in
favor of the single-controller convenience; the trade-off is a larger diff (every existing
match binding gets a `.and(RobotModeTriggers.teleop())` guard added).

## Scope decisions (from user interview)

- **Drivetrain / SysId: excluded.** `CommandSwerveDrivetrain` already has Tuner
  X-generated `sysIdQuasistatic`/`sysIdDynamic` routines (translation + steer), currently
  unbound. These characterize the drivetrain by moving the whole robot through voltage
  ramps — a different risk class than a stationary bench mechanism, and not something the
  team asked for. Left unbound; a separate task if ever wanted.
- **Default drive command: untouched.** Not gated by a button/trigger, so out of scope for
  this change. Sticks already drive the robot in Test mode today (pre-existing behavior,
  not introduced by this change) — flagged to the team, not changed.
- **Intake jackshaft: NOT a separate mechanism.** Team confirmed during interview: the
  SparkMax at CAN 22 (in code today as `intakeSliderMotor`, driven by
  `extendSliderCommand()`/`retractSliderCommand()`) **is** the jackshaft — there is no
  separate 3rd intake motor. This means `docs/handoff-2026-07-21.md` issue #1 ("intake
  jackshaft motor — BLOCKED on team answer... needs 24+") is **stale**: the jackshaft
  already has a CAN ID and is already in code under the "slider" name. Not corrected as
  part of this change (out of scope) — flagged for a follow-up doc update.
- **No vision/auto-aim in any test bind.** All shooter test binds use fixed setpoints
  (existing `MIN_ANGLE`/`MAX_ANGLE`/`RB_FEED_ANGLE`/`RB_FEED_SURFACE_SPEED` constants),
  never `visionShotCommand()` or anything that reads the Limelight.
- **Hopper belts: both per-motor AND together.** The existing `directionTest(motorA)`
  diagnostic (built to isolate a fighting-inversion fault) gets its own bind per motor,
  **plus** a third bind that runs both belts together for normal-operation checkout.
- **Shooting test bind must actually fire fuel.** A flywheel-only spin test doesn't
  exercise the belts/kicker, so it doesn't verify a ball would actually launch. The test
  fire bind reuses the exact same manual/fixed pattern as the match RB binding (hood
  `RB_FEED_ANGLE`, flywheel `RB_FEED_SURFACE_SPEED`, kicker gated on
  `isFlywheelAtSpeed()`) — proven values, no invention, no vision.

## Implementation shape

Zero new subsystem methods. Every test bind reuses an existing public command factory.
Trigger verb depends on how each underlying command already terminates — spelled out
explicitly here so implementation has zero ambiguity to resolve:

- **`whileTrue` only** (LT, RT, LB, RB, X, B, Y): these wrap existing `runEnd`-style
  commands that already stop themselves in their own end-lambda when the scheduler cancels
  them on button release. No separate `.onFalse()` needed — matches the existing pattern
  for e.g. `joystick2.b().whileTrue(hopperSS.manualRunCommand())` in match bindings. Y adds
  `.onFalse(shooterSS.stopShooterCommand())` anyway, purely to mirror the existing match RB
  binding's belt-and-suspenders pattern exactly (RB does the same on top of
  `feedAngleShotCommand()`'s own end-lambda).
- **`onTrue`, self-terminating, no `.onFalse()`** (DPAD UP, DPAD DOWN): `extendSliderCommand()`/
  `retractSliderCommand()` are stall-cutoff/timeout sequences that run to their own natural
  completion regardless of how long the button is held — same verb already used for the
  DPAD ±90 `rotateBy` binds. A short press is enough; holding does nothing extra.
- **`whileTrue`, no `.onFalse()`** (DPAD LEFT, DPAD RIGHT): a `shooterSS.run(...)` command
  (never finishes on its own) that, every loop while held, advances `desired_Angle` toward
  MIN_ANGLE/MAX_ANGLE through a shared `SlewRateLimiter` (5 deg/sec) via the existing
  `setDesired_Angle()` — still fully closed-loop and clamped, NOT the raw open-loop hood
  jog removed 2026-07-16 (that bypassed position feedback entirely, which is why it was
  unsafe; this rate-limits the *setpoint*, so it can never be driven past a limit no matter
  how long it's held). `.beforeStarting()` resets the limiter to the CURRENT setpoint
  (`getDesiredAngle()`) each time the hold starts, so direction changes continue smoothly
  with no jump. Release freezes the setpoint wherever it got to — deliberate, so the hood
  can be parked at a precise angle to read the encoder anchors (handoff on-robot verify
  item 2) — so there is no `.onFalse()` reset.

| Button | Mechanism | Command |
|---|---|---|
| LT (hold) | Intake rollers IN | `intakeSS.rollerTestCommand(true)` |
| RT (hold) | Intake rollers OUT | `intakeSS.rollerTestCommand(false)` |
| DPAD UP (press) | Slider/jackshaft extend | `intakeSS.extendSliderCommand()` |
| DPAD DOWN (press) | Slider/jackshaft retract | `intakeSS.retractSliderCommand()` |
| LB (hold) | Hopper belt A alone | `hopperSS.directionTest(true)` |
| RB (hold) | Hopper belt B alone | `hopperSS.directionTest(false)` |
| X (hold) | Hopper belts A+B together | `hopperSS.intakeFeedCommand()` |
| B (hold) | Kicker alone | `hopperSS.kickerTestCommand()` |
| Y (hold) | Manual test-fire (hood+flywheel+belts+kicker) | `shooterSS.feedAngleShotCommand().alongWith(hopperSS.feedShooterCommand(() -> true, shooterSS::isFlywheelAtSpeed))`, `.onFalse(shooterSS.stopShooterCommand())` |
| DPAD LEFT (hold) | Jog hood DOWN toward MIN, 5 deg/sec (flywheels off) | `shooterSS.run(() -> shooterSS.setDesired_Angle(hoodJogLimiter.calculate(MIN_ANGLE)))`, reset on start |
| DPAD RIGHT (hold) | Jog hood UP toward MAX, 5 deg/sec (flywheels off) | `shooterSS.run(() -> shooterSS.setDesired_Angle(hoodJogLimiter.calculate(MAX_ANGLE)))`, reset on start |

Free/unused in Test mode: A, Start, Back, LSB, RSB.

Note: DPAD UP/DOWN (intake test) and DPAD LEFT/RIGHT (hood test) share physical buttons
with DPAD UP (search-align) and DPAD LEFT/RIGHT (rotate ±90) in Teleop — safe only because
`RobotModeTriggers.teleop()`/`.test()` are mutually exclusive, never both true.

All bindings live in a new `configureTestBindings()` method in `RobotContainer.java`,
called once from the constructor alongside `configureBindings()`, each line ending in
`.and(RobotModeTriggers.test())`.

Every existing binding in `configureBindings()` (13 lines) gets `.and(RobotModeTriggers.teleop())`
inserted immediately after the button selector and before the action verb
(`.whileTrue`/`.onTrue`/`.toggleOnTrue`) — a single uniform mechanical edit, no change to
any command composition or logic. `RobotModeTriggers.disabled()` (the existing idle-while-disabled
binding) is untouched — orthogonal to this change.

## Files touched

- `src/main/java/frc/robot/RobotContainer.java` — teleop guards on existing binds, new
  `configureTestBindings()` method.
- `driver-companion/src/renderer/panels.ts` — new CONTROLS panel group documenting the
  Test-mode map (same hand-sync convention already used for the match bindings).

## Verification

- `gradlew build` + all existing tests green.
- Read-through diff confirming every match binding's logic (not just presence) is
  byte-identical in Teleop — only the trigger gate changed.
- On-robot / bench: DS must be switched to Test mode AND enabled for any test bind to
  actuate (same enable/disable HAL safety net as every other command in this codebase).

## Deliberately not done

- Not updating `Hardware-Data-Sheet.md` or `handoff-2026-07-21.md` to reflect the
  jackshaft/slider finding — flagged to the team as a separate follow-up, not bundled into
  this change.
- Not adding a "stop everything" panic bind — every test bind already stops on release
  (hold-to-run), so a dedicated stop-all would be redundant.

Developed by Team 9704 with Claude Code.
