# Helios (FRC 9704) — Power-Limiting Review

**Date:** 2026-07-17
**Scope:** Read-only review of the whole `Helios-2026` robot codebase for places electrical power (battery draw / brownout risk / motor heat / breaker load) could be *further* limited or better coordinated.
**Status:** Report only — nothing was implemented.
**Method:** 6 review dimensions (static limits, dynamic/variable limits, teleop coordination, autonomous/disabled, duty/ramp/neutral, peak budget) reviewed at high effort, then **all 25 candidates adversarially verified against the actual code**. 21 killed, 4 survived.

> ⚠️ Amp figures below are **uninstrumented reviewer estimates** for prioritization, not measured values. Confirm on the robot with `PowerTelemetry` before acting on any of them.

---

## Bottom line

The codebase is already unusually power-conscious (see inventory below). The verification was decisive: **21 of 25 candidates were rejected** — most because they would *undo a documented on-robot fix*. The survivors converged: three independent dimensions plus their three verifiers all landed on the **same** idea — a battery-voltage-adaptive load-shed supervisor. That is exactly the two levers the request named: *variable current limits* + *disabling subsystems while others work*.

- **1 opportunity worth building** (high ceiling, high effort): voltage-adaptive load-shed supervisor.
- **1 cheap consistency nit** (low value, near-zero risk): hopper belt open-loop ramp.
- **Everything else:** already handled, or real-but-not-worthwhile (documented below so the good decisions don't get undone).
- **Gating question first:** confirm the brownouts still reproduce on a *healthy, load-tested* battery — the dominant documented cause is battery health, not software.

---

## Already in place (the floor — do NOT re-recommend these)

| Load | Limit / mitigation | Location |
|---|---|---|
| Drive Kraken X60 ×4 | stator = `kSlipCurrent` 120 A; supply 45 A → **30 A fold-back after 0.5 s**; Brake | `TunerConstants.java:84-91` |
| Steer Kraken X44 ×4 | stator 60 A; supply 40 A (flat); Brake | `TunerConstants.java:100-106` |
| Flywheel Kraken X60 ×4 | stator 80 A; supply 25 A (flat); Coast; "off" = `CoastOut`, not `VelocityVoltage(0)` (avoids regen brake) | `ShooterSubsystem.java:155-165`, `:667` |
| Hood NEO 550 | `smartCurrentLimit(20)`; Brake; ±12 V clamp; stow interlock | `ShooterSubsystem.java:169-172`, `:684-690` |
| Intake roller Kraken X44 | stator 40 A; supply 25 A; Coast; 0.25 s open-loop ramp; `StaticBrake` on stow | `IntakeSubsystem.java:77-90` |
| Intake slider NEO 2.0 | **tiered** `smartCurrentLimit(15,20,1000)`; 0.25 s ramp; Brake; current+encoder stall cutoff | `IntakeSubsystem.java:105-109` |
| Hopper NEO 2.0 ×2 | `smartCurrentLimit(20)`; Coast | `HopperSubsystem.java:48-52` |
| Kicker Victor SPX / CIM | no current sensing possible; low duty (0.3) during intake; 0.25 s ramp; Coast | `HopperSubsystem.java:57-59` |
| Teleop drive | global 0.8× speed cap + 0.8 rot/s | `RobotContainer.java:34-36` |
| Teleop translation | per-axis slew limiter 4.0/s | `RobotContainer.java:54-56` |
| Intake-live | translation halved (0.5×) while rollers spin | `RobotContainer.java:62-65` |
| Shot (RT/RB) | drivetrain **frozen** (`Idle`) + intake locked, via `kCancelIncoming` | `RobotContainer.java:305-310` |
| Hood | not driven into stow region while flywheels spin | `ShooterSubsystem.java:684-688` |
| Kicker | gated on `hasShotTarget` / `isReadyToShoot` | `RobotContainer.java:283`, `HopperSubsystem.java:121` |
| Auto drive-forward | aborts on measured drive/steer stator stall | `CommandSwerveDrivetrain.java:673-712` |
| Auto trajectories | current-limited, no-slip envelope (`driveCurrentLimit 40`, `wheelCOF 1.2`) | `deploy/pathplanner/settings.json:23-24` |
| SysId | dynamic step reduced to 4 V "to prevent brownout" | `CommandSwerveDrivetrain.java:85` |
| Monitoring | 10 Hz stator/supply/temp/volts on all 17 motors + battery V | `PowerTelemetry.java` (publish-only) |

---

## ⭐ Opportunity worth building: battery-voltage-adaptive load-shed supervisor

**Priority:** high ceiling, **but high effort / high risk — not a cheap config win.** Verified REAL by three independent verifiers, each refining it from "high" toward "medium."

**Gap:** `PowerTelemetry.java:162` already samples `RobotController.getBatteryVoltage()` and every motor's supply current at 10 Hz, and **acts on none of it**. Every supply cap in the code is flat or time-based — never *battery-state* responsive. This is the one predictive lever the codebase lacks, and it targets the team's stated late-match brownout history. It is exactly the "variable current limit" + "disable subsystems while others work" the review was asked to find.

**Mechanism (report only):**
- A small supervisor (in `Robot.robotPeriodic` or a tiny subsystem) filters battery voltage (median-of-N to reject single-sample sag), and when the rail holds below a threshold *under load*, progressively sheds load, recovering with hysteresis.
- Plumbing already exists: `RobotContainer.java:34` `MaxSpeed` is a mutable field the drive suppliers (`:157-160`) read live inside the lambda each loop — mutating/scaling it throttles drive with **zero rebinding**.
- **Shed order (least-critical first):** (1) drive translation authority → (2) intake rollers + hopper belts → (3) flywheel supply **last** (scoring mechanism, protect longest).

**Hard caveats (the verifiers insisted):**
- The 10 Hz display Notifier **cannot catch a tens-of-ms transient** — the roboRIO FPGA hardware brownout owns that floor. This is a *coarse governor* for sustained pin/push sag only.
- It **cannot save a weak/dead battery** — the dominant failure mode. It only buys margin on a *marginal* pack.
- Must be hysteretic + debounced, or a legitimate accel/spin-up sag cuts drive mid-defense or bogs the shot it's firing.
- Trigger on the **voltage threshold**, not `isBrownedOut()` (that only trips after the fact).

**Starting thresholds to tune on robot:** trip ~7.0 V under load (debounced > 0.2 s), recover ~7.6 V; shed one tier at a time.

---

## Cheap consistency nit: hopper belts have no open-loop ramp

**Priority:** low. **Verdict:** REAL but marginal.

`HopperSubsystem.java:48-52` sets only `smartCurrentLimit(20)` + `kCoast` — no `openLoopRampRate`, while every sibling spun-under-load motor has a 0.25 s ramp (roller `IntakeSubsystem.java:87`, slider `:107`, kicker `:59`). `indexFuel()` steps both belts `0 → 0.5` instantly. The `smartCurrentLimit(20)` already firmware-clamps each NEO, so the ramp only softens the *rise* into an already-clamped limit, and the belts never run coincident with another stepping high-draw load. Worth it as pattern-consistent inrush smoothing, near-zero risk — not a meaningful brownout lever.

---

## Considered and rejected — with reasons (protects existing good decisions)

Several "obvious" ideas would **re-break something already fixed on the robot.** Do not chase these.

| Candidate | Verdict | Why it fails |
|---|---|---|
| Supply fold-back on **steer** ×4 | ✗ not worthwhile | 40/20 conservative caps *already* starved fast module swings — "modules visibly locked mid-swing" (`TunerConstants.java:96-99`, 2026-07-08). Folding to ~20 A re-creates that regression. Steady azimuth draw is near-zero, so it saves ~nothing normally. |
| Supply fold-back on **flywheels** ×4 | ✗ not worthwhile | An at-speed flywheel draws only single-digit windage amps — 100 A is a paper ceiling, not real load. Any fold-time short enough to bite starves the between-ball re-acceleration the **80 A stator raise was made to cure** ("barely left the robot", 2026-07-17). |
| Thermal fold-back on flywheels / hood | ✗ not worthwhile | Kraken/Phoenix firmware already self-derates on temp; software duplicates it + adds a false-trigger mode that silently undoes the anti-bog raise. No burnout history — the failure is brownout (a supply problem). |
| Raise flywheel supply during the shot-freeze | ✗ not worthwhile | Not battery-neutral: during the freeze the bus is *near-idle* (the weak-battery moment), so +28 A is real added draw. 4 blocking `configurator.apply()` on the freeze edge also risks a loop overrun. |
| Rotation slew limiter | ✗ not worthwhile | Deliberate design ("turning stays crisp", `RobotContainer.java:52-53`). Steer/drive already supply-capped; a slew can't lower those ceilings, only de-coincide them — at the cost of defensive dodges. |
| Lower **roller** supply 25→15 A | ✗ not worthwhile | 40/25 was *raised from* 30/15 for grab torque (2026-07-10); duty is trending up. 15 A leaves ~3 A over draw and clips un-jam torque. ~10 A on a rare single-motor jam. |
| Lower **kicker** duty 0.75→0.6 | ✗ not worthwhile | Fires only into *already-spinning* flywheels (`isReadyToShoot` gate); the stall-risk case already uses 0.3. Duty here is a feed-reliability/scoring knob, not safety — lowering it risks missed shots for a few amps. |
| Gate hood PID when in-tolerance | ✗ not worthwhile | Already 20 A-capped; a settled P-controller outputs ~0. Single-digit-amp saving vs. real accuracy risk (hood sags) and interlock-deadlock risk. |
| Flywheel closed-loop spin-up ramp | ✗ not worthwhile | Supply is *already* the 25 A cap regardless of ramp; a ramp holds the 100 A window **longer**, not shorter, and the shot-freeze already lands it on an idle bus. |
| Manual-hopper (B) drive slowdown | ✗ not worthwhile | Drive already supply-fold-backed; the uncapped wildcard is the CIM, which a translation cut doesn't touch. B is a rare, stationary jam-clear. |
| A (search-align) + B (manual hopper) interlock | ✗ not worthwhile | Ordinary full-speed driving already stacks with B with no lockout; the slow in-place search rotate is a smaller load. Would remove a legal manual jam-clear option. |
| RT phase-1 spin-up ↔ aim stagger | ✗ not worthwhile | Both loads already config-capped; in-place aim is the lowest-force maneuver, and the heading servo's current peaks at *start* and decays to ~0 by the shot moment (then freezes). Deferring spin-up just lengthens every shot. |
| SparkMax voltage compensation | ✗ leave off | Would *increase* draw under sag — opposite of the desired lever. Correctly left off. |
| Auto acceleration cap | ✓ already handled | `settings.json:23-24` current-limits auto trajectories; drive fold-back applies in auto. |
| Auto cross-subsystem stagger / NamedCommands | ✗ YAGNI | Zero shooter/intake autos exist (only drive-forward). Build coordination *when* a shoot-on-move auto is written. |
| Match-phase-aware bias (`MatchStatus`) | ✗ not worthwhile | Rides on the unbuilt supervisor; keys off DS-approximate match time; would shift load-shed into peak-scoring endgame. |
| Pause `PowerTelemetry` while disabled | ✗ not worthwhile | Saves mA; blinds the pre-match battery/temp readout drivers watch while disabled. |

---

## Gating question before any of it (adversarial view)

The dominant documented cause of the brownouts is **battery health, not software.** The 2026-07-12 capture that drove the drive fold-back noted the pack *rested* at 11.5 V and sagged ~4 V under load — a tired battery.

**Confirm first:** do the late-match brownouts still reproduce with a freshly-charged, load-tested battery (internal resistance < ~0.020 Ω)?
- If only on tired packs → the supervisor buys marginal seconds; **battery triage is the real fix.**
- If on a *healthy* pack → the supervisor is genuinely worth building.

---

*Developed by Team 9704 with Claude Code.*
