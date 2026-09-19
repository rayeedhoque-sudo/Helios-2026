# Helios V2 — Current-Limiting & Brownout Review

**Date:** 2026-07-25
**Scope:** Read-only review of `C:\FRC\Helios-2026-V2` for places the code could further limit
current draw (variable limits or otherwise) and better resist brownouts late in a match.
**Status:** Report only — **no code was changed.**
**Companion docs:** `C:\FRC\Helios-2026\docs\power-limiting-review.md` (V1, 2026-07-17) and
`docs/deep-review-2026-07-24.md` §7. This review does **not** repeat either; it covers what is
new or still open in V2.

---

## Read this first: the clock is the wrong trigger

The request names "near the end of the match," and `MatchStatus` now exists and knows the match
phase — which makes a time-gated load-shed tempting and easy to write. **Don't build that.**

END GAME (0:30–0:00) is when both hubs are active and scoring peaks. A supervisor keyed to the
match clock would cut drive and feed authority at exactly the moment points are on the table,
and it would do it on a *fresh* battery just as eagerly as on a tired one — the clock carries no
information about the pack.

Late-match brownouts are not caused by the clock; they are caused by the battery's **state** at
that point in the match (depleted, internal resistance risen). So every trigger below keys off
measured battery state — voltage, current, and internal resistance — and none off match time.
`MatchStatus` stays what it is: a display and a rumble.

---

## The team's answer to the gating question, and what it actually rules out

**Team report, 2026-07-25:** the brownouts happen **near the end of the match, on a fresh battery
that has been on the charger for a long period immediately beforehand.**

This is the V1 review's gating question answered, and it does promote the software work — that
review said explicitly that on a healthy pack the supervisor is genuinely worth building. But it
rules out less than it appears to, and the arithmetic points somewhere specific.

**A freshly-charged battery is not the same as a healthy one.** Charge state and internal
resistance are independent. An aged pack charges to 13+ V happily and then collapses under load,
and a battery taken straight off the charger carries a surface charge that reads high for the
first minute of use and then falls away. "Long charge immediately before" does not exclude a tired
pack — if anything, a pack that takes an unusually long time to charge is itself a mild warning.

**More importantly, depletion is probably not the mechanism at all.** A healthy FRC pack is not
flattened by one 2:30 match — even a sustained 100 A average over 150 s is only about 4 Ah out of
a nominally 18 Ah pack. Teams routinely run a full match and finish resting near 12 V. So "it
browns out by the end" on a genuinely good pack is not a capacity story.

**The arithmetic says it has to be resistance or current.** Brownout is `V_rest − I × R` reaching
the RIO2 floor of 6.75 V. From a ~12 V rest that needs about 5.3 V of sag. At the ~280 A worst
case the current limits in this tree are sized for, that requires `R ≈ 0.019 Ω` — which sits right
on the "retire this pack" line. So one of two things is true, and they need opposite fixes:

1. **The resistance really is that high.** Note this is the resistance of the *whole path*, not
   just the pack: main breaker, SB50 connector, battery lugs, wiring. **This fits "near the end of
   the match" better than anything else**, because those connections heat under sustained current
   and their resistance climbs as they do — a marginal SB50 or a loose lug behaves fine for the
   first minute and progressively worse for the next two. A Battery Beak on a disconnected pack
   would exonerate the battery and miss this entirely.
2. **The actual current is well above what the configured limits suggest.** There is exactly one
   motor on this robot that no software limit governs: the **kicker CIM** (finding 2), ~131 A at
   stall, run at 75 % duty. And endgame is when the most fuel is being fed — i.e. the phase with
   the most kicker activity and the most chances for a ball to jam in it. This fits the reported
   timing too.

**Free physical check, no code, do it at the next brownout match:** the moment the robot comes off
the field, check the SB50 connector, the battery lugs and the main breaker by hand. Warm is normal;
hot, or discoloured, is the answer — and it is hypothesis 1, not a software problem at all.

The re-ranking below follows from this: finding 3 becomes the primary diagnostic (it is measured at
the RIO, so unlike a Beak it *includes* the breaker/SB50/wiring that hypothesis 1 implicates), and
finding 2 rises with it.

---

## Bottom line

The static current limits in this tree are in good shape and were tuned on the robot — nothing
below proposes changing a single one of them. The gap is not in the limits; it is that **the
robot never measures its own total battery draw**, and therefore cannot act on it.

Ranking updated 2026-07-25 after the team's answer above.

| # | Finding | Value | Effort | Risk |
|---|---|---|---|---|
| 1 | No `PowerDistribution` object anywhere — total battery-side current/energy is never measured | High | Low | None (read-only) |
| 3 | Live battery **+ wiring** internal-resistance estimate — **now the primary diagnostic** | **Highest** | Low | None (read-only) |
| 2 | Kicker CIM could get software current sensing from the PD channel — the only unlimited motor, and endgame is peak kicker use | **High** | Low–Med | Low |
| 5 | Brownout events are never recorded in code | Medium | Trivial | None |
| 4 | Voltage-adaptive load-shed supervisor — **now justified** by the team's answer, but still the expensive option and it treats a symptom | High ceiling | **High** | **Medium** |
| 6 | `PowerTelemetry` publishes 17 currents but never their sum | Low | Trivial | None |
| 7 | PathPlanner `defaultNominalVoltage` = 12.0 — worth checking, not yet a finding | Low | Low | Low |

Items 1, 3, 5 and 6 are pure instrumentation — they change no control path and cannot make the
robot behave differently. They are also what makes items 2 and 4 possible.

**Build order:** 1 → 3 and 5 together → look at one match of data → then decide between 2 and 4.
The supervisor (4) buys margin against a sag whose cause is still unidentified; if the cause turns
out to be a hot SB50 or a jamming kicker, it would have masked a hardware fault rather than fixed
it. Measure first — items 1, 3 and 5 are a few lines each on a Notifier that already runs.

---

## What V2 already does (the floor — do not re-recommend)

Every mitigation in the V1 review's inventory is present in V2 unchanged, plus these, added
2026-07-21/24 and already attacking the same problem:

| Mitigation | Location |
|---|---|
| `MotorConfigs` factory — a motor **cannot** be configured without a current limit + neutral mode; failures reported loudly to the DS | `util/MotorConfigs.java`, all four subsystems |
| RT withholds flywheel spin-up until the aim is made (`aimAchieved`) — de-coincides the drive/steer burst from the 4-Kraken spin-up | `ShooterSubsystem.java:151-166`, `:623-626` |
| Aim rotation rate capped lower than the other heading servos (π vs 1.5π rad/s) | `FieldConstants.AIM_MAX_ROTATION_RATE_RAD_PER_SEC`, used at `CommandSwerveDrivetrain.java:839` |
| Hood stall cutoff — measured current + stopped encoder, debounced, latches the jammed direction | `ShooterSubsystem.java:816-833` |
| Hood feedback sanity gate — 0 V rather than closing the loop on a garbage encoder reading | `ShooterSubsystem.java:784-787` |
| Teleop-only drive mode gate (sticks dead in TEST/AUTO) | `RobotContainer.java:203-209` |
| Hopper belt 0.25 s open-loop ramp | `HopperSubsystem.java:95-98` — **the V1 review's one open nit is already fixed** |

The V1 review's **21 rejected candidates** apply to V2 verbatim — V2 is byte-identical to V1 on
every line they touch. Do not re-chase them; several would undo an on-robot fix.

---

## 1. Nothing in the code measures total battery current

**Verified:** `grep -rn "PowerDistribution\|PDH\|PDP\|getTotalCurrent\|getTotalEnergy" src/`
returns **zero matches**. `PowerTelemetry` and `Telemetry` both lack one.

`PowerTelemetry.java` samples 13 TalonFX supply currents and 4 SparkMax output currents at 10 Hz.
That sum is not the robot's draw. It misses:

- the **kicker CIM** (VictorSPX — no sensing at the controller at all)
- the roboRIO, radio, Limelight, and every other non-motor load
- the difference between a SparkMax's *output* current and its *supply* current (`PowerTelemetry.java:174`
  already publishes `NaN` for SparkMax supply, honestly)

The PD module measures all of it at the battery side. `edu.wpi.first.wpilibj.PowerDistribution`
(javap-verified against `wpilibj-2026.2.1`) offers `getTotalCurrent()`, `getAllCurrents()`,
`getCurrent(int)`, `getVoltage()`, `getTotalPower()`, `getTotalEnergy()`, `resetTotalEnergy()`,
and `getStickyFaults()`. The same class covers both a REV PDH and a CTRE PDP via its
`ModuleType` argument.

**Cost:** one object, a handful of extra NT topics on the Notifier that already runs at 10 Hz.
It writes nothing and controls nothing.

**On `getTotalEnergy()` — do not read it as a fuel gauge.** It accumulates joules from power-on
and knows nothing about the pack's capacity or state of charge. To mean anything per-match it
needs `resetTotalEnergy()` at the start of each match, and even then it is only useful as a
*relative* trend — "this match drew 15 % more than our usual" — not as "38 % battery remaining."

---

## 2. The kicker could have software current sensing after all

**This reopens a decision the team made on 2026-07-24, with new information — it is not an
oversight, and it is the team's call whether to revisit it.**

`deep-review-2026-07-24.md` §7.1 records: *"Team decision 2026-07-24: leave the duty as-is,
document the risk,"* premised on *"the VictorSPX has no current sensing, so software cannot limit
it — the breaker is the only protection."*

That premise is true of the **controller** and false of the **robot**. The PD module measures the
current on the kicker's channel regardless of what the VictorSPX can report. `pdh.getCurrent(ch)`
+ a `Debouncer` is exactly the stall-cutoff pattern already proven twice in this codebase (intake
slider, hood). The kicker is the only motor on the robot with zero software protection, and it is
also the largest single stall on it (CIM ≈ 131 A at 12 V), reachable today by holding **B** into a
jammed ball at 75 % duty.

**Caveats that must be settled before building it:**

- **Needs the PD channel number for CAN 17.** Not recorded in the Hardware-Data-Sheet — see the
  open questions below. Blocking.
- **PD per-channel current is slower and coarser than a TalonFX stator signal.** The slider's
  0.05 s current debounce is not transferable; this needs its own threshold and a longer debounce,
  tuned against a real jam on the robot.
- **Check the channel's measurement range.** PD channels have a maximum reportable current; if it
  saturates below a CIM stall the reading pins rather than climbing, which changes how the
  threshold must be written.
- It protects against a *sustained* jam, not an instantaneous one. The breaker remains the real
  backstop, and its size is still an open TODO (deep-review §5 item 2).

---

## 3. Live internal-resistance estimate — now the primary diagnostic

**Promoted 2026-07-25.** With the team reporting brownouts on a *freshly charged* pack late in the
match, this is no longer a background health check — it is the measurement that separates the two
surviving hypotheses (see the gating-question section above), and it is the only one of the two
that a Battery Beak cannot make, because the Beak tests a disconnected pack and the suspect
resistance may live in the breaker, the SB50 or the lugs.

Prior evidence for the battery-health branch: the 2026-07-12 capture had the pack resting at
11.5 V and sagging ~4 V under load.

With #1 in place it becomes a computation. Regress bus voltage against total current over a rolling
window; the slope is the resistance of the whole power path. Rising resistance across a match, or
match to match, is exactly the "it browns out late" signature, visible *before* the match where it
bites.

**What this number actually includes — state it on the dashboard, do not mislabel it.** Measured at
the roboRIO, the slope covers the **pack plus the main breaker, the SB50 connector, and all the
battery wiring**. That is arguably more useful than a pack-only figure — it catches a loose lug or a
heat-damaged SB50, which a Beak on a disconnected pack cannot — but it means the familiar
"retire above ~0.020 Ω" threshold does **not** transfer. It is a pack-only number. Build this to
compare *against itself* over time (per pack, per match) and calibrate its absolute scale against a
Beak reading once, rather than importing a threshold that measures something else.

---

## 4. Voltage-adaptive load-shed supervisor — justified now, but still the expensive option

**Status change 2026-07-25:** the V1 review made this contingent on the brownouts reproducing on a
healthy pack, and the team now reports exactly that. So it is justified. But it is still high
effort, medium risk, and — read against the arithmetic in the gating-question section — it
**treats the symptom**. If the sag turns out to be a hot SB50 or a jamming kicker CIM, a supervisor
would have quietly compensated for a hardware fault instead of surfacing it. Build items 1, 3 and 5
first, look at one match of real data, and only then decide whether this is still the right answer.

**Do not re-derive this.** The mechanism, shed order, thresholds and caveats are already written up
in `Helios-2026\docs\power-limiting-review.md` § "Opportunity worth building," and V2 is
byte-identical to V1 on every line it touches. Only the V2 deltas are recorded here:

**(a) Scale commands, not current limits.** The request says "variable current limits," but on this
hardware the cheap, safe implementation is variable *setpoints*. Both
`TalonFX.getConfigurator().apply()` and `SparkMax.configure()` are blocking CAN round-trips; running
either at loop rate risks a loop overrun. The static limits already enforce the ceiling — the
supervisor's job is to stay under it. `MaxSpeed` and `MaxAngularRate` (`RobotContainer.java:37-39`)
are mutable fields read live inside the drive lambdas at `RobotContainer.java:189-193`, so scaling
them throttles drive with zero rebinding and zero CAN traffic. Same logic for the flywheels: lower
the commanded surface speed, don't re-apply a supply limit.
*(For reference, Phoenix 6 does expose an `apply(configs, timeoutSeconds)` overload — javap-verified
— and a 0 s timeout is the documented non-blocking form. If a genuine runtime limit change is ever
needed, that is the shape to verify. It is not needed for the above.)*

**(b) Part of what it was aimed at is already handled.** V2's `aimAchieved` latch and the lower aim
rotation cap specifically de-coincide the RT drive-burst + flywheel-spin-up stack that was one of
the supervisor's motivating cases. The remaining coincidence cases are hard-accel driving stacked
with an intake or feed, which is where the supervisor still earns its keep.

**(c) Optional refinement, not a prerequisite.** With #1 and #3 landed, the trigger can be a
*predicted* rail voltage (`V_rest − I_total × R_path`) instead of measured sag — a leading indicator
rather than a lagging one. Treat that as an upgrade. The supervisor is viable on measured voltage
alone, exactly as the V1 review specified; don't make it contingent on the other two.

**The V1 caveats all still hold** and are the reason this is ranked high-ceiling but high-effort:
a 10 Hz loop cannot catch a tens-of-ms transient (the RIO's FPGA owns that floor); it cannot save a
dead battery; and without hysteresis and debouncing it will cut drive mid-defense or bog the shot
it is firing.

---

## 5. Brownouts are never recorded in code

Nothing in V2 reads `RobotController.isBrownedOut()` (javap-verified; also
`getBrownoutVoltage()`/`setBrownoutVoltage()`). A PD module additionally carries sticky faults
including brownout, readable via `getStickyFaults()` / clearable via `clearStickyFaults()`.

Today the only record of a brownout is the DS log, which someone has to remember to open. Latching
a per-enable brownout count, the minimum bus voltage seen, and when it happened — published to the
existing `CompanionTelemetry` table — costs a few lines on the Notifier that already runs, and turns
"we think we brown out late in matches" into a number the team can point at. Given that this is the
premise behind the entire supervisor, measuring it first is the cheapest possible next step.

---

## 6. `PowerTelemetry` never publishes a sum

`PowerTelemetry.java:181-185` publishes 17 individual stator and supply values plus battery voltage.
There is no `totalSupplyAmps` topic — the one number a driver can actually watch during a practice
match. It is a sum over data already refreshed in `update()`. Note it is the *motor-side* sum with
the gaps listed in #1, so it should be labelled as such, and superseded by the PD total if #1 lands.

---

## 7. PathPlanner nominal voltage — open question, not a finding

`src/main/deploy/pathplanner/settings.json` has `"defaultNominalVoltage": 12.0`. PathPlanner 2026
exposes `nominalVoltageVolts()` on its config (javap-verified in `pathplanner-2026.1.2`), but **I did
not verify what the planner actually does with it** — whether it bounds the planned velocity profile
or only scales feedforward.

The reason it is worth checking: if it bounds the profile, planning at 12 V when the bus really sits
near 11 V under load means the planner commands speeds the drivetrain cannot reach, and the velocity
loop saturates burning current for no extra speed. That is the same bug class as the
`kSpeedAt12Volts` 4.93 → 4.76 correction already made in `TunerConstants.java:127`. Low priority
regardless — only `driveForwardAuto` is actually bound today, and it is not a PathPlanner path.

---

## Where not to spend effort

- **Do not gate anything on `MatchStatus` match time.** See the top of this document.
- **Do not touch `RobotController.setBrownoutVoltage()`.** Lowering it delays the RIO's own
  protection and pushes toward a stage-3 blackout (a full reboot, tens of seconds dead). Raising it
  just browns out sooner. The default exists for good reasons.
- **Do not use a PD switchable channel to shed load.** It exists (`setSwitchableChannel(boolean)`),
  but everything on this robot worth switching off — the Limelight above all — is load-bearing for
  vision and auto-aim.
- **Do not re-chase the V1 review's 21 rejected candidates.** Several would re-break something
  already fixed on the robot (the steer supply starve, the flywheel stator anti-bog raise).
- **Do not change any static current limit.** Every one of them was sized against on-robot behaviour;
  none is the gap.

---

## Open questions (need the team / the robot)

1. **Does the robot have a REV PDH or a CTRE PDP, and what is its CAN ID?** Not recorded anywhere in
   `Helios-2026-Robot-Data`. Both work through the same WPILib class, so findings 1, 3, 5 hold either
   way — but the code needs the right `ModuleType`.
2. **Which PD channel is the kicker (CAN 17) on?** Blocking for finding 2.
3. ~~Do the late-match brownouts still reproduce on a freshly-charged battery?~~ **ANSWERED
   2026-07-25: yes — near the end of the match, on a pack charged for a long period immediately
   before.** See the gating-question section; this promoted findings 3 and 2 and justified 4.
4. **Kicker breaker size** — still open from `deep-review-2026-07-24.md` §5 item 2, and it is the
   kicker's only protection until finding 2 is built. Now more urgent than it was: the kicker is the
   leading candidate for hypothesis 2.

New questions raised by the team's answer:

5. **Is the robot driven at all between coming off the charger and the match start?** Pit testing or
   queue-line driving means the pack is not actually fresh at the tape, and a surface charge off the
   charger reads high for the first minute regardless.
6. **How old are the packs, and is the same one used for back-to-back matches?** "Fresh" needs to
   mean rested-and-recovered, not just recently disconnected from a charger.
7. **After a brownout match, are the SB50 / lugs / main breaker hot or discoloured?** The free
   physical check for hypothesis 1 — do this before writing any code.
8. **Does the brownout correlate with feeding fuel (kicker running), or with driving/pushing?** If
   the driver can say which, that alone separates hypotheses 1 and 2 at zero cost.

---

*Developed by Team 9704 with Claude Code.*
