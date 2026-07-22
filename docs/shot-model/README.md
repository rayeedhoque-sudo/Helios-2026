# Shot Model — distance → (hood angle, flywheel speed)  (2026-07-17)

Why the hood angle now varies with distance, where the numbers came from, and how to tune
them on the robot. Supersedes the 2026-07-16 fixed-max-angle rule.

## Verified hub geometry (official sources)

| Fact | Value | Source |
|---|---|---|
| Hub body | 47 × 47 in prism | GE-26300 drawing sheet 4/4 (2026-field-dimension-dwgs.pdf p.25) |
| Funnel mouth | LEVEL hexagon, outside across-flats 41.932 ±0.5 in (inside ≈ 41.7 in) | GE-26300 4/4 + manual §5.4 |
| Front lip height | 72.000 ±0.5 in above carpet | GE-26300 4/4 |
| Funnel panels | 6 × identical GE-26329 trapezoids, 73.9° base angle → mouth is level, not tilted | TE-26300 build instructions p.6, p.19 |
| **Net backboard** | back of hub, 58.41 in wide, **top at 120.36 in** (~4 ft above the mouth), leans back ~10 in | GE-26300 4/4 |
| FUEL ball | 5.91 in dia, 0.203–0.227 kg foam | manual §5.10.1 |

The net exists to block shots from prohibited areas (it faces the neutral zone), but from
our alliance zone it is a **backboard**: any shot that clears the front lip and meets the
net face below its top drops into the funnel.

## Why fixed 44.5° was wrong (the old policy)

Fixed-max-angle was optimal **only for a swish** through the level mouth: at 3 m the ball
crosses just past apex (entry only ~13° below horizontal) and the along-track acceptance is
±0.455 m → a **±3% speed window**. Our biggest unknown (ball/surface efficiency, assumed
0.50) is a ±10–20% class error — the swish policy cannot absorb it, which is why practice
shots were inconsistent.

## The bank-shot policy (new)

Flatten the hood as range grows and aim the trajectory to cross the **net plane 0.45–0.74 m
above mouth level** while clearing the front lip by 0.39–0.77 m. On-model shots still swish
(they cross the mouth just past apex, −1.7…−2.6°); off-model shots hit the net and drop in.
Measured windows from the integrator: **±10–18% speed error still scores** at 4.5–6.2 m.
Close range (3.0–3.5 m) stays at max angle 44.5° — the lip is too high relative to the
robot for a flatter launch there. Feeds flatten too (40°→33°): the min-energy ground lob is
~40–42° and the team already found 44.5° feeds "arc too high".

Bonus: score time-of-flight becomes nearly constant (~0.55–0.65 s across the whole
envelope), which shrinks moving-shot compensation error.

## Where the numbers live

- Seed tables: `ShooterSubsystemConstants.SCORE_HOOD_DEG_BY_DIST`, `SCORE_SURFACE_MPS_BY_DIST`,
  `FEED_HOOD_DEG_BY_DIST`, `FEED_SURFACE_MPS_BY_DIST` (InterpolatingDoubleTreeMap — linear
  between knots, clamped at the ends). Distances are **robot → hub center** (score) or
  **robot → feed aim point** (feed), in meters.
- Generator: [`ballistics.py`](ballistics.py) — quadratic-drag point-mass (Cd 0.5,
  m 0.215 kg, release 0.686 m, efficiency 0.50). Re-run it after changing geometry or the
  angle schedule; paste the printed table into the constants.

## Tuning order on the robot

1. **`SHOT_SPEED_SCALE`** first — one knob rescales every speed. Shoot at 4.5 m; if short,
   raise (RB blind-feed history suggests ~1.1–1.2 may be needed, i.e. real efficiency < 0.50).
2. Re-measure **release height** to the actual ball exit point (27 in was to the unextended
   hood) and re-run the generator if it moved.
3. Per-knot touch-ups in the tables (edit the `.put()` pairs; `ShotTableTest` enforces
   monotonicity, band limits, and the motor ceiling at build time).
4. Drag/ToF last — refit only if moving shots systematically miss radially.

## Known limitations / on-robot TODOs

- Hood angle is ASSUMED equal to launch elevation 1:1 (same assumption as before; the
  `0.0317428` deg-conversion fudge in the hood ratio is still unexplained). Table tuning
  absorbs a constant offset; verify at two distances early.
- Limelight camera **mount pose in the web UI is still unset/unverified** — until it is,
  botpose/MegaTag2 are systematically wrong, their in-field gates reject them, and the
  camera-space fallback (SCORE only) carries the distance. Set the mount pose first session.
- RT kicker follows `hasShotTarget` (adversarial review 2026-07-17): it closes on any
  refusal (fixes the refused-shot CIM-stall grind the ungated binding allowed) but still
  does NOT wait for at-speed/at-angle — first balls can fly while wheels spin up and the
  hood travels. RB stays fully ungated (team-tuned). Re-gate both on `isReadyToShoot()`
  once tolerances are trusted on the robot.
- Min score range could shrink from 3.0 m to ~2.6 m under the bank policy (lip clearance
  permitting) — left at 3.0 m until tested on carpet.
