# Refit of the REPO's fixed-angle shot model at MAX_ANGLE = 38 deg (2026-07-21).
#
# The repo's ShooterSubsystem.modelSurfaceSpeed() is: no-drag closed form at the FIXED
# MAX_ANGLE, times a linear drag multiplier mult(d) = BASE + PER_METER*d, divided by
# SHOT_EFFICIENCY. Its fit constants were derived at the old 44.5 deg MAX_ANGLE
# (shot-math spec 2026-07-16) and went stale when MAX_ANGLE dropped to 38 (belt-skip
# inset). This script re-derives them at 38 deg from the same quadratic-drag integrator
# as ballistics.py (which generates the DIFFERENT, un-ported varying-angle tables --
# see README note). Drag multipliers are ball-speed ratios, so SHOT_EFFICIENCY never
# enters the fits.
#
# Outputs to paste into ShooterSubsystemConstants:
#   SCORE_DRAG_MULT_BASE / _PER_METER, SCORE_TOF_BASE_SEC / _SEC_PER_METER
#   FEED_DRAG_MULT_BASE / _PER_METER,  FEED_TOF_BASE_SEC / _SEC_PER_METER
#   MIN_SCORE_DISTANCE_METERS (from the speed-window analysis below)
# Plus pinned truth values for ShotModelTest.java.
import math

# Physics -- identical to ballistics.py (GE-26300 / manual 5.10.1).
G = 9.81
RHO = 1.225
CD = 0.5
R_BALL = 0.0751          # m
M_BALL = 0.215           # kg
K = RHO * CD * math.pi * R_BALL**2 / (2 * M_BALL)

RELEASE_H = 0.686        # m (27 in, TODO re-measure to actual exit point)
MOUTH_H = 1.829          # m (72 in front lip)
DH_MOUTH = MOUTH_H - RELEASE_H          # 1.143 m
THETA = 38.0             # deg -- the repo's MAX_ANGLE
APOTHEM_USABLE = 0.455   # m usable along-track half-width of the mouth (apothem - ball radius)

def fly(v0, theta_deg, x_stop, dt=0.0005):
    """Integrate to x_stop. Returns (y, t, gamma_deg) there."""
    th = math.radians(theta_deg)
    vx, vy = v0 * math.cos(th), v0 * math.sin(th)
    x = y = t = 0.0
    while x < x_stop and y > -1.5:
        v = math.hypot(vx, vy)
        ax, ay = -K * v * vx, -G - K * v * vy
        x += vx * dt; y += vy * dt
        vx += ax * dt; vy += ay * dt
        t += dt
    return y, t, math.degrees(math.atan2(vy, vx))

def descend_cross_x(v0, theta_deg, y_target, dt=0.0005):
    """x where the trajectory crosses y_target DESCENDING (nan if it never does)."""
    th = math.radians(theta_deg)
    vx, vy = v0 * math.cos(th), v0 * math.sin(th)
    x = y = 0.0
    while y > -1.5:
        v = math.hypot(vx, vy)
        ax, ay = -K * v * vx, -G - K * v * vy
        x_prev, y_prev = x, y
        x += vx * dt; y += vy * dt
        vx += ax * dt; vy += ay * dt
        if vy < 0 and y_prev >= y_target > y:
            f = (y_prev - y_target) / (y_prev - y)   # linear sub-step interpolation
            return x_prev + f * (x - x_prev)
    return float('nan')

def range_on_floor(v0, theta_deg, dt=0.0005):
    """(x, t) where the ball returns to carpet (y = -RELEASE_H), descending."""
    th = math.radians(theta_deg)
    vx, vy = v0 * math.cos(th), v0 * math.sin(th)
    x = y = t = 0.0
    while y > -RELEASE_H or vy > 0:
        v = math.hypot(vx, vy)
        ax, ay = -K * v * vx, -G - K * v * vy
        x += vx * dt; y += vy * dt
        vx += ax * dt; vy += ay * dt
        t += dt
        if t > 10:
            return float('nan'), t
    return x, t

def solve_v(f_err, lo=3.0, hi=45.0, tol=1e-4):
    """Bisection for monotone-increasing f_err(v)."""
    if f_err(lo) > 0 or f_err(hi) < 0:
        return float('nan')
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if f_err(mid) > 0:
            hi = mid
        else:
            lo = mid
        if hi - lo < tol:
            break
    return 0.5 * (lo + hi)

def lin_fit(xs, ys):
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    b = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    a = (sy - b * sx) / n
    return a, b

def v_nodrag(d, theta_deg, dh):
    """The Java closed form: sqrt(g d^2 / (2 cos^2 th (d tan th - dh)))."""
    th = math.radians(theta_deg)
    reach = d * math.tan(th)
    if reach <= 2 * dh:   # same refusal as modelSurfaceSpeed
        return float('nan')
    return math.sqrt(G * d * d / (2 * math.cos(th)**2 * (reach - dh)))

# ---- MIN score distance: speed window through the mouth at 38 deg ----
print(f"=== SCORE window analysis at {THETA} deg (usable half-width +-{APOTHEM_USABLE} m) ===")
print("descending-entry floor d_min = 2*dH/tan(th) =",
      f"{2 * DH_MOUTH / math.tan(math.radians(THETA)):.3f} m")
for d in [3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.7, 4.0]:
    v = solve_v(lambda vv: fly(vv, THETA, d)[0] - DH_MOUTH)
    if math.isnan(v):
        print(f"d={d:4.1f}  NO descending solution")
        continue
    _, _, gam = fly(v, THETA, d)
    if gam >= 0:
        print(f"d={d:4.1f}  crossing not descending (gamma {gam:+.1f} deg) -> unusable")
        continue
    # widest v range whose DESCENDING mouth crossing stays inside [d-.455, d+.455]
    v_lo = solve_v(lambda vv: descend_cross_x(vv, THETA, DH_MOUTH) - (d - APOTHEM_USABLE))
    v_hi = solve_v(lambda vv: descend_cross_x(vv, THETA, DH_MOUTH) - (d + APOTHEM_USABLE))
    win = (v_hi - v_lo) / (2 * v) * 100 if not (math.isnan(v_lo) or math.isnan(v_hi)) else float('nan')
    print(f"d={d:4.1f}  vball={v:5.2f}  entry gamma={gam:6.1f} deg  speed window +-{win:5.1f}%")

# ---- SCORE fits over the envelope (integrator truth vs Java closed form) ----
def score_fit(d_min, d_max):
    ds = [d_min + i * 0.2 for i in range(int(round((d_max - d_min) / 0.2)) + 1)]
    mults, tofs = [], []
    print(f"\n=== SCORE fit at {THETA} deg over [{d_min}, {d_max}] m ===")
    for d in ds:
        v_true = solve_v(lambda vv: fly(vv, THETA, d)[0] - DH_MOUTH)
        _, tof, gam = fly(v_true, THETA, d)
        vn = v_nodrag(d, THETA, DH_MOUTH)
        mults.append(v_true / vn)
        tofs.append(tof)
        print(f"d={d:4.1f}  v_true={v_true:5.2f}  v_nodrag={vn:5.2f}  mult={v_true/vn:.4f}  "
              f"ToF={tof:.3f}s  gamma={gam:6.1f}")
    a, b = lin_fit(ds, mults)
    resid = max(abs(a + b * d - m) / m for d, m in zip(ds, mults)) * 100
    print(f"SCORE_DRAG_MULT: {a:.4f} + {b:.5f}*d   (max residual {resid:.2f}%)")
    ta, tb = lin_fit(ds, tofs)
    tresid = max(abs(ta + tb * d - t) for d, t in zip(ds, tofs))
    print(f"SCORE_TOF: {ta:.3f} + {tb:.3f}*d s   (max residual {tresid:.3f} s)")
    return a, b, ta, tb

# ---- FEED fits (lob lands at d on carpet; code clamps d to [4, 9]) ----
def feed_fit():
    ds = [4.0 + i * 0.5 for i in range(11)]
    mults, tofs = [], []
    print(f"\n=== FEED fit at {THETA} deg over [4, 9] m ===")
    for d in ds:
        v_true = solve_v(lambda vv: range_on_floor(vv, THETA)[0] - d)
        _, tof = range_on_floor(v_true, THETA)
        vn = v_nodrag(d, THETA, -RELEASE_H)
        mults.append(v_true / vn)
        tofs.append(tof)
        print(f"d={d:4.1f}  v_true={v_true:5.2f}  v_nodrag={vn:5.2f}  mult={v_true/vn:.4f}  ToF={tof:.3f}s")
    a, b = lin_fit(ds, mults)
    resid = max(abs(a + b * d - m) / m for d, m in zip(ds, mults)) * 100
    print(f"FEED_DRAG_MULT: {a:.4f} + {b:.5f}*d   (max residual {resid:.2f}%)")
    ta, tb = lin_fit(ds, tofs)
    tresid = max(abs(ta + tb * d - t) for d, t in zip(ds, tofs))
    print(f"FEED_TOF: {ta:.3f} + {tb:.3f}*d s   (max residual {tresid:.3f} s)")

score_fit(3.5, 6.2)   # fit start = MIN_SCORE_DISTANCE_METERS chosen from the window analysis
feed_fit()

# ---- pinned truth for ShotModelTest (integrator ball speeds, m/s) ----
print("\n=== pinned truth (ball m/s from integrator; surface = ball / SHOT_EFFICIENCY) ===")
for d in [3.5, 4.5, 5.5, 6.2]:
    v = solve_v(lambda vv: fly(vv, THETA, d)[0] - DH_MOUTH)
    print(f"SCORE d={d}: vball={v:.3f}")
for d in [5.0, 7.0, 9.0]:
    v = solve_v(lambda vv: range_on_floor(vv, THETA)[0] - d)
    print(f"FEED  d={d}: vball={v:.3f}")
