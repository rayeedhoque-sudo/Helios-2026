# Seed-table generator for Helios 2026 shot model (quadratic-drag point-mass).
# Geometry from official sources (2026-07-17):
#   - GE-26300 field drawing sheet 4/4: mouth front lip 72.000+-0.5 in, hex outside
#     across-flats 41.932 in, net top 120.36 in, net width 58.41 in, body 47x47 in.
#   - Game manual 5.4: opening 41.7 in across (inside), front edge 72 in.
#   - Manual 5.10.1 FUEL: 5.91 in dia, 0.203-0.227 kg.
# Robot: release height 27 in (user-measured, TODO re-measure), SHOT_EFFICIENCY 0.50,
# flywheel surface = 2*pi*0.0508*1.5*motor_rps -> ceiling 95 rps = 45.5 m/s surface.
import math

G = 9.81
RHO = 1.225
CD = 0.5
R_BALL = 0.0751          # m (5.91 in dia)
M_BALL = 0.215           # kg (mid of 0.203-0.227)
K = RHO * CD * math.pi * R_BALL**2 / (2 * M_BALL)   # drag accel = K*v^2, per meter

RELEASE_H = 0.686        # m above carpet (27 in)
MOUTH_H = 1.829          # m above carpet (72 in front lip)
NET_TOP_H = 3.057        # m above carpet (120.36 in)
LIP_BACK = 0.53          # m from hub center to front lip (hex apothem, flat toward shooter)
NET_PAST = 0.56          # m from hub center to modeled (vertical) net face
EFFICIENCY = 0.50        # ball speed / flywheel surface speed

DH_MOUTH = MOUTH_H - RELEASE_H          # 1.143 m
DH_NET_TOP = NET_TOP_H - RELEASE_H      # 2.371 m


def fly(v0, theta_deg, x_stop, dt=0.0005):
    """Integrate until x >= x_stop or ball below release-1.5m. Returns (y, t, gamma_deg) at x_stop."""
    th = math.radians(theta_deg)
    vx, vy = v0 * math.cos(th), v0 * math.sin(th)
    x = y = t = 0.0
    while x < x_stop and y > -1.5:
        v = math.hypot(vx, vy)
        ax, ay = -K * v * vx, -G - K * v * vy
        x += vx * dt
        y += vy * dt
        vx += ax * dt
        vy += ay * dt
        t += dt
    return y, t, math.degrees(math.atan2(vy, vx))


def range_on_floor(v0, theta_deg, dt=0.0005):
    """Distance where the ball returns to carpet level (y = -RELEASE_H). Returns (x, t)."""
    th = math.radians(theta_deg)
    vx, vy = v0 * math.cos(th), v0 * math.sin(th)
    x = y = t = 0.0
    while y > -RELEASE_H or vy > 0:
        v = math.hypot(vx, vy)
        ax, ay = -K * v * vx, -G - K * v * vy
        x += vx * dt
        y += vy * dt
        vx += ax * dt
        vy += ay * dt
        t += dt
        if t > 10:
            return float('nan'), t
    return x, t


def solve_v(f_err, lo=3.0, hi=25.0, tol=1e-4):
    """Bisection on launch speed for monotone-increasing error function f_err(v)."""
    flo, fhi = f_err(lo), f_err(hi)
    if flo > 0 or fhi < 0:
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


print("=== SCORE (bank-primary: cross net plane aim_above_mouth; verify lip clearance) ===")
score_knots = [(3.0, 44.5), (3.5, 44.5), (4.0, 41.5), (4.5, 38.5), (5.0, 36.0), (5.5, 34.0), (6.2, 31.5)]
score_rows = []
for d, th in score_knots:
    x_lip, x_net = d - LIP_BACK, d + NET_PAST
    aim = DH_MOUTH + 0.45 + 0.09 * (d - 3.0)          # target height above release at net plane
    v = solve_v(lambda vv: fly(vv, th, x_net)[0] - aim)
    y_lip, t_lip, g_lip = fly(v, th, x_lip)
    y_mouth, t_mouth, g_mouth = fly(v, th, d)
    y_net, t_net, g_net = fly(v, th, x_net)
    surface = v / EFFICIENCY
    rps = surface / (2 * math.pi * 0.0508 * 1.5)
    lip_clear = y_lip - DH_MOUTH
    score_rows.append((d, th, v, surface, rps, t_mouth))
    print(f"d={d:4.1f} th={th:4.1f}  vball={v:6.2f}  surf={surface:6.2f} m/s  rps={rps:5.1f}  "
          f"lip+{lip_clear:5.2f} m(g{g_lip:6.1f})  mouth+{y_mouth - DH_MOUTH:5.2f}(g{g_mouth:6.1f})  "
          f"net+{y_net - DH_MOUTH:5.2f}  netTopMargin={DH_NET_TOP - y_net:4.2f}  ToF_mouth={t_mouth:4.2f}s")

a, b = lin_fit([r[0] for r in score_rows], [r[5] for r in score_rows])
print(f"SCORE ToF fit: {a:.3f} + {b:.3f}*d")

print("\n=== FEED (lob lands at distance d on carpet) ===")
feed_knots = [(4.0, 40.0), (5.0, 38.0), (6.0, 36.5), (7.0, 35.0), (8.0, 34.0), (9.0, 33.0)]
feed_rows = []
for d, th in feed_knots:
    v = solve_v(lambda vv: range_on_floor(vv, th)[0] - d)
    rng, tof = range_on_floor(v, th)
    surface = v / EFFICIENCY
    rps = surface / (2 * math.pi * 0.0508 * 1.5)
    feed_rows.append((d, th, v, surface, rps, tof))
    print(f"d={d:4.1f} th={th:4.1f}  vball={v:6.2f}  surf={surface:6.2f} m/s  rps={rps:5.1f}  "
          f"range={rng:5.2f}  ToF={tof:4.2f}s")

a, b = lin_fit([r[0] for r in feed_rows], [r[5] for r in feed_rows])
print(f"FEED ToF fit: {a:.3f} + {b:.3f}*d")

print("\n=== sensitivity check: +-5% speed at mid knots ===")
for d, th in [(4.5, 38.5)]:
    x_net = d + NET_PAST
    aim = DH_MOUTH + 0.45 + 0.09 * (d - 3.0)
    v = solve_v(lambda vv: fly(vv, th, x_net)[0] - aim)
    for s in (0.95, 1.0, 1.05):
        y_lip, _, _ = fly(v * s, th, d - LIP_BACK)
        y_net, _, _ = fly(v * s, th, x_net)
        ok = y_lip > DH_MOUTH + R_BALL and y_net < DH_NET_TOP - R_BALL
        print(f"SCORE d={d} v*{s:4.2f}: lip+{y_lip - DH_MOUTH:5.2f} net+{y_net - DH_MOUTH:5.2f} above-mouth  -> {'SCORES' if ok else 'MISS'}")
