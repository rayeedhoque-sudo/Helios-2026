# V2 LB feed-shot SEED generator (2026-10-12) -- produced the FEED_* tables in AutoAimConstants.
# 1) Fits a quadratic-drag point-mass model to the scored V2 RT hub shots:
#    launch angle = TH0 - SLOPE x hood, ball speed = EFF x flywheel surface speed (2 in radius),
#    release height H0 [assumed 0.6 m]. The fit is loose (rms ~0.4 m); slopes 0.2..1.0 fit almost
#    as well and move the feed range +-25%, so the middle (0.6) was used.
# 2) Prints the full-hood floor-landing rpm table and the over-the-hub lob table (clear the 3.06 m
#    net top + margin on the near side, stay above the mouth past the far lip, land 2.5 m past centre).
# Replace the tables with measured datapoints as soon as the team has them.
# Hub geometry: GE-26300 (see README.md). Drag constants as ballistics.py.
import math
G=9.81; RHO=1.225; CD=0.5; RB=0.0751; M=0.215
K=RHO*CD*math.pi*RB**2/(2*M)
MOUTH=1.829; NET_TOP=3.057; R_FW=0.0508
def traj(v,th,h0,dt=0.002):
    t=math.radians(th); vx,vy=v*math.cos(t),v*math.sin(t); x=0;y=h0;pts=[(x,y)]
    while y>0 or vy>0:
        s=math.hypot(vx,vy); vx+=-K*s*vx*dt; vy+=(-G-K*s*vy)*dt; x+=vx*dt; y+=vy*dt; pts.append((x,y))
        if len(pts)>20000: break
    return pts
def cross_desc(pts,h):
    for (x0,y0),(x1,y1) in zip(pts,pts[1:]):
        if y0>=h>y1: return x0+(x1-x0)*(y0-h)/(y0-y1)
    return float('nan')
def y_at(pts,xq):
    for (x0,y0),(x1,y1) in zip(pts,pts[1:]):
        if x0<=xq<x1: return y0+(y1-y0)*(xq-x0)/(x1-x0)
    return float('nan')
def vball(rpm,e): return e*rpm/60*2*math.pi*R_FW
# scored hub shots: (key m = camera->hub centre, flywheel rpm, hood deg)
DATA=[(1.778,1300,12.39),(2.306,1300,18.81),(2.356,1400,15.82),(2.868,1450,19.16),(3.083,1700,23.38),(3.222,1500,28.65),(3.989,1600,26.02)]

def floor_x(v, th, h0): return traj(v, th, h0)[-1][0]
def rpm_for(fn, lo=800, hi=4500):
    """Smallest flywheel rpm with fn(rpm) True (fn monotone in rpm)."""
    if not fn(hi): return float('nan')
    for _ in range(40):
        m = (lo + hi) / 2
        if fn(m): hi = m
        else: lo = m
    return hi

if __name__ == '__main__':
    H0, SLOPE, TH0, EFF = 0.6, 0.6, 82, 0.94   # chosen fit (see header)
    HOOD_MAX = 29
    print('fit check (hub shots): d, rpm, hood -> where the ball crosses mouth height descending')
    for d, rpm, hood in DATA:
        print(f'  {d:.2f} {rpm} {hood:5.1f} -> {cross_desc(traj(vball(rpm, EFF), TH0 - SLOPE * hood, H0), MOUTH):.2f} m')
    print('FEED_RPM_BY_LANDING_DIST (full hood):')
    for L in (3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12.5):
        print(f'  {L:5.1f} -> {rpm_for(lambda r: floor_x(vball(r, EFF), TH0 - SLOPE * HOOD_MAX, H0) >= L):.0f}')
    NET_X_BACK, NET_CLEAR = 0.85, 3.057 + 0.075 + 0.25   # net top 0.85 m before hub centre (leans back)
    FAR_X, FAR_CLEAR, LAND_PAST = 0.60, 1.829 + 0.075 + 0.25, 2.5
    print('FEED_OVER_HUB (hub dist -> hood, rpm):')
    for d, hood in ((2.0, 10), (2.5, 14), (3.0, 17), (4.0, 20), (5.0, 20), (6.0, 20), (7.0, 20), (8.0, 20)):
        th = TH0 - SLOPE * hood
        def ok(r):
            p = traj(vball(r, EFF), th, H0)
            return y_at(p, d - NET_X_BACK) >= NET_CLEAR and y_at(p, d + FAR_X) >= FAR_CLEAR and p[-1][0] >= d + LAND_PAST
        print(f'  {d:.1f} -> hood {hood}, {rpm_for(ok):.0f} rpm')
