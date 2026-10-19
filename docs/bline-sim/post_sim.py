import json, math, sys
# Point-mass sim of BLine v0.9.2 FollowPath translation on a path file (ideal velocity tracking).
def sim(path, caps_override=None, a=3.0, kp=2.0, kct=0.2, dt=0.02, vmin=0.15):
    d=json.load(open(path))
    wps=[e for e in d['path_elements'] if e['type'] in ('waypoint','translation')]
    pts=[(w['translation_target'] if w['type']=='waypoint' else w) for w in wps]
    P=[(p['x_meters'],p['y_meters']) for p in pts]
    R=[p.get('intermediate_handoff_radius_meters',0.2) for p in pts]
    caps=[3.0]*len(P)
    for c in d['constraints'].get('max_velocity_meters_per_sec',[]):
        for o in range(c['start_ordinal'],c['end_ordinal']+1):
            if o<len(caps): caps[o]=c['value']
    if caps_override:
        for k,v in caps_override.items(): caps[k]=v
    x,y=P[0]; vx=vy=0.0; i=1; t=0; trace=[]
    while t<20:
        # handoff (t-ratio or radius)
        while i<len(P)-1:
            ax,ay=P[i-1]; bx,by=P[i]; L=math.hypot(bx-ax,by-ay)
            prog=((x-ax)*(bx-ax)+(y-ay)*(by-ay))/L/L
            if prog>=max(0,1-R[i]/L) or math.hypot(bx-x,by-y)<=R[i]: i+=1
            else: break
        tx,ty=P[i]
        rem=math.hypot(tx-x,ty-y)+sum(math.hypot(P[k+1][0]-P[k][0],P[k+1][1]-P[k][1]) for k in range(i,len(P)-1))
        sp=min(kp*rem,caps[i])
        if rem>0.03: sp=max(sp,vmin)
        ang=math.atan2(ty-y,tx-x)
        dvx,dvy=sp*math.cos(ang),sp*math.sin(ang)
        ax,ay=P[i-1]; bx,by=P[i]; L=math.hypot(bx-ax,by-ay)
        ux,uy=(bx-ax)/L,(by-ay)/L; nx,ny=-uy,ux
        cte=(x-ax)*nx+(y-ay)*ny
        dvx-=kct*cte*nx; dvy-=kct*cte*ny
        s=math.hypot(dvx,dvy)
        if s>caps[i]: dvx,dvy=dvx*caps[i]/s,dvy*caps[i]/s
        ddx,ddy=dvx-vx,dvy-vy; dd=math.hypot(ddx,ddy)
        if dd>a*dt: ddx,ddy=ddx*a*dt/dd,ddy*a*dt/dd
        vx+=ddx; vy+=ddy; x+=vx*dt; y+=vy*dt; t+=dt
        trace.append((t,x,y,math.hypot(vx,vy),i))
        if i==len(P)-1 and rem<0.03 and math.hypot(vx,vy)<0.05: break
    return trace
def report(tr, label):
    ymax=max(tr,key=lambda r:r[2])
    print(f"{label}: end t={tr[-1][0]:.2f}s  max y={ymax[2]:.2f} at x={ymax[1]:.2f}")
    for xb in (5.6,5.19,4.6,3.99):
        r=min(tr,key=lambda r:abs(r[1]-xb) if r[4]==len([0])*0+r[4] else 9)
        cand=[q for q in tr if q[4]==max(z[4] for z in tr)]
        r=min(cand,key=lambda q:abs(q[1]-xb))
        print(f"   x={xb}: y={r[2]:.2f}  v={r[3]:.2f}")
if __name__=='__main__':
    p=sys.argv[1]
    report(sim(p),"as authored")
    for cap in (2.0,1.5,1.0):
        report(sim(p,{3:cap,4:cap}),f"corner legs (to wp3,wp4) capped {cap}")
