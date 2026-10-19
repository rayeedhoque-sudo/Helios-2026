"""Feed-shot data capture (2026-10-17). Reads the live robot NT for a few seconds and saves what
the Limelight sees at one setpoint, plus the robot's pose / heading and the launcher state.

  python capture.py <point-id> [seconds] [22=28 ...]  -> docs/feed-data/<point-id>.json (+ printed summary)

  22=28: a printed tag 22 stands in at tag 28's field spot (raw ids are kept; geometry uses 28's pose).

Read-only: subscribes to NT, writes nothing to the robot. Needs `ntcore` (robotpy) and the robot
network (10.97.4.2). Tag layout = WPILib 2026-rebuilt-welded (field.json beside this script).
"""
import json, math, os, statistics as st, struct, sys, time, urllib.request
import ntcore

HERE = os.path.dirname(os.path.abspath(__file__))
FIELD = {t["ID"]: t["pose"] for t in json.load(open(os.path.join(HERE, "field.json")))["tags"]}
LL = "/limelight-knight/"
TAB = "/Shuffleboard/Shooter Subsystem Tab/"
ALIAS = {}  # seen id -> field id it stands in for (practice tags), from the command line


def tag_xy_yaw(tid):
    p = FIELD[ALIAS.get(tid, tid)]
    q = p["rotation"]["quaternion"]
    return p["translation"]["x"], p["translation"]["y"], math.degrees(2 * math.atan2(q["Z"], q["W"]))


def main():
    pid = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    ALIAS.update({int(a): int(b) for a, b in (arg.split("=") for arg in sys.argv[3:])})
    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4("feedcap"); nt.setServer("10.97.4.2")
    raw = nt.getDoubleArrayTopic(LL + "rawfiducials").subscribe([])
    mt1 = nt.getDoubleArrayTopic(LL + "botpose_wpiblue").subscribe([])
    tcs = nt.getDoubleArrayTopic(LL + "targetpose_cameraspace").subscribe([])
    tid = nt.getDoubleTopic(LL + "tid").subscribe(-1)
    mount = nt.getDoubleArrayTopic(LL + "camerapose_robotspace").subscribe([])  # all 0 = mount pose unset
    pipe = nt.getDoubleTopic(LL + "getpipe").subscribe(-1)
    pose = nt.getRawTopic("/DriveState/Pose").subscribe("struct:Pose2d", b"")
    ctrl = nt.getIntegerTopic("/FMSInfo/FMSControlData").subscribe(-1)
    red = nt.getBooleanTopic("/FMSInfo/IsRedAlliance").subscribe(False)
    launch = {k: nt.getDoubleTopic(TAB + k).subscribe(float("nan")) for k in
              ["Current Velocity", "Desired Velocity", "Current Angle", "Desired Angle"]}
    time.sleep(1.0)
    if not nt.isConnected():
        sys.exit("not connected to 10.97.4.2")

    tags, poses, mt1s, primary, launcher, enabled = {}, [], [], [], [], False
    n = int(secs * 50)
    for _ in range(n):
        r = raw.get()
        for i in range(0, len(r) - 6, 7):
            tags.setdefault(int(r[i]), []).append(r[i + 1:i + 7])  # txnc tync ta distCam distRobot ambiguity
        b = pose.get()
        if len(b) == 24 and not math.isnan(struct.unpack("<3d", b)[0]):  # the pose flickers NaN (MT2 NaN fused)
            poses.append(struct.unpack("<3d", b))
        m = mt1.get()
        if len(m) >= 6 and m[0] == m[0] and (m[0] or m[1]):
            mt1s.append(m[:6])
        c = tcs.get()
        if len(c) == 6 and any(c):
            primary.append((int(tid.get()), c))
        launcher.append({k: s.get() for k, s in launch.items()})
        enabled |= bool(ctrl.get() & 1)
        time.sleep(0.02)

    nan = float("nan")
    x, y = (st.fmean(v[i] for v in poses) for i in range(2)) if poses else (nan, nan)
    hdeg = math.degrees(math.atan2(sum(math.sin(v[2]) for v in poses), sum(math.cos(v[2]) for v in poses))) if poses else nan
    out = {"point": pid, "when": time.strftime("%Y-%m-%d %H:%M:%S"), "samples": n,
           "alliance": "red" if red.get() else "blue", "enabled_during": enabled,
           "ll_mount_pose": list(mount.get()), "ll_pipeline": pipe.get(),
           "tag_alias": {str(k): v for k, v in ALIAS.items()},
           "odom_pose": {"x": x, "y": y, "heading_deg": hdeg, "valid_samples": len(poses)},
           "launcher_mean": {k: st.fmean(l[k] for l in launcher) for k in launch},
           "tags": [], "shot": {"rpm": None, "hood_deg": None, "landing": None, "notes": None}}
    if mt1s:
        out["mt1_camera_pose"] = {"x": st.fmean(m[0] for m in mt1s), "y": st.fmean(m[1] for m in mt1s),
                                  "yaw_deg": math.degrees(math.atan2(sum(math.sin(math.radians(m[5])) for m in mt1s),
                                                                     sum(math.cos(math.radians(m[5])) for m in mt1s))),
                                  "frames": len(mt1s)}
    for t, rows in sorted(tags.items(), key=lambda kv: -len(kv[1])):
        col = lambda i: [r[i] for r in rows]
        txnc, tync, ta, dist, amb = (st.fmean(col(i)) for i in (0, 1, 2, 3, 5))
        tx_, ty_, tyaw = tag_xy_yaw(t)
        ground = dist * math.cos(math.radians(tync))
        # Robot (camera) field spot from this tag alone + the Pigeon heading, as FeedAim does (TX_SIGN +1).
        bearing = math.radians(hdeg - txnc)
        rx, ry = tx_ - ground * math.cos(bearing), ty_ - ground * math.sin(bearing)
        out["tags"].append({"id": t, "seen_frac": round(len(rows) / n, 2), "txnc": txnc, "tync": tync, "ta": ta,
                            "dist_cam_m": dist, "dist_sd_m": st.pstdev(col(3)), "ambiguity": amb,
                            "ground_m": ground, "tag_field": [tx_, ty_, tyaw],
                            "robot_from_tag": [rx, ry]})
    # Gyro-free pose: the heading at which every tag's (bearing, range) agrees on one robot spot.
    # ponytail: 0.1 deg grid search, fine for <= a handful of tags; needs 2+ tags (1 tag = Pigeon only).
    if len(out["tags"]) >= 2:
        best = None
        for i in range(3600):
            th = i / 10
            pts = [(tg["tag_field"][0] - tg["ground_m"] * math.cos(math.radians(th - tg["txnc"])),
                    tg["tag_field"][1] - tg["ground_m"] * math.sin(math.radians(th - tg["txnc"]))) for tg in out["tags"]]
            cx, cy = st.fmean(p[0] for p in pts), st.fmean(p[1] for p in pts)
            err = max(math.dist(p, (cx, cy)) for p in pts)
            # The tags must face the camera (a tag is only readable from its front half-plane).
            facing = all(math.cos(math.radians(tg["tag_field"][2])) * (cx - tg["tag_field"][0])
                         + math.sin(math.radians(tg["tag_field"][2])) * (cy - tg["tag_field"][1]) > 0 for tg in out["tags"])
            if facing and (best is None or err < best[0]):
                best = (err, th, cx, cy)
        if best:
            out["tag_pose"] = {"x": best[2], "y": best[3], "heading_deg": best[1], "spread_m": best[0],
                               "pigeon_minus_tag_deg": (hdeg - best[1] + 180) % 360 - 180}
    # Per-tag 3D results (LL REST): t6c_ts = camera in the tag's frame (x = viewer's right, z = into the
    # tag, so the camera sits at -z). One tag + the official layout gives a gyro-free camera spot; the
    # single-tag PnP can mirror x, so both are kept and the off-field / behind-the-tag one is flagged.
    try:
        for _ in range(5):  # a snapshot can come back with no fiducials even while the tag is in view
            res = json.load(urllib.request.urlopen("http://10.97.4.11:5807/results", timeout=2))
            if res.get("Fiducial"):
                break
            time.sleep(0.2)
        out["ll_fiducials_3d"] = [{k: f.get(k) for k in ("fID", "t6c_ts", "t6t_cs", "t6r_fs", "tx_nocross", "ty_nocross")}
                                  for f in res.get("Fiducial", [])]
    except Exception as e:  # the 2 s NT capture is still good without it
        out["ll_fiducials_3d"] = []
        print(f"  (LL REST failed: {e})")
    for f in out["ll_fiducials_3d"]:
        if f["fID"] not in FIELD or not f["t6c_ts"]:
            continue
        tx_, ty_, tyaw = tag_xy_yaw(f["fID"])
        n = (math.cos(math.radians(tyaw)), math.sin(math.radians(tyaw)))
        right = (-n[1], n[0])  # viewer's right, looking at the tag face
        cx_, _, cz_ = f["t6c_ts"][:3]
        f["cam_field"] = []
        for sx in (1, -1):
            px, py = tx_ - cz_ * n[0] + sx * cx_ * right[0], ty_ - cz_ * n[1] + sx * cx_ * right[1]
            hd = (math.degrees(math.atan2(ty_ - py, tx_ - px)) + f["tx_nocross"] + 180) % 360 - 180
            on = 0 <= px <= 16.541 and 0 <= py <= 8.069
            f["cam_field"].append({"x": px, "y": py, "heading_deg": hd, "on_field": on, "mirrored": sx < 0})
    path = os.path.join(HERE, f"{pid}.json")
    json.dump(out, open(path, "w"), indent=2)

    print(f"point {pid}  {out['alliance']}  enabled={enabled}  odom=({x:.2f}, {y:.2f}) hdg {hdeg:.1f}")
    if mt1s:
        m = out["mt1_camera_pose"]
        print(f"  MT1 camera pose ({m['x']:.2f}, {m['y']:.2f}) yaw {m['yaw_deg']:.1f}  [{m['frames']} frames]")
    for tg in out["tags"]:
        print(f"  tag {tg['id']:2d} seen {tg['seen_frac']:.0%} txnc {tg['txnc']:6.1f} tync {tg['tync']:6.1f} "
              f"dist {tg['dist_cam_m']:.2f}+-{tg['dist_sd_m']:.3f} amb {tg['ambiguity']:.2f} "
              f"-> robot ({tg['robot_from_tag'][0]:.2f}, {tg['robot_from_tag'][1]:.2f})")
    if "tag_pose" in out:
        tp = out["tag_pose"]
        print(f"  TAG POSE (no gyro) ({tp['x']:.2f}, {tp['y']:.2f}) hdg {tp['heading_deg']:.1f}  spread {tp['spread_m']:.2f} m"
              f"  | Pigeon off by {tp['pigeon_minus_tag_deg']:+.1f} deg")
    for f in out["ll_fiducials_3d"]:
        for c in f.get("cam_field", []):
            print(f"  3D tag {f['fID']:2d}{' mirror' if c['mirrored'] else '       '} -> ({c['x']:.2f}, {c['y']:.2f}) "
                  f"hdg {c['heading_deg']:.1f}{'' if c['on_field'] else '  OFF FIELD'}")
    if not out["tags"]:
        print("  NO TAGS SEEN")
    print(f"  saved {path}")


if __name__ == "__main__":
    main()
