"""Shot log capture: one row per scoring shot, most of it read off the live robot.

Usage (robot on, tethered or on the robot's wifi):
    python tools/shotlog.py peek                  # show live values, write nothing
    python tools/shotlog.py add 10ft --scored     # log a shot that went in
    python tools/shotlog.py add 3.2m --miss -n "short, hit the rim"

You supply the ONE thing the robot cannot know -- the measured distance -- and
whether it scored.

MEASUREMENT CONVENTION (team, 2026-08-29), do not change it mid-table: tape
measure, horizontal, from the FRONT OF THE DRIVETRAIN (shooter side) to the hub,
taken LOW on the hub. Low on the hub is the outer wall, which is also the plane
the AprilTag is mounted on -- the hub footprint is 47 in square, so its wall sits
~23.5 in from centre and the tag plane sits 23.8 in (0.6035 m) from centre. Those
agree, which is why this tape reading and the camera's tag distance differ only
by fixed camera geometry (setback behind the bumper, plus mounting height) rather
than by anything range-dependent and messy. Everything
else (RT flywheel RPM, hood position, what the Limelight sees) is read from
NetworkTables at the moment you run it, so the numbers in the log are the numbers
that were actually on the robot for that shot. Take the shot, then run this before
touching the DPAD.

Rows append to docs/shot-log.csv, which opens directly in Excel/Sheets.

ponytail: no live-follow mode, no GUI. One shot, one command, one row.
Self-check: python tools/shotlog.py selftest
"""

import argparse
import csv
import datetime
import math
import os
import statistics
import sys
import time

ROBOT = "10.97.4.2"          # roboRIO (USB tether is 172.22.11.2 -- pass --server to override)
LIMELIGHT = "limelight-knight"
SHOOTER_TAB = "/Shuffleboard/Shooter Subsystem Tab/"
# Tag face -> hub center along the tag's inward normal. Same constant the robot code
# uses (FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS); only meaningful when the tag is
# actually mounted on the hub.
TAG_FACE_TO_HUB_DEPTH_M = 0.6035

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "shot-log.csv")

COLUMNS = [
    "when",                  # local timestamp
    "dist_m",                # MEASURED: tape, horizontal, front of drivetrain -> hub wall (low)
    "dist_in",               # same, inches (convenience)
    "scored",                # yes / no
    "rt_rpm",                # RT flywheel TARGET, motor RPM (what the DPAD trims)
    "flywheel_rpm",          # measured flywheel speed at capture (0 if already spun down)
    "hood_units_above_base", # <-- THE tuning column: goes straight into AUTOAIM_HOOD_TABLE
    "hood_base",             # enable-time datum, raw units
    "hood_current",          # base + travel, raw units
    "hood_raw_enc",          # absolute encoder reading, rotations
    "ll_tv",                 # 1 = Limelight has a target
    "ll_tag_id",
    "ll_tx_deg",             # target bearing, +right
    "ll_ty_deg",
    "ll_ta",                 # target area (% of image)
    "ll_cam_x_m",            # camera space: +x right
    "ll_cam_z_m",            # camera space: +z forward (depth)
    "ll_tag_dist_m",         # hypot(x, z) -- camera to the TAG FACE, mean over the sample window
    "ll_tag_dist_sd_m",      # spread of that distance while standing still = the camera's own noise
    "ll_samples",            # how many frames went into the mean
    # + face-to-hub depth. THIS IS THE COLUMN TO INDEX THE TABLE ON: it is exactly what
    # ShooterSubsystem.autoAimDistanceMeters computes, so a table keyed on it needs no
    # conversion and carries no bias -- the same number is used to tune and to look up.
    # NOTE it is only valid for THIS tag setup. The 0.6035 m term assumes the tag sits on
    # a real hub face; move the tag to a different mounting and every key here shifts.
    "ll_hub_dist_m",
    "notes",
]


def parse_distance_m(text):
    """'10ft' / '120in' / '3.5m' / '3.5' (bare = meters) -> meters."""
    t = str(text).strip().lower().replace(" ", "")
    for suffix, per_meter in (("mm", 0.001), ("cm", 0.01), ("in", 0.0254), ('"', 0.0254),
                              ("ft", 0.3048), ("'", 0.3048), ("m", 1.0)):
        if t.endswith(suffix):
            return float(t[: -len(suffix)]) * per_meter
    return float(t)   # bare number = meters


def read_robot(server, settle_sec=1.0, sample_sec=2.0):
    """One snapshot of everything the robot and the Limelight are publishing."""
    import ntcore

    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4("claude-shotlog")
    nt.setServer(server)

    ll = "/" + LIMELIGHT + "/"
    doubles = {
        "rt_rpm": SHOOTER_TAB + "RT Target (motor RPM)",
        "flywheel_rpm": SHOOTER_TAB + "Current Flywheel (motor RPM)",
        "hood_units_above_base": SHOOTER_TAB + "Hood Travel Since Datum (deg)",
        "hood_base": SHOOTER_TAB + "Hood Base Angle (deg)",
        "hood_current": SHOOTER_TAB + "Current Angle",
        "hood_raw_enc": SHOOTER_TAB + "Hood Encoder Raw (rot)",
        "ll_tv": ll + "tv",
        "ll_tag_id": ll + "tid",
        "ll_tx_deg": ll + "tx",
        "ll_ty_deg": ll + "ty",
        "ll_ta": ll + "ta",
    }
    # Full paths, subscribed explicitly -- NT topic discovery does not work from here.
    subs = {k: nt.getDoubleTopic(p).subscribe(float("nan")) for k, p in doubles.items()}
    pose_sub = nt.getDoubleArrayTopic(ll + "targetpose_cameraspace").subscribe([])

    deadline = time.time() + 5.0
    while time.time() < deadline and not nt.isConnected():
        time.sleep(0.1)
    if not nt.isConnected():
        sys.exit(f"No NetworkTables connection to {server}. Is the robot on and this PC on its network?")
    time.sleep(settle_sec)   # let the retained values land

    # SAMPLE the camera distance rather than snapshotting it. A single frame cannot tell
    # you whether the number is trustworthy; the spread over a couple of seconds with the
    # robot standing still IS the camera's own noise, and that noise is what decides how
    # finely the table can be indexed. A row whose sd is larger than the gap between two
    # table rows means those rows cannot be told apart on the field.
    xs, zs, dists = [], [], []
    end = time.time() + sample_sec
    while time.time() < end:
        pose = list(pose_sub.get())
        if len(pose) >= 3 and (pose[0] or pose[2]):
            xs.append(pose[0])
            zs.append(pose[2])
            dists.append(math.hypot(pose[0], pose[2]))
        time.sleep(0.05)

    out = {k: s.get() for k, s in subs.items()}
    if dists:
        out["ll_cam_x_m"] = statistics.mean(xs)
        out["ll_cam_z_m"] = statistics.mean(zs)
        out["ll_tag_dist_m"] = statistics.mean(dists)
        out["ll_tag_dist_sd_m"] = statistics.pstdev(dists) if len(dists) > 1 else 0.0
        out["ll_samples"] = float(len(dists))
        out["ll_hub_dist_m"] = math.hypot(out["ll_cam_x_m"], out["ll_cam_z_m"] + TAG_FACE_TO_HUB_DEPTH_M)
    else:
        for k in ("ll_cam_x_m", "ll_cam_z_m", "ll_tag_dist_m", "ll_tag_dist_sd_m",
                  "ll_samples", "ll_hub_dist_m"):
            out[k] = float("nan")
    return out


def fmt(v):
    if isinstance(v, float):
        return "" if math.isnan(v) else f"{v:.4g}"
    return v


def show(row):
    for k in COLUMNS:
        if k in row:
            print(f"  {k:24s} {fmt(row[k])}")


def append_row(row):
    path = os.path.normpath(CSV_PATH)
    is_new = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
        if is_new:
            w.writeheader()
        w.writerow({k: fmt(row.get(k, "")) for k in COLUMNS})
    return path


def selftest():
    assert abs(parse_distance_m("3.5m") - 3.5) < 1e-9
    assert abs(parse_distance_m("3.5") - 3.5) < 1e-9, "bare number is meters"
    assert abs(parse_distance_m("10ft") - 3.048) < 1e-9
    assert abs(parse_distance_m("120in") - 3.048) < 1e-9
    assert abs(parse_distance_m('120"') - 3.048) < 1e-9
    assert abs(parse_distance_m("10'") - 3.048) < 1e-9
    assert abs(parse_distance_m(" 350 CM ") - 3.5) < 1e-9
    # inches column is just the conversion back
    assert abs(3.048 / 0.0254 - 120.0) < 1e-9
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["add", "peek", "selftest"])
    ap.add_argument("distance", nargs="?", help="measured front-of-drivetrain -> hub, e.g. 10ft / 120in / 3.05m")
    ap.add_argument("--scored", action="store_true", help="the shot went in")
    ap.add_argument("--miss", action="store_true", help="the shot did not go in")
    ap.add_argument("-n", "--notes", default="")
    ap.add_argument("--server", default=ROBOT)
    # Logging a shot AFTER the hood or speed has already been changed: pass the numbers
    # that were actually on the robot for the shot. They override the live read.
    ap.add_argument("--hood", type=float, help="hood units above base, overriding the live read")
    ap.add_argument("--rpm", type=float, help="RT flywheel RPM, overriding the live read")
    # No robot on the network (or the tag was not visible): log what we have, leave the
    # rest blank. A row with a distance and a hood number is still a table row.
    ap.add_argument("--no-robot", action="store_true", help="skip NetworkTables entirely")
    args = ap.parse_args()

    if args.action == "selftest":
        selftest()
        return

    row = {k: float("nan") for k in COLUMNS} if args.no_robot else read_robot(args.server)
    if args.hood is not None:
        row["hood_units_above_base"] = args.hood
    if args.rpm is not None:
        row["rt_rpm"] = args.rpm
    row["when"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    row["notes"] = args.notes

    if args.action == "peek":
        print("LIVE (nothing written):")
        show(row)
        return

    if not args.distance:
        sys.exit("add needs a distance, e.g.  python tools/shotlog.py add 10ft --scored")
    if args.scored == args.miss:
        sys.exit("say exactly one of --scored / --miss")

    row["dist_m"] = parse_distance_m(args.distance)
    row["dist_in"] = row["dist_m"] / 0.0254
    row["scored"] = "yes" if args.scored else "no"
    if row.get("ll_tv") != 1.0:
        print("WARNING: the Limelight had NO target at capture -- the ll_* columns are empty.")
    path = append_row(row)
    print(f"logged to {path}:")
    show(row)


if __name__ == "__main__":
    main()
