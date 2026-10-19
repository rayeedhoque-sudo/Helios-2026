package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.utility.LimelightHelpers.RawFiducial;

public class SubsystemConstants {

        /**
         * Master gate for live PID tuning from the driver-companion app.
         *
         * <p>MUST BE FALSE FOR COMPETITION. When false, frc.robot.util.Tunable publishes nothing,
         * subscribes to nothing, and every gain is the compiled constant below -- there is no path
         * for a dashboard value to reach a motor controller. Flip to true and redeploy once at the
         * start of a tuning session, then flip it back.
         */
        public static final boolean TUNING_MODE = true;

        // LOW-BATTERY DRIVE CUT (team request 2026-09-30): when the battery drops below
        // LOW_BATTERY_CUT_VOLTS the 4 drive motors' current limits (stator + supply) are scaled
        // by LOW_BATTERY_LIMIT_SCALE. They come back only once the battery is above
        // LOW_BATTERY_RESTORE_VOLTS for LOW_BATTERY_RESTORE_SEC -- without that gap, cutting the
        // current lets the voltage recover, which would restore the limits, which sags it again
        // (the limits would flap on and off every loop). [assumed] TODO tune restore on robot.
        public static class DriveLimitConstants{
            public static final double LOW_BATTERY_CUT_VOLTS = 8.0;   // team 2026-09-30 (was 9.0)
            public static final double LOW_BATTERY_CUT_SEC = 0.1;      // must stay below for this long
            public static final double LOW_BATTERY_RESTORE_VOLTS = 10.0;
            public static final double LOW_BATTERY_RESTORE_SEC = 1.0;
            public static final double LOW_BATTERY_LIMIT_SCALE = 0.5;  // "cut in half"
        }

        // APRILTAG STAND-INS (team 2026-10-17; replaces the old PRACTICE_TEST_TAG / TEST_TAG_* system).
        // TAG_n = the tag ID our code treats a SEEN tag n as. For matches every TAG_n = n.
        // Practice: a printed tag hung at ANOTHER tag's field spot, e.g. TAG_22 = 28 = "the 22 we hung
        // where tag 28 goes" -- RT tag aim, LB feed and the zone idle then use tag 28's ID and field pose
        // whenever the camera sees 22 (and a real 22 elsewhere would count as 28 too). Any tag can be
        // set. Ignored while an FMS is attached (never in a real match); the driver station warns at
        // boot while any is set. The Limelight's own MegaTag1/2 poses (vision fusion, the boot seed)
        // use ITS uploaded field map and never see these.
        public static class AprilTagConstants{
            public static final int TAG_1 = 1;
            public static final int TAG_2 = 2;
            public static final int TAG_3 = 3;
            public static final int TAG_4 = 4;
            public static final int TAG_5 = 5;
            public static final int TAG_6 = 6;
            public static final int TAG_7 = 7;
            public static final int TAG_8 = 8;
            public static final int TAG_9 = 9;
            public static final int TAG_10 = 10;
            public static final int TAG_11 = 11;
            public static final int TAG_12 = 12;
            public static final int TAG_13 = 13;
            public static final int TAG_14 = 14;
            public static final int TAG_15 = 15;
            public static final int TAG_16 = 16;
            public static final int TAG_17 = 17;
            public static final int TAG_18 = 18;
            public static final int TAG_19 = 19;
            public static final int TAG_20 = 20;
            public static final int TAG_21 = 21;
            public static final int TAG_22 = 22;
            public static final int TAG_23 = 23;
            public static final int TAG_24 = 24;
            public static final int TAG_25 = 25;
            public static final int TAG_26 = 26;
            public static final int TAG_27 = 27;
            public static final int TAG_28 = 28;
            public static final int TAG_29 = 29;
            public static final int TAG_30 = 30;
            public static final int TAG_31 = 31;
            public static final int TAG_32 = 32;

            private static final int[] STAND_INS = {0, TAG_1, TAG_2, TAG_3, TAG_4, TAG_5, TAG_6, TAG_7, TAG_8, TAG_9, TAG_10, TAG_11, TAG_12, TAG_13, TAG_14, TAG_15, TAG_16, TAG_17, TAG_18, TAG_19, TAG_20, TAG_21, TAG_22, TAG_23, TAG_24, TAG_25, TAG_26, TAG_27, TAG_28, TAG_29, TAG_30, TAG_31, TAG_32};

            /**
             * The tag ID a seen tag counts as. Active in every mode (teleop, auto, test, disabled) with
             * no FMS; itself while an FMS is attached.
             */
            public static int tagAs(int seenId){
                return tagAs(seenId, STAND_INS);
            }

            /** tagAs against any table, gated on the live FMS flag (AprilTagConstantsTest drives it). */
            static int tagAs(int seenId, int[] table){
                return tagAs(seenId, DriverStation.isFMSAttached(), table);
            }

            /** Pure version (AprilTagConstantsTest): table[seenId], or seenId with an FMS / out of range. */
            static int tagAs(int seenId, boolean fmsAttached, int[] table){
                return !fmsAttached && seenId > 0 && seenId < table.length ? table[seenId] : seenId;
            }

            /** One frame's tags with each ID swapped for the tag it stands in for. Null stays null. */
            public static RawFiducial[] remap(RawFiducial[] seen){
                if (seen == null) {
                    return null;
                }
                RawFiducial[] out = new RawFiducial[seen.length];
                for (int i = 0; i < seen.length; i++) {
                    RawFiducial f = seen[i];
                    out[i] = new RawFiducial(tagAs(f.id), f.txnc, f.tync, f.ta, f.distToCamera, f.distToRobot, f.ambiguity);
                }
                return out;
            }

            /** Every stand-in that is set, e.g. "22->28", or "" when none (the boot warning). */
            public static String standIns(){
                StringBuilder sb = new StringBuilder();
                for (int id = 1; id < STAND_INS.length; id++) {
                    if (STAND_INS[id] != id) {
                        sb.append(sb.length() == 0 ? "" : ", ").append(id).append("->").append(STAND_INS[id]);
                    }
                }
                return sb.toString();
            }
        }

        // RB SHOOT-ON-THE-MOVE (redesigned 2026-10-09, see ShotCalculator). Replaces the
        // tag-relative aim-only RB (its filter / tag-hold / view-angle / tag-alias tuning lives
        // in git history, last present in commit 76fca40).
        public static class AutoAimConstants{
            // Heading servo (SwerveRequest.FieldCentricFacingAngle). Radians in, rad/s out.
            // kP and the spin cap are the last on-robot values from the aim-only RB.
            public static final double HEADING_kP = 5.0;   // TODO tune on robot (while moving now)
            public static final double HEADING_kI = 0.0;
            public static final double HEADING_kD = 0.0;
            public static final double MAX_ROTATION_RAD_PER_SEC = 1.5 * Math.PI; // spin cap
            // RB MODE (team 2026-10-09): false = TAG AIM (TagAim: robot holds still and turns
            // the crosshair onto tag 10/26, aim only). true = SHOOT ON THE MOVE (ShotCalculator).
            public static final boolean RB_SHOOT_ON_THE_MOVE = false;
            // TAG SHOOT-ON-THE-MOVE (2026-10-10, after Ninjas 4744 / 1833; ON -- team 2026-10-10,
            // first robot test pending). Only matters while RB_SHOOT_ON_THE_MOVE = false. true =
            // during the RT tag aim the LEFT stick still drives (capped at 10%; the right stick
            // is ignored -- rotation is automatic) and the robot faces the hub
            // LEAD-CORRECTED for its velocity: each tag frame pins the hub centre in odometry, then
            // every loop aims at hub - velocity x TIME_OF_FLIGHT_SEC_BY_DISTANCE (still
            // [placeholder] 1.0 s) and keys the tag shot tables on the distance to that point.
            // Standing still it settles exactly like the stand-still aim (crosshair + trim).
            // false = the stand-still tag aim, unchanged. TODO verify on robot in open space,
            // then tune TIME_OF_FLIGHT_SEC_BY_DISTANCE: moving shots landing AHEAD in the
            // direction of travel (or long when driving at the hub) = raise it; behind = lower.
            public static final boolean TAG_SHOOT_ON_THE_MOVE = true;
            // Tag aim turn direction. +1 = target heading = heading MINUS tx (the paper
            // convention). ON-ROBOT 2026-10-09 (team): -1 turned the opposite way, so +1. (The
            // 2026-08-29 V1 aim needed -1 -- the Limelight and Pigeon have both moved since.)
            public static final double TAG_AIM_TX_SIGN = 1.0;
            // Tag aim trim (deg, + = settle further LEFT / counter-clockwise). Team 2026-10-10:
            // RB settled right shooting from the LEFT side of the hub (team: 3 -> 1). TODO verify head-on
            // and from the right side: if the right side now misses ~2 deg left, the error is
            // angle-dependent (hub depth / camera mount offset), not a constant -- fix that instead.
            // Stacks on the Limelight web-UI crosshair calibration: tune one, not both.
            public static final double TAG_AIM_LEFT_TRIM_DEG = 1.0;
            // RB TAG-AIM SHOT TABLES (team 2026-10-10, RT shots that SCORED on a real hub, taken
            // AFTER the hood kG/kP hold fix; the 2026-10-09 head-on rows were shot while the hood
            // drooped and read ~3-4 deg low, so they were all replaced). Rows were shot
            // from ~50 deg off the front face and square to a side face -- the key is distance
            // only, so left/right and both hubs share it. Key = TagAim.hubDistanceMeters(): the
            // tag's Limelight 3D range carried on to the hub centre. Keyed on the camera's own reading (not the tape),
            // so the Limelight's ~5 in long bias past 1.5 m is baked in. Rpm = FLYWHEEL rpm (RT
            // target; motor rpm = x1.444). Hood = the dashboard angle while shooting. Between rows
            // interpolated; past the ends it holds the end row (no data under 1.78 m / past 3.99 m).
            // Added to every table rpm (flywheel rpm) -- one knob to trim the whole RB shot up/down
            // while the table keeps the raw scored data. Team 2026-10-09: +100.
            public static final double TAG_SHOT_RPM_OFFSET = 0.0;
            // Same idea for the hood: degrees added to every table hood angle (+ = hood up).
            // The hood still clamps to 0..HOOD_MAX_DEG. Team 2026-10-09: 0 for now.
            public static final double TAG_SHOT_HOOD_OFFSET_DEG = 0.0;
            public static final InterpolatingDoubleTreeMap TAG_SHOT_RPM_BY_HUB_DIST = new InterpolatingDoubleTreeMap();
            public static final InterpolatingDoubleTreeMap TAG_SHOT_HOOD_DEG_BY_HUB_DIST = new InterpolatingDoubleTreeMap();
            static {
                // hub dist (m)                      flywheel rpm  // tape (frame -> hub face), motor rpm
                TAG_SHOT_RPM_BY_HUB_DIST.put(1.778, 1300.0);      // 54 in, square-ish to side face 5/8, by the bump
                TAG_SHOT_RPM_BY_HUB_DIST.put(2.356, 1400.0);      // 68.5 in, 51 deg off front face
                TAG_SHOT_RPM_BY_HUB_DIST.put(2.868, 1450.0);      // 84 in, 52 deg off front face
                TAG_SHOT_RPM_BY_HUB_DIST.put(3.083, 1450.0);      // 115.5 in, square to side face 5/8 (RT make 1700/23.4; RB overshot at the trench -> team 10-10: -100 rpm, +1 deg; still overshot -> another -100; team 10-17: another -50)
                TAG_SHOT_RPM_BY_HUB_DIST.put(3.989, 1600.0);      // 131 in, side tag 8 only, ~45 deg, clean
                TAG_SHOT_HOOD_DEG_BY_HUB_DIST.put(1.778, 12.4);   // deg above the bottom stop
                TAG_SHOT_HOOD_DEG_BY_HUB_DIST.put(2.356, 15.8);
                TAG_SHOT_HOOD_DEG_BY_HUB_DIST.put(2.868, 19.2);
                TAG_SHOT_HOOD_DEG_BY_HUB_DIST.put(3.083, 24.4);
                TAG_SHOT_HOOD_DEG_BY_HUB_DIST.put(3.989, 26.0);
            }
            // LB FEED SHOT (REMADE 2026-10-17 from 11 team field points, docs/feed-data/p*.json; the
            // 2026-10-12 trench/hub-tag landing points + the DERIVED table are gone -- last in commit 435ccb1).
            // Any tag in view places the robot (FeedAim); everything else is worked out in the RED frame
            // (blue = the field turned 180 deg) and never uses which side (y) the robot is on, so the
            // low-y red data covers the high-y side mirrored, and blue mirrors red. Every feed goes
            // STRAIGHT toward our alliance wall at HOOD_MAX_DEG (every point was shot at max hood:
            // commanded 29, read 28.0-28.7). No position known yet (no tag since boot / a pose reset) =
            // turn toward our hub by odometry, flywheels at LB_SHOT_FLYWHEEL_RPM, belts WAIT (team 2026-10-17).
            // GENTLE TURNING (team 2026-10-17, "not turn as much"): inside +-WINDOW of the feed heading the
            // robot is not turned at all; outside, it turns back only to the window's edge, at most
            // MAX_ROTATION. Belts feed within TOLERANCE = WINDOW + 2 deg (a robot held at the edge still
            // feeds; the 10 good points spanned -15..+6 deg). Drive stays at the 10% shot cap.
            public static final double FEED_HEADING_WINDOW_DEG = 10.0;
            public static final double FEED_HEADING_TOLERANCE_DEG = 12.0;
            public static final double FEED_MAX_ROTATION_RAD_PER_SEC = Math.toRadians(90); // RT/RB: 270 deg/s
            // The robot's tag-pinned position is dropped when odometry jumps more than this in ONE loop
            // (a pose reset: auto start, the MT1 boot seed; full speed is ~0.05 m/loop).
            public static final double FEED_MAX_ODOM_JUMP_METERS = 0.5;
            // ROBOT MOTION (team 2026-10-17): the rpm is picked for where the robot WILL be after the
            // ball's flight time (robot velocity x this), so driving at our wall feeds shorter.
            // [derived] docs/shot-model drag model at full hood (1600 rpm -> 1.50 s, 2000 -> 1.81 s),
            // NOT measured -- TODO film a feed at 240 fps (wheel -> floor) and replace. Same wall-distance key.
            public static final InterpolatingDoubleTreeMap FEED_TOF_SEC_BY_WALL_DIST = new InterpolatingDoubleTreeMap();
            static {
                FEED_TOF_SEC_BY_WALL_DIST.put(8.0, 1.50);   // 1600 rpm  TODO [derived] measure
                FEED_TOF_SEC_BY_WALL_DIST.put(8.8, 1.60);   // 1730 rpm
                FEED_TOF_SEC_BY_WALL_DIST.put(13.5, 1.81);  // 2000 rpm
            }
            // BEHIND A HUB = NO SHOT (team 2026-10-17, point p6: "can not work here or closer"): a hub
            // between the robot and our wall, the robot inside HALF_WIDTH of its centre line and less than
            // DEPTH in front of its centre = the ball hits it; LB only aims. Both hubs (ours from the neutral
            // zone, theirs from inside their zone). Once blocked, both grow by HYST (no flicker at the edge).
            public static final double FEED_HUB_BLOCK_DEPTH_METERS = 1.8;      // p6 failed 1.71 m ahead, p5 worked 2.52 [data] TODO narrow
            public static final double FEED_HUB_BLOCK_HALF_WIDTH_METERS = 0.8; // hub half 0.60 + ball 0.08 + aim error [assumed] TODO check
            public static final double FEED_HUB_BLOCK_HYST_METERS = 0.1;
            // Distance from OUR alliance wall (m, red frame 16.541 - x of the camera) -> flywheel rpm. Every
            // row = shots that WORKED (team). Between rows interpolated; past the ends the end row holds.
            public static final InterpolatingDoubleTreeMap FEED_RPM_BY_WALL_DIST = new InterpolatingDoubleTreeMap();
            static {
                FEED_RPM_BY_WALL_DIST.put(7.1, 1600.0);  // p5 7.14, p2 7.32, p3 7.41: all 1600
                FEED_RPM_BY_WALL_DIST.put(8.0, 1600.0);  // p4 7.84, p1 7.97: 1600
                FEED_RPM_BY_WALL_DIST.put(8.8, 1730.0);  // p9 8.60 1700, p7 8.79 1800, p8 8.99 1700 (all worked: mean)
                // 8.8 -> 13.5 m: interpolated, NO data (the far half of the neutral zone) TODO add points
                FEED_RPM_BY_WALL_DIST.put(13.5, 2000.0); // p10 13.50 (their zone; tag 22 standing in for 28)
                FEED_RPM_BY_WALL_DIST.put(14.0, 2000.0); // p11 14.01 (their zone; tags 27/18). Far side: whether
                                                         // "worked" meant reaching OUR zone was never confirmed
            }
            // Rotation feedforward = this x the rate the hub bearing changes as the robot
            // drives. 1.0 = the exact geometric rate; lower it if the heading overshoots.
            public static final double HEADING_RATE_FF_SCALE = 1.0; // TODO tune on robot
            // Virtual-target refinement passes (LUNATECS 10, Ninjas/1833 5; converges in ~3).
            public static final int VIRTUAL_TARGET_ITERATIONS = 5;
            // Launcher position from robot centre, robot frame (x fwd, y left), metres. Team
            // 2026-10-09: ball leaves along robot +X (shooter side = front), on the centreline.
            public static final Translation2d ROBOT_TO_LAUNCHER_METERS = new Translation2d(0.0, 0.0); // TODO measure on robot

            // SHOT TABLES, keyed on distance launcher -> hub centre (m). Between rows the value
            // is interpolated; past the ends it holds the end row.
            // *** ALL ROWS ARE [placeholder] -- NO V2 SHOT DATA EXISTS YET (V1 tables are invalid
            // for V2, data sheet sec.4.2). *** Flat rpm/hood = RT's start rpm and the homed
            // bottom stop, so RB never spins or tilts further than the team already runs.
            // HOW TO TUNE: park at a distance (read "Shot Distance (m)" on the Shooter tab), find
            // the rpm + hood that score using RT + DPAD, add a row here. Time of flight: film a
            // shot at 240 fps, count frames from leaving the wheel to entering the hub.
            public static final InterpolatingDoubleTreeMap SHOT_RPM_BY_DISTANCE = new InterpolatingDoubleTreeMap();
            public static final InterpolatingDoubleTreeMap SHOT_HOOD_DEG_BY_DISTANCE = new InterpolatingDoubleTreeMap();
            public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_SEC_BY_DISTANCE = new InterpolatingDoubleTreeMap();
            static {
                SHOT_RPM_BY_DISTANCE.put(1.0, 2000.0);       // TODO [placeholder] flywheel rpm
                SHOT_RPM_BY_DISTANCE.put(6.0, 2000.0);       // TODO [placeholder]
                SHOT_HOOD_DEG_BY_DISTANCE.put(1.0, 0.0);     // TODO [placeholder] deg above bottom stop
                SHOT_HOOD_DEG_BY_DISTANCE.put(6.0, 0.0);     // TODO [placeholder]
                // ~1 s is what the three reference robots measured at 2-4 m -- NOT Helios data.
                TIME_OF_FLIGHT_SEC_BY_DISTANCE.put(1.0, 1.0); // TODO [placeholder] measure
                TIME_OF_FLIGHT_SEC_BY_DISTANCE.put(6.0, 1.0); // TODO [placeholder] measure
            }
        }

        public static class Vision{
            public static final String CAM_LIMELIGHT = "limelight-knight";

            // AprilTag pose correction (CommandSwerveDrivetrain.updateVisionPose), enabled by
            // team request 2026-07-17: MegaTag2 corrects X/Y continuously (yaw stays
            // Pigeon-owned) and MegaTag1 seeds the full pose ONLY until the first enable
            // after boot. The in-match auto re-seed was REMOVED 2026-07-21 by team request:
            // once the match starts, field centering happens ONLY via the driver's MENU
            // press. PREREQUISITE -- the Limelight's camera mount pose MUST be set in its
            // web UI first: botpose is systematically wrong without it, and the gates below
            // cannot catch a consistent mount-offset error. TODO verify on robot before
            // first match.
            public static final boolean ENABLE_MEGATAG2_POSE = true;

            // Boot-seed quality gates (robot sitting still pre-match, never yet enabled):
            // adopt the full MT1 tag pose whenever it disagrees with odometry.
            public static final double VISION_SEED_MAX_TAG_DIST_METERS = 5.0;  // ignore solves with avg tag distance beyond this [assumed] TODO tune
            public static final double VISION_SEED_POS_TOL_METERS = 0.25;      // reseed when position is off by more than this
            public static final double VISION_SEED_YAW_TOL_DEG = 3.0;          // reseed when heading is off by more than this

            // VISION AUTO ("BLine: path 1 + vision", team 2026-10-17): MegaTag2 X/Y keeps correcting the pose
            // WHILE BLine drives. Built from the 10-10 shot-session Limelight data (14 standing-still points,
            // practice hub): the robot-space tag distance matched tape to ~5 cm RMS from 1.0 to 3.3 m, with no
            // trend vs distance or ambiguity, and detection fell off past ~2.9 m head-on (a single tag at
            // 2.9 m / 0.17% area was seen in only 54% of frames). Tag size tracks distance, so the distance
            // cap covers it. There is NO moving data, so the std devs sit well above that 5 cm: against the
            // 0.05 m odometry std dev (TunerConstants) a 5 cm vision std dev would pull half the gap per frame
            // and snap the pose; 0.15 m pulls ~1/4, 0.25 m ~1/6, x(1 + (d/3.3)^2) further out.
            // Vision is OFF while the Limelight reports a zero mount pose (unset in its web UI): unset, botpose
            // is the camera's spot, ~half a robot length ahead of centre and swinging with the heading.
            public static final double VISION_AUTO_MAX_TAG_DIST_METERS = 3.3;   // data's range [data 2026-10-10] TODO extend with farther points
            public static final double VISION_AUTO_STDDEV_MULTI_TAG_METERS = 0.15; // 2+ tags [assumed: 3x the static scatter] TODO tune on robot
            public static final double VISION_AUTO_STDDEV_ONE_TAG_METERS = 0.25;   // 1 tag [assumed: 5x] TODO tune on robot
            public static final double VISION_AUTO_MAX_JUMP_METERS = 0.5;       // farther from odometry = bad solve, dropped [assumed] TODO tune
            public static final double VISION_AUTO_MAX_YAW_RATE_DEG_PER_SEC = 180; // motion blur while turning [assumed] TODO tune
            // Path 1 + vision STEP 3 (post-feed-shot, team 2026-10-17): the OPPONENT's hub + trench tags count out to
            // this far (every other tag keeps the 3.3 m cap) -- on the leg back past the trench they sit 3.4-4.5 m
            // away and the 3.3 m cap threw out every frame. Past our 3.3 m data; trust still falls with distance
            // (~1/3 at 4.5 m). TODO verify with moving data.
            public static final double VISION_AUTO_FAR_TAG_MAX_DIST_METERS = 4.5; // [assumed, team-picked]
            public static final java.util.Set<Integer> RED_HUB_TRENCH_TAGS = java.util.Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
            public static final java.util.Set<Integer> BLUE_HUB_TRENCH_TAGS = java.util.Set.of(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28);

            // ---- Limelight thermal / power management (LimelightThermalManager) ----
            // HARDWARE REALITY, measured on THIS camera 2026-07-18 via its /status endpoint:
            // the Limelight 4's heat is dominated by its on-board Hailo AI accelerator -- a
            // FIXED ~3.75 W / ~74 C load that is powered whenever the camera runs and does NOT
            // respond to pipeline resolution, framerate, LED state, or CPU load (verified by
            // sweeping all of them: hailoTemp/hailoPower never moved off 74 C / 3.75 W, and it
            // is already 74 C ~2 s after a cold boot). The AprilTag pipeline does not even use
            // the Hailo (its neural runtimes are CPU), yet it stays hot.
            //   => Robot code CANNOT throttle the actual hot chip. This manager does what it
            //      CAN: (1) force LEDs OFF (the only camera power the RIO controls; passive
            //      AprilTags never need them); (2) drop to a lower-res "idle" AprilTag pipeline
            //      when not shooting -- shaves only the CPU/board heat and keeps low-res
            //      MegaTag2 pose fusion alive; (3) read the temps and WARN the driver.
            //   => The real fix for genuine overheating is PHYSICAL: confirm the camera's fan
            //      spins and improve enclosure airflow/heatsinking. Software only warns.
            //
            // Pipeline indices. NORMAL = full-res AprilTag (best shot ranging). IDLE = lower-
            // res AprilTag (still feeds pose fusion, a bit less CPU/board heat). IDLE MUST be an
            // AprilTag pipeline that EXISTS on the camera -- a non-fiducial or missing idle
            // pipeline blinds MegaTag2 when the shooter is idle. Set LL_PIPELINE_IDLE = 0 (same
            // as NORMAL) to DISABLE idle switching until pipeline 1 is created on the Limelight
            // (clone 0, LEDs off, lower resolution). TODO create/confirm pipeline 1 on robot.
            public static final int LL_PIPELINE_NORMAL = 0;
            // Set to 0 (== NORMAL) 2026-07-18 to DISABLE idle-pipeline switching. If pipeline 1
            // isn't a working AprilTag pipeline on this camera, the thermal manager was switching to
            // it whenever the shooter idled and BLINDING vision -- which is exactly the RT auto-aim
            // bootstrap state (flywheels stay off until a target is found), so auto-aim could never
            // acquire a tag (e.g. tag 18). Idle switching only sheds board/CPU heat (never the
            // dominant Hailo load), so disabling it costs ~nothing. Create pipeline 1 (clone 0,
            // lower res, LEDs off), confirm it exists, then set this back to 1 to restore idle
            // power-saving. TODO create/confirm pipeline 1 on robot.
            public static final int LL_PIPELINE_IDLE = 0;

            // The camera reports hailoTemp/temp only over HTTP (verified: the NT 'hw' array does
            // not carry the Hailo temp). Polled OFF the main loop by a Notifier; robotPeriodic
            // only reads cached values, so a slow/at camera never stalls the 20 ms loop.
            public static final int    LL_STATUS_PORT = 5807;
            public static final double LL_STATUS_POLL_SEC = 1.0;    // thermal changes are slow; 1 Hz is plenty
            public static final double LL_STATUS_TIMEOUT_SEC = 0.3; // per-request connect+read cap

            // Thermal thresholds on hailoTemp (deg C), hysteresis: engage idle at HOT, release
            // at COOL. WARN just alerts the driver. PLACEHOLDERS -- the LL4 Hailo-8 shutdown
            // point is not published and it idles ~74 C in open air (climbs in an enclosure);
            // tune these against the real temperature the camera reaches on the robot. TODO tune.
            public static final double LL_TEMP_WARN_C = 80.0;   // TODO tune: driver warning
            public static final double LL_TEMP_HOT_C  = 83.0;   // TODO tune: engage thermal idle
            public static final double LL_TEMP_COOL_C = 78.0;   // TODO tune: release thermal idle

            // Plausibility gate: only act on a reading inside a sane range, so a bad/missing
            // read can never force the idle pipeline (and degrade vision) on garbage data.
            public static final double LL_TEMP_SANE_MIN_C = 10.0;
            public static final double LL_TEMP_SANE_MAX_C = 120.0;

            // Hold the full pipeline this long after the shooter last went inactive, so rapid
            // spin-up/down does not churn the pipeline (each switch costs a few frames of no
            // targets) and the camera is already full-res the instant the next shot starts.
            public static final double LL_IDLE_HOLD_FULL_SEC = 3.0;  // TODO tune

            // Rate-limit the over-temp warning so a sustained event does not spam the DS console.
            public static final double LL_WARN_INTERVAL_SEC = 5.0;
        }

        public static class IntakeSubsystemConstants{
            public static final int INTAKE_MOTOR_ID = 16;
            public static final int INTAKE_PIVOT_MOTOR_ID = 22;

            //HARDWARE CONSTANTS

                // Slider remade 2026-07-14, dead simple: fixed SLOW duty, tiered smart limit
                // (Hardware-Data-Sheet sec.7 conservative start -- too weak to grind anything
                // at a hard stop), and TWO stop rules (either -> cut output, brake holds):
                //   a) encoder stopped turning = hard stop or jam;
                //   b) current pegged at the smart limit = stalled even though the motor is
                //      still spinning (belt/gear slip at full extension -- observed 2026-07-15:
                //      encoder-only detection missed it and the motor kept pushing).
                public static final double SLIDER_DUTY = 0.45;               // travel duty (+50% from 0.3 per team request 2026-07-16); TODO verify hard-stop impact on robot
                public static final double SLIDER_STALL_MOTOR_RPM = 300;    // motor rpm below this = "not moving" (raised from 100 so slow slip-creep at the hard stop also reads as stalled; free travel at 0.45 duty is ~2500 motor rpm); TODO tune on robot
                public static final double SLIDER_STALL_CURRENT_AMPS = 12;  // measured amps at/above this = stalled (smart limit clamps ~15 A at 0 rpm; free travel draws only a few amps) [assumed] TODO tune on robot
                public static final double SLIDER_CURRENT_STALL_DEBOUNCE_SEC = 0.05; // fast path: current needs no spin-up grace; ~2-3 loop cycles is the floor for a real signal
                public static final double SLIDER_STALL_DEBOUNCE_SEC = 0.15; // velocity path (was 0.25; tightened for faster cutoff 2026-07-16)
                public static final double SLIDER_GRACE_SEC = 0.3;          // covers spin-up from rest (encoder reads 0 rpm at move start; must outlast the 0.25 s ramp)
                public static final double SLIDER_MOVE_TIMEOUT_SEC = 2.5;   // backstop only (was 3.0; travel is ~33% shorter at 0.45 duty); TODO measure real travel time
                // RT/RB/LB shot agitation: extend this long, then retract to stow, repeat (team
                // 2026-09-30; removed 2026-10-10 for a one-time slow stow, re-added 2026-10-10).
                // 0.5 -> 0.35 (team 2026-10-10, "a little faster"): shorter stroke at the same
                // speed, so ~30% more cycles and no harder hits on the stow stop.
                public static final double SLIDER_CYCLE_EXTEND_SEC = 0.35;

            //SPEED CONSTANTS
                // Roller duty cycles (driver-tuned 2026-07-10: 35% intake was too fast;
                // outtake halved 2026-07-14 per team request). Worst-case steady supply draw
                // = duty x stator limit (40 A, set in IntakeSubsystem): intake 0.90 x 40 =
                // 36 A (a stall is clipped by the 25 A supply limit), outtake 0.360 x 40 = 14.4 A.
                // Inrush softened by the duty-cycle ramp.
                public static final double INTAKE_SPEED = 0.900;    // team 2026-10-17 (0.70 on 2026-10-16, was 0.80, driver-tuned 2026-08-28)
                public static final double OUTTAKE_SPEED = 0.360;   // driver-tuned on robot 2026-08-28
                // Rollers (still pulling in) while LT's release stows the slider -- team 2026-10-14:
                // 10% of the intake speed; 70% on 2026-10-16, then 25%; 12.5% = 0.1125 on 2026-10-17. A stall is held
                // at the 40 A stator limit, ~0.1125 x 40 A = 4.5 A supply.
                public static final double STOW_ROLLER_SPEED = INTAKE_SPEED * 0.125;
                // Roller duty for the whole RT/RB/LB shot hold (team 2026-10-07: 5%; raised to 10%
                // 2026-10-09, "a little faster"; back to 5% 2026-10-10, "spin at their slowest").
                // Inward while the slider extends, outward while it retracts (team 2026-10-10).
                // Stall worst case ~0.05 x 40 A stator = 2 A. TODO tune.
                public static final double ROLLER_AGITATE_SPEED = 0.05;

        }

        public static class HopperSubsystemConstants{
            public static final int HOPPER_ID_A = 20;
            public static final int HOPPER_ID_B = 21;
            // (Kicker Victor SPX at CAN 17 REMOVED 2026-09-30 -- V2 feeder is LauncherConstants.FEEDER_ID.)
            // Motor -> belt pulley reduction, total (team 2026-10-17: now 5:1; the data sheet's CAD read was
            // 3.889:1). Record only -- every belt speed below is a raw motor duty, so at the same duty the
            // belts now run ~22% slower than at 3.889:1 with ~29% more force at the same current limit.
            public static final double HOPPER_GEAR_RATIO = 5.0; // motor rotations per belt pulley rotation
            
            //SPEED CONSTANTS
                //                                                                                                                                                                          Belt duty. Conservative start (old code ran 0.8, never verified on robot);
                // direction test 2026-07-14 confirmed both motors agree, positive = tested
                // direction. TODO raise toward 0.8 once feed direction + throughput verified.
                public static final double HOPPER_SPEED = 1;   // +20% team request 2026-08-24
                // Belt duty while the intake runs (LT / Y feed) -- team 2026-10-09: 50% of
                // HOPPER_SPEED; team 2026-10-10: halved again -> 25%. The A manual run and the
                // shots keep their own speeds.
                public static final double HOPPER_INTAKE_SPEED = HOPPER_SPEED * 0.25;
                // Belt duty for a SHOT (B feed shot / RT / RB) -- team request 2026-08-29:
                // "twice as fast". Doubling 0.6 is 1.2, which is not a thing: 1.0 is full
                // output, so this lands at 1.0 and the real increase is 1.67x, not 2x. Written
                // as an explicit min() rather than the literal 1.0 so that if HOPPER_SPEED is
                // ever lowered, the doubling comes back on its own instead of silently staying
                // pinned at full.
                //
                // ONLY the shot paths use this; the LT intake feed and the A manual hopper run
                // stay at HOPPER_SPEED, which is what the team asked for.
                //
                // WATCH THE CURRENT LIMIT, not the duty: the belts run at
                // HOPPER_SHOT_CURRENT_LIMIT_A (24 A since 2026-09-30) during a shot, so under real fuel load the
                // smart limit -- not this number -- may be what decides the speed. If the belts
                // do not actually run faster, that limit is the thing to raise, and raising it
                // is a team decision, not a side effect of this one.
                public static final double HOPPER_SHOT_SPEED = Math.min(1.0, HOPPER_SPEED * 2.0);
                // Reverse duty for the unjam button (belts, held; the feeder has its own).
                public static final double UNJAM_SPEED = 0.3;
                // RT shot: belts run backward at UNJAM_SPEED (and the launcher feeder backward at
                // FEEDER_SHOT_REVERSE_DUTY) for this long at the start of every shot, then feed
                // normally (team request 2026-10-05; feeder added 2026-10-07). Tunable.
                public static final double HOPPER_SHOT_REVERSE_SEC = 1.0; // 0.5 -> 1.0 team 2026-10-10
                // Slow duty for the single-belt direction tests (VIEW / DPAD-DOWN): slow enough
                // that even a full stall sits well under the 20 A smart limit.
                public static final double DIRECTION_TEST_SPEED = 0.05;
                // (INTAKE_KICKER_SPEED removed 2026-07-18: team spec "during intake only the
                // hopper runs, not the kicker" -- intakeFeedCommand no longer touches the kicker.
                // The stall analysis it carried still applies to ANY kicker use against stopped
                // flywheels: CIM stall at 12 V is ~131 A and the Victor SPX has no current
                // sensing, so duty is the only software knob -- see manualRunCommand.)
                
                // Belt smart current limits (team, 2026-09-30): 5 -> 20 A normal, later 30, 35 on
                // 2026-10-14. The RT shot limit is x1.2 but capped at 40 A (team 2026-10-14) so it
                // never exceeds the branch breaker. Manual (A held) stays 40 A. Data sheet sec.7 cap is 60 A.
                public static final int HOPPER_CURRENT_LIMIT_A = 35;
                public static final int HOPPER_SHOT_CURRENT_LIMIT_A = Math.min((int)(HOPPER_CURRENT_LIMIT_A * 1.2), 40);
                // Belt limit while B (manual hopper run) is held -- team request 2026-08-27
                // (30 -> 60), lowered to 40 on 2026-08-28. 40 A sits at/below the branch
                // breaker, so the software limit now acts before the breaker does, and stays
                // under the data-sheet 60 A cap for a hopper NEO 2.0.
                public static final int HOPPER_MANUAL_CURRENT_LIMIT_A = 40;
        }

        // V2 LAUNCHER (drum shooter, 2026-09-30). Hardware: Hardware-Data-Sheet sec.4.2 [CAD-v2].
        // Replaces the V1 ShooterSubsystemConstants (deleted 2026-09-30 with the V1 shooter;
        // last present in commit ccd5075).
        public static class LauncherConstants{
            // CAN IDs -- team-assigned 2026-09-30. 15 = top X60 on the 4-motor panel (belts the
            // flywheel), 14 = the X60 under it (belted 1:1 motor-to-motor to 15), 13 = the single
            // X60 on the other panel, 18 = hood X44 (the old intake-roller Kraken).
            public static final int FLYWHEEL_LEADER_ID = 15;
            public static final int FLYWHEEL_BELTED_ID = 14;
            public static final int FLYWHEEL_RIGHT_ID = 13;
            public static final int HOOD_ID = 18;
            // Feeder = the BRAND-NEW X60 (lowest on the 4-motor panel). 24 is the first ID never
            // used on this robot. TODO team: set this motor to CAN 24 in Phoenix Tuner X --
            // until then the feeder reads 0 rpm, the belt gate never opens, and RT won't feed.
            public static final int FEEDER_ID = 24;

            //FLYWHEEL
                // 18T motor pulley -> 26T flywheel pulley = 1.444 motor rot per flywheel rot [CAD-v2].
                public static final double FLYWHEEL_MOTOR_ROT_PER_FLYWHEEL_ROT = 26.0 / 18.0;
                // RT target in FLYWHEEL rpm (team 2026-09-30): start 2000, DPAD L/R -/+100,
                // clamped 0-4000. 4000 is ~X60 free speed through 26:18 -- not reachable under load.
                public static final double FLYWHEEL_START_RPM = 2000;
                public static final double FLYWHEEL_STEP_RPM = 100;
                public static final double FLYWHEEL_MAX_RPM = 4000;
                // LB spin-up rpm while no position is known yet (was the 2026-10-10 fixed shot; since 2026-10-17
                // the belts wait for a tag instead of firing it -- FeedAim).
                public static final double LB_SHOT_FLYWHEEL_RPM = 1800;
                // TELEOP ZONE IDLE (team 2026-10-17, AllianceZone): flywheel rpm the wheels idle at in our
                // alliance zone. Feedforward volts only (~2.4 V), so it settles a little under this.
                public static final double IDLE_FLYWHEEL_RPM = 800;
                // Idle runs this long after the last zone-tag sighting; each sighting restarts it (team 2026-10-17).
                public static final double IDLE_HOLD_SEC = 5.0;
                // Past that, it stays on only while the robot is within this distance of where it last saw a
                // zone tag (turned in place, didn't drive away) -- odometry. Team 2026-10-17. [assumed] TODO tune.
                public static final double IDLE_ODOMETRY_HOLD_MAX_MOVE_METERS = 0.3;
                // Idle stall cutoff (team 2026-10-17): pushing but under this flywheel rpm for this long =
                // jammed -> coast (a stopped wheel at 2.4 V pulls ~74 A stator per motor). Re-arms after
                // the next shot or a zone exit. [assumed] TODO check on the robot that spin-up never trips it.
                public static final double IDLE_STALL_RPM = 100;
                public static final double IDLE_STALL_SEC = 0.5;
                // "At speed" band in flywheel rpm. Team 2026-10-10: the belts START only once the
                // wheels reach the target (target .. +this), then keep feeding down to target -
                // FLYWHEEL_FEED_DROP_TOL_RPM (LauncherSubsystem.flywheelAtSpeed). [assumed] TODO tune.
                public static final double FLYWHEEL_AT_SPEED_TOL_RPM = 100;
                // 100 -> 50 (team 2026-10-10): slowed balls fed at -100 rpm (~7% slow) were missing,
                // so the belts now pause sooner and wait for the target again.
                public static final double FLYWHEEL_FEED_DROP_TOL_RPM = 50;
                // BANG-BANG flywheel control (team request 2026-10-05; WPILib BangBangController,
                // the pattern from the WPILib docs): volts = (below target ? 12 : 0) + FF_SCALE *
                // kV * target. The feedforward is held under 100% so it can never over-speed the
                // wheel on its own -- bang-bang only pushes forward, it can't slow the wheel down.
                // kV in volts per MOTOR rps (X60: ~12 V / 96.7 rps; V1's measured 0.12625).
                // Tune: if the speed overshoots on its own, lower FF_SCALE; if recovery between
                // balls is slow, the current limits below (not this) are what cap it.
                public static final double FLYWHEEL_kV = 0.12625;
                public static final double FLYWHEEL_BANG_BANG_FF_SCALE = 0.9;
                public static final double FLYWHEEL_BANG_BANG_VOLTS = 12.0;
                // Current limits, per motor (x3). 50/25 -> 80/40 (team 2026-10-10): a ball slowed
                // the wheels and the next few missed; at shot speeds both limits bound at ~50 A
                // stator, so this is ~40% more torque to recover. 40 A supply = data-sheet cap
                // (sec.7): up to ~120 A from the 3 while recovering -- watch for brownouts.
                public static final double FLYWHEEL_STATOR_A = 80;
                public static final double FLYWHEEL_SUPPLY_A = 40;
                // Flywheel direction. Flipped false -> true 2026-10-05 (team: the flywheels spun the
                // wrong way on the robot). 14 inherits this (XOR its own flag) and 13 follows 15's
                // actual direction, so this one constant reverses all three together.
                public static final boolean FLYWHEEL_LEADER_INVERTED = true;
                // Direction relative to 15 [CAD-inferred, NOT verified]: 14 faces the same way as
                // 15 -> same sign; 13 is mirrored on the other panel -> Opposed.
                // A wrong 14 makes 14 and 15 fight at the current limit -- spin-test first.
                public static final boolean FLYWHEEL_BELTED_OPPOSED = false;
                // HARDWARE != CAD (team, 2026-10-05): CAN 14's pulley is 22T, not the CAD's 18T. It
                // belts to the 18T on 15's shaft, so 14 turns 18/22 = 0.818x as fast as 15. 14 is
                // therefore NOT a plain follower: it gets the leader's voltage x this ratio, which
                // puts both motors at the same fraction of free speed so they share the load
                // instead of 14 pushing while 15 brakes.
                public static final double FLYWHEEL_BELTED_SPEED_RATIO = 18.0 / 22.0;
                public static final boolean FLYWHEEL_RIGHT_OPPOSED = true;

            //FEEDER (X60 -> 12T:30T = 2.5:1 -> 3 tube rollers)
                public static final double FEEDER_TARGET_MOTOR_RPM = 3000; // team 2026-09-30
                public static final double FEEDER_AT_SPEED_FRACTION = 0.10; // within 10% = at speed
                public static final double FEEDER_kP = 0.1;   // motor rps; [assumed] TODO tune
                public static final double FEEDER_kV = 0.12;  // ~12 V / 96.7 rps X60 free speed
                public static final double FEEDER_STATOR_A = 30; // team-approved 2026-09-30
                public static final double FEEDER_SUPPLY_A = 20;
                public static final double FEEDER_RAMP_SEC = 0.25;
                // Feeder duty BACKWARD during the HOPPER_SHOT_REVERSE_SEC window at the start of
                // every shot (RB and RT), and while VIEW unjam is held (team 2026-10-07).
                // 0.05 -> 0.30 (team 2026-10-10, "+25% power": it stalled too easily at 5%).
                // 0.30 -> 0.70 (team 2026-10-10). A jam now hits the 20 A supply cap (~21 A
                // supply at 30 A stator x 0.70 duty) -- don't hold VIEW on one.
                // [assumed] TODO tune on robot.
                public static final double FEEDER_SHOT_REVERSE_DUTY = 0.4;
                public static final boolean FEEDER_INVERTED = false; // TODO verify feed direction

            //HOOD (X44 -> 12T:30T -> 10T pinion -> 170T sector = 42.5:1) [CAD-v2; sector teeth unconfirmed]
                public static final double HOOD_MOTOR_ROT_PER_HOOD_ROT = 2.5 * 17.0;
                public static final double HOOD_MAX_DEG = 29;   // above the homed bottom stop. Measured 2026-10-07: hard top = 29.7 deg above the bottom (4.13 -> 33.84 deg); 0.7 deg margin so Motion Magic never drives into the stop
                public static final double HOOD_JOG_DEG_PER_SEC = 15; // DPAD up/down hold rate (0->30 in 30 s), team 2026-10-07
                public static final double HOOD_STATOR_A = 20;  // team-approved 2026-09-30
                public static final double HOOD_SUPPLY_A = 15;
                // TODO verify on robot: negative output must move the hood DOWN. Watch the first
                // homing after every deploy -- if the hood moves UP, disable and flip this.
                public static final boolean HOOD_INVERTED = false;
                // Homing (every enable): drive down at a low voltage until the stator current
                // shows the bottom stop. The grace window keeps startup inrush from reading as
                // "hit the stop" before the hood has moved (same idea as the slider's grace).
                public static final double HOOD_HOMING_VOLTS = 1.5;
                public static final double HOOD_HOMING_GRACE_SEC = 0.3;
                public static final double HOOD_HOMING_STOP_AMPS = 12;     // [assumed] TODO tune
                public static final double HOOD_HOMING_DEBOUNCE_SEC = 0.1;
                public static final double HOOD_HOMING_TIMEOUT_SEC = 3.0;
                // Motion Magic in HOOD rotations (SensorToMechanismRatio = 42.5).
                public static final double HOOD_CRUISE_RPS = 0.25;  // 90 deg/s
                public static final double HOOD_ACCEL_RPS2 = 1.0;
                // Hood droop fix (team 2026-10-09: "hood is slightly going down during shooting").
                // At kP 150 it settled 0.7-1.0 deg BELOW target (9.5/10.5, 14.2/15.0, 17.3/18.0) --
                // gravity balanced by error alone, ~0.35 V. kG supplies that voltage up front;
                // kP 300 = 1 deg droop asks ~0.83 V ~ 19 A, i.e. the whole 20 A stator cap, so
                // more kP can't add holding force. Was kP 150, no kG. TODO verify on robot: no
                // buzz/overshoot on DPAD moves, and the hood holds during a shot.
                public static final double HOOD_kP = 300;  // V per hood rot: 1 deg error ~ 0.83 V
                public static final double HOOD_kG = 0.3;  // V, constant (Elevator_Static): cancels the hood's weight
                public static final double HOOD_kV = 4.0;  // ~12 V / (125.5 rps / 42.5)
                // Stall cutoff during normal moves = the hood is PINNED against something (wrong
                // travel, sign, or ratio): well away from target, not moving, and current near the
                // 20 A cap, for 0.5 s. Deliberately NOT "any current while off target" -- a
                // stationary X44 draws ~23 A per volt (0.044 ohm), so a normal P-loop settle ~1.6 deg
                // short already reads ~15 A. Latches a fault; the hood goes neutral (brake) until
                // the next enable re-homes it. A stop within ~3 deg of target isn't flagged, but the
                // motor is capped at 20 A (~12.5 N*m at the sector) either way.
                public static final double HOOD_STALL_AMPS = 18;          // [assumed] TODO tune
                public static final double HOOD_STALL_DEBOUNCE_SEC = 0.5;
                public static final double HOOD_STALL_MIN_ERROR_DEG = 3.0;
                public static final double HOOD_STALL_MAX_SPEED_DEG_PER_SEC = 2.0;
                public static final double HOOD_REACHED_TOL_DEG = 1.0;   // dashboard "reached" + the RB feed gate. TODO widen if RB waits on the hood
        }
}
