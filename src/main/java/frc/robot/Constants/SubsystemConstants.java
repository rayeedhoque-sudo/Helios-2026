package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
            // PRACTICE (team 2026-10-09): the test tag (tag 4; was 18) acts like front tags
            // 26/10. Tag 4 is the RED hub's centred neutral-zone tag in the field map (hub centre
            // 0.6035 m behind it, same as 26/10), so RB aims at the red hub whatever alliance the
            // DS says, and "forward" = facing tag 4 (field heading 0), so MENU pressed facing the
            // tag zeroes correctly. Ignored whenever an FMS is attached. Set false to practise on
            // a full field without FMS. To change tag: the hub it sits on + the field heading
            // of a robot facing it (2026-rebuilt-welded.json tag yaw + 180 deg).
            public static final boolean PRACTICE_TEST_TAG = true;
            public static final Translation2d TEST_TAG_HUB = FieldConstants.HUB_CENTER_RED;  // tag 4
            public static final double TEST_TAG_FACING_DEG = 0.0; // tag 4 faces -X (yaw 180)
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
                // RT shot agitation: extend this long, then retract to stow, repeat (team 2026-09-30).
                public static final double SLIDER_CYCLE_EXTEND_SEC = 0.5;

            //SPEED CONSTANTS
                // Roller duty cycles (driver-tuned 2026-07-10: 35% intake was too fast;
                // outtake halved 2026-07-14 per team request). Worst-case steady supply draw
                // = duty x stator limit (40 A, set in IntakeSubsystem): intake 0.254 x 40 =
                // 10.2 A, outtake 0.360 x 40 = 14.4 A, both well under the 25 A supply limit.
                // Inrush softened by the duty-cycle ramp.
                public static final double INTAKE_SPEED = 0.254;    // driver-tuned on robot 2026-08-28
                public static final double OUTTAKE_SPEED = 0.360;   // driver-tuned on robot 2026-08-28
                // Rollers INWARD at this duty for the whole RT/RB shot hold (team 2026-10-07: 5%;
                // raised to 10% 2026-10-09, "a little faster"). Stall worst case ~0.10 x 40 A
                // stator = 4 A. TODO tune.
                public static final double ROLLER_AGITATE_SPEED = 0.10;

        }

        public static class HopperSubsystemConstants{
            public static final int HOPPER_ID_A = 20;
            public static final int HOPPER_ID_B = 21;
            // (Kicker Victor SPX at CAN 17 REMOVED 2026-09-30 -- V2 feeder is LauncherConstants.FEEDER_ID.)
            
            //SPEED CONSTANTS
                //                                                                                                                                                                          Belt duty. Conservative start (old code ran 0.8, never verified on robot);
                // direction test 2026-07-14 confirmed both motors agree, positive = tested
                // direction. TODO raise toward 0.8 once feed direction + throughput verified.
                public static final double HOPPER_SPEED = 1;   // +20% team request 2026-08-24
                // Belt duty while the intake runs (LT / Y feed) -- team 2026-10-09: 50% of
                // HOPPER_SPEED. The A manual run and the shots keep their own speeds.
                public static final double HOPPER_INTAKE_SPEED = HOPPER_SPEED * 0.5;
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
                public static final double HOPPER_SHOT_REVERSE_SEC = 0.5;
                // Slow duty for the single-belt direction tests (VIEW / DPAD-DOWN): slow enough
                // that even a full stall sits well under the 20 A smart limit.
                public static final double DIRECTION_TEST_SPEED = 0.05;
                // (INTAKE_KICKER_SPEED removed 2026-07-18: team spec "during intake only the
                // hopper runs, not the kicker" -- intakeFeedCommand no longer touches the kicker.
                // The stall analysis it carried still applies to ANY kicker use against stopped
                // flywheels: CIM stall at 12 V is ~131 A and the Victor SPX has no current
                // sensing, so duty is the only software knob -- see manualRunCommand.)
                
                // Belt smart current limits (team, 2026-09-30): 5 -> 20 A normal, so the RT shot
                // limit (x1.2) is 24 A. Manual (A held) stays 40 A. Data sheet sec.7 cap is 60 A.
                public static final int HOPPER_CURRENT_LIMIT_A = 30;
                public static final int HOPPER_SHOT_CURRENT_LIMIT_A = (int)(HOPPER_CURRENT_LIMIT_A * 1.2);
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
                // "At speed" band in flywheel rpm. [assumed] TODO tune: if the belts never start,
                // widen this before touching gains (V1's too-tight band never latched).
                public static final double FLYWHEEL_AT_SPEED_TOL_RPM = 100;
                // BANG-BANG flywheel control (team request 2026-10-05; WPILib BangBangController,
                // the pattern from the WPILib docs): volts = (below target ? 12 : 0) + FF_SCALE *
                // kV * target. The feedforward is held under 100% so it can never over-speed the
                // wheel on its own -- bang-bang only pushes forward, it can't slow the wheel down.
                // kV in volts per MOTOR rps (X60: ~12 V / 96.7 rps; V1's measured 0.12625).
                // Tune: if the speed overshoots on its own, lower FF_SCALE; if recovery between
                // balls is slow, the 25 A supply limit (not this) is what caps it.
                public static final double FLYWHEEL_kV = 0.12625;
                public static final double FLYWHEEL_BANG_BANG_FF_SCALE = 0.9;
                public static final double FLYWHEEL_BANG_BANG_VOLTS = 12.0;
                // Current limits: V1's team-tuned 50 A stator / 25 A supply, per motor (now x3).
                public static final double FLYWHEEL_STATOR_A = 50;
                public static final double FLYWHEEL_SUPPLY_A = 25;
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
                // every RT shot, and while VIEW unjam is held -- "super slow" (team 2026-10-07).
                // [assumed] TODO tune on robot.
                public static final double FEEDER_SHOT_REVERSE_DUTY = 0.05;
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
                public static final double HOOD_kP = 150;  // V per hood rot: 5 deg error ~ 2 V. TODO tune
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
