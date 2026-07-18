package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class SubsystemConstants {

        public static class LightSensor{
            public static final int INDEXER_SENSOR_ID_A = 8;
            public static final int INDEXER_SENSOR_ID_B = 9;
        }

        public static class Vision{
            public static final String CAM_LIMELIGHT = "limelight-knight";
                public static final double LL_X_MULTIPLIER = 1;
                public static final double LL_Y__MULTIPLIER = 1;
                public static final double LL_Z_MULTIPLIER = 1;

            // AprilTag pose correction (CommandSwerveDrivetrain.updateVisionPose), enabled by
            // team request 2026-07-17: MegaTag2 corrects X/Y continuously (yaw stays
            // Pigeon-owned) and MegaTag1 auto-seeds the heading/pose ("auto re-zero") behind
            // the quality gates below. PREREQUISITE -- the Limelight's camera mount pose MUST
            // be set in its web UI first: botpose is systematically wrong without it, and the
            // gates below cannot catch a consistent mount-offset error. TODO verify on robot
            // before first match; MENU re-zero is the manual recovery either way.
            public static final boolean ENABLE_MEGATAG2_POSE = true;

            // Auto-seed quality gates. Disabled robot (pre-match, sitting still): adopt the
            // full MT1 tag pose whenever it disagrees with odometry. Enabled (in-match):
            // yaw-only re-seed, and only from a multi-tag solve while nearly stationary --
            // a moving single-tag MT1 yaw is exactly the noise the Pigeon exists to reject.
            public static final double VISION_SEED_MAX_TAG_DIST_METERS = 5.0;  // ignore solves with avg tag distance beyond this [assumed] TODO tune
            public static final double VISION_SEED_POS_TOL_METERS = 0.25;      // disabled-mode reseed when position is off by more than this
            public static final double VISION_SEED_YAW_TOL_DEG = 3.0;          // reseed when heading is off by more than this
            public static final int    VISION_SEED_ENABLED_MIN_TAGS = 2;       // in-match yaw reseed requires a multi-tag solve
            public static final double VISION_SEED_MAX_SPEED_MPS = 0.3;        // in-match reseed only while nearly stationary
            public static final double VISION_SEED_MAX_YAW_RATE_DPS = 10.0;    // and not rotating

            // PathPlanner pathfinding constraints (driveToPose). Conservative starts -- the old
            // zeros meant pathfinding could not move at all. TODO tune on robot.
            public static final double PP_MAX_VELOCITY = 2.5;                       // m/s (robot max ~4.93)
            public static final double PP_MAX_ACCELERATION = 2.5;                   // m/s^2
            public static final double PP_MAX_ANGULAR_VELOCITY = Math.PI;           // rad/s (180 deg/s)
            public static final double PP_MAX_ANGULAR_ACCELERATION = 2 * Math.PI;   // rad/s^2

        }

        public static class ClimbSubsystemConstants{
            public static final int CLIMBMOTOR_A = 0;
            public static final int CLIMBMOTOR_B = 0;

            //SPEED CONSTANTS
                public static final double CLIMB_MANUAL_SPEED = 0;
                public static final double CLIMB_MAX_VELOCITY = 0;
                public static final double CLIMB_MAX_ACCELERATION = 0;
            //PID - Climb
                public static double CLIMB_kP = 0;
                public static double CLIMB_kI = 0;
                public static double CLIMB_kD = 0;
            //FEEDFORWARD - Climb
                public static double CLIMB_kS = 0;
                public static double CLIMB_kG = 0;
                public static double CLIMB_kV = 0;
                public static double Climb_kA = 0;

            //HARDWARE CONSTANTS
                public static double CLIMB_WHEEL_RADIUS_INCHES = 0;
            
            //CLIMB HEIGHT CONSTRAINTS
                public static double MAX_INCHES = 0;
                public static double MIN_INCHES = 0;
        
        }

        public static class IntakeSubsystemConstants{
            public static final int INTAKE_MOTOR_ID = 18;
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

            //SPEED CONSTANTS
                // Roller duty cycles (driver-tuned 2026-07-10: 35% intake was too fast;
                // outtake halved 2026-07-14 per team request). Worst-case steady supply draw
                // = duty x stator limit (40 A, set in IntakeSubsystem): intake 0.25 x 40 =
                // 10 A, outtake 0.3 x 40 = 12 A, both well under the 25 A supply limit.
                // Inrush softened by the duty-cycle ramp.
                public static final double INTAKE_SPEED = 0.25;
                public static final double OUTTAKE_SPEED = 0.3;

        }

        public static class HopperSubsystemConstants{
            public static final int HOPPER_ID_A = 20;
            public static final int HOPPER_ID_B = 21;
            // Kicker Victor SPX at CAN 17 -- VERIFIED LIVE on the bus 2026-07-16 via the
            // Phoenix diagnostics server (Victor SPX, fw 22.1, "Running Application").
            // Earlier "kicker not working" was NOT the ID: the kicker only fires when the
            // flywheels are at speed (ShooterSubsystem.isReadyToShoot), and the shooter was disabled.
            public static final int KICKER_MOTOR_ID = 17;
            
            //SPEED CONSTANTS
                public static final double INDEXER_SPEED = 0.75;
                // Belt duty. Conservative start (old code ran 0.8, never verified on robot);
                // direction test 2026-07-14 confirmed both motors agree, positive = tested
                // direction. TODO raise toward 0.8 once feed direction + throughput verified.
                public static final double HOPPER_SPEED = 0.5;
                // Reverse duty for the unjam button (belts + kicker together, held).
                public static final double UNJAM_SPEED = 0.3;
                // Slow duty for the (unbound) kicker-only test (bypasses the at-speed gate).
                public static final double KICKER_TEST_SPEED = 0.3;
                // Kicker duty while INTAKING (LT): the flywheels are STOPPED during intake,
                // so fuel pressed against them can stall the kicker CIM -- and the Victor
                // SPX has no current sensing, so this deliberately LOW duty is the only
                // thing keeping a momentary stall breaker-survivable (CIM stall at 12 V is
                // ~131 A; at 0.3 duty a stall draws roughly a third of that -- brief events
                // ride the breaker curve). [assumed] TODO tune on robot; keep it low.
                public static double INTAKE_KICKER_SPEED = 0.3;
        }

        public static class ShooterSubsystemConstants{
            // AprilTag sets moved to FieldConstants (FEED_TAGS / SCORE_TAGS_RED / SCORE_TAGS_BLUE,
            // team partition 2026-07-16) -- one source so the shooter and drivetrain can't drift.
            public static final int SHOOTER_ANGLE_ID = 19;
            public static final int SHOOTER_ID_A = 13;
            public static final int SHOOTER_ID_B = 14;
            public static final int SHOOTER_ID_C = 15;
            public static final int SHOOTER_ID_D = 16;
            //PID - Angle
                public static double SHOOTER_ANGLE_kP = 0.275;
                public static double SHOOTER_ANGLE_kI = 0.0;
                public static double SHOOTER_ANGLE_kD = 0.0;
            //PID - Speed
                public static double SHOOTER_SPEED_kP = 0.4;
                public static double SHOOTER_SPEED_kI = 0;
                public static double SHOOTER_SPEED_kD = 0.01;
            //FEEDFORWARD - Speed
                public static double SHOOTER_SPEED_kS = 0;
                public static double SHOOTER_SPEED_kV = 0.12625;
                public static double SHOOTER_SPEED_kA = 0;
            //HARDWARE CONSTANTS
                public static final double FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION = 1.5;
                    // Flywheel rotations per motor rotation.
                    // Gear ratio is 3:2 (3 flywheel rotations for every 2 Kraken rotations).
                public static final double NEO550_ROTATIONS_PER_HOOD_ROTATION = 187.5;
                    // Motor rotations required for 1 degree of hood movement.
                    // Gear ratio is 187.5 : 1( 0.00533333333 NEO550 rotations = 1 hood rotation).
                public static final double NEO550_ROTATIONS_PER_HOOD_DEGREE = NEO550_ROTATIONS_PER_HOOD_ROTATION * 0.0317428;
                    // Motor rotations required for 1 degree of hood movement.
                public static final double SHOOTER_ANGLE_OFFSET = 63;
            //SHOT MODEL (distance -> velocity; shot-math spec 2026-07-16, numerically integrated
            // quadratic-drag ballistics for the OFFICIAL FUEL ball -- 5.91 in / 0.203-0.227 kg
            // foam, Cd~0.5, manual sec.5.10.1 -- validated against the no-drag closed form.
            // Angle rule: FIXED at MAX_ANGLE (44.5 deg). Spec 2.4 swept 25-44.5 deg over the whole
            // legal envelope: 44.5 needs the LOWEST speed AND the steepest entry everywhere, so
            // scoring never varies the hood; its lower range exists only for stowing.)
                // Ball release height above carpet. User-measured 2026-07-10: bottom of swerve
                // to the UNEXTENDED hood = 27 in. TODO re-measure to the actual ball exit point.
                public static final double SHOT_RELEASE_HEIGHT_METERS = Units.inchesToMeters(27);
                // Ball exit speed / flywheel surface speed (energy lost to backspin ~half).
                // Biggest single unknown -- every model velocity scales with it; tune FIRST,
                // drag slopes LAST. TODO tune on robot (0.45-0.55).
                public static double SHOT_EFFICIENCY = 0.50;
                // Distance-dependent drag multipliers replacing the old fixed 1.05 fudge (exact
                // only at ~4.7 m; at 3 m it shot 0.25 m long = a miss, at 6.2 m 0.21 m short).
                // Linear fits to the integrator, residual < 0.03% (score) / < 0.2% (feed):
                //   score: mult(d) = 1.0002 + 0.01063*d, fitted over 2.4-6.2 m (spec 2.2)
                //   feed:  mult(d) = 0.9955 + 0.0113*d, fitted over 4-9 m (spec 3.2)
                // Endpoints tunable on robot; the spec 2.2/3.2 tables are the tuning reference.
                public static double SCORE_DRAG_MULT_BASE = 1.000;
                public static double SCORE_DRAG_MULT_PER_METER = 0.0106;   // 1/m
                public static double FEED_DRAG_MULT_BASE = 0.9955;
                public static double FEED_DRAG_MULT_PER_METER = 0.0113;    // 1/m
                // SCORE envelope to the own hub center (m). Below 3.0 the ball can't fit through
                // the opening within the speed/angle tolerance budget (physical floor 2.63 m;
                // dead zone 1.0-3.0 m is real -- drivers must back off ~2 m from the hub face).
                // Above 6.2 the robot cannot legally be in its zone (G407 geometric limit 6.14 m).
                // Outside the envelope the model REFUSES (velocity 0 -> no belts, no kicker).
                public static double MIN_SCORE_DISTANCE_METERS = 3.0;   // [derived] TODO tune on robot
                public static final double MAX_SCORE_DISTANCE_METERS = 6.2;
                // FEED distance clamp (m): outside 4-9 command the clamped endpoint instead of
                // refusing -- a slightly-short lob still lands in friendly territory. [assumed]
                public static double MIN_FEED_DISTANCE_METERS = 4.0;
                public static double MAX_FEED_DISTANCE_METERS = 9.0;
                // RB blind feed: flywheel SURFACE speed for a ~6.4 m lob (typical blind-feed
                // positions span 5.5-7 m -> +-0.8 m landing scatter, fine for a bulk feed).
                // Raised 16 -> 22 m/s 2026-07-17 (practice-match: the shot "barely left the robot").
                // 22 m/s = ~46 motor rps = 48% of the 95 rps ceiling (5.8 V FF of 12) -- ample motor
                // headroom, and paired with the flywheel STATOR raise (ShooterSubsystem) so the wheel
                // can actually hold it under ball load. STARTING POINT -- retest and dial in: if it now
                // overshoots the feed zone, drop it; if still weak, nudge up AND check flywheel stator
                // amps in PowerTelemetry (pegged at 80 = still torque-limited, not speed-limited).
                // [assumed] TODO tune vs actual feed positions on robot.
                public static double RB_FEED_SURFACE_SPEED = 22.0;   // m/s surface
                // RB blind feed hood angle (deg). Set to 38 by team request 2026-07-17 (was
                // MAX_ANGLE 44.5). The 16 m/s speed above was sized for the 44.5-deg lob --
                // TODO retune RB_FEED_SURFACE_SPEED for the new landing distance on robot.
                public static double RB_FEED_ANGLE = 38.0;           // deg
                // Time-of-flight linear fits (s) for moving-shot compensation, good to ~+-0.03 s
                // against the integrator (spec 5): score 0.185 + 0.138*d, feed 0.618 + 0.093*d.
                public static double SCORE_TOF_BASE_SEC = 0.185;
                public static double SCORE_TOF_SEC_PER_METER = 0.138;
                public static double FEED_TOF_BASE_SEC = 0.618;
                public static double FEED_TOF_SEC_PER_METER = 0.093;
                // Moving-shot radial compensation (spec 5): d_eff = d + gain * v_radial * ToF,
                // two fixed-point iterations, clamped. Gain 0 disables while tuning everything
                // else. Beyond +-1.5 m the robot moves >~1.9 m/s and the shot shouldn't be
                // trusted; the clamp also keeps d_eff from jumping the envelope gate erratically.
                public static double MOVING_COMP_GAIN = 1.0;             // [assumed, tunable; 0 disables]
                public static double MOVING_COMP_MAX_METERS = 1.5;
                // RT tag-flicker ride-through (auto-aim, 2026-07-17): after the last real tag
                // classification, the firing solution stays live this long on the drivetrain's
                // fused pose before RT refuses. Initial acquisition still needs a real tag.
                public static double TARGET_HOLD_SEC = 0.5;              // s [assumed] TODO tune on robot
                // VelocityVoltage ceiling: ~12 V / 0.12625 kV = 95 motor rps sustainable.
                public static final double SHOT_MAX_MOTOR_RPS = 95.0;
            //SHOOTER CONSTRAINTS
                public static double MAX_ANGLE = 44.5;
                public static double MIN_ANGLE = 3.224;
                // Surface-speed gate for the kicker, m/s. TIGHTENED 0.3 -> 0.2 (spec 6): at the
                // 3.0 m minimum, 0.3 m/s maps to 0.30 m of along-track error -- more than the
                // 0.226 m half-window through the opening; 0.2 closes the budget exactly.
                public static double SPEED_TOLERANCE = 0.2;
                public static double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);
                // Hood-lower interlock (team request 2026-07-17): the hood may only be driven DOWN
                // when the flywheel SURFACE speed is at/below this -- above it the hood holds its
                // current angle and waits for the flywheels to coast down (raising is never gated).
                // Low value = "nearly stopped". [assumed] TODO tune on robot: raise if the hood
                // should start dropping sooner, lower it toward 0 to wait for a fuller stop.
                public static double HOOD_LOWER_MAX_SURFACE_SPEED = 2.0;   // m/s surface
                // Hood gate, deg: worst contribution 0.078 m at 3 m, shrinking with distance.
                public static double ANGLE_TOLERANCE = 0.5;
        }
}