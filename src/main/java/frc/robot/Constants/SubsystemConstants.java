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

            // Fuse Limelight MegaTag2 pose estimates into odometry (CommandSwerveDrivetrain.periodic).
            // TODO flip to true once the Limelight's mounting/pipeline is verified on the robot.
            public static final boolean ENABLE_MEGATAG2_POSE = false;

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

            // Slider DISABLED (team decision 2026-07-12): moveSliderUntilStall() no-ops,
            // so intake/outtake/stow run rollers-only and never command the slider motor
            // (brake idle mode holds whatever position it's in). Re-enable: set true.
            public static final boolean SLIDER_ENABLED = false;

            //HARDWARE CONSTANTS

                // Slider stall detection (NEO 2.0 on SPARK MAX, measured getOutputCurrent).
                // Under closed-loop velocity control a blocked slider saturates the loop and
                // pins the 60 A smart limit regardless of commanded speed. The 20:1 gearbox
                // also halves what hopper drag draws vs the old ~10:1, so the gap between
                // drag current and the 60 A stall pin is wide.
                public static final double SLIDER_STALL_AMPS = 45;          // TODO tune on robot
                public static final double SLIDER_STALL_DEBOUNCE_SEC = 0.15;
                public static final double SLIDER_GRACE_SEC = 0.25;         // ignore inrush at move start
                public static final double SLIDER_MOVE_TIMEOUT_SEC = 3.0;   // backstop only -- stall is the real stop; TODO measure real travel time
                // Slider travel speed, MOTOR rpm (closed-loop velocity). Sized for the 20:1
                // gearbox (2026-07-10): 2000 motor rpm = 100 rpm at the output, ~25% faster
                // linear travel than the old 800 rpm on ~10:1, while the deeper reduction
                // doubles output torque. The velocity loop adds current (up to the 60 A
                // limit) to hold this speed through hopper drag.
                public static final double SLIDER_TRAVEL_MOTOR_RPM = 2000;  // TODO tune on robot
                // Velocity-loop gains (motor rpm units). FF = 1/5676 (NEO free speed at 12 V)
                // gets ~the right duty at speed; kP sized so a fully BLOCKED slider saturates
                // the output (error 800 rpm x kP + FF > 0.6 duty -> current pins the 40 A
                // limit, which is what makes stall detection fire). Don't shrink kP below
                // ~4.5e-4 without rechecking SLIDER_STALL_AMPS.
                public static double SLIDER_VELOCITY_kP = 6e-4;             // TODO tune on robot
                public static double SLIDER_VELOCITY_FF = 1.0 / 5676.0;

            //SPEED CONSTANTS
                // Roller duty cycles (driver-tuned 2026-07-10: 35% intake was too fast).
                // Worst-case steady supply draw = duty x stator limit (40 A, set in
                // IntakeSubsystem): intake 0.25 x 40 = 10 A, outtake 0.6 x 40 = 24 A,
                // both under the 25 A supply limit. Inrush softened by the duty-cycle ramp.
                public static final double INTAKE_SPEED = 0.25;
                public static final double OUTTAKE_SPEED = 0.6;

            //PID - Slider
                public static double INTAKE_SLIDER_kP = 0;
                public static double INTAKE_SLIDER_kI = 0;
                public static double INTAKE_SLIDER_kD = 0;
            
            //Slider Macros
                public static double INTAKE_SLIDER_INCHES = 0;
                public static double OUTTAKE_SLIDER_INCHES = 0;
                public static double STOW_SLIDER_INCHES = 0;
        }

        public static class HopperSubsystemConstants{
            public static final int HOPPER_ID_A = 20;
            public static final int HOPPER_ID_B = 21;
            public static final int KICKER_MOTOR_ID = 17;
            
            //SPEED CONSTANTS
                public static final double INDEXER_SPEED = 0.75;
                public static final double HOPPER_SPEED = 0.8;
                // Reverse duty for the unjam button (belts + kicker together, held).
                public static final double UNJAM_SPEED = 0.3;
                // Belt direction test: 5% duty ~= 285 rpm motor / ~24 rpm at rollers (12:1).
                // Bump to 0.10 if belt friction keeps it from moving.
                public static final double DIRECTION_TEST_SPEED = 0.05;
        }

        public static class ShooterSubsystemConstants{
            // Official 2026 REBUILT field AprilTags (WPILib 2026-rebuilt-welded.json; verified in
            // Hardware-Data-Sheet sec.3). The old single-ID constants were scrambled: 7 is a RED
            // TRENCH tag (was labeled RED_HUB) and 8 is a RED HUB tag (was labeled BLUE_HUB and
            // BLUE_TRENCH) -- vision would never have matched a real BLUE tag at all.
            public static final int[] APRILTAG_RED_HUB_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
            public static final int[] APRILTAG_BLUE_HUB_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
            public static final int[] APRILTAG_RED_TRENCH_IDS = {1, 6, 7, 12};
            public static final int[] APRILTAG_BLUE_TRENCH_IDS = {17, 22, 23, 28};
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
            //SHOT MODEL (distance -> velocity/angle; derived 2026-07-10 from the WPILib
            // 2026-rebuilt-welded.json tag layout + a no-drag ballistics closed form with a
            // drag fudge factor. Angle rule: FIXED at MAX_ANGLE (44.5 deg) -- the unconstrained
            // minimum-speed angle (~50 deg) exceeds the hood, so max hood always minimizes the
            // required speed AND guarantees the ball enters the hub descending.)
                // Ball release height above carpet. User-measured 2026-07-10: bottom of swerve
                // to the UNEXTENDED hood = 27 in. TODO re-measure to the actual ball exit point.
                public static final double SHOT_RELEASE_HEIGHT_METERS = Units.inchesToMeters(27);
                // Hub fuel-opening front-edge height above carpet: 72 in (official 2026 Game
                // Manual sec.5.4; hexagonal opening 41.7 in across, hub footprint 47x47 in).
                public static final double HUB_OPENING_HEIGHT_METERS = 1.829;
                // Field center (mid-field fuel pile) to hub-ring center, horizontal. From the
                // WPILib 2026-rebuilt-welded.json HUB tag centroids: RED (12.004, 4.035),
                // BLUE (4.537, 4.035), field center (8.2705, 4.0345) -> 3.734 m both alliances.
                public static final double MIDFIELD_TO_HUB_CENTER_METERS = 3.734;
                // Ball exit speed / flywheel surface speed (energy lost to backspin ~half).
                public static double SHOT_EFFICIENCY = 0.50;   // TODO tune on robot (0.45-0.55)
                // Extra exit speed demanded by air drag vs the no-drag closed form. Integrated
                // numerically for the OFFICIAL FUEL ball (5.91 in / 0.203-0.227 kg foam, manual
                // sec.5.10.1, Cd~0.5): x1.04 at the 3.73 m mid-field shot, x1.10 at 9 m.
                public static double SHOT_DRAG_FUDGE = 1.05;   // TODO tune on robot (1.03-1.10)
                // VelocityVoltage ceiling: ~12 V / 0.12625 kV = 95 motor rps sustainable.
                public static final double SHOT_MAX_MOTOR_RPS = 95.0;
            //SHOOTER CONSTRAINTS
                public static double MAX_ANGLE = 44.5;
                public static double MIN_ANGLE = 3.224;
                public static double SHOOTER_LOW_SPEED = 0;
                public static double SHOOTER_HIGH_SPEED = 25;
                public static double DISTANCE_SHORT = 0;
                public static double DISTANCE_FAR = 0;
                public static double SPEED_TOLERANCE = 0.3;
                public static double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);
                public static double ANGLE_TOLERANCE = 0.5;
            //PERIODIC CONSTANTS
                public static boolean desiredVelReached = false;
                public static boolean desiredAngleReached = false;
        }
}