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

            //HARDWARE CONSTANTS

                // Slider stall detection (NEO 2.0 on SPARK MAX, measured getOutputCurrent).
                // Free travel is a few amps; against the hard stop the 20 A smart limit pins it.
                public static final double SLIDER_STALL_AMPS = 15;          // TODO tune on robot
                public static final double SLIDER_STALL_DEBOUNCE_SEC = 0.15;
                public static final double SLIDER_GRACE_SEC = 0.25;         // ignore inrush at move start
                public static final double SLIDER_MOVE_TIMEOUT_SEC = 1.5;   // backstop; TODO measure real travel time
                public static final double SLIDER_TRAVEL_SPEED = 0.3;       // duty cycle, conservative

            //SPEED CONSTANTS
                // 50% duty roller test. Safe under the existing conservative limits: a jam is
                // capped at 30 A stator / 20 A supply regardless of duty cycle.
                public static final double INTAKE_SPEED = 0.5;
                public static final double OUTTAKE_SPEED = 0.5;

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