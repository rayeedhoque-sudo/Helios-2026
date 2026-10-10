package frc.robot.Constants;


import com.pathplanner.lib.util.FlippingUtil;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// 2026 REBUILT field geometry used by the drivetrain (field bounds + auto start spots).
// Every number here is dumped/derived from the official WPILib 2026-rebuilt-welded.json tag
// layout (apriltag-java-2026.2.1.jar) via the shot-math derivation of 2026-07-16 -- cited per
// constant. Tunables (team strategy choices / on-robot unknowns) are non-final per team convention.
public class FieldConstants {

    //FIELD DIMENSIONS (WPILib 2026-rebuilt-welded.json; blue origin at x = 0)
        public static final double FIELD_LENGTH_METERS = 16.541;
        public static final double FIELD_WIDTH_METERS = 8.069;

    //AUTO START SPOT (blue-origin), picked per match on the driver-companion Field tab.
        // The companion publishes one of these names to /Companion/StartSpot; at auto start
        // the robot resets odometry (X, Y AND heading) to that spot. No pick = NO reset.
        // Every spot: robot FACING THE NEUTRAL ZONE (heading 0 deg on blue -- team 2026-09-30),
        // bumpers flush against the named field element.
        // TODO MEASURE ON THE FIELD: all coordinates below are [inferred], not measured.
        //   Trench y = the trench tag y (2026-rebuilt-welded.json tags 17/22/23/28); hub face
        //   x = tags 25/26 (4.022 m). Bump extents were read off the companion's field image
        //   (assets/2026-field.png, calibrated by 2026-rebuilt.json): x 3.99..5.19 m,
        //   left bump y 4.67..6.79 m, right bump y 1.31..3.43 m.
        // ROBOT_HALF_LENGTH: half of the 0.9779 m with-bumpers length (PathPlanner config,
        // team confirmed 2026-09-30 it includes bumpers and is front-to-back).
        public static final double ROBOT_HALF_LENGTH_METERS = 0.9779 / 2.0;
        public static final double HUB_X_METERS = 4.626;            // TODO [inferred] trench/hub centerline x
        public static final double HUB_ALLIANCE_FACE_X_METERS = 4.022; // TODO [inferred] tags 25/26
        public static final double BUMP_ALLIANCE_EDGE_X_METERS = 3.99;  // TODO [inferred] field image

        /** Start spots. Left/right are from OUR driver station's view (blue: left = +Y). */
        public enum StartSpot {
            LEFT_TRENCH(HUB_X_METERS, 7.425),   // TODO [inferred] under the trench, tag 22/23 y
            LEFT_BUMP(BUMP_ALLIANCE_EDGE_X_METERS - ROBOT_HALF_LENGTH_METERS, 5.73), // TODO [inferred]
            HUB(HUB_ALLIANCE_FACE_X_METERS - ROBOT_HALF_LENGTH_METERS, 4.035),       // TODO [inferred]
            RIGHT_BUMP(BUMP_ALLIANCE_EDGE_X_METERS - ROBOT_HALF_LENGTH_METERS, 2.37), // TODO [inferred]
            RIGHT_TRENCH(HUB_X_METERS, 0.644);  // TODO [inferred] under the trench, tag 17/28 y

            /** Blue-alliance pose, facing the neutral zone (+X). */
            public final Pose2d bluePose;

            StartSpot(double xMeters, double yMeters) {
                this.bluePose = new Pose2d(xMeters, yMeters, Rotation2d.kZero);
            }
        }

        /**
         * Pure lookup: the pose for a start-spot name on the given alliance, or empty for a
         * blank/unknown name. Red is the blue pose rotated by PathPlanner's FlippingUtil (same
         * flip AutoBuilder uses for paths), which keeps "left" = the red driver's left.
         */
        public static Optional<Pose2d> startPose(String spotName, boolean isRed) {
            for (StartSpot spot : StartSpot.values()) {
                if (spot.name().equals(spotName)) {
                    return Optional.of(isRed ? FlippingUtil.flipFieldPose(spot.bluePose) : spot.bluePose);
                }
            }
            return Optional.empty();
        }

    //RB SHOOT-ON-THE-MOVE GEOMETRY (hub centres unchanged from commit ccd5075)
        // Geometric hub centers = midpoints of the 4 tag face planes. NOT the mean of the 8 tag
        // positions -- the tags sit pinwheel-asymmetric on the faces (shot-math spec 1.1).
        public static final Translation2d HUB_CENTER_BLUE = new Translation2d(4.6255, 4.0346);
        public static final Translation2d HUB_CENTER_RED = new Translation2d(11.9155, 4.0346);

        // "Aimed At Target" tolerance (shot-math spec 6) -- RB only feeds inside it. TODO tune
        // on robot: widen if RB rarely fires while moving.
        public static final double HEADING_TOLERANCE_DEG = 2.0;

        // Own hub center.
        public static Translation2d ownHubCenter(Alliance alliance) {
            return alliance == Alliance.Red ? HUB_CENTER_RED : HUB_CENTER_BLUE;
        }
}
