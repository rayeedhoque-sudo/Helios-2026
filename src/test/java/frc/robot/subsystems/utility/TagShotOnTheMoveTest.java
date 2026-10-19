package frc.robot.subsystems.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.AutoAimConstants;

/** Pins the (switched-off) tag shoot-on-the-move math in TagAim. Table-agnostic. */
class TagShotOnTheMoveTest {

    /** Straight on a middle tag: the hub is dead ahead, one face depth past the tag. */
    @Test
    void hubSitsBehindATagSeenStraightOn() {
        Translation2d hub = TagAim.hubInRobot(0, 0, 2.0, Rotation2d.kZero, 0, 1.0);
        assertEquals(2.0 + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS, hub.getX(), 1e-9);
        assertEquals(0.0, hub.getY(), 1e-9);
    }

    /** Standing still: face the anchored hub, table key = distance + offset. */
    @Test
    void stillRobotAimsAtTheAnchor() {
        Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.kCCW_90deg);
        var shot = TagAim.movingShot(new Translation2d(4.0, 5.0), 0.1, pose, new ChassisSpeeds(), 0);
        assertEquals(Math.toDegrees(Math.atan2(4.0, 3.0)), shot.heading().getDegrees(), 1e-9);
        assertEquals(5.0 + 0.1, shot.hubDistanceMeters(), 1e-9);
        assertEquals(0.0, shot.headingRateRadPerSec(), 1e-9);
    }

    /** Strafing left past a hub dead ahead: aim right of it (lead), and the rate feedforward turns left. */
    @Test
    void strafingRobotLeadsAndFeedsForward() {
        Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
        var shot = TagAim.movingShot(new Translation2d(3.0, 0), 0, pose, new ChassisSpeeds(0, 0.8, 0), 0);
        double tof = AutoAimConstants.TIME_OF_FLIGHT_SEC_BY_DISTANCE.get(shot.hubDistanceMeters());
        assertTrue(shot.heading().getDegrees() < 0, "moving left must aim right of the hub");
        assertEquals(Math.toDegrees(Math.atan2(-0.8 * tof, 3.0)), shot.heading().getDegrees(), 1e-3);
        assertTrue(shot.headingRateRadPerSec() < 0, "hub drifts right as the robot moves left");
    }

    /** Standing still, the moving aim settles exactly where the tuned stand-still aim does (crosshair + trim). */
    @Test
    void stillMovingShotMatchesTheStandStillAim() {
        Pose2d pose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(30));
        double txnc = 6.0, tync = 10.0, dist = 2.5, crosshair = 0.6, hubLeft = -0.3556;
        Rotation2d inward = Rotation2d.fromDegrees(45);
        Translation2d anchor = pose.getTranslation()
            .plus(TagAim.hubInRobot(txnc, tync, dist, inward, hubLeft, 1.0).rotateBy(pose.getRotation()));
        var shot = TagAim.movingShot(anchor, 0, pose, new ChassisSpeeds(), TagAim.aimOffsetDeg(crosshair));
        Rotation2d still = TagAim.targetHeading(pose.getRotation(),
            TagAim.errorToHubDeg(txnc, tync, dist, inward, hubLeft, crosshair, AutoAimConstants.TAG_AIM_TX_SIGN)
                - AutoAimConstants.TAG_AIM_LEFT_TRIM_DEG);
        assertEquals(0.0, shot.heading().minus(still).getDegrees(), 1e-9);
    }
}
