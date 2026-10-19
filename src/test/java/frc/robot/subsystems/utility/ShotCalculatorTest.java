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

/** Pins the RB shoot-on-the-move math (ShotCalculator's pure helpers). Table-agnostic. */
class ShotCalculatorTest {

    private static final Translation2d HUB = FieldConstants.HUB_CENTER_BLUE;

    /** Standing still, the virtual target IS the hub. */
    @Test
    void stillRobotAimsAtTheHub() {
        Translation2d target = ShotCalculator.virtualTarget(HUB, new Translation2d(2.0, 4.0), Translation2d.kZero);
        assertEquals(HUB.getX(), target.getX(), 1e-9);
        assertEquals(HUB.getY(), target.getY(), 1e-9);
    }

    /** Moving, the aim point shifts AGAINST the motion by velocity x flight time at that point. */
    @Test
    void movingRobotLeadsOppositeItsVelocity() {
        Translation2d launcher = new Translation2d(2.0, 4.0);
        Translation2d velocity = new Translation2d(0.0, 0.8); // strafing left (+Y)
        Translation2d target = ShotCalculator.virtualTarget(HUB, launcher, velocity);
        assertTrue(target.getY() < HUB.getY(), "moving left must aim right of the hub");
        double tof = AutoAimConstants.TIME_OF_FLIGHT_SEC_BY_DISTANCE.get(target.getDistance(launcher));
        assertEquals(HUB.getY() - 0.8 * tof, target.getY(), 1e-3, "converged: lead = v x tof(d)");
        assertEquals(HUB.getX(), target.getX(), 1e-9, "no X motion, no X lead");
    }

    /** Robot-relative speeds rotate into the field frame by the heading. */
    @Test
    void fieldVelocityRotatesByHeading() {
        // Facing +Y (90 deg) and driving robot-forward 1 m/s = field +Y.
        Translation2d v = ShotCalculator.fieldVelocity(new ChassisSpeeds(1.0, 0, 0), Rotation2d.kCCW_90deg);
        assertEquals(0.0, v.getX(), 1e-9);
        assertEquals(1.0, v.getY(), 1e-9);
    }

    /** Hub dead ahead, strafing left: the hub drifts right, so the heading must turn CW (-). */
    @Test
    void headingRateSign() {
        double rate = ShotCalculator.headingRate(new Translation2d(2.0, 0.0), new Translation2d(0.0, 1.0));
        assertEquals(-0.5, rate, 1e-9, "v_perp / d = 1 / 2 rad/s, clockwise");
        assertEquals(0.0, ShotCalculator.headingRate(new Translation2d(2.0, 0.0), new Translation2d(1.0, 0.0)), 1e-9,
            "driving straight at the hub needs no turn");
    }

    /** Full solution: still robot on the hub's centreline faces it (+X), at the right distance. */
    @Test
    void calculateFacesTheHubFromStraightBehind() {
        Pose2d pose = new Pose2d(HUB.getX() - 3.0, HUB.getY(), Rotation2d.k180deg); // heading irrelevant at rest
        ShotCalculator.Shot shot = ShotCalculator.calculate(pose, new ChassisSpeeds(), HUB);
        assertEquals(0.0, shot.heading().getDegrees(), 1e-6);
        // Launcher offset is (0,0) today; with a real offset this distance is launcher-to-hub.
        assertEquals(HUB.getDistance(pose.getTranslation().plus(
            AutoAimConstants.ROBOT_TO_LAUNCHER_METERS.rotateBy(pose.getRotation()))), shot.distanceMeters(), 1e-9);
        assertEquals(AutoAimConstants.SHOT_RPM_BY_DISTANCE.get(shot.distanceMeters()), shot.flywheelRpm(), 1e-9);
        assertEquals(0.0, shot.headingRateRadPerSec(), 1e-9);
    }
}
