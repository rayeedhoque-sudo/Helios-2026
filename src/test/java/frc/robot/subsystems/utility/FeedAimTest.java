package frc.robot.subsystems.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.AutoAimConstants;
import frc.robot.subsystems.utility.LimelightHelpers.RawFiducial;

/** Pins the LB feed shot (FeedAim, remade 2026-10-17) against the team's field points. */
class FeedAimTest {

    private static final double SIGN = AutoAimConstants.TAG_AIM_TX_SIGN;
    private static final double L = FieldConstants.FIELD_LENGTH_METERS;
    private static final double W = FieldConstants.FIELD_WIDTH_METERS;

    /** A team point (docs/feed-data/p*.json): camera spot on the RED alliance (red frame) + the rpm that worked. */
    record Point(String name, double x, double y, double rpm) {
        Translation2d at() { return new Translation2d(x, y); }
    }

    private static final Point[] WORKED = {
        new Point("p1", 8.57, 1.70, 1600), new Point("p2", 9.22, 2.43, 1600), new Point("p3", 9.13, 3.16, 1600),
        new Point("p4", 8.70, 3.44, 1600), new Point("p5", 9.40, 3.44, 1600), new Point("p7", 7.75, 2.69, 1800),
        new Point("p8", 7.55, 2.45, 1700), new Point("p9", 7.94, 1.67, 1700), new Point("p10", 3.04, 1.09, 2000),
        new Point("p11", 2.53, 2.80, 2000),
    };
    private static final Translation2d P6_NO_SHOT = new Translation2d(10.21, 3.80); // behind the red hub

    private static double rpmAt(Translation2d redFrame) {
        return AutoAimConstants.FEED_RPM_BY_WALL_DIST.get(L - redFrame.getX());
    }

    /** Every working point gets an rpm inside what worked there (1700 and 1800 both worked at ~8.8 m). */
    @Test
    void everyWorkingPointGetsItsRpm() {
        for (Point p : WORKED) {
            assertEquals(p.rpm(), rpmAt(p.at()), 100, p.name());
        }
        for (String name : new String[] {"p1", "p2", "p3", "p4", "p5"}) {
            Point p = java.util.Arrays.stream(WORKED).filter(q -> q.name().equals(name)).findFirst().orElseThrow();
            assertEquals(1600, rpmAt(p.at()), 1e-9, name + " is in the 1600 cluster");
        }
    }

    /** p6 ("here or closer can't work") is blocked; no working point is. */
    @Test
    void onlyTheNoShotSpotIsBlocked() {
        assertTrue(FeedAim.blocked(false, P6_NO_SHOT));
        for (Point p : WORKED) {
            assertFalse(FeedAim.blocked(false, p.at()), p.name());
        }
    }

    /** The low-y data mirrors to the high-y side: same rpm, same block. */
    @Test
    void leftAndRightSidesMatch() {
        for (Point p : WORKED) {
            Translation2d mirrored = new Translation2d(p.x(), W - p.y());
            assertEquals(rpmAt(p.at()), rpmAt(mirrored), 1e-9, p.name());
            assertFalse(FeedAim.blocked(false, mirrored), p.name());
        }
        assertTrue(FeedAim.blocked(false, new Translation2d(P6_NO_SHOT.getX(), W - P6_NO_SHOT.getY())));
    }

    /** Blue is red turned 180 deg about the field centre: same red-frame spot, feed heading 180. */
    @Test
    void blueMirrorsRed() {
        for (Point p : WORKED) {
            Translation2d blueField = new Translation2d(L - p.x(), W - p.y());
            Translation2d back = FeedAim.redFrame(blueField, Alliance.Blue);
            assertEquals(p.x(), back.getX(), 1e-9, p.name());
            assertEquals(p.y(), back.getY(), 1e-9, p.name());
        }
        assertEquals(p(3.0, 4.0), FeedAim.redFrame(p(3.0, 4.0), Alliance.Red));
        assertEquals(0.0, FeedAim.wallHeading(Alliance.Red).getDegrees(), 1e-9);
        assertEquals(180.0, Math.abs(FeedAim.wallHeading(Alliance.Blue).getDegrees()), 1e-9);
    }

    /** Their hub blocks the same way from inside their zone; beside it or far enough back is clear. */
    @Test
    void theirHubBlocksFromInsideTheirZone() {
        assertTrue(FeedAim.blocked(false, p(3.0, 4.0)), "1.63 m in front of their hub");
        assertFalse(FeedAim.blocked(false, p(2.0, 4.0)), "2.63 m back clears it");
        assertFalse(FeedAim.blocked(false, p(3.0, 2.8)), "beside it");
        assertFalse(FeedAim.blocked(false, p(13.0, 4.0)), "our hub behind us (in our zone) never blocks");
    }

    /** Once blocked, it takes HYST more to clear: no flicker at the edge. */
    @Test
    void blockHasHysteresis() {
        double hub = FieldConstants.HUB_CENTER_RED.getX();
        Translation2d edge = p(hub - AutoAimConstants.FEED_HUB_BLOCK_DEPTH_METERS - 0.05, 4.0);
        assertFalse(FeedAim.blocked(false, edge));
        assertTrue(FeedAim.blocked(true, edge));
    }

    /** Standing still = the plain wall distance; driving at our wall = that much closer per flight second. */
    @Test
    void robotMotionShiftsTheRpmKey() {
        Translation2d p1 = p(8.57, 1.70);
        double still = L - p1.getX();
        assertEquals(still, FeedAim.leadWallDistance(p1, Translation2d.kZero), 1e-9);
        double tof = AutoAimConstants.FEED_TOF_SEC_BY_WALL_DIST.get(still);
        assertEquals(still - 0.24 * tof, FeedAim.leadWallDistance(p1, p(0.24, 0)), 1e-9, "toward our wall: shorter");
        assertEquals(still + 0.24 * tof, FeedAim.leadWallDistance(p1, p(-0.24, 0)), 1e-9, "away: longer");
        assertEquals(still, FeedAim.leadWallDistance(p1, p(0, 0.24)), 1e-9, "sideways: same depth");
        assertTrue(AutoAimConstants.FEED_RPM_BY_WALL_DIST.get(FeedAim.leadWallDistance(p(7.55, 2.45), p(-0.24, 0)))
            > AutoAimConstants.FEED_RPM_BY_WALL_DIST.get(L - 7.55), "backing away needs more rpm");
    }

    /** Inside the window the robot is left where it points; outside it turns back only to the edge. */
    @Test
    void headingIsHeldInsideTheWindow() {
        double w = AutoAimConstants.FEED_HEADING_WINDOW_DEG;
        Rotation2d red = FeedAim.wallHeading(Alliance.Red);
        assertEquals(-6.0, FeedAim.holdInWindow(Rotation2d.fromDegrees(-6), red, w).getDegrees(), 1e-9);
        assertEquals(w, FeedAim.holdInWindow(Rotation2d.fromDegrees(40), red, w).getDegrees(), 1e-9);
        assertEquals(-w, FeedAim.holdInWindow(Rotation2d.fromDegrees(-90), red, w).getDegrees(), 1e-9);
        // Blue feeds at 180: the window wraps through +-180 the short way.
        Rotation2d blue = FeedAim.wallHeading(Alliance.Blue);
        assertEquals(175.0, FeedAim.holdInWindow(Rotation2d.fromDegrees(175), blue, w).getDegrees(), 1e-9);
        assertEquals(-175.0, FeedAim.holdInWindow(Rotation2d.fromDegrees(-175), blue, w).getDegrees(), 1e-9);
        assertEquals(180.0 - w, FeedAim.holdInWindow(Rotation2d.fromDegrees(120), blue, w).getDegrees(), 1e-9);
        // A robot held at the edge still feeds.
        assertTrue(AutoAimConstants.FEED_HEADING_TOLERANCE_DEG > w);
    }

    /** A sighting at a known pose: field spot -> camera numbers -> robotField gives the spot back. */
    private static Translation2d seenFrom(Translation2d robot, Rotation2d trueHeading, Rotation2d gyroHeading, int tagId) {
        Translation2d tag = FeedAim.tagPose(tagId).orElseThrow().getTranslation();
        Translation2d tagInRobot = tag.minus(robot).rotateBy(trueHeading.unaryMinus());
        double txnc = -tagInRobot.getAngle().getDegrees() / SIGN; // tag to the right = +txnc
        return FeedAim.robotField(txnc, 0, tagInRobot.getNorm(), gyroHeading, tag, SIGN);
    }

    /** Any tag places the robot, red side and far side. */
    @Test
    void robotFieldRoundTripsThroughTheCamera() {
        for (int id : new int[] {4, 6, 27, 28}) {
            Translation2d robot = id == 27 || id == 28 ? p(2.53, 2.80) : p(9.13, 3.16);
            Rotation2d heading = Rotation2d.fromDegrees(-3);
            Translation2d seen = seenFrom(robot, heading, heading, id);
            assertEquals(robot.getX(), seen.getX(), 1e-9, "tag " + id);
            assertEquals(robot.getY(), seen.getY(), 1e-9, "tag " + id);
            assertTrue(FeedAim.plausible(seen, FeedAim.tagPose(id).orElseThrow()), "tag " + id);
        }
    }

    /** MENU pressed the wrong way (Pigeon 180 deg off): the placement lands behind the tag and is thrown away. */
    @Test
    void aFlippedHeadingNeverPlaces() {
        Rotation2d truth = Rotation2d.fromDegrees(-1);
        for (int id : new int[] {3, 4, 6}) {
            Translation2d seen = seenFrom(p(9.13, 3.16), truth, truth.plus(Rotation2d.k180deg), id);
            assertFalse(FeedAim.plausible(seen, FeedAim.tagPose(id).orElseThrow()), "tag " + id);
        }
        Pose2d tag4 = FeedAim.tagPose(4).orElseThrow();
        assertFalse(FeedAim.plausible(p(-0.5, 4.0), tag4), "off the field");
    }

    /** The biggest tag on the field map wins; ids off the map or with no range are skipped. */
    @Test
    void picksTheBiggestFieldTag() {
        RawFiducial far6 = new RawFiducial(6, 12.0, 0, 0.2, 5.0, 5.0, 0.1);
        RawFiducial near3 = new RawFiducial(3, 20.0, 0, 0.9, 2.0, 2.0, 0.1);
        RawFiducial offMap = new RawFiducial(40, 0, 0, 5.0, 1.0, 1.0, 0);
        RawFiducial noRange = new RawFiducial(4, 0, 0, 3.0, 0, 0, 0);
        assertEquals(3, FeedAim.pickTag(new RawFiducial[] {far6, near3, offMap, noRange}).id);
        assertNull(FeedAim.pickTag(new RawFiducial[] {offMap, noRange}));
        assertNull(FeedAim.pickTag(null));
    }

    private static Translation2d p(double x, double y) {
        return new Translation2d(x, y);
    }
}
