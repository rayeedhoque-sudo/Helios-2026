package frc.robot.subsystems.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.Set;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.AutoAimConstants;
import frc.robot.subsystems.utility.LimelightHelpers.RawFiducial;

/** Pins the RB tag aim: which tag it locks onto, and which way it turns. */
class TagAimTest {

    private static RawFiducial tag(int id, double ta) {
        return new RawFiducial(id, 0, 0, ta, 0, 0, 0);
    }

    /** Only the middle front tags (10, 26) count -- the biggest one wins. */
    @Test
    void picksTheBiggestFrontTag() {
        RawFiducial[] seen = { tag(25, 9.0), tag(10, 1.0), tag(26, 2.0), tag(27, 5.0) };
        assertEquals(26, TagAim.pickTag(seen, Set.of()).id);
        assertNull(TagAim.pickTag(new RawFiducial[] { tag(25, 9.0), tag(2, 3.0) }, Set.of()));
        assertNull(TagAim.pickTag(null, Set.of()));
    }

    /**
     * Robot at (2.0, 5.0) facing tag 26 from its LEFT (heading -25.5 deg). The hub centre is
     * 0.6035 m behind the tag along +X, which from here is ~5.3 deg LEFT of the tag -- the
     * true bearing difference to HUB_CENTER_BLUE. Straight on, the hub is right behind the tag.
     */
    @Test
    void fromTheSideTheAimMovesPastTheTagToTheHubCentre() {
        var robot = new edu.wpi.first.math.geometry.Translation2d(2.0, 5.0);
        var tag26 = new edu.wpi.first.math.geometry.Translation2d(4.022, 4.035);
        var hub = frc.robot.Constants.FieldConstants.HUB_CENTER_BLUE;
        Rotation2d heading = tag26.minus(robot).getAngle();          // facing the tag
        double range = tag26.getDistance(robot);
        double hubRight = TagAim.errorToHubDeg(0, 0, range,
            TagAim.inwardHeading(26).minus(heading), 0, 0, 1.0);
        double truth = -hub.minus(robot).getAngle().minus(heading).getDegrees();
        assertEquals(truth, hubRight, 0.05);
        assertTrue(hubRight < -4, "hub centre is LEFT of the tag from the left side");

        // Straight on: no offset.
        assertEquals(0.0, TagAim.errorToHubDeg(0, 0, 3.0, Rotation2d.kZero, 0, 0, 1.0), 1e-9);
        // No range from the camera: plain tag aim, crosshair-relative.
        assertEquals(3.0, TagAim.errorToHubDeg(5.0, 0, 0, Rotation2d.kCCW_90deg, 0, 2.0, 1.0), 1e-9);
    }

    /** 26's face points into the hub along +X (0 deg), 10's along -X. */
    @Test
    void inwardHeadingPerTag() {
        assertEquals(0.0, TagAim.inwardHeading(26).getDegrees(), 1e-9);
        assertEquals(180.0, Math.abs(TagAim.inwardHeading(10).getDegrees()), 1e-9);
    }

    /** Tag 5 deg right of the crosshair: turn clockwise, to 5 deg less than the heading at the frame. */
    @Test
    void tagRightTurnsClockwise() {
        Rotation2d target = TagAim.targetHeading(Rotation2d.fromDegrees(30), 5.0);
        assertEquals(25.0, target.getDegrees(), 1e-9);
        assertEquals(0.0, TagAim.targetHeading(Rotation2d.fromDegrees(30), 0).minus(
            Rotation2d.fromDegrees(30)).getDegrees(), 1e-9, "centred = stay put");
    }

    /** No-tag fallback: face our own hub by odometry; unknown alliance = hold. */
    @Test
    void odometryHeadingFacesTheHub() {
        Translation2d westOfBlue = FieldConstants.HUB_CENTER_BLUE.minus(new Translation2d(3, 0));
        assertEquals(0.0, TagAim.odometryHeading(westOfBlue, Optional.of(Alliance.Blue))
            .orElseThrow().getDegrees(), 1e-9);
        Translation2d northOfRed = FieldConstants.HUB_CENTER_RED.plus(new Translation2d(0, 2));
        assertEquals(-90.0, TagAim.odometryHeading(northOfRed, Optional.of(Alliance.Red))
            .orElseThrow().getDegrees(), 1e-9);
        assertEquals(Optional.empty(), TagAim.odometryHeading(westOfBlue, Optional.empty()));
    }

    /** The left trim (subtracted from the error) settles the robot exactly that much counter-clockwise. */
    @Test
    void leftTrimTurnsCounterClockwise() {
        double trim = AutoAimConstants.TAG_AIM_LEFT_TRIM_DEG;
        Rotation2d trimmed = TagAim.targetHeading(Rotation2d.fromDegrees(30), 5.0 - trim);
        assertEquals(trim, trimmed.minus(TagAim.targetHeading(Rotation2d.fromDegrees(30), 5.0)).getDegrees(), 1e-9);
    }

    /** Straight on, the table key is the tag range + 0.6035 -- how the rows were keyed. */
    @Test
    void hubDistanceStraightOnIsRangePlusDepth() {
        assertEquals(1.0456 + 0.6035, TagAim.hubDistanceMeters(0, 1.0456, Rotation2d.kZero, 0, 1.0), 1e-9);
        // From the side the hub centre is still 0.6035 m behind the tag, just not straight away.
        double side = TagAim.hubDistanceMeters(0, 2.0, Rotation2d.fromDegrees(-40), 0, 1.0);
        assertTrue(side > 2.0 && side < 2.0 + 0.6035, "side " + side);
    }

    /** The table returns the scored data at each row and interpolates between rows. */
    @Test
    void shotTableHitsTheData() {
        assertEquals(1300, AutoAimConstants.TAG_SHOT_RPM_BY_HUB_DIST.get(1.778), 1e-9);
        assertEquals(26.0, AutoAimConstants.TAG_SHOT_HOOD_DEG_BY_HUB_DIST.get(3.989), 1e-9);
        assertEquals(1425, AutoAimConstants.TAG_SHOT_RPM_BY_HUB_DIST.get((2.356 + 2.868) / 2), 1e-6);
        assertEquals(1300, AutoAimConstants.TAG_SHOT_RPM_BY_HUB_DIST.get(0.0), 1e-9); // no tag yet = closest row
    }

    /** 10/26 always win, even over a bigger side tag; a side tag only when no front tag is seen. */
    @Test
    void frontTagsBeatSideTags() {
        Set<Integer> red = TagAim.fallbackTags(Optional.of(Alliance.Red));
        assertEquals(10, TagAim.pickTag(new RawFiducial[] { tag(9, 8.0), tag(10, 1.0) }, red).id);
        assertEquals(9, TagAim.pickTag(new RawFiducial[] { tag(8, 2.0), tag(9, 3.0) }, red).id);
        // The OTHER hub's side tags are never used; alliance unknown = front tags only.
        assertNull(TagAim.pickTag(new RawFiducial[] { tag(25, 5.0) }, red));
        assertNull(TagAim.pickTag(new RawFiducial[] { tag(9, 5.0) }, TagAim.fallbackTags(Optional.empty())));
    }

    /** 10 and 26 keep exactly the old offset: straight behind, nothing sideways. */
    @Test
    void middleTagsKeepTheOldOffset() {
        assertEquals(0.0, TagAim.hubTag(10).hubLeftMeters());
        assertEquals(0.0, TagAim.hubTag(26).hubLeftMeters());
        assertEquals(180.0, TagAim.inwardHeading(10).getDegrees(), 1e-9);
        assertEquals(0.0, TagAim.inwardHeading(26).getDegrees(), 1e-9);
    }

    /**
     * Side tags aim at the TRUE hub centre: a robot facing tag 9 (beside 10) or side tag 8 gets
     * the real bearing and range to HUB_CENTER_RED, checked against the field-map tag positions.
     */
    @Test
    void sideTagsAimAtTheRealHubCentre() {
        checkHubFrom(9, new Translation2d(12.519, 3.679), new Translation2d(14.5, 3.0));
        checkHubFrom(8, new Translation2d(12.271, 3.431), new Translation2d(13.8, 1.6));
        checkHubFrom(25, new Translation2d(4.022, 4.390), new Translation2d(2.0, 5.5));
    }

    private static void checkHubFrom(int id, Translation2d tagPos, Translation2d robot) {
        Translation2d hub = id < 17 ? FieldConstants.HUB_CENTER_RED : FieldConstants.HUB_CENTER_BLUE;
        Rotation2d heading = tagPos.minus(robot).getAngle(); // facing the tag
        double range = tagPos.getDistance(robot);
        Rotation2d inward = TagAim.inwardHeading(id).minus(heading);
        double left = TagAim.hubTag(id).hubLeftMeters();
        double truthRight = -hub.minus(robot).getAngle().minus(heading).getDegrees();
        assertEquals(truthRight, TagAim.errorToHubDeg(0, 0, range, inward, left, 0, 1.0), 0.05, "tag " + id);
        assertEquals(hub.getDistance(robot), TagAim.hubDistanceMeters(0, range, inward, left, 1.0), 0.005, "tag " + id);
    }
}
