package frc.robot.subsystems.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Set;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.utility.LimelightHelpers.RawFiducial;

/** Pins the teleop zone-idle logic in AllianceZone. */
class AllianceZoneTest {

    private static RawFiducial[] tags(int... ids) {
        RawFiducial[] out = new RawFiducial[ids.length];
        for (int i = 0; i < ids.length; i++) {
            out[i] = new RawFiducial(ids[i], 0, 0, 0, 2.0, 2.0, 0);
        }
        return out;
    }

    /** Team list: red 7-12 only; blue = the same tags mirrored through the field centre (23-28). */
    @Test
    void zoneTagsAreTheTeamListAndBlueIsTheMirror() {
        assertEquals(Set.of(7, 8, 9, 10, 11, 12), AllianceZone.RED_ZONE_TAGS);
        assertEquals(Set.of(23, 24, 25, 26, 27, 28), AllianceZone.BLUE_ZONE_TAGS);
        for (int red : AllianceZone.RED_ZONE_TAGS) {
            Translation2d mirrored = new Translation2d(16.541, 8.069).minus(FeedAim.tagPose(red).orElseThrow().getTranslation());
            int match = -1;
            for (int blue : AllianceZone.BLUE_ZONE_TAGS) {
                if (FeedAim.tagPose(blue).orElseThrow().getTranslation().getDistance(mirrored) < 0.01) {
                    match = blue;
                }
            }
            assertTrue(match != -1, "red " + red + " has a mirrored blue zone tag");
        }
    }

    /** Only our own alliance's zone tags count; back, hub-neutral-side and the other side's tags don't. */
    @Test
    void seesOnlyOurZoneTags() {
        assertTrue(AllianceZone.seesZoneTag(tags(10), Alliance.Red));
        assertTrue(AllianceZone.seesZoneTag(tags(7), Alliance.Red));
        assertTrue(AllianceZone.seesZoneTag(tags(4, 12), Alliance.Red), "a back tag in the frame doesn't matter");
        assertFalse(AllianceZone.seesZoneTag(tags(1, 2, 3, 4, 5, 6, 26), Alliance.Red));
        assertTrue(AllianceZone.seesZoneTag(tags(28), Alliance.Blue));
        assertFalse(AllianceZone.seesZoneTag(tags(17, 18, 19, 20, 21, 22, 10), Alliance.Blue));
        assertFalse(AllianceZone.seesZoneTag(null, Alliance.Blue));
        assertFalse(AllianceZone.seesZoneTag(tags(), Alliance.Blue));
    }

    /** 5 s after the last sighting; past that only while the robot hasn't moved more than 0.3 m; never outside teleop. */
    @Test
    void holdsFiveSecondsThenOnlyWhileNotMoved() {
        assertTrue(AllianceZone.idleOn(true, 0, 0), "just seen");
        assertTrue(AllianceZone.idleOn(true, 4.9, 3.0), "within 5 s, even after driving away");
        assertFalse(AllianceZone.idleOn(true, 5.1, 3.0), "5 s up and moved: off");
        assertTrue(AllianceZone.idleOn(true, 30, 0.25), "turned in place: odometry holds it");
        assertFalse(AllianceZone.idleOn(true, 30, 0.35), "drove away: off");
        assertFalse(AllianceZone.idleOn(true, Double.POSITIVE_INFINITY, 0), "never seen: off");
        assertFalse(AllianceZone.idleOn(false, 0, 0), "off outside teleop");
    }
}
