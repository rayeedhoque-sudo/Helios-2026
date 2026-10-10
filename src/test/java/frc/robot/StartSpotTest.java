package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.FieldConstants;

/** Pins the companion start-spot lookup: names, blank = no reset, and the red flip. */
class StartSpotTest {
    private static final double TOL = 0.05;

    @Test
    void blankOrUnknownNameMeansNoReset() {
        assertTrue(FieldConstants.startPose("", false).isEmpty());
        assertTrue(FieldConstants.startPose("left_trench", false).isEmpty()); // exact names only
    }

    @Test
    void blueSpotsFaceNeutralZone() {
        Pose2d hub = FieldConstants.startPose("HUB", false).orElseThrow();
        assertEquals(4.022 - 0.9779 / 2, hub.getX(), 1e-9);
        assertEquals(4.035, hub.getY(), 1e-9);
        assertEquals(0.0, hub.getRotation().getDegrees(), 1e-9);
        // Blue driver's left is +Y.
        assertTrue(FieldConstants.startPose("LEFT_TRENCH", false).orElseThrow().getY() > 4.0);
        assertTrue(FieldConstants.startPose("RIGHT_BUMP", false).orElseThrow().getY() < 4.0);
    }

    @Test
    void redFlipKeepsDriverLeftAndFacesNeutralZone() {
        // Red driver faces -X, so their left is -Y: red LEFT_TRENCH must land on red trench
        // tag 6 (11.878, 0.644) side, not tag 1 (y 7.425). Fails if the flip is a mirror.
        Pose2d redLeft = FieldConstants.startPose("LEFT_TRENCH", true).orElseThrow();
        assertEquals(16.541 - 4.626, redLeft.getX(), TOL);
        assertEquals(0.644, redLeft.getY(), TOL);
        assertEquals(180.0, Math.abs(redLeft.getRotation().getDegrees()), 1e-6);
    }
}
