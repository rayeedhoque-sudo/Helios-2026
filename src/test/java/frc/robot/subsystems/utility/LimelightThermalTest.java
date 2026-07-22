package frc.robot.subsystems.utility;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Checks the two pure decision functions behind the Limelight thermal manager, so a flipped
 * comparison or a broken hysteresis latch fails the build instead of the robot: (1) the
 * throttle must only engage on a trustworthy over-temp and stay latched through the deadband;
 * (2) an active shot must ALWAYS get the full-res pipeline, and a hot idle must drop to idle.
 */
public class LimelightThermalTest {

    private static final double HOT = 83.0;
    private static final double COOL = 78.0;

    @Test
    void throttleNeverEngagesOnInvalidReading() {
        // Even a scorching value must be ignored when the read isn't trusted (stale/out-of-range).
        assertFalse(LimelightThermalManager.nextThrottled(true, false, 200.0, HOT, COOL));
        assertFalse(LimelightThermalManager.nextThrottled(false, false, 90.0, HOT, COOL));
    }

    @Test
    void throttleHysteresis() {
        // Engages at/above HOT.
        assertTrue(LimelightThermalManager.nextThrottled(false, true, HOT, HOT, COOL));
        assertTrue(LimelightThermalManager.nextThrottled(false, true, 90.0, HOT, COOL));
        // Releases at/below COOL.
        assertFalse(LimelightThermalManager.nextThrottled(true, true, COOL, HOT, COOL));
        assertFalse(LimelightThermalManager.nextThrottled(true, true, 74.0, HOT, COOL));
        // Deadband (COOL < t < HOT): holds whatever it was.
        assertTrue(LimelightThermalManager.nextThrottled(true, true, 80.0, HOT, COOL));
        assertFalse(LimelightThermalManager.nextThrottled(false, true, 80.0, HOT, COOL));
    }

    @Test
    void activeShotAlwaysGetsFullPipeline() {
        // Shooting wins over everything -- even while throttled (a pipeline drop wouldn't cool
        // the fixed Hailo, so an active shot is never sacrificed to the throttle).
        assertTrue(LimelightThermalManager.wantFullPipeline(true, true, false));
        assertTrue(LimelightThermalManager.wantFullPipeline(true, false, false));
    }

    @Test
    void idleArbitration() {
        // Not shooting + hot -> idle.
        assertFalse(LimelightThermalManager.wantFullPipeline(false, true, true));
        // Not shooting, cool, recently active -> hold full (avoid churn between shots).
        assertTrue(LimelightThermalManager.wantFullPipeline(false, false, true));
        // Not shooting, cool, not recent -> idle (power saving).
        assertFalse(LimelightThermalManager.wantFullPipeline(false, false, false));
    }
}
