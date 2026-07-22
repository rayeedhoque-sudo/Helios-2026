package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins ShooterSubsystem.modelSurfaceSpeed() against the quadratic-drag integrator's truth
 * (docs/shot-model/refit_38.py, run 2026-07-21 at MAX_ANGLE = 38 deg), so a model
 * regression -- or an angle/fit mismatch like the 44.5-vs-38 staleness this refit fixed --
 * fails at the desk instead of missing on the field.
 *
 * Pins are BALL speeds (m/s). modelSurfaceSpeed returns SURFACE speed = ball/SHOT_EFFICIENCY,
 * so each check multiplies the output BY the efficiency -- the pins stay valid however
 * SHOT_EFFICIENCY is tuned on the robot. Tolerance 3%: roomy enough for small on-robot
 * drag-endpoint tweaks, far tighter than the ~6% a stale-angle fit produces.
 *
 * If this fails after DELIBERATELY changing MAX_ANGLE or the fit constants: that is the
 * guard working. Re-run docs/shot-model/refit_38.py at the new angle and update the fit
 * constants AND these pins together.
 */
public class ShotModelTest {

    private static final double TOL_FRAC = 0.03;
    // Target height above release for a SCORE shot (hub mouth) -- same terms periodic() uses.
    private static final double DH_SCORE = FieldConstants.HUB_OPENING_HEIGHT_METERS
            - ShooterSubsystemConstants.SHOT_RELEASE_HEIGHT_METERS;
    // FEED lob lands on the carpet.
    private static final double DH_FEED = -ShooterSubsystemConstants.SHOT_RELEASE_HEIGHT_METERS;

    private static double scoreMult(double d) {
        return ShooterSubsystemConstants.SCORE_DRAG_MULT_BASE
                + ShooterSubsystemConstants.SCORE_DRAG_MULT_PER_METER * d;
    }

    private static double feedMult(double d) {
        return ShooterSubsystemConstants.FEED_DRAG_MULT_BASE
                + ShooterSubsystemConstants.FEED_DRAG_MULT_PER_METER * d;
    }

    private static void assertBallSpeed(double expectedBallMps, double distance, double deltaH,
            double dragMult) {
        double surface = ShooterSubsystem.modelSurfaceSpeed(distance, deltaH, dragMult);
        assertTrue(surface > 0, "model refused a shot inside the envelope at d=" + distance);
        assertEquals(expectedBallMps,
                surface * ShooterSubsystemConstants.SHOT_EFFICIENCY,
                expectedBallMps * TOL_FRAC,
                "ball speed off integrator truth at d=" + distance);
    }

    @Test
    void scoreSpeedsMatchIntegrator() {
        // refit_38.py pinned truth (ball m/s at 38 deg).
        assertBallSpeed(8.069, 3.5, DH_SCORE, scoreMult(3.5));
        assertBallSpeed(8.580, 4.5, DH_SCORE, scoreMult(4.5));
        assertBallSpeed(9.186, 5.5, DH_SCORE, scoreMult(5.5));
        assertBallSpeed(9.626, 6.2, DH_SCORE, scoreMult(6.2));
    }

    @Test
    void feedSpeedsMatchIntegrator() {
        assertBallSpeed(6.876, 5.0, DH_FEED, feedMult(5.0));
        assertBallSpeed(8.481, 7.0, DH_FEED, feedMult(7.0));
        assertBallSpeed(9.939, 9.0, DH_FEED, feedMult(9.0));
    }

    @Test
    void refusesOutsidePhysicalLimits() {
        // Below the 38-deg descending-entry floor (2.93 m) the ball can't cross the mouth
        // past apex -- must refuse (0), never lob a guaranteed miss.
        assertEquals(0, ShooterSubsystem.modelSurfaceSpeed(2.8, DH_SCORE, scoreMult(2.8)));
        // Degenerate distances refuse.
        assertEquals(0, ShooterSubsystem.modelSurfaceSpeed(0, DH_SCORE, 1.0));
        assertEquals(0, ShooterSubsystem.modelSurfaceSpeed(-1, DH_SCORE, 1.0));
        // Far beyond the motor velocity ceiling (95 rps) -- refuse rather than lob short.
        assertEquals(0, ShooterSubsystem.modelSurfaceSpeed(40, DH_SCORE, scoreMult(40)));
    }

    @Test
    void envelopeSitsAboveDescendingFloor() {
        // Consistency guard: the commandable minimum must clear the physical descending-entry
        // floor 2*dH/tan(MAX_ANGLE) with margin -- catches a MIN or MAX_ANGLE edit made
        // without re-running the refit.
        double floorMeters = 2 * DH_SCORE
                / Math.tan(Math.toRadians(ShooterSubsystemConstants.MAX_ANGLE));
        assertTrue(ShooterSubsystemConstants.MIN_SCORE_DISTANCE_METERS >= floorMeters + 0.4,
                "MIN_SCORE_DISTANCE (" + ShooterSubsystemConstants.MIN_SCORE_DISTANCE_METERS
                        + " m) too close to the descending-entry floor (" + floorMeters
                        + " m) -- re-run refit_38.py");
    }
}
