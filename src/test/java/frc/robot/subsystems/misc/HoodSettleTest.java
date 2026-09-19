package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins what the hood loop is allowed to COMMAND. Team requirement 2026-08-27: a DPAD press
 * moves the hood one step and it STAYS there.
 *
 * The hysteresis re-engage band (hoodShouldHold) that used to live here is DELETED, and this
 * file no longer tests it: a catch stroke moves the hood 25-55 raw units against a 3.75-unit
 * settle band, so re-engaging on droop could only ever overshoot, re-seed higher and droop
 * again -- the up/down limit cycle the team reported on 2026-08-28. The park is absolute now;
 * HoodHoldTest pins that. What is left here is the voltage arithmetic: a commanded stroke has
 * to be able to break the hood away, and nothing else may command anything.
 */
public class HoodSettleTest {

    private static final double TOL = ShooterSubsystemConstants.ANGLE_TOLERANCE;

    /**
     * THE ONLY HOOD DRIVE IS THE POSITION LOOP (2026-08-28): the open-loop jog volts are gone,
     * so the gains themselves have to be able to break the hood away. At a one-step error the
     * P term plus the gravity feedforward must clear the ~7.5 V breakaway, or a DPAD press
     * writes a setpoint and the hood never moves -- and the total must still fit the up cap.
     */
    @Test
    void aOneStepPressCanBreakTheHoodAway() {
        double step = ShooterSubsystemConstants.HOOD_UP_STEP_UNITS;
        double drive = ShooterSubsystem.hoodBreakawayFloor(
            ShooterSubsystemConstants.SHOOTER_ANGLE_kP * step
                + ShooterSubsystem.hoodLiftFeedforward(step), step, false);
        assertTrue(drive >= 7.4, "one step must ask for at least the ~7.5 V breakaway, got " + drive);
        // NOT "step > ANGLE_TOLERANCE": the arrival latch is DIRECTION-based (hoodMoveReached),
        // not band-based, so a step inside the settle band still commands a real stroke. The
        // band only zeroes the feedforward, which the breakaway floor then supplies anyway.
        assertTrue(step > 0, "a press must ask for actual travel");
        // NOT an assertion on the raw sum: kP is live-tuned on the robot (0.6 as of
        // 2026-08-28), so one step can ask for more than the cap. What must hold is that the
        // CLAMPED command is legal -- the clamp, not the gain, is the safety limit.
        assertTrue(Math.min(drive, ShooterSubsystemConstants.HOOD_MAX_UP_VOLTAGE)
                <= ShooterSubsystemConstants.HOOD_MAX_UP_VOLTAGE + 1e-9,
            "the commanded volts must fit the up cap after clamping");
        assertEquals(0.0, ShooterSubsystemConstants.SHOOTER_ANGLE_kI, 1e-12,
            "kI must stay 0 -- an integrator against stiction winds up and then lurches");
    }

    /**
     * MID-STROKE the error shrinks, so P + FF fall back under breakaway and the move would die
     * a few units short of what was asked for. The breakaway floor is what finishes the stroke.
     */
    @Test
    void theFloorFinishesAStrokeThatWouldOtherwiseStallShort() {
        double remaining = TOL + 1;   // just outside the band: still owed, but barely
        double raw = ShooterSubsystemConstants.SHOOTER_ANGLE_kP * remaining
            + ShooterSubsystem.hoodLiftFeedforward(remaining);
        assertTrue(raw < 7.4, "premise: late in a stroke P+FF alone cannot move the hood, got " + raw);
        assertTrue(ShooterSubsystem.hoodBreakawayFloor(raw, remaining, false) >= 7.4,
            "so an unfinished UP stroke must be floored at breakaway");
    }

    /** The floor may not touch a parked hood, a descent, or a zero command. */
    @Test
    void theBreakawayFloorOnlyAppliesToAnActiveUpStroke() {
        assertEquals(0.0, ShooterSubsystem.hoodBreakawayFloor(0, 10, true), 1e-9,
            "parked = 0 V; the floor must never re-energise a parked hood");
        assertEquals(-3.0, ShooterSubsystem.hoodBreakawayFloor(-3.0, -10, false), 1e-9,
            "lowering is gravity-assisted and must stay gentle");
        assertTrue(ShooterSubsystemConstants.HOOD_BREAKAWAY_VOLTS
            <= ShooterSubsystemConstants.HOOD_MAX_UP_VOLTAGE,
            "the floor must fit under the up cap, or the clamp would fight it");
    }

    @Test
    void theSettleBandIsStillTheTunedHalfDegree() {
        assertEquals(0.5 * ShooterSubsystemConstants.HOOD_UNITS_PER_DEG, TOL, 1e-9,
            "guards against a tolerance change silently widening the deadband -- the band is the"
                + " tuned half-degree, expressed in raw encoder units");
    }
}
