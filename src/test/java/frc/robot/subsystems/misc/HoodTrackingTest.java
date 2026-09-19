package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins the CONTINUOUS hood tracking (2026-08-22). The hood angle is a datum captured at
 * enable plus every shortest-path delta since, instead of a fresh interpretation of each
 * raw reading.
 *
 * Why: interpreting one reading through a fixed wrap split flips 56 deg across a single
 * count near the split (raw 103.4 = +52.0 deg, raw 103.6 = -4.3 deg). At the negative
 * reading the travel guard believes the hood is at the bottom and drives UP at full
 * feedforward -- the hood shooting up uncontrollably that the team reported. Relative
 * tracking cannot produce that jump, and the noise deadband stops slow encoder drift from
 * moving the angle at all.
 */
public class HoodTrackingTest {

    /**
     * An arbitrary commanded step, in degrees. These tests exercise clampDesiredAngle, which
     * does not care HOW a setpoint was asked for -- the DPAD steps the hood open loop and
     * stops on the measured angle at a fixed target, so a local constant keeps the clamp tests
     * independent of that tuning. clampDesiredAngle is also what bounds a DPAD step's target.
     */
    private static final double STEP = 2.0;

    private static final double DEG_PER_UNIT = ShooterSubsystemConstants.HOOD_DEG_PER_RAW_UNIT;

    // ---- shortest-path delta: the rollover must be a non-event ----

    @Test
    void rolloverUpIsASmallPositiveDelta() {
        assertEquals(1.5, ShooterSubsystem.shortestRawDelta(359.0, 0.5), 1e-9,
            "359 -> 0.5 is +1.5, not -358.5");
    }

    @Test
    void rolloverDownIsASmallNegativeDelta() {
        assertEquals(-1.5, ShooterSubsystem.shortestRawDelta(0.5, 359.0), 1e-9,
            "0.5 -> 359 is -1.5, not +358.5");
    }

    @Test
    void ordinaryMotionIsUnchanged() {
        assertEquals(5.0, ShooterSubsystem.shortestRawDelta(150.0, 155.0), 1e-9);
        assertEquals(-5.0, ShooterSubsystem.shortestRawDelta(155.0, 150.0), 1e-9);
    }

    /** The exact failure: sweeping across the old split must accumulate smoothly. */
    @Test
    void sweepingAcrossTheOldSplitAccumulatesSmoothly() {
        double[] sweep = { 95.0, 100.0, 103.4, 103.6, 108.0, 115.0 };
        double travelled = 0;
        for (int i = 1; i < sweep.length; i++) {
            double step = ShooterSubsystem.hoodDeltaDegrees(
                ShooterSubsystem.shortestRawDelta(sweep[i - 1], sweep[i]));
            assertTrue(Math.abs(step) < 8.0,
                "no step across the old split may be large; got " + step + " units");
            travelled += step;
        }
        // The 103.4 -> 103.6 step lands right on the noise deadband and may be filtered out
        // (binary 0.2 is a hair under it), so allow one deadband of slack.
        assertEquals((115.0 - 95.0) * DEG_PER_UNIT, travelled,
            ShooterSubsystemConstants.HOOD_RAW_NOISE_DEADBAND + 0.05,
            "total travel must equal the raw span times the scale");
    }

    // ---- noise and glitch rejection: "slight changes must not affect the hood" ----

    @Test
    void noiseBelowTheDeadbandIsNotTravel() {
        assertEquals(0, ShooterSubsystem.hoodDeltaDegrees(0.08), 1e-9, "measured noise is +-0.08");
        assertEquals(0, ShooterSubsystem.hoodDeltaDegrees(-0.15), 1e-9);
    }

    @Test
    void noiseCannotAccumulateOverManyLoops() {
        double drift = 0;
        for (int loop = 0; loop < 5000; loop++) { // ~100 s at 50 Hz, jittering one way
            drift += ShooterSubsystem.hoodDeltaDegrees(0.08);
        }
        assertEquals(0, drift, 1e-9, "a still hood must never accumulate phantom travel");
    }

    @Test
    void glitchesAboveTheMaxStepAreRejected() {
        assertEquals(0, ShooterSubsystem.hoodDeltaDegrees(150.0), 1e-9,
            "150 units in one loop is a garbled frame; above 180 the wrap logic could not even"
                + " tell which way it went");
    }

    /**
     * THE REGRESSION THAT GROUND THE BELT (2026-08-27). A powered move steps the encoder 25-55
     * raw units per 20 ms loop; the old 20-unit ceiling rejected all of it, so the tracked angle
     * froze mid-move, the MAX_ANGLE travel guard never fired, and the hood drove into its stop.
     * Powered-speed deltas must COUNT.
     */
    @Test
    void poweredSpeedDeltasAreMotionNotGlitches() {
        assertEquals(55.0 * DEG_PER_UNIT, ShooterSubsystem.hoodDeltaDegrees(55.0), 1e-9,
            "the fastest measured powered step must be tracked, not filtered away");
    }

    @Test
    void realMotionStillCountsBetweenTheTwoFilters() {
        assertEquals(2.0 * DEG_PER_UNIT, ShooterSubsystem.hoodDeltaDegrees(2.0), 1e-9);
    }

    // ---- datum capture: offset-independent by construction ----

    /**
     * THE DATUM IS THE RAW ENCODER READING (2026-08-27, team request: the hood angle is the raw
     * encoder angle, and the raw value at enable IS the base). Everything the hood bounds
     * itself with is therefore placed around that reading -- nothing compares it against a
     * physical degree, which is what killed the DPAD before (the resting hood at raw 311.169
     * was read as 44.5 deg, the TOP stop, so the guard never allowed an upward volt).
     */
    @Test
    void theDatumIsWhateverTheEncoderReadsAtEnable() {
        double base = 311.169;   // the live resting reading, captured 2026-08-27
        assertEquals(base, ShooterSubsystem.hoodFloorAngle(base, true), 1e-9,
            "the floor is the raw reading at enable");
        assertEquals(base + ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG,
            ShooterSubsystem.hoodCeilingAngle(base, true), 1e-9,
            "the ceiling is one full travel above it -- MAX_ANGLE may never bound a raw reading");
        assertEquals(base + 5,
            ShooterSubsystem.clampDesiredAngle(base + 5, base, true), 1e-9,
            "DPAD-RIGHT's base + 5 must survive the clamp at any encoder offset");
        // The window is now 290 (team direction 2026-08-30), deliberately SHORT of the full
        // geometric stroke so the top stop keeps a margin. The two constants must stay
        // separate: HOOD_RAW_UNITS_PER_FULL_TRAVEL is the SCALE behind HOOD_UNITS_PER_DEG and
        // every hood gain, so re-aliasing the window to it would rescale the whole loop.
        assertTrue(ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG
                < ShooterSubsystemConstants.HOOD_RAW_UNITS_PER_FULL_TRAVEL,
            "the ceiling must hold margin back from the physical top stop");
        assertTrue(ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG
                > ShooterSubsystemConstants.FEED_SHOT_HOOD_UNITS,
            "the feed shot angle must still fit under the ceiling");
    }

    /**
     * THE HEADLINE PROPERTY (2026-08-27): the tracked angle IS the raw encoder reading. Datum
     * at enable plus 1:1 deltas must land back on whatever the encoder says, wander included.
     */
    @Test
    void theTrackedAngleIsTheRawEncoderReading() {
        double[] raws = { 311.169, 313.0, 318.5, 316.0, 314.25 };
        double base = raws[0];
        double relative = 0;
        for (int i = 1; i < raws.length; i++) {
            relative = ShooterSubsystem.accumulateHoodTravel(relative, base,
                ShooterSubsystem.hoodDeltaDegrees(
                    ShooterSubsystem.shortestRawDelta(raws[i - 1], raws[i])));
        }
        assertEquals(raws[raws.length - 1], base + relative, 1e-9,
            "the hood angle must equal the raw encoder angle, not a scaled version of it");
    }

    /**
     * The degree-tuned bands and gains were converted into raw units by ONE factor. If it ever
     * drifts from the measured anchors the whole loop is mistuned silently, so pin it.
     */
    @Test
    void theUnitFactorMatchesTheMeasuredAnchors() {
        assertEquals(ShooterSubsystemConstants.HOOD_RAW_UNITS_PER_FULL_TRAVEL
                / (ShooterSubsystemConstants.HOOD_DEG_AT_FULL_UP
                    - ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN),
            ShooterSubsystemConstants.HOOD_UNITS_PER_DEG, 1e-9,
            "HOOD_UNITS_PER_DEG must stay in step with the calibration anchors");
        assertEquals(1.0, ShooterSubsystemConstants.HOOD_DEG_PER_RAW_UNIT, 1e-9,
            "the tracker counts raw units 1:1");
    }

    // ---- travel window ----

    @Test
    void windowLimitsTravelFromTheEnablePosition() {
        double base = 10.0;
        double window = ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG;
        assertEquals(base + window,
            ShooterSubsystem.clampDesiredAngle(9999, base, true), 1e-9,
            "cannot be commanded further up than the window allows");
        assertEquals(base,
            ShooterSubsystem.clampDesiredAngle(-9999, base, true), 1e-9,
            "can never be commanded BELOW the enable-time datum");
    }

    @Test
    void softLimitsStillApplyWithoutADatum() {
        assertEquals(ShooterSubsystemConstants.MAX_ANGLE,
            ShooterSubsystem.clampDesiredAngle(999, 0, false), 1e-9);
        assertEquals(ShooterSubsystemConstants.MIN_ANGLE,
            ShooterSubsystem.clampDesiredAngle(-999, 0, false), 1e-9);
    }

    // ---- anti-windup: the tracked angle can never leave the physical stops ----

    @Test
    void travelSaturatesAtTheUpStop() {
        double base = 311.169;
        double window = ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG;
        double relative = ShooterSubsystem.accumulateHoodTravel(window - 1, base, 10.0);
        assertEquals(base + window, base + relative, 1e-9,
            "accumulation must saturate one full travel above the datum, not run past it");
    }

    @Test
    void travelSaturatesAtTheDownStop() {
        double base = 311.169;
        double relative = ShooterSubsystem.accumulateHoodTravel(-1.0, base, -40.0);
        assertEquals(base, base + relative, 1e-9,
            "the tracked angle may never fall below the enable-time datum");
    }

    @Test
    void ordinaryTravelAccumulatesNormally() {
        assertEquals(7.0, ShooterSubsystem.accumulateHoodTravel(5.0, 20.0, 2.0), 1e-9);
    }

    // ---- feedback validity is now ONLY the dead-encoder test ----

    @Test
    void anyDialPositionIsValidFeedback() {
        // The old absolute band rejected these, which forced 0 V and left the hood dead.
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(103.6), "dead-band raw must be usable now");
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(60.0));
        assertTrue(ShooterSubsystem.isHoodFeedbackValid(359.9));
    }

    @Test
    void deadEncoderIsStillRejected() {
        assertTrue(!ShooterSubsystem.isHoodFeedbackValid(0.0),
            "bit-exact 0.0 is a silent controller, not a position");
    }

    // ---- DPAD nudge: repeated clicks accumulate but stay bounded ----

    /**
     * The nudge steps from the CURRENT SETPOINT, so leaning on the button walks the setpoint
     * up -- and must stop at the window edge rather than running away from the hood.
     */
    @Test
    void repeatedNudgesSaturateAtTheWindowEdge() {
        double base = 10.0;
        double setpoint = base;
        // Enough clicks to walk the whole window (309.6 units at 2 per click) and then some.
        for (int click = 0; click < 400; click++) {
            setpoint = ShooterSubsystem.clampDesiredAngle(
                setpoint + STEP, base, true);
        }
        assertEquals(base + ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG,
            setpoint, 1e-9, "400 steps up must stop at the window edge");

        for (int click = 0; click < 400; click++) {
            setpoint = ShooterSubsystem.clampDesiredAngle(
                setpoint - STEP, base, true);
        }
        assertEquals(base, setpoint, 1e-9,
            "400 steps down must stop at the enable-time datum, never below it");
    }

    /**
     * The floor is the ENABLE-TIME DATUM, whether that sits above or below the fixed MIN_ANGLE
     * soft limit -- a hood enabled raised cannot be driven back down past where it started,
     * and a hood enabled resting below MIN_ANGLE is still not dragged up to it. Without a
     * datum there is nothing measured to floor against, so MIN_ANGLE stands in.
     */
    @Test
    void theFloorIsTheEnableTimeDatum() {
        double resting = ShooterSubsystemConstants.MIN_ANGLE - 1.8;   // the ~3.2 deg rest
        assertEquals(resting, ShooterSubsystem.hoodFloorAngle(resting, true), 1e-9);
        assertEquals(resting, ShooterSubsystem.clampDesiredAngle(-999, resting, true), 1e-9);
        double raised = 30.0;
        assertEquals(raised, ShooterSubsystem.hoodFloorAngle(raised, true), 1e-9);
        assertEquals(raised, ShooterSubsystem.clampDesiredAngle(5.0, raised, true), 1e-9,
            "a commanded shot angle below the enable position is floored too");
        assertEquals(ShooterSubsystemConstants.MIN_ANGLE,
            ShooterSubsystem.hoodFloorAngle(raised, false), 1e-9,
            "no datum yet -> the fixed soft limit stands in");
    }

    /** One click must move the setpoint by exactly the step, mid-range. */
    @Test
    void oneClickMovesExactlyOneStep() {
        double base = 20.0;
        assertEquals(20.0 + STEP,
            ShooterSubsystem.clampDesiredAngle(
                20.0 + STEP, base, true), 1e-9,
            "one step moves exactly one step, whatever the step is");
    }

    /** A datum far outside the soft band must still produce a usable, non-inverted range. */
    @Test
    void aDatumOutsideTheBandStillYieldsARealSetpoint() {
        double clamped = ShooterSubsystem.clampDesiredAngle(20, 80, true);
        assertTrue(Double.isFinite(clamped), "must produce a real setpoint, not NaN");
        // The band now widens to include the datum instead of collapsing, so the limits can
        // never cross. The setpoint stays between the datum and the band it is heading toward.
        assertTrue(clamped >= ShooterSubsystemConstants.MIN_ANGLE && clamped <= 80,
            "must be a reachable setpoint between the soft floor and the datum: " + clamped);
    }

    // ---- the soft band must never DEMAND motion (team request 2026-08-26) ----
    // The hood RESTS at ~3.2 deg, below the 5.0 deg MIN_ANGLE floor. The band used to clamp the
    // enable-time seed up to the floor, so every enable drove the hood up at ~7.6 V. These pin
    // that it holds instead.

    /** THE REGRESSION GUARD: enabling with the hood parked below the soft floor must not move it. */
    @Test
    void enablingBelowTheSoftFloorDoesNotCommandMotion() {
        double rest = ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN; // ~3.224, below MIN_ANGLE
        assertTrue(rest < ShooterSubsystemConstants.MIN_ANGLE,
            "precondition: the hood's rest position is below the soft floor");
        assertEquals(rest, ShooterSubsystem.clampDesiredAngle(rest, rest, true), 1e-9,
            "the seeded setpoint must equal the enable position exactly, or the hood lurches on enable");
    }

    /** DPAD-down at the floor holds; it must never jump the hood UP to satisfy MIN_ANGLE. */
    @Test
    void nudgingDownBelowTheFloorHoldsInsteadOfRising() {
        double rest = ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN;
        double commanded = ShooterSubsystem.clampDesiredAngle(
            rest - STEP, rest, true);
        assertEquals(rest, commanded, 1e-9, "must hold at the enable position, not rise to MIN_ANGLE");
    }

    /** Enabled below the floor, the hood may still be commanded UP toward the band. */
    @Test
    void nudgingUpFromBelowTheFloorStillWorks() {
        double rest = ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN;
        assertEquals(rest + STEP,
            ShooterSubsystem.clampDesiredAngle(rest + STEP, rest, true),
            1e-9, "the band opens toward the hood, it does not trap it");
    }

    /**
     * THE REGRESSION THIS REPLACES (2026-08-27): the floor used to be min(base, MAX_ANGLE), so
     * a raw datum of ~311 collapsed the floor to 38 -- the down guard could never fire and a
     * lowering command would have driven ~270 units into the bottom stop. A raw reading is
     * never compared against a physical degree now: the floor is the datum, wherever it sits.
     */
    @Test
    void aRawDatumAboveMaxAngleIsStillFlooredAtTheDatum() {
        double high = 311.169;   // far above MAX_ANGLE (38) -- a raw reading, not a degree
        assertEquals(high, ShooterSubsystem.hoodFloorAngle(high, true), 1e-9,
            "the floor may never collapse to MAX_ANGLE");
        assertEquals(high, ShooterSubsystem.clampDesiredAngle(high - 200, high, true), 1e-9,
            "no command may take the hood below the datum");
        assertEquals(high + STEP,
            ShooterSubsystem.clampDesiredAngle(high + STEP, high, true), 1e-9,
            "and it must still be able to rise");
    }

    /** The widened band must never invert, whatever the datum. */
    @Test
    void theBandAlwaysContainsTheDatum() {
        for (double base = -10; base <= 360; base += 0.5) {
            assertEquals(base, ShooterSubsystem.clampDesiredAngle(base, base, true), 1e-9,
                "clamping the datum to itself must be a no-op at base=" + base);
        }
    }
}
