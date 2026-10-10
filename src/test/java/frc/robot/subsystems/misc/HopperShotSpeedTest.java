package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;

/**
 * The shot belt duty (RT) against the normal one (LT intake, A manual run).
 * Team asked for "twice as fast" on the shot paths only, 2026-08-29.
 */
class HopperShotSpeedTest {

    /** Doubled, but full output is the ceiling -- 0.6 doubled is 1.2, which does not exist. */
    @Test
    void theShotDutyIsDoubledAndClampedToFullOutput() {
        double normal = HopperSubsystemConstants.HOPPER_SPEED;
        double shot = HopperSubsystemConstants.HOPPER_SHOT_SPEED;
        assertEquals(Math.min(1.0, normal * 2.0), shot, 1e-12);
        assertTrue(shot <= 1.0, "a duty above full output is not a thing");
        // Team set HOPPER_SPEED to 1.0 on 2026-10-09, so the shot can no longer be faster --
        // both run at full output. It must never be SLOWER.
        assertTrue(shot >= normal, "the shot paths must never be slower");
        // With HOPPER_SPEED at 0.6 the real gain is 1.67x, not the 2x asked for -- worth
        // failing loudly if anyone later assumes it is a true doubling.
        if (normal * 2.0 > 1.0) {
            assertEquals(1.0, shot, 1e-12, "pinned at full output, so the gain is 1/normal");
        }
    }

    /** The normal duty is untouched: LT intake and the A manual hopper run must not speed up. */
    @Test
    void theNormalDutyIsUnchanged() {
        assertEquals(1.0, HopperSubsystemConstants.HOPPER_SPEED, 1e-12,
            "changing this silently re-times the intake feed as well as the shot");
    }
}
