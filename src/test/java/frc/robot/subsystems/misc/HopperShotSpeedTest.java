package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;

/**
 * The shot belt duty (B feed shot / RT / RB) against the normal one (LT intake, A manual run).
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
        assertTrue(shot > normal, "the shot paths must actually be faster");
        // With HOPPER_SPEED at 0.6 the real gain is 1.67x, not the 2x asked for -- worth
        // failing loudly if anyone later assumes it is a true doubling.
        if (normal * 2.0 > 1.0) {
            assertEquals(1.0, shot, 1e-12, "pinned at full output, so the gain is 1/normal");
        }
    }

    /** The kicker's shot duty is doubled and clamped the same way the belts are. */
    @Test
    void theKickerShotDutyIsDoubledAndClamped() {
        double normal = HopperSubsystemConstants.INDEXER_SPEED;
        double shot = HopperSubsystemConstants.KICKER_SHOT_SPEED;
        assertEquals(Math.min(1.0, normal * 2.0), shot, 1e-12);
        assertTrue(shot <= 1.0, "a duty above full output is not a thing");
        assertTrue(shot > normal, "the shot paths must actually kick faster");
    }

    /**
     * The REVERSE is deliberately not doubled. It would fight the now-full-speed belts harder
     * and make the reverse-to-forward plugging step worse on a CIM with no current sensing --
     * the one part of this change that was not worth making.
     */
    @Test
    void theKickerReverseIsNotDoubled() {
        assertEquals(0.3, HopperSubsystemConstants.UNJAM_SPEED, 1e-12);
        assertTrue(HopperSubsystemConstants.UNJAM_SPEED
            < HopperSubsystemConstants.KICKER_SHOT_SPEED / 2,
            "the reverse must stay well below the forward shot duty");
    }

    /** The normal duty is untouched: LT intake and the A manual hopper run must not speed up. */
    @Test
    void theNormalDutyIsUnchanged() {
        assertEquals(0.6, HopperSubsystemConstants.HOPPER_SPEED, 1e-12,
            "changing this silently re-times the intake feed as well as the shot");
    }
}
