package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/** Pins the auto stall abort windows: drive 0.3 s, steer 1.0 s (team 2026-10-14). */
public class StallCheckTest {
    @BeforeEach
    void pause() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
    }

    @AfterEach
    void resume() {
        SimHooks.resumeTiming();
    }

    @Test
    void steerStallNeedsAFullSecond() {
        var check = CommandSwerveDrivetrain.stallCheck(() -> false, () -> true);
        check.getAsBoolean();
        SimHooks.stepTiming(0.6); // module 0's normal-turn spikes were ~0.3 s
        assertFalse(check.getAsBoolean(), "0.6 s of steer at the limit must not abort");
        SimHooks.stepTiming(0.5);
        assertTrue(check.getAsBoolean(), "a steer motor jammed past 1 s still aborts");
    }

    @Test
    void driveStallStillAbortsAfterPointThreeSeconds() {
        var check = CommandSwerveDrivetrain.stallCheck(() -> true, () -> false);
        check.getAsBoolean();
        SimHooks.stepTiming(0.2);
        assertFalse(check.getAsBoolean());
        SimHooks.stepTiming(0.15);
        assertTrue(check.getAsBoolean(), "pushing a wall still aborts at 0.3 s");
    }
}
