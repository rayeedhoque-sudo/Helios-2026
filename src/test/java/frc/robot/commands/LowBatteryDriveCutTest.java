package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.DriveLimitConstants;

/** Pins the low-battery drive-limit cut (CommandSwerveDrivetrain.nextDriveLimitsCut). */
class LowBatteryDriveCutTest {

    @Test
    void cutsWhenLowAndOnlyRestoresWhenRecovered() {
        assertFalse(CommandSwerveDrivetrain.nextDriveLimitsCut(false, false, false), "healthy stays normal");
        assertTrue(CommandSwerveDrivetrain.nextDriveLimitsCut(false, true, false), "low battery cuts");
        // Once cut, merely rising above the cut voltage (not yet "recovered") must NOT restore -- otherwise the
        // cut lets the voltage recover, restores, sags again, and the limits flap every loop.
        assertTrue(CommandSwerveDrivetrain.nextDriveLimitsCut(true, false, false), "stays cut until recovered");
        assertFalse(CommandSwerveDrivetrain.nextDriveLimitsCut(true, false, true), "restores once recovered");
    }

    @Test
    void thresholdsLeaveAGapAndHalveTheLimits() {
        assertEquals(8.0, DriveLimitConstants.LOW_BATTERY_CUT_VOLTS, 1e-9);
        assertTrue(DriveLimitConstants.LOW_BATTERY_RESTORE_VOLTS > DriveLimitConstants.LOW_BATTERY_CUT_VOLTS);
        assertEquals(0.5, DriveLimitConstants.LOW_BATTERY_LIMIT_SCALE, 1e-9);
    }
}
