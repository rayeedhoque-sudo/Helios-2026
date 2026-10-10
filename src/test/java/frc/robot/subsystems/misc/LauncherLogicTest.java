package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.controller.BangBangController;
import frc.robot.Constants.SubsystemConstants.LauncherConstants;

/** Pins the V2 launcher's pure logic (LauncherSubsystem static helpers). */
class LauncherLogicTest {

    /** RT rpm: DPAD steps can't push the target below 0 or above the 4000 ceiling. */
    @Test
    void flywheelRpmIsClamped() {
        assertEquals(0, LauncherSubsystem.clampFlywheelRpm(-100), 1e-9);
        assertEquals(4000, LauncherSubsystem.clampFlywheelRpm(4100), 1e-9);
        assertEquals(2000, LauncherSubsystem.clampFlywheelRpm(LauncherConstants.FLYWHEEL_START_RPM), 1e-9);
    }

    /** Hood: 0 deg (homed bottom stop) to the 29 deg travel limit, nothing outside. */
    @Test
    void hoodTargetIsClamped() {
        assertEquals(0, LauncherSubsystem.clampHoodDeg(-5), 1e-9);
        assertEquals(29, LauncherSubsystem.clampHoodDeg(35), 1e-9);
        assertEquals(15, LauncherSubsystem.clampHoodDeg(15), 1e-9);
    }

    /** 18T -> 26T: the motor turns 26/18 times per flywheel turn, and the conversion round-trips. */
    @Test
    void flywheelRatioConversion() {
        // 1800 flywheel rpm = 30 rps flywheel = 30 * 26/18 = 43.33 motor rps
        assertEquals(30.0 * 26.0 / 18.0, LauncherSubsystem.flywheelRpmToMotorRps(1800), 1e-9);
        assertEquals(1800, LauncherSubsystem.motorRpsToFlywheelRpm(
            LauncherSubsystem.flywheelRpmToMotorRps(1800)), 1e-9);
        // The 4000 ceiling is right at X60 free speed (~96.7 rps) -- keep it that way.
        assertTrue(LauncherSubsystem.flywheelRpmToMotorRps(LauncherConstants.FLYWHEEL_MAX_RPM) <= 97);
    }

    /** Bang-bang flywheel: full voltage below target, only the 90% feedforward at/above it, never negative. */
    @Test
    void flywheelBangBangVolts() {
        double target = 40.0; // motor rps
        double ff = LauncherConstants.FLYWHEEL_BANG_BANG_FF_SCALE * LauncherConstants.FLYWHEEL_kV * target;
        assertEquals(12.0, LauncherSubsystem.flywheelBangBangVolts(new BangBangController(), 30.0, target), 1e-9,
            "below target = full voltage (clamped to 12 V)");
        assertEquals(ff, LauncherSubsystem.flywheelBangBangVolts(new BangBangController(), 41.0, target), 1e-9,
            "above target = feedforward only, no braking");
        assertTrue(ff < LauncherConstants.FLYWHEEL_kV * target, "feedforward alone must stay under the target speed");
        assertTrue(LauncherSubsystem.flywheelBangBangVolts(new BangBangController(), 100.0, target) >= 0);
    }

    /** CAN 14 has a 22T pulley onto 15's 18T: it runs 18/22 as fast, so it gets 18/22 of the voltage. */
    @Test
    void beltedMotorGetsScaledVoltage() {
        assertEquals(18.0 / 22.0, LauncherConstants.FLYWHEEL_BELTED_SPEED_RATIO, 1e-12);
        assertEquals(12.0 * 18.0 / 22.0, LauncherSubsystem.beltedMotorVolts(12.0), 1e-9);
        assertEquals(0.0, LauncherSubsystem.beltedMotorVolts(0.0), 1e-9);
    }

    /** Belt gate: a zero target is never "at speed", and the band is inclusive. */
    @Test
    void atSpeedGate() {
        assertFalse(LauncherSubsystem.atSpeed(0, 0, 100), "stopped wheels must never open the belts");
        assertTrue(LauncherSubsystem.atSpeed(1900, 2000, 100));
        assertFalse(LauncherSubsystem.atSpeed(1899, 2000, 100));
        assertTrue(LauncherSubsystem.atSpeed(2100, 2000, 100));
    }

    /**
     * Homing: startup inrush during the grace window must NOT count as hitting the bottom stop
     * (a false home puts 0 deg mid-travel and the 30 deg limit past the real top stop).
     */
    @Test
    void homingIgnoresInrushDuringGrace() {
        double amps = LauncherConstants.HOOD_STATOR_A; // pegged at the limit
        assertFalse(LauncherSubsystem.homingSawStop(0.0, amps));
        assertFalse(LauncherSubsystem.homingSawStop(LauncherConstants.HOOD_HOMING_GRACE_SEC - 0.01, amps));
        assertTrue(LauncherSubsystem.homingSawStop(LauncherConstants.HOOD_HOMING_GRACE_SEC, amps));
        assertFalse(LauncherSubsystem.homingSawStop(1.0, LauncherConstants.HOOD_HOMING_STOP_AMPS - 1),
            "free travel current must not read as the stop");
        // The stop threshold must be reachable under the hood's own stator limit.
        assertTrue(LauncherConstants.HOOD_HOMING_STOP_AMPS < LauncherConstants.HOOD_STATOR_A);
        assertTrue(LauncherConstants.HOOD_STALL_AMPS < LauncherConstants.HOOD_STATOR_A);
    }

    /**
     * Stall cutoff: a normal P-loop settle a couple of degrees short (~15 A standing still) must
     * NOT latch a fault; a hood pinned 5 deg short at the 20 A cap must.
     */
    @Test
    void hoodStallCutoffOnlyFiresWhenPinned() {
        assertFalse(LauncherSubsystem.hoodPinned(-2.0, 0.0, 15.0), "ordinary settle error");
        assertFalse(LauncherSubsystem.hoodPinned(-5.0, 20.0, 20.0), "still moving toward target");
        assertFalse(LauncherSubsystem.hoodPinned(-5.0, 0.0, 10.0), "not pushing hard");
        assertTrue(LauncherSubsystem.hoodPinned(-5.0, 0.0, 20.0), "pinned against a stop");
        assertTrue(LauncherSubsystem.hoodPinned(5.0, 0.5, 19.0), "either direction");
    }
}
