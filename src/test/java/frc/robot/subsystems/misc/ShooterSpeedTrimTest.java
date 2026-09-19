package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;

/**
 * Pins the DPAD UP/DOWN flywheel trim (team request 2026-08-22): the driver adjusts the RT
 * target in motor RPM, everything downstream works in surface speed, and the conversion
 * between them must match what the velocity command does in the other direction.
 *
 * The clamp matters most: the DPAD must not be able to walk the target past the motor
 * ceiling the shot model already refuses above, nor below zero into a negative command.
 */
public class ShooterSpeedTrimTest {

    private static final double CEILING = ShooterSubsystemConstants.SHOT_MAX_MOTOR_RPS * 2 * Math.PI
        * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
        * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION;

    @Test
    void rpmConvertsToSurfaceSpeedTheSameWayTheVelocityCommandDoes() {
        // Inverse of the periodic() conversion: surface -> motorRps.
        double surface = ShooterSubsystem.surfaceSpeedForMotorRpm(1200);
        double motorRps = surface / (2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
            * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION);
        assertEquals(20.0, motorRps, 1e-9, "1200 RPM is 20 rev/sec at the motor");
    }

    @Test
    void twoHundredRpmIsTheExpectedStep() {
        assertEquals(1.596, ShooterSubsystem.surfaceSpeedForMotorRpm(200), 0.001,
            "200 motor RPM on a 2 in wheel through 1.5:1 is ~1.6 m/s of surface speed");
    }

    @Test
    void trimsUpAndDownSymmetrically() {
        double start = 15.84;
        double up = ShooterSubsystem.trimSurfaceSpeed(start, 200);
        assertEquals(start, ShooterSubsystem.trimSurfaceSpeed(up, -200), 1e-9,
            "up then down must return to where it started");
    }

    @Test
    void cannotBeTrimmedPastTheMotorCeiling() {
        double atCeiling = ShooterSubsystem.trimSurfaceSpeed(CEILING, 200);
        assertEquals(CEILING, atCeiling, 1e-9, "the DPAD must not exceed SHOT_MAX_MOTOR_RPS");
        // Even a long lean on the button.
        double walked = 15.84;
        for (int press = 0; press < 200; press++) {
            walked = ShooterSubsystem.trimSurfaceSpeed(walked, 200);
        }
        assertEquals(CEILING, walked, 1e-9, "200 presses must still stop at the ceiling");
    }

    @Test
    void cannotBeTrimmedBelowZero() {
        double walked = 15.84;
        for (int press = 0; press < 200; press++) {
            walked = ShooterSubsystem.trimSurfaceSpeed(walked, -200);
        }
        assertEquals(0, walked, 1e-9, "the floor is 0 -- a refused shot, never a negative command");
    }

    @Test
    void theSeedValueIsInsideTheLegalRange() {
        assertTrue(ShooterSubsystemConstants.RT_FLYWHEEL_SURFACE_SPEED > 0
                && ShooterSubsystemConstants.RT_FLYWHEEL_SURFACE_SPEED < CEILING,
            "the constant the trim starts from must itself be commandable");
    }
}
