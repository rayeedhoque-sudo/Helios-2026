package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.TunerConstants;

/** Pins the 2026-10-10 odometry scale and that full-stick motor speed did not change with it. */
public class OdometryScaleTest {
    @Test
    void wheelRadiusIsScaledAndFullStickMotorSpeedIsUnchanged() {
        double radius = TunerConstants.FrontLeft.WheelRadius;
        assertEquals(0.0508 * 1.1016, radius, 1e-9, "effective wheel radius (m)");
        // Motor speed at full stick ~ speed / radius: must match the pre-scale 4.76 m/s / 2 in.
        assertEquals(4.76 / 0.0508, TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / radius, 1e-6);
    }
}
