package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Pins the NaN vision guard (2026-10-17): a NaN MegaTag2 solve is never fused into the pose. */
public class VisionNaNGuardTest {
    @Test
    void nanSolvesAreRejected() {
        // What the LL4 sent with its internal IMU reading NaN: botpose_orb = [nan, nan, ..., yaw 0].
        assertFalse(CommandSwerveDrivetrain.isFinitePose(new Pose2d(Double.NaN, Double.NaN, Rotation2d.kZero)));
        assertFalse(CommandSwerveDrivetrain.isFinitePose(new Pose2d(3, Double.NaN, Rotation2d.kZero)));
        assertFalse(CommandSwerveDrivetrain.isFinitePose(new Pose2d(Double.POSITIVE_INFINITY, 2, Rotation2d.kZero)));
        assertFalse(CommandSwerveDrivetrain.isFinitePose(new Pose2d(3, 2, Rotation2d.fromRadians(Double.NaN))));
    }

    @Test
    void realSolvesPass() {
        assertTrue(CommandSwerveDrivetrain.isFinitePose(new Pose2d(8.37, 1.39, Rotation2d.fromDegrees(17.6))));
    }
}
