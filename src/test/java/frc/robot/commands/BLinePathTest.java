package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.Optional;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.Path;

/**
 * Proves every BLine file in src/main/deploy/autos parses at the desk and begins at a companion
 * start spot (the robot auto-adds them all to the chooser), and pins the start-spot guard.
 */
public class BLinePathTest {
    private static Path path;

    @BeforeAll
    static void load() {
        HAL.initialize(500, 0);
        path = new Path(new File("src/main/deploy/autos"), "forward-3m-left-3m");
    }

    @Test
    void testPathParsesAndStartsAtRightTrench() {
        assertTrue(path.isValid());
        assertEquals(0.0, path.getStartPose().getTranslation().getDistance(
            FieldConstants.StartSpot.RIGHT_TRENCH.bluePose.getTranslation()), 1e-9);
        // config.json must win over BLine's 4.5 m/s / 7 m/s^2 fallback (team: match PathPlanner).
        assertEquals(3.0, path.getDefaultGlobalConstraints().getMaxVelocityMetersPerSec(), 1e-9);
        assertEquals(3.0, path.getDefaultGlobalConstraints().getMaxAccelerationMetersPerSec2(), 1e-9);
    }

    @Test
    void guardOnlyPassesForTheMatchingStartSpot() {
        assertTrue(CommandSwerveDrivetrain.startSpotMatchesPath(path, false,
            FieldConstants.startPose("RIGHT_TRENCH", false)));
        assertTrue(CommandSwerveDrivetrain.startSpotMatchesPath(path, true,
            FieldConstants.startPose("RIGHT_TRENCH", true)), "red flip lines up with red spot");
        assertFalse(CommandSwerveDrivetrain.startSpotMatchesPath(path, false,
            FieldConstants.startPose("HUB", false)), "wrong spot refuses");
        assertFalse(CommandSwerveDrivetrain.startSpotMatchesPath(path, true,
            FieldConstants.startPose("RIGHT_TRENCH", false)), "blue spot on red refuses");
        assertFalse(CommandSwerveDrivetrain.startSpotMatchesPath(path, false, Optional.empty()),
            "no pick refuses");
    }

    @Test
    void everyDeployedPathParsesAndStartsAtAStartSpot() {
        File autos = new File("src/main/deploy/autos");
        var names = RobotContainer.blinePathNames(new File(autos, "paths"));
        assertTrue(names.contains("forward-3m-left-3m"), "lister finds the deployed paths");
        for (String name : names) {
            Path p = new Path(autos, name);
            assertTrue(p.isValid(), name + " is a valid path");
            assertNotNull(CommandSwerveDrivetrain.startSpotFor(p),
                name + " must begin within 0.5 m of a FieldConstants.StartSpot, or it can never run");
        }
        assertEquals("RIGHT_TRENCH", CommandSwerveDrivetrain.startSpotFor(path));
        assertTrue(RobotContainer.blinePathNames(new File(autos, "missing")).isEmpty(), "no folder = no paths");
    }
}
