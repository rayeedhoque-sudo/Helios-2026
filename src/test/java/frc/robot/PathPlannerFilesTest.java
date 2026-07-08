package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.HAL;

/**
 * Proves the PathPlanner files in src/main/deploy load with the installed 2026 library.
 * The .path/.auto files on disk are still stamped "version": "2025.0" (deep-review D11);
 * if the 2026 parser ever rejects them (FileVersionException), these tests fail at the desk
 * instead of the robot silently doing nothing in auto at a match.
 */
public class PathPlannerFilesTest {

    @BeforeAll
    static void initHal() {
        // WPILib natives must be initialized before library code touches HAL-backed classes.
        HAL.initialize(500, 0);
    }

    @Test
    void pathFilesParseWithInstalledLib() throws Exception {
        assertNotNull(PathPlannerPath.fromPathFile("Drive Test"));
        assertNotNull(PathPlannerPath.fromPathFile("Rotation Test"));
    }

    @Test
    void autoFileParsesWithInstalledLib() throws Exception {
        assertFalse(PathPlannerAuto.getPathGroupFromAutoFile("Auto PID").isEmpty(),
            "Auto PID should reference at least one path");
    }
}
