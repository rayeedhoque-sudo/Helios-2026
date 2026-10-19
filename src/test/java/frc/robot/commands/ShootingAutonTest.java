package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

/** Pins the Shooting auton (team 2026-10-10): file, intake window, red reset, speed cap. */
public class ShootingAutonTest {
    private static final File AUTOS = new File("src/main/deploy/autos");
    private static final Translation2d[] PTS = {new Translation2d(3.48013, 4.99915),
        new Translation2d(0.80, 5.05405), new Translation2d(0.80, 7.34605), new Translation2d(1.94297, 7.14018)};

    @Test
    void fileStartsAtTheAuthoredSpotWithOne075Cap() {
        HAL.initialize(500, 0);
        Path path = new Path(AUTOS, "Shooting_auton");
        assertEquals(0.0, path.getStartPose().getTranslation().getDistance(PTS[0]), 1e-9);
        // Wall clearance (team 2026-10-10): the editor's 0.52 / 0.42 put the bumper at/past the wall;
        // 0.90 kept the intake off the depot fuel, so 0.80 (11 in intake [assumed] tip ~3 cm off the wall).
        assertEquals(0.80, ((Path.Waypoint) path.getElement(1)).translationTarget().translation().getX(), 1e-9);
        assertEquals(0.80, ((Path.TranslationTarget) path.getElement(3)).translation().getX(), 1e-9);
        var caps = path.getPathConstraints().getMaxVelocityMetersPerSec().orElseThrow();
        assertEquals(1, caps.size());
        assertEquals(0.75, caps.get(0).value(), 1e-9);
        assertEquals(0, caps.get(0).startOrdinal());
        assertEquals(3, caps.get(0).endOrdinal());
    }

    @Test
    void intakeWindowRunsFromTheEventToTheEndOfTheWallTranslation() {
        HAL.initialize(500, 0);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        BLineEventWindow window = new BLineEventWindow();
        FollowPath.setDoubleLoggingConsumer(p -> {
            if (p.getFirst().equals("FollowPath/translationElementIndex")) window.onIndex(p.getSecond());
        });
        FollowPath.registerEventTrigger("event", window::onEvent);
        var dt = TunerConstants.createDrivetrain();
        var cmd = dt.followBLinePathOnField(new Path(AUTOS, "Shooting_auton"), () -> Double.POSITIVE_INFINITY);
        window.reset();
        cmd.initialize();
        for (int leg = 0; leg < 3; leg++) {
            for (int i = 0; i <= 100; i += 5) {
                dt.resetPose(new Pose2d(PTS[leg].interpolate(PTS[leg + 1], i / 100.0), Rotation2d.kZero));
                cmd.execute();
                String at = "leg " + leg + " at " + i + "%";
                if (leg == 1 && i >= 20 && i <= 75) {
                    assertTrue(window.isOpen(), at); // event at t = 0.1527 .. 0.45 m handoff before the end
                } else if (leg != 1 || i <= 10 || i >= 85) {
                    assertFalse(window.isOpen(), at); // also stays shut when the 2nd "event" fires at the end
                }
            }
        }
        cmd.end(true);
    }

    @Test
    void redResetsToTheFlippedStart() {
        HAL.initialize(500, 0);
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        Path path = new Path(AUTOS, "Shooting_auton");
        Path red = path.copy();
        red.flip();
        var dt = TunerConstants.createDrivetrain();
        dt.resetPose(new Pose2d(1, 1, Rotation2d.kZero));
        var cmd = dt.followBLinePathOnField(path, () -> Double.POSITIVE_INFINITY);
        cmd.initialize();
        assertEquals(0.0, dt.getState().Pose.getTranslation().getDistance(red.getStartPose().getTranslation()), 1e-6);
        cmd.end(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1); // leave the sim on blue
        DriverStationSim.notifyNewData();
    }

    @Test
    void capKeepsDirectionAndRotation() {
        var capped = CommandSwerveDrivetrain.capTranslation(new ChassisSpeeds(0.6, 0.8, 2.0), 0.42);
        assertEquals(0.42, Math.hypot(capped.vxMetersPerSecond, capped.vyMetersPerSecond), 1e-9);
        assertEquals(0.8 / 0.6, capped.vyMetersPerSecond / capped.vxMetersPerSecond, 1e-9); // same direction
        assertEquals(2.0, capped.omegaRadiansPerSecond, 1e-9);
        var slow = new ChassisSpeeds(0.1, 0.1, 1.0);
        assertEquals(slow, CommandSwerveDrivetrain.capTranslation(slow, 0.42));
    }
}
