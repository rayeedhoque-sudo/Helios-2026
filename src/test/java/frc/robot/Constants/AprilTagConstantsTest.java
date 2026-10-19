package frc.robot.Constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.SubsystemConstants.AprilTagConstants;
import frc.robot.subsystems.utility.LimelightHelpers.RawFiducial;

/** Pins the AprilTag stand-in lookup (team 2026-10-17): TAG_n = the tag a seen tag n counts as. */
class AprilTagConstantsTest {

    @BeforeAll
    static void hal() {
        HAL.initialize(500, 0); // tagAs() asks the DriverStation whether an FMS is attached
    }

    /** A table with only 22 -> 28 set (a printed 22 hung where tag 28 goes). */
    private static int[] standIn22As28() {
        int[] table = new int[33];
        for (int id = 0; id < table.length; id++) {
            table[id] = id;
        }
        table[22] = 28;
        return table;
    }

    @Test
    void aSeenTagCountsAsItsStandIn() {
        int[] table = standIn22As28();
        assertEquals(28, AprilTagConstants.tagAs(22, false, table), "seen 22 = tag 28");
        assertEquals(28, AprilTagConstants.tagAs(28, false, table), "a real 28 is still 28");
        assertEquals(17, AprilTagConstants.tagAs(17, false, table), "untouched tags are themselves");
    }

    @Test
    void anFmsTurnsEveryStandInOff() {
        assertEquals(22, AprilTagConstants.tagAs(22, true, standIn22As28()));
    }

    /** Through the real DriverStation flags: on in auto and teleop with no FMS, off once an FMS attaches. */
    @Test
    void standInsWorkInAutoAndTeleopButNotWithAnFms() {
        int[] table = standIn22As28();
        try {
            ds(false, true);
            assertEquals(28, AprilTagConstants.tagAs(22, table), "auto, no FMS");
            ds(false, false);
            assertEquals(28, AprilTagConstants.tagAs(22, table), "teleop, no FMS");
            ds(true, true);
            assertEquals(22, AprilTagConstants.tagAs(22, table), "auto, FMS");
            ds(true, false);
            assertEquals(22, AprilTagConstants.tagAs(22, table), "teleop, FMS");
        } finally {
            ds(false, false);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            DriverStation.refreshData();
        }
    }

    /** Enabled robot, auto or teleop, FMS attached or not. */
    private static void ds(boolean fms, boolean auto) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setFmsAttached(fms);
        DriverStationSim.setAutonomous(auto);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        assertEquals(fms, DriverStation.isFMSAttached());
        assertEquals(auto, DriverStation.isAutonomousEnabled());
        assertEquals(!auto, DriverStation.isTeleopEnabled());
    }

    @Test
    void idsOffTheTableAreLeftAlone() {
        int[] table = standIn22As28();
        assertEquals(0, AprilTagConstants.tagAs(0, false, table));
        assertEquals(33, AprilTagConstants.tagAs(33, false, table));
        assertEquals(-1, AprilTagConstants.tagAs(-1, false, table));
    }

    /** remap swaps only the ID: every camera number rides along unchanged. */
    @Test
    void remapKeepsTheCameraNumbers() {
        RawFiducial[] out = AprilTagConstants.remap(new RawFiducial[] {new RawFiducial(22, 1, 2, 3, 4, 5, 6)});
        assertEquals(AprilTagConstants.tagAs(22), out[0].id);
        assertEquals(1, out[0].txnc);
        assertEquals(2, out[0].tync);
        assertEquals(3, out[0].ta);
        assertEquals(4, out[0].distToCamera);
        assertEquals(5, out[0].distToRobot);
        assertEquals(6, out[0].ambiguity);
        assertNull(AprilTagConstants.remap(null));
    }
}
