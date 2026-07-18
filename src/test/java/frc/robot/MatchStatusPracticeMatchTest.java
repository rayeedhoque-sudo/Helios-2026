package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.misc.MatchStatus;

/**
 * Simulated PRACTICE MATCH for MatchStatus: walks the real subsystem through the
 * 2026 REBUILT match timeline via the sim HAL (DriverStationSim) and asserts the
 * published /Match state and the exact rumble the controller would receive.
 * Timeline facts: manual Table 6-3 (shift boundaries at teleop-remaining
 * 130/105/80/55/30 s); game data char = alliance whose hub goes inactive first.
 */
class MatchStatusPracticeMatchTest {
    private XboxController hid;
    private GenericHIDSim hidSim; // sim-side read-back of the rumble sent to the controller
    private MatchStatus matchStatus;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.resetData();
        hid = new XboxController(0);
        hidSim = new GenericHIDSim(0);
        matchStatus = new MatchStatus(hid);
    }

    /** Put the sim DS in the given match instant, then run one periodic() of the real code. */
    private void step(boolean auto, boolean enabled, double matchTime, String gameData) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(auto);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.setMatchTime(matchTime);
        DriverStationSim.setGameSpecificMessage(gameData);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        matchStatus.periodic();
    }

    private String period() {
        return NetworkTableInstance.getDefault().getTable("Match").getEntry("period").getString("");
    }

    private String activeHub() {
        return NetworkTableInstance.getDefault().getTable("Match").getEntry("activeHub").getString("");
    }

    private double secToSwap() {
        return NetworkTableInstance.getDefault().getTable("Match").getEntry("secToSwap").getDouble(-99);
    }

    /** setRumble(kBothRumble, v) writes both channels; read one back via the sim HAL. */
    private double rumble() {
        return hidSim.getRumble(RumbleType.kLeftRumble);
    }

    @Test
    void fullPracticeMatchRedInactiveFirst() {
        // AUTO (clock 20->0): both hubs active, no rumble.
        step(true, true, 12, "");
        assertEquals("AUTO", period());
        assertEquals("BOTH", activeHub());
        assertEquals(0.0, rumble(), 1e-9);

        // TRANSITION SHIFT (teleop 140->130): both active; swap is 5 s out -> quiet.
        step(false, true, 135, "R");
        assertEquals("TRANSITION", period());
        assertEquals("BOTH", activeHub());
        assertEquals(5.0, secToSwap(), 1e-9);
        assertEquals(0.0, rumble(), 1e-9);

        // Countdown to the SHIFT 1 swap (boundary 130): pulse at the top of each of
        // the last 3 seconds, silent in the gaps between pulses.
        step(false, true, 132.8, "R"); // "3" pulse (2.8 s out, 0.8 into the second)
        assertEquals(1.0, rumble(), 1e-9);
        step(false, true, 132.5, "R"); // gap
        assertEquals(0.0, rumble(), 1e-9);
        step(false, true, 131.9, "R"); // "2" pulse
        assertEquals(1.0, rumble(), 1e-9);
        step(false, true, 130.7, "R"); // "1" pulse
        assertEquals(1.0, rumble(), 1e-9);

        // SHIFT 1: game data 'R' = RED goes inactive first -> BLUE hub active.
        step(false, true, 128, "R");
        assertEquals("SHIFT 1", period());
        assertEquals("BLUE", activeHub());
        assertEquals(0.0, rumble(), 1e-9);

        // SHIFT 2-4 alternate: RED, BLUE, RED.
        step(false, true, 104, "R");
        assertEquals("SHIFT 2", period());
        assertEquals("RED", activeHub());
        step(false, true, 79, "R");
        assertEquals("SHIFT 3", period());
        assertEquals("BLUE", activeHub());
        step(false, true, 54, "R");
        assertEquals("SHIFT 4", period());
        assertEquals("RED", activeHub());

        // Countdown into END GAME (boundary 30) still pulses -- both hubs go active.
        step(false, true, 31.8, "R");
        assertEquals(1.0, rumble(), 1e-9);

        // END GAME: both active, no further swaps, rumble cleared.
        step(false, true, 20, "R");
        assertEquals("END GAME", period());
        assertEquals("BOTH", activeHub());
        assertEquals(-1.0, secToSwap(), 1e-9);
        assertEquals(0.0, rumble(), 1e-9);
    }

    @Test
    void mirrorScheduleWhenBlueInactiveFirst() {
        step(false, true, 128, "B");
        assertEquals("SHIFT 1", period());
        assertEquals("RED", activeHub());
        step(false, true, 104, "B");
        assertEquals("BLUE", activeHub());
    }

    @Test
    void missingGameDataMidShiftReadsUnknown() {
        step(false, true, 100, "");
        assertEquals("SHIFT 2", period());
        assertEquals("UNKNOWN", activeHub());
    }

    @Test
    void noMatchClockInShopTeleop() {
        // Enabled teleop with no FMS/practice clock: getMatchTime() = -1.
        step(false, true, -1, "");
        assertEquals("TELEOP", period());
        assertEquals("BOTH", activeHub());
        assertEquals(0.0, rumble(), 1e-9);
    }

    @Test
    void disableMidPulseClearsRumble() {
        step(false, true, 132.8, "R"); // pulse on
        assertEquals(1.0, rumble(), 1e-9);
        step(false, false, -1, ""); // robot disabled mid-pulse
        assertEquals("DISABLED", period());
        assertEquals(0.0, rumble(), 1e-9); // must never latch on
    }
}
