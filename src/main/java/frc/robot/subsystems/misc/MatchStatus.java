package frc.robot.subsystems.misc;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Match clock / HUB-shift tracker. Controls no motors. Publishes the match state to
 * NetworkTables (table "Match") for the driver-companion field map, and rumbles the
 * driver controller with a 3-2-1 countdown before every HUB swap.
 *
 * Game facts (2026 REBUILT manual sec.6.4.1 / Table 6-3, verified 2026-07-17 via
 * frcmanual.com/2026/game-details; game-data format from the WPILib 2026 game data
 * docs): AUTO is 0:20, TELEOP is 2:20. Both hubs are active during AUTO, the
 * TRANSITION SHIFT (2:20-2:10) and END GAME (0:30-0:00); SHIFT 1-4 (25 s each)
 * alternate one active hub. Game data = one char, the alliance whose hub goes
 * INACTIVE FIRST ('R'/'B'); that alliance's hub is active in shifts 2 and 4. It
 * arrives ~3 s into teleop (inside the transition shift) and is EMPTY with no FMS.
 */
public class MatchStatus extends SubsystemBase {

    // HUB swap moments, in TELEOP seconds REMAINING (Table 6-3): starts of shifts
    // 1-4 and of END GAME. The active hub changes at every one of these.
    private static final double[] SWAP_BOUNDARIES_SEC = { 130, 105, 80, 55, 30 };

    // Rumble the last 0.35 s of each countdown second (3... 2... 1...) at full
    // strength -- three distinct pulses before every swap. TODO tune feel with driver.
    private static final double COUNTDOWN_SECONDS = 3.0;
    private static final double PULSE_LENGTH_SEC = 0.35;

    private final XboxController controller;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Match");
    private final DoublePublisher timePub = table.getDoubleTopic("time").publish();
    private final StringPublisher periodPub = table.getStringTopic("period").publish();
    private final StringPublisher activeHubPub = table.getStringTopic("activeHub").publish();
    private final DoublePublisher secToSwapPub = table.getDoubleTopic("secToSwap").publish();

    public MatchStatus(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public void periodic() {
        // Seconds remaining in the CURRENT period (auto 20->0, teleop 140->0); -1
        // outside a real match / DS practice session. Approximate (DS-relayed), which
        // is fine for display and 0.35 s rumble pulses -- never use it to gate scoring.
        double t = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        String period;
        String activeHub;
        double secToSwap = -1;

        if (DriverStation.isAutonomousEnabled()) {
            period = "AUTO";
            activeHub = "BOTH";
        } else if (DriverStation.isTeleopEnabled() && t >= 0) {
            if (t > 130) {
                period = "TRANSITION";
                activeHub = "BOTH";
            } else if (t > 30) {
                int shift = (t > 105) ? 1 : (t > 80) ? 2 : (t > 55) ? 3 : 4;
                period = "SHIFT " + shift;
                if (gameData == null || gameData.isEmpty()) {
                    activeHub = "UNKNOWN"; // no FMS data (shouldn't happen in a real match)
                } else {
                    // The inactive-first alliance is active in the EVEN shifts (2, 4).
                    boolean inactiveFirstIsRed = gameData.charAt(0) == 'R';
                    boolean redActive = (shift % 2 == 0) == inactiveFirstIsRed;
                    activeHub = redActive ? "RED" : "BLUE";
                }
            } else {
                period = "END GAME";
                activeHub = "BOTH";
            }
            for (double boundary : SWAP_BOUNDARIES_SEC) {
                if (t > boundary) {
                    secToSwap = t - boundary;
                    break;
                }
            }
        } else if (DriverStation.isTeleopEnabled()) {
            period = "TELEOP"; // enabled with no match clock (shop driving, no FMS/practice)
            activeHub = "BOTH";
        } else {
            period = "DISABLED";
            activeHub = "BOTH";
        }

        timePub.set(t);
        periodPub.set(period);
        activeHubPub.set(activeHub);
        secToSwapPub.set(secToSwap);

        // 3-2-1 swap countdown: one pulse at the top of each of the last 3 seconds
        // before a swap boundary. Teleop-only (nobody holds the controller in auto),
        // and always explicitly cleared otherwise so a pulse can never latch on.
        boolean buzz = false;
        if (DriverStation.isTeleopEnabled() && secToSwap > 0 && secToSwap <= COUNTDOWN_SECONDS) {
            double intoSecond = secToSwap - Math.floor(secToSwap); // 1.0 -> 0.0 as each second elapses
            buzz = intoSecond >= (1.0 - PULSE_LENGTH_SEC);
        }
        controller.setRumble(RumbleType.kBothRumble, buzz ? 1.0 : 0.0);
    }
}
