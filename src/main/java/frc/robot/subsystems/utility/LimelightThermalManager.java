package frc.robot.subsystems.utility;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.function.BooleanSupplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SubsystemConstants.Vision;

/**
 * Limelight 4 thermal / power management.
 *
 * <p>WHAT THIS ACTUALLY DOES (read the long note in {@code Vision} constants for the measured
 * hardware reality): the LL4's dominant heat is its Hailo AI accelerator -- a FIXED ~3.75 W /
 * ~74 C load that robot code cannot throttle. So this manager only does the three things the
 * RIO genuinely controls:
 * <ol>
 *   <li>Forces the LEDs OFF every loop (passive AprilTags never use them -- the only camera
 *       power the RIO controls).</li>
 *   <li>Switches to a lower-res "idle" AprilTag pipeline when the shooter is inactive (and back
 *       to the full-res pipeline for shooting), shedding CPU/board heat while keeping low-res
 *       MegaTag2 pose fusion alive. Also forces idle when the Hailo crosses a HOT threshold
 *       (hysteresis), except never during an active shot.</li>
 *   <li>Reads hailoTemp/board temp off the main loop and WARNS the driver before a thermal
 *       event.</li>
 * </ol>
 *
 * <p>Registered as a {@link SubsystemBase} so the scheduler calls {@link #periodic()} every loop
 * with no manual wiring. Temperature is polled by a background {@link Notifier} (HTTP to the
 * camera's {@code /status}); the loop only ever reads cached values, so a slow/unreachable camera
 * never stalls the 20 ms robot loop.
 *
 * <p>Developed by Team 9704 with Claude Code.
 */
public class LimelightThermalManager extends SubsystemBase {

    private final String llName;
    private final BooleanSupplier shooterActive;
    private final URL statusUrl;
    private final Notifier statusPoller;

    // Extract the two numeric fields we need straight out of the /status JSON. The quotes make
    // "temp" match ONLY the board-temp key, not "hailoTemp" (which is a different substring).
    private static final Pattern HAILO_PAT = Pattern.compile("\"hailoTemp\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?)");
    private static final Pattern BOARD_PAT = Pattern.compile("\"temp\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?)");

    // Written by the poller thread, read by periodic() -- volatile is enough (single writer,
    // single reader, independent scalars). NaN = never read successfully yet.
    private volatile double hailoTempC = Double.NaN;
    private volatile double boardTempC = Double.NaN;
    private volatile double lastGoodReadSec = Double.NEGATIVE_INFINITY;

    // Main-thread state (periodic only).
    private boolean throttled = false;          // hysteresis latch: hot -> true, cool -> false
    private int lastCommandedPipe = -1;         // avoid re-sending the same pipeline every loop
    private double lastActiveSec = Double.NEGATIVE_INFINITY;
    private double lastWarnSec = Double.NEGATIVE_INFINITY;

    private final DoublePublisher hailoTempPub;
    private final DoublePublisher boardTempPub;
    private final DoublePublisher pipelinePub;
    private final BooleanPublisher throttledPub;
    private final BooleanPublisher tempValidPub;

    /**
     * @param limelightName NT/mDNS name of the camera (e.g. "limelight-knight").
     * @param shooterActive true while the flywheels are commanded to spin (needs the full-res
     *                      pipeline); false lets the camera drop to the idle pipeline.
     */
    public LimelightThermalManager(String limelightName, BooleanSupplier shooterActive) {
        this.llName = limelightName;
        this.shooterActive = shooterActive;

        URL url = null;
        try {
            url = new URL("http://" + limelightName + ".local:" + Vision.LL_STATUS_PORT + "/status");
        } catch (Exception e) {
            DriverStation.reportError("LimelightThermalManager: bad status URL for " + limelightName, false);
        }
        this.statusUrl = url;

        var table = NetworkTableInstance.getDefault().getTable("Limelight");
        hailoTempPub = table.getDoubleTopic("hailoTempC").publish();
        boardTempPub = table.getDoubleTopic("boardTempC").publish();
        pipelinePub  = table.getDoubleTopic("commandedPipeline").publish();
        throttledPub = table.getBooleanTopic("thermalThrottled").publish();
        tempValidPub = table.getBooleanTopic("tempReadValid").publish();

        if (statusUrl != null) {
            statusPoller = new Notifier(this::poll);
            statusPoller.setName("LL-thermal-poll");
            statusPoller.startPeriodic(Vision.LL_STATUS_POLL_SEC);
        } else {
            statusPoller = null;
        }
    }

    /** Background poll of the camera's /status (runs on the Notifier thread, never the loop). */
    private void poll() {
        HttpURLConnection conn = null;
        try {
            conn = (HttpURLConnection) statusUrl.openConnection();
            conn.setConnectTimeout((int) (Vision.LL_STATUS_TIMEOUT_SEC * 1000));
            conn.setReadTimeout((int) (Vision.LL_STATUS_TIMEOUT_SEC * 1000));
            conn.setRequestMethod("GET");
            if (conn.getResponseCode() != 200) {
                return;
            }
            StringBuilder sb = new StringBuilder();
            try (BufferedReader r = new BufferedReader(new InputStreamReader(conn.getInputStream()))) {
                String line;
                while ((line = r.readLine()) != null) {
                    sb.append(line);
                }
            }
            String body = sb.toString();
            double h = extract(HAILO_PAT, body);
            double b = extract(BOARD_PAT, body);
            if (!Double.isNaN(h)) {
                hailoTempC = h;
            }
            if (!Double.isNaN(b)) {
                boardTempC = b;
            }
            if (!Double.isNaN(h) || !Double.isNaN(b)) {
                lastGoodReadSec = Timer.getFPGATimestamp();
            }
        } catch (Exception e) {
            // Camera unreachable / timeout: keep the last cached values; periodic() gates on
            // staleness so a dead camera never leaves us acting on an old temperature.
        } finally {
            if (conn != null) {
                conn.disconnect();
            }
        }
    }

    private static double extract(Pattern p, String body) {
        Matcher m = p.matcher(body);
        if (m.find()) {
            try {
                return Double.parseDouble(m.group(1));
            } catch (NumberFormatException e) {
                // fall through
            }
        }
        return Double.NaN;
    }

    @Override
    public void periodic() {
        // LEDs OFF, always, re-asserted every loop so it survives a camera reboot. This is the
        // only Limelight power the RIO controls, and AprilTags never use the LEDs.
        LimelightHelpers.setLEDMode_ForceOff(llName);

        double now = Timer.getFPGATimestamp();
        boolean active = shooterActive.getAsBoolean();
        if (active) {
            lastActiveSec = now;
        }
        boolean recentlyActive = (now - lastActiveSec) <= Vision.LL_IDLE_HOLD_FULL_SEC;

        // Hailo temperature with staleness + plausibility gating: only trust a recent reading
        // inside a sane range, so a bad/missing read can never force the idle pipeline blind.
        double temp = hailoTempC;
        boolean tempValid = !Double.isNaN(temp)
                && temp >= Vision.LL_TEMP_SANE_MIN_C && temp <= Vision.LL_TEMP_SANE_MAX_C
                && (now - lastGoodReadSec) <= (Vision.LL_STATUS_POLL_SEC * 5);

        // Hysteresis latch on the throttle: engage at HOT, release at COOL, hold in between.
        throttled = nextThrottled(throttled, tempValid, temp,
                Vision.LL_TEMP_HOT_C, Vision.LL_TEMP_COOL_C);

        // Rate-limited driver warning.
        if (tempValid && temp >= Vision.LL_TEMP_WARN_C
                && (now - lastWarnSec) >= Vision.LL_WARN_INTERVAL_SEC) {
            DriverStation.reportWarning(String.format(
                    "Limelight Hailo temp %.0f C (>= %.0f C). Check camera fan/airflow.",
                    temp, Vision.LL_TEMP_WARN_C), false);
            lastWarnSec = now;
        }

        boolean wantFull = wantFullPipeline(active, throttled, recentlyActive);
        int targetPipe = wantFull ? Vision.LL_PIPELINE_NORMAL : Vision.LL_PIPELINE_IDLE;
        // If IDLE == NORMAL (idle switching disabled until pipeline 1 exists) this simply never
        // changes, so we never blind vision by switching to a pipeline that isn't set up.
        if (targetPipe != lastCommandedPipe) {
            LimelightHelpers.setPipelineIndex(llName, targetPipe);
            lastCommandedPipe = targetPipe;
        }

        hailoTempPub.set(Double.isNaN(hailoTempC) ? -1 : hailoTempC);
        boardTempPub.set(Double.isNaN(boardTempC) ? -1 : boardTempC);
        pipelinePub.set(targetPipe);
        throttledPub.set(throttled);
        tempValidPub.set(tempValid);
    }

    /**
     * Pure hysteresis + validity gate for the thermal-throttle latch (package-private for tests).
     * Engage at {@code hotC}, release at {@code coolC}, hold the previous state in the deadband
     * between them, and NEVER throttle on an untrustworthy reading.
     */
    static boolean nextThrottled(boolean prevThrottled, boolean tempValid, double tempC,
                                 double hotC, double coolC) {
        if (!tempValid) {
            return false; // no trustworthy temp -> never throttle blind
        }
        if (tempC >= hotC) {
            return true;
        }
        if (tempC <= coolC) {
            return false;
        }
        return prevThrottled; // deadband: hold
    }

    /**
     * Pure pipeline arbitration (package-private for tests):
     * <ul>
     *   <li>Shooting -> FULL res (best ranging). A thermal drop wouldn't cool the fixed Hailo
     *       anyway, so an active shot is never sacrificed to the throttle.</li>
     *   <li>Else too hot -> IDLE (throttle: sheds board/LED heat, keeps low-res pose fusion).</li>
     *   <li>Else recently active -> FULL (hold window, avoids churn between shots).</li>
     *   <li>Else -> IDLE (shooter-idle power saving).</li>
     * </ul>
     */
    static boolean wantFullPipeline(boolean shooterActive, boolean throttled, boolean recentlyActive) {
        if (shooterActive) {
            return true;
        }
        if (throttled) {
            return false;
        }
        return recentlyActive;
    }
}
