package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;

import frc.robot.Constants.SubsystemConstants;

/**
 * A single gain that the driver-companion app can change live over NetworkTables.
 *
 * <p>Ownership rule, and the whole reason this class exists: the ROBOT seeds the topic exactly
 * once (setDefault, which only writes if nothing has ever set the topic) and thereafter only
 * READS it. The dashboard is the sole writer. The old Shuffleboard gain boxes in
 * ShooterSubsystem had both sides writing the same entry every loop, so a typed value survived
 * only if it landed in the gap between the echo and the read -- tuning appeared to work, then
 * silently reverted.
 *
 * <p>Values are deliberately VOLATILE. They live only in the roboRIO's NT server, so a robot
 * reboot or redeploy re-seeds every gain from Constants. Use the companion app's "Copy as Java"
 * button to bake a good session's numbers back into SubsystemConstants.
 *
 * <p>When {@link SubsystemConstants#TUNING_MODE} is false (the competition default) nothing is
 * published, nothing is subscribed, and {@link #get()} returns the compiled constant verbatim.
 */
public class Tunable {
    private static final String TABLE_NAME = "Tuning";

    // Only populated while tuning mode is on -- so publishKeys() naturally publishes nothing
    // when the gate is off.
    private static final List<Tunable> ALL = new ArrayList<>();
    private static final List<String> RO_KEYS = new ArrayList<>();
    // Publishers must outlive the call that creates them: a closed/collected publisher drops
    // its value off the server.
    private static final List<DoublePublisher> RO_PUBS = new ArrayList<>();
    private static StringArrayPublisher keysPub;
    private static DoubleArrayPublisher defaultsPub;
    private static StringArrayPublisher roKeysPub;

    private static boolean enabled = SubsystemConstants.TUNING_MODE;

    private final String name;
    private final double defaultValue;
    /** null when tuning mode is off. */
    private final DoubleEntry entry;
    private double last;

    /**
     * @param name  slash-grouped key, e.g. "Shooter/Flywheel/kP" -- the companion app groups
     *              rows on everything before the last slash.
     * @param defaultValue the compiled constant this gain falls back to.
     */
    public Tunable(String name, double defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
        this.last = defaultValue;
        if (enabled) {
            entry = table().getDoubleTopic(name).getEntry(defaultValue);
            entry.setDefault(defaultValue); // seeds ONLY if the topic has no value yet
            ALL.add(this);
        } else {
            entry = null;
        }
    }

    /** Current value: the dashboard's if tuning is on and it wrote one, else the constant. */
    public double get() {
        if (entry == null) return defaultValue;
        double v = entry.get(defaultValue);
        // Anything can write this topic. A NaN/Inf gain would poison a motor controller AND
        // make hasChanged() fire every loop (NaN != NaN), so never let one through.
        return Double.isFinite(v) ? v : defaultValue;
    }

    /**
     * True once per distinct value -- gate every re-apply on this. A getConfigurator().apply()
     * every 20 ms floods the CAN bus.
     *
     * <p>Callers with several gains in one group must evaluate them ALL into locals and then OR:
     * a short-circuiting {@code a.hasChanged() || b.hasChanged()} leaves b unpolled, and b's
     * change is then consumed-but-unapplied on the next pass.
     */
    public boolean hasChanged() {
        double v = get();
        if (v == last) return false;
        last = v;
        return true;
    }

    public String name() {
        return name;
    }

    public double defaultValue() {
        return defaultValue;
    }

    /**
     * Publish the key/default lists the companion app enumerates. Call once after all
     * subsystems are constructed. Because the UI is driven off these lists, adding a gain on
     * the robot needs no app change.
     */
    public static void publishKeys() {
        if (!enabled) return;
        NetworkTable t = table();
        if (keysPub == null) {
            keysPub = t.getStringArrayTopic("keys").publish();
            defaultsPub = t.getDoubleArrayTopic("defaults").publish();
            roKeysPub = t.getStringArrayTopic("ro/keys").publish();
        }
        String[] names = new String[ALL.size()];
        double[] defaults = new double[ALL.size()];
        for (int i = 0; i < ALL.size(); i++) {
            names[i] = ALL.get(i).name;
            defaults[i] = ALL.get(i).defaultValue;
        }
        keysPub.set(names);
        defaultsPub.set(defaults);
        roKeysPub.set(RO_KEYS.toArray(new String[0]));
    }

    /**
     * Publish a gain the app shows but must NOT write -- swerve steer/drive (Tuner X owns them,
     * and a bad steer kP is a violent mechanism) and the PathPlanner constants (baked into the
     * holonomic controller at configure time). Changing these still needs a redeploy.
     */
    public static void publishReadOnly(String name, double value) {
        if (!enabled) return;
        DoublePublisher p = table().getDoubleTopic("ro/" + name).publish();
        p.set(value);
        RO_PUBS.add(p);
        RO_KEYS.add(name);
    }

    private static NetworkTable table() {
        return NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    }

    /**
     * Test-only: the real gate is the compile-time SubsystemConstants.TUNING_MODE.
     *
     * <p>NEVER call this from robot code. The gate's safety claim -- that with TUNING_MODE false
     * there is no path from a dashboard value to a motor controller -- holds because nothing in
     * frc.robot flips this. Package-private so it can only be reached from frc.robot.util.
     */
    static void setEnabledForTest(boolean e) {
        enabled = e;
    }

    /** Test-only: drop registrations so each test starts clean. */
    static void resetForTest() {
        ALL.clear();
        RO_KEYS.clear();
        RO_PUBS.clear();
    }
}
