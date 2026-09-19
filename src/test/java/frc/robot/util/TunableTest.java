package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Pins the two properties the live-tuning path depends on: the TUNING_MODE gate really cuts
 * every wire, and the robot never fights the dashboard for ownership of a gain.
 */
class TunableTest {
    private NetworkTableInstance nt;

    @BeforeEach
    void setUp() {
        // Tunable talks to the DEFAULT instance; give it a clean one with no server/client.
        nt = NetworkTableInstance.getDefault();
        for (var topic : nt.getTable("Tuning").getTopics()) {
            topic.getGenericEntry().unpublish();
        }
        Tunable.resetForTest();
    }

    @AfterEach
    void tearDown() {
        Tunable.setEnabledForTest(false);
        Tunable.resetForTest();
    }

    private DoublePublisher pub(String name) {
        return nt.getTable("Tuning").getDoubleTopic(name).publish();
    }

    @Test
    void gateOffMeansTheConstantAndNothingOnTheWire() {
        Tunable.setEnabledForTest(false);
        Tunable t = new Tunable("Off/kP", 0.4);

        // A dashboard write must not reach the gain when tuning mode is off.
        try (DoublePublisher p = pub("Off/kP")) {
            p.set(99.0);
            nt.flushLocal();
            assertEquals(0.4, t.get(), 0.0);
            assertFalse(t.hasChanged());
        }
        // publishKeys() is a no-op, so the panel shows "tuning mode OFF".
        Tunable.publishKeys();
        assertFalse(nt.getTable("Tuning").getTopic("keys").exists());
    }

    @Test
    void gateOnAppliesDashboardWrites() {
        Tunable.setEnabledForTest(true);
        Tunable t = new Tunable("On/kP", 0.4);
        assertEquals(0.4, t.get(), 0.0);
        assertFalse(t.hasChanged(), "no write yet");

        try (DoublePublisher p = pub("On/kP")) {
            p.set(0.8);
            nt.flushLocal();
            assertEquals(0.8, t.get(), 0.0);
            assertTrue(t.hasChanged(), "fires once for a new value");
            assertFalse(t.hasChanged(), "and only once -- this gates the CAN re-apply");

            p.set(0.9);
            nt.flushLocal();
            assertTrue(t.hasChanged());
            assertFalse(t.hasChanged());
        }
    }

    /**
     * The revert bug this whole design exists to prevent: a gain already set by the dashboard
     * must survive the robot constructing its Tunable. setDefault() only writes an unset topic.
     */
    @Test
    void seedingDoesNotStompAValueTheDashboardAlreadySet() {
        Tunable.setEnabledForTest(true);
        try (DoublePublisher p = pub("Seed/kP")) {
            p.set(0.8);
            nt.flushLocal();

            Tunable t = new Tunable("Seed/kP", 0.4); // constructed AFTER the dashboard wrote
            assertEquals(0.8, t.get(), 0.0);
        }
    }

    /**
     * Two gains in one group changing in the same loop must BOTH be seen. A short-circuiting
     * `a.hasChanged() || b.hasChanged()` at the call site would leave b unpolled.
     */
    @Test
    void everyGainInAGroupIsPolledWhenSeveralChangeAtOnce() {
        Tunable.setEnabledForTest(true);
        Tunable a = new Tunable("Group/kP", 1.0);
        Tunable b = new Tunable("Group/kD", 2.0);

        try (DoublePublisher pa = pub("Group/kP"); DoublePublisher pb = pub("Group/kD")) {
            pa.set(1.5);
            pb.set(2.5);
            nt.flushLocal();

            boolean aChanged = a.hasChanged();
            boolean bChanged = b.hasChanged();
            assertTrue(aChanged && bChanged);
            assertEquals(1.5, a.get(), 0.0);
            assertEquals(2.5, b.get(), 0.0);
        }
    }

    /** A NaN/Inf gain would poison the motor controller AND fire hasChanged() forever. */
    @Test
    void nonFiniteWritesFallBackToTheConstant() {
        Tunable.setEnabledForTest(true);
        Tunable t = new Tunable("Nan/kP", 0.4);

        try (DoublePublisher p = pub("Nan/kP")) {
            p.set(Double.NaN);
            nt.flushLocal();
            assertEquals(0.4, t.get(), 0.0);
            assertFalse(t.hasChanged(), "must not re-apply every loop");

            p.set(Double.POSITIVE_INFINITY);
            nt.flushLocal();
            assertEquals(0.4, t.get(), 0.0);
        }
    }

    @Test
    void publishKeysEnumeratesRegisteredGains() {
        Tunable.setEnabledForTest(true);
        new Tunable("Keys/kP", 0.4);
        new Tunable("Keys/kD", 0.01);
        Tunable.publishReadOnly("Swerve/Steer/kP", 50.0);
        Tunable.publishKeys();
        nt.flushLocal();

        String[] keys = nt.getTable("Tuning").getStringArrayTopic("keys").subscribe(new String[0]).get();
        double[] defaults = nt.getTable("Tuning").getDoubleArrayTopic("defaults").subscribe(new double[0]).get();
        String[] roKeys = nt.getTable("Tuning").getStringArrayTopic("ro/keys").subscribe(new String[0]).get();

        assertEquals(2, keys.length);
        assertEquals("Keys/kP", keys[0]);
        assertEquals(0.4, defaults[0], 0.0);
        assertEquals(1, roKeys.length);
        assertEquals("Swerve/Steer/kP", roKeys[0]);
    }
}
