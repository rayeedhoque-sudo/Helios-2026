package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.misc.HopperSubsystem;
import frc.robot.subsystems.misc.IntakeSubsystem;
import frc.robot.subsystems.misc.ShooterSubsystem;

/**
 * Read-only power/temperature telemetry publisher for an optional driver companion dashboard.
 * Reads existing motor objects through getters added to each subsystem -- never writes to,
 * configures, or sends a control request to any motor. Runs on its own Notifier thread so it
 * costs nothing on the main robot loop, and the robot behaves identically whether or not any
 * dashboard is listening on NetworkTables.
 */
public class PowerTelemetry {

    // Companion dashboard refresh rate -- 10 Hz is plenty for a human-readable current/temp
    // readout and keeps this well off the control loop's own signal rates.
    private static final double UPDATE_PERIOD_SECONDS = 0.1; // s (10 Hz)

    // Monitored mechanism count: 8 swerve (4 modules x drive+steer) + 4 shooter flywheels
    // + 1 hood + 1 intake roller + 1 intake slider + 2 hopper belts = 17.
    private static final int NUM_MOTORS = 17;

    // Publish order/labels -- index-aligned with stator/supply/temp below.
    private static final String[] NAMES = {
        "DriveFL", "SteerFL", "DriveFR", "SteerFR", "DriveBL", "SteerBL", "DriveBR", "SteerBR",
        "ShooterA", "ShooterB", "ShooterC", "ShooterD", "Hood",
        "IntakeRoller", "IntakeSlider", "HopperA", "HopperB"
    };

    // TalonFX motors (13: 8 swerve + 4 shooter flywheels + 1 intake roller) and the NAMES
    // index each one publishes into.
    private final TalonFX[] talons;
    private final int[] talonSlots;

    // SparkMax motors (4: hood, intake slider, 2x hopper) and their NAMES indices.
    private final SparkMax[] sparks;
    private final int[] sparkSlots;

    // One StatusSignal per TalonFX per measurement, refreshed together each cycle.
    private final BaseStatusSignal[] statorSignals;
    private final BaseStatusSignal[] supplySignals;
    private final BaseStatusSignal[] tempSignals;
    private final BaseStatusSignal[] allSignals; // stator+supply+temp, for refreshAll()

    // Preallocated publish buffers, index-aligned with NAMES -- reused every cycle.
    private final double[] statorAmps = new double[NUM_MOTORS];
    private final double[] supplyAmps = new double[NUM_MOTORS];
    private final double[] tempCelsius = new double[NUM_MOTORS];

    private final StringArrayPublisher namesPub;
    private final DoubleArrayPublisher statorPub;
    private final DoubleArrayPublisher supplyPub;
    private final DoubleArrayPublisher tempPub;
    private final DoublePublisher voltagePub;

    private final Notifier notifier;

    /**
     * Wires up telemetry for the given already-constructed subsystems. Does not construct or
     * configure any motor -- only reads objects the subsystems already own via their getters.
     */
    public PowerTelemetry(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooterSS,
                           IntakeSubsystem intakeSS, HopperSubsystem hopperSS) {
        TalonFX[] flywheels = shooterSS.getFlywheelMotors();
        SparkMax[] hopperMotors = hopperSS.getHopperMotors();

        // Module order 0=FL,1=FR,2=BL,3=BR matches TunerConstants.createDrivetrain()'s
        // (FrontLeft, FrontRight, BackLeft, BackRight) module argument order.
        talons = new TalonFX[] {
            drivetrain.getModule(0).getDriveMotor(), drivetrain.getModule(0).getSteerMotor(),
            drivetrain.getModule(1).getDriveMotor(), drivetrain.getModule(1).getSteerMotor(),
            drivetrain.getModule(2).getDriveMotor(), drivetrain.getModule(2).getSteerMotor(),
            drivetrain.getModule(3).getDriveMotor(), drivetrain.getModule(3).getSteerMotor(),
            flywheels[0], flywheels[1], flywheels[2], flywheels[3],
            intakeSS.getRollerMotor()
        };
        talonSlots = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13 };

        sparks = new SparkMax[] {
            shooterSS.getHoodMotor(), intakeSS.getSliderMotor(), hopperMotors[0], hopperMotors[1]
        };
        sparkSlots = new int[] { 12, 14, 15, 16 };

        // Collect one StatusSignal per TalonFX per measurement (read-only -- StatusSignal has
        // no setter). setUpdateFrequencyForAll below only touches THESE signals, never the
        // position/velocity signals the swerve/shooter control loops depend on.
        statorSignals = new BaseStatusSignal[talons.length];
        supplySignals = new BaseStatusSignal[talons.length];
        tempSignals = new BaseStatusSignal[talons.length];
        for (int i = 0; i < talons.length; i++) {
            statorSignals[i] = talons[i].getStatorCurrent();
            supplySignals[i] = talons[i].getSupplyCurrent();
            tempSignals[i] = talons[i].getDeviceTemp();
        }
        allSignals = new BaseStatusSignal[talons.length * 3];
        System.arraycopy(statorSignals, 0, allSignals, 0, talons.length);
        System.arraycopy(supplySignals, 0, allSignals, talons.length, talons.length);
        System.arraycopy(tempSignals, 0, allSignals, talons.length * 2, talons.length);
        BaseStatusSignal.setUpdateFrequencyForAll(1.0 / UPDATE_PERIOD_SECONDS, allSignals);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("CompanionTelemetry");
        namesPub = table.getStringArrayTopic("names").publish();
        statorPub = table.getDoubleArrayTopic("stator").publish();
        supplyPub = table.getDoubleArrayTopic("supply").publish();
        tempPub = table.getDoubleArrayTopic("temp").publish();
        voltagePub = table.getDoubleTopic("voltage").publish();

        // Static labels -- published once, here.
        namesPub.set(NAMES);

        notifier = new Notifier(this::update);
        notifier.setName("PowerTelemetry");
        notifier.startPeriodic(UPDATE_PERIOD_SECONDS);
    }

    /** Refresh all TalonFX signals, read the SparkMaxes, and publish -- runs on the Notifier thread. */
    private void update() {
        BaseStatusSignal.refreshAll(allSignals);

        for (int i = 0; i < talons.length; i++) {
            int slot = talonSlots[i];
            statorAmps[slot] = statorSignals[i].getValueAsDouble();
            supplyAmps[slot] = supplySignals[i].getValueAsDouble();
            tempCelsius[slot] = tempSignals[i].getValueAsDouble();
        }
        for (int i = 0; i < sparks.length; i++) {
            int slot = sparkSlots[i];
            statorAmps[slot] = sparks[i].getOutputCurrent();
            supplyAmps[slot] = Double.NaN; // SparkMax has no supply-current measurement
            tempCelsius[slot] = sparks[i].getMotorTemperature();
        }

        statorPub.set(statorAmps);
        supplyPub.set(supplyAmps);
        tempPub.set(tempCelsius);
        voltagePub.set(RobotController.getBatteryVoltage());
    }
}
