package frc.robot.subsystems.misc;

import java.util.function.BooleanSupplier;

import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;

public class HopperSubsystem extends SubsystemBase{

    //Hopper Motors
        private final SparkMax hopperMotorA = new SparkMax( HopperSubsystemConstants.HOPPER_ID_A, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        private final SparkMax hopperMotorB = new SparkMax( HopperSubsystemConstants.HOPPER_ID_B, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    // (Kicker REMOVED 2026-09-30: the V1 Victor SPX + CIM at CAN 17 is gone in the V2 redesign.
    // Its job is now the launcher's feeder X60 -- see LauncherSubsystem.)

    //Data
        private ShuffleboardTab HopperSubsystemTab = Shuffleboard.getTab("Hopper Subsystem Tab");

        // Live hopper-motor diagnostics (2026-07-18). Applied output = what the controller is
        // ACTUALLY commanding (~0.5 while B held if the command lands; 0 if the SparkMax is
        // ignoring .set()). Current = whether the motor is drawing. Read them while holding B:
        //   output ~0.5 + current ~0 + no spin  -> wire after the controller / motor decoupled
        //   output 0    while B held            -> command not applied (config/CAN), NOT the belts
        private GenericEntry hopperAOutputEntry;
        private GenericEntry hopperBOutputEntry;
        private GenericEntry hopperACurrentEntry;
        private GenericEntry hopperBCurrentEntry;
        // Bus voltage = liveness probe: a powered, on-bus SparkMax reports ~12 V; a controller
        // that lost CAN or power sticks at 0. Distinguishes "code not commanding" from
        // "controller not on the bus" without touching the robot.
        private GenericEntry hopperABusVEntry;
        private GenericEntry hopperBBusVEntry;

    // (Beam-break "fuel detected" display DELETED 2026-07-21: Sensors was a stub hardcoding
    // true, so the dashboard lied. DIO 8/9 assignments live in the data sheet + LightSensor
    // constants; construct real DigitalInputs only after the team confirms wiring/polarity.)

    public HopperSubsystem(){
        // Conservative-start current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
            // Hopper A/B: NEO 2.0 - 20A smart limit, Coast.
            // disableFollowerMode(): both belts are driven by explicit .set() (software lockstep,
            // NOT a hardware follower). A SparkMax left in follower mode from a stale flash IGNORES
            // .set() and silently sits idle -- the "hopper dead on B and RB while the kicker runs"
            // symptom (2026-07-18). The two motors sit in different positions of a COMMON gearbox
            // (Hardware-Data-Sheet: ~12:1 to rollers, "only direction differs"), so they need
            // OPPOSITE inversion to drive that shared output together -- A=false, B=true, set
            // EXPLICITLY so a stale-flashed value can't leave them fighting.
            //
            // RESET MODE (2026-07-18): kResetSafeParameters, NOT kNoReset. The belts stayed dead
            // even after disableFollowerMode() was added, and the leading remaining software cause
            // is a stale flash (e.g. a stuck follower/inversion) that a NON-resetting apply left in
            // place. A full factory reset THEN re-applying every field the belts need
            // (smartCurrentLimit + Coast + disableFollowerMode + inverted) guarantees a clean,
            // deterministic state -- nothing critical for a plain belt motor is set only via the
            // REV Hardware Client, so wiping non-config params is safe here.
            // TODO on robot: hold B. If the belts now feed UP toward the shooter -> done. If they
            // run BACKWARD -> swap the two booleans (A=true, B=false), same together-direction,
            // reversed. If they are STILL dead -> it is NOT software: read "Hopper A/B Applied
            // Output" + "Current" on the Hopper tab (output ~0 / current ~0 = command not reaching
            // the motor -> check CAN 20/21 wiring + power; output ~0.5 / current high = fighting or
            // jammed).
            SparkMaxConfig hopperConfig = new SparkMaxConfig();
            hopperConfig.smartCurrentLimit(HopperSubsystemConstants.HOPPER_CURRENT_LIMIT_A);
            hopperConfig.idleMode(IdleMode.kCoast);
            // 0.25 s open-loop ramp, same inrush softening every sibling motor already has
            // (kicker, intake roller/slider) -- the belts were the one unramped pair
            // (power-limiting-review nit). Nothing gates on belt spin-up time.
            hopperConfig.openLoopRampRate(0.25);
            hopperConfig.disableFollowerMode();
            // INVERSION: SAME sign for BOTH motors -- measured on-robot 2026-07-18. With A=false/
            // B=true (the "opposite inversion" the Hardware-Data-Sheet note suggested) the pair
            // hard-stalled at the 20 A limit whenever driven together, while motor B ALONE spun
            // the belts freely at 0.1 A: the gearbox geometry already handles the direction
            // difference between the two mounting positions, so opposite CODE inversion makes the
            // motors fight. This matches the original 2026-07-14 direction test ("positive output
            // moves BOTH belts the same way"). If the belts feed BACKWARD (away from the shooter),
            // flip BOTH booleans together -- never just one.
            hopperConfig.inverted(false);
            REVLibError hopperACfg = hopperMotorA.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            REVLibError hopperBCfg = hopperMotorB.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // Surface configure failures LOUDLY (2026-07-18): if configure() fails (CAN timeout =
            // controller unpowered or off the bus), NONE of the above -- reset, follower-disable,
            // inversion, current limit -- ever reached the controller, and the belts stay dead no
            // matter what the code commands. This is the prime "belts dead, kicker (CAN 17) fine"
            // hardware signature: a CAN/power break to 20/21, e.g. from the 07-16 kicker-swap work.
            if (hopperACfg != REVLibError.kOk) {
                DriverStation.reportError("Hopper A (CAN 20) configure FAILED: " + hopperACfg + " -- check CAN/power to the controller", false);
            }
            if (hopperBCfg != REVLibError.kOk) {
                DriverStation.reportError("Hopper B (CAN 21) configure FAILED: " + hopperBCfg + " -- check CAN/power to the controller", false);
            }
        //Data
            hopperAOutputEntry = HopperSubsystemTab.add("Hopper A Applied Output", 0.0).getEntry();
            hopperBOutputEntry = HopperSubsystemTab.add("Hopper B Applied Output", 0.0).getEntry();
            hopperACurrentEntry = HopperSubsystemTab.add("Hopper A Current (A)", 0.0).getEntry();
            hopperBCurrentEntry = HopperSubsystemTab.add("Hopper B Current (A)", 0.0).getEntry();
            hopperABusVEntry = HopperSubsystemTab.add("Hopper A Bus Voltage (V)", 0.0).getEntry();
            hopperBBusVEntry = HopperSubsystemTab.add("Hopper B Bus Voltage (V)", 0.0).getEntry();
    }

    /**
     * KILL SWITCH for the whole hopper (added 2026-08-29). false = NO hopper motor is ever
     * driven: the belts are commanded 0 no matter which command is running, in teleop,
     * in test mode and in autonomous alike. The bindings all still exist and the commands all
     * still schedule -- they simply move nothing -- so this is one word either way and cannot
     * leave a mechanism half-wired.
     *
     * CURRENTLY TRUE: the hopper runs normally (disabled briefly on 2026-08-29, re-enabled the
     * same day by team request).
     *
     * What false costs, so it is not a surprise on the field: nothing feeds the shooter. Every
     * shot binding still spins the launcher, and no fuel arrives.
     * The intake still collects, but nothing indexes it. Unjam does nothing either.
     */
    private static final boolean HOPPER_ENABLED = true;

    /** Drive both belt motors, or 0 when the hopper is disabled. Every belt write goes here. */
    private void setBelts(double duty){
        hopperMotorA.set(HOPPER_ENABLED ? duty : 0);
        hopperMotorB.set(HOPPER_ENABLED ? duty : 0);
    }

    /**
     * UNJAM: reverse both belts while held, to back a jammed ball out. (The VIEW binding
     * pairs this with LauncherSubsystem.feederUnjamCommand + the intake slider cycle.)
     */
    public Command unjamCommand(){
        return runEnd(
            () -> setBelts(-HopperSubsystemConstants.UNJAM_SPEED),
            () -> stopIndex());
    }

    /**
     * DIRECTION TEST (diagnostic, 2026-07-18): run ONE belt motor alone at 5% duty while
     * held; the other motor stays in coast and back-drives through the shared gearbox.
     * Purpose: distinguish the two stall causes seen live (both motors pinned at the 20 A
     * smart limit at ~6% applied output while B is held): if ONE motor alone spins the
     * belts freely, the two motors are FIGHTING (flip the relative inversion); if even one
     * motor alone stalls, the gearbox/belts are mechanically jammed. At 5% duty a stall
     * stays well under the 20 A limit -- safe to hold and observe.
     */
    public Command directionTest(boolean motorA){
        SparkMax motor = motorA ? hopperMotorA : hopperMotorB;
        return runEnd(
            () -> motor.set(HOPPER_ENABLED ? HopperSubsystemConstants.DIRECTION_TEST_SPEED : 0),
            () -> stopIndex());
    }

    /**
     * MANUAL RUN (A button, hold): belts ONLY -- the launcher feeder stays off.
     * Release = stop the belts.
     */
    public Command manualRunCommand(){
        return runEnd(
            () -> indexFuel(),
            () -> {
                stopIndex();
                setBeltCurrentLimit(HopperSubsystemConstants.HOPPER_CURRENT_LIMIT_A);
            })
            // Belts get 30 A while B is held (team request 2026-08-27), restored in end()
            // above -- same raise/restore pattern the shot feed already uses.
            .beforeStarting(() -> setBeltCurrentLimit(HopperSubsystemConstants.HOPPER_MANUAL_CURRENT_LIMIT_A));
    }

    // Shot start reverse (team request 2026-10-05): restarted on every RT press.
    private final Timer shotReverseTimer = new Timer();

    /**
     * SHOT FEED (RT, hold): belts run at HOPPER_SHOT_SPEED only while beltsOn -- the launcher's
     * flywheels AND feeder at speed (team spec 2026-09-30) -- re-checked every loop, so a dip
     * from a ball passing through pauses the belts until both recover. End = stop.
     * NOTE (V2): the hopper motors/gearbox are unchanged but the belt-floor hopper may run
     * faster or slower at these duties than the V1 rollers did -- re-check throughput.
     */
    public Command feedShooterCommand(BooleanSupplier beltsOn){
        return runEnd(
            () -> {
                // First HOPPER_SHOT_REVERSE_SEC of every shot: belts BACKWARD at the unjam duty,
                // pulling fuel away from the feeder while the wheels spin up. Then the normal
                // at-speed-gated feed.
                if (!shotReverseTimer.hasElapsed(HopperSubsystemConstants.HOPPER_SHOT_REVERSE_SEC)) {
                    setBelts(-HopperSubsystemConstants.UNJAM_SPEED);
                } else if (beltsOn.getAsBoolean()) { indexFuelForShot(); } else { stopIndex(); }
            },
            () -> {
                stopIndex();
                setBeltCurrentLimit(HopperSubsystemConstants.HOPPER_CURRENT_LIMIT_A);
            })
            // Raise the belt limit 20% for the shot only (team request 2026-08-22), restored
            // in end() above -- runs once per press, not every loop.
            .beforeStarting(() -> {
                setBeltCurrentLimit(HopperSubsystemConstants.HOPPER_SHOT_CURRENT_LIMIT_A);
                shotReverseTimer.restart();
            });
    }

    /**
     * Re-apply just the smart current limit to both belt motors, leaving every other
     * configured field (idle mode, ramp, inversion, follower-disable) untouched:
     * kNoResetSafeParameters + kNoPersistParameters = a live tweak, nothing flashed.
     */
    private void setBeltCurrentLimit(int amps){
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.smartCurrentLimit(amps);
        hopperMotorA.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        hopperMotorB.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * INTAKE FEED (LT/Y, runs only while the intake rollers are rolling -- never while the
     * slider moves): belts at HOPPER_INTAKE_SPEED (half the normal duty). End = stop the belts.
     */
    public Command intakeFeedCommand(){
        return runEnd(
            () -> setBelts(HopperSubsystemConstants.HOPPER_INTAKE_SPEED),
            () -> stopIndex());
    }

    public void indexFuel(){
        setBelts(HopperSubsystemConstants.HOPPER_SPEED);
    }

    /**
     * Belts at the SHOT duty (HOPPER_SHOT_SPEED) rather than the normal one -- used by
     * feedShooterCommand only, so the RT shot feeds faster while the LT intake feed and the
     * A manual run are untouched (team request 2026-08-29).
     */
    public void indexFuelForShot(){
        setBelts(HopperSubsystemConstants.HOPPER_SHOT_SPEED);
    }

    public void stopIndex(){
        setBelts(0);
    }

    @Override
    public void periodic(){
        // Telemetry only -- motor control lives entirely in the command factories now
        // (2026-07-16 team-spec rewrite deleted the HOPPERSTATE machine).
            // Hopper motor diagnostics -- getAppliedOutput()/getOutputCurrent() are the same
            // read-only signals PowerTelemetry already uses (no control side effect).
            hopperAOutputEntry.setDouble(hopperMotorA.getAppliedOutput());
            hopperBOutputEntry.setDouble(hopperMotorB.getAppliedOutput());
            hopperACurrentEntry.setDouble(hopperMotorA.getOutputCurrent());
            hopperBCurrentEntry.setDouble(hopperMotorB.getOutputCurrent());
            hopperABusVEntry.setDouble(hopperMotorA.getBusVoltage());
            hopperBBusVEntry.setDouble(hopperMotorB.getBusVoltage());
    }

    // Read-only motor access for PowerTelemetry (no control).
    public SparkMax[] getHopperMotors(){
        return new SparkMax[] { hopperMotorA, hopperMotorB };
    }
}
