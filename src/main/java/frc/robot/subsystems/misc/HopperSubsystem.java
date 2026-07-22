package frc.robot.subsystems.misc;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;

public class HopperSubsystem extends SubsystemBase{

    //Hopper Motors
        private final SparkMax hopperMotorA = new SparkMax( HopperSubsystemConstants.HOPPER_ID_A, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        private final SparkMax hopperMotorB = new SparkMax( HopperSubsystemConstants.HOPPER_ID_B, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //Index Motor
        // CAN 17 is a VICTOR SPX (team CAN chart 2026-07-08, staying this season), NOT the
        // Kraken X60 the CAD showed. Victor SPX = Phoenix 5, brushed motors only.
        private final VictorSPX kickerMotor = new VictorSPX(HopperSubsystemConstants.KICKER_MOTOR_ID);

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
            hopperConfig.smartCurrentLimit(20);
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
            // Kicker: Victor SPX driving a CIM (motor swapped 2026-07-16). WARNING - the
            // Victor SPX has NO current sensing, so a current limit is impossible in
            // software; the breaker is the only stall protection (CIM stall ~131 A). Ramp
            // softens inrush. TODO: confirm the kicker breaker size.
            kickerMotor.configFactoryDefault();
            kickerMotor.setNeutralMode(NeutralMode.Coast);
            kickerMotor.configOpenloopRamp(0.25); // seconds from 0 to full output

        //Data
            hopperAOutputEntry = HopperSubsystemTab.add("Hopper A Applied Output", 0.0).getEntry();
            hopperBOutputEntry = HopperSubsystemTab.add("Hopper B Applied Output", 0.0).getEntry();
            hopperACurrentEntry = HopperSubsystemTab.add("Hopper A Current (A)", 0.0).getEntry();
            hopperBCurrentEntry = HopperSubsystemTab.add("Hopper B Current (A)", 0.0).getEntry();
            hopperABusVEntry = HopperSubsystemTab.add("Hopper A Bus Voltage (V)", 0.0).getEntry();
            hopperBBusVEntry = HopperSubsystemTab.add("Hopper B Bus Voltage (V)", 0.0).getEntry();
    }

    /**
     * UNJAM: reverse both belts AND the kicker while held, to back a jammed ball out.
     * (Commands own the motors outright now -- the old periodic state machine and its
     * manualOverride flag are gone, 2026-07-16 team-spec rewrite.)
     */
    public Command unjamCommand(){
        return runEnd(
            () -> {
                hopperMotorA.set(-HopperSubsystemConstants.UNJAM_SPEED);
                hopperMotorB.set(-HopperSubsystemConstants.UNJAM_SPEED);
                kickerMotor.set(ControlMode.PercentOutput, -HopperSubsystemConstants.UNJAM_SPEED);
            },
            () -> {
                stopIndex();
                stopKickFuel();
            });
    }

    /**
     * KICKER TEST: run the kicker alone at a slow duty, bypassing the at-speed gate.
     * Verified the swapped CIM/Victor works (2026-07-16). UNBOUND utility -- bind it
     * anywhere if the kicker needs isolating again.
     */
    public Command kickerTestCommand(){
        return runEnd(
            () -> kickerMotor.set(ControlMode.PercentOutput, HopperSubsystemConstants.KICKER_TEST_SPEED),
            () -> stopKickFuel());
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
            () -> motor.set(HopperSubsystemConstants.DIRECTION_TEST_SPEED),
            () -> stopIndex());
    }

    /**
     * MANUAL RUN (B button, hold): belts + kicker together, UNGATED -- the kicker does
     * not wait for flywheels-at-speed, so this works even with the shooter idle.
     * Release = stop everything.
     */
    public Command manualRunCommand(){
        return runEnd(
            () -> {
                indexFuel();
                kickFuel();
            },
            () -> {
                stopIndex();
                stopKickFuel();
            });
    }

    /**
     * SHOT FEED (RT/RB, hold): belts and kicker follow their own live gates EVERY loop --
     * belts run at HOPPER_SPEED while beltsOn (a valid shot is active), kicker runs at
     * INDEXER_SPEED only while kickerOn (flywheels at speed AND hood at angle, via
     * ShooterSubsystem.isReadyToShoot()). A gate going false stops that motor the same
     * loop, so fuel is never kicked into flywheels that aren't ready. End = stop both.
     */
    public Command feedShooterCommand(BooleanSupplier beltsOn, BooleanSupplier kickerOn){
        return runEnd(
            () -> {
                if (beltsOn.getAsBoolean()) { indexFuel(); } else { stopIndex(); }
                if (kickerOn.getAsBoolean()) { kickFuel(); } else { stopKickFuel(); }
            },
            () -> {
                stopIndex();
                stopKickFuel();
            });
    }

    /**
     * INTAKE FEED (LT/Y, runs only while the intake rollers are rolling -- never while the
     * slider moves): belts at normal duty, KICKER OFF (team spec 2026-07-18: "during intake
     * only the hopper should run, not the kicker"). Keeping the kicker stopped also removes
     * the old stopped-flywheel CIM-stall exposure this command carried even at low duty.
     * End = stop the belts (and re-assert the kicker stopped, belt-and-suspenders).
     */
    public Command intakeFeedCommand(){
        return runEnd(
            () -> indexFuel(),
            () -> {
                stopIndex();
                stopKickFuel();
            });
    }

    public void indexFuel(){
        hopperMotorA.set(HopperSubsystemConstants.HOPPER_SPEED);
        hopperMotorB.set(HopperSubsystemConstants.HOPPER_SPEED);
    }

    public void kickFuel(){
        kickerMotor.set(ControlMode.PercentOutput, HopperSubsystemConstants.INDEXER_SPEED);
    }
    
    public void stopIndex(){
        hopperMotorA.set(0);
        hopperMotorB.set(0);
    }

    public void stopKickFuel(){
        kickerMotor.set(ControlMode.PercentOutput, 0);
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
