package frc.robot.subsystems.misc;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;
import frc.robot.subsystems.utility.Sensors;

public class HopperSubsystem extends SubsystemBase{

    //Hopper Motors
        private final SparkMax hopperMotorA = new SparkMax( HopperSubsystemConstants.HOPPER_ID_A, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        private final SparkMax hopperMotorB = new SparkMax( HopperSubsystemConstants.HOPPER_ID_B, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //Index Motor
        // CAN 17 is a VICTOR SPX (team CAN chart 2026-07-08, staying this season), NOT the
        // Kraken X60 the CAD showed. Victor SPX = Phoenix 5, brushed motors only.
        private final VictorSPX kickerMotor = new VictorSPX(HopperSubsystemConstants.KICKER_MOTOR_ID);

    //Tracker Variables
        private boolean fuelDetectedIndexer;

    //Data
        private ShuffleboardTab HopperSubsystemTab = Shuffleboard.getTab("Hopper Subsystem Tab");
        private GenericEntry fuelDetectedIndexerEntry;
    
    //Sensors 
        Sensors sensors = new Sensors();
    
    public HopperSubsystem(){
        // Conservative-start current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
            // Hopper A/B: NEO 2.0 - 20A smart limit, Coast. kNoResetSafeParameters preserves the
            // flashed inversion, which the 2026-07-14 direction test verified: positive output
            // moves BOTH belts the same way, so no relative inversion is set in code.
            SparkMaxConfig hopperConfig = new SparkMaxConfig();
            hopperConfig.smartCurrentLimit(20);
            hopperConfig.idleMode(IdleMode.kCoast);
            hopperMotorA.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            hopperMotorB.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            // Kicker: Victor SPX driving a CIM (motor swapped 2026-07-16). WARNING - the
            // Victor SPX has NO current sensing, so a current limit is impossible in
            // software; the breaker is the only stall protection (CIM stall ~131 A). Ramp
            // softens inrush. TODO: confirm the kicker breaker size.
            kickerMotor.configFactoryDefault();
            kickerMotor.setNeutralMode(NeutralMode.Coast);
            kickerMotor.configOpenloopRamp(0.25); // seconds from 0 to full output

        //Tracker Variables
            fuelDetectedIndexer = false;

        //Data
            fuelDetectedIndexerEntry = HopperSubsystemTab.add("Fuel Detected Indexer", false).getEntry();
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
     * INTAKE FEED (LT, runs only while the intake rollers are rolling -- never while the
     * slider moves): belts at normal duty, kicker at the LOW intake duty. During intake
     * the flywheels are STOPPED, so fuel pressed against them can stall the kicker CIM --
     * and the Victor SPX has no current sensing, so the low duty (a software current
     * limit is impossible here) is what keeps a momentary stall breaker-survivable.
     * End = stop both.
     */
    public Command intakeFeedCommand(){
        return runEnd(
            () -> {
                indexFuel();
                kickerMotor.set(ControlMode.PercentOutput, HopperSubsystemConstants.INTAKE_KICKER_SPEED);
            },
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
            fuelDetectedIndexer = sensors.getIndexSensorA() && sensors.getIndexSensorB();
            fuelDetectedIndexerEntry.setBoolean(fuelDetectedIndexer);
    }

    // Read-only motor access for PowerTelemetry (no control).
    public SparkMax[] getHopperMotors(){
        return new SparkMax[] { hopperMotorA, hopperMotorB };
    }
}
