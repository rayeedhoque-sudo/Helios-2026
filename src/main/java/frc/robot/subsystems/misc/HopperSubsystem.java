package frc.robot.subsystems.misc;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;
import frc.robot.subsystems.utility.Sensors;

public class HopperSubsystem extends SubsystemBase{

    //Hopper Motors
        private final SparkMax hopperMotorA = new SparkMax( HopperSubsystemConstants.HOPPER_ID_A, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        private final SparkMax hopperMotorB = new SparkMax( HopperSubsystemConstants.HOPPER_ID_B, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //Index Motor
        // CAN 17 is a VICTOR SPX (team CAN chart 2026-07-08, staying this season), NOT the
        // Kraken X60 the CAD showed. Victor SPX = Phoenix 5, brushed motors only.
        private final VictorSPX kickerMotor = new VictorSPX(HopperSubsystemConstants.KICKER_MOTOR_ID);

    // STATE MACHINES
        public enum HOPPERSTATE {
            RUN, STOW
        }
    //Tracker Variables
        private boolean fuelDetectedIndexer;
        private HOPPERSTATE kickState;
        private boolean kickManually;
        private boolean desiredVelReached;
        private boolean desiredAngleReached;
    
    //Data
        private ShuffleboardTab HopperSubsystemTab = Shuffleboard.getTab("Hopper Subsystem Tab");
        private GenericEntry fuelDetectedIndexerEntry;
        private GenericEntry indexStateEntry;
        private GenericEntry desiredVelReachedEntry;
        private GenericEntry desiredAngleReachedEntry;
    
    //Sensors 
        Sensors sensors = new Sensors();
    
    public HopperSubsystem(){
        // Conservative-start current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
            // Hopper A/B: NEO 2.0 - 20A smart limit, Coast. kNoResetSafeParameters preserves flashed inversion.
            SparkMaxConfig hopperConfig = new SparkMaxConfig();
            hopperConfig.smartCurrentLimit(20);
            hopperConfig.idleMode(IdleMode.kCoast);
            hopperMotorA.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            hopperMotorB.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            // Kicker: Victor SPX. WARNING - the Victor SPX has NO current sensing, so a current
            // limit is impossible in software; the breaker is the only stall protection. Ramp
            // softens inrush. TODO: confirm which brushed motor it drives + its breaker size.
            kickerMotor.configFactoryDefault();
            kickerMotor.setNeutralMode(NeutralMode.Coast);
            kickerMotor.configOpenloopRamp(0.25); // seconds from 0 to full output

        //Tracker Variables
            fuelDetectedIndexer = false;
            kickState = HOPPERSTATE.STOW;
            desiredVelReached = ShooterSubsystemConstants.desiredVelReached;
            desiredAngleReached = ShooterSubsystemConstants.desiredAngleReached;
    
        //Data
            fuelDetectedIndexerEntry = HopperSubsystemTab.add("Fuel Detected Indexer", false).getEntry();
            indexStateEntry = HopperSubsystemTab.add("Current Index State", kickState.name()).getEntry();
            desiredAngleReachedEntry = HopperSubsystemTab.add("Desired Angle Reached", desiredAngleReached).getEntry();
            desiredVelReachedEntry = HopperSubsystemTab.add("desiredVelocityReached", desiredVelReached).getEntry();
    }

    /**
     * Direction test: runs ONE hopper motor as slowly as practical (5% duty) while the button is
     * held, so the belt direction can be checked before setting relative inversion. The other
     * motor stays in coast and just back-drives through the shared gearbox. Suspends the normal
     * periodic state machine while active so it can't fight the test output.
     */
    public Command directionTest(boolean motorA){
        SparkMax motor = motorA ? hopperMotorA : hopperMotorB;
        return runEnd(
            () -> {
                directionTestActive = true;
                motor.set(HopperSubsystemConstants.DIRECTION_TEST_SPEED);
            },
            () -> {
                directionTestActive = false;
                stopIndex();
            });
    }

    /**
     * Both-belt test: runs BOTH hopper motors at the slow test speed (5% duty) while held.
     * This is the step AFTER the single-motor direction tests -- once each motor's direction is
     * known, this confirms the two agree before ever using full-speed RUN. If the directions are
     * wrong the motors fight through the shared gearbox, but at 5% duty the stall current
     * (~16 A on a NEO) stays under the 20 A smart limit, so it's safe to observe and release.
     * Suspends the periodic state machine the same way the single-motor test does.
     */
    public Command bothBeltsTest(){
        return runEnd(
            () -> {
                directionTestActive = true;
                hopperMotorA.set(HopperSubsystemConstants.DIRECTION_TEST_SPEED);
                hopperMotorB.set(HopperSubsystemConstants.DIRECTION_TEST_SPEED);
            },
            () -> {
                directionTestActive = false;
                stopIndex();
            });
    }

    private boolean directionTestActive = false;

    /**
     * UNJAM: reverse both belts AND the kicker while held, to back a jammed ball out.
     * Reuses the periodic-suspend flag so the STOW branch can't fight the reverse output.
     * NOTE: belt relative inversion is still unverified in code -- "reverse" is relative to
     * whatever is flashed on the SPARK MAXes, same caveat as forward RUN.
     */
    public Command unjamCommand(){
        return runEnd(
            () -> {
                directionTestActive = true;
                hopperMotorA.set(-HopperSubsystemConstants.UNJAM_SPEED);
                hopperMotorB.set(-HopperSubsystemConstants.UNJAM_SPEED);
                kickerMotor.set(ControlMode.PercentOutput, -HopperSubsystemConstants.UNJAM_SPEED);
            },
            () -> {
                directionTestActive = false;
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

    public Command setHopperState(HOPPERSTATE state){
        return Commands.runOnce(()->{
            this.kickState = state;
            if(kickState == HOPPERSTATE.STOW){
                kickManually = true;
            }
        });
    }


    @Override
    public void periodic(){
        //Update Tracker Variables
            fuelDetectedIndexer = sensors.getIndexSensorA() && sensors.getIndexSensorB();
            desiredVelReached = ShooterSubsystemConstants.desiredVelReached;
            desiredAngleReached = ShooterSubsystemConstants.desiredAngleReached;
        //Data
            fuelDetectedIndexerEntry.setBoolean(fuelDetectedIndexer);
            indexStateEntry.setString(kickState.name());
            desiredVelReachedEntry.setBoolean(desiredVelReached);
            desiredAngleReachedEntry.setBoolean(desiredAngleReached);

        //Index Based On State Input (skipped while the direction test owns the motors,
        //otherwise the STOW branch would fight the test's slow output every loop)
        if(directionTestActive){
            return;
        }
        if(kickState == HOPPERSTATE.RUN){
            indexFuel();
            if(desiredVelReached){
                kickFuel();
            } else {
                stopKickFuel();
            }
        } else if (kickState == HOPPERSTATE.STOW){
            stopIndex();
            stopKickFuel();
        }
    }

    // Read-only motor access for PowerTelemetry (no control).
    public SparkMax[] getHopperMotors(){
        return new SparkMax[] { hopperMotorA, hopperMotorB };
    }
}
