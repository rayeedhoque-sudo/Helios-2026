package frc.robot.subsystems.misc;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.IntakeSubsystemConstants;
import frc.robot.subsystems.utility.Sensors;

public class IntakeSubsystem extends SubsystemBase {

    //Type
        boolean enableComp;
        boolean enableTest = false;
    
    //Intake Motor
        private final TalonFX intakeMotor = new TalonFX(IntakeSubsystemConstants.INTAKE_MOTOR_ID);
        // Active roller stop for STOW: the Coast neutral mode let the rollers visibly
        // spin down after a stow (team report 2026-07-16); StaticBrake halts them.
        private final StaticBrake rollerBrake = new StaticBrake();
        private final SparkMax intakeSliderMotor = new SparkMax(IntakeSubsystemConstants.INTAKE_PIVOT_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //State Machine
        public enum IntakeSSTATE{
            INTAKE_STATE, OUTTAKE_STATE, STOW_STATE;
        }
    
    //Tracker Variables
        private IntakeSSTATE currentState;
        private IntakeSSTATE desiredState;

    //Data
        private ShuffleboardTab IntakeSubsystemTab = Shuffleboard.getTab("Intake Subsystem Tab");
        private GenericEntry currentStateEntry;
        private GenericEntry desiredStateEntry;
        private GenericEntry sliderCurrentEntry;
        private GenericEntry sliderVelocityEntry;
    
    // //Sensors
        Sensors sensors = new Sensors();

    //Slider stall detection (measured signals, not commanded output).
        // Recreated at the START of every move: a Debouncer keeps its last state, so a shared
        // instance still reads "stalled" from the previous move and would end the next move
        // the moment the grace period expires (slider never reaches the hard stop).
        // Two independent detectors: current (fast, 0.1 s, no grace needed -- the open-loop
        // ramp keeps startup current low) and encoder velocity (slower, needs the spin-up
        // grace because the encoder reads 0 rpm at move start).
        private Debouncer sliderCurrentStallDebouncer;
        private Debouncer sliderVelStallDebouncer;
        private final Timer sliderMoveTimer = new Timer();

    public IntakeSubsystem(){

        // Conservative-start current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
            // Front rollers: Kraken X44 - stator 40A / supply 25A, Coast (bumped 2026-07-10 from
            // the conservative 30/15 start; matches Hardware-Data-Sheet sec.7 mid column, max is
            // 60/40 if grab torque is still short). Stator = grab torque; supply caps battery
            // draw. Worst-case steady supply = duty x stator: 0.5 x 40 = 20 A < 25 A, so the
            // supply limit never clips normal running. (FOC would cut amps-per-torque further
            // but our TalonFXs are unlicensed -- see TunerConstants.)
            CurrentLimitsConfigs rollerLimits = new CurrentLimitsConfigs();
            rollerLimits.StatorCurrentLimit = 40;
            rollerLimits.StatorCurrentLimitEnable = true;
            rollerLimits.SupplyCurrentLimit = 25;
            rollerLimits.SupplyCurrentLimitEnable = true;
            MotorOutputConfigs rollerOutput = new MotorOutputConfigs();
            rollerOutput.NeutralMode = NeutralModeValue.Coast;
            // Duty-cycle ramp: 0.25 s from 0 to full output (~0.09 s to reach 35%). Kills the
            // spin-up inrush spike -- the one place an unloaded roller actually hits its limits.
            OpenLoopRampsConfigs rollerRamp = new OpenLoopRampsConfigs();
            rollerRamp.DutyCycleOpenLoopRampPeriod = 0.25;
            intakeMotor.getConfigurator().apply(rollerLimits);
            intakeMotor.getConfigurator().apply(rollerOutput);
            intakeMotor.getConfigurator().apply(rollerRamp);
            // Slider: NEO 2.0 through a 20:1 gearbox. PULLEY-SNAP GUARD (a slider pulley
            // snapped pre-redesign from sustained tension + gear grinding — these three
            // protections must never be removed):
            //   1. TIERED smart limit: 15 A at stall (0 rpm) ramping to 20 A above 1000 motor
            //      rpm — full torque while traveling, 25% LESS belt tension than before once
            //      the slider hits the hard stop or jams. TODO verify 15 A still breaks the
            //      slider away from rest on robot; raise the stall arg toward 20 if not.
            //   2. 0.25 s open-loop ramp: no instantaneous torque step slamming the belt.
            //   3. Stall cutoff (moveSliderUntilStall): output is CUT ~0.05 s after the
            //      current pegs at the smart limit (catches belt/gear-slip stalls where
            //      the motor keeps spinning) or ~0.15 s after the encoder drops below the
            //      stall rpm, so it can never sit grinding at a hard stop.
            // Brake holds position at zero output (zero holding current).
            // kNoResetSafeParameters preserves flashed inversion.
            SparkMaxConfig sliderConfig = new SparkMaxConfig();
            sliderConfig.smartCurrentLimit(15, 20, 1000);
            sliderConfig.openLoopRampRate(0.25);
            sliderConfig.idleMode(IdleMode.kBrake);
            intakeSliderMotor.configure(sliderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        //Initializing Tracker Variables
            currentState = IntakeSSTATE.STOW_STATE;
            desiredState = IntakeSSTATE.STOW_STATE;
        
        // Initializing Shuffleboard Entries
            desiredStateEntry = IntakeSubsystemTab.add("Desired Intake State", desiredState.name()).getEntry();
            currentStateEntry = IntakeSubsystemTab.add("Current Intake State", currentState.name()).getEntry();
            sliderCurrentEntry = IntakeSubsystemTab.add("Slider Current (A)", 0.0).getEntry();
            sliderVelocityEntry = IntakeSubsystemTab.add("Slider Velocity (motor rpm)", 0.0).getEntry();

    }

    //Subsystem Methods
        public void intake(){
                intakeMotor.set(IntakeSubsystemConstants.INTAKE_SPEED);
        }

        public void outtake(){
                intakeMotor.set(-IntakeSubsystemConstants.OUTTAKE_SPEED);
        }

        // Interrupt-safety: brake the rollers and reset the state machine so periodic()
        // can't keep them spinning after the owning command group is cancelled without
        // running stowCommand (e.g. RT/RB/B stealing the shared hopper cancels the LT/Y
        // group mid-hold). Does NOT touch the slider -- brake mode holds it where it is.
        public void stopRollers(){
            desiredState = IntakeSSTATE.STOW_STATE;
            intakeMotor.setControl(rollerBrake);
        }

        // True while the rollers are commanded to spin (intake OR outtake) -- the
        // drivetrain reads this to halve its translation speed while the intake is live.
        public boolean rollersRunning(){
            return desiredState != IntakeSSTATE.STOW_STATE;
        }

        // True while the slider motor is not actually turning (measured encoder rpm, not
        // commanded output). Not turning while commanded = hard stop or jam.
        public boolean isSliderEncoderStalled(){
            return Math.abs(intakeSliderMotor.getEncoder().getVelocity())
                    < IntakeSubsystemConstants.SLIDER_STALL_MOTOR_RPM;
        }

        // True while the measured motor current is pegged near the smart limit -- the
        // slider is loaded up against the end of travel (or jammed), even if belt/gear
        // slip keeps the motor itself spinning. Fires with no grace period: the 0.25 s
        // open-loop ramp keeps startup current well below this threshold.
        public boolean isSliderCurrentPegged(){
            return intakeSliderMotor.getOutputCurrent()
                    >= IntakeSubsystemConstants.SLIDER_STALL_CURRENT_AMPS;
        }

    //Command Based methods

        // Move the slider at a fixed slow duty until it stalls -- current pegged (fast
        // path, ~0.05 s) or encoder below stall rpm (backup path, ~0.15 s after the grace
        // window) -- then stop the motor: it never keeps pushing where it can't go.
        // Grace period covers spin-up from rest on the VELOCITY path only (encoder reads
        // 0 rpm at move start, which would otherwise look like an instant stall); the
        // current path needs no grace because the open-loop ramp keeps startup current
        // low. Timeout is a backstop so the motor can never sit stalled if detection
        // misses. Brake mode holds position afterward.
        private Command moveSliderUntilStall(double dutyCycle){
            return Commands.startEnd(
                    ()->{
                        sliderCurrentStallDebouncer = new Debouncer(
                            IntakeSubsystemConstants.SLIDER_CURRENT_STALL_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
                        sliderVelStallDebouncer = new Debouncer(
                            IntakeSubsystemConstants.SLIDER_STALL_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
                        sliderMoveTimer.restart();
                        intakeSliderMotor.set(dutyCycle);
                    },
                    ()-> intakeSliderMotor.stopMotor(),
                    this)
                // Current check FIRST so it is fed every cycle (|| short-circuits). The
                // velocity debouncer is fed false during the grace window (instead of
                // gating the result) so the 0-rpm spin-up can never pre-charge it toward
                // a false stall.
                .until(()->
                    sliderCurrentStallDebouncer.calculate(isSliderCurrentPegged())
                    || sliderVelStallDebouncer.calculate(
                        sliderMoveTimer.hasElapsed(IntakeSubsystemConstants.SLIDER_GRACE_SEC)
                            && isSliderEncoderStalled()))
                .withTimeout(IntakeSubsystemConstants.SLIDER_MOVE_TIMEOUT_SEC);
        }

        // Extend the intake slider out until the hard stop. TODO verify sign on robot
        // (negative = extend, matching the old deploy direction).
        public Command extendSliderCommand(){
            return moveSliderUntilStall(-IntakeSubsystemConstants.SLIDER_DUTY);
        }

        // Retract (de-extend) the intake slider until the hard stop.
        public Command retractSliderCommand(){
            return moveSliderUntilStall(IntakeSubsystemConstants.SLIDER_DUTY);
        }

        // ROLLER-ONLY TEST: spin the rollers (Kraken X44, CAN 18) while held, stop on release.
        // Never commands the slider -- it stays wherever it is (brake mode holds it).
        public Command rollerTestCommand(boolean inward){
            return startEnd(
                ()-> desiredState = inward ? IntakeSSTATE.INTAKE_STATE : IntakeSSTATE.OUTTAKE_STATE,
                ()-> desiredState = IntakeSSTATE.STOW_STATE);
        }

        public Command intakeCommand(){
            return Commands.sequence(
                    extendSliderCommand(),
                    Commands.runOnce(()->{
                        desiredState = IntakeSSTATE.INTAKE_STATE;
                    })
            );
        }

        public Command outtakeCommand(){
            return Commands.sequence(
                    extendSliderCommand(),
                    Commands.runOnce(()->{
                        desiredState = IntakeSSTATE.OUTTAKE_STATE;
                    })
            );
        }

        public Command stowCommand(){
            return Commands.sequence(
                    // Immediate, unconditional roller stop (don't wait a loop for
                    // periodic to see the state change) -- rollers must be stopped
                    // the instant a manual stow starts, no matter what.
                    Commands.runOnce(this::stopRollers),
                    retractSliderCommand()
            );
        }


    @Override
        public void periodic(){
            //STATE MACHINE
                if(desiredState == IntakeSSTATE.INTAKE_STATE){
                    intake();
                }
                else if(desiredState == IntakeSSTATE.OUTTAKE_STATE){
                    outtake();
                } else {
                    // STOW: actively brake (stopMotor = coast, which reads as "still rolling").
                    intakeMotor.setControl(rollerBrake);
                }
            //Data
                currentStateEntry.setString(currentState.name());
                desiredStateEntry.setString(desiredState.name());
                sliderCurrentEntry.setDouble(intakeSliderMotor.getOutputCurrent());
                sliderVelocityEntry.setDouble(intakeSliderMotor.getEncoder().getVelocity());
        }

    // Read-only motor access for PowerTelemetry (no control).
    public TalonFX getRollerMotor(){
        return intakeMotor;
    }

    // Read-only motor access for PowerTelemetry (no control).
    public SparkMax getSliderMotor(){
        return intakeSliderMotor;
    }
}
