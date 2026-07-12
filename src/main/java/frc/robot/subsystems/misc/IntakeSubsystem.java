package frc.robot.subsystems.misc;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase;
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
        private GenericEntry sliderPeakCurrentEntry;
        private GenericEntry sliderVelocityEntry;
        // Peak current seen since the last slider move started (reset in moveSliderUntilStall)
        // -- catches spikes too brief to read live off the dashboard.
        private double sliderPeakAmps = 0;
    
    // //Sensors
        Sensors sensors = new Sensors();

    //Slider stall detection (measured current, not commanded output).
        // Recreated at the START of every move: a Debouncer keeps its last state, so a shared
        // instance still reads "stalled" from the previous move and would end the next move
        // the moment the grace period expires (slider never reaches the hard stop).
        private Debouncer sliderStallDebouncer;
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
            // Slider: NEO 2.0 through a 20:1 gearbox (swapped 2026-07-10 from ~10:1). 60A smart
            // limit = data-sheet sec.7 max column -- torque prioritized per team request; safe
            // because every move is stall-detected + 3s-timeout capped, so the motor never sits
            // at 60A for more than the 0.15s debounce. Brake to hold position.
            // kNoResetSafeParameters preserves flashed inversion.
            SparkMaxConfig sliderConfig = new SparkMaxConfig();
            sliderConfig.smartCurrentLimit(60);
            sliderConfig.idleMode(IdleMode.kBrake);
            // Velocity loop on the built-in NEO encoder (motor rpm): slow commanded speed with
            // full torque authority -- see SLIDER_VELOCITY_* comments in constants.
            sliderConfig.closedLoop
                .p(IntakeSubsystemConstants.SLIDER_VELOCITY_kP)
                .velocityFF(IntakeSubsystemConstants.SLIDER_VELOCITY_FF);
            intakeSliderMotor.configure(sliderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        //Initializing Tracker Variables
            currentState = IntakeSSTATE.STOW_STATE;
            desiredState = IntakeSSTATE.STOW_STATE;
        
        // Initializing Shuffleboard Entries
            desiredStateEntry = IntakeSubsystemTab.add("Desired Intake State", desiredState.name()).getEntry();
            currentStateEntry = IntakeSubsystemTab.add("Current Intake State", currentState.name()).getEntry();
            sliderCurrentEntry = IntakeSubsystemTab.add("Slider Current (A)", 0.0).getEntry();
            sliderPeakCurrentEntry = IntakeSubsystemTab.add("Slider Peak Current This Move (A)", 0.0).getEntry();
            sliderVelocityEntry = IntakeSubsystemTab.add("Slider Velocity (motor rpm)", 0.0).getEntry();

    }

    //Subsystem Methods
        public void intake(){
                intakeMotor.set(IntakeSubsystemConstants.INTAKE_SPEED);
        }

        public void outtake(){
                intakeMotor.set(-IntakeSubsystemConstants.OUTTAKE_SPEED);
        }

        // True while the slider motor draws stall-level current (measured, not commanded).
        public boolean isIntakeSliderStalled(){
            return intakeSliderMotor.getOutputCurrent() > IntakeSubsystemConstants.SLIDER_STALL_AMPS;
        }

    //Command Based methods

        // Drive the slider at a slow closed-loop velocity (signed motor rpm) until it hits a
        // hard stop (debounced stall current), then stop the motor instantly. Grace period
        // skips the startup inrush; timeout is a backstop so the motor can never sit stalled
        // if detection misses. Brake mode holds position at zero current afterward.
        private Command moveSliderUntilStall(double motorRpm){
            return Commands.startEnd(
                    ()->{
                        sliderStallDebouncer = new Debouncer(
                            IntakeSubsystemConstants.SLIDER_STALL_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
                        sliderPeakAmps = 0;
                        sliderMoveTimer.restart();
                        intakeSliderMotor.getClosedLoopController()
                            .setReference(motorRpm, SparkBase.ControlType.kVelocity);
                    },
                    ()-> intakeSliderMotor.stopMotor(),
                    this)
                // Feed the debouncer false during the grace window (instead of gating the result)
                // so startup inrush current can never pre-charge it toward a false stall.
                .until(()->
                    sliderStallDebouncer.calculate(
                        sliderMoveTimer.hasElapsed(IntakeSubsystemConstants.SLIDER_GRACE_SEC)
                            && isIntakeSliderStalled()))
                .withTimeout(IntakeSubsystemConstants.SLIDER_MOVE_TIMEOUT_SEC);
        }

        // Extend the intake slider out until the hard stop. TODO verify sign on robot
        // (negative = extend, matching the old deploy direction).
        public Command extendSliderCommand(){
            return moveSliderUntilStall(-IntakeSubsystemConstants.SLIDER_TRAVEL_MOTOR_RPM);
        }

        // Retract (de-extend) the intake slider until the hard stop.
        public Command retractSliderCommand(){
            return moveSliderUntilStall(IntakeSubsystemConstants.SLIDER_TRAVEL_MOTOR_RPM);
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
                    Commands.runOnce(()->{
                        desiredState = IntakeSSTATE.STOW_STATE;
                    }),
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
                    intakeMotor.stopMotor();
                }
            //Data
                currentStateEntry.setString(currentState.name());
                desiredStateEntry.setString(desiredState.name());
                double sliderAmps = intakeSliderMotor.getOutputCurrent();
                sliderPeakAmps = Math.max(sliderPeakAmps, sliderAmps);
                sliderCurrentEntry.setDouble(sliderAmps);
                sliderPeakCurrentEntry.setDouble(sliderPeakAmps);
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
