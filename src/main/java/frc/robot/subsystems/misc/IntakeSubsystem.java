package frc.robot.subsystems.misc;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
    
    // //Sensors
        Sensors sensors = new Sensors();

    //Slider stall detection (measured current, not commanded output)
        private final Debouncer sliderStallDebouncer =
            new Debouncer(IntakeSubsystemConstants.SLIDER_STALL_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
        private final Timer sliderMoveTimer = new Timer();

    public IntakeSubsystem(){

        // Conservative-start current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
            // Front rollers: Kraken X44 - stator 30A / supply 20A, Coast.
            CurrentLimitsConfigs rollerLimits = new CurrentLimitsConfigs();
            rollerLimits.StatorCurrentLimit = 30;
            rollerLimits.StatorCurrentLimitEnable = true;
            rollerLimits.SupplyCurrentLimit = 20;
            rollerLimits.SupplyCurrentLimitEnable = true;
            MotorOutputConfigs rollerOutput = new MotorOutputConfigs();
            rollerOutput.NeutralMode = NeutralModeValue.Coast;
            intakeMotor.getConfigurator().apply(rollerLimits);
            intakeMotor.getConfigurator().apply(rollerOutput);
            // Slider: NEO 2.0 - 20A smart limit, Brake to hold position. kNoResetSafeParameters preserves flashed inversion.
            SparkMaxConfig sliderConfig = new SparkMaxConfig();
            sliderConfig.smartCurrentLimit(20);
            sliderConfig.idleMode(IdleMode.kBrake);
            intakeSliderMotor.configure(sliderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        //Initializing Tracker Variables
            currentState = IntakeSSTATE.STOW_STATE;
            desiredState = IntakeSSTATE.STOW_STATE;
        
        // Initializing Shuffleboard Entries
            desiredStateEntry = IntakeSubsystemTab.add("Desired Intake State", desiredState.name()).getEntry();
            currentStateEntry = IntakeSubsystemTab.add("Current Intake State", currentState.name()).getEntry();

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

        // Drive the slider until it hits a hard stop (debounced stall current), then stop
        // the motor instantly. Grace period skips the startup inrush; timeout is a backstop
        // so the motor can never sit stalled if detection misses. Brake mode holds position
        // at zero current afterward.
        private Command moveSliderUntilStall(double speed){
            return Commands.startEnd(
                    ()->{
                        sliderMoveTimer.restart();
                        intakeSliderMotor.set(speed);
                    },
                    ()-> intakeSliderMotor.stopMotor(),
                    this)
                .until(()->
                    sliderStallDebouncer.calculate(isIntakeSliderStalled())
                        && sliderMoveTimer.hasElapsed(IntakeSubsystemConstants.SLIDER_GRACE_SEC))
                .withTimeout(IntakeSubsystemConstants.SLIDER_MOVE_TIMEOUT_SEC);
        }

        // Extend the intake slider out until the hard stop. TODO verify sign on robot
        // (negative = extend, matching the old deploy direction).
        public Command extendSliderCommand(){
            return moveSliderUntilStall(-IntakeSubsystemConstants.SLIDER_TRAVEL_SPEED);
        }

        // Retract (de-extend) the intake slider until the hard stop.
        public Command retractSliderCommand(){
            return moveSliderUntilStall(IntakeSubsystemConstants.SLIDER_TRAVEL_SPEED);
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
        }
}
