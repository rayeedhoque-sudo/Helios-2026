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
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {

    //Intake Motor
        private final TalonFX intakeMotor = new TalonFX(IntakeSubsystemConstants.INTAKE_MOTOR_ID);
        // Active roller stop for STOW: the Coast neutral mode let the rollers visibly
        // spin down after a stow (team report 2026-07-16); StaticBrake halts them.
        private final StaticBrake rollerBrake = new StaticBrake();
        private final SparkMax intakeSliderMotor = new SparkMax(IntakeSubsystemConstants.INTAKE_PIVOT_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //State Machine
        public enum IntakeSSTATE{
            INTAKE_STATE, OUTTAKE_STATE, AGITATE_STATE, STOW_INTAKE_STATE, STOW_STATE;
        }
    
    //Tracker Variables
        // (currentState DELETED 2026-07-21: it was never written after the constructor, so
        // its dashboard entry sat frozen at STOW forever. periodic() actuates desiredState
        // directly every loop -- there is no separate observed state to display.)
        private IntakeSSTATE desiredState;
        // AGITATE roller direction: +1 inward (slider extending), -1 outward (retracting).
        private double agitateDirection = 1;

    //Data
        private ShuffleboardTab IntakeSubsystemTab = Shuffleboard.getTab("Intake Subsystem Tab");
        private GenericEntry desiredStateEntry;
        private GenericEntry sliderCurrentEntry;
        private GenericEntry sliderVelocityEntry;
    
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
            // Front rollers: Kraken X60 (was X44; CAN 16 since 2026-09-30) - stator 40A / supply 25A, Coast (bumped 2026-07-10 from
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
            desiredState = IntakeSSTATE.STOW_STATE;

        // Initializing Shuffleboard Entries
            desiredStateEntry = IntakeSubsystemTab.add("Desired Intake State", desiredState.name()).getEntry();
            sliderCurrentEntry = IntakeSubsystemTab.add("Slider Current (A)", 0.0).getEntry();
            sliderVelocityEntry = IntakeSubsystemTab.add("Slider Velocity (motor rpm)", 0.0).getEntry();

    }

    /**
     * KILL SWITCH for the whole intake (team request 2026-10-10), same shape as the hopper's
     * HOPPER_ENABLED. false = NEITHER intake motor is ever driven: the rollers and the slider
     * are commanded 0 no matter which command is running -- teleop (LT/Y/X, RT/RB shot), test
     * mode and autonomous alike. Bindings and commands still schedule; they just move nothing
     * (slider moves end on their own via the stall/timeout checks). Stops (roller brake,
     * slider stopMotor) still go out, and brake mode holds the slider WHEREVER IT IS NOW.
     */
    private static final boolean INTAKE_ENABLED = true;

    //Subsystem Methods
        public void intake(){
                intakeMotor.set(INTAKE_ENABLED ? IntakeSubsystemConstants.INTAKE_SPEED : 0);
        }

        public void outtake(){
                intakeMotor.set(INTAKE_ENABLED ? -IntakeSubsystemConstants.OUTTAKE_SPEED : 0);
        }

        // Interrupt-safety: brake the rollers and reset the state machine so periodic()
        // can't keep them spinning after the owning command group is cancelled without
        // running stowCommand (e.g. RT/RB/B stealing the shared hopper cancels the LT/Y
        // group mid-hold). Does NOT touch the slider -- brake mode holds it where it is.
        public void stopRollers(){
            desiredState = IntakeSSTATE.STOW_STATE;
            intakeMotor.setControl(rollerBrake);
        }

        // LT release (team 2026-10-14): rollers already pulling in drop to STOW_ROLLER_SPEED
        // while the slider stows. Rollers that never started (tap release) stay stopped.
        public void slowRollersForStow(){
            if (desiredState == IntakeSSTATE.INTAKE_STATE) {
                desiredState = IntakeSSTATE.STOW_INTAKE_STATE;
            }
        }

        // True while the rollers are commanded to spin (intake, outtake, or the slow roll-in
        // during LT's release stow) -- the drivetrain reads this to halve its translation speed
        // while the intake is live, and LT's release belts run while it is true.
        // AGITATE (shot/unjam slow roll) does not count -- those set their own drive speed.
        public boolean rollersRunning(){
            return INTAKE_ENABLED
                    && (desiredState == IntakeSSTATE.INTAKE_STATE || desiredState == IntakeSSTATE.OUTTAKE_STATE
                        || desiredState == IntakeSSTATE.STOW_INTAKE_STATE);
        }

        // True while the slider motor is turning slower than stallRpm (measured encoder rpm,
        // not commanded output). Not turning while commanded = hard stop or jam.
        public boolean isSliderEncoderStalled(double stallRpm){
            return Math.abs(intakeSliderMotor.getEncoder().getVelocity()) < stallRpm;
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
        // A slower duty than SLIDER_DUTY scales the stall rpm down and the timeout up by the
        // same ratio (1.0 at SLIDER_DUTY = unchanged), so a slow move is not read as a stall
        // or cut short mid-travel.
        private Command moveSliderUntilStall(double dutyCycle){
            double speedScale = Math.abs(dutyCycle) / IntakeSubsystemConstants.SLIDER_DUTY;
            return Commands.startEnd(
                    ()->{
                        sliderCurrentStallDebouncer = new Debouncer(
                            IntakeSubsystemConstants.SLIDER_CURRENT_STALL_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
                        sliderVelStallDebouncer = new Debouncer(
                            IntakeSubsystemConstants.SLIDER_STALL_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
                        sliderMoveTimer.restart();
                        intakeSliderMotor.set(INTAKE_ENABLED ? dutyCycle : 0);
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
                            && isSliderEncoderStalled(IntakeSubsystemConstants.SLIDER_STALL_MOTOR_RPM * speedScale)))
                .withTimeout(IntakeSubsystemConstants.SLIDER_MOVE_TIMEOUT_SEC / speedScale);
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

        // AUTO START (team 2026-10-16): slider fully out to its stop and back in, twice. Rollers
        // untouched (left braked). Each stroke is the normal stall-stopped move, so it ends stowed.
        public Command sliderCycleCommand(){
            return Commands.sequence(extendSliderCommand(), retractSliderCommand(),
                                     extendSliderCommand(), retractSliderCommand());
        }

        // ROLLER-ONLY TEST: spin the rollers (Kraken X60, CAN 16) while held, stop on release.
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

        // RT / RB / LB SHOT: the slider extends for only SLIDER_CYCLE_EXTEND_SEC (a partial
        // stroke, never to the outer stop), then retracts fully to the stow stop, and repeats
        // for the whole hold (agitation re-added 2026-10-10, team request). The rollers spin at
        // ROLLER_AGITATE_SPEED inward while it extends and outward while it retracts, alternating
        // every leg (team 2026-10-10); stopped on end/interrupt. Both legs are the same
        // stall-stopped move as extend/retract (pulley-snap guard intact); the extend leg is just
        // cut short by the timeout. The binding's onFalse(stowCommand) brings it home.
        // INTAKE WHILE SHOOTING (team 2026-10-10): while intakeHeld (LT), agitation stops and the
        // normal intake runs instead (slider out to its stop, then rollers in at INTAKE_SPEED);
        // letting go goes straight back to agitation. The shot keeps the hopper, so the belts
        // stay on the shot feed.
        public Command shotIntakeCommand(BooleanSupplier intakeHeld){
            return shotIntakeCommand(intakeHeld, () -> false);
        }

        /**
         * Same, plus LB's "behind the hub" block (team 2026-10-17): while shotBlocked (and LT is
         * not held) there is no agitation -- the slider stows (rollers stopped) and waits; once
         * clear, agitation starts again. LT still wins either way.
         */
        public Command shotIntakeCommand(BooleanSupplier intakeHeld, BooleanSupplier shotBlocked){
            Command agitate = Commands.sequence(
                    Commands.runOnce(() -> desiredState = IntakeSSTATE.AGITATE_STATE),
                    Commands.repeatingSequence(
                        Commands.runOnce(() -> agitateDirection = 1),
                        extendSliderCommand().withTimeout(IntakeSubsystemConstants.SLIDER_CYCLE_EXTEND_SEC),
                        Commands.runOnce(() -> agitateDirection = -1),
                        retractSliderCommand()))
                .until(() -> intakeHeld.getAsBoolean() || shotBlocked.getAsBoolean());
            Command intake = intakeCommand().andThen(Commands.idle())
                .until(() -> !intakeHeld.getAsBoolean());
            Command stowed = stowCommand().andThen(Commands.idle())
                .until(() -> intakeHeld.getAsBoolean() || !shotBlocked.getAsBoolean());
            return Commands.repeatingSequence(
                    Commands.either(intake, Commands.either(stowed, agitate, shotBlocked), intakeHeld))
                .finallyDo(this::stopRollers);
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
                }
                else if(desiredState == IntakeSSTATE.AGITATE_STATE){
                    intakeMotor.set(INTAKE_ENABLED ? agitateDirection * IntakeSubsystemConstants.ROLLER_AGITATE_SPEED : 0);
                }
                else if(desiredState == IntakeSSTATE.STOW_INTAKE_STATE){
                    intakeMotor.set(INTAKE_ENABLED ? IntakeSubsystemConstants.STOW_ROLLER_SPEED : 0);
                } else {
                    // STOW: actively brake (stopMotor = coast, which reads as "still rolling").
                    intakeMotor.setControl(rollerBrake);
                }
            //Data
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
