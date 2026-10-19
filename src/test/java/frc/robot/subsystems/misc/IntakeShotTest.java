package frc.robot.subsystems.misc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Constants.SubsystemConstants.IntakeSubsystemConstants;

/** Pins intake-while-shooting (LT overrides the shot agitation) and the LT release stow. */
class IntakeShotTest {

    // One per JVM: SPARK CAN IDs and Shuffleboard titles can't be created twice.
    private static IntakeSubsystem intake;

    @BeforeAll
    static void build() {
        assertTrue(HAL.initialize(500, 0));
        intake = new IntakeSubsystem();
    }

    @Test
    void ltIntakesDuringAShotThenGoesBackToAgitating() {
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        CommandScheduler.getInstance().registerSubsystem(intake);
        try {
            boolean[] lt = {false};
            Command shot = intake.shotIntakeCommand(() -> lt[0]);
            CommandScheduler.getInstance().schedule(shot);

            run(1.0);
            assertFalse(intake.rollersRunning(), "agitating, not intaking");

            lt[0] = true;
            run(3.0); // slider extend stall-stops (sim encoder reads 0) or hits its 2.5 s backstop
            assertTrue(intake.rollersRunning(), "LT overrides agitation");

            lt[0] = false;
            run(0.04);
            assertFalse(intake.rollersRunning(), "releasing LT goes back to agitating");
            assertTrue(shot.isScheduled(), "the shot's intake part keeps running");
        } finally {
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().unregisterAllSubsystems();
            SimHooks.resumeTiming();
        }
    }

    // LB behind the hub (team 2026-10-17): blocked = no agitation (slider stows, rollers braked);
    // LT still intakes while blocked; clear again = agitation again.
    @Test
    void blockedShotStowsInsteadOfAgitating() {
        SimHooks.pauseTiming();
        setEnabled(true);
        CommandScheduler.getInstance().registerSubsystem(intake);
        try {
            boolean[] lt = {false};
            boolean[] blocked = {false};
            Command shot = intake.shotIntakeCommand(() -> lt[0], () -> blocked[0]);
            CommandScheduler.getInstance().schedule(shot);

            run(0.1);
            assertEquals(IntakeSubsystemConstants.ROLLER_AGITATE_SPEED, Math.abs(rollerDuty(intake)), 1e-9, "agitating");

            blocked[0] = true;
            run(0.1);
            assertTrue(intake.getRollerMotor().getAppliedControl() instanceof StaticBrake, "blocked: rollers braked, slider stows");

            lt[0] = true;
            run(3.0);
            assertTrue(intake.rollersRunning(), "LT still intakes while blocked");

            lt[0] = false;
            run(0.1);
            assertTrue(intake.getRollerMotor().getAppliedControl() instanceof StaticBrake, "LT off, still blocked: stowed again");

            blocked[0] = false;
            run(0.1);
            assertEquals(IntakeSubsystemConstants.ROLLER_AGITATE_SPEED, Math.abs(rollerDuty(intake)), 1e-9, "clear: agitating again");
            assertTrue(shot.isScheduled());
        } finally {
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().unregisterAllSubsystems();
            SimHooks.resumeTiming();
        }
    }

    // LT release (team 2026-10-14): rollers + belts run until the slider is stowed; disable still brakes at once.
    @Test
    void ltReleaseKeepsRollersAndBeltsUntilStowed() {
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        setEnabled(true);
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.registerSubsystem(intake);
        HopperSubsystem hopper = new HopperSubsystem();
        try {
            boolean[] lt = {false};
            RobotContainer.bindLtIntake(new Trigger(() -> lt[0]).and(RobotModeTriggers.teleop()), intake, hopper);

            // Tap: released before the slider is out -- rollers never started, so they stay off.
            lt[0] = true;
            run(0.1);
            lt[0] = false;
            run(0.04);
            assertFalse(intake.rollersRunning(), "tap release never starts the rollers");
            assertNull(scheduler.requiring(hopper), "tap release never starts the belts");
            run(1.0);

            lt[0] = true;
            run(3.0);
            assertTrue(intake.rollersRunning(), "rollers in once the slider is out");
            assertEquals(IntakeSubsystemConstants.INTAKE_SPEED, rollerDuty(intake), 1e-9);

            lt[0] = false;
            run(0.1); // retract stall-stops ~0.45 s in (0.3 s grace + 0.15 s debounce)
            assertTrue(intake.rollersRunning(), "rollers keep pulling in while the slider retracts");
            assertEquals(0.125 * IntakeSubsystemConstants.INTAKE_SPEED, rollerDuty(intake), 1e-9, "at 12.5% of intake speed");
            assertNotNull(scheduler.requiring(hopper), "belts keep running while the slider retracts");
            assertNotNull(scheduler.requiring(intake), "stow is retracting");

            run(1.0);
            assertNull(scheduler.requiring(intake), "stowed");
            assertFalse(intake.rollersRunning(), "rollers stop once stowed");
            assertNull(scheduler.requiring(hopper), "belts stop once stowed");

            // Disable mid-hold: the release stow can't run disabled, so the hold itself must brake.
            lt[0] = true;
            run(3.0);
            assertTrue(intake.rollersRunning());
            setEnabled(false);
            run(0.04);
            assertFalse(intake.rollersRunning(), "disable brakes the rollers");
            lt[0] = false;
            run(0.04);
            setEnabled(true);
            run(0.04);
            assertFalse(intake.rollersRunning(), "re-enable does not restart them");
            run(1.0);

            // A held across the LT release: A keeps the belts, the stow still retracts.
            lt[0] = true;
            run(3.0);
            Command manual = hopper.manualRunCommand();
            scheduler.schedule(manual);
            run(0.04);
            lt[0] = false;
            run(0.04);
            assertTrue(manual.isScheduled(), "LT release doesn't cancel A");
            assertNotNull(scheduler.requiring(intake), "stow is retracting");
            scheduler.cancel(manual);
            run(1.0);

            // Path 1's auto intake (team 2026-10-14): the SAME binding fed by the auto's window, in
            // auto. Holding = LT held; the window closing = LT let go (rollers 10% + belts until stowed).
            DriverStationSim.setAutonomous(true);
            setEnabled(true);
            boolean[] window = {false};
            RobotContainer.bindLtIntake(new Trigger(() -> window[0]), intake, hopper);
            window[0] = true;
            run(3.0);
            assertTrue(intake.rollersRunning(), "auto: rollers in once the slider is out");
            assertEquals(IntakeSubsystemConstants.INTAKE_SPEED, rollerDuty(intake), 1e-9);
            window[0] = false;
            run(0.1);
            assertTrue(intake.rollersRunning(), "auto release keeps the rollers pulling in, like LT");
            assertEquals(0.125 * IntakeSubsystemConstants.INTAKE_SPEED, rollerDuty(intake), 1e-9);
            assertNotNull(scheduler.requiring(hopper), "auto release keeps the belts, like LT");
            assertNotNull(scheduler.requiring(intake), "auto release stows, like LT");
            run(1.0);
            assertNull(scheduler.requiring(intake), "stowed");
            assertFalse(intake.rollersRunning(), "rollers stop once stowed");
            assertNull(scheduler.requiring(hopper), "belts stop once stowed");
        } finally {
            DriverStationSim.setAutonomous(false);
            scheduler.cancelAll();
            scheduler.unregisterAllSubsystems();
            scheduler.getActiveButtonLoop().clear();
            SimHooks.resumeTiming();
        }
    }

    // Auto start (team 2026-10-16): slider out, in, out, in at the normal stall-stopped duty, rollers braked throughout.
    @Test
    void sliderCycleGoesOutAndInTwiceWithRollersOff() {
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        setEnabled(true);
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.registerSubsystem(intake);
        try {
            intake.stopRollers();
            Command cycle = intake.sliderCycleCommand();
            scheduler.schedule(cycle);
            List<Double> strokes = new ArrayList<>();
            for (int i = 0; i < 600 && cycle.isScheduled(); i++) { // 12 s > 4 x the 2.5 s backstop
                double duty = intake.getSliderMotor().get();
                if (duty != 0 && (strokes.isEmpty() || Math.signum(duty) != Math.signum(strokes.get(strokes.size() - 1)))) {
                    strokes.add(duty);
                }
                assertFalse(intake.rollersRunning(), "rollers stay off");
                run(0.02);
            }
            double d = IntakeSubsystemConstants.SLIDER_DUTY;
            assertEquals(List.of(-d, d, -d, d), strokes, "extend, retract, extend, retract");
            assertFalse(cycle.isScheduled(), "done after the 4th stroke");
            assertEquals(0.0, intake.getSliderMotor().get(), 1e-9, "slider stopped, stowed");
            assertTrue(intake.getRollerMotor().getAppliedControl() instanceof StaticBrake, "rollers braked");
        } finally {
            scheduler.cancelAll();
            scheduler.unregisterAllSubsystems();
            SimHooks.resumeTiming();
        }
    }

    // Which autos get the cycle scheduled beside them: only ones that drive and don't hold the intake.
    @Test
    void sliderCycleRunsBesideDrivingAutosOnly() {
        Subsystem drive = new Subsystem() {}, intakeSub = new Subsystem() {};
        assertTrue(RobotContainer.runsSliderCycleBeside(Commands.idle(drive), drive, intakeSub), "drive-only auto");
        assertFalse(RobotContainer.runsSliderCycleBeside(Commands.none(), drive, intakeSub), "\"None\" moves nothing");
        assertFalse(RobotContainer.runsSliderCycleBeside(Commands.idle(drive, intakeSub), drive, intakeSub),
            "an auto holding the intake runs it inside");
    }

    // Duty the rollers were last commanded (read-only; not a stall signal).
    private static double rollerDuty(IntakeSubsystem intake) {
        return ((DutyCycleOut) intake.getRollerMotor().getAppliedControl()).Output;
    }

    private static void setEnabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }

    // Run the scheduler for sec of simulated time, 20 ms per loop.
    private static void run(double sec) {
        for (int i = 0; i < Math.round(sec / 0.02); i++) {
            CommandScheduler.getInstance().run();
            SimHooks.stepTiming(0.02);
        }
    }
}
