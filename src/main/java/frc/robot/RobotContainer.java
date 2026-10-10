// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.TunerConstants;
import frc.robot.util.Tunable;
import frc.robot.Constants.SubsystemConstants.LauncherConstants;
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.misc.HopperSubsystem;
import frc.robot.subsystems.misc.IntakeSubsystem;
import frc.robot.subsystems.misc.MatchStatus;
import frc.robot.subsystems.misc.LauncherSubsystem;
import frc.robot.subsystems.utility.ShotCalculator;
import frc.robot.subsystems.utility.LimelightThermalManager;

public class RobotContainer {
    // Global 20% slowdown (team request 2026-07-17): translation AND rotation capped at 80% of
    // full for more controllable driving. Applied at the top-level scalar so it flows into every
    // teleop drive path (default drive + the shaped/scaled suppliers). Autos use explicit speeds
    // and are NOT affected. TODO tune the 0.8 factor to driver preference.
    private double MaxSpeed = 0.8 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // 80% of kSpeedAt12Volts
    // 0.8 rot/s teleop spin rate (was 1.0; -20% per the same request). Physical ceiling ~1.9 rot/s.
    private double MaxAngularRate = RotationsPerSecond.of(0.8).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // Deadband is applied to the RAW sticks in shapeAxis() below, so the request's own
            // deadband is left at 0 (applying it twice would eat real low-speed commands).
            .withDriveRequestType(DriveRequestType.Velocity); // Velocity = CLOSED-loop velocity control
    // Smooth translation commands: caps how fast the drive command can change (units: full-stick
    // fraction per second; 2.5 = stop-to-full in 0.4 s). Softens acceleration spikes so 4 Krakens
    // don't slam their supply limit together (brownout/wheel-slip protection) without feeling laggy.
    // Lowered 4.0 -> 2.5 (2026-07-18): 4.0 allowed ~15 m/s^2 equivalent -- above the physical
    // traction limit, so it never actually constrained current, and a full reversal (2.0 axis
    // units) fit entirely inside the drives' supply burst window. At 2.5 a reversal takes 0.8 s,
    // outlasting the 0.25 s burst window so the 30 A fold-back engages mid-maneuver. ~9.5 m/s^2
    // equivalent still exceeds real traction accel, so the stick should not feel laggy.
    // One limiter PER AXIS -- a single shared limiter cross-couples X and Y. Rotation is left
    // unlimited so turning stays crisp. TODO tune rate (per-second) to driver preference on robot.
    private static final double kTranslationSlewRate = 2.5;
    private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(kTranslationSlewRate);

    // DRIVE-ONLY MODE (team request 2026-08-30): in TELEOP the driver gets the two sticks,
    // MENU re-zero, DPAD LEFT/RIGHT hood steps, and LB (hood back to this enable's floor in
    // one press -- kept 2026-08-30 so a run of DPAD steps can always be undone). Every intake,
    // hopper and shot binding, plus DPAD UP/DOWN (RT speed trim), is simply not registered
    // while this is true. Set false to bring the full match layer back; that one
    // flag is the whole switch, so nothing had to be deleted to get here.
    // The subsystems are still CONSTRUCTED on purpose -- construction is what applies their
    // current limits and brake modes, so skipping it would leave whatever was last flashed.
    // With no command ever scheduled, IntakeSubsystem.periodic() idles into its roller brake
    // and HopperSubsystem.periodic() is telemetry-only, so no motor is driven.
    // NOT AFFECTED, deliberately: the TEST-mode layer (configureTestBindings) is untouched --
    // flipping the DS to Test still exercises every mechanism -- and the "Forward 3 m then
    // Left 3 m intaking" auto still runs the intake, since this is a teleop-layer request.
    // OFF as of 2026-09-30 (team request, V2 launcher bring-up): the full match layer is live.
    // Everything below stays in place so this is a one-word switch if drive-only is wanted again.
    private static final boolean DRIVE_ONLY_MODE = false;

    // Intake-live slowdown: halve translation while the intake rollers spin (LT/Y hold,
    // intake or outtake) so the extended intake can't be rammed at full speed. Applied
    // BEFORE the slew limiter, so the 50% drop/restore ramps instead of stepping.
    // Rotation is untouched. TODO tune factor with driver.
    private static final double kRollerSlowFactor = 0.5;
    // RT shot slowdown (team spec 2026-09-30): while RT is held the drive still works, but at
    // 10% of max -- translation AND rotation. Wins over the intake slowdown.
    private static final double kShotDriveFactor = 0.1;
    // RB shoot-on-the-move translation cap (team 2026-10-09): 20% of max, ~0.8 m/s. Wins over
    // both slowdowns above -- RB also runs the launcher, so isShooting() is true during it.
    // TODO raise once the shot table + flight times are tuned (lead error grows with speed).
    private static final double kSotmDriveFactor = 0.2;
    private double driveScale() {
        if (shotCalc.isActive()) return kSotmDriveFactor;
        if (launcherSS.isShooting()) return kShotDriveFactor;
        return intakeSS.rollersRunning() ? kRollerSlowFactor : 1.0;
    }

    /** Deadband the raw stick, then square it (sign-preserving) for finer low-speed control. */
    /** Names (without .json) of the BLine path files in a folder, sorted; empty if the folder is missing. */
    public static List<String> blinePathNames(File pathsDir) {
        File[] files = pathsDir.listFiles((dir, n) -> n.endsWith(".json"));
        if (files == null) {
            return List.of();
        }
        return Arrays.stream(files)
            .map(f -> f.getName().substring(0, f.getName().length() - ".json".length()))
            .sorted()
            .toList();
    }

    private static double shapeAxis(double raw) {
        double v = MathUtil.applyDeadband(raw, 0.1);
        return Math.copySign(v * v, v);
    }

    // Auto chooser -- populated from the PathPlanner autos on the roboRIO (deploy/pathplanner/autos).
    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick2 = new CommandXboxController(0);

    // Full robot: all subsystems constructed.
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // V2 launcher (2026-09-30). The V1 ShooterSubsystem was deleted (last in commit ccd5075).
    public final LauncherSubsystem launcherSS = new LauncherSubsystem();
    public final IntakeSubsystem intakeSS = new IntakeSubsystem();
    public final HopperSubsystem hopperSS = new HopperSubsystem();
    // Match clock / HUB-shift tracker: publishes Match/* to NT for the companion field
    // map and rumbles the controller 3-2-1 before each HUB swap. No motors, no bindings.
    private final MatchStatus matchStatus = new MatchStatus(joystick2.getHID());
    // Limelight LED-off + shooter-idle low-res pipeline + Hailo over-temp warning. Auto-runs
    // via the scheduler; "shooter active" = flywheels commanded to spin OR RB auto-aim held
    // (both need the full-res pipeline).
    public final LimelightThermalManager limelightThermal = new LimelightThermalManager(
        Vision.CAM_LIMELIGHT, () -> launcherSS.isShooting() || joystick2.getHID().getRightBumperButton());
    // RB shoot-on-the-move solution (virtual target, heading, table rpm/hood), every loop.
    private final ShotCalculator shotCalc = new ShotCalculator(drivetrain);

    public RobotContainer() {
        // Build the auto chooser AFTER the drivetrain constructor has run AutoBuilder.configure.
        // If PathPlanner config failed to load (see configurePathPlanner), fall back to a
        // do-nothing chooser instead of crashing robot code on boot.
        SendableChooser<Command> chooser;
        if (AutoBuilder.isConfigured()) {
            try {
                // No default-auto argument -> the chooser defaults to "None" (do nothing).
                // SAFETY: the drive team must deliberately SELECT an auto on the dashboard;
                // otherwise an incidental auto-enable would drive the robot on its own.
                chooser = AutoBuilder.buildAutoChooser();
            } catch (Exception e) {
                // The .auto/.path files on disk are still stamped version 2025.0 -- if the 2026
                // parser rejects them, fail LOUDLY into a do-nothing auto instead of crashing
                // robot code on boot. (Re-save the files in the PathPlanner 2026 GUI to migrate.)
                DriverStation.reportError("PathPlanner auto files failed to load: " + e.getMessage(), e.getStackTrace());
                chooser = new SendableChooser<>();
                chooser.setDefaultOption("None (auto files failed to load)", Commands.none());
            }
        } else {
            chooser = new SendableChooser<>();
            chooser.setDefaultOption("None (AutoBuilder NOT configured)", Commands.none());
        }
        autoChooser = chooser;
        // Basic drive-forward auto (aborts on any drive/steer motor stall). Added here rather than
        // as a PathPlanner file so it's selectable even if the .auto/.path files fail to load, and
        // needs no AutoBuilder config -- it's pure odometry + swerve requests.
        // Both autos are bracketed by a pose reset (team request 2026-08-29): full reset to the
        // alliance start pose first, then X/Y-only back to it at the end -- the end reset keeps
        // the live heading on purpose, see resetAutoEndTranslation.
        autoChooser.addOption("Drive Forward 3 m @ 1 m/s",
                drivetrain.resetToAutoStartPose()
                        .andThen(drivetrain.driveForwardAuto(3.0, 1.0))
                        .andThen(drivetrain.resetAutoEndTranslation()));
        // Same building blocks, run back to back: forward leg then a left (+Y) strafe leg.
        autoChooser.addOption("Forward 3 m then Left 3 m @ 1 m/s",
                drivetrain.resetToAutoStartPose()
                        .andThen(drivetrain.driveForwardAuto(3.0, 1.0))
                        .andThen(drivetrain.driveLeftAuto(3.0, 1.0))
                        .andThen(drivetrain.resetAutoEndTranslation()));
        // Same as above, but the LEFT leg runs the intake -- exactly what holding LT does in
        // teleop (team request 2026-08-29). The left leg is the DEADLINE: when it ends the
        // intake group is cancelled, its finallyDo stops the rollers, and stowCommand then
        // retracts the slider until stall. Same group and same release order as the LT binding
        // at configureBindings, reused rather than rebuilt.
        autoChooser.addOption("Forward 3 m then Left 3 m intaking @ 1 m/s",
                drivetrain.resetToAutoStartPose()
                        .andThen(drivetrain.driveForwardAuto(3.0, 1.0))
                        .andThen(drivetrain.driveLeftAuto(3.0, 1.0)
                                .deadlineFor(intakeSS.intakeCommand()
                                        .andThen(hopperSS.intakeFeedCommand())
                                        .finallyDo(intakeSS::stopRollers)))
                        .andThen(intakeSS.stowCommand())
                        .andThen(drivetrain.resetAutoEndTranslation()));
        // BLine paths (team 2026-10-09): every deploy/autos/paths/*.json is added automatically,
        // labelled with the companion start spot it begins at. Pick that spot or the path refuses
        // to move. A bad file reports to the DS and is skipped instead of crashing boot.
        for (String name : blinePathNames(new File(Filesystem.getDeployDirectory(), "autos/paths"))) {
            try {
                Path path = new Path(name);
                String spot = CommandSwerveDrivetrain.startSpotFor(path);
                autoChooser.addOption("BLine: " + name + " (" + (spot != null ? spot : "NO START SPOT - won't run") + ")",
                        drivetrain.followBLinePath(path));
            } catch (RuntimeException e) {
                DriverStation.reportError("BLine path '" + name + "' failed to load: " + e.getMessage(), e.getStackTrace());
            }
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        configureTestBindings();

        // Read-only current/temp/voltage publisher for the driver companion app (app optional).
        new PowerTelemetry(drivetrain, launcherSS, intakeSS, hopperSS);

        // Live PID tuning for the driver-companion "PID Tuning" panel. All of this no-ops when
        // SubsystemConstants.TUNING_MODE is false (the competition default).
        // Swerve gains are published READ-ONLY: Tuner X owns TunerConstants.java, and a
        // fat-fingered steer kP is a violent mechanism. Read off the public module constants so
        // regenerating that file stays safe. Changing them still needs a redeploy.
        Tunable.publishReadOnly("Swerve/Steer/kP", TunerConstants.FrontLeft.SteerMotorGains.kP);
        Tunable.publishReadOnly("Swerve/Steer/kD", TunerConstants.FrontLeft.SteerMotorGains.kD);
        Tunable.publishReadOnly("Swerve/Drive/kP", TunerConstants.FrontLeft.DriveMotorGains.kP);
        Tunable.publishReadOnly("Swerve/Drive/kV", TunerConstants.FrontLeft.DriveMotorGains.kV);
        // Must run AFTER every subsystem is constructed -- this is what the panel enumerates.
        Tunable.publishKeys();
    }

    private void configureBindings() {
        // NOTE: the driver-companion "Controls" panel (driver-companion/src/renderer/
        // panels.ts, CONTROLS table) mirrors these bindings by hand — update it when
        // changing anything here.
        //
        // TEAM SPEC 2026-09-30 (V2 launcher; this is the FULL binding list -- no other keybinds
        // may exist):
        //   L stick     = translate (field-centric)      R stick X = rotate
        //   LT (hold)   = intake (slider out -> rollers + belts); release = stow
        //   Y (hold)    = outtake (same choreography, rollers out); release = stow
        //   X           = manual stow
        //   A (hold)    = manual hopper belts only
        //   VIEW (hold) = unjam: reverse belts + feeder (slow); intake untouched
        //   RT (hold)   = SHOT: flywheels at the DPAD-set rpm + feeder; belts (and feeder, slowly) reverse for the first
        //                 HOPPER_SHOT_REVERSE_SEC, then run ONLY while both are at speed; slider extends 0.5 s then stows, repeating (rollers in at 10%); drive 10%.
        //                 Release: flywheels/feeder coast, slider stows. Hood NOT commanded.
        //   DPAD L/R    = RT flywheel target -/+ 100 flywheel RPM (start 2000, 0-4000)
        //   DPAD UP/DN  = (hold) hood up/down at 1 deg/s (0-29 deg above the bottom stop; the hood
        //                 re-homes on every enable and ignores the DPAD until homed)
        //   MENU        = manual heading re-zero
        //   RB (hold)   = SHOOT ON THE MOVE: left stick still drives (capped at 20%), the robot
        //                 faces the lead-corrected hub, flywheel rpm + hood follow the shot
        //                 table, and the belts feed (RT's choreography) only while aimed, hood
        //                 at target and flywheels + feeder at speed. RB and RT exclude each other.
        // REMOVED 2026-09-30: LB hood-to-floor, B feed shot, old DPAD trim/hood steps.

        //DRIVE SUBSYSTEM
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            // Reverted 2026-07-17: translation back on LEFT stick, rotation back on RIGHT-X
            // (the 07-13 layout) -- undoes the 07-16 left->right stick swap.
            // Shared stick pipeline (raw -> deadband -> square -> intake slowdown -> slew ->
            // scale), used by BOTH the default drive and the RT aim-lock layer: same limiter
            // objects, so the handoff into RT mid-strafe is seamless (the ~20 ms gap between
            // the two commands is negligible for the slew clock).
                final java.util.function.DoubleSupplier driveVelX =
                    () -> xSlewLimiter.calculate(shapeAxis(-joystick2.getLeftY()) * driveScale()) * MaxSpeed; // Forward = negative LEFT-Y (WPILib convention)
                final java.util.function.DoubleSupplier driveVelY =
                    () -> ySlewLimiter.calculate(shapeAxis(-joystick2.getLeftX()) * driveScale()) * MaxSpeed; // Left = negative LEFT-X
                final java.util.function.DoubleSupplier driveRot =
                    () -> shapeAxis(-joystick2.getRightX()) * MaxAngularRate
                        * (launcherSS.isShooting() ? kShotDriveFactor : 1.0); // Rotate on RIGHT-X, counterclockwise; 10% during RT
                // While RB runs (shotCalc.isActive()) the SAME default command swaps to the aim request:
                // same sticks + slew limiters, heading servoed to the shot. Aiming here instead
                // of in an RB drivetrain command keeps the limiters continuous (no stop/restart
                // dip on press or release) and leaves MENU re-zero usable mid-shot.
                drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically.
                    drivetrain.applyRequest(() -> {
                        if (shotCalc.isActive()) {
                            ShotCalculator.Shot shot = shotCalc.getShot();
                            return drivetrain.faceHeading(driveVelX.getAsDouble(), driveVelY.getAsDouble(),
                                shot.heading(), shot.headingRateRadPerSec());
                        }
                        return drive.withVelocityX(driveVelX.getAsDouble())
                            .withVelocityY(driveVelY.getAsDouble())
                            .withRotationalRate(driveRot.getAsDouble());
                    })
                    // Zero the limiters every time the default command (re)starts -- after auto or
                    // any other command releases the drivetrain. SlewRateLimiter
                    // bounds change by rate*elapsed-since-last-calculate, so after a multi-second gap
                    // the first calculate() would otherwise jump straight to the stick with no limit.
                    .beforeStarting(() -> {
                        xSlewLimiter.reset(0);
                        ySlewLimiter.reset(0);
                    })
                );

            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                    drivetrain.applyRequest(() -> idle).ignoringDisable(true)
                );


            // MENU = manual heading re-zero (re-added 2026-07-17 by team request): point the
            // SHOOTER SIDE away from the driver, press once. Since 2026-07-21 (team request)
            // the AprilTag auto-seed (CommandSwerveDrivetrain.updateVisionPose) stops at the
            // first enable after boot, so MENU is the ONLY way to re-center the field during
            // a match (collision skew, gyro drift, camera down -- all of it lands here).
                joystick2.start().and(RobotModeTriggers.teleop()).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


            drivetrain.registerTelemetry(logger::telemeterize);

        // DRIVE-ONLY MODE STOPS HERE (team request 2026-08-30). Everything above -- the two
        // sticks, the disabled-idle that applies brake neutral, MENU re-zero -- stays live.
        // Everything below (intake, hopper, launcher) is never registered. Flip DRIVE_ONLY_MODE to false to restore it all.
            if (DRIVE_ONLY_MODE) {
                return;
            }

        //INTAKE + HOPPER FEED
            // LT (hold) = intake: slider extends until stall-stop, THEN rollers spin in AND
            // belts run -- kicker stays OFF during intake (team spec 2026-07-18; intakeFeedCommand
            // starts only after intakeCommand's sequence finishes, i.e. never while the slider
            // moves). Release: the hold cancels the group (belts stop), then stowCommand stops
            // the rollers unconditionally FIRST and retracts the slider until stall. A stalled
            // slider move is complete -- stow never re-pushes.
            // finallyDo = interrupt-safety: if another binding steals the shared hopper
            // (RT/RB/B) the group cancels WITHOUT onFalse firing -- without this the roller
            // state machine stays latched in periodic() with no command owning intakeSS.
            // Redundant on normal release (stowCommand stops rollers again), harmless.
            // RE-ENABLED 2026-08-22 (team request), exactly as it was before the disable.
            joystick2.leftTrigger().and(RobotModeTriggers.teleop())
                .whileTrue(intakeSS.intakeCommand().andThen(hopperSS.intakeFeedCommand())
                    .finallyDo(intakeSS::stopRollers))
                .onFalse(intakeSS.stowCommand());
            // Y (hold) = outtake: same choreography, rollers out. (Moved to A earlier on
            // 2026-08-29, then straight back to Y the same day -- team request.)
            joystick2.y().and(RobotModeTriggers.teleop())
                .whileTrue(intakeSS.outtakeCommand().andThen(hopperSS.intakeFeedCommand())
                    .finallyDo(intakeSS::stopRollers))
                .onFalse(intakeSS.stowCommand());
            // X = manual stow: stop rollers immediately, then retract slider until stall.
            joystick2.x().and(RobotModeTriggers.teleop()).onTrue(intakeSS.stowCommand());
            // A (hold) = MANUAL hopper run: belts ONLY, kicker stays OFF (team spec 2026-08-22).
            // Moved off B 2026-08-29 to give B the feed shot.
            joystick2.a().and(RobotModeTriggers.teleop()).whileTrue(hopperSS.manualRunCommand());
            // VIEW (hold) = UNJAM: belts backward + feeder backward super slowly (team 2026-10-07).
            // No intake (team 2026-10-09): the slider and rollers stay wherever they are.
            // (Direction-test diagnostics removed 2026-07-18 after the fix was confirmed: the
            // belts were fighting from opposite flashed inversion; both motors now same-sign.)
            // V2: the kicker is gone, so the launcher feeder reverses with the belts instead.
            joystick2.back().and(RobotModeTriggers.teleop())
                .whileTrue(hopperSS.unjamCommand().alongWith(launcherSS.feederUnjamCommand()));

        //LAUNCHER (V2, team spec 2026-09-30)
            // RT (hold) = SHOT. The launcher spins the flywheels (DPAD-set rpm) and the feeder
            // together; the hopper belts run ONLY while both are at speed, re-checked every loop,
            // so a ball dipping the wheels pauses the belts until they recover. The intake
            // slider extends for 0.5 s, retracts to full stow, and repeats (rollers in at 10%) for the whole hold. The drivetrain is NOT
            // required -- the default drive keeps running at 10% (driveScale). kCancelIncoming:
            // releasing RT is the only way out. Release: flywheels/feeder coast, slider stows.
            joystick2.rightTrigger().and(RobotModeTriggers.teleop())
                .whileTrue(launcherSS.shootCommand()
                    .alongWith(
                        hopperSS.feedShooterCommand(launcherSS::isReadyToFeed),
                        intakeSS.sliderCycleCommand())
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                .onFalse(intakeSS.stowCommand());
            // DPAD LEFT/RIGHT (press) = RT flywheel target -/+ one step. DPAD UP/DOWN (hold) =
            // hood moves up/down at a constant rate. No subsystem requirement, so they work mid-shot.
            joystick2.povLeft().and(RobotModeTriggers.teleop())
                .onTrue(launcherSS.stepFlywheelRpmCommand(-LauncherConstants.FLYWHEEL_STEP_RPM));
            joystick2.povRight().and(RobotModeTriggers.teleop())
                .onTrue(launcherSS.stepFlywheelRpmCommand(LauncherConstants.FLYWHEEL_STEP_RPM));
            joystick2.povUp().and(RobotModeTriggers.teleop())
                .whileTrue(launcherSS.jogHoodCommand(LauncherConstants.HOOD_JOG_DEG_PER_SEC));
            joystick2.povDown().and(RobotModeTriggers.teleop())
                .whileTrue(launcherSS.jogHoodCommand(-LauncherConstants.HOOD_JOG_DEG_PER_SEC));
            // RB (hold) = SHOOT ON THE MOVE (redesigned 2026-10-09 after LUNATECS, Ninjas 4744
            // and 1833). Aiming happens in the default drive while shotCalc is active (see above). The
            // group is RT's shot with the table rpm/hood, and the belts also wait for aim + hood.
            // kCancelIncoming, like RT: whichever of RB/RT is pressed first wins. Release:
            // flywheels/feeder coast, slider stows, the drive goes back to normal sticks.
            // TODO verify on robot in open space, Limelight mount pose set first: robot must turn
            // TOWARD the hub, and the field map must show the robot where it really is.
            joystick2.rightBumper().and(RobotModeTriggers.teleop())
                .whileTrue(launcherSS.autoShootCommand(
                        () -> shotCalc.getShot().flywheelRpm(), () -> shotCalc.getShot().hoodDeg())
                    .alongWith(
                        hopperSS.feedShooterCommand(() -> launcherSS.isReadyToFeed()
                            && launcherSS.isHoodAtTarget() && shotCalc.isAimed()),
                        intakeSS.sliderCycleCommand())
                    .beforeStarting(() -> shotCalc.setActive(true))
                    .finallyDo(() -> shotCalc.setActive(false))
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                .onFalse(intakeSS.stowCommand());
    }

    /**
     * TEST MODE bindings (Driver Station Test mode ONLY -- 2026-07-21 team request). The
     * SAME physical controller (joystick2, port 0) that carries the match bindings above
     * (each gated .and(RobotModeTriggers.teleop())) also carries this per-mechanism test
     * layer (each gated .and(RobotModeTriggers.test())). The two gates are mutually
     * exclusive -- the DS is only ever in one mode at a time -- so a physical button safely
     * means two different things depending on DS mode. Every bind below reuses an existing
     * command factory and existing tuned constants; nothing here uses vision or auto-aim.
     */
    private void configureTestBindings() {
        //INTAKE
            // LT / RT (hold) = rollers in / out, isolated (no slider, no hopper).
            joystick2.leftTrigger().and(RobotModeTriggers.test())
                .whileTrue(intakeSS.rollerTestCommand(true));
            joystick2.rightTrigger().and(RobotModeTriggers.test())
                .whileTrue(intakeSS.rollerTestCommand(false));
            // DPAD UP / DOWN (press) = slider/jackshaft extend / retract. CAN 22 (in code as
            // intakeSliderMotor) IS the jackshaft -- team-confirmed 2026-07-21, there is no
            // separate 3rd intake motor. Self-terminating (stall-cutoff/timeout): a press is
            // enough, holding the button does nothing extra.
            joystick2.povUp().and(RobotModeTriggers.test())
                .onTrue(intakeSS.extendSliderCommand());
            joystick2.povDown().and(RobotModeTriggers.test())
                .onTrue(intakeSS.retractSliderCommand());

        //HOPPER
            // LB / RB (hold) = belt A / B ALONE -- the same per-motor isolation diagnostic
            // that already caught a real fighting-inversion bug on this robot (2026-07-18).
            joystick2.leftBumper().and(RobotModeTriggers.test())
                .whileTrue(hopperSS.directionTest(true));
            joystick2.rightBumper().and(RobotModeTriggers.test())
                .whileTrue(hopperSS.directionTest(false));
            // X (hold) = both belt motors together, no kicker.
            joystick2.x().and(RobotModeTriggers.test())
                .whileTrue(hopperSS.intakeFeedCommand());

    }

    public Command getAutonomousCommand() {
        // Run whatever auto the drive team picked on the dashboard (chooser defaults to "None").
        Command selected = autoChooser.getSelected();
        return (selected != null) ? selected : Commands.none();
    }
}
