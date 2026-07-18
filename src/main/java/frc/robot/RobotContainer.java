// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.TunerConstants;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.misc.HopperSubsystem;
import frc.robot.subsystems.misc.IntakeSubsystem;
import frc.robot.subsystems.misc.MatchStatus;
import frc.robot.subsystems.misc.ShooterSubsystem;

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
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // Zero-output request used to FREEZE the drivetrain while a shot is held. The drive motors
    // are Brake-neutral (TunerConstants), so Idle coasts to a stop and then holds -- a low-current
    // "disabled" that still plants the robot. Reused every loop by lockDriveAndIntake().
    private final SwerveRequest.Idle shotFreeze = new SwerveRequest.Idle();

    // Smooth translation commands: caps how fast the drive command can change (units: full-stick
    // fraction per second; 4.0 = stop-to-full in 0.25 s). Softens acceleration spikes so 4 Krakens
    // don't slam their supply limit together (brownout/wheel-slip protection) without feeling laggy.
    // One limiter PER AXIS -- a single shared limiter cross-couples X and Y. Rotation is left
    // unlimited so turning stays crisp. TODO tune rate (per-second) to driver preference on robot.
    private static final double kTranslationSlewRate = 4.0;
    private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(kTranslationSlewRate);

    // Intake-live slowdown: halve translation while the intake rollers spin (LT/Y hold,
    // intake or outtake) so the extended intake can't be rammed at full speed. Applied
    // BEFORE the slew limiter, so the 50% drop/restore ramps instead of stepping.
    // Rotation is untouched. TODO tune factor with driver.
    private static final double kRollerSlowFactor = 0.5;
    private double driveScale() {
        return intakeSS.rollersRunning() ? kRollerSlowFactor : 1.0;
    }

    /** Deadband the raw stick, then square it (sign-preserving) for finer low-speed control. */
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
    public final ShooterSubsystem shoterSS = new ShooterSubsystem(drivetrain);
    public final IntakeSubsystem intakeSS = new IntakeSubsystem();
    public final HopperSubsystem hopperSS = new HopperSubsystem();
    // Match clock / HUB-shift tracker: publishes Match/* to NT for the companion field
    // map and rumbles the controller 3-2-1 before each HUB swap. No motors, no bindings.
    private final MatchStatus matchStatus = new MatchStatus(joystick2.getHID());

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
        autoChooser.addOption("Drive Forward 3 m @ 1 m/s", drivetrain.driveForwardAuto(3.0, 1.0));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();

        // Read-only current/temp/voltage publisher for the driver companion app (app optional).
        new PowerTelemetry(drivetrain, shoterSS, intakeSS, hopperSS);
    }

    private void configureBindings() {
        // NOTE: the driver-companion "Controls" panel (driver-companion/src/renderer/
        // panels.ts, CONTROLS table) mirrors these bindings by hand — update it when
        // changing anything here.
        //
        // TEAM SPEC 2026-07-16 (shooter re-enabled by team directive; this is the FULL
        // binding list — no other keybinds may exist):
        //   L stick     = translate (field-centric)      R stick X = rotate  (reverted 2026-07-17)
        //   LB          = toggle X-lock brake
        //   LT (hold)   = intake (slider out -> rollers + belts + low kicker); release = stow
        //   Y (hold)    = outtake (same choreography, rollers out); release = stow
        //   X           = manual stow
        //   RT (hold)   = precision vision shot: AUTO-AIM in place, THEN freeze drive + feed
        //                 (kicker UNGATED). Intake disabled all hold; drive disabled once aimed.
        //   RB (hold)   = fixed feed shot (38 deg hood + fixed speed). Kicker UNGATED.
        //                 Full drive + intake lockout (blind feed -- no target to aim at).
        //   (Both updated 2026-07-17.)
        //   B (hold)    = manual hopper belts + kicker, ungated
        //   A / DPAD-UP (hold) = search-align to our alliance's scoring tag
        //   DPAD-LEFT/RIGHT    = rotate exactly +90 / -90 deg
        //   MENU        = manual heading re-zero (backstop; AprilTags auto-zero when in view)
        // REMOVED 2026-07-16: DPAD hood jog, RT test shot. (MENU re-zero re-added 2026-07-17.)

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
                    () -> shapeAxis(-joystick2.getRightX()) * MaxAngularRate; // Rotate on RIGHT-X, counterclockwise
                drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically.
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(driveVelX.getAsDouble())
                            .withVelocityY(driveVelY.getAsDouble())
                            .withRotationalRate(driveRot.getAsDouble())
                    )
                    // Zero the limiters every time the default command (re)starts -- after auto, the
                    // brake button, or any other command releases the drivetrain. SlewRateLimiter
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

            // LB = enable/disable X-lock (toggle: press to lock wheels in an X, press to release).
                joystick2.leftBumper().toggleOnTrue(drivetrain.applyRequest(() -> brake));

            // MENU = manual heading re-zero (re-added 2026-07-17 by team request): point the
            // SHOOTER SIDE away from the driver, press once. Normally the AprilTag auto-seed
            // (CommandSwerveDrivetrain.updateVisionPose) keeps the pose zeroed on its own --
            // MENU is the recovery path when vision is out (no tags visible / camera down).
                joystick2.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

            // A / DPAD-UP (hold) = search-align: rotate slowly until a SCORING tag of OUR
            // alliance is seen, then face it (bearing re-sampled every loop). Two separate
            // command instances -- one instance on two triggers cross-cancels on release.
                joystick2.a().whileTrue(drivetrain.searchAndAlignCommand(
                    shoterSS::seesScoringTag,
                    () -> drivetrain.getState().Pose.getRotation().getDegrees()
                          + shoterSS.getDegreesToAlignToTarget()));
                joystick2.povUp().whileTrue(drivetrain.searchAndAlignCommand(
                    shoterSS::seesScoringTag,
                    () -> drivetrain.getState().Pose.getRotation().getDegrees()
                          + shoterSS.getDegreesToAlignToTarget()));

            // DPAD-LEFT/RIGHT = rotate exactly +90 (CCW) / -90 deg from the heading at press
            // (rotateBy defers, so the snapshot happens at schedule time, not at boot).
                joystick2.povLeft().onTrue(drivetrain.rotateBy(90));
                joystick2.povRight().onTrue(drivetrain.rotateBy(-90));

            drivetrain.registerTelemetry(logger::telemeterize);

        //INTAKE + HOPPER FEED
            // LT (hold) = intake: slider extends until stall-stop, THEN rollers spin in AND
            // belts + LOW-duty kicker run (intakeFeedCommand starts only after intakeCommand's
            // sequence finishes, i.e. never while the slider moves). Release: the hold cancels
            // the group (intakeFeedCommand's end stops belts + kicker), then stowCommand stops
            // the rollers unconditionally FIRST and retracts the slider until stall. A stalled
            // slider move is complete -- stow never re-pushes.
            // finallyDo = interrupt-safety: if another binding steals the shared hopper
            // (RT/RB/B) the group cancels WITHOUT onFalse firing -- without this the roller
            // state machine stays latched in periodic() with no command owning intakeSS.
            // Redundant on normal release (stowCommand stops rollers again), harmless.
            joystick2.leftTrigger()
                .whileTrue(intakeSS.intakeCommand().andThen(hopperSS.intakeFeedCommand())
                    .finallyDo(intakeSS::stopRollers))
                .onFalse(intakeSS.stowCommand());
            // Y (hold) = outtake: same choreography, rollers out.
            joystick2.y()
                .whileTrue(intakeSS.outtakeCommand().andThen(hopperSS.intakeFeedCommand())
                    .finallyDo(intakeSS::stopRollers))
                .onFalse(intakeSS.stowCommand());
            // X = manual stow: stop rollers immediately, then retract slider until stall.
            joystick2.x().onTrue(intakeSS.stowCommand());
            // B (hold) = MANUAL hopper run: belts + kicker, UNGATED (works with shooter idle).
            joystick2.b().whileTrue(hopperSS.manualRunCommand());

        //SHOOTER (re-enabled by team directive 2026-07-16)
            // SHOT LOCKOUT (team request 2026-07-17): while RT or RB is held, ONLY the shooter +
            // hopper/kicker act. Both groups REQUIRE drivetrain + intake and run kCancelIncoming,
            // so every drive / intake / other binding is BLOCKED for the whole hold. The driver
            // can no longer ABORT a shot by pressing another control -- RELEASING the trigger is
            // the only way out (whileTrue cancels on release).
            //
            // RT (hold) = precision vision shot, AUTO-AIM then FREEZE (team request 2026-07-17):
            //   phase 1 -- rotate IN PLACE to the vision firing bearing (aimUntilAligned); intake
            //             locked, flywheels already spinning up, NOTHING feeds yet.
            //   phase 2 -- once aimed on a REAL target: FREEZE the drivetrain (Idle -> brake) and
            //             feed (belts while hasShotTarget, kicker UNGATED). Drive is disabled only
            //             AFTER the aim is made, so the shot can never fire mid-rotation.
            //   No tag in view -> aim never latches -> holds heading, never freezes/feeds (release
            //   to exit; use A / DPAD to acquire a tag first). Release: flywheels coast, hood
            //   recesses once they slow (hood interlock), drive + intake unlock. Strafe is OFF
            //   during aim -- pre-position for distance first. (driveWithAimLockCommand now unused.)
            joystick2.rightTrigger()
                .whileTrue(shoterSS.visionShotCommand()
                    .alongWith(
                        // Intake stays disabled the WHOLE hold (never intake mid-shot).
                        intakeSS.run(intakeSS::stopRollers),
                        // Drivetrain: AUTO-AIM in place, THEN (once aimed on a real target) freeze
                        // + feed. Ending the aim on the shooter's own gate (present + isAimed)
                        // means no target -> never ends -> never fires a blind shot.
                        drivetrain.aimUntilAligned(
                                () -> shoterSS.getAimHeadingDegrees()
                                    .orElse(drivetrain.getState().Pose.getRotation().getDegrees()),
                                () -> shoterSS.getAimHeadingDegrees().isPresent()
                                    && shoterSS.isAimedAtTarget())
                            .andThen(Commands.parallel(
                                drivetrain.applyRequest(() -> shotFreeze),
                                hopperSS.feedShooterCommand(shoterSS::hasShotTarget, () -> true))))
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                .onFalse(shoterSS.stopShooterCommand());
            // RB (hold) = fixed feed shot: hood 38 deg + fixed tunable feed speed, belts always,
            // kicker UNGATED. Same drive + intake lockout as RT. Release: flywheels coast, hood
            // recesses once they slow.
            joystick2.rightBumper()
                .whileTrue(shoterSS.feedAngleShotCommand()
                    .alongWith(
                        hopperSS.feedShooterCommand(() -> true, () -> true),
                        lockDriveAndIntake())
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                .onFalse(shoterSS.stopShooterCommand());
    }

    /**
     * FREEZE the drivetrain AND lock the intake for the duration of a shot (team request
     * 2026-07-17: "no subsystem but hopper + kicker works while shooting"). Returned as a
     * never-ending parallel so it holds until the shot group is cancelled on trigger release.
     * The shot groups compose this in and run kCancelIncoming, so requiring these two
     * subsystems is what BLOCKS every drive / intake binding for the whole hold.
     */
    private Command lockDriveAndIntake() {
        return Commands.parallel(
            drivetrain.applyRequest(() -> shotFreeze), // requires drivetrain -> blocks default drive + all drive binds
            intakeSS.run(intakeSS::stopRollers)        // requires intake -> blocks LT/Y/X, keeps rollers braked
        );
    }

    public Command getAutonomousCommand() {
        // Run whatever auto the drive team picked on the dashboard (chooser defaults to "None").
        Command selected = autoChooser.getSelected();
        return (selected != null) ? selected : Commands.none();
    }
}
