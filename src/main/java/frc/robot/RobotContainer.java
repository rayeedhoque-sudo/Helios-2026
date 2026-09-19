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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.TunerConstants;
import frc.robot.util.Tunable;
import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.misc.HopperSubsystem;
import frc.robot.subsystems.misc.IntakeSubsystem;
import frc.robot.subsystems.misc.MatchStatus;
import frc.robot.subsystems.misc.ShooterSubsystem;
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
    // Zero-output request used to FREEZE the drivetrain while a shot is held. The drive motors
    // are Brake-neutral (TunerConstants), so Idle coasts to a stop and then holds -- a low-current
    // "disabled" that still plants the robot. Reused every loop by lockDriveAndIntake().
    private final SwerveRequest.Idle shotFreeze = new SwerveRequest.Idle();

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
    // Spin-up clock for the RT shot's kicker: restarted on every press, so the kicker stays
    // shut for KICKER_SPINUP_DELAY_SEC while the flywheels wind up.
    private final Timer kickerSpinupTimer = new Timer();
    // DPAD UP/DOWN step for the RT flywheel target, in motor RPM (team request 2026-08-22;
    // 200 -> 50 on 2026-08-28, 50 -> 25 on 2026-08-29, each for a finer trim).
    private static final double kRtSpeedTrimRpm = 25.0;
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
    // OFF as of 2026-08-30 (team request): the full match layer is live again. Everything
    // below stays in place so this is a one-word switch if drive-only is wanted again.
    private static final boolean DRIVE_ONLY_MODE = true;

    // KILL SWITCH for the teleop DPAD hood moves (2026-08-27). false = the bindings are not
    // registered at all, so nothing can drive the hood from the driver station; the hood still
    // holds on its brake and commanded shot angles are unaffected. See the bindings below.
    // ON (team direction 2026-08-27: "in teleop the hood should go up 5 degrees every
    // press"), with the overshoot accepted knowingly. THE RISK, restated so it is not
    // rediscovered on the belt: a powered hood steps 25-55 raw units per 20 ms loop, so ONE
    // press actually moves 25-55 units (3-7 physical deg), not 10 -- no loop can resolve a move
    // shorter than one sample of its own travel, closed loop included. Driving softer is not
    // available either: the hood does not break away below ~7.5 V. So each press is roughly one
    // physical "notch" of ~3-7 deg, and about 6-12 of them reach the ceiling guard at base +
    // one full travel, which is what protects the top stop.
    // Set false to kill the teleop bindings outright; the test-mode bindings are separate.
    private static final boolean HOOD_DPAD_MOVES_ENABLED = true;

    // Hood: DPAD LEFT/RIGHT are PRESSED to step the hood one HOOD_UP_STEP_UNITS DOWN / UP from
    // where it currently sits -- in teleop AND in test mode, the same closed-loop
    // move in both (ShooterSubsystem.moveHoodToCommand). Since 2026-08-28 there is no hood
    // voltage binding of any kind: the position loop is the only thing that drives it.

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
    public final ShooterSubsystem shooterSS = new ShooterSubsystem(drivetrain);
    public final IntakeSubsystem intakeSS = new IntakeSubsystem();
    public final HopperSubsystem hopperSS = new HopperSubsystem();
    // Match clock / HUB-shift tracker: publishes Match/* to NT for the companion field
    // map and rumbles the controller 3-2-1 before each HUB swap. No motors, no bindings.
    private final MatchStatus matchStatus = new MatchStatus(joystick2.getHID());
    // Limelight LED-off + shooter-idle low-res pipeline + Hailo over-temp warning. Auto-runs
    // via the scheduler; "shooter active" = flywheels commanded to spin (needs full-res ranging).
    public final LimelightThermalManager limelightThermal =
        new LimelightThermalManager(Vision.CAM_LIMELIGHT, () -> shooterSS.getDesiredFlywheelVelocity() != 0);

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
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        configureTestBindings();

        // Read-only current/temp/voltage publisher for the driver companion app (app optional).
        new PowerTelemetry(drivetrain, shooterSS, intakeSS, hopperSS);

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
        // TEAM SPEC 2026-07-16 (shooter re-enabled by team directive; this is the FULL
        // binding list — no other keybinds may exist):
        //   L stick     = translate (field-centric)      R stick X = rotate  (reverted 2026-07-17)
        //   LB          = hood all the way DOWN to this enable's base position (2026-08-29)
        //   LT (hold)   = intake (slider out -> rollers + belts; kicker stays OFF); release = stow
        //   A (hold)    = outtake (same choreography, rollers out); release = stow (was Y
        //                 until 2026-08-29)
        //   X           = manual stow
        //   RT (hold)   = flywheels-only shot (2026-08-22): fixed flywheel speed, belts always,
        //                 kicker opens 2 s after the press, HOOD NOT COMMANDED (DPAD L/R).
        //                 Full drive + intake lockout; no vision, no auto-aim.
        //   RB (hold)   = AUTO-AIM SHOT (2026-08-29): constant flywheel speed, hood angle
        //                 looked up from the AprilTag distance, AND the robot turns itself to
        //                 face the hub CENTRE (not the tag). Belts always; the kicker runs
        //                 BACKWARD until the flywheels are at speed (spin-up delay as the
        //                 backstop), then feeds. Sticks locked out for the hold.
        //   (Kicker at-speed gate added 2026-07-18 by team request.)
        //   B (hold)    = manual hopper belts only (kicker OFF)
        //   VIEW (hold) = hopper unjam: reverse belts + kicker (added 2026-07-18)
        //   Y (hold)    = fixed feed shot (2026-08-29): fixed hood units + fixed flywheel
        //                 RPM, otherwise identical to RT. (No search-align binding is left
        //                 on the match layer -- A used to hold it.)
        //   DPAD-UP/DOWN       = RT flywheel target +/- 25 motor RPM per press (2026-08-22;
        //                        200 -> 50 -> 25 by 2026-08-29). Does NOT affect RB's auto-aim
        //                        speed.
        //   DPAD-LEFT/RIGHT    = step the hood DOWN / UP HOOD_UP_STEP_UNITS per press, on the
        //                        position loop (2026-08-27/28; replaced the held voltage jog,
        //                        which replaced the +-90 deg heading snaps)
        //   MENU        = manual heading re-zero -- the ONLY in-match re-center (2026-07-21:
        //                 AprilTag auto-seed now stops at the first enable after boot)
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

            // A used to be search-align (rotate until a SCORING tag of our alliance is seen,
            // then face it). REASSIGNED to outtake 2026-08-29 by team request; there is no
            // search-align binding left on the match layer. searchAndAlignCommand and
            // seesScoringTag/getDegreesToAlignToTarget are all still there if it comes back.
            // DPAD-UP / DOWN (press) = trim the RT flywheel target by +/- kRtSpeedTrimRpm motor
            // RPM (team
            // request 2026-08-22). ~1.6 m/s of surface speed per press. Takes effect mid-hold:
            // flywheelOnlyShotCommand re-reads the target every loop. Clamped to
            // [0, SHOT_MAX_MOTOR_RPS] in the subsystem, and NOT persisted -- a redeploy returns
            // it to RT_FLYWHEEL_SURFACE_SPEED. Trims RT ONLY -- RB's auto-aim speed is a
            // separate constant and is not trimmable from the controller.
            // Not registered in DRIVE_ONLY_MODE -- it trims the RT shot, and RT is gone.
                if (!DRIVE_ONLY_MODE) {
                    joystick2.povUp().and(RobotModeTriggers.teleop())
                        .onTrue(shooterSS.trimRtSpeedCommand(kRtSpeedTrimRpm));
                    joystick2.povDown().and(RobotModeTriggers.teleop())
                        .onTrue(shooterSS.trimRtSpeedCommand(-kRtSpeedTrimRpm));
                }

            // DPAD-RIGHT (PRESS) = raise the hood by HOOD_UP_STEP_UNITS, EVERY PRESS (team
            // direction 2026-08-27). The target is read off the hood itself at the moment of
            // the press -- current + step -- not off a stored setpoint, so presses keep working
            // from wherever the hood actually ended up (see ShooterSubsystem.getHoodAngle).
            // DPAD-LEFT is the mirror: one step DOWN from wherever the hood is (team request
            // 2026-08-28, replacing the old "return to this enable's base in one press"). Both
            // read the live angle, so a press always steps from reality; clampDesiredAngle still
            // stops LEFT at the enable-time floor and RIGHT at the ceiling.
            //
            // CLOSED LOOP since 2026-08-27 (team direction: "make the DPAD use PID"). A press
            // writes a setpoint and nothing else; the position loop in periodic() drives it,
            // bounded by the settle band, the voltage caps and the travel guards. Read
            // ShooterSubsystem.moveHoodToCommand before tuning this -- it needs a NON-ZERO kP
            // (>= ~0.17) to break the hood away at all, and there is no move timeout any more.
                if (HOOD_DPAD_MOVES_ENABLED) {
                    // LB (press) = send the hood all the way DOWN to this enable's base position
                    // in one press (team request 2026-08-29), replacing the X-lock brake toggle
                    // that used to live here. Same closed-loop path as the DPAD steps -- it just
                    // asks for the floor instead of one step. getHoodFloorAngle IS the
                    // clampDesiredAngle floor, so the descent stops at the enable-time datum and
                    // cannot be driven into the bottom stop. Inside the kill switch with the
                    // DPAD moves, so HOOD_DPAD_MOVES_ENABLED = false still kills every
                    // driver-station hood move.
                    // LB stays live in DRIVE_ONLY_MODE (team request 2026-08-30): it is the
                    // one-press way back to this enable's hood floor, so the driver can always
                    // undo a run of DPAD steps.
                    joystick2.leftBumper().and(RobotModeTriggers.teleop())
                        .onTrue(shooterSS.moveHoodToCommand(shooterSS::getHoodFloorAngle));
                    joystick2.povLeft().and(RobotModeTriggers.teleop())
                        .onTrue(shooterSS.moveHoodToCommand(
                            () -> shooterSS.getHoodAngle()
                                - ShooterSubsystemConstants.HOOD_UP_STEP_UNITS));
                    joystick2.povRight().and(RobotModeTriggers.teleop())
                        .onTrue(shooterSS.moveHoodToCommand(
                            () -> shooterSS.getHoodAngle()
                                + ShooterSubsystemConstants.HOOD_UP_STEP_UNITS));
                }

            drivetrain.registerTelemetry(logger::telemeterize);

        // DRIVE-ONLY MODE STOPS HERE (team request 2026-08-30). Everything above -- the two
        // sticks, the disabled-idle that applies brake neutral, MENU re-zero, LB hood-to-floor
        // and the DPAD LEFT/RIGHT hood steps -- stays live. Everything below (intake, hopper, and all four
        // shot bindings) is never registered. Flip DRIVE_ONLY_MODE to false to restore it all.
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
            // VIEW (hold) = UNJAM: reverse belts + kicker at low duty to back a stuck ball out.
            // (Direction-test diagnostics removed 2026-07-18 after the fix was confirmed: the
            // belts were fighting from opposite flashed inversion; both motors now same-sign.)
            joystick2.back().and(RobotModeTriggers.teleop()).whileTrue(hopperSS.unjamCommand());

        //SHOOTER (re-enabled by team directive 2026-07-16)
            // SHOT LOCKOUT (team request 2026-07-17): while RT or RB is held, ONLY the shooter +
            // hopper/kicker act. Both groups REQUIRE drivetrain + intake and run kCancelIncoming,
            // so every drive / intake / other binding is BLOCKED for the whole hold. The driver
            // can no longer ABORT a shot by pressing another control -- RELEASING the trigger is
            // the only way out (whileTrue cancels on release).
            //
            // RT (hold) = FLYWHEELS-ONLY SHOT (team request 2026-08-22, replaced the vision
            // auto-aim shot): spin the flywheels to RT_FLYWHEEL_SURFACE_SPEED, belts always,
            // kicker after a 2 s spin-up delay (KICKER_SPINUP_DELAY_SEC). The HOOD IS NOT COMMANDED -- it stays
            // wherever the DPAD jog left it, and it stays there on release too (no
            // stopShooterCommand, which no longer moves the hood at all). Aiming is the driver's
            // job: no vision, no auto-rotate. Same drive + intake lockout as RB, so aim BEFORE
            // pressing. Release: flywheels coast, hood unchanged.
            joystick2.rightTrigger().and(RobotModeTriggers.teleop())
                .whileTrue(shooterSS.flywheelOnlyShotCommand()
                    .alongWith(
                        // Belts always; kicker opens KICKER_SPINUP_DELAY_SEC after the trigger
                        // (team request 2026-08-22) instead of on isFlywheelAtSpeed(), which
                        // never opened with the velocity loop untuned. The timer restarts with
                        // the group below, so every press waits the full spin-up.
                        // NO REVERSE (team request 2026-08-28). The kicker used to run
                        // BACKWARDS for the whole spin-up window (2026-08-24) to hold fuel
                        // off the flywheels; it now simply sits stopped until the delay
                        // expires, as it did before that. Passing () -> false rather than
                        // deleting the parameter keeps feedShooterCommand's shape -- the
                        // commented-out RB binding below already passes false the same way.
                        hopperSS.feedShooterCommand(() -> true,
                            () -> kickerSpinupTimer.hasElapsed(
                                HopperSubsystemConstants.KICKER_SPINUP_DELAY_SEC),
                            () -> false),
                        lockDriveAndIntake())
                    .beforeStarting(kickerSpinupTimer::restart)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
            // B (hold) = FIXED FEED SHOT (team request 2026-08-29; it briefly sat on Y the
            // same day): hood to FEED_SHOT_HOOD_UNITS above this enable's floor,
            // flywheels to FEED_SHOT_MOTOR_RPM. Everything else is RT's behaviour, reusing RT's
            // parts: belts always, kicker only after KICKER_SPINUP_DELAY_SEC, same drive+intake
            // lockout, same kCancelIncoming hold. The hood is commanded ONCE at the press (see
            // feedShotCommand) and is NOT returned on release.
            joystick2.b().and(RobotModeTriggers.teleop())
                .whileTrue(shooterSS.feedShotCommand()
                    .alongWith(
                        hopperSS.feedShooterCommand(() -> true,
                            () -> kickerSpinupTimer.hasElapsed(
                                HopperSubsystemConstants.KICKER_SPINUP_DELAY_SEC),
                            () -> false),
                        lockDriveAndIntake())
                    .beforeStarting(kickerSpinupTimer::restart)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
            // RB (hold) = AUTO-AIM SHOT (team request 2026-08-29). Constant flywheel speed
            // (AUTOAIM_FLYWHEEL_MOTOR_RPM), belts always, hood commanded from the measured
            // AprilTag distance, and -- unlike every other shot -- THE ROBOT TURNS ITSELF to put
            // the shooter on the hub CENTRE (team request 2026-08-29).
            //
            // The rotation is why this does NOT use lockDriveAndIntake(): that freezes the
            // drivetrain with an Idle request, which is the opposite of what a self-aiming shot
            // needs. Instead the drivetrain runs the SAME heading servo the other align commands
            // use (aimUntilAligned), with () -> false so it never ends -- it holds aim for the
            // whole press. It still REQUIRES the drivetrain, so the driver's sticks are locked
            // out exactly as before; the robot simply turns instead of standing still.
            // Translation is zero throughout, so the robot pivots in place and the distance the
            // hood was set from does not change underneath it.
            //
            // It aims at the HUB CENTRE, not the tag. The tags sit off to one side of the hub
            // face (0.3556 m on tags 9/25, dead centre on 10/26 -- FieldConstants), which at 3 m
            // is ~6.8 deg, more than 3x the 2 deg aim tolerance. Aiming at the tag would clip
            // the rim on the offset faces.
            //
            // THE KICKER RUNS BACKWARD until the flywheels are at speed (or the spin-up delay
            // expires), then feeds -- never idle in between. The aim veto that briefly gated the
            // feed here was REMOVED 2026-08-29 after it stalled the shot on the robot; see the
            // gates below. If the kicker never
            // opens on the robot, the heading servo not converging is the first suspect --
            // watch "Aimed At Target" on the Shooter dashboard tab.
            //
            // The hood moving on its own here is a deliberate, team-approved exception to the
            // 2026-08-26 no-automatic-hood-motion rule -- see autoAimShotCommand.
            // Shares kickerSpinupTimer with RT: both groups require the drivetrain and run
            // kCancelIncoming, so only one of them can ever own the timer at a time.
            joystick2.rightBumper().and(RobotModeTriggers.teleop())
                .whileTrue(shooterSS.autoAimShotCommand()
                    .alongWith(
                        // THE KICKER IS NEVER IDLE DURING A SHOT: it is either backing fuel
                        // off the winding-up wheels, or feeding. The two gates below are exact
                        // complements, so there is no window where it just sits there.
                        //
                        // That window is the bug the team reported on 2026-08-29 ("the kicker
                        // stops running after the flywheels are at speed"). The forward gate
                        // used to be `timer AND isAimedAtTarget()`, so once the reverse ended
                        // the kicker sat STOPPED until the heading servo came within 2 deg --
                        // and if the servo never converged, forever. The aim veto is gone: with
                        // the servo unproven on the robot, a gate that can block the shot
                        // indefinitely is worse than a shot taken slightly off-aim. The robot
                        // still rotates; the aim just no longer vetoes the feed. Restore the
                        // `&& shooterSS.isAimedAtTarget()` once the servo is confirmed to
                        // converge -- "Aimed At Target" on the Shooter tab is the check.
                        hopperSS.feedShooterCommand(() -> true,
                            // FEED once the wheels are up -- or once the spin-up delay has run
                            // out, the backstop for the untuned velocity loop (isFlywheelAtSpeed
                            // has never reliably latched on this robot, which is why RT runs off
                            // a timer at all).
                            () -> shooterSS.isFlywheelAtSpeed()
                                || kickerSpinupTimer.hasElapsed(
                                    HopperSubsystemConstants.KICKER_SPINUP_DELAY_SEC),
                            // REVERSE until then (team request 2026-08-29), holding fuel off the
                            // winding-up wheels. Exactly the complement of the gate above.
                            () -> !shooterSS.isFlywheelAtSpeed()
                                && !kickerSpinupTimer.hasElapsed(
                                    HopperSubsystemConstants.KICKER_SPINUP_DELAY_SEC)),
                        // Turn to face the hub centre, and keep the intake locked out. Never
                        // ends -- the hold is what cancels it.
                        drivetrain.aimUntilAligned(
                            shooterSS::getAutoAimFieldHeadingDegrees, () -> false),
                        intakeSS.run(intakeSS::stopRollers))
                    .beforeStarting(kickerSpinupTimer::restart)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
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
            // B (hold) = kicker alone.
            joystick2.b().and(RobotModeTriggers.test())
                .whileTrue(hopperSS.kickerTestCommand());

        //SHOOTER (manual/fixed setpoints only -- no vision, no auto-aim)
            // Y (hold) = manual test-fire: feedAngleShotCommand's fixed hood angle + flywheel
            // speed (RB_FEED_ANGLE / RB_FEED_SURFACE_SPEED -- named for the binding RB used to
            // hold; RB is the auto-aim shot since 2026-08-29 and no longer shares this).
            // Belts always, kicker gated on at-speed. Actually launches a
            // ball (not just a flywheel spin), so it verifies the whole feed path end to end.
            joystick2.y().and(RobotModeTriggers.test())
                .whileTrue(shooterSS.feedAngleShotCommand()
                    .alongWith(hopperSS.feedShooterCommand(() -> true, shooterSS::isFlywheelAtSpeed,
                        () -> false)))
                .onFalse(shooterSS.stopShooterCommand());
            // DPAD LEFT / RIGHT (PRESS) = the SAME closed-loop hood moves as the match
            // bindings (ShooterSubsystem.moveHoodToCommand): right steps up by
            // HOOD_UP_STEP_UNITS from wherever the hood is, left steps down by the same amount.
            // Flywheels stay off; periodic() still applies the travel guard and the
            // feedback gate.
            //
            // The held OPEN-LOOP jog that used to live here is GONE (2026-08-28, team
            // direction: hood voltage throttle removed everywhere). CONSEQUENCE: the hood can
            // no longer be driven ONTO a hard stop from the controller -- every setpoint is
            // clamped to [enable datum, datum + travel] -- so the encoder-anchor calibration
            // (handoff on-robot verify item 2) has to be done by moving the hood by hand.
            joystick2.povLeft().and(RobotModeTriggers.test())
                .onTrue(shooterSS.moveHoodToCommand(
                    () -> shooterSS.getHoodAngle() - ShooterSubsystemConstants.HOOD_UP_STEP_UNITS));
            joystick2.povRight().and(RobotModeTriggers.test())
                .onTrue(shooterSS.moveHoodToCommand(
                    () -> shooterSS.getHoodAngle() + ShooterSubsystemConstants.HOOD_UP_STEP_UNITS));
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
