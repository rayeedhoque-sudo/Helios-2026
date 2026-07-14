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
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.misc.HopperSubsystem;
import frc.robot.subsystems.misc.IntakeSubsystem;
import frc.robot.subsystems.misc.ShooterSubsystem;
import frc.robot.subsystems.misc.HopperSubsystem.HOPPERSTATE;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired tp speed
    // 1.0 rot/s teleop spin rate (physical ceiling ~1.9 rot/s at free speed). Raised from the
    // 0.75 template default for full-performance driving; push toward 1.5 only if the driver
    // wants it -- faster spin also demands more from the steer loop (see TunerConstants gains).
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // Deadband is applied to the RAW sticks in shapeAxis() below, so the request's own
            // deadband is left at 0 (applying it twice would eat real low-speed commands).
            .withDriveRequestType(DriveRequestType.Velocity); // Velocity = CLOSED-loop velocity control
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // Smooth translation commands: caps how fast the drive command can change (units: full-stick
    // fraction per second; 4.0 = stop-to-full in 0.25 s). Softens acceleration spikes so 4 Krakens
    // don't slam their supply limit together (brownout/wheel-slip protection) without feeling laggy.
    // One limiter PER AXIS -- a single shared limiter cross-couples X and Y. Rotation is left
    // unlimited so turning stays crisp. TODO tune rate (per-second) to driver preference on robot.
    private static final double kTranslationSlewRate = 4.0;
    private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(kTranslationSlewRate);

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
    public final ShooterSubsystem shoterSS = new ShooterSubsystem(false, drivetrain);
    public final IntakeSubsystem intakeSS = new IntakeSubsystem();
    public final HopperSubsystem hopperSS = new HopperSubsystem();

    public RobotContainer() {
        // Shooter DISABLED (team decision, still in effect 2026-07-13). Subsystem stays
        // constructed so current limits apply at boot; periodic() coasts the flywheels and
        // zeroes the hood every loop, and desiredVelReached stays false so the kicker never
        // fires. Re-enable: delete this call and restore the RB/RT bindings below.
        shoterSS.disableSubsystem();

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
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();

        // Read-only current/temp/voltage publisher for the driver companion app (app optional).
        new PowerTelemetry(drivetrain, shoterSS, intakeSS, hopperSS);
    }

    private void configureBindings() {
        // NOTE: the driver-companion "Controls" panel (driver-companion/src/renderer/
        // panels.ts, CONTROLS table) mirrors these bindings by hand — update it when
        // changing anything here.

        //DRIVE SUBSYSTEM (re-enabled 2026-07-10)
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically.
                    // Pipeline per axis: raw stick -> deadband -> square (shapeAxis) -> slew limit -> scale.
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(xSlewLimiter.calculate(shapeAxis(-joystick2.getLeftY())) * MaxSpeed) // Forward = negative left-Y (WPILib convention)
                            .withVelocityY(ySlewLimiter.calculate(shapeAxis(-joystick2.getLeftX())) * MaxSpeed) // Left = negative left-X
                            .withRotationalRate(shapeAxis(-joystick2.getRightX()) * MaxAngularRate) // Drive counterclockwise
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

            // MENU = reset pose (re-zero field-centric heading; point robot downfield, press once).
                joystick2.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

            // A = auto align only: rotate to face the hub tag the Limelight sees. Target =
            // current heading + camera bearing, re-sampled every loop (supplier form). With
            // vision off/no tag the bearing is 0, so it just holds heading briefly -- harmless.
                joystick2.a().whileTrue(drivetrain.rotateToAngle(
                    () -> drivetrain.getState().Pose.getRotation().getDegrees()
                          + shoterSS.getDegreesToAlignToTarget()));
                joystick2.povUp().whileTrue(drivetrain.rotateToAngle(
                    () -> drivetrain.getState().Pose.getRotation().getDegrees()
                          + shoterSS.getDegreesToAlignToTarget()));

            // D-pad feed positions -- TODO [needs field poses]: left/gen/right feed Pose2d
            // coordinates are not defined anywhere yet. Wire with drivetrain.driveToPose(pose)
            // once the team picks the three positions (and verify PathPlanner pathfinding works).
            // NOTE: DPAD left/down/right are TEMPORARILY used by the hopper direction test below;
            // remove those test bindings before wiring feed poses here.
                // joystick2.povLeft().whileTrue(drivetrain.driveToPose(LEFT_FEED_POSE));
                // joystick2.povDown().whileTrue(drivetrain.driveToPose(GEN_FEED_POSE));
                // joystick2.povRight().whileTrue(drivetrain.driveToPose(RIGHT_FEED_POSE));

            drivetrain.registerTelemetry(logger::telemeterize);

        //Intake (competition layout 2026-07-10: press-to-start, X stops everything)
            // HOPPER DISABLED: the hopper halves of LT/X and the B/VIEW bindings are commented
            // out below -- hopper state stays STOW, periodic() holds belts + kicker stopped.
            // Slider re-enabled 2026-07-13 (SLIDER_ENABLED=true), so LT/Y/X move it again.
            // LT = intake: extend slider + rollers in. Runs until X. (Hopper feed disabled.)
            joystick2.leftTrigger().onTrue(intakeSS.intakeCommand());
            // joystick2.leftTrigger().onTrue(Commands.parallel(
            //     intakeSS.intakeCommand(),
            //     hopperSS.setHopperState(HOPPERSTATE.RUN)));
            // Y = outtake on intake: extend slider + rollers out (no hopper). Runs until X.
            joystick2.y().onTrue(intakeSS.outtakeCommand());
            // X = intake stop and stow: rollers off, slider retracted.
            joystick2.x().onTrue(intakeSS.stowCommand());
            // joystick2.x().onTrue(Commands.parallel(
            //     intakeSS.stowCommand(),
            //     hopperSS.setHopperState(HOPPERSTATE.STOW)));
        //Hopper -- DISABLED. Re-enable: restore these + the LT/X hopper composites above.
            // WARNING when re-enabling B: belt relative inversion is STILL unverified in code
            // (flashed on the SPARK MAXes) -- if the belts fight, both NEOs pin their 20 A limit.
            // joystick2.b().onTrue(hopperSS.setHopperState(HOPPERSTATE.RUN))
            //              .onFalse(hopperSS.setHopperState(HOPPERSTATE.STOW));
            // joystick2.back().whileTrue(hopperSS.unjamCommand());

        //Hopper DIRECTION TEST (enabled 2026-07-12) -- slow 5% duty, hold to run, release to stop.
            // Order: DPAD-LEFT (motor A alone), DPAD-RIGHT (motor B alone), note each belt's
            // direction; then DPAD-DOWN (both) to confirm they agree before any full-speed RUN.
            // Safe even if they disagree: 5% duty stall stays under the 20 A smart limit.
            joystick2.povLeft().whileTrue(hopperSS.directionTest(true));   // motor A, CAN 20 (front)
            joystick2.povRight().whileTrue(hopperSS.directionTest(false)); // motor B, CAN 21 (back)
            joystick2.povDown().whileTrue(hopperSS.bothBeltsTest());       // both belts together
        //Shooter -- DISABLED (see disableSubsystem() in the constructor). Re-enable bindings:
            // RB = manual shoot (hold): fixed setpoint -- max hood + SHOOTER_HIGH_SPEED.
            // joystick2.rightBumper().whileTrue(shoterSS.setAngleAndVelocityCommand(
            //         ShooterSubsystemConstants.MAX_ANGLE, ShooterSubsystemConstants.SHOOTER_HIGH_SPEED))
            //     .onFalse(shoterSS.stopShooterCommand());
            // RT = auto shoot (hold): ballistics-model shot for the mid-field -> hub distance.
            // joystick2.rightTrigger().whileTrue(shoterSS.midFieldShotCommand())
            //     .onFalse(shoterSS.stopShooterCommand());
    }

    public Command getAutonomousCommand() {
        // Run whatever auto the drive team picked on the dashboard (chooser defaults to "None").
        Command selected = autoChooser.getSelected();
        return (selected != null) ? selected : Commands.none();
    }
}
