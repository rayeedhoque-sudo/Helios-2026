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
    }

    private void configureBindings() {
        
        //DRIVE SUBSYSTEM
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

                //Auto Align -- NOTE: bind with the METHOD REFERENCE (supplier) form so the target
                // re-samples every loop; passing shoterSS.getDegreesToAlignToTarget() as a double
                // would freeze whatever the value was at boot (0 deg).
                // joystick2.a().onTrue(drivetrain.rotateToAngle(shoterSS::getDegreesToAlignToTarget));
            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                    drivetrain.applyRequest(() -> idle).ignoringDisable(true)
                );

            //Drive Train Break
                joystick2.start().whileTrue(drivetrain.applyRequest(() -> brake));

            // Reset (re-zero) the field-centric heading with the Y button. Use a dedicated button so
            // it only fires on a deliberate press -- povCenter() is true whenever the D-pad is at rest,
            // so the old binding re-seeded heading on any incidental D-pad tap-and-release.
                joystick2.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

            drivetrain.registerTelemetry(logger::telemeterize);

        //IntakeSubsystem
            // Normal bindings: extend slider (stall-detected) + run rollers; stow on release.
            // NOTE: slider extend sign is still TODO-verify on robot (IntakeSubsystem.extendSliderCommand).
            joystick2.rightBumper().whileTrue(intakeSS.intakeCommand()).whileFalse(intakeSS.stowCommand());
            joystick2.leftBumper().whileTrue(intakeSS.outtakeCommand()).whileFalse(intakeSS.stowCommand());
            // Roller-only test bindings (kept for reference; conflict with the bumpers above):
            // joystick2.rightBumper().whileTrue(intakeSS.rollerTestCommand(true));
            // joystick2.leftBumper().whileTrue(intakeSS.rollerTestCommand(false));
        //Hopper Subsystem
            // Belt DIRECTION TEST bindings (D-pad up/down = motor A/B at 5% duty). Run these FIRST:
            // the two hopper NEOs share one gearbox and their relative inversion is NOT set in code
            // (it relies on whatever is flashed on the SPARK MAXes) -- verify before using X/RUN.
            joystick2.povUp().whileTrue(hopperSS.directionTest(true));
            joystick2.povDown().whileTrue(hopperSS.directionTest(false));
            // WARNING: if the belt directions are wrong, RUN pins both NEOs at their 20 A limit
            // (motors fighting through the shared gearbox). Verify with the D-pad test first.
            joystick2.x().whileTrue(hopperSS.setHopperState(HOPPERSTATE.RUN)).onFalse(hopperSS.setHopperState(HOPPERSTATE.STOW));
        // Shooter Subsystem
            joystick2.rightTrigger().whileTrue(
                shoterSS.enableLiveData(true)
            ).whileFalse(
                shoterSS.enableLiveData(false)
            );
    }

    public Command getAutonomousCommand() {
        // Run whatever auto the drive team picked on the dashboard (defaults to "Auto PID").
        Command selected = autoChooser.getSelected();
        return (selected != null) ? selected : Commands.none();
    }
}
