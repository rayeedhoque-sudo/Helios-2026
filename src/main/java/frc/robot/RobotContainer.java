// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

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
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.TunerConstants;
import frc.robot.util.Tunable;
import frc.robot.Constants.SubsystemConstants.AprilTagConstants;
import frc.robot.Constants.SubsystemConstants.AutoAimConstants;
import frc.robot.Constants.SubsystemConstants.LauncherConstants;
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.BLineEventWindow;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.commands.Path1Events;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.misc.HopperSubsystem;
import frc.robot.subsystems.misc.IntakeSubsystem;
import frc.robot.subsystems.misc.MatchStatus;
import frc.robot.subsystems.misc.LauncherSubsystem;
import frc.robot.subsystems.utility.ShotCalculator;
import frc.robot.subsystems.utility.TagAim;
import frc.robot.subsystems.utility.FeedAim;
import frc.robot.subsystems.utility.AllianceZone;
import frc.robot.subsystems.utility.LimelightThermalManager;

public class RobotContainer {
    // Global 20% slowdown (team request 2026-07-17): translation AND rotation capped at 80% of
    // full for more controllable driving. Applied at the top-level scalar so it flows into every
    // teleop drive path (default drive + the shaped/scaled suppliers). Autos use explicit speeds
    // and are NOT affected. TODO tune the 0.8 factor to driver preference.
    // Translation cut 30% more (team request 2026-10-14): 0.8 x 0.7 = 56% of kSpeedAt12Volts.
    // Rotation unchanged. The shot/SOTM/intake slowdowns are fractions of this, so they drop too.
    // Translation cut 30% again (team request 2026-10-17): 0.8 x 0.7 x 0.7 = 39.2% of kSpeedAt12Volts.
    // Translation up 15% (team request 2026-10-17, later): 0.392 x 1.15 = 45.08% of kSpeedAt12Volts.
    // Translation up 15% again (team request 2026-10-17, later): 0.4508 x 1.15 = 51.84% of kSpeedAt12Volts.
    private double MaxSpeed = 0.8 * 0.7 * 0.7 * 1.15 * 1.15 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // 51.84% of kSpeedAt12Volts
    // 0.8 rot/s teleop spin rate (was 1.0; -20% per the same request). Physical ceiling ~1.9 rot/s.
    // x kOdometryScale (team 2026-10-10) keeps the real full-stick spin what the drivers had.
    private double MaxAngularRate = RotationsPerSecond.of(0.8 * TunerConstants.kOdometryScale).in(RadiansPerSecond);

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
    private static final double kRollerSlowFactor = 0.9; // was 0.5; team 2026-10-17: 90%
    // RT shot slowdown (team spec 2026-09-30): while RT is held the drive still works, but at
    // 10% of max -- translation AND rotation. Wins over the intake slowdown.
    private static final double kShotDriveFactor = 0.1;
    // Shoot-on-the-move translation cap: 20% of max (team 2026-10-09), cut to 10% ~0.4 m/s
    // (team 2026-10-10). Wins over
    // both slowdowns above -- RB also runs the launcher, so isShooting() is true during it.
    // TODO raise once the shot table + flight times are tuned (lead error grows with speed).
    private static final double kSotmDriveFactor = 0.1;
    private double driveScale() {
        if (shotCalc.isActive()) return kSotmDriveFactor;
        if (AutoAimConstants.TAG_SHOOT_ON_THE_MOVE && tagAim.isActive()) return kSotmDriveFactor;
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
    // via the scheduler; "shooter active" = flywheels commanded to spin OR RT auto-aim held (swapped from RB 2026-10-10)
    // (both need the full-res pipeline).
    public final LimelightThermalManager limelightThermal = new LimelightThermalManager(
        Vision.CAM_LIMELIGHT, () -> launcherSS.isShooting() || joystick2.getRightTriggerAxis() > 0.5);
    // RB shoot-on-the-move solution (virtual target, heading, table rpm/hood), every loop.
    private final ShotCalculator shotCalc = new ShotCalculator(drivetrain);
    private final TagAim tagAim = new TagAim(drivetrain);
    // LB feed shot (remade 2026-10-17): any tag -> robot on the field -> heading, rpm, hub block.
    private final FeedAim feedAim = new FeedAim(drivetrain);
    // Teleop zone idle (2026-10-17): zone tag seen -> flywheels idle until the pose leaves our zone.
    private final AllianceZone allianceZone = new AllianceZone(drivetrain);

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
        // retracts the slider until stall. Same group as the LT hold; unlike LT's release
        // (2026-10-14), the rollers stop before the retract.
        autoChooser.addOption("Forward 3 m then Left 3 m intaking @ 1 m/s",
                drivetrain.resetToAutoStartPose()
                        // The auto-start slider cycle runs here (this auto holds the intake), cut short by the intake leg.
                        .andThen(drivetrain.driveForwardAuto(3.0, 1.0).deadlineFor(intakeSS.sliderCycleCommand()))
                        .andThen(drivetrain.driveLeftAuto(3.0, 1.0)
                                .deadlineFor(intakeSS.intakeCommand()
                                        .andThen(hopperSS.intakeFeedCommand())
                                        .finallyDo(intakeSS::stopRollers)))
                        .andThen(intakeSS.stowCommand())
                        .andThen(drivetrain.resetAutoEndTranslation()));
        // BLine paths (team 2026-10-09): every deploy/autos/paths/*.json is added automatically.
        // Each runs RELATIVE to wherever the robot sits when auto starts (team 2026-10-10): no
        // pose reset, no field position, no companion start spot. A bad file reports to the DS and is skipped instead of crashing boot.
        for (String name : blinePathNames(new File(Filesystem.getDeployDirectory(), "autos/paths"))) {
            if (name.equals(SHOOTING_AUTON_PATH)) {
                continue; // its own auto below (intake + shot), not a bare path
            }
            try {
                autoChooser.addOption("BLine: " + name, drivetrain.followBLinePath(new Path(name)));
                previewBLine("BLine: " + name, name);
            } catch (RuntimeException e) {
                DriverStation.reportError("BLine path '" + name + "' failed to load: " + e.getMessage(), e.getStackTrace());
            }
        }
        // BLine's logged translation index feeds every event window (one global consumer).
        FollowPath.setDoubleLoggingConsumer(p -> {
            if (p.getFirst().equals("FollowPath/translationElementIndex")) {
                shootingAutonIntake.onIndex(p.getSecond());
                pt1SoloIntake.onIndex(p.getSecond());
            }
            if (p.getFirst().equals("FollowPath/remainingPathDistanceMeters")) {
                blineRemainingMeters = p.getSecond();
            }
        });
        // Path 1 (team 2026-10-14): part 1, hold, part 2, then "path1-pt2-postfeedshot" (team
        // 2026-10-16). Each later part carries on from where the one before left BLine (keepFrame):
        // pt 2's own start is 17 cm / 7 deg off pt 1's end, and restarting there would swing all of
        // pt 2 toward the side wall. ~24 s in all: a 15 s match auto ends partway through pt 2.
        // Event triggers (team 2026-10-16), one key per action -- see Path1Events:
        //   pt 1: "intake" = LT held, "intake_stop" = LT let go. Its "hub_shot" sits at t = 1.0, which
        //         BLine can stop just short of, so it also runs here once pt 1 ends: RT's hub aim + shot
        //         standing still for PATH_1_HUB_SHOT_SEC, then intake + hood in before pt 2 moves (<= 1 s).
        //   pt 2: "stow" = intake + hood in again (before the trench), "intake" = LT held,
        //         "feed_shot" = LB's feed shot, aimed at the feed target while driving.
        //   post: "feed_stop" = feed shot off (LT still held), "hub_shot" = LT let go + RT's hub aim +
        //         shot while driving; once the path ends it keeps aiming + shooting standing still.
        // The mechanisms run off the flags through the real LT / LB / RT groups; the auto itself
        // requires only the drivetrain, so no event's intake or shot can cancel it. Aims while a
        // path drives replace BLine's rotation (CommandSwerveDrivetrain.setBLineAim).
        FollowPath.registerEventTrigger("intake", () -> { // flags only: never a Command
            path1.onIntake();
            if (pt1SoloRunning) pt1SoloIntake.onEvent(); // the pt 1 solo auto below shares the key
        });
        FollowPath.registerEventTrigger("intake_stop", path1::onIntakeStop);
        FollowPath.registerEventTrigger("stow", path1::onStow);
        FollowPath.registerEventTrigger("feed_shot", path1::onFeedShot);
        FollowPath.registerEventTrigger("feed_stop", path1::onFeedStop);
        FollowPath.registerEventTrigger("hub_shot", path1::onHubShot);
        drivetrain.setBLineAim(
            () -> path1.hubShot() ? Optional.of(tagAim.getTargetHeading())
                // Feed = LB's aim (held within +-10 deg of our wall, LB's gentle turn rate) once a tag
                // places the robot. Until then the path keeps its own rotation: LB's "turn toward the
                // hub by odometry" is not used mid-path (relative autos' odometry isn't field-true).
                : path1.feedShot() && feedAim.hasTarget() ? Optional.of(feedAim.getTargetHeading())
                : Optional.empty(),
            () -> path1.hubShot() ? tagAim.getHeadingRateFeedforward() : 0.0,
            () -> path1.feedShot() && !path1.hubShot() ? AutoAimConstants.FEED_MAX_ROTATION_RAD_PER_SEC
                : AutoAimConstants.MAX_ROTATION_RAD_PER_SEC);
        Command path1StowAll = Commands.parallel(intakeSS.stowCommand(), launcherSS.hoodToFloorCommand());
        bindLtIntake(new Trigger(path1::intake), intakeSS, hopperSS);
        new Trigger(path1::stow).onTrue(path1StowAll);
        // kCancelIncoming like teleop LB / RT, so an LT let-go stow at the same moment never wins.
        // Feed off with LT still held goes back to LT's hold (teleop LB's release would stow).
        new Trigger(path1::feedShot)
            .whileTrue(feedShotCommand(path1::intake).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
            .onFalse(Commands.either(
                ltHoldCommand(path1::intake, intakeSS, hopperSS).until(() -> !path1.intake()),
                intakeSS.stowCommand(), path1::intake));
        new Trigger(path1::hubShot)
            .whileTrue(tagShotCommand(path1::intake).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
        // 3-step path 1's first hub shot starts at once (team 2026-10-17): from "intake_stop" the flywheels +
        // hood spin up to the tag-distance shot (nothing fed; the hub shot takes the launcher over), and
        // that shot skips the 1 s start-of-shot reverse.
        new Trigger(path1::preSpin)
            .whileTrue(launcherSS.spinUpCommand(this::hubShotRpm, this::hubShotHoodDeg)
                .beforeStarting(() -> tagAim.setActive(true))
                .finallyDo(() -> tagAim.setActive(false)));
        launcherSS.setSkipShotReverse(path1::skipShotReverse);
        hopperSS.setSkipShotReverse(path1::skipShotReverse);
        try {
            String path1Label = "BLine: path 1 (pt 1, hub shot, pt 2, post-feed-shot)";
            autoChooser.addOption(path1Label, path1Auto(
                drivetrain.followBLinePath(new Path("path_1_pt.1")),
                drivetrain.followBLinePath(new Path("path-1-pt-2"), true),
                drivetrain.followBLinePath(new Path("path1-pt2-postfeedshot"), true),
                path1StowAll));
            // PATH 1 + VISION (team 2026-10-17): the same auto on the FIELD pose, with Limelight MegaTag2
            // correcting odometry while it drives (gates + trust from the 10-10 Limelight shot data, see
            // Vision.VISION_AUTO_*). The pose is reset to pt 1's authored start, mirrored on red: place the
            // robot there, square. Vision stays off while the Limelight mount pose is unset (DS warning).
            previewBLine(path1Label, "path_1_pt.1", "path-1-pt-2", "path1-pt2-postfeedshot");
            String path1VisionLabel = "BLine: path 1 + vision (pt 1, hub shot, pt 2, post-feed-shot)";
            autoChooser.addOption(path1VisionLabel, path1Auto(
                drivetrain.followBLinePathWithVision(new Path("path_1_pt.1"), true),
                drivetrain.followBLinePathWithVision(new Path("path-1-pt-2"), false),
                // Step 3: the opponent's hub + trench tags count out to 4.5 m (team 2026-10-17); setVisionAuto(false) clears it.
                Commands.runOnce(() -> drivetrain.setVisionAutoFarTags(true))
                    .andThen(drivetrain.followBLinePathWithVision(new Path("path1-pt2-postfeedshot"), false)),
                path1StowAll)
                .beforeStarting(() -> {
                    drivetrain.setVisionAuto(true);
                    drivetrain.setVisionAutoRedSideOnly(true); // on red: only tags 1-16 place the robot (team 2026-10-17)
                })
                .finallyDo(() -> drivetrain.setVisionAuto(false)));
            previewBLine(path1VisionLabel, "path_1_pt.1", "path-1-pt-2", "path1-pt2-postfeedshot");
            // PATH 1 PT 2 (team 2026-10-17): path 1 from pt 2 on -- same events/aims/shots, no vision,
            // relative. pt 2 starts fresh (no pt 1 before it): place the robot at pt 2's authored start
            // (1.87, 7.01) facing its start heading (-57.5 deg), or the trench pass is off by the same.
            String path1Pt2Label = "BLine: path 1 pt 2 (pt 2, post-feed-shot)";
            autoChooser.addOption(path1Pt2Label, Commands.sequence(
                    drivetrain.followBLinePath(new Path("path-1-pt-2")),
                    drivetrain.followBLinePath(new Path("path1-pt2-postfeedshot"), true),
                    drivetrain.applyRequest(() -> drivetrain.faceHeading(0, 0,
                        tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward())))
                .beforeStarting(path1::start)
                .finallyDo(path1::stop));
            previewBLine(path1Pt2Label, "path-1-pt-2", "path1-pt2-postfeedshot");
            // PATH 1 PT 3 (team 2026-10-17): post-feed-shot alone, no vision, relative. Starts with LT held
            // (as pt 2 hands over) but no feed shot (team: it can't spin up before "feed_stop" ~0.5 s in);
            // "hub_shot" = LT let go + RT's hub shot, then aim + shoot standing still. Place the robot at the
            // path's authored start (7.47, 4.26) facing 145 deg. As authored it runs ~0.6 m past the centre line.
            String path1Pt3Label = "BLine: path 1 pt 3 (post-feed-shot)";
            autoChooser.addOption(path1Pt3Label, Commands.sequence(
                    drivetrain.followBLinePath(new Path("path1-pt2-postfeedshot")),
                    drivetrain.applyRequest(() -> drivetrain.faceHeading(0, 0,
                        tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward())))
                .beforeStarting(() -> {
                    path1.start();
                    path1.onIntake();
                })
                .finallyDo(path1::stop));
            previewBLine(path1Pt3Label, "path1-pt2-postfeedshot");
            // VISION COPIES of pt 2 and pt 3 (team 2026-10-17): same actions on the FIELD pose with Limelight
            // MegaTag2 fused while driving, like path 1 + vision. Pose reset once to the first path's authored
            // start (mirrored on red): place the robot there, square. Vision off while the mount pose is unset.
            String path1Pt2VisionLabel = "BLine: path 1 pt 2 + vision (pt 2, post-feed-shot)";
            autoChooser.addOption(path1Pt2VisionLabel, withVisionAuto(Commands.sequence(
                    drivetrain.followBLinePathWithVision(new Path("path-1-pt-2"), true),
                    drivetrain.followBLinePathWithVision(new Path("path1-pt2-postfeedshot"), false),
                    drivetrain.applyRequest(() -> drivetrain.faceHeading(0, 0,
                        tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward())))
                .beforeStarting(path1::start)
                .finallyDo(path1::stop)));
            previewBLine(path1Pt2VisionLabel, "path-1-pt-2", "path1-pt2-postfeedshot");
            String path1Pt3VisionLabel = "BLine: path 1 pt 3 + vision (post-feed-shot)";
            autoChooser.addOption(path1Pt3VisionLabel, withVisionAuto(Commands.sequence(
                    drivetrain.followBLinePathWithVision(new Path("path1-pt2-postfeedshot"), true),
                    drivetrain.applyRequest(() -> drivetrain.faceHeading(0, 0,
                        tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward())))
                .beforeStarting(() -> {
                    path1.start();
                    path1.onIntake();
                })
                .finallyDo(path1::stop)));
            previewBLine(path1Pt3VisionLabel, "path1-pt2-postfeedshot");
        } catch (RuntimeException e) {
            DriverStation.reportError("Path 1 auto failed to load: " + e.getMessage(), e.getStackTrace());
        }
        // PATH 1 PT 1 SOLO (team 2026-10-16, its own export "path_1_pt.1-2", relative like the other
        // BLine autos). "intake" = LT pressed, held until BLine reaches waypoint 3 (0.80, 7.16; handoff
        // ~0.45 m before it), then LT let go (rollers 25% + belts until stowed). "shoot" = RT pressed:
        // it sits at t = 1.0, which BLine can stop just short of, so RT is also pressed once the path
        // ends -- tag aim + shot standing still for PT1_SOLO_SHOT_SEC, then RT let go (stow). The
        // middle "event" trigger does nothing here. Wall points edited 0.53 / 0.61 -> 0.80 and the wall
        // approach 3.03 -> 1.0 m/s (team, 2026-10-16): the held-out intake would hit the wall.
        FollowPath.registerEventTrigger("shoot", () -> { // flag only
            if (pt1SoloRunning) pt1SoloShoot = true;
        });
        bindLtIntake(new Trigger(() -> pt1SoloRunning && pt1SoloIntake.isOpen()), intakeSS, hopperSS);
        new Trigger(() -> pt1SoloShoot)
            .whileTrue(tagShotCommand(() -> false).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
            .onFalse(intakeSS.stowCommand());
        try {
            String pt1SoloLabel = "BLine: path 1 pt 1 (LT, then RT shot)";
            autoChooser.addOption(pt1SoloLabel, pt1SoloAuto(drivetrain.followBLinePath(new Path(PT1_SOLO_PATH))));
            previewBLine(pt1SoloLabel, PT1_SOLO_PATH);
            // VISION COPY (team 2026-10-17): same actions on the FIELD pose with Limelight MegaTag2 fused
            // while it drives, like path 1 + vision. Pose reset to the path's authored start (mirrored on
            // red): place the robot there, square. Vision stays off while the Limelight mount pose is unset.
            String pt1SoloVisionLabel = "BLine: path 1 pt 1 + vision (LT, then RT shot)";
            autoChooser.addOption(pt1SoloVisionLabel, withVisionAuto(pt1SoloAuto(
                drivetrain.followBLinePathWithVision(new Path(PT1_SOLO_PATH), true))));
            previewBLine(pt1SoloVisionLabel, PT1_SOLO_PATH);
        } catch (RuntimeException e) {
            DriverStation.reportError("Path 1 pt 1 auto failed to load: " + e.getMessage(), e.getStackTrace());
        }
        try {
            autoChooser.addOption("Shooting auton", shootingAuton());
            previewBLine("Shooting auton", SHOOTING_AUTON_PATH);
        } catch (RuntimeException e) {
            DriverStation.reportError("Shooting auton failed to load: " + e.getMessage(), e.getStackTrace());
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
        try { // robot size for the companion's BLine preview footprints
            SmartDashboard.putString("BLine Config", Files.readString(new File(Filesystem.getDeployDirectory(), "autos/config.json").toPath()));
        } catch (IOException e) {
            DriverStation.reportWarning("BLine config for the auto preview failed: " + e.getMessage(), false);
        }

        // Practice tag stand-ins (AprilTagConstants) must never be forgotten: say so at every boot.
        if (!AprilTagConstants.standIns().isEmpty()) {
            DriverStation.reportWarning("AprilTag stand-ins ACTIVE (practice only; ignored with an FMS): "
                + AprilTagConstants.standIns(), false);
        }
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
        //   LT (hold)   = intake (slider out -> rollers + belts); release = stow, rollers (at 10%) + belts run until stowed
        //   Y (hold)    = outtake (same choreography, rollers out); release = stow
        //   X           = manual stow
        //   A (hold)    = manual hopper belts only
        //   VIEW (hold) = unjam: reverse belts + feeder (slow); intake untouched
        //   RB (hold)   = SHOT (RT/RB swapped 2026-10-10): flywheels at the DPAD-set rpm + feeder; belts (and feeder, slowly) reverse for the first
        //                 HOPPER_SHOT_REVERSE_SEC, then run ONLY while both are at speed; slider cycles extend 0.5 s / stow (rollers in at 5%); drive 10%.
        //                 Release: flywheels/feeder coast, slider stows. Hood NOT commanded.
        //   LB (hold)   = FEED SHOT: same shot choreography, hood max (29 deg), drive 10%, LT intakes. Position
        //                 from any tag (either alliance, kept between presses via odometry; FeedAim): heading held
        //                 within +-10 deg of straight at our wall (turns back gently, 90 deg/s), rpm by distance to
        //                 our wall lead-corrected for motion; no position yet = turns toward our hub, belts wait;
        //                 right behind a hub = aim only, no shot. Release: hood back to its pre-LB angle.
        //   DPAD L/R    = RB flywheel target -/+ 100 flywheel RPM (start 2000, 0-4000)
        //   DPAD UP/DN  = (hold) hood up/down at 1 deg/s (0-29 deg above the bottom stop; the hood
        //                 re-homes on every enable and ignores the DPAD until homed)
        //   MENU        = manual heading re-zero
        //   RT (hold)   = TAG AIM + SHOT (RB_SHOOT_ON_THE_MOVE = false, team 2026-10-09): robot holds still
        //                 and turns until the crosshair is on tag 10/26 (test tag off-FMS); flywheel rpm +
        //                 hood follow the tag-distance table; belts feed (RB's choreography) only once
        //                 aimed, hood at target and flywheels + feeder at speed. RB and RT exclude each other.
        //   RT, if RB_SHOOT_ON_THE_MOVE = true = SHOOT ON THE MOVE: left stick still drives (capped at 10%), the robot
        //                 faces the lead-corrected hub, flywheel rpm + hood follow the shot
        //                 table, and the belts feed (RB's choreography) only while aimed, hood
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
                        if (tagAim.isActive() && AutoAimConstants.TAG_SHOOT_ON_THE_MOVE) {
                            // Tag shoot-on-the-move (ON 2026-10-10): LEFT stick still drives (capped,
                            // driveScale), right stick ignored, heading servoed to the lead-corrected hub.
                            return drivetrain.faceHeading(driveVelX.getAsDouble(), driveVelY.getAsDouble(),
                                tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward());
                        }
                        if (feedAim.isActive()) {
                            // LB feed (team 2026-10-17): LEFT stick still drives (10%, driveScale), right
                            // stick ignored; heading = FeedAim's (held inside +-10 deg of our wall's
                            // direction, or toward our hub until a tag gives a position), turned gently.
                            return drivetrain.faceHeading(driveVelX.getAsDouble(), driveVelY.getAsDouble(),
                                feedAim.getTargetHeading(), 0, AutoAimConstants.FEED_MAX_ROTATION_RAD_PER_SEC);
                        }
                        if (tagAim.isActive()) {
                            // RB tag aim: hold X/Y still, turn only (team 2026-10-09). The limiters
                            // ramp the translation down to 0 (no slam stop if RB is hit mid-drive),
                            // and release ramps back up from there.
                            return drivetrain.faceHeading(xSlewLimiter.calculate(0) * MaxSpeed,
                                ySlewLimiter.calculate(0) * MaxSpeed, tagAim.getTargetHeading(), 0);
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
            // moves). Release (team 2026-10-14): rollers (slowed to 10%) + belts keep running while
            // the slider retracts until stall, and stop once it is stowed -- see bindLtIntake.
            // RE-ENABLED 2026-08-22 (team request), exactly as it was before the disable.
            bindLtIntake(joystick2.leftTrigger().and(RobotModeTriggers.teleop()), intakeSS, hopperSS);
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
            // RB (hold) = SHOT (was RT; RT/RB swapped, team 2026-10-10). The launcher spins the flywheels (DPAD-set rpm) and the feeder
            // together; the hopper belts run ONLY while both are at speed, re-checked every loop,
            // so a ball dipping the wheels pauses the belts until they recover. The intake
            // slider cycles extend 0.35 s / retract to stow (rollers in/out at 5%, alternating) for the whole hold;
            // holding LT during any shot (RB/RT/LB) intakes instead, and releasing it goes back to
            // agitating (LT's own binding and its release-stow are refused while a shot runs). The drivetrain is NOT
            // required -- the default drive keeps running at 10% (driveScale). kCancelIncoming:
            // releasing RB is the only way out. Release: flywheels/feeder coast, slider stows.
            joystick2.rightBumper().and(RobotModeTriggers.teleop())
                .whileTrue(launcherSS.shootCommand()
                    .alongWith(
                        hopperSS.feedShooterCommand(launcherSS::isReadyToFeed),
                        intakeSS.shotIntakeCommand(joystick2.leftTrigger()))
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                .onFalse(intakeSS.stowCommand());
            // LB (hold) = FEED SHOT (FeedAim, team 2026-10-17): RT's shot group at HOOD_MAX_DEG, left
            // stick drives (10%), LT intakes. No position known yet: turns toward our hub by odometry,
            // flywheels spin up at LB_SHOT_FLYWHEEL_RPM, belts wait (the old fixed shot is gone). Once a
            // tag (any, either alliance) places the robot: heading held within +-10 deg of straight at
            // our wall, rpm from FEED_RPM_BY_WALL_DIST lead-corrected for motion, belts once at speed,
            // hood up and aimed. Right behind a hub (FEED_HUB_BLOCK_*) NOTHING shoots, it only
            // aims. Release: hood back to the angle it had before LB,
            // flywheels/feeder coast, slider stows. kCancelIncoming, like RB/RT: whichever is pressed first wins.
            joystick2.leftBumper().and(RobotModeTriggers.teleop())
                .whileTrue(feedShotCommand(joystick2.leftTrigger())
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
            // NO BUTTON = TELEOP ZONE IDLE (team 2026-10-17): a zone tag seen (red 7-12, blue 23-28) idles
            // the flywheels at IDLE_FLYWHEEL_RPM (feedforward only, no 12 V bursts) for 5 s, restarted by
            // every sighting; after that only while the robot hasn't moved 0.3 m from that spot. Every shot
            // takes over from it. Coast in auto/test/disabled. Hands clear of the shooter when enabled.
            launcherSS.setDefaultCommand(launcherSS.idleSpinCommand(allianceZone::shouldIdleSpin));
            // RT (hold) = SHOOT ON THE MOVE (was RB; swapped 2026-10-10; redesigned 2026-10-09 after LUNATECS, Ninjas 4744
            // and 1833). Aiming happens in the default drive while shotCalc is active (see above). The
            // group is RB's manual shot with the table rpm/hood, and the belts also wait for aim + hood.
            // kCancelIncoming, like RB: whichever of RB/RT is pressed first wins. Release:
            // flywheels/feeder coast, slider stows, the drive goes back to normal sticks.
            // TODO verify on robot in open space, Limelight mount pose set first: robot must turn
            // TOWARD the hub, and the field map must show the robot where it really is.
            if (AutoAimConstants.RB_SHOOT_ON_THE_MOVE) {
            joystick2.rightTrigger().and(RobotModeTriggers.teleop())
                .whileTrue(launcherSS.autoShootCommand(
                        () -> shotCalc.getShot().flywheelRpm(), () -> shotCalc.getShot().hoodDeg())
                    .alongWith(
                        hopperSS.feedShooterCommand(() -> launcherSS.isReadyToFeed()
                            && launcherSS.isHoodAtTarget() && shotCalc.isAimed()),
                        intakeSS.shotIntakeCommand(joystick2.leftTrigger()))
                    .beforeStarting(() -> shotCalc.setActive(true))
                    .finallyDo(() -> shotCalc.setActive(false))
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                .onFalse(intakeSS.stowCommand());
            } else {
            // RT (hold) = TAG AIM + SHOT (was RB; swapped 2026-10-10). With TAG_SHOOT_ON_THE_MOVE
            // (ON 2026-10-10) the left stick still drives at 10% and the robot turns to the
            // lead-corrected hub, rpm/hood following the moving distance; off = it holds still and
            // turns until the crosshair is on tag 10/26 (test tag off-FMS) -- the turning
            // happens in the default drive (see above). Same shot group as the shoot-on-the-move
            // branch, with rpm/hood from the tag distance and the belts waiting for tagAim.isAimed().
            // TODO verify on robot: shots land at each table distance.
            joystick2.rightTrigger().and(RobotModeTriggers.teleop())
                .whileTrue(tagShotCommand(joystick2.leftTrigger())
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming))
                .onFalse(intakeSS.stowCommand());
            }
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

    /**
     * LT (hold) = intake: slider out until stall, then rollers in + belts. Release (team 2026-10-14):
     * rollers (slowed to STOW_ROLLER_SPEED, 10% of intake) AND belts keep running while the slider
     * retracts, and stop once it is stowed (or the stow
     * is interrupted). A tap released before the slider is out never started them, so they stay off.
     * The hold's finallyDo still brakes the rollers at once when the hold is cancelled while LT is held
     * (A / VIEW / Y / a shot took it) or by a disable -- only a plain release while enabled, which
     * always schedules the stow below in the same loop, leaves them spinning for it. (Enabled, not
     * teleop-enabled, since 2026-10-14: path 1's auto intake drives this same binding. The teleop LT
     * trigger is gated on teleop, so for it the two checks are identical.)
     */
    public static void bindLtIntake(Trigger lt, IntakeSubsystem intakeSS, HopperSubsystem hopperSS) {
        lt.whileTrue(ltHoldCommand(lt, intakeSS, hopperSS))
            .onFalse(intakeSS.retractSliderCommand()
                .beforeStarting(intakeSS::slowRollersForStow)
                .finallyDo(intakeSS::stopRollers))
            // Belts follow the rollers. Their own command, so A / VIEW taking the hopper mid-stow
            // can't abort the retract. asProxy: the gate itself requires nothing, so an A / VIEW
            // held across the release isn't cancelled when the belts don't run.
            .onFalse(hopperSS.intakeFeedCommand()
                .onlyWhile(intakeSS::rollersRunning)
                .asProxy()
                .onlyIf(intakeSS::rollersRunning));
    }

    /** LT's hold: slider out until stall, then rollers in + belts. Brakes the rollers on end unless LT was let go while enabled. */
    static Command ltHoldCommand(BooleanSupplier lt, IntakeSubsystem intakeSS, HopperSubsystem hopperSS) {
        return intakeSS.intakeCommand().andThen(hopperSS.intakeFeedCommand())
            .finallyDo(() -> {
                if (lt.getAsBoolean() || !DriverStation.isEnabled()) {
                    intakeSS.stopRollers();
                }
            });
    }

    /**
     * LB feed shot group (also path 1's auto feed shot), hood max. No position known yet: flywheels
     * spin up at LB_SHOT_FLYWHEEL_RPM and the belts WAIT (team 2026-10-17: no more fixed shot). Position
     * known: lead-corrected rpm by wall distance; belts once at speed, hood up and aimed.
     */
    private Command feedShotCommand(BooleanSupplier intakeHeld) {
        Command shot = launcherSS.autoShootCommand(feedAim::getFlywheelRpm, () -> LauncherConstants.HOOD_MAX_DEG)
            .alongWith(hopperSS.feedShooterCommand(() -> launcherSS.isReadyToFeed() && launcherSS.isHoodAtTarget()
                && feedAim.isAimed()));
        // Behind the hub (FeedAim blocked, team 2026-10-17): the shot stops (flywheels/feeder coast,
        // belts off, hood back down) and waits, still holding the launcher + hopper so nothing else
        // spins them; the slider stows; only the aim keeps running. Clear again = the shot restarts.
        return Commands.sequence(Commands.waitUntil(() -> !feedAim.isBlocked()), shot.until(feedAim::isBlocked))
                .repeatedly()
            .alongWith(intakeSS.shotIntakeCommand(intakeHeld, feedAim::isBlocked))
            .beforeStarting(() -> feedAim.setActive(true))
            .finallyDo(() -> feedAim.setActive(false));
    }

    /** Shooting auton's intake hold group: slider out until stall, then rollers in + belts. */
    private Command intakeHoldCommand() {
        return intakeSS.intakeCommand().andThen(hopperSS.intakeFeedCommand())
            .finallyDo(intakeSS::stopRollers);
    }

    /**
     * RT tag aim + shot group (also the Shooting auton's shot). The turning itself is done by
     * whoever drives: the default drive in teleop, an explicit aim request in auto.
     */
    private Command tagShotCommand(BooleanSupplier intakeHeld) {
        return launcherSS.autoShootCommand(this::hubShotRpm, this::hubShotHoodDeg)
            .alongWith(
                hopperSS.feedShooterCommand(() -> launcherSS.isReadyToFeed()
                    && launcherSS.isHoodAtTarget() && tagAim.isAimed()),
                intakeSS.shotIntakeCommand(intakeHeld))
            .beforeStarting(() -> tagAim.setActive(true))
            .finallyDo(() -> tagAim.setActive(false));
    }

    /** RT tag shot's flywheel rpm by the tag hub distance. */
    private double hubShotRpm() {
        return AutoAimConstants.TAG_SHOT_RPM_BY_HUB_DIST.get(tagAim.getHubDistanceMeters())
            + AutoAimConstants.TAG_SHOT_RPM_OFFSET;
    }

    /** RT tag shot's hood angle (deg) by the tag hub distance. */
    private double hubShotHoodDeg() {
        return AutoAimConstants.TAG_SHOT_HOOD_DEG_BY_HUB_DIST.get(tagAim.getHubDistanceMeters())
            + AutoAimConstants.TAG_SHOT_HOOD_OFFSET_DEG;
    }

    // BLine's logged remaining path distance (m), for path 1's early pt 1 / pt 2 finishes.
    private double blineRemainingMeters = Double.POSITIVE_INFINITY;
    // Path 1 pt 1 and pt 2 count as done this close to their ends (team 2026-10-17: the final creep into
    // BLine's 3 cm / 2 deg end tolerance, at its 0.15 m/s floor, delayed the shot and stalled the robot
    // where the feed shot used to be). TODO tune on robot.
    public static final double PATH1_EARLY_END_METERS = 0.15; // [assumed]

    private static final String SHOOTING_AUTON_PATH = "Shooting_auton";
    private static final double SHOOTING_AUTON_SHOT_SEC = 10.0; // team 2026-10-10 (overruns a 15 s match auto)
    private final BLineEventWindow shootingAutonIntake = new BLineEventWindow();
    private final Path1Events path1 = new Path1Events();
    private static final double PATH_1_HUB_SHOT_SEC = 4.0; // team 2026-10-16 (was a 2 s plain hold)
    // Path 1 pt 1 solo auto: LT from "intake" across waypoint 2 to waypoint 3, then RT for this long.
    private static final String PT1_SOLO_PATH = "path_1_pt.1-2";
    private static final double PT1_SOLO_SHOT_SEC = 10.0; // team 2026-10-16 (overruns a 15 s match auto)
    private final BLineEventWindow pt1SoloIntake = new BLineEventWindow(2);
    private boolean pt1SoloRunning = false, pt1SoloShoot = false;

    /** Path 1 pt 1 solo's sequence (team 2026-10-16); the vision copy passes its own follower. */
    private Command pt1SoloAuto(Command follow) {
        return Commands.sequence(
                follow,
                Commands.runOnce(() -> pt1SoloShoot = true),
                drivetrain.applyRequest(() -> drivetrain.faceHeading(0, 0,
                    tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward())).withTimeout(PT1_SOLO_SHOT_SEC),
                Commands.runOnce(() -> pt1SoloShoot = false))
            .beforeStarting(() -> {
                pt1SoloIntake.reset();
                pt1SoloShoot = false;
                pt1SoloRunning = true;
            })
            .finallyDo(() -> {
                pt1SoloRunning = false;
                pt1SoloShoot = false;
                pt1SoloIntake.reset();
            });
    }

    /** Vision-auto mode (Limelight fused while BLine drives) on for this auto, off when it ends. */
    private Command withVisionAuto(Command auto) {
        return auto
            .beforeStarting(() -> drivetrain.setVisionAuto(true))
            .finallyDo(() -> drivetrain.setVisionAuto(false));
    }

    /**
     * Path 1's sequence around its three parts (team 2026-10-16); the vision copy passes its own followers.
     * No feed shot (team 2026-10-17): pt 2's "feed_shot" event is ignored here (LT stays held through it).
     * The first hub shot starts at once (team 2026-10-17): pt 1 ends PATH1_EARLY_END_METERS short of its end
     * instead of creeping in, the flywheels + hood are pre-spun from "intake_stop", and no 1 s reverse.
     */
    private Command path1Auto(Command pt1, Command pt2, Command post, Command stowAll) {
        return Commands.sequence(
                Commands.runOnce(() -> blineRemainingMeters = Double.POSITIVE_INFINITY),
                pt1.until(() -> blineRemainingMeters <= PATH1_EARLY_END_METERS), // no final creep (team 2026-10-17)
                Commands.runOnce(path1::onHubShot),
                drivetrain.applyRequest(() -> drivetrain.faceHeading(0, 0,
                    tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward())).withTimeout(PATH_1_HUB_SHOT_SEC),
                Commands.runOnce(path1::endHubShot),
                Commands.waitUntil(() -> !launcherSS.isShooting()), // shot group off: the intake is free
                new ScheduleCommand(stowAll),
                Commands.waitUntil(() -> !stowAll.isScheduled() && launcherSS.isHoodAtTarget()).withTimeout(1.0),
                Commands.runOnce(() -> {
                    if (stowAll.isScheduled() || !launcherSS.isHoodAtTarget()) {
                        DriverStation.reportWarning("Path 1: intake/hood not confirmed in after 1 s -- pt 2 starting anyway", false);
                    }
                }),
                Commands.runOnce(() -> blineRemainingMeters = Double.POSITIVE_INFINITY),
                pt2.until(() -> blineRemainingMeters <= PATH1_EARLY_END_METERS), // straight on into post (team 2026-10-17)
                post,
                drivetrain.applyRequest(() -> drivetrain.faceHeading(0, 0,
                    tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward())))
            .beforeStarting(path1::startThreeStep)
            .finallyDo(path1::stop);
    }

    /**
     * SHOOTING AUTON (team 2026-10-10, BLine "Shooting_auton"). The pose is reset to the path
     * start (flipped for red) -- the robot must be placed there -- and the path runs on the real
     * field at the file's 0.75 m/s with vision paused. From the path's "event" trigger until the
     * end of the wall translation it intakes exactly like holding LT, slowed to the 10%
     * shoot-on-the-move speed; then it stows like releasing LT. At the end of the path it runs
     * RT's tag aim + shot, standing still, for SHOOTING_AUTON_SHOT_SEC, then stows.
     */
    // Publishes the BLine path files of one chooser option for the driver companion's auto
    // preview (drawn the way BLine Web draws them): one raw JSON text per part, in run order.
    private static void previewBLine(String label, String... pathNames) {
        try {
            String[] files = new String[pathNames.length];
            for (int i = 0; i < pathNames.length; i++) {
                files[i] = Files.readString(new File(Filesystem.getDeployDirectory(), "autos/paths/" + pathNames[i] + ".json").toPath());
            }
            SmartDashboard.putStringArray("Auto Preview/" + label, files);
        } catch (IOException | RuntimeException e) {
            DriverStation.reportWarning("Auto preview for '" + label + "' failed: " + e.getMessage(), false);
        }
    }

    private Command shootingAuton() {
        Path path = new Path(SHOOTING_AUTON_PATH);
        FollowPath.registerEventTrigger("event", shootingAutonIntake::onEvent); // flag only: never a Command
        // m/s: 10% of the pre-2026-10-14 teleop max, so the 30% teleop cut leaves this auto alone.
        double intakeSpeed = kSotmDriveFactor * 0.8 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // The auto-start slider cycle runs here (this auto holds the intake), cut short if the window opens first.
        Command intakeStretch = Commands.waitUntil(shootingAutonIntake::isOpen).deadlineFor(intakeSS.sliderCycleCommand())
            .andThen(intakeHoldCommand().until(() -> !shootingAutonIntake.isOpen()))
            .andThen(intakeSS.stowCommand());
        Command shot = tagShotCommand(() -> false)
            .alongWith(drivetrain.applyRequest(() -> drivetrain.faceHeading(0, 0,
                tagAim.getTargetHeading(), tagAim.getHeadingRateFeedforward())))
            .withTimeout(SHOOTING_AUTON_SHOT_SEC);
        return Commands.sequence(
                Commands.deadline(
                    drivetrain.followBLinePathOnField(path,
                        () -> shootingAutonIntake.isOpen() ? intakeSpeed : Double.POSITIVE_INFINITY),
                    intakeStretch),
                shot,
                intakeSS.stowCommand())
            .beforeStarting(shootingAutonIntake::reset)
            .finallyDo(shootingAutonIntake::reset);
    }

    public Command getAutonomousCommand() {
        // Run whatever auto the drive team picked on the dashboard (chooser defaults to "None").
        Command selected = autoChooser.getSelected();
        return (selected != null) ? selected : Commands.none();
    }

    /**
     * AUTO START SLIDER CYCLE (team 2026-10-16): slider fully out and in, twice, rollers off, scheduled
     * beside the auto so the drive never waits; any intake use the auto makes (LT / RT / stow) takes
     * over from it. Only for an auto that drives (never "None": nothing moves unless an auto is picked)
     * and doesn't hold the intake itself -- those run it inside (a separate one would cancel them).
     */
    public Command autoStartSliderCycle(Command auto) {
        return runsSliderCycleBeside(auto, drivetrain, intakeSS) ? intakeSS.sliderCycleCommand() : Commands.none();
    }

    /**
     * AUTO START UNJAM (team 2026-10-17): VIEW's unjam (belts + feeder backward, slow) for
     * AUTO_START_UNJAM_SEC beside every auto that drives (never "None"). Scheduled BEFORE the auto, so
     * an auto that holds the hopper/launcher from its first tick simply cancels it (the auto wins).
     */
    public Command autoStartUnjam(Command auto) {
        return auto.hasRequirement(drivetrain)
            ? hopperSS.unjamCommand().alongWith(launcherSS.feederUnjamCommand()).withTimeout(AUTO_START_UNJAM_SEC)
            : Commands.none();
    }

    private static final double AUTO_START_UNJAM_SEC = 1.0; // team 2026-10-17

    public static boolean runsSliderCycleBeside(Command auto, Subsystem drive, Subsystem intake) {
        return auto.hasRequirement(drive) && !auto.hasRequirement(intake);
    }
}
