package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

/**
 * Proves every BLine file in src/main/deploy/autos parses at the desk (the robot auto-adds them
 * all to the chooser), and pins the relative following on both alliances.
 */
public class BLinePathTest {
    private static Path path;

    @BeforeAll
    static void load() {
        HAL.initialize(500, 0);
        path = new Path(new File("src/main/deploy/autos"), "forward-3m-left-3m");
    }

    @Test
    void testPathParsesAndStartsAtRightTrench() {
        assertTrue(path.isValid());
        assertEquals(0.0, path.getStartPose().getTranslation().getDistance(
            FieldConstants.StartSpot.RIGHT_TRENCH.bluePose.getTranslation()), 1e-9);
        // config.json must win over BLine's 4.5 m/s / 7 m/s^2 fallback (team: match PathPlanner).
        assertEquals(3.0, path.getDefaultGlobalConstraints().getMaxVelocityMetersPerSec(), 1e-9);
        assertEquals(3.0, path.getDefaultGlobalConstraints().getMaxAccelerationMetersPerSec2(), 1e-9);
    }

    @Test
    void followRunsRelativeToWhereTheRobotIsOnBothAlliances() {
        // Team 2026-10-10: BLine sees the authored start (never flipped), the field pose is untouched.
        var drivetrain = TunerConstants.createDrivetrain();
        Pose2d robot = new Pose2d(10.0, 3.0, Rotation2d.kCCW_90deg);
        for (var station : new AllianceStationID[] {AllianceStationID.Red1, AllianceStationID.Blue1}) {
            DriverStationSim.setAllianceStationId(station);
            DriverStationSim.notifyNewData();
            drivetrain.resetPose(robot);
            var cmd = drivetrain.followBLinePath(path);
            cmd.initialize();
            assertEquals(0.0, drivetrain.getState().Pose.minus(robot).getTranslation().getNorm(), 1e-6,
                station.name() + ": field pose not reset");
            Pose2d seen = drivetrain.blineRelativePose();
            assertEquals(0.0, seen.getTranslation().getDistance(path.getStartPose().getTranslation()), 1e-6,
                station.name() + ": BLine starts at the authored start");
            assertEquals(path.getStartPose().getRotation().getDegrees(), seen.getRotation().getDegrees(), 1e-6);
            cmd.end(true);
        }
    }

    @Test
    void chainedPathCarriesOnFromWhereThePreviousOneLeftOff() {
        // Team 2026-10-14 (path 1): part 2 starts 17 cm / 7 deg off part 1's end; restarting at its
        // own start would rotate all of part 2 7 deg toward the side wall. keepFrame keeps BLine's pose.
        File autos = new File("src/main/deploy/autos");
        Path pt1 = new Path(autos, "path_1_pt.1");
        Path pt2 = new Path(autos, "path-1-pt-2");
        var drivetrain = TunerConstants.createDrivetrain();
        drivetrain.resetPose(new Pose2d(10.0, 3.0, Rotation2d.kCCW_90deg));
        var first = drivetrain.followBLinePath(pt1);
        first.initialize();
        first.end(false);
        var chained = drivetrain.followBLinePath(pt2, true);
        chained.initialize();
        Pose2d seen = drivetrain.blineRelativePose();
        assertEquals(0.0, seen.getTranslation().getDistance(pt1.getStartPose().getTranslation()), 1e-6,
            "part 2 starts where part 1 left BLine");
        assertEquals(pt1.getStartPose().getRotation().getDegrees(), seen.getRotation().getDegrees(), 1e-6);
        chained.end(true);
        var fresh = drivetrain.followBLinePath(pt2);
        fresh.initialize();
        assertEquals(0.0, drivetrain.blineRelativePose().getTranslation().getDistance(pt2.getStartPose().getTranslation()),
            1e-6, "unchained restarts at its own start");
        fresh.end(true);
    }

    @Test
    void chainedPathCarriesTheTurnButNotTheShiftBetweenPaths() {
        // Team 2026-10-16: path 1 aims (turns in place) between its parts. That turn must carry into the
        // next part or all of it runs rotated; the X/Y between parts (vision can shift it) is dropped.
        Pose2d seen = new Pose2d(2.04, 7.07, Rotation2d.fromDegrees(-50));
        Pose2d last = new Pose2d(10.0, 3.0, Rotation2d.fromDegrees(90));
        Pose2d now = new Pose2d(10.3, 2.8, Rotation2d.fromDegrees(100)); // turned +10, shifted 0.36 m
        Pose2d carried = CommandSwerveDrivetrain.carryTurn(seen, last, now);
        assertEquals(0.0, carried.getTranslation().getDistance(seen.getTranslation()), 1e-9, "X/Y dropped");
        assertEquals(-40.0, carried.getRotation().getDegrees(), 1e-9, "turn carried");
    }

    @Test
    void visionAutoGatesFromTheShotData() {
        // Team 2026-10-17 ("path 1 + vision"): mount pose first, then tags in the 10-10 data's 3.3 m range.
        assertEquals("mount pose unset", CommandSwerveDrivetrain.visionAutoReject(false, 2, 1.0, 0, 0));
        assertEquals("no tags", CommandSwerveDrivetrain.visionAutoReject(true, 0, 0, 0, 0));
        assertEquals("tags too far", CommandSwerveDrivetrain.visionAutoReject(true, 2, 3.4, 0, 0));
        assertEquals("turning", CommandSwerveDrivetrain.visionAutoReject(true, 2, 1.0, 0, -200));
        assertEquals("jump", CommandSwerveDrivetrain.visionAutoReject(true, 2, 1.0, 0.6, 0));
        assertNull(CommandSwerveDrivetrain.visionAutoReject(true, 1, 3.3, 0.5, 180), "edge values still fuse");
        // Under the blue trench: nearest tag (21/24) 2.4 m fuses even with wall tags 31/32 in frame (average ~3.8 m),
        // at lower trust than the near tags alone would get.
        assertNull(CommandSwerveDrivetrain.visionAutoReject(true, 4, 2.4, 0, 60), "trench frame fuses");
        assertTrue(CommandSwerveDrivetrain.visionAutoStdDev(4, 3.8) > CommandSwerveDrivetrain.visionAutoStdDev(2, 2.4));
        // Trust: 2+ tags 0.15 m, 1 tag 0.25 m, doubling by 3.3 m -- never near the 5 cm static scatter.
        assertEquals(0.15, CommandSwerveDrivetrain.visionAutoStdDev(3, 0.0), 1e-9);
        assertEquals(0.30, CommandSwerveDrivetrain.visionAutoStdDev(2, 3.3), 1e-9);
        assertEquals(0.25 * (1 + Math.pow(2.0 / 3.3, 2)), CommandSwerveDrivetrain.visionAutoStdDev(1, 2.0), 1e-9);
        // Step 3 far tags (team 2026-10-17): the opponent's hub/trench tags pass the 3.3 m gate out to 4.5 m.
        var red = frc.robot.Constants.SubsystemConstants.Vision.RED_HUB_TRENCH_TAGS;
        assertEquals(3.3, CommandSwerveDrivetrain.gateTagDist(1, 4.5, red), 1e-9);
        assertNull(CommandSwerveDrivetrain.visionAutoReject(true, 1, CommandSwerveDrivetrain.gateTagDist(3, 4.4, red), 0, 0));
        assertEquals("tags too far", CommandSwerveDrivetrain.visionAutoReject(true, 1, CommandSwerveDrivetrain.gateTagDist(3, 4.6, red), 0, 0));
        assertEquals(3.4, CommandSwerveDrivetrain.gateTagDist(17, 3.4, red), 1e-9, "other tags keep the 3.3 m cap");
        assertEquals(4.4, CommandSwerveDrivetrain.gateTagDist(3, 4.4, java.util.Set.of()), 1e-9, "off = every tag at 3.3 m");
    }

    @Test
    void blineAimTurnsTowardTheTargetAndIsCapped() {
        // Team 2026-10-16: path 1's aims replace BLine's rotation with faceHeading's gains and rate cap.
        var pid = new edu.wpi.first.math.controller.PIDController(5.0, 0, 0);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        double small = CommandSwerveDrivetrain.aimOmega(pid, Rotation2d.kZero, Rotation2d.fromDegrees(5), 0);
        assertEquals(5.0 * Math.toRadians(5), small, 1e-9, "turns CCW toward a CCW target");
        assertEquals(-1.5 * Math.PI, CommandSwerveDrivetrain.aimOmega(pid, Rotation2d.fromDegrees(170),
            Rotation2d.fromDegrees(10), 0), 1e-9, "capped, short way round (CW through 90)");
        assertEquals(0.3, CommandSwerveDrivetrain.aimOmega(pid, Rotation2d.kZero, Rotation2d.kZero, 0.3), 1e-9,
            "feedforward added");
        // Team 2026-10-17: the auto feed aim turns at LB's gentler cap.
        double feedCap = frc.robot.Constants.SubsystemConstants.AutoAimConstants.FEED_MAX_ROTATION_RAD_PER_SEC;
        assertEquals(-feedCap, CommandSwerveDrivetrain.aimOmega(pid, Rotation2d.fromDegrees(170),
            Rotation2d.fromDegrees(10), 0, feedCap), 1e-9, "feed aim capped at LB's turn rate");
    }

    @Test
    void path1StaysOffTheWallsAndCentredUnderTheTrench() {
        // Team 2026-10-14: authored with the wall corners at x = 0.53/0.61 m at 3 m/s. BLine doesn't
        // slow for a corner and config.json allows 3 m/s^2, so it would hit the wall at ~2.5 m/s.
        // Moved to Shooting auton's 0.80 m with the wall legs at 1 m/s (then 0.60 m 2026-10-17: the bumper ran
        // ~10 in off the wall, team wants ~2 in; ideal sim 5.6 in, the robot runs ~3.5 in closer); part 2's trench point is
        // centred on the trench tags (y = 7.425, ~6 cm per side while it turns). A BLine Web
        // re-export would undo these -- this catches it.
        File autos = new File("src/main/deploy/autos");
        var pt1 = new Path(autos, "path_1_pt.1").getPathElementsWithConstraintsNoWaypoints().stream()
            .filter(p -> p.getFirst() instanceof Path.TranslationTarget).toList();
        for (int i : new int[] {1, 2}) {
            assertEquals(0.60, ((Path.TranslationTarget) pt1.get(i).getFirst()).translation().getX(), 1e-9,
                "wall point " + i);
            assertEquals(1.0, ((Path.TranslationTargetConstraint) pt1.get(i).getSecond()).maxVelocityMetersPerSec(),
                1e-9, "wall leg " + i);
        }
        var pt2 = new Path(autos, "path-1-pt-2").getPathElementsWithConstraintsNoWaypoints().stream()
            .filter(p -> p.getFirst() instanceof Path.TranslationTarget).toList();
        assertEquals(7.425, ((Path.TranslationTarget) pt2.get(1).getFirst()).translation().getY(), 1e-9,
            "trench point");
    }

    @Test
    void postFeedShotPathStartsWherePart2Ends() {
        // Team 2026-10-16: chained straight after pt 2 (keepFrame), so its start must be pt 2's end.
        File autos = new File("src/main/deploy/autos");
        var pt2 = new Path(autos, "path-1-pt-2").getPathElementsWithConstraintsNoWaypoints().stream()
            .filter(p -> p.getFirst() instanceof Path.TranslationTarget).toList();
        var pt2End = ((Path.TranslationTarget) pt2.get(pt2.size() - 1).getFirst()).translation();
        Pose2d post = new Path(autos, "path1-pt2-postfeedshot").getStartPose();
        assertEquals(0.0, post.getTranslation().getDistance(pt2End), 1e-9);
        assertEquals(2.53265, post.getRotation().getRadians(), 1e-9, "pt 2's end heading");
    }

    @Test
    void travelCountsShortOnlyWhileTurningSlowly() {
        // Team 2026-10-12/14: straight travel counts 1:1; travel while turning under 1 m/s counts 0.88,
        // so BLine drives farther on slow rotating paths only. Heading always passes through unscaled.
        Pose2d start = new Pose2d(1.0, 7.0, Rotation2d.kZero);
        Pose2d last = new Pose2d(5.0, 2.0, Rotation2d.kCCW_90deg);
        Pose2d now = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(135)); // 1 m along the robot's forward
        Pose2d straight = CommandSwerveDrivetrain.stepRelative(start, last, now, 0.05, 0.5);
        assertEquals(2.0, straight.getX(), 1e-9);
        assertEquals(7.0, straight.getY(), 1e-9);
        Pose2d turning = CommandSwerveDrivetrain.stepRelative(start, last, now, -0.36, 0.5);
        assertEquals(0.88, CommandSwerveDrivetrain.kBLineTurningDistanceFactor, 1e-9);
        assertEquals(1.88, turning.getX(), 1e-9);
        assertEquals(45.0, turning.getRotation().getDegrees(), 1e-9);
        Pose2d fastTurn = CommandSwerveDrivetrain.stepRelative(start, last, now, -0.36, 2.5);
        assertEquals(2.0, fastTurn.getX(), 1e-9, "fast turning legs (path 1) count 1:1");
    }

    @Test
    void path1Part1HoldsLtFromIntakeToIntakeStopThenHubShots() {
        // Team 2026-10-16: part 1 intakes (LT held) from its "intake" event (t = 0.49 on the leg to
        // waypoint 2) until "intake_stop" (t = 0.51 on the last leg), and "hub_shot" sits at its end.
        Translation2d[] pts = {new Translation2d(3.6151, 4.94494), new Translation2d(0.60, 5.00189),
            new Translation2d(0.60, 7.15818), new Translation2d(2.04, 7.07)};
        Path1Events events = new Path1Events();
        FollowPath.registerEventTrigger("intake", events::onIntake);
        FollowPath.registerEventTrigger("intake_stop", events::onIntakeStop);
        FollowPath.registerEventTrigger("hub_shot", events::onHubShot);
        Path pt1 = new Path(new File("src/main/deploy/autos"), "path_1_pt.1");
        var drivetrain = TunerConstants.createDrivetrain();
        Rotation2d heading = pt1.getStartPose().getRotation();
        drivetrain.resetPose(new Pose2d(pts[0], heading)); // relative frame = field frame here
        var cmd = drivetrain.followBLinePath(pt1);
        // Path 1's early pt 1 / pt 2 finish (team 2026-10-17) reads this key: a BLine rename would undo it.
        double[] remaining = {Double.NaN, Double.POSITIVE_INFINITY}; // first, smallest
        FollowPath.setDoubleLoggingConsumer(p -> {
            if (p.getFirst().equals("FollowPath/remainingPathDistanceMeters")) {
                if (Double.isNaN(remaining[0])) remaining[0] = p.getSecond();
                remaining[1] = Math.min(remaining[1], p.getSecond());
            }
        });
        events.start();
        cmd.initialize();
        for (int leg = 0; leg < 3; leg++) {
            for (int i = 0; i <= 100; i += 5) {
                drivetrain.resetPose(new Pose2d(pts[leg].interpolate(pts[leg + 1], i / 100.0), heading));
                cmd.execute();
                String at = "leg " + leg + " at " + i + "%";
                if ((leg == 0 && i >= 50) || leg == 1 || (leg == 2 && i <= 50)) { // events at t = 0.4946 and 0.5105
                    assertTrue(events.intake(), at);
                } else {
                    assertFalse(events.intake(), at);
                }
                assertEquals(leg == 2 && i == 100, events.hubShot(), at);
            }
        }
        cmd.end(true);
        events.stop();
        FollowPath.setDoubleLoggingConsumer(p -> {});
        assertTrue(remaining[0] > 5.0, "remaining distance logged from the start: " + remaining[0]);
        assertTrue(remaining[1] <= frc.robot.RobotContainer.PATH1_EARLY_END_METERS, "drops under the early-finish line: " + remaining[1]);
    }

    @Test
    void pt1SoloHoldsLtFromIntakeToWaypoint3ThenShoots() {
        // Team 2026-10-16 (path 1 pt 1 solo, "path_1_pt.1-2"): LT held from "intake" (t = 0.555 on the
        // leg to waypoint 2) until BLine reaches waypoint 3 (0.45 m handoff), "shoot" at the very end.
        Translation2d[] pts = {new Translation2d(3.6151, 4.94494), new Translation2d(0.80, 5.00189),
            new Translation2d(0.80, 7.16), new Translation2d(2.04, 7.07)};
        BLineEventWindow window = new BLineEventWindow(2);
        boolean[] shoot = {false};
        FollowPath.setDoubleLoggingConsumer(p -> {
            if (p.getFirst().equals("FollowPath/translationElementIndex")) window.onIndex(p.getSecond());
        });
        FollowPath.registerEventTrigger("intake", window::onEvent);
        FollowPath.registerEventTrigger("shoot", () -> shoot[0] = true);
        Path pt1 = new Path(new File("src/main/deploy/autos"), "path_1_pt.1-2");
        var drivetrain = TunerConstants.createDrivetrain();
        Rotation2d heading = pt1.getStartPose().getRotation();
        drivetrain.resetPose(new Pose2d(pts[0], heading)); // relative frame = field frame here
        var cmd = drivetrain.followBLinePath(pt1);
        window.reset();
        cmd.initialize();
        for (int leg = 0; leg < 3; leg++) {
            for (int i = 0; i <= 100; i += 5) {
                drivetrain.resetPose(new Pose2d(pts[leg].interpolate(pts[leg + 1], i / 100.0), heading));
                cmd.execute();
                String at = "leg " + leg + " at " + i + "%";
                if ((leg == 0 && i >= 60) || (leg == 1 && i <= 70)) {
                    assertTrue(window.isOpen(), at);
                } else if ((leg == 0 && i <= 55) || (leg == 1 && i >= 85) || leg == 2) {
                    assertFalse(window.isOpen(), at);
                }
                assertEquals(leg == 2 && i == 100, shoot[0], at);
            }
        }
        cmd.end(true);
    }

    @Test
    void pt1SoloStaysOffTheWall() {
        // Team 2026-10-16: exported with the wall points at x = 0.53 / 0.61 and the wall approach at
        // 3.03 m/s -- with LT held the intake would hit the wall. Moved to 0.80 at 1 m/s; a BLine Web
        // re-export would undo it -- this catches it.
        File autos = new File("src/main/deploy/autos");
        var pts = new Path(autos, "path_1_pt.1-2").getPathElementsWithConstraintsNoWaypoints().stream()
            .filter(p -> p.getFirst() instanceof Path.TranslationTarget).toList();
        for (int i : new int[] {1, 2}) {
            assertEquals(0.80, ((Path.TranslationTarget) pts.get(i).getFirst()).translation().getX(), 1e-9,
                "wall point " + i);
            assertEquals(1.0, ((Path.TranslationTargetConstraint) pts.get(i).getSecond()).maxVelocityMetersPerSec(),
                1e-9, "wall leg " + i);
        }
        assertEquals(java.util.List.of("intake", "event", "shoot"), eventKeys("path_1_pt.1-2"));
    }

    @Test
    void path1EventKeysAreOnePerAction() {
        // Team 2026-10-16: triggers 1-8 in order. BLine Web exports every trigger as "event", so a
        // re-export without renaming them there would silently drop every action -- this catches it.
        assertEquals(java.util.List.of("intake", "intake_stop", "hub_shot"), eventKeys("path_1_pt.1"));
        assertEquals(java.util.List.of("stow", "intake"), eventKeys("path-1-pt-2")); // feed shot removed 2026-10-17
        assertEquals(java.util.List.of("feed_stop", "hub_shot"), eventKeys("path1-pt2-postfeedshot"));
    }

    private static java.util.List<String> eventKeys(String name) {
        return new Path(new File("src/main/deploy/autos"), name).getPathElements().stream()
            .filter(e -> e instanceof Path.EventTrigger).map(e -> ((Path.EventTrigger) e).libKey()).toList();
    }

    @Test
    void path1EventsOnlyCountWhileTheAutoRuns() {
        // BLine's keys are global: running part 2 on its own must never leave the feed shot on.
        Path1Events events = new Path1Events();
        events.onFeedShot();
        events.onIntake();
        assertFalse(events.feedShot() || events.intake(), "ignored before the auto starts");
        events.start();
        events.onIntake();
        events.onFeedShot();
        events.onFeedStop();
        assertTrue(events.intake(), "feed off keeps LT held (team: intake runs until trigger 8)");
        events.onHubShot();
        assertTrue(events.hubShot());
        assertFalse(events.intake(), "the hub shot lets LT go");
        events.onStow();
        events.stop();
        assertFalse(events.hubShot() || events.stow() || events.feedShot() || events.intake(), "all cleared");
        events.onHubShot();
        assertFalse(events.hubShot(), "ignored after the auto ends");
    }

    @Test
    void redVisionAutoFusesOnlyRedSideTagFrames() {
        // Team 2026-10-17: path 1 + vision on red trusts only tags 1-16; one blue-side tag drops the frame.
        java.util.function.IntFunction<frc.robot.subsystems.utility.LimelightHelpers.RawFiducial> tag =
            id -> new frc.robot.subsystems.utility.LimelightHelpers.RawFiducial(id, 0, 0, 1, 2, 2, 0.1);
        var ll = new frc.robot.subsystems.utility.LimelightHelpers.RawFiducial[] {tag.apply(10), tag.apply(1), tag.apply(16)};
        assertTrue(CommandSwerveDrivetrain.allRedSideTags(ll), "red hub + trench + wall tags");
        assertFalse(CommandSwerveDrivetrain.allRedSideTags(new frc.robot.subsystems.utility.LimelightHelpers.RawFiducial[] {tag.apply(10), tag.apply(17)}), "a blue-side tag drops it");
        assertFalse(CommandSwerveDrivetrain.allRedSideTags(new frc.robot.subsystems.utility.LimelightHelpers.RawFiducial[] {tag.apply(32)}), "blue only");
        assertFalse(CommandSwerveDrivetrain.allRedSideTags(new frc.robot.subsystems.utility.LimelightHelpers.RawFiducial[] {null}), "no tag data");
        assertFalse(CommandSwerveDrivetrain.allRedSideTags(null), "no per-tag data");
    }

    @Test
    void threeStepFirstHubShotIsPreSpunWithNoReverse() {
        // Team 2026-10-17: pre-spin from intake_stop until the first hub shot, which skips the reverse;
        // the later hub shot (post) keeps it. Plain start (pt 2 / pt 3 autos) never pre-spins or skips.
        Path1Events e = new Path1Events();
        e.startThreeStep();
        e.onIntake();
        assertFalse(e.preSpin(), "not while intaking");
        e.onIntakeStop();
        assertTrue(e.preSpin(), "pre-spin from intake_stop");
        assertFalse(e.skipShotReverse(), "nothing to skip before the shot");
        e.onHubShot();
        assertFalse(e.preSpin(), "the shot takes over");
        assertTrue(e.skipShotReverse(), "first hub shot: no reverse");
        e.endHubShot();
        e.onIntake();
        e.onIntakeStop();
        assertFalse(e.preSpin(), "only before the first shot");
        e.onHubShot();
        assertFalse(e.skipShotReverse(), "later hub shots keep the reverse");
        e.stop();
        e.start();
        e.onIntakeStop();
        e.onHubShot();
        assertFalse(e.preSpin() || e.skipShotReverse(), "plain start: normal shot");
    }

    @Test
    void threeStepPath1IgnoresTheFeedShot() {
        // Team 2026-10-17: path 1 (+ vision) runs no feed shot; LT stays held through the event.
        Path1Events events = new Path1Events();
        events.startThreeStep();
        events.onIntake();
        events.onFeedShot();
        assertFalse(events.feedShot(), "feed_shot ignored");
        assertTrue(events.intake(), "LT still held");
        events.start();
        events.onFeedShot();
        assertTrue(events.feedShot(), "the next plain start counts it again (path 1 pt 2)");
    }

    @Test
    void postFeedShotCornerIsCappedSoItCrossesTheBump() {
        // Team 2026-10-17: at the authored 4.05 / 2.96 m/s up-field the robot overshot the corner (3 m/s^2 turn)
        // and crossed toward the trench's inner post (6.79) instead of over the bump -- the old export at y ~6.0,
        // the 17:47 re-export ((8.76, 5.54) -> (8.02, 6.13)) within ~4 cm in an ideal sim. Both legs (targets 3
        // and 4) are capped at 2.0 m/s (re-export: frame top 6.05, bottom 4.90 by the hub). Re-make in BLine Web.
        var caps = new Path(new File("src/main/deploy/autos"), "path1-pt2-postfeedshot")
            .getPathConstraints().getMaxVelocityMetersPerSec().orElseThrow();
        for (int target : new int[] {3, 4}) {
            var cap = caps.stream().filter(c -> c.startOrdinal() <= target && c.endOrdinal() >= target).findFirst().orElseThrow();
            assertEquals(2.0, cap.value(), 1e-9, "target " + target);
        }
    }

    @Test
    void turnAroundPathsAreCappedAt1MetrePerSecond() {
        // BLine doesn't slow for a 180-degree turn-around; at 3 m/s it would overshoot ~1-1.3 m.
        // A malformed constraint is silently ignored by the parser, so pin that the cap loaded.
        for (String name : new String[] {"forward-2m-back-2m", "pid-move-tune"}) {
            Path p = new Path(new File("src/main/deploy/autos"), name);
            var caps = p.getPathConstraints().getMaxVelocityMetersPerSec().orElseThrow();
            assertEquals(1, caps.size(), name);
            assertEquals(1.0, caps.get(0).value(), 1e-9, name);
            assertEquals(0, caps.get(0).startOrdinal(), name);
            assertEquals(2, caps.get(0).endOrdinal(), name);
        }
    }

    @Test
    void calibPathsAre1MetreStraightsCappedAtHalfAMetrePerSecond() {
        // Team 2026-10-10: authored in BLine Web with no cap; 0.5 m/s added for the first runs.
        // rot-calib is the same drive with a 90 deg left turn on the way; calib2 starts at (1.0, 7.0);
        // rot-calib2 (team 2026-10-12) is a 2 m drive from (1.0, 7.0) with the same 90 deg left turn.
        // calib3-side (team 2026-10-12) is a 2 m strafe right from (2.0, 7.0), no turn.
        for (String name : new String[] {"calib", "rot-calib", "calib2", "rot-calib2", "calib3-side"}) {
            Path calib = new Path(new File("src/main/deploy/autos"), name);
            var start = name.endsWith("calib2") ? new Translation2d(1.0, 7.0)
                : name.equals("calib3-side") ? new Translation2d(2.0, 7.0) : new Translation2d(3.0, 0.5);
            assertEquals(0.0, calib.getStartPose().getTranslation().getDistance(start), 1e-9, name);
            var caps = calib.getPathConstraints().getMaxVelocityMetersPerSec().orElseThrow();
            assertEquals(1, caps.size(), name);
            assertEquals(0.5, caps.get(0).value(), 1e-9, name);
            assertEquals(0, caps.get(0).startOrdinal(), name);
            assertEquals(1, caps.get(0).endOrdinal(), name);
        }
    }

    @Test
    void everyDeployedPathParses() {
        File autos = new File("src/main/deploy/autos");
        var names = RobotContainer.blinePathNames(new File(autos, "paths"));
        assertTrue(names.contains("forward-3m-left-3m"), "lister finds the deployed paths");
        for (String name : names) {
            assertTrue(new Path(autos, name).isValid(), name + " is a valid path");
        }
        assertTrue(RobotContainer.blinePathNames(new File(autos, "missing")).isEmpty(), "no folder = no paths");
    }

    @Test
    void followAddsTheFrictionFloorsWithoutTouchingThePath() {
        // Team 2026-10-12: floors live in code, not in the files -- the authored path stays as-is.
        File autos = new File("src/main/deploy/autos");
        Path authored = new Path(autos, "rot-calib2");
        Path floored = CommandSwerveDrivetrain.withFrictionFloors(authored);
        assertTrue(authored.getPathConstraints().getMinVelocityMetersPerSec().isEmpty(), "file has no floor");
        var minT = floored.getPathConstraints().getMinVelocityMetersPerSec().orElseThrow();
        var minR = floored.getPathConstraints().getMinVelocityDegPerSec().orElseThrow();
        assertEquals(0.15, minT.get(0).value(), 1e-9);
        assertEquals(20.0, minR.get(0).value(), 1e-9);
        assertEquals(Integer.MAX_VALUE, minT.get(0).endOrdinal(), "whole path");
        assertEquals(0.5, floored.getPathConstraints().getMaxVelocityMetersPerSec().orElseThrow().get(0).value(), 1e-9,
            "authored cap kept");
    }
}
