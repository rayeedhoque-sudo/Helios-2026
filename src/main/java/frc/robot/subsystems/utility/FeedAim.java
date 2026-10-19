package frc.robot.subsystems.utility;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.AprilTagConstants;
import frc.robot.Constants.SubsystemConstants.AutoAimConstants;
import frc.robot.Constants.SubsystemConstants.LauncherConstants;
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.utility.LimelightHelpers.RawFiducial;

/**
 * LB FEED SHOT (remade 2026-10-17 from the team's field datapoints, docs/feed-data/p*.json).
 * ALWAYS (LB held or not), ANY AprilTag in view -- either alliance -- places the robot on the
 * field: the tag's field spot minus where the camera sees it, turned by the Pigeon heading (so
 * MENU must be zeroed right). A placement behind the tag's face or off the field (a flipped
 * heading) is thrown away. Each placement is pinned as an offset from odometry, so the position
 * carries between frames, between LB presses and while driving; a pose reset / jump of more than
 * FEED_MAX_ODOM_JUMP_METERS in one loop (auto start, the MT1 boot seed) drops it until the next tag.
 * <p>
 * Everything after that is worked out in the RED-alliance frame (blue = the field turned 180 deg
 * about its centre) and never uses which side (y) of the field the robot is on, so the team's
 * low-y red data covers the high-y side mirrored, and blue mirrors red:
 * <ul>
 * <li>heading: straight toward OUR alliance wall (+ TagAim's crosshair + trim offset). Inside
 *     +-FEED_HEADING_WINDOW_DEG of it the robot is NOT turned; outside, only back to the window's
 *     edge (RobotContainer caps that turn at FEED_MAX_ROTATION_RAD_PER_SEC);</li>
 * <li>flywheel rpm: FEED_RPM_BY_WALL_DIST at the distance to our wall from where the robot WILL be
 *     after the ball's flight time (robot motion, team 2026-10-17); hood: max (RobotContainer);</li>
 * <li>NO SHOT, aim only, while a hub is in the way: behind either hub, inside its width and
 *     closer than FEED_HUB_BLOCK_DEPTH to its centre (team point p6).</li>
 * </ul>
 * No position known yet (no tag since boot / since a pose reset): turn toward our own hub by the
 * drivetrain's odometry to find tags (TagAim's no-tag fallback); the belts wait (team 2026-10-17).
 */
public class FeedAim extends SubsystemBase {

    private static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final double MAX_FRAME_AGE_SEC = 0.25; // same as TagAim

    private final CommandSwerveDrivetrain drivetrain;
    private boolean active = false;  // LB held
    private boolean placed = false;  // a tag placed the robot (kept across presses, dropped on a pose jump)
    private Translation2d lastOdom = Translation2d.kZero;
    private Translation2d odomMinusField = Translation2d.kZero; // robot field position = odometry - this
    private Rotation2d targetHeading = Rotation2d.kZero;
    private double wallDistanceMeters = 0;
    private boolean aimed = false;
    private boolean blocked = false;
    private double crosshairOffsetDeg = 0;
    private double lastFrameSec = -1;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter Subsystem Tab");
    private final GenericEntry tagEntry = tab.add("Feed Tag", -1.0).getEntry();
    private final GenericEntry wallDistanceEntry = tab.add("Feed Wall Distance (m)", 0.0).getEntry();
    private final GenericEntry aimedEntry = tab.add("Feed Aimed", false).getEntry();
    private final GenericEntry blockedEntry = tab.add("Feed Blocked (behind hub)", false).getEntry();

    public FeedAim(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    /** LB pressed / released. The robot's position (if known) carries over; aim and block start fresh. */
    public void setActive(boolean active){
        this.active = active;
        aimed = false;
        blocked = false;
    }

    public boolean isActive(){ return active; }

    /** The robot's field position is known (a tag placed it). False = turning to our hub, belts waiting. */
    public boolean hasTarget(){ return placed; }

    /** Position known, not blocked, heading within FEED_HEADING_TOLERANCE_DEG of the feed heading. */
    public boolean isAimed(){ return aimed; }

    /** A hub is between the robot and our wall, too close to clear: no shooting, aim only. */
    public boolean isBlocked(){ return blocked; }

    /** Blue-origin heading to servo to: the windowed feed heading, or toward our hub until placed. */
    public Rotation2d getTargetHeading(){ return targetHeading; }

    /** Flywheel rpm: the wall-distance table once placed, else LB_SHOT_FLYWHEEL_RPM (spin-up while waiting). */
    public double getFlywheelRpm(){
        return placed ? AutoAimConstants.FEED_RPM_BY_WALL_DIST.get(wallDistanceMeters)
            : LauncherConstants.LB_SHOT_FLYWHEEL_RPM;
    }

    @Override
    public void periodic(){
        var state = drivetrain.getState();
        Pose2d pose = state.Pose;
        // A pose reset (auto start, the MT1 boot seed) moves odometry under the pinned offset: drop it.
        if (pose.getTranslation().getDistance(lastOdom) > AutoAimConstants.FEED_MAX_ODOM_JUMP_METERS) {
            placed = false;
        }
        lastOdom = pose.getTranslation();
        String cam = Vision.CAM_LIMELIGHT;
        if (LimelightHelpers.getTV(cam)) {
            crosshairOffsetDeg = LimelightHelpers.getTXNC(cam) - LimelightHelpers.getTX(cam);
        }
        // Same frame rules as TagAim: each frame used once, and only while fresh.
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
        boolean newFrame = estimate != null && estimate.timestampSeconds != lastFrameSec
            && Timer.getFPGATimestamp() - estimate.timestampSeconds <= MAX_FRAME_AGE_SEC;
        if (newFrame) {
            lastFrameSec = estimate.timestampSeconds;
            RawFiducial tag = pickTag(AprilTagConstants.remap(estimate.rawFiducials));
            Optional<Pose2d> poseAtFrame = tag != null ? drivetrain.samplePoseAt(estimate.timestampSeconds) : Optional.empty();
            if (poseAtFrame.isPresent()) {
                Pose2d tagPose = tagPose(tag.id).orElseThrow(); // pickTag only returns field tags
                Translation2d robot = robotField(tag.txnc, tag.tync, tag.distToCamera,
                    poseAtFrame.get().getRotation(), tagPose.getTranslation(), AutoAimConstants.TAG_AIM_TX_SIGN);
                if (plausible(robot, tagPose)) {
                    odomMinusField = poseAtFrame.get().getTranslation().minus(robot);
                    placed = true;
                    tagEntry.setDouble(tag.id);
                }
            }
        }
        if (!active) {
            aimed = false;
            aimedEntry.setBoolean(false);
            blockedEntry.setBoolean(false);
            return;
        }
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (placed && alliance.isPresent()) {
            // Every loop, not just on frames: between frames odometry carries the shot.
            Translation2d red = redFrame(pose.getTranslation().minus(odomMinusField), alliance.get());
            Translation2d velocity = ShotCalculator.fieldVelocity(state.Speeds, pose.getRotation());
            wallDistanceMeters = leadWallDistance(red, alliance.get() == Alliance.Red ? velocity : velocity.unaryMinus());
            Rotation2d feedHeading = wallHeading(alliance.get())
                .plus(Rotation2d.fromDegrees(TagAim.aimOffsetDeg(crosshairOffsetDeg)));
            targetHeading = holdInWindow(pose.getRotation(), feedHeading, AutoAimConstants.FEED_HEADING_WINDOW_DEG);
            blocked = blocked(blocked, red);
            aimed = !blocked && Math.abs(feedHeading.minus(pose.getRotation()).getDegrees())
                <= AutoAimConstants.FEED_HEADING_TOLERANCE_DEG;
            wallDistanceEntry.setDouble(wallDistanceMeters);
        } else {
            // No position yet: face our hub by odometry to bring tags into view; never feed on it.
            targetHeading = TagAim.odometryHeading(pose.getTranslation(), alliance).orElse(pose.getRotation());
            aimed = false;
            blocked = false;
        }
        aimedEntry.setBoolean(aimed);
        blockedEntry.setBoolean(blocked);
    }

    //Pure helpers (package-private for FeedAimTest)
        /** The biggest (closest) tag in view that is on the field map, with a camera range. Else null. */
        static RawFiducial pickTag(RawFiducial[] fiducials){
            RawFiducial best = null;
            if (fiducials == null) {
                return null;
            }
            for (RawFiducial f : fiducials) {
                if (tagPose(f.id).isPresent() && f.distToCamera > 0.01 && (best == null || f.ta > best.ta)) {
                    best = f;
                }
            }
            return best;
        }

        /** The tag's ground pose on the field (2026-rebuilt-welded.json): spot + the way its face points. */
        static Optional<Pose2d> tagPose(int tagId){
            return FIELD.getTagPose(tagId).map(Pose3d::toPose2d);
        }

        /**
         * The robot's field position from one tag sighting: the tag's field spot minus where the camera
         * sees it (ground range + bearing), turned into the field by the heading at the frame.
         * ponytail: camera taken as the robot centre, like TagAim (the feed data was keyed the same way).
         */
        static Translation2d robotField(double txncDeg, double tyncDeg, double distToCamera,
                Rotation2d headingAtFrame, Translation2d tagField, double txSign){
            double ground = distToCamera * Math.cos(Math.toRadians(tyncDeg)); // drop the height
            return tagField.minus(new Translation2d(ground, Rotation2d.fromDegrees(-txSign * txncDeg))
                .rotateBy(headingAtFrame));
        }

        /**
         * A placement the camera could really have seen from: in front of the tag's face and on the
         * field. A heading 180 deg off (MENU pressed the wrong way) puts the robot behind the tag.
         */
        static boolean plausible(Translation2d robot, Pose2d tag){
            Translation2d out = robot.minus(tag.getTranslation());
            boolean inFront = out.getX() * tag.getRotation().getCos() + out.getY() * tag.getRotation().getSin() > 0;
            return inFront && robot.getX() >= 0 && robot.getX() <= FieldConstants.FIELD_LENGTH_METERS
                && robot.getY() >= 0 && robot.getY() <= FieldConstants.FIELD_WIDTH_METERS;
        }

        /** A field position seen from the red alliance: blue turns the field 180 deg about its centre. */
        static Translation2d redFrame(Translation2d field, Alliance alliance){
            return alliance == Alliance.Red ? field : new Translation2d(
                FieldConstants.FIELD_LENGTH_METERS - field.getX(), FieldConstants.FIELD_WIDTH_METERS - field.getY());
        }

        /**
         * Red-frame wall distance from where the ball effectively leaves: the robot moved on by its
         * red-frame velocity x the feed's flight time (driving at our wall = less rpm).
         * ponytail: one pass -- flight time barely changes over the ~0.4 m lead at 10% drive.
         */
        static double leadWallDistance(Translation2d redFrameRobot, Translation2d redFrameVelocity){
            double now = FieldConstants.FIELD_LENGTH_METERS - redFrameRobot.getX();
            return now - redFrameVelocity.getX() * AutoAimConstants.FEED_TOF_SEC_BY_WALL_DIST.get(now);
        }

        /**
         * Inside +-windowDeg of the feed heading: the current heading (no turn). Outside: the window's
         * nearest edge, so the robot only turns back as far as it must.
         */
        static Rotation2d holdInWindow(Rotation2d heading, Rotation2d feedHeading, double windowDeg){
            double offDeg = heading.minus(feedHeading).getDegrees(); // -180..180
            return feedHeading.plus(Rotation2d.fromDegrees(MathUtil.clamp(offDeg, -windowDeg, windowDeg)));
        }

        /** Blue-origin heading straight toward our alliance wall: red +X (0 deg), blue -X (180 deg). */
        static Rotation2d wallHeading(Alliance alliance){
            return alliance == Alliance.Red ? Rotation2d.kZero : Rotation2d.k180deg;
        }

        /**
         * Red frame: a hub (ours = HUB_CENTER_RED, theirs = HUB_CENTER_BLUE) between the robot and our
         * wall, the robot inside FEED_HUB_BLOCK_HALF_WIDTH of its centre line and under
         * FEED_HUB_BLOCK_DEPTH in front of its centre = the straight feed hits it. Once blocked, both
         * limits grow by FEED_HUB_BLOCK_HYST so tag noise at the edge can't flicker the shot.
         */
        static boolean blocked(boolean wasBlocked, Translation2d redFrameRobot){
            double grow = wasBlocked ? AutoAimConstants.FEED_HUB_BLOCK_HYST_METERS : 0;
            for (Translation2d hub : List.of(FieldConstants.HUB_CENTER_RED, FieldConstants.HUB_CENTER_BLUE)) {
                double ahead = hub.getX() - redFrameRobot.getX();
                if (ahead > 0 && ahead < AutoAimConstants.FEED_HUB_BLOCK_DEPTH_METERS + grow
                        && Math.abs(redFrameRobot.getY() - hub.getY()) < AutoAimConstants.FEED_HUB_BLOCK_HALF_WIDTH_METERS + grow) {
                    return true;
                }
            }
            return false;
        }
}
