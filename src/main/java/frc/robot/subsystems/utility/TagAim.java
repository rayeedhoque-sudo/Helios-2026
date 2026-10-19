package frc.robot.subsystems.utility;

import java.util.Map;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.utility.LimelightHelpers.RawFiducial;

/**
 * RB TAG AIM (team 2026-10-09; shoot-on-the-move is switched off): while RB is held the robot
 * stands still and turns until the Limelight crosshair's vertical line is on the HUB CENTRE
 * behind the middle front tag -- 10 or 26 (seen tags go through AprilTagConstants stand-ins); only when none of
 * those is in view, behind any other tag of the hub (own hub in a match), each with its own offset
 * to the hub centre (HubTag). Straight on a middle tag that
 * is the tag itself; from the side the hub centre (0.6035 m behind the face) is past the tag, so
 * the aim moves past it. Which way the face points comes from the Pigeon heading + the tag's
 * known field direction, so MENU must be zeroed right. Aim only: RT still shoots.
 *
 * Each new camera frame: target heading = the robot's heading WHEN THAT FRAME WAS TAKEN minus
 * the hub centre's angle right of the crosshair. Using the frame-time heading means camera latency
 * can't make the turn overshoot. The target is a field heading, so it stays valid between
 * frames and if the tag drops out (the robot isn't moving). Needs no field POSITION (only the heading).
 *
 * The crosshair is the Limelight's calibrated crosshair, so the team can shift the aim point
 * in the Limelight web UI (crosshair calibration) without touching code.
 */
public class TagAim extends SubsystemBase {

    private static final Set<Integer> FRONT_HUB_TAGS = Set.of(10, 26);

    /**
     * Every hub tag (2026-rebuilt-welded.json): the field heading pointing INTO the hub through its
     * face (tag yaw + 180), and how far the hub centre sits LEFT of the tag looking into the hub
     * (m; 0 for a face's middle tag, +-0.3556 for the tag beside it). All sit TAG_FACE_TO_HUB_DEPTH
     * in front of the hub centre. 10/26 are the front-middle tags: same offset (0) as before.
     */
    record HubTag(double inwardDeg, double hubLeftMeters) {}
    static final Map<Integer, HubTag> RED_HUB_TAGS = Map.of(
        2, new HubTag(270, 0), 3, new HubTag(0, -0.3556), 4, new HubTag(0, 0), 5, new HubTag(90, 0),
        8, new HubTag(90, 0.3555), 9, new HubTag(180, -0.3556), 10, new HubTag(180, 0), 11, new HubTag(270, -0.3555));
    static final Map<Integer, HubTag> BLUE_HUB_TAGS = Map.of(
        18, new HubTag(90, 0), 19, new HubTag(180, -0.3556), 20, new HubTag(180, 0), 21, new HubTag(270, 0),
        24, new HubTag(270, 0.3555), 25, new HubTag(0, -0.3556), 26, new HubTag(0, 0), 27, new HubTag(90, -0.3555));
    private static final double MAX_FRAME_AGE_SEC = 0.25; // a live LL frame is ~30-60 ms old

    private final CommandSwerveDrivetrain drivetrain;
    private boolean active = false;   // RB held
    private boolean hasTarget = false; // a front tag seen since RB was pressed
    private Rotation2d targetHeading = Rotation2d.kZero; // blue-origin
    private boolean aimed = false;
    private double crosshairOffsetDeg = 0; // crosshair x from the image centre (deg), last known
    private double lastFrameSec = -1;      // only NEW frames move the target
    private double hubDistanceMeters = 0;  // shot-table key, last seen tag
    // TAG_SHOOT_ON_THE_MOVE only: hub centre pinned in odometry by the last tag frame, the
    // table-key minus ground distance at that frame, and this loop's lead-corrected solution.
    private boolean hasAnchor = false;
    private Translation2d hubAnchor = Translation2d.kZero;
    private double keyOffsetMeters = 0;
    private MovingShot movingShot = new MovingShot(Rotation2d.kZero, 0, 0);

    /** One loop's shoot-on-the-move solution: blue-origin heading, rate feedforward, table key. */
    record MovingShot(Rotation2d heading, double headingRateRadPerSec, double hubDistanceMeters) {}

    private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter Subsystem Tab");
    private final GenericEntry aimedEntry = tab.add("Aimed At Target", false).getEntry();
    private final GenericEntry tagEntry = tab.add("Tag Aim Tag", -1.0).getEntry();
    private final GenericEntry errorEntry = tab.add("Tag Aim Error (deg, + = hub right)", 0.0).getEntry();
    // How far the hub centre is from the tag, seen from here: 0 straight on, grows off to the side.
    private final GenericEntry hubOffsetEntry = tab.add("Tag Aim Hub Offset (deg, + = right of tag)", 0.0).getEntry();
    private final GenericEntry hubDistanceEntry = tab.add("Tag Aim Hub Distance (m)", 0.0).getEntry();

    public TagAim(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    /** RB pressed / released. Every press starts fresh (no remembered target). */
    public void setActive(boolean active){
        this.active = active;
        hasTarget = false;
        hasAnchor = false;
    }

    public boolean isActive(){ return active; }

    /** Heading in tolerance of a seen tag. */
    public boolean isAimed(){ return aimed; }

    /** Shot-table distance from the last seen tag (see hubDistanceMeters). 0 = none seen yet. */
    public double getHubDistanceMeters(){
        if (AutoAimConstants.TAG_SHOOT_ON_THE_MOVE && hasAnchor) {
            return movingShot.hubDistanceMeters();
        }
        return hubDistanceMeters;
    }

    /** Blue-origin heading to face: the tag target, or the current heading (hold) until one is seen. */
    public Rotation2d getTargetHeading(){
        if (AutoAimConstants.TAG_SHOOT_ON_THE_MOVE && hasAnchor) {
            return movingShot.heading();
        }
        if (!AutoAimConstants.TAG_SHOOT_ON_THE_MOVE && hasTarget) {
            return targetHeading;
        }
        // No tag yet this press: turn toward the hub by odometry until one comes into view
        // (turn only -- isAimed() still needs a tag, so the belts never feed on odometry).
        // A tag seen then lost keeps its target (above), moving or not.
        Pose2d pose = drivetrain.getState().Pose;
        return odometryHeading(pose.getTranslation(), DriverStation.getAlliance()).orElse(pose.getRotation());
    }

    /** Rotation feedforward (rad/s) for the moving aim; 0 for the stand-still aim. */
    public double getHeadingRateFeedforward(){
        return AutoAimConstants.TAG_SHOOT_ON_THE_MOVE && hasAnchor ? movingShot.headingRateRadPerSec() : 0;
    }

    @Override
    public void periodic(){
        if (!active) {
            aimed = false;
            aimedEntry.setBoolean(false);
            tagEntry.setDouble(-1);
            return;
        }
        String cam = Vision.CAM_LIMELIGHT;
        if (LimelightHelpers.getTV(cam)) {
            // tx is from the crosshair, txnc from the image centre: the difference is the crosshair.
            crosshairOffsetDeg = LimelightHelpers.getTXNC(cam) - LimelightHelpers.getTX(cam);
        }
        // MegaTag2's estimate carries every visible tag's txnc AND the frame's timestamp.
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
        // A frame is used ONCE: a dead camera leaves its last frame in NT, and re-applying that
        // error to the moving heading every loop would spin the robot.
        // Older than MAX_FRAME_AGE_SEC = stale too (e.g. the first press after the camera died).
        boolean newFrame = estimate != null && estimate.timestampSeconds != lastFrameSec
            && Timer.getFPGATimestamp() - estimate.timestampSeconds <= MAX_FRAME_AGE_SEC;
        RawFiducial tag = newFrame ? pickTag(AprilTagConstants.remap(estimate.rawFiducials),
            fallbackTags(DriverStation.getAlliance())) : null;
        Optional<Pose2d> poseAtFrame = tag != null ? drivetrain.samplePoseAt(estimate.timestampSeconds) : Optional.empty();
        if (newFrame) {
            lastFrameSec = estimate.timestampSeconds;
        }
        if (poseAtFrame.isPresent()) {
            Rotation2d headingAtFrame = poseAtFrame.get().getRotation();
            Rotation2d inwardRobot = inwardHeading(tag.id).minus(headingAtFrame);
            double hubLeft = hubTag(tag.id).hubLeftMeters();
            double errorDeg = errorToHubDeg(tag.txnc, tag.tync, tag.distToCamera,
                inwardRobot, hubLeft, crosshairOffsetDeg, AutoAimConstants.TAG_AIM_TX_SIGN)
                - AutoAimConstants.TAG_AIM_LEFT_TRIM_DEG; // trim: aim this much further left (CCW)
            if (tag.distToCamera > 0.01) {
                hubDistanceMeters = hubDistanceMeters(tag.txnc, tag.distToCamera, inwardRobot, hubLeft,
                    AutoAimConstants.TAG_AIM_TX_SIGN);
                hubDistanceEntry.setDouble(hubDistanceMeters);
            }
            if (AutoAimConstants.TAG_SHOOT_ON_THE_MOVE && tag.distToCamera > 0.01) {
                Translation2d hubRobot = hubInRobot(tag.txnc, tag.tync, tag.distToCamera, inwardRobot, hubLeft,
                    AutoAimConstants.TAG_AIM_TX_SIGN);
                hubAnchor = poseAtFrame.get().getTranslation().plus(hubRobot.rotateBy(headingAtFrame));
                keyOffsetMeters = hubDistanceMeters - hubRobot.getNorm();
                hasAnchor = true;
            }
            targetHeading = targetHeading(headingAtFrame, errorDeg);
            hasTarget = true;
            tagEntry.setDouble(tag.id);
            errorEntry.setDouble(errorDeg);
            hubOffsetEntry.setDouble(errorDeg + AutoAimConstants.TAG_AIM_LEFT_TRIM_DEG
                - AutoAimConstants.TAG_AIM_TX_SIGN * (tag.txnc - crosshairOffsetDeg));
        } else if (newFrame) {
            tagEntry.setDouble(-1);
        }
        Rotation2d heading = drivetrain.getState().Pose.getRotation();
        if (AutoAimConstants.TAG_SHOOT_ON_THE_MOVE) {
            // Every loop, not just on frames: between frames odometry carries the aim.
            var state = drivetrain.getState();
            if (hasAnchor) {
                movingShot = movingShot(hubAnchor, keyOffsetMeters, state.Pose, state.Speeds,
                    aimOffsetDeg(crosshairOffsetDeg));
            }
            aimed = hasAnchor && Math.abs(movingShot.heading().minus(heading).getDegrees())
                <= FieldConstants.HEADING_TOLERANCE_DEG;
            aimedEntry.setBoolean(aimed);
            return;
        }
        aimed = hasTarget && Math.abs(targetHeading.minus(heading).getDegrees())
            <= FieldConstants.HEADING_TOLERANCE_DEG;
        aimedEntry.setBoolean(aimed);
    }

    //Pure helpers (package-private for TagAimTest)
        /**
         * The biggest (closest) front hub tag in view. Only if none is in view: the biggest of
         * fallbackTags (the other hub tags). Else null.
         */
        static RawFiducial pickTag(RawFiducial[] fiducials, Set<Integer> fallbackTags){
            RawFiducial best = null;
            RawFiducial bestFallback = null;
            if (fiducials == null) {
                return null;
            }
            for (RawFiducial f : fiducials) {
                boolean front = FRONT_HUB_TAGS.contains(f.id);
                if (front && (best == null || f.ta > best.ta)) {
                    best = f;
                } else if (!front && fallbackTags.contains(f.id) && (bestFallback == null || f.ta > bestFallback.ta)) {
                    bestFallback = f;
                }
            }
            return best != null ? best : bestFallback;
        }

        /**
         * Side tags RB may fall back to: only OWN hub's tags (never aim at the other hub);
         * alliance unknown = none.
         */
        static Set<Integer> fallbackTags(Optional<Alliance> alliance){
            return alliance.map(a -> a == Alliance.Red ? RED_HUB_TAGS.keySet() : BLUE_HUB_TAGS.keySet())
                .orElse(Set.of());
        }

        /** This tag's hub geometry; a non-hub id (never picked) = a middle tag facing 0 deg. */
        static HubTag hubTag(int tagId){
            HubTag t = RED_HUB_TAGS.get(tagId);
            return t != null ? t : BLUE_HUB_TAGS.getOrDefault(tagId, new HubTag(0, 0));
        }

        /**
         * Field heading pointing INTO the hub through this tag's face (= the heading of a robot
         * facing it square-on): 26 -> 0 deg, 10 -> 180 deg (tag yaw + 180, 2026-rebuilt-welded.json).
         */
        static Rotation2d inwardHeading(int tagId){
            return Rotation2d.fromDegrees(hubTag(tagId).inwardDeg());
        }

        /** Tag -> hub centre, robot frame: TAG_FACE_TO_HUB_DEPTH along inwardRobot, hubLeft to its left. */
        static Translation2d tagToHub(Rotation2d inwardRobot, double hubLeftMeters){
            return new Translation2d(FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS, hubLeftMeters).rotateBy(inwardRobot);
        }

        /**
         * Degrees the HUB CENTRE sits right of the crosshair (+ = right). The tag's ground point
         * comes from the camera (txnc / tync / distToCamera); the hub centre is
         * TAG_FACE_TO_HUB_DEPTH behind it along inwardRobot (the into-the-hub direction, robot
         * frame, CCW+) and hubLeftMeters to the left of that (0 for 10/26). Straight on a middle
         * tag, that's the tag itself; from the side it's past the tag.
         * Falls back to the tag when the camera gives no range.
         */
        static double errorToHubDeg(double txncDeg, double tyncDeg, double distToCamera,
                Rotation2d inwardRobot, double hubLeftMeters, double crosshairOffsetDeg, double txSign){
            double tagRightDeg = txSign * txncDeg;
            boolean ranged = distToCamera > 0.01;
            double ground = distToCamera * Math.cos(Math.toRadians(tyncDeg)); // drop the height
            // Robot frame: x forward, y left. A tag to the right is -y.
            Translation2d tag = new Translation2d(ground, Rotation2d.fromDegrees(-tagRightDeg));
            Translation2d hub = tag.plus(tagToHub(inwardRobot, hubLeftMeters));
            double hubRightDeg = ranged ? -hub.getAngle().getDegrees() : tagRightDeg;
            return hubRightDeg - txSign * crosshairOffsetDeg;
        }

        /**
         * RB shot-table key (m): the tag's 3D camera range, then on to the hub centre (tagToHub).
         * Straight on a middle tag = range + 0.6035 -- exactly how the table rows were
         * keyed, so it must stay the 3D range (no height drop) or the table shifts.
         * ponytail: height left in, so from the side it reads slightly long; re-key if angled shots miss long.
         */
        static double hubDistanceMeters(double txncDeg, double distToCamera, Rotation2d inwardRobot,
                double hubLeftMeters, double txSign){
            Translation2d tag = new Translation2d(distToCamera, Rotation2d.fromDegrees(-txSign * txncDeg));
            return tag.plus(tagToHub(inwardRobot, hubLeftMeters)).getNorm();
        }

        /**
         * TAG_SHOOT_ON_THE_MOVE: the hub centre's ground point, robot frame (x fwd, y left), m --
         * the same tag + tagToHub vector errorToHubDeg aims at.
         * ponytail: camera taken as the robot centre, like the rest of TagAim; add the mount offset if moving shots miss sideways.
         */
        static Translation2d hubInRobot(double txncDeg, double tyncDeg, double distToCamera,
                Rotation2d inwardRobot, double hubLeftMeters, double txSign){
            double ground = distToCamera * Math.cos(Math.toRadians(tyncDeg));
            return new Translation2d(ground, Rotation2d.fromDegrees(-txSign * txncDeg))
                .plus(tagToHub(inwardRobot, hubLeftMeters));
        }

        /**
         * TAG_SHOOT_ON_THE_MOVE solution for one loop: aim at hub - velocity x flight time
         * (ShotCalculator's virtual target), turned aimOffsetDeg CCW (aimOffsetDeg(): the same
         * crosshair + trim the stand-still aim uses, so standing still both settle alike), and
         * key the tag tables on the distance to that point plus keyOffset (the tables' 3D-range
         * key minus the ground distance).
         */
        static MovingShot movingShot(Translation2d hubAnchor, double keyOffsetMeters, Pose2d pose,
                ChassisSpeeds robotRelative, double aimOffsetDeg){
            Translation2d velocity = ShotCalculator.fieldVelocity(robotRelative, pose.getRotation());
            Translation2d toTarget = ShotCalculator.virtualTarget(hubAnchor, pose.getTranslation(), velocity)
                .minus(pose.getTranslation());
            return new MovingShot(toTarget.getAngle().plus(Rotation2d.fromDegrees(aimOffsetDeg)),
                AutoAimConstants.HEADING_RATE_FF_SCALE * ShotCalculator.headingRate(toTarget, velocity),
                toTarget.getNorm() + keyOffsetMeters);
        }

        /**
         * Degrees CCW to turn a face-the-hub heading so the CROSSHAIR (not the image centre)
         * lands on the hub, plus the left trim -- what errorToHubDeg + the trim do for the
         * stand-still aim.
         */
        static double aimOffsetDeg(double crosshairOffsetDeg){
            return AutoAimConstants.TAG_AIM_TX_SIGN * crosshairOffsetDeg + AutoAimConstants.TAG_AIM_LEFT_TRIM_DEG;
        }

        /** Hub to the RIGHT of the crosshair (+) = turn clockwise = a smaller heading. */
        static Rotation2d targetHeading(Rotation2d headingAtFrame, double errorDeg){
            return headingAtFrame.minus(Rotation2d.fromDegrees(errorDeg));
        }

        /**
         * No-tag fallback: field heading from the robot to our own hub. Alliance unknown = empty
         * (hold, never guess a hub).
         */
        static Optional<Rotation2d> odometryHeading(Translation2d robot, Optional<Alliance> alliance){
            return alliance.map(a -> FieldConstants.ownHubCenter(a).minus(robot).getAngle());
        }
}
