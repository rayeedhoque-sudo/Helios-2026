package frc.robot.Constants;

import java.util.Set;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// 2026 REBUILT field geometry + AprilTag shooting partition.
// Every number here is dumped/derived from the official WPILib 2026-rebuilt-welded.json tag
// layout (apriltag-java-2026.2.1.jar) via the shot-math derivation of 2026-07-16 -- cited per
// constant. Tunables (team strategy choices / on-robot unknowns) are non-final per team convention.
public class FieldConstants {

    //FIELD DIMENSIONS (WPILib 2026-rebuilt-welded.json; blue origin at x = 0)
        public static final double FIELD_LENGTH_METERS = 16.541;
        public static final double FIELD_WIDTH_METERS = 8.069;

    //AUTO START POSE (blue-origin), used by the odometry autos' start/end pose reset.
        // The robot starts these autos in the RIGHT TRENCH. The exact field coordinates have
        // NOT been measured -- this is a deliberate do-nothing placeholder, not a value.
        // TODO MEASURE ON THE FIELD: blue-alliance right-trench start pose (x m, y m, heading).
        // WHILE THIS IS ZERO the reset writes (0, 0, 0 deg) into odometry, which makes every
        // pose-based feature wrong (auto-aim bearing, G407 zone legality). Do not run these
        // autos in a match until it is filled in.
        // Reference for whoever fills it in: the BLUE trench tags sit at x = 4.588/4.663 m,
        // y = 0.644 m (driver's right) and y = 7.425 m (left) in 2026-rebuilt-welded.json.
        public static Pose2d AUTO_START_POSE_BLUE = Pose2d.kZero;

        /**
         * Auto start pose for the current alliance: the blue value, mirrored by PathPlanner's
         * FlippingUtil on red (same flip AutoBuilder uses for paths, so both stay consistent).
         * TODO verify FlippingUtil.symmetryType matches the 2026 field once the pose is real.
         */
        public static Pose2d autoStartPose() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? FlippingUtil.flipFieldPose(AUTO_START_POSE_BLUE)
                : AUTO_START_POSE_BLUE;
        }

    //HUB GEOMETRY
        // Geometric hub centers = midpoints of the 4 tag face planes (blue -x/+x faces at
        // x = 4.0219/5.2292 m, red at 11.3119/12.5192 m, both +-y faces at y = 3.4312/4.6380 m).
        // NOT the mean of the 8 tag positions -- the tags sit pinwheel-asymmetric on the faces,
        // which biases that mean 0.0885 m toward the neutral zone (shot-math spec 1.1).
        public static final Translation2d HUB_CENTER_BLUE = new Translation2d(4.6255, 4.0346);
        public static final Translation2d HUB_CENTER_RED = new Translation2d(11.9155, 4.0346);
        // Hub fuel-opening front-edge height above carpet: 72 in (official 2026 Game Manual
        // sec.5.4; hexagonal opening 41.7 in across, hub footprint 47x47 in).
        public static final double HUB_OPENING_HEIGHT_METERS = 1.829;

    //ALLIANCE ZONES (zone depth 158.6 in = 4.028 m; rule G407: launching into the hub is only
    // legal with bumpers at least partially inside the own alliance zone)
        public static final double BLUE_ZONE_MAX_X_METERS = 4.028;
        public static final double RED_ZONE_MIN_X_METERS = 12.513;   // 16.541 - 4.028
        // Grace beyond the zone line for the botpose legality gate -- covers "bumpers partially
        // in" plus pose noise. [assumed] TODO tune on robot.
        public static double ZONE_LEGALITY_GRACE_METERS = 0.3;

    //FEED AIM POINTS (precision RT lob target, per alliance). The literal alliance-half center
    // sits INSIDE the hub footprint and the zone center is directly behind the hub, so both are
    // rejected (shot-math spec 3.1): aim mid-zone in x, 2 m laterally off the hub line so no lob
    // crosses the hub structure. y-side is a strategy choice. [assumed, tunable] TODO pick the
    // y-side the hopper-emptying robot works.
        public static Translation2d FEED_TARGET_BLUE = new Translation2d(2.0, 2.0);
        public static Translation2d FEED_TARGET_RED = new Translation2d(14.541, 2.0);   // x mirror

    //TAG PARTITION (team spec 2026-07-16, implement verbatim). FEED tags are the hub faces +
    // trench tags visible from midfield, where scoring is illegal per G407; SCORE tags are the
    // remaining own-hub faces, only visible from shootable ground. Any other tag (7, 12, 23, 28,
    // 13-16, 29-32) and "no tag" = NO shooting operations at all.
        public static final Set<Integer> FEED_TAGS = Set.of(1, 3, 4, 6, 17, 19, 20, 22);
        public static final Set<Integer> SCORE_TAGS_RED = Set.of(2, 5, 8, 9, 10, 11);
        public static final Set<Integer> SCORE_TAGS_BLUE = Set.of(18, 21, 24, 25, 26, 27);

    //CAMERA-SPACE FALLBACK GEOMETRY (SCORE distance without botpose: tag translation -> hub
    // center). Every hub tag plane is 0.6035 m in front of the hub center along its inward
    // normal; on each face one tag is centered and one is offset 0.3556 m to a side
    // (shot-math spec 1.2, derived from the tag layout json).
        public static final double TAG_FACE_TO_HUB_DEPTH_METERS = 0.6035;
        public static final double TAG_LATERAL_OFFSET_METERS = 0.3556;
        // The face-offset tags (the centered ones -- 2, 4, 5, 10, 18, 20, 21, 26 -- have 0 offset).
        public static final Set<Integer> LATERALLY_OFFSET_TAGS = Set.of(3, 8, 9, 11, 19, 24, 25, 27);
        // THE SIGN IS PER TAG, NOT GLOBAL (2026-08-29). It used to be one unverified
        // TAG_LATERAL_OFFSET_SIGN applied to every offset tag, defaulted to 0.0 to disable the
        // term. That could never have been right: the tags sit PINWHEEL-asymmetric on the hub
        // (the same asymmetry noted at HUB_CENTER_BLUE), so the hub centre is to the LEFT of
        // some offset tags and to the RIGHT of others. One global sign is wrong for half of them.
        //
        // These are MEASURED, not assumed: for every tag the 2026-rebuilt-welded.json layout was
        // read, the hub centre vector was rotated into the tag's own frame, and the component
        // across the face recorded. Every offset came out +-0.3556 m and every depth -0.6034 m,
        // matching TAG_LATERAL_OFFSET_METERS and TAG_FACE_TO_HUB_DEPTH_METERS exactly -- which is
        // the cross-check that the frame convention here is right.
        //
        // SIGN CONVENTION: positive = the hub centre lies to the RIGHT in the camera image of a
        // robot facing that tag (camera +x is right), i.e. THE TAG IS ON THE LEFT of the hub
        // face. That is the team's 2026-08-29 observation ("the AprilTags are on the left side
        // of the front face"), and the layout agrees for tags 9, 25 -- while 10 and 26 are dead
        // centre (0.0000 m), so on those faces there is nothing to correct.
        public static final java.util.Map<Integer, Double> TAG_LATERAL_OFFSET_SIGN_BY_TAG =
            java.util.Map.of(
                3, +1.0,  9, +1.0, 11, +1.0,   // red hub: hub centre to the camera's right
                8, -1.0,                        // red hub: the mirrored one -- hub centre LEFT
                19, +1.0, 25, +1.0, 27, +1.0,   // blue hub
                24, -1.0);                      // blue hub: the mirrored one

    //DRIVETRAIN-FACING TUNABLES (owned here so the shooter/drivetrain agents share one source)
        // A / DPAD-UP search spin rate. Slow enough that detect->stop overshoot (~4-7 deg at
        // 50-100 ms latency) is absorbed by the face-the-tag servo. [assumed, tunable 0.1-0.3]
        // TODO tune on carpet.
        public static double SEARCH_ROTATE_RATE_ROT_PER_SEC = 0.2;
        // Heading tolerance for tag alignment: worst lateral miss 0.214 m at max legal range vs
        // the 0.455 m usable half-width of the hub opening (shot-math spec 6).
        public static double HEADING_TOLERANCE_DEG = 2.0;

    //HELPERS (pure lookups -- no I/O, no state)
        // Own hub center for scoring shots.
        public static Translation2d ownHubCenter(Alliance alliance) {
            return alliance == Alliance.Red ? HUB_CENTER_RED : HUB_CENTER_BLUE;
        }

        // Own feed aim point for precision lobs.
        public static Translation2d ownFeedTarget(Alliance alliance) {
            return alliance == Alliance.Red ? FEED_TARGET_RED : FEED_TARGET_BLUE;
        }

        // The SCORE tag set for the given alliance (never shoot into the opponent hub).
        public static Set<Integer> ownScoreTags(Alliance alliance) {
            return alliance == Alliance.Red ? SCORE_TAGS_RED : SCORE_TAGS_BLUE;
        }

        // G407 legality gate for botpose-based scoring: robot x inside the own alliance zone
        // plus grace. Closes the hole where a robot just outside its zone beside the hub still
        // sees a +-y-face SCORE tag (shot-math spec 1.3).
        public static boolean isLegalScoringX(double robotXMeters, Alliance alliance) {
            return alliance == Alliance.Red
                ? robotXMeters >= RED_ZONE_MIN_X_METERS - ZONE_LEGALITY_GRACE_METERS
                : robotXMeters <= BLUE_ZONE_MAX_X_METERS + ZONE_LEGALITY_GRACE_METERS;
        }

        // Signed lateral offset (m) to ADD TO CAMERA X to go from the tag to its hub centre,
        // for a robot facing that tag. 0 for a centred tag and for anything not on a hub.
        // See TAG_LATERAL_OFFSET_SIGN_BY_TAG for the sign convention and where it came from.
        public static double tagLateralOffsetMeters(int tagId) {
            return TAG_LATERAL_OFFSET_SIGN_BY_TAG.getOrDefault(tagId, 0.0)
                * TAG_LATERAL_OFFSET_METERS;
        }
}
