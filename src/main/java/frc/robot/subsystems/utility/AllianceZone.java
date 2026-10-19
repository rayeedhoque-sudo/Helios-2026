package frc.robot.subsystems.utility;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.AprilTagConstants;
import frc.robot.Constants.SubsystemConstants.LauncherConstants;
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.utility.LimelightHelpers.RawFiducial;

/**
 * TELEOP ZONE IDLE (team 2026-10-17): says when the flywheels idle at IDLE_FLYWHEEL_RPM.
 * ON when the Limelight sees one of OUR alliance-zone tags (red 7-12, blue 23-28: the hub's
 * zone-facing tags + the trench tags facing our zone; no other tag counts). Stays on for
 * IDLE_HOLD_SEC after the last sighting -- every sighting restarts it. After that, it stays on
 * only while the robot is within IDLE_ODOMETRY_HOLD_MAX_MOVE_METERS of where it last saw a zone
 * tag (turned in place, didn't drive away). Always off outside teleop or with no alliance.
 */
public class AllianceZone extends SubsystemBase {

    // Red 7/12 = trench tags facing the red zone, 8/11 = hub side tags at the zone end, 9/10 = hub
    // front face. Blue 23-28 = the same tags mirrored. Pinned in AllianceZoneTest.
    static final Set<Integer> RED_ZONE_TAGS = Set.of(7, 8, 9, 10, 11, 12);
    static final Set<Integer> BLUE_ZONE_TAGS = Set.of(23, 24, 25, 26, 27, 28);
    private static final double MAX_FRAME_AGE_SEC = 0.25; // same as TagAim / FeedAim

    private final CommandSwerveDrivetrain drivetrain;
    private double lastSeenSec = Double.NEGATIVE_INFINITY;  // last zone-tag sighting (FPGA s)
    private Translation2d lastSeenAt = Translation2d.kZero; // odometry position at that sighting
    private boolean idle = false;

    private final GenericEntry idleEntry =
        Shuffleboard.getTab("Shooter Subsystem Tab").add("Zone Idle Spin", false).getEntry();

    public AllianceZone(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    /** True while the flywheels should idle (teleop, a zone tag seen recently or not moved since). */
    public boolean shouldIdleSpin(){ return idle; }

    @Override
    public void periodic(){
        double now = Timer.getFPGATimestamp();
        Translation2d position = drivetrain.getState().Pose.getTranslation();
        boolean teleop = DriverStation.isTeleopEnabled();
        if (!teleop) {
            lastSeenSec = Double.NEGATIVE_INFINITY; // no sighting carries into the next teleop
        }
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // Fresh frames only: a dead camera's last tags must not keep restarting the timer.
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Vision.CAM_LIMELIGHT);
        boolean fresh = estimate != null && now - estimate.timestampSeconds <= MAX_FRAME_AGE_SEC;
        if (teleop && fresh && alliance.isPresent() && seesZoneTag(AprilTagConstants.remap(estimate.rawFiducials), alliance.get())) {
            lastSeenSec = now;
            lastSeenAt = position;
        }
        idle = idleOn(teleop, now - lastSeenSec, position.getDistance(lastSeenAt));
        idleEntry.setBoolean(idle);
    }

    //Pure helpers (package-private for AllianceZoneTest)
        /** One of our alliance's zone tags in this frame. */
        static boolean seesZoneTag(RawFiducial[] fiducials, Alliance alliance){
            if (fiducials == null) {
                return false;
            }
            Set<Integer> zoneTags = alliance == Alliance.Red ? RED_ZONE_TAGS : BLUE_ZONE_TAGS;
            for (RawFiducial f : fiducials) {
                if (zoneTags.contains(f.id)) {
                    return true;
                }
            }
            return false;
        }

        /**
         * Teleop, and either a zone tag within IDLE_HOLD_SEC or (odometry) not moved more than
         * IDLE_ODOMETRY_HOLD_MAX_MOVE_METERS since the last one. Never seen = infinite seconds.
         */
        static boolean idleOn(boolean teleop, double secSinceSeen, double movedSinceSeenMeters){
            return teleop && (secSinceSeen <= LauncherConstants.IDLE_HOLD_SEC
                || (secSinceSeen != Double.POSITIVE_INFINITY
                    && movedSinceSeenMeters <= LauncherConstants.IDLE_ODOMETRY_HOLD_MAX_MOVE_METERS));
        }
}
