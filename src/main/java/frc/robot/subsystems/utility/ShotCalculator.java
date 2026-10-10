package frc.robot.subsystems.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.AutoAimConstants;
import frc.robot.commands.CommandSwerveDrivetrain;

/**
 * RB SHOOT-ON-THE-MOVE math (redesigned 2026-10-09 after LUNATECS / Ninjas 4744 / 1833).
 * Every loop: robot pose + field velocity -> "virtual target" = hub centre minus
 * velocity x time of flight (iterated, since flight time depends on distance) -> the heading
 * to face, its rate (rotation feedforward), and flywheel rpm + hood angle from the distance
 * tables in AutoAimConstants.
 *
 * Aims from the FIELD POSE (odometry + MegaTag2), not the tag in view: the Limelight mount
 * pose must be set in its web UI or every number here is off. No motors and no commands --
 * it's a subsystem only so periodic() runs before the RB commands read getShot(). It runs
 * all the time, so "Shot Distance" is live on the dashboard for building the shot table.
 */
public class ShotCalculator extends SubsystemBase {

    /** One loop's shot solution. Heading is blue-origin; the ball leaves along robot +X. */
    public record Shot(Translation2d virtualTarget, double distanceMeters, Rotation2d heading,
            double headingRateRadPerSec, double flywheelRpm, double hoodDeg) {}

    private final CommandSwerveDrivetrain drivetrain;
    private Shot shot = new Shot(Translation2d.kZero, 0, Rotation2d.kZero, 0, 0, 0);
    private boolean aimed = false;
    private boolean active = false; // RB group running
    private boolean allianceWarned = false;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter Subsystem Tab");
    private final GenericEntry aimedEntry = tab.add("Aimed At Target", false).getEntry();
    private final GenericEntry distanceEntry = tab.add("Shot Distance (m)", 0.0).getEntry();
    private final GenericEntry headingErrorEntry = tab.add("Shot Heading Error (deg)", 0.0).getEntry();
    private final GenericEntry tableRpmEntry = tab.add("Shot Table RPM", 0.0).getEntry();
    private final GenericEntry tableHoodEntry = tab.add("Shot Table Hood (deg)", 0.0).getEntry();

    public ShotCalculator(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }

    /** This loop's shot (computed in periodic, before any command runs). */
    public Shot getShot(){ return shot; }

    /** Robot heading within HEADING_TOLERANCE_DEG of the shot heading. */
    public boolean isAimed(){ return aimed; }

    /** RB shoot-on-the-move running: the default drive aims, and the dashboard shows "Aimed". */
    public boolean isActive(){ return active; }
    public void setActive(boolean active){ this.active = active; }

    @Override
    public void periodic(){
        var state = drivetrain.getState();
        shot = calculate(state.Pose, state.Speeds,
            hubToAimAt(practiceTestTag(), ownAlliance()));
        double errorDeg = shot.heading().minus(state.Pose.getRotation()).getDegrees();
        aimed = Math.abs(errorDeg) <= FieldConstants.HEADING_TOLERANCE_DEG;

        // Only while RB runs, so the dashboard never shows a stale true (team, commit 5a1a852).
        aimedEntry.setBoolean(active && aimed);
        distanceEntry.setDouble(shot.distanceMeters());
        headingErrorEntry.setDouble(errorDeg);
        tableRpmEntry.setDouble(shot.flywheelRpm());
        tableHoodEntry.setDouble(shot.hoodDeg());
    }

    /** Test-tag practice mode: switch on and no FMS attached (never in a real match). */
    public static boolean practiceTestTag(){
        return AutoAimConstants.PRACTICE_TEST_TAG && !DriverStation.isFMSAttached();
    }

    // Own alliance; DriverStation may not know yet -- default BLUE and warn once.
    private Alliance ownAlliance(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            if (!allianceWarned) {
                DriverStation.reportWarning("ShotCalculator: alliance unknown -- aiming at the BLUE hub", false);
                allianceWarned = true;
            }
            return Alliance.Blue;
        }
        return alliance.get();
    }

    //Pure helpers (no I/O, package-private for ShotCalculatorTest)
        /** Own hub -- or the test tag's hub while practising off-FMS. */
        static Translation2d hubToAimAt(boolean practiceTestTag, Alliance alliance){
            return practiceTestTag ? AutoAimConstants.TEST_TAG_HUB
                : FieldConstants.ownHubCenter(alliance);
        }

        /** The full solution for one pose / robot-relative speed / hub. */
        static Shot calculate(Pose2d pose, ChassisSpeeds robotRelative, Translation2d hub){
            Translation2d launcher = pose.getTranslation()
                .plus(AutoAimConstants.ROBOT_TO_LAUNCHER_METERS.rotateBy(pose.getRotation()));
            Translation2d velocity = fieldVelocity(robotRelative, pose.getRotation());
            Translation2d target = virtualTarget(hub, launcher, velocity);
            Translation2d toTarget = target.minus(launcher);
            double distance = toTarget.getNorm();
            return new Shot(target, distance, toTarget.getAngle(),
                AutoAimConstants.HEADING_RATE_FF_SCALE * headingRate(toTarget, velocity),
                AutoAimConstants.SHOT_RPM_BY_DISTANCE.get(distance),
                AutoAimConstants.SHOT_HOOD_DEG_BY_DISTANCE.get(distance));
        }

        /** Robot-relative chassis speeds -> field (blue-origin) velocity, m/s. */
        static Translation2d fieldVelocity(ChassisSpeeds robotRelative, Rotation2d heading){
            return new Translation2d(robotRelative.vxMetersPerSecond, robotRelative.vyMetersPerSecond)
                .rotateBy(heading);
        }

        /**
         * Where to aim so the ball, which keeps the robot's velocity, lands in the hub:
         * hub - velocity x flightTime(distance to the aim point), refined a few times.
         */
        static Translation2d virtualTarget(Translation2d hub, Translation2d launcher, Translation2d velocity){
            Translation2d target = hub;
            for (int i = 0; i < AutoAimConstants.VIRTUAL_TARGET_ITERATIONS; i++) {
                double tof = AutoAimConstants.TIME_OF_FLIGHT_SEC_BY_DISTANCE.get(target.getDistance(launcher));
                target = hub.minus(velocity.times(tof));
            }
            return target;
        }

        /**
         * How fast the bearing to a fixed point changes as the robot translates (rad/s, CCW +):
         * d/dt atan2(r) with dr/dt = -v. Fed forward so the heading loop isn't always a step behind.
         */
        static double headingRate(Translation2d toTarget, Translation2d velocity){
            double r2 = toTarget.getX() * toTarget.getX() + toTarget.getY() * toTarget.getY();
            if (r2 < 1e-6) {
                return 0;
            }
            return MathUtil.clamp(
                (toTarget.getY() * velocity.getX() - toTarget.getX() * velocity.getY()) / r2,
                -AutoAimConstants.MAX_ROTATION_RAD_PER_SEC, AutoAimConstants.MAX_ROTATION_RAD_PER_SEC);
        }
}
