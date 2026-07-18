package frc.robot.subsystems.misc;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.OptionalDouble;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;
import frc.robot.Constants.SubsystemConstants.Vision;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.subsystems.utility.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase{

    //Type
        public boolean enableComp;
   
    //Shooter Motor
        private final TalonFX shooterA = new TalonFX(ShooterSubsystemConstants.SHOOTER_ID_A);
          private final TalonFX shooterB = new TalonFX(ShooterSubsystemConstants.SHOOTER_ID_B);
        private final TalonFX shooterC = new TalonFX(ShooterSubsystemConstants.SHOOTER_ID_C);
          private final TalonFX shooterD = new TalonFX(ShooterSubsystemConstants.SHOOTER_ID_D);
        private final SparkMax shooterAngle = new SparkMax(ShooterSubsystemConstants.SHOOTER_ANGLE_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
            private final SparkAbsoluteEncoder shooterAngleEncoder = shooterAngle.getAbsoluteEncoder();
    
     //Shooter Speed - PID & FF
        private final VelocityVoltage m_velocity = new VelocityVoltage(0);
        // Coast request for "flywheels off": closing the velocity loop on 0 rps would actively
        // reverse-brake 4 spinning Krakens (regen current dump + gearbox stress) -- the Coast
        // neutral mode only applies when no closed-loop request is latched.
        private final CoastOut m_flywheelCoast = new CoastOut();
        private final Slot0Configs shooterVelConfigs = 
            new Slot0Configs().
            withKP(ShooterSubsystemConstants.SHOOTER_SPEED_kP). 
            withKD(ShooterSubsystemConstants.SHOOTER_SPEED_kD).
            withKV(ShooterSubsystemConstants.SHOOTER_SPEED_kV);
    //Shooter Angle - PID 
        private final PIDController shooterAnglePID = new PIDController(
            ShooterSubsystemConstants.SHOOTER_ANGLE_kP, 
            ShooterSubsystemConstants.SHOOTER_ANGLE_kI, 
            ShooterSubsystemConstants.SHOOTER_ANGLE_kD
        );
    //Drive Train
        private final CommandSwerveDrivetrain drivetrain;
        private double degreesToAlignToTarget;
    //Data
        private final ShuffleboardTab ShooterSubsystemTab = Shuffleboard.getTab("Shooter Subsystem Tab");
            private final GenericEntry currentVelEntry;
            private final GenericEntry desiredVelEntry;
            private final GenericEntry currentAngleEntry;
            private final GenericEntry desiredAngleEntry;
            private final GenericEntry targetDistanceEntry;
            private final GenericEntry targetClassEntry;
            private final GenericEntry shotSetpointEntry;
            private final GenericEntry readyToShootEntry;
            private final GenericEntry aimedEntry;
            private final GenericEntry movingCompEntry;
            private final GenericEntry subsystemStateEntry;
            private final GenericEntry degreedToAlignToTargEntry;
            private final GenericEntry shooterSpeed_kP;
            private final GenericEntry shooterSpeed_kD;
            private final GenericEntry shooterSpeed_kV;
            private final GenericEntry shooterAngle_kP;
            private final GenericEntry shooterAngle_kI;
            private final GenericEntry shooterAngle_kD;
            private final GenericEntry desiredVelReachedEntry;
            private final GenericEntry desiredAngleReachedEntry;
            private final GenericEntry debugEntry;

    //Tracker Variables
       private boolean enableSubsystem;
       private double desired_Velocity;
       private double desired_Angle;
       private double target_distance;

    //Targeting State (team spec 2026-07-16). Tag classification refreshes every loop in
    // periodic() (cheap NT reads); the distance/velocity model runs only while
    // visionShotCommand() is scheduled. velReached/angleReached REPLACE the old mutable
    // statics in ShooterSubsystemConstants (hidden cross-subsystem coupling) -- the hopper
    // now gates its kicker through the isReadyToShoot() supplier instead.
       private enum TargetClass { SCORE, FEED, NONE }
       private TargetClass targetClass = TargetClass.NONE;
       private int visibleTagId = -1;
       private Pose3d tagCameraPose = new Pose3d();
       private boolean hasShotTarget = false;
       private boolean velReached = false;
       private boolean angleReached = false;
       private double movingCompMeters = 0.0;
       private boolean allianceWarned = false;
       // RT auto-aim (team request 2026-07-17): the blue-origin field heading the robot must
       // face to take the current shot. Valid only while runVisionTargeting has a target;
       // consumed by the drivetrain's aim-lock drive layer and the isAimedAtTarget() gate.
       private double aimHeadingDeg = 0.0;
       private boolean aimHeadingValid = false;
       // Tag-flicker ride-through: remember the last non-NONE classification so a momentary
       // dropout (< TARGET_HOLD_SEC) rides on the drivetrain's fused pose instead of
       // restarting the flywheels. Initial acquisition still requires a real tag.
       private TargetClass heldClass = TargetClass.NONE;
       private final Timer targetHoldTimer = new Timer();

       // Vision targeting now lives entirely inside visionShotCommand() -- the old enableVision
       // constructor flag is gone. INTEGRATOR NOTE: construct with just the drivetrain and
       // delete the RobotContainer boot-time disableSubsystem() call; the shot commands enable
       // the subsystem themselves (shooter re-enabled by team directive 2026-07-16).
       public ShooterSubsystem(CommandSwerveDrivetrain drivetrain){

            //Coniguring Motors
                shooterA.getConfigurator().apply(shooterVelConfigs);

                // Current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
                // Flywheels: Kraken X60 - stator 80A / supply 25A, Coast (let them spin down freely).
                // STATOR raised 40 -> 80 A (Regular tier) 2026-07-17: the feed/precision shots "barely
                // left the robot" -- the 40 A cap choked the torque IMPULSE when a ball hits, so the
                // wheel sagged deep on contact instead of driving the ball. 80 A doubles the anti-bog
                // torque; that spike is brief (ball contact ~tens of ms) and sourced mostly from the
                // flywheel's inertia + DC link, NOT sustained battery draw. Kraken X60 stalls at 366 A
                // so 80 A intermittent is well within thermal margin (watch DeviceTemp in PowerTelemetry).
                // SUPPLY deliberately HELD at 25 A: supply is the battery-draw / BROWNOUT knob and the
                // team browns out late-match -- keeping it at 25 A means this change adds ~0 to the
                // spin-up draw (still 25 A x4 = 100 A). If rapid-fire recovery between balls is still
                // weak on retest, raise supply toward 30 A (Regular tier) -- but only on a healthy
                // battery, and expect ~+20 A worst-case drivetrain-shot overlap. Cap is 120/40, never exceed.
                CurrentLimitsConfigs flywheelLimits = new CurrentLimitsConfigs();
                flywheelLimits.StatorCurrentLimit = 80;
                flywheelLimits.StatorCurrentLimitEnable = true;
                flywheelLimits.SupplyCurrentLimit = 25;
                flywheelLimits.SupplyCurrentLimitEnable = true;
                MotorOutputConfigs flywheelOutput = new MotorOutputConfigs();
                flywheelOutput.NeutralMode = NeutralModeValue.Coast;
                for (TalonFX flywheel : new TalonFX[] { shooterA, shooterB, shooterC, shooterD }) {
                    flywheel.getConfigurator().apply(flywheelLimits);
                    flywheel.getConfigurator().apply(flywheelOutput);
                }

                // Hood: NEO 550 - 20A smart limit (fragile; hard cap 30A), Brake to hold angle.
                // kNoResetSafeParameters keeps any inversion/encoder settings flashed via REV Hardware Client.
                SparkMaxConfig hoodConfig = new SparkMaxConfig();
                hoodConfig.smartCurrentLimit(20);
                hoodConfig.idleMode(IdleMode.kBrake);
                shooterAngle.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

                // Follower alignments. All 4 must drive the flywheel the same physical direction.
                // B Aligned, D (CAN 16) Opposed (spun wrong way Aligned, 2026-07-16). C (CAN 15)
                // set Opposed 2026-07-17: team observed it spinning opposite the array while
                // Aligned (likely rewired/remounted since the 2026-07-16 test that had it Aligned).
                // TODO on-robot: confirm C no longer fights/skips -- if it does, revert to Aligned.
                shooterB.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Aligned));
                shooterC.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Opposed));
                shooterD.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Opposed));

            //Initializing Drivetrain
                this.drivetrain = drivetrain;
                this.degreesToAlignToTarget = 0.0;
        
            //Initializing Tracker Variables
                enableSubsystem = true;
                desired_Velocity = 0.0;
                desired_Angle = 0.0;
                target_distance = 0.0;

            //Initializing Shuffleboard Entries
                currentVelEntry = ShooterSubsystemTab.add("Current Velocity", 0.0).getEntry();
                desiredVelEntry = ShooterSubsystemTab.add("Desired Velocity", 0.0).getEntry();
                currentAngleEntry = ShooterSubsystemTab.add("Current Angle", 0.0).getEntry();
                desiredAngleEntry = ShooterSubsystemTab.add("Desired Angle", 0.0).getEntry();
                targetDistanceEntry = ShooterSubsystemTab.add("Target Distance", 0.0).getEntry();
                targetClassEntry = ShooterSubsystemTab.add("Target Class", TargetClass.NONE.name()).getEntry();
                shotSetpointEntry = ShooterSubsystemTab.add("Shot Velocity Setpoint", 0.0).getEntry();
                readyToShootEntry = ShooterSubsystemTab.add("Ready To Shoot", false).getEntry();
                aimedEntry = ShooterSubsystemTab.add("Aimed At Target", true).getEntry();
                movingCompEntry = ShooterSubsystemTab.add("Moving Comp Delta", 0.0).getEntry();
                subsystemStateEntry = ShooterSubsystemTab.add("Subsystem State", true).getEntry();
                degreedToAlignToTargEntry = ShooterSubsystemTab.add("Degrees to Align to Target", 0.0).getEntry();
                shooterSpeed_kP = ShooterSubsystemTab.add("SHOOTER KP", shooterVelConfigs.kP).getEntry();
                shooterSpeed_kD = ShooterSubsystemTab.add("SHOOTER KD", shooterVelConfigs.kD).getEntry();
                shooterSpeed_kV = ShooterSubsystemTab.add("SHOOTER KV", shooterVelConfigs.kV).getEntry();
                shooterAngle_kP = ShooterSubsystemTab.add("SHOOTER ANGLE KP", shooterAnglePID.getP()).getEntry();
                shooterAngle_kI = ShooterSubsystemTab.add("SHOOTER ANGLE KI", shooterAnglePID.getI()).getEntry();
                shooterAngle_kD = ShooterSubsystemTab.add("SHOOTER ANGLE KD", shooterAnglePID.getD()).getEntry();
                // Initialized false: with the subsystem disabled at boot these entries were
                // never written again, so a `true` here showed "At Speed: YES" on dashboards
                // while the flywheels coasted.
                desiredVelReachedEntry = ShooterSubsystemTab.add("Desired Velocity Reached", false).getEntry();
                desiredAngleReachedEntry = ShooterSubsystemTab.add("Desired Angle Reached", false).getEntry();
                debugEntry = ShooterSubsystemTab.add("Debug Field", true).getEntry();
       }

    //Utility Methods
        private double getShooterFlywheelVelocity(){
            return 
                shooterA.getVelocity().getValueAsDouble() * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION * 2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS ;
        }

        private double getShooterAngleDegrees(){
            return (shooterAngleEncoder.getPosition() - ShooterSubsystemConstants.SHOOTER_ANGLE_OFFSET) / ShooterSubsystemConstants.NEO550_ROTATIONS_PER_HOOD_DEGREE;
        }

        // Own alliance for the tag partition. DriverStation may not know yet (no FMS / DS
        // pick): default BLUE and warn ONCE instead of spamming every 20 ms loop.
        private Alliance ownAlliance(){
            var alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                if (!allianceWarned) {
                    DriverStation.reportWarning(
                        "ShooterSubsystem: DriverStation alliance unknown -- defaulting to BLUE tag partition", false);
                    allianceWarned = true;
                }
                return Alliance.Blue;
            }
            return alliance.get();
        }

        // Team tag partition (FieldConstants, spec 2026-07-16): SCORE only on OUR alliance's
        // scoring faces (never the opponent hub), FEED on the neutral-facing feed set (either
        // alliance), anything else = NONE (no flywheels, no belts, no kicker).
        private TargetClass classifyTag(int tagId){
            if (FieldConstants.ownScoreTags(ownAlliance()).contains(tagId)) return TargetClass.SCORE;
            if (FieldConstants.FEED_TAGS.contains(tagId)) return TargetClass.FEED;
            return TargetClass.NONE;
        }
    //Subsystem Methods
        public void enableSubsystem(){
            enableSubsystem = true;
        }

        public void disableSubsystem(){
            enableSubsystem = false;
        }

        public void setDesiredFlywheelVelocity(double velocity){
            desired_Velocity = velocity;
        }

        public double getDesiredFlywheelVelocity(){
            return desired_Velocity;
        }

        public void setDesired_Angle(double angle){
            desired_Angle = MathUtil.clamp(angle, ShooterSubsystemConstants.MIN_ANGLE, ShooterSubsystemConstants.MAX_ANGLE);
        }
        public double  getDesiredAngle(){
            return desired_Angle;
        }
        // CCW-positive heading correction (deg) to face the primary SCORE tag of OUR alliance
        // (add it to the current heading), refreshed every loop in periodic(); 0 when no such
        // tag is visible (align commands just hold heading).
        public double getDegreesToAlignToTarget(){
            return degreesToAlignToTarget;
        }

        // True while the primary in-view tag is a SCORE tag of OUR alliance (search-align gate).
        public boolean seesScoringTag(){
            return targetClass == TargetClass.SCORE;
        }

        // True while the active vision shot has a live, legal, in-envelope firing solution.
        public boolean hasShotTarget(){
            return hasShotTarget;
        }

        // Blue-origin field heading (deg) the robot must face to take the current RT shot;
        // empty when no vision aim target exists (no target, or RB blind feed). Consumed by
        // the drivetrain's aim-lock drive layer, re-sampled every loop.
        public OptionalDouble getAimHeadingDegrees(){
            return aimHeadingValid ? OptionalDouble.of(aimHeadingDeg) : OptionalDouble.empty();
        }

        // Aim gate: robot heading within HEADING_TOLERANCE_DEG of the firing bearing.
        // True when no vision aim target exists -- the RB blind feed (driver-aimed) must
        // still be able to open the kicker. Rotation2d.minus wraps, so 179 vs -179 = 2 deg.
        public boolean isAimedAtTarget(){
            if (!aimHeadingValid) {
                return true;
            }
            double errDeg = drivetrain.getState().Pose.getRotation()
                .minus(Rotation2d.fromDegrees(aimHeadingDeg)).getDegrees();
            return Math.abs(errDeg) <= FieldConstants.HEADING_TOLERANCE_DEG;
        }

        // Kicker gate: at-speed AND at-angle AND aimed (when a vision aim target exists) AND a
        // shot actually commanded. A refused or absent target commands velocity 0, which can
        // never read "at speed", so this one predicate is safe for both the RT model shot and
        // the RB fixed feed without extra flags. The aim term physically prevents feeding a
        // shot pointed the wrong way (team request 2026-07-17).
        public boolean isReadyToShoot(){
            return velReached && angleReached && isAimedAtTarget();
        }

        // Shot model core (SHOT MODEL block in ShooterSubsystemConstants has the derivation).
        // No-drag closed form for exit speed through (distance, deltaH) at the FIXED max hood
        // angle (spec 2.4: 44.5 deg needs the lowest speed AND steepest entry everywhere in the
        // envelope), times the distance-fitted drag multiplier, converted to flywheel SURFACE
        // speed (m/s). Returns 0 (= refuse: CoastOut path, at-speed gate never opens, kicker
        // never feeds) when the ball cannot cross the target height DESCENDING or the motor
        // velocity ceiling is exceeded.
        private static double modelSurfaceSpeed(double distance, double deltaH, double dragMult){
            double theta = Math.toRadians(ShooterSubsystemConstants.MAX_ANGLE);
            double reach = distance * Math.tan(theta);
            // Must cross the target height DESCENDING (past apex): d*tan(theta) > 2*deltaH.
            if (distance <= 0 || reach <= 2 * deltaH) {
                return 0;
            }
            double cos = Math.cos(theta);
            double vBall = Math.sqrt(9.81 * distance * distance / (2 * cos * cos * (reach - deltaH)))
                    * dragMult;
            double vSurface = vBall / ShooterSubsystemConstants.SHOT_EFFICIENCY;
            double motorRps = vSurface / (2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
                    * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION);
            if (motorRps > ShooterSubsystemConstants.SHOT_MAX_MOTOR_RPS) {
                return 0; // out of range -- refuse rather than lob short into the field
            }
            return vSurface;
        }

        // Time-of-flight linear fit (s) for the given target class (moving-shot comp, spec 5).
        private static double timeOfFlightSec(double distance, TargetClass cls){
            return cls == TargetClass.SCORE
                ? ShooterSubsystemConstants.SCORE_TOF_BASE_SEC
                    + ShooterSubsystemConstants.SCORE_TOF_SEC_PER_METER * distance
                : ShooterSubsystemConstants.FEED_TOF_BASE_SEC
                    + ShooterSubsystemConstants.FEED_TOF_SEC_PER_METER * distance;
        }

        // No firing solution: wheels to 0 (CoastOut path -- the at-speed gate can never open,
        // so belts/kicker never feed). Hood arg: MAX = staged (valid tag in view, range/legality
        // wrong), MIN = stowed (no usable target at all).
        private void refuseShot(double hoodAngleDeg){
            hasShotTarget = false;
            movingCompMeters = 0;
            setDesiredFlywheelVelocity(0);
            setDesired_Angle(hoodAngleDeg);
        }

        // RT targeting core -- one pass per loop while visionShotCommand() is scheduled.
        // Classification (SCORE own alliance / FEED / NONE) was already refreshed by periodic()
        // this loop (the scheduler runs subsystem periodic() before command execute()).
        private void runVisionTargeting(){
            // Effective class for THIS loop: a fresh tag wins; a momentary dropout within
            // TARGET_HOLD_SEC of the last real tag keeps the previous class alive (ride-through,
            // team request 2026-07-17) so the flywheels don't restart on a one-frame flicker.
            // Initial acquisition still requires a real tag: heldClass is cleared when the RT
            // command starts (beforeStarting) and when it ends.
            TargetClass effClass = targetClass;
            boolean holdover = false;
            if (effClass == TargetClass.NONE && heldClass != TargetClass.NONE
                    && !targetHoldTimer.hasElapsed(ShooterSubsystemConstants.TARGET_HOLD_SEC)) {
                effClass = heldClass;
                holdover = true;
            }
            if (effClass == TargetClass.NONE) {
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot(ShooterSubsystemConstants.MIN_ANGLE);
                return;
            }

            // Field-pose source: a fresh Limelight botpose while a tag is in view (wpiBlue
            // frame; getBotPoseEstimate returns null on empty NT data); during a holdover the
            // drivetrain's FUSED pose instead -- MegaTag2-corrected odometry carries the last
            // vision fix forward across the dropout, so strafing keeps both distance and aim
            // accurate with no tag in frame. NOTE: botpose is only as good as the camera mount
            // pose set in the Limelight web UI -- TODO on-robot: verify the mount config; until
            // then the camera-space fallback below is effectively the primary path.
            var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(Vision.CAM_LIMELIGHT);
            boolean botposeValid = !holdover && poseEstimate != null && poseEstimate.tagCount > 0;
            Pose2d fieldPose = botposeValid ? poseEstimate.pose
                             : holdover ? drivetrain.getState().Pose
                             : null;

            double distance;   // horizontal m, robot -> target point
            double vRadial;    // m/s along the target line, positive = moving AWAY from target
            if (fieldPose != null) {
                Alliance alliance = ownAlliance();
                Translation2d robot = fieldPose.getTranslation();
                Translation2d target = effClass == TargetClass.SCORE
                    ? FieldConstants.ownHubCenter(alliance)
                    : FieldConstants.ownFeedTarget(alliance);
                distance = robot.getDistance(target);
                // AUTO-AIM bearing (team request 2026-07-17): blue-origin field heading from the
                // SAME pose used for distance to the target point. Stays valid through the
                // legality/envelope refusals below, so the drivetrain holds aim while the driver
                // strafes back into range.
                if (distance > 0.01) {
                    aimHeadingDeg = Math.toDegrees(Math.atan2(
                        target.getY() - robot.getY(), target.getX() - robot.getX()));
                    aimHeadingValid = true;
                } else {
                    aimHeadingValid = false;
                }
                if (effClass == TargetClass.SCORE
                        && !FieldConstants.isLegalScoringX(robot.getX(), alliance)) {
                    // Rule G407: never spin up a scoring shot from outside our alliance zone.
                    target_distance = 0;
                    refuseShot(ShooterSubsystemConstants.MAX_ANGLE);
                    return;
                }
                // Radial speed: rotate the robot-relative chassis speeds into the field frame
                // with the SAME pose's rotation (one consistent frame), then project onto the
                // robot->target unit vector. Positive = distance growing = moving away.
                ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    drivetrain.getState().Speeds, fieldPose.getRotation());
                double ux = (target.getX() - robot.getX()) / distance;
                double uy = (target.getY() - robot.getY()) / distance;
                vRadial = -(fieldSpeeds.vxMetersPerSecond * ux + fieldSpeeds.vyMetersPerSecond * uy);
            } else if (effClass == TargetClass.SCORE) {
                // Distance source 2 (fallback, fresh SCORE tag only): camera-space tag
                // translation + the fixed tag-face -> hub-center geometry (every hub tag plane
                // sits 0.6035 m in front of the hub center). Camera space X = right, Z =
                // forward; treated as the ROBOT frame until the camera mount pose is measured
                // (TODO on-robot -- a mount offset shifts every fallback distance).
                double x = tagCameraPose.getX() + FieldConstants.tagLateralOffsetMeters(visibleTagId);
                double z = tagCameraPose.getZ() + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS;
                distance = Math.hypot(x, z);
                // Radial speed in the robot frame (x forward, y LEFT; the camera bearing is
                // positive to the RIGHT, so the target direction is (cos b, -sin b)).
                ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;
                double bearing = Math.atan2(x, z);
                // AUTO-AIM bearing, fallback path: current heading MINUS the right-positive
                // camera bearing = CCW-positive field heading to the HUB CENTER (the lateral
                // term steers to center, not the tag face, once TAG_LATERAL_OFFSET_SIGN is set).
                aimHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees()
                    - Math.toDegrees(bearing);
                aimHeadingValid = true;
                vRadial = -(robotSpeeds.vxMetersPerSecond * Math.cos(bearing)
                          - robotSpeeds.vyMetersPerSecond * Math.sin(bearing));
            } else {
                // FEED with no usable botpose and no holdover pose: a camera-space tag cannot
                // locate a field aim point, so REFUSE (no shooting operations). The driver
                // still has RB for a deliberate blind lob -- NOT auto-substituted here
                // (deviation from shot-math spec 3.2 per team task spec: refusing is the
                // conservative branch).
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot(ShooterSubsystemConstants.MIN_ANGLE);
                return;
            }
            // Degenerate-pose guard: a garbage botpose sitting ON the target point would divide
            // by ~0 above and float a NaN through every envelope gate (NaN comparisons are all
            // false) straight into VelocityVoltage. Refuse instead.
            if (!(distance > 0.01)) {   // also catches NaN
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot(ShooterSubsystemConstants.MIN_ANGLE);
                return;
            }
            target_distance = distance;

            // Moving-shot radial compensation (spec 5): virtual-target fixed point in effective
            // distance, exactly two iterations, clamped to +-1.5 m. Lateral (tangential) motion
            // is NOT compensated -- documented limitation; the aim lock now keeps the nose on
            // target while strafing, but the ball still inherits tangential velocity: shoot
            // roughly still or radial. A fast approach into the score dead zone correctly
            // REFUSES via the envelope gate below.
            double gain = ShooterSubsystemConstants.MOVING_COMP_GAIN;
            double d1 = distance + gain * vRadial * timeOfFlightSec(distance, effClass);
            movingCompMeters = MathUtil.clamp(gain * vRadial * timeOfFlightSec(d1, effClass),
                -ShooterSubsystemConstants.MOVING_COMP_MAX_METERS,
                ShooterSubsystemConstants.MOVING_COMP_MAX_METERS);
            double dEff = distance + movingCompMeters;

            double vSurface;
            if (effClass == TargetClass.SCORE) {
                // Outside [3.0, 6.2] m the shot is a guaranteed miss or illegal (spec 2.3) --
                // refuse, hood staged at MAX (a valid tag is in view, only the range is wrong).
                if (dEff < ShooterSubsystemConstants.MIN_SCORE_DISTANCE_METERS
                        || dEff > ShooterSubsystemConstants.MAX_SCORE_DISTANCE_METERS) {
                    refuseShot(ShooterSubsystemConstants.MAX_ANGLE);
                    return;
                }
                vSurface = modelSurfaceSpeed(dEff,
                    FieldConstants.HUB_OPENING_HEIGHT_METERS
                        - ShooterSubsystemConstants.SHOT_RELEASE_HEIGHT_METERS,
                    ShooterSubsystemConstants.SCORE_DRAG_MULT_BASE
                        + ShooterSubsystemConstants.SCORE_DRAG_MULT_PER_METER * dEff);
            } else {
                // FEED lob lands on the carpet (deltaH = -release height). Clamp, never refuse
                // for range: a slightly-short lob still lands in friendly territory (spec 3.2).
                dEff = MathUtil.clamp(dEff,
                    ShooterSubsystemConstants.MIN_FEED_DISTANCE_METERS,
                    ShooterSubsystemConstants.MAX_FEED_DISTANCE_METERS);
                vSurface = modelSurfaceSpeed(dEff,
                    -ShooterSubsystemConstants.SHOT_RELEASE_HEIGHT_METERS,
                    ShooterSubsystemConstants.FEED_DRAG_MULT_BASE
                        + ShooterSubsystemConstants.FEED_DRAG_MULT_PER_METER * dEff);
            }
            setDesired_Angle(ShooterSubsystemConstants.MAX_ANGLE);
            setDesiredFlywheelVelocity(vSurface);
            hasShotTarget = vSurface > 0;
        }

    //Command Based Methods
        public Command enableSubsystemCommand(){
            return Commands.runOnce(()->{
                        this.enableSubsystem();
                    });
        }
        
        public Command disableSubsystemCommand(){
            return Commands.runOnce(()->{
                        this.disableSubsystem();
                    });
        }

        public Command setAngleAndVelocityCommand(double angle, double velocity){
            return runOnce(()->{
                        this.setDesired_Angle(angle);
                        this.setDesiredFlywheelVelocity(velocity);
                    });
        }

        public Command enableLiveData(boolean isEnabled){
           return Commands.runOnce(
            ()->{
                this.enableComp = isEnabled;
                // Leaving live-data mode: zero the setpoints ONCE here (this used to happen
                // every loop in periodic(), which also stomped preset shot commands).
                if (!isEnabled) {
                    desired_Velocity = 0;
                    desired_Angle = ShooterSubsystemConstants.MIN_ANGLE;
                }
            }
            );
        }

        // Coast the flywheels down and drop the hood to its floor angle.
        public Command stopShooterCommand(){
            return setAngleAndVelocityCommand(ShooterSubsystemConstants.MIN_ANGLE, 0);
        }

        // RT (hold) = precision vision shot. Every loop: classify the primary tag (SCORE own
        // alliance / FEED / NONE), pick the distance source (botpose, else camera-space for
        // SCORE only), moving-comp, envelope gates, then hood MAX + model velocity. NONE or any
        // refusal keeps velocity 0 so belts/kicker never feed. Release: flywheels coast
        // (0 -> CoastOut), hood recesses to MIN, target flags clear.
        // (The old testSpinCommand/hoodJogCommand and their RT/DPAD bindings are superseded and
        // deleted per team spec 2026-07-16.)
        public Command visionShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();   // shot commands re-enable per team directive 2026-07-16
                    runVisionTargeting();
                },
                () -> {
                    refuseShot(ShooterSubsystemConstants.MIN_ANGLE);
                    target_distance = 0;
                    aimHeadingValid = false;
                    heldClass = TargetClass.NONE; // next press must acquire a fresh tag
                })
                // Clear any pre-press residue so the spec holds exactly: pressing RT with no
                // tag found does nothing, even if one was seen moments before the press.
                .beforeStarting(() -> heldClass = TargetClass.NONE);
        }

        // RB (hold) = fixed blind feed: fixed hood angle + fixed tunable feed surface speed.
        // The kicker stays gated through isReadyToShoot() in the hopper bindings (at-speed +
        // at-angle). Release: flywheels coast, hood recesses to MIN.
        public Command feedAngleShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();
                    setDesired_Angle(ShooterSubsystemConstants.RB_FEED_ANGLE);
                    setDesiredFlywheelVelocity(ShooterSubsystemConstants.RB_FEED_SURFACE_SPEED);
                },
                () -> refuseShot(ShooterSubsystemConstants.MIN_ANGLE));
        }

    @Override
    public void periodic(){
        // Vision sensing -- read-only, EVERY loop (even while disabled): classify the primary
        // tag and keep the align bearing fresh for the drivetrain's A / DPAD-UP search-align.
        // Cheap NetworkTables reads for the PRIMARY target only -- the old getLatestResults()
        // call deserialized the Limelight's full JSON dump with Jackson every 20 ms loop,
        // which stalls the whole robot loop.
        targetClass = TargetClass.NONE;
        visibleTagId = -1;
        if (LimelightHelpers.getTV(Vision.CAM_LIMELIGHT)) {
            visibleTagId = (int) LimelightHelpers.getFiducialID(Vision.CAM_LIMELIGHT);
            targetClass = classifyTag(visibleTagId);
        }
        // Record the tag-flicker ride-through state (see runVisionTargeting): any real
        // classification refreshes the hold window.
        if (targetClass != TargetClass.NONE) {
            heldClass = targetClass;
            targetHoldTimer.restart();
        }
        if (targetClass == TargetClass.SCORE) {
            // Limelight camera space: X = right, Y = DOWN, Z = forward (depth). Cached here for
            // the fallback distance path. TODO verify signs/axes on the robot with a real tag.
            tagCameraPose = LimelightHelpers.getTargetPose3d_CameraSpace(Vision.CAM_LIMELIGHT);
            // Camera X is RIGHT-positive but WPILib field heading is CCW-positive, so the
            // bearing is NEGATED: a tag to the robot's right needs a clockwise (negative)
            // heading delta. Consumers ADD this value to the current heading to face the tag.
            degreesToAlignToTarget = Math.toDegrees(Math.atan2(-tagCameraPose.getX(), tagCameraPose.getZ()));
        } else {
            degreesToAlignToTarget = 0; // no OWN score tag -> align commands just hold heading
        }

        if(enableSubsystem){
            //Shooter Speed
                m_velocity.Slot = 0;
                double motorRps = desired_Velocity /(2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION);
                if (motorRps == 0) {
                    // "Off" = coast down freely, never VelocityVoltage(0) (see m_flywheelCoast note).
                    shooterA.setControl(m_flywheelCoast);
                } else {
                    shooterA.setControl(m_velocity.withVelocity(motorRps));
                }

            //Shooter Angle
                // Hood-lower interlock (team request 2026-07-17): never drive the hood DOWN while
                // the flywheels are still spinning above HOOD_LOWER_MAX_SURFACE_SPEED -- hold the
                // current angle and let them coast down first. Raising the hood (staging a shot) is
                // always allowed. desired_Angle stays latched, so the hood drops on its own the
                // moment the wheels are slow enough -- "hood only auto-lowers when flywheels are
                // low / stopped".
                double currentHoodAngle = getShooterAngleDegrees();
                double hoodTarget = MathUtil.clamp(desired_Angle, ShooterSubsystemConstants.MIN_ANGLE, ShooterSubsystemConstants.MAX_ANGLE);
                if (hoodTarget < currentHoodAngle
                        && Math.abs(getShooterFlywheelVelocity()) > ShooterSubsystemConstants.HOOD_LOWER_MAX_SURFACE_SPEED) {
                    hoodTarget = currentHoodAngle; // flywheels too fast -- hold, don't lower yet
                }
                double anglePID = shooterAnglePID.calculate(currentHoodAngle, hoodTarget);
                    shooterAngle.setVoltage(MathUtil.clamp(anglePID, -12, 12));
            //At-speed / at-angle kicker gates -- instance state (the mutable statics in
            // ShooterSubsystemConstants are DELETED; the hopper polls isReadyToShoot()).
                velReached =
                    desired_Velocity != 0 ? Math.abs(desired_Velocity-getShooterFlywheelVelocity()) < ShooterSubsystemConstants.SPEED_TOLERANCE : false;
                angleReached =
                     desired_Angle != 0 ? Math.abs(desired_Angle-getShooterAngleDegrees()) < ShooterSubsystemConstants.ANGLE_TOLERANCE : false;
            //Data (tuning entries only -- these double as dashboard INPUTS below, so they
            // are written only while enabled; the read-only status entries moved to the
            // always-published block at the bottom of periodic()).
                shooterSpeed_kP.setDouble(shooterVelConfigs.kP);
                shooterSpeed_kD.setDouble(shooterVelConfigs.kD);
                shooterSpeed_kV.setDouble(shooterVelConfigs.kV);
                shooterAngle_kP.setDouble(shooterAnglePID.getP());
                shooterAngle_kI.setDouble(shooterAnglePID.getI());
                shooterAngle_kD.setDouble(shooterAnglePID.getD());
                debugEntry.setBoolean(enableComp);

            //Physics Lab: live-data mode reads setpoints straight off Shuffleboard. No else --
            // zeroing on exit happens ONCE in enableLiveData(false); an every-loop else here
            // would stomp the shot commands (visionShotCommand/feedAngleShotCommand) every loop.
                if(enableComp){
                    desired_Velocity = desiredVelEntry.getDouble(0);
                    desired_Angle = desiredAngleEntry.getDouble(0);
                }

            //PID + FF Tuning
                //Speed
                    if(shooterVelConfigs.kP != shooterSpeed_kP.getDouble(shooterVelConfigs.kP) ||
                       shooterVelConfigs.kD != shooterSpeed_kD.getDouble(shooterVelConfigs.kD) ||
                       shooterVelConfigs.kV != shooterSpeed_kV.getDouble(shooterVelConfigs.kV)){
                        shooterVelConfigs.kP = shooterSpeed_kP.getDouble(shooterVelConfigs.kP);
                        shooterVelConfigs.kD = shooterSpeed_kD.getDouble(shooterVelConfigs.kD);
                        shooterVelConfigs.kV = shooterSpeed_kV.getDouble(shooterVelConfigs.kV);
                        shooterA.getConfigurator().apply(shooterVelConfigs);
                    }
                //Angle
                    // Compare the live PID against the SHUFFLEBOARD entries (like the speed block
                    // above) -- the old guard compared against the static constants the PID was
                    // built from, which never change, so slider edits silently never applied.
                    if(shooterAnglePID.getP() != shooterAngle_kP.getDouble(shooterAnglePID.getP()) ||
                       shooterAnglePID.getI() != shooterAngle_kI.getDouble(shooterAnglePID.getI()) ||
                       shooterAnglePID.getD() != shooterAngle_kD.getDouble(shooterAnglePID.getD())){
                        shooterAnglePID.setPID(
                            shooterAngle_kP.getDouble(shooterAnglePID.getP()),
                            shooterAngle_kI.getDouble(shooterAnglePID.getI()),
                            shooterAngle_kD.getDouble(shooterAnglePID.getD())
                        );
                    }
            } else {
                // Subsystem disabled while the robot is still enabled: VelocityVoltage LATCHES on
                // the TalonFX, so just skipping the body would leave the flywheels spinning at the
                // last setpoint forever. Explicitly coast the flywheels and stop the hood (its
                // Brake idle mode holds the angle).
                shooterA.setControl(m_flywheelCoast);
                shooterAngle.setVoltage(0);
                desired_Velocity = 0;
                // Drop the kicker gates too: these flags latch the last enabled-loop value,
                // and the hopper's kicker fires on isReadyToShoot() -- a disable mid-shot must
                // never leave them true while the flywheels coast down.
                velReached = false;
                angleReached = false;
                hasShotTarget = false;
                aimHeadingValid = false;
            }

        //Data -- read-only status, published EVERY loop (enabled or not) so dashboards
        // never show stale state (the old enabled-only write froze "Subsystem State" /
        // "Desired Velocity Reached" at their boot values while disabled).
            currentVelEntry.setDouble(getShooterFlywheelVelocity());
            currentAngleEntry.setDouble(getShooterAngleDegrees());
            targetDistanceEntry.setDouble(target_distance);
            targetClassEntry.setString(targetClass.name());
            shotSetpointEntry.setDouble(desired_Velocity);
            readyToShootEntry.setBoolean(isReadyToShoot());
            aimedEntry.setBoolean(isAimedAtTarget());
            movingCompEntry.setDouble(movingCompMeters);
            subsystemStateEntry.setBoolean(enableSubsystem);
            degreedToAlignToTargEntry.setDouble(degreesToAlignToTarget);
            desiredVelReachedEntry.setBoolean(velReached);
            desiredAngleReachedEntry.setBoolean(angleReached);
            // Desired Velocity/Angle double as INPUTS in live-data mode (enableComp reads
            // them back above) -- only echo the real setpoints when NOT in that mode, so a
            // dashboard edit is never stomped mid-tune.
            if (!enableComp) {
                desiredVelEntry.setDouble(desired_Velocity);
                desiredAngleEntry.setDouble(desired_Angle);
            }
    }

    // Read-only motor access for PowerTelemetry (no control).
    public TalonFX[] getFlywheelMotors(){
        return new TalonFX[] { shooterA, shooterB, shooterC, shooterD };
    }

    // Read-only motor access for PowerTelemetry (no control).
    public SparkMax getHoodMotor(){
        return shooterAngle;
    }
}
