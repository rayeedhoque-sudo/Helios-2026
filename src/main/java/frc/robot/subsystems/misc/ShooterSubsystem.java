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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

import java.util.function.DoubleSupplier;

import java.util.OptionalDouble;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;
import frc.robot.util.Tunable;
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

    // HOOD CLOSED-LOOP ENABLE (kill switch). false = hold the hood at 0 V (brake idle holds
    // position) -- the safe state if the angle feedback ever misbehaves again. Re-enabled 2026-07-18
    // after getShooterAngleDegrees() was recalibrated to the absolute encoder's real two-point map
    // (HOOD_*_AT_FULL_UP / HOOD_*_AT_FULL_DOWN); the old motor-rotation math railed the hood.
    // The absolute raw -> degrees map it refers to is DELETED (2026-08-27): the angle comes
    // from the fixed enable-time datum plus accumulated raw deltas, so no raw reading is ever
    // interpreted as a position. See captureHoodDatum. NOTE: the hood PID gains and HOOD_RAISE_FF_VOLTS were tuned against
    // the OLD broken mapping, so they are effectively untuned -- expect overshoot or
    // sluggishness and retune on robot.
    private static final boolean HOOD_CLOSED_LOOP_ENABLED = true;

     //Shooter Speed - PID & FF
        private final VelocityVoltage m_velocity = new VelocityVoltage(0);
        // Coast request for "flywheels off": closing the velocity loop on 0 rps would actively
        // reverse-brake 4 spinning Krakens (regen current dump + gearbox stress) -- the Coast
        // neutral mode only applies when no closed-loop request is latched.
        private final CoastOut m_flywheelCoast = new CoastOut();
        // All six gains are applied, not just kP/kD/kV: kI/kS/kA existed as constants but were
        // never reaching the controller, so the PID panel would have shown three knobs that did
        // nothing. Their defaults are 0, so wiring them up changes no behavior.
        private final Slot0Configs shooterVelConfigs =
            new Slot0Configs().
            withKP(ShooterSubsystemConstants.SHOOTER_SPEED_kP).
            withKI(ShooterSubsystemConstants.SHOOTER_SPEED_kI).
            withKD(ShooterSubsystemConstants.SHOOTER_SPEED_kD).
            withKS(ShooterSubsystemConstants.SHOOTER_SPEED_kS).
            withKV(ShooterSubsystemConstants.SHOOTER_SPEED_kV).
            withKA(ShooterSubsystemConstants.SHOOTER_SPEED_kA);
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
            // MEASURED flywheel speed in motor RPM (2026-08-22). Current Velocity is the same
            // reading in m/s of SURFACE speed -- the units the shot model works in -- but the
            // DPAD trims in RPM and RPM is what the team reads off the shooter, so publish both
            // rather than making anyone convert.
            private final GenericEntry currentRpmEntry;
            private final GenericEntry desiredAngleEntry;
            private final GenericEntry targetDistanceEntry;
            private final GenericEntry targetClassEntry;
            private final GenericEntry shotSetpointEntry;
            private final GenericEntry readyToShootEntry;
            private final GenericEntry aimedEntry;
            private final GenericEntry movingCompEntry;
            private final GenericEntry subsystemStateEntry;
            private final GenericEntry degreedToAlignToTargEntry;
            // Gains moved off Shuffleboard onto the driver-companion PID panel (NT /Tuning/*).
            // The old GenericEntry boxes had the robot and the dashboard both writing the same
            // entry every loop, so a typed value survived only if it landed in the gap between
            // the echo and the read -- tuning appeared to work, then silently reverted. Tunable
            // is read-only on the robot side, which removes that race. See frc.robot.util.Tunable.
            private final Tunable tuneSpeedKp = new Tunable("Shooter/Flywheel/kP", ShooterSubsystemConstants.SHOOTER_SPEED_kP);
            private final Tunable tuneSpeedKi = new Tunable("Shooter/Flywheel/kI", ShooterSubsystemConstants.SHOOTER_SPEED_kI);
            private final Tunable tuneSpeedKd = new Tunable("Shooter/Flywheel/kD", ShooterSubsystemConstants.SHOOTER_SPEED_kD);
            private final Tunable tuneSpeedKs = new Tunable("Shooter/Flywheel/kS", ShooterSubsystemConstants.SHOOTER_SPEED_kS);
            private final Tunable tuneSpeedKv = new Tunable("Shooter/Flywheel/kV", ShooterSubsystemConstants.SHOOTER_SPEED_kV);
            private final Tunable tuneSpeedKa = new Tunable("Shooter/Flywheel/kA", ShooterSubsystemConstants.SHOOTER_SPEED_kA);
            private final Tunable tuneAngleKp = new Tunable("Shooter/Hood/kP", ShooterSubsystemConstants.SHOOTER_ANGLE_kP);
            private final Tunable tuneAngleKi = new Tunable("Shooter/Hood/kI", ShooterSubsystemConstants.SHOOTER_ANGLE_kI);
            private final Tunable tuneAngleKd = new Tunable("Shooter/Hood/kD", ShooterSubsystemConstants.SHOOTER_ANGLE_kD);
            private final GenericEntry desiredVelReachedEntry;
            private final GenericEntry desiredAngleReachedEntry;
            private final GenericEntry debugEntry;
            // Raw absolute-encoder position (rotations, pre-conversion) -- the anchor readout for
            // rebuilding getShooterAngleDegrees(). Hand-move the hood to each hard stop (robot
            // DISABLED) and record this at min + max; if it never changes, the encoder isn't tracking.
            private final GenericEntry rawHoodEncoderEntry;
            // Hood drive telemetry (2026-08-22): the commanded volts and the measured current,
            // published every loop. Added to tune the ascent out of stick-slip -- the volts at
            // the instant the hood starts moving IS the breakaway threshold, and there was no
            // way to see it before. Current also shows a stall (driving hard, not moving).
            private final GenericEntry hoodVoltsEntry;
            private final GenericEntry hoodCurrentEntry;
            // Tracker state (2026-08-22): the datum the angle is measured from, and how far the
            // hood has travelled since. Without these the tracked angle is unfalsifiable.
            private final GenericEntry hoodBaseAngleEntry;
            private final GenericEntry hoodRelativeEntry;
            // Live RT flywheel target, both units -- the DPAD trims in RPM, the model works in
            // surface speed, and the driver needs to see what the next press is adjusting.
            private final GenericEntry rtSpeedEntry;
            private final GenericEntry rtSpeedRpmEntry;
            // AUTO-AIM VISIBILITY (2026-08-29). Without these the only way to tell which tag RB
            // locked onto, and how far it thinks it must turn, was to infer it from the robot's
            // behaviour -- which is how a 2 deg geometry error stayed invisible for a session.
            private final GenericEntry autoAimTagEntry;
            private final GenericEntry autoAimBearingEntry;
            private final GenericEntry autoAimViewAngleEntry;

    //Tracker Variables
       private boolean enableSubsystem;
       // Last voltage actually commanded to the hood, for the telemetry block at the bottom of
       // periodic() (which publishes whether or not the subsystem is enabled).
       private double lastHoodVolts;
       // CONTINUOUS HOOD TRACKING (2026-08-22). The live angle is a datum captured at enable
       // plus every shortest-path delta since, NOT a fresh interpretation of each reading --
       // see HOOD_DEG_PER_RAW_UNIT for why the absolute reading alone cannot be trusted.
       // Live RT flywheel target (m/s surface), trimmed by the DPAD UP/DOWN bindings. Seeded
       // from the constant and NOT persisted -- a reboot or redeploy returns it to the constant.
       private double rtSurfaceSpeed = ShooterSubsystemConstants.RT_FLYWHEEL_SURFACE_SPEED;
       // True while the hood is parked at the angle a move left it at, held at 0 V. The latch
       // is ABSOLUTE (2026-08-28): set on arrival, cleared ONLY by setDesired_Angle. Nothing
       // the hood does on its own can restart the drive, so it cannot hunt.
       private boolean hoodHolding;
       private double hoodBaseAngleDeg;   // absolute angle at the datum
       private double hoodRelativeDeg;    // signed degrees travelled since the datum
       private double hoodLastRaw;        // previous raw reading, for the delta
       private boolean hoodDatumValid;    // false until a datum has been captured
       private boolean hoodWasEnabled;    // rising-edge detect on DriverStation.isEnabled()
       // Which way the current setpoint asked the hood to go, captured when the setpoint was
       // written. The arrival latch in periodic() needs it -- see hoodMoveReached.
       private boolean hoodMoveRising;
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
       // The DAMPED heading the RB rotation servo chases (see AUTOAIM_ROTATION_SCALE). Separate
       // from aimHeadingDeg, which stays the true bearing so the kicker's aim gate is not
       // widened by the damping.
       private double autoAimServoHeadingDeg = 0.0;
       private boolean autoAimServoSeeded = false;   // false = adopt the next target outright
       private int autoAimTagId = -1;                // which tag auto-aim actually chose
       private double autoAimBearingDeg = 0.0;       // how far it wants to turn, + = right
       private double autoAimViewAngleDeg = 0.0;     // how far off the face's normal we are
       private boolean aimHeadingValid = false;
       // Tag-flicker ride-through: remember the last non-NONE classification so a momentary
       // dropout (< TARGET_HOLD_SEC) rides on the drivetrain's fused pose instead of
       // restarting the flywheels. Initial acquisition still requires a real tag.
       private TargetClass heldClass = TargetClass.NONE;
       private final Timer targetHoldTimer = new Timer();
       // RB auto-aim tag-loss ride-through (team request 2026-08-29). Its own clock, not
       // targetHoldTimer's: that one is restarted by periodic() on ANY class of tag, while
       // this must only count from the last frame auto-aim actually used.
       private final Timer autoAimTagTimer = new Timer();
       private boolean autoAimHadTag = false;
       // The hood target auto-aim last COMMANDED, and whether it has commanded one this hold.
       // NOT the same thing as desired_Angle -- see autoAimShouldRetarget for why reading the
       // setpoint back is what made the hood oscillate.
       private double autoAimLastTarget = 0;
       private boolean autoAimHasCommanded = false;

       // Vision targeting now lives entirely inside visionShotCommand() -- the old enableVision
       // constructor flag is gone. INTEGRATOR NOTE: construct with just the drivetrain and
       // delete the RobotContainer boot-time disableSubsystem() call; the shot commands enable
       // the subsystem themselves (shooter re-enabled by team directive 2026-07-16).
       public ShooterSubsystem(CommandSwerveDrivetrain drivetrain){

            //Coniguring Motors
                shooterA.getConfigurator().apply(shooterVelConfigs);

                // Current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
                // Flywheels: Kraken X60 - stator 80A / supply 25A, Coast (let them spin down freely).
                // STATOR was raised 40 -> 80 A (Regular tier) 2026-07-17: the feed/precision shots
                // "barely left the robot" -- the 40 A cap choked the torque IMPULSE when a ball hits,
                // so the wheel sagged deep on contact instead of driving the ball. 80 A doubled the
                // anti-bog torque; that spike is brief (ball contact ~tens of ms) and sourced mostly
                // from the flywheel's inertia + DC link, NOT sustained battery draw.
                // STATOR 80 -> 50 A (2026-08-21, team-directed). NOTE: this walks the 07-17 fix most
                // of the way back toward the 40 A that produced weak shots, so the "barely left the
                // robot" symptom is the expected failure mode -- if it returns, restore 80 FIRST.
                // It buys nothing on either axis it might look like it should: supply (unchanged at
                // 25 A) is the brownout knob, and a Kraken X60 stalls at 366 A so 80 A intermittent
                // was never a thermal problem (watch DeviceTemp in PowerTelemetry).
                // UNVERIFIED ON ROBOT: the hood encoder reads 0 (dead) as of this change, so no
                // representative shot could be taken -- retest shots once the hood is restored.
                // SUPPLY deliberately HELD at 25 A: supply is the battery-draw / BROWNOUT knob and the
                // team browns out late-match -- keeping it at 25 A means this change adds ~0 to the
                // spin-up draw (still 25 A x4 = 100 A). If rapid-fire recovery between balls is still
                // weak on retest, raise supply toward 30 A (Regular tier) -- but only on a healthy
                // battery, and expect ~+20 A worst-case drivetrain-shot overlap. Cap is 120/40, never exceed.
                CurrentLimitsConfigs flywheelLimits = new CurrentLimitsConfigs();
                flywheelLimits.StatorCurrentLimit = 50;
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
                currentRpmEntry = ShooterSubsystemTab.add("Current Flywheel (motor RPM)", 0.0).getEntry();
                desiredAngleEntry = ShooterSubsystemTab.add("Desired Angle", 0.0).getEntry();
                targetDistanceEntry = ShooterSubsystemTab.add("Target Distance", 0.0).getEntry();
                targetClassEntry = ShooterSubsystemTab.add("Target Class", TargetClass.NONE.name()).getEntry();
                shotSetpointEntry = ShooterSubsystemTab.add("Shot Velocity Setpoint", 0.0).getEntry();
                readyToShootEntry = ShooterSubsystemTab.add("Ready To Shoot", false).getEntry();
                aimedEntry = ShooterSubsystemTab.add("Aimed At Target", true).getEntry();
                movingCompEntry = ShooterSubsystemTab.add("Moving Comp Delta", 0.0).getEntry();
                subsystemStateEntry = ShooterSubsystemTab.add("Subsystem State", true).getEntry();
                degreedToAlignToTargEntry = ShooterSubsystemTab.add("Degrees to Align to Target", 0.0).getEntry();
                // Initialized false: with the subsystem disabled at boot these entries were
                // never written again, so a `true` here showed "At Speed: YES" on dashboards
                // while the flywheels coasted.
                desiredVelReachedEntry = ShooterSubsystemTab.add("Desired Velocity Reached", false).getEntry();
                desiredAngleReachedEntry = ShooterSubsystemTab.add("Desired Angle Reached", false).getEntry();
                debugEntry = ShooterSubsystemTab.add("Debug Field", true).getEntry();
                rawHoodEncoderEntry = ShooterSubsystemTab.add("Hood Encoder Raw (rot)", 0.0).getEntry();
                hoodVoltsEntry = ShooterSubsystemTab.add("Hood Volts (cmd)", 0.0).getEntry();
                hoodCurrentEntry = ShooterSubsystemTab.add("Hood Current (A)", 0.0).getEntry();
                hoodBaseAngleEntry = ShooterSubsystemTab.add("Hood Base Angle (deg)", 0.0).getEntry();
                hoodRelativeEntry = ShooterSubsystemTab.add("Hood Travel Since Datum (deg)", 0.0).getEntry();
                rtSpeedEntry = ShooterSubsystemTab.add("RT Target (m/s)", 0.0).getEntry();
                rtSpeedRpmEntry = ShooterSubsystemTab.add("RT Target (motor RPM)", 0.0).getEntry();
                autoAimTagEntry = ShooterSubsystemTab.add("Auto-Aim Tag", -1.0).getEntry();
                autoAimBearingEntry = ShooterSubsystemTab.add("Auto-Aim Bearing (deg, + = hub right of crosshair)", 0.0).getEntry();
                autoAimViewAngleEntry = ShooterSubsystemTab.add("Auto-Aim View Angle (deg)", 0.0).getEntry();
       }

    //Utility Methods
        private double getShooterFlywheelVelocity(){
            return 
                shooterA.getVelocity().getValueAsDouble() * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION * 2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS ;
        }

        // Is a raw hood reading trustworthy enough to close the loop on? Package-private +
        // static for HoodAngleMapTest.
        //
        // TWO checks, and the second one only exists because of the unwrap. Before the wrap
        // fix, a dead encoder's raw 0.0 fell outside the raw band and was caught for free.
        // Unwrapped, 0.0 lifts to 360 -- a perfectly plausible mid-travel ~35.9 deg -- so the
        // band alone would now close the loop on a silent encoder and hold the hood at a
        // fictional angle. A live encoder crossing the rollover reads 0.024, 359.98, and so on;
        // bit-exact 0.0 is what a controller with no encoder data publishes. Rejecting it costs
        // at most a loop or two of 0 V if a real reading ever lands exactly on zero, and the
        // brake idle holds the hood through that.
        // TODO on-robot: replace the zero test with the SPARK MAX's own sensor fault flag once
        // someone confirms which REVLib 2026 fault bit reports a missing absolute encoder.
        static boolean isHoodFeedbackValid(double raw){
            // ONLY the dead-encoder test. The absolute band that used to live here was removed
            // 2026-08-22: with the angle tracked RELATIVELY, no dial position is illegal -- the
            // encoder drifts against the hood, so the band eventually rejects wherever the
            // encoder has wandered to, forcing 0 V and leaving the hood dead on the DPAD. That
            // is exactly the failure it caused: an angle of -32.96 deg with a +38 deg error that
            // never produced a volt. Bad DATA is caught where it belongs now -- in the delta
            // filters (hoodDeltaDegrees), which reject noise and glitches frame by frame.
            //
            // A silent SPARK MAX still publishes bit-exact 0.0, and that is not a position.
            return raw != 0.0;
        }

        // (hoodShouldHold DELETED 2026-08-28. It re-engaged the loop whenever the hood drifted
        // past HOOD_REENGAGE_DEG, and that re-engagement WAS the oscillation the team saw: a
        // catch stroke is 25-55 raw units against a 3.75-unit settle band, so every catch
        // overshot, the arrival latch re-seeded higher, the hood drooped again, and it caught
        // again -- forever. No gain fixes that; the smallest move the mechanism makes is 3-5x
        // the band it is meant to settle in. The latch is now ABSOLUTE: it is set on arrival
        // and cleared ONLY by a setpoint write, i.e. a driver press or a shot command. See
        // hoodHolding and setDesired_Angle.)

        // Gravity LIFT feedforward (volts) for a given angle error, target minus current.
        // RAMPS from 0 V at ANGLE_TOLERANCE to HOOD_RAISE_FF_VOLTS at ANGLE_TOLERANCE +
        // HOOD_FF_FADE_DEG. It used to be a step at ANGLE_TOLERANCE, which drove the hood in
        // visible lurches on the DPAD jog (see HOOD_FF_FADE_DEG). Endpoints are deliberately
        // unchanged: 0 V at/inside the target so the hood is never held against a stop by the
        // FF, full 5 V at a large error so breakaway still clears gravity.
        // Never negative -- lowering is gravity-assisted and stays FF-free.
        // Package-private + static for HoodFeedforwardTest.
        static double hoodLiftFeedforward(double errorDeg){
            // LOWERING (error negative): the hood used to get P only on this stroke, and P is
            // far too small -- -4.90 V measured on the robot moved it not at all. Mirror the
            // ramp downward, at HOOD_LOWER_FF_VOLTS (less than the raise value, since gravity
            // helps here). Without this the 2 deg click could never descend: 2 deg of error is
            // 0.55 V from P alone.
            if (errorDeg < 0) {
                double belowTolerance = -errorDeg - ShooterSubsystemConstants.ANGLE_TOLERANCE;
                double fraction = MathUtil.clamp(
                    belowTolerance / ShooterSubsystemConstants.HOOD_FF_FADE_DEG, 0, 1);
                return -fraction * ShooterSubsystemConstants.HOOD_LOWER_FF_VOLTS;
            }
            double aboveTolerance = errorDeg - ShooterSubsystemConstants.ANGLE_TOLERANCE;
            double fraction = MathUtil.clamp(aboveTolerance / ShooterSubsystemConstants.HOOD_FF_FADE_DEG, 0, 1);
            return fraction * ShooterSubsystemConstants.HOOD_RAISE_FF_VOLTS;
        }

        // A commanded UP voltage below breakaway heats the motor and moves nothing. Once the
        // loop has decided to drive (not holding) and the hood is BELOW its target, floor the
        // command at HOOD_BREAKAWAY_VOLTS so the stroke actually happens. Without this the
        // hood sags a full HOOD_UP_STEP_UNITS after every press before P + FF grow enough to
        // lift it -- the "it goes up then falls right back down" report of 2026-08-28.
        // Untouched: holding (0 V), the descent (negative), and the caps/travel guards applied
        // after this. Package-private + static for HoodSettleTest.
        static double hoodBreakawayFloor(double volts, double errorDeg, boolean holding){
            if (holding || errorDeg <= 0 || volts <= 0) {
                return volts;
            }
            return Math.max(volts, ShooterSubsystemConstants.HOOD_BREAKAWAY_VOLTS);
        }

        // RB AUTO-AIM (team request 2026-08-29): distance (m) -> hood position in RAW UNITS
        // ABOVE THE ENABLE DATUM, linearly interpolated. InterpolatingDoubleTreeMap CLAMPS
        // outside the table rather than extrapolating, which is what we want -- an
        // extrapolated hood angle at an untabled distance is a guess that can reach the top
        // stop. Package-private + static for AutoAimTest.
        static InterpolatingDoubleTreeMap buildAutoAimHoodTable(){
            InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
            for (double[] row : ShooterSubsystemConstants.AUTOAIM_HOOD_TABLE) {
                table.put(row[0], row[1]);
            }
            return table;
        }
        private final InterpolatingDoubleTreeMap autoAimHoodTable = buildAutoAimHoodTable();

        // TEST-ONLY tag alias: treat the one tag the team actually has (17) as a scoring tag,
        // so it behaves like tag 10 (both are CENTERED on their hub face, so the camera-space
        // geometry already matches -- only the classification differs). Applied inside
        // classifyTag, i.e. in ONE place, so the pose caching in periodic(), the align bearing
        // and auto-aim all follow the same path. Package-private + static for AutoAimTest.
        // TODO set AUTOAIM_TEST_TAG_ALIAS_ENABLED false before competition.
        static boolean isAliasedScoreTag(int tagId){
            return ShooterSubsystemConstants.AUTOAIM_TEST_TAG_ALIAS_ENABLED
                && tagId == ShooterSubsystemConstants.AUTOAIM_TEST_TAG_ID;
        }

        // Horizontal distance (m) from the camera to the HUB CENTER, from the camera-space tag
        // pose: camera X = right, Z = forward, plus the fixed tag-face -> hub-center depth.
        // Deliberately NOT the botpose path that runVisionTargeting prefers: a test tag hung
        // somewhere other than its real field location yields a confident, badly wrong botpose,
        // while camera space measures the thing actually in front of the robot.
        // Package-private + static for AutoAimTest.
        static double autoAimDistanceMeters(double camX, double camZ, double lateralOffsetMeters){
            return Math.hypot(camX + lateralOffsetMeters,
                camZ + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS);
        }

        // Is auto-aim inside its tag-loss hold? True while a tag HAS been seen this hold and
        // the dropout is still shorter than AUTOAIM_TAG_HOLD_SEC -- in which case the whole
        // targeting pass is skipped, so the hood setpoint and the flywheel target keep their
        // last values instead of being refused and then re-commanded on reacquire.
        // Package-private + static for AutoAimTest.
        static boolean autoAimHoldingThroughDropout(boolean hadTag, double secSinceLastTag){
            return hadTag && secSinceLastTag < ShooterSubsystemConstants.AUTOAIM_TAG_HOLD_SEC;
        }

        // Bearing (deg, POSITIVE = the hub centre is to the camera's RIGHT) from the camera-space
        // tag pose, corrected to the HUB CENTRE rather than the tag itself: the lateral term
        // steps across the hub face, the depth term steps back to the centre. Both come from the
        // real 2026 tag layout (FieldConstants) -- the tags sit off to one side of the face, so
        // aiming at the tag aims off-centre by up to 0.3556 m -- 6.8 deg at 3 m from the hub
        // centre, still 4.1 deg at 5 m. That is more than twice HEADING_TOLERANCE_DEG at any
        // legal scoring range, so it is the difference between centred and clipping the rim. Package-private + static for AutoAimTest.
        static double autoAimBearingDegrees(double camX, double camZ, double lateralOffsetMeters){
            return Math.toDegrees(Math.atan2(camX + lateralOffsetMeters,
                camZ + FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS));
        }

        // WHICH VISIBLE TAG AUTO-AIM SHOULD USE (team direction 2026-08-29: "10 or 26 should be
        // prioritised over 25 or 9"). The Limelight's own primary target is whatever is biggest
        // or nearest the crosshair, which is not what we want: a CENTRED tag (10, 26) sits on the
        // hub's centreline, so aiming at it needs no lateral correction at all, while a SIDE tag
        // (9, 25) needs the 0.3556 m across-face step -- and that correction assumes the robot is
        // square to the face, which it usually is not. Every bit of that assumption is error we
        // can simply decline to take on when a centred tag is also in frame.
        //
        // Order: legal SCORE tags only, centred before offset, then biggest area (nearest /
        // most reliable solve) as the tie-break. Returns null when nothing qualifies.
        // Package-private + static for AutoAimTest.
        static LimelightHelpers.RawFiducial pickAutoAimTag(
                LimelightHelpers.RawFiducial[] fiducials, java.util.Set<Integer> ownScoreTags){
            if (fiducials == null) {
                return null;
            }
            LimelightHelpers.RawFiducial best = null;
            int bestRank = Integer.MAX_VALUE;
            for (LimelightHelpers.RawFiducial f : fiducials) {
                if (f == null) {
                    continue;
                }
                boolean legal = isAliasedScoreTag(f.id) || ownScoreTags.contains(f.id);
                if (!legal) {
                    continue;
                }
                // 0 = centred on its face (no lateral correction), 1 = offset to one side.
                int rank = FieldConstants.tagLateralOffsetMeters(f.id) == 0.0 ? 0 : 1;
                if (best == null || rank < bestRank || (rank == bestRank && f.ta > best.ta)) {
                    best = f;
                    bestRank = rank;
                }
            }
            return best;
        }

        // Camera-space GROUND point of a raw fiducial: bearing (txnc, +right) and range
        // (distToCamera) back to the (x, z) the aim helpers work in.
        //
        // THE VERTICAL DROP MUST COME OUT FIRST (fix 2026-08-29). distToCamera is a 3D range and
        // the tag on this robot sits well BELOW the camera -- tync runs -41 deg at 39 in, -19 deg
        // at 90 in. Feeding the raw 3D range in as if it were horizontal overstated the depth by
        // 33% up close, and since the bearing is atan2(across, depth), an inflated depth makes
        // every correction TOO SMALL: a side tag got 12.6 deg of turn where it needed 14.6.
        // Multiplying by cos(tync) projects onto the ground plane, which is also the plane the
        // team measures distances in with a tape.
        // Package-private + static for AutoAimTest.
        static double fiducialGroundRange(double tyncDeg, double distToCamera){
            return distToCamera * Math.cos(Math.toRadians(tyncDeg));
        }
        static double fiducialCamX(double txncDeg, double groundRange){
            return groundRange * Math.sin(Math.toRadians(txncDeg));
        }
        static double fiducialCamZ(double txncDeg, double groundRange){
            return groundRange * Math.cos(Math.toRadians(txncDeg));
        }

        // The hub centre in camera GROUND coordinates (x right, z forward), stepping off the
        // tag by its across-face offset and the fixed face-to-centre depth -- with both steps
        // rotated by phi, the angle the tag is being VIEWED at.
        //
        // phi = 0 reduces exactly to (camX + lateral, camZ + depth), which is what the code did
        // before and what is correct when square to the face. It is wrong off to one side,
        // because the depth step belongs along the TAG'S normal, not along the camera's axis:
        // the error is depth*sin(phi) across the aim line, 0.21 m at 20 deg.
        // Returns { x, z }. Package-private + static for AutoAimTest.
        static double[] hubPointInCameraFrame(double camXTag, double camZTag, double lateral,
                double phiDeg){
            double phi = Math.toRadians(phiDeg);
            double depth = FieldConstants.TAG_FACE_TO_HUB_DEPTH_METERS;
            return new double[] {
                camXTag + lateral * Math.cos(phi) + depth * Math.sin(phi),
                camZTag + depth * Math.cos(phi) - lateral * Math.sin(phi),
            };
        }

        // The angle the tag is being viewed at, or 0 when it cannot be trusted. Only the
        // Limelight's PRIMARY target publishes an orientation (targetpose_cameraspace describes
        // whichever tag the camera itself picked), so if auto-aim chose a different tag there is
        // no viewing angle for it and the square-on assumption stands -- correct at small angles,
        // and the tag-in-view clamp bounds it either way.
        // Package-private + static for AutoAimTest.
        static double autoAimViewAngleDegrees(int pickedTagId, int primaryTagId, double tagYawDeg){
            if (!ShooterSubsystemConstants.AUTOAIM_VIEW_ANGLE_COMP_ENABLED
                    || pickedTagId != primaryTagId) {
                return 0;
            }
            return tagYawDeg * ShooterSubsystemConstants.AUTOAIM_VIEW_ANGLE_SIGN;
        }

        // Clamp the commanded turn so the tag CANNOT leave the frame. After turning by the
        // returned bearing, a tag at txnc sits at (txnc - bearing) in the image, so bounding the
        // bearing to within AUTOAIM_MAX_TAG_OFFSET_DEG of txnc bounds where the tag ends up.
        // Package-private + static for AutoAimTest.
        static double clampTurnToKeepTagInView(double bearingDeg, double txncDeg){
            return MathUtil.clamp(bearingDeg,
                txncDeg - ShooterSubsystemConstants.AUTOAIM_MAX_TAG_OFFSET_DEG,
                txncDeg + ShooterSubsystemConstants.AUTOAIM_MAX_TAG_OFFSET_DEG);
        }

        // May the aim target be refreshed this loop? Only while the robot is turning slowly:
        // vision lags a few frames, so a bearing measured mid-turn belongs to a heading the
        // robot has already left. Package-private + static for AutoAimTest.
        static boolean autoAimTargetIsTrustworthy(double omegaDegPerSec){
            return Math.abs(omegaDegPerSec)
                <= ShooterSubsystemConstants.AUTOAIM_AIM_UPDATE_MAX_DEG_PER_SEC;
        }

        // Should auto-aim WRITE a new hood setpoint this loop? Only when the distance-derived
        // target has moved AUTOAIM_RETARGET_DEADBAND_UNITS from THE TARGET AUTO-AIM LAST ASKED
        // FOR. THIS GUARD IS LOAD-BEARING: setDesired_Angle clears hoodHolding (the park latch),
        // and an unparked hood below its target is driven at the ~7.5 V breakaway floor -- so
        // writing the setpoint every 20 ms loop re-starts the drive every loop.
        //
        // COMPARE AGAINST THE LAST COMMANDED TARGET, NEVER getDesiredAngle() (bug found on the
        // robot 2026-08-29 -- "the hood keeps going up and down"). desired_Angle does not stay
        // where auto-aim put it: when the hood ARRIVES, the latch in periodic() calls
        // setDesired_Angle(currentHoodAngle), re-seeding the setpoint to where the hood actually
        // LANDED -- and a landing is 25-55 units past what was asked, because that is one
        // powered loop of travel. So the next pass saw a 25-55 unit disagreement against a
        // perfectly stable distance, drove the hood back down to the table value, overshot
        // downward, got re-seeded low by the same latch, and drove back up. Forever, with the
        // tag sitting still. Remembering what WE asked for closes the loop: a stable distance
        // commands the hood exactly once.
        // Package-private + static for AutoAimTest.
        static boolean autoAimShouldRetarget(double newTarget, double lastCommandedTarget){
            return Math.abs(newTarget - lastCommandedTarget)
                > ShooterSubsystemConstants.AUTOAIM_RETARGET_DEADBAND_UNITS;
        }

        // Surface speed (m/s) equivalent to a motor speed in RPM. Same geometry the velocity
        // command uses in reverse: rev/s at the motor -> rev/s at the wheel -> rim speed.
        // Package-private + static for ShooterSpeedTrimTest.
        static double surfaceSpeedForMotorRpm(double rpm){
            return rpm / 60.0 * 2 * Math.PI * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
                * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION;
        }

        // Apply an RPM trim to an RT target, clamped to [0, the motor ceiling]. Zero is a legal
        // floor: it means "refuse", which the at-speed gate can never satisfy, so the kicker
        // stays shut. The ceiling is SHOT_MAX_MOTOR_RPS, the same limit the shot model refuses
        // above -- the DPAD must not be able to ask for a speed the motors cannot hold.
        // Package-private + static for ShooterSpeedTrimTest.
        static double trimSurfaceSpeed(double currentSurfaceSpeed, double rpmDelta){
            double ceiling = ShooterSubsystemConstants.SHOT_MAX_MOTOR_RPS * 2 * Math.PI
                * ShooterSubsystemConstants.FLYWHEEL_RADIUS_METERS
                * ShooterSubsystemConstants.FLYWHEEL_ROTATIONS_PER_MOTOR_ROTATION;
            return MathUtil.clamp(currentSurfaceSpeed + surfaceSpeedForMotorRpm(rpmDelta), 0, ceiling);
        }

        // Current RT flywheel target, m/s surface (dashboard + telemetry).
        public double getRtSurfaceSpeed(){
            return rtSurfaceSpeed;
        }

        // DPAD LEFT/RIGHT (press) = move the hood ON THE POSITION LOOP. Team direction
        // 2026-08-27 ("make the DPAD use PID"), completed 2026-08-28 ("voltage throttle should
        // be entirely removed"). A press only writes a SETPOINT; the closed-loop branch in
        // periodic() does all the driving, with its settle band, its gravity feedforward and its
        // asymmetric voltage caps. THERE IS NO OPEN-LOOP HOOD PATH LEFT ANYWHERE -- the held jog
        // and hoodJogVolts are deleted, in teleop and in test mode alike.
        //
        // RIGHT asks for current + HOOD_UP_STEP_UNITS, read off the hood at the press, so every
        // press steps again from wherever it actually ended up. LEFT asks for the enable-time
        // base. setDesired_Angle applies clampDesiredAngle, so neither can ask for a rise past
        // the ceiling or a descent below the floor.
        //
        // runOnce: the press is one setpoint write, and the loop owns everything after it.
        // Requires NO subsystem, deliberately -- the RT shot group runs kCancelIncoming and
        // would block a requiring command for the whole hold, killing the mid-hold hood trim.
        //
        // WHAT THIS DEPENDS ON, and what the old open-loop version did not:
        //  - NON-ZERO GAINS. The hood needs ~7.5 V to break away and hoodLiftFeedforward tops
        //    out at HOOD_RAISE_FF_VOLTS (7.0), deliberately just BELOW breakaway so the
        //    feedforward can never move the hood by itself -- P supplies the rest. kP is 0.18
        //    for exactly that reason (0.18 x a 10-unit step = 1.8 V, and 5.83 + 1.8 clears the
        //    7.5 V breakaway); drop it to 0 and a press writes a setpoint and nothing moves.
        //    See SHOOTER_ANGLE_kP for the full gain rationale.
        //  - NO MOVE TIMEOUT. A setpoint has no end, so a jammed hood is held against by the
        //    loop until the setpoint changes. What bounds it is the 20 A smart limit, the
        //    travel guards, and the driver pressing LEFT.
        //  - A LONG LANDING at this step size. One powered loop moves the hood 25-55 raw units
        //    and the step is 10, so the hood arrives past its target however it is driven; kD
        //    (0.004) cuts the drive early so it coasts in, and the settle band latches on
        //    whatever it lands on rather than chasing back down. If it still hunts on the robot,
        //    raise kD first, then widen ANGLE_TOLERANCE -- not kP.
        public Command moveHoodToCommand(DoubleSupplier targetDegrees){
            return Commands.runOnce(() -> setDesired_Angle(targetDegrees.getAsDouble()));
        }

        // Has the hood ARRIVED? Direction-aware: a rising move is done at or above its target, a
        // falling one at or below it. THE HOOD MUST STAY WHERE A PRESS PUT IT (team requirement
        // 2026-08-27), and the settle band alone cannot deliver that: the band is 3.75 raw units
        // wide and one powered 20 ms loop carries the hood 25-55 units, so a press lands well
        // OUTSIDE the band with a large negative error -- which the loop reads as "drive back
        // down", at up to HOOD_MAX_DOWN_VOLTAGE with gravity helping. It overshoots downward,
        // and that is the limit cycle. Latching on ARRIVAL instead of on the band closes it:
        // the moment the hood is at or past what was asked for, the setpoint is re-seeded to
        // where it actually landed and the brake parks it there.
        // Package-private + static for HoodMoveTest.
        static boolean hoodMoveReached(double currentDeg, double targetDeg, boolean rising){
            return rising ? currentDeg >= targetDeg : currentDeg <= targetDeg;
        }

        // The floor this enable. No longer a binding target (DPAD-LEFT steps by
        // HOOD_UP_STEP_UNITS since 2026-08-28); kept as the dashboard/telemetry read of where
        // clampDesiredAngle will stop a descent. See hoodFloorAngle.
        public double getHoodFloorAngle(){
            return hoodFloorAngle(hoodBaseAngleDeg, hoodDatumValid);
        }

        // Where the hood is right now, in raw encoder units. Public so DPAD-RIGHT can ask for a
        // step RELATIVE TO THE HOOD rather than to a stored setpoint: one press at 7 V lands
        // 25-55 units past the target (a powered loop is bigger than the step), and a target
        // that stepped from itself would sit BELOW the hood afterwards -- every later press
        // would be already-satisfied and do nothing, which is exactly the "clicks register and
        // the hood never moves" failure of 2026-08-22. Reading the hood each press is
        // self-correcting: the next target is always 5 above wherever it actually ended up.
        public double getHoodAngle(){
            return getShooterAngleDegrees();
        }

        // DPAD UP/DOWN (press) = trim the RT flywheel target by rpmDelta motor RPM (team request
        // 2026-08-22). Takes effect on the NEXT loop of a live shot too, since
        // flywheelOnlyShotCommand re-reads the field every loop. runOnce, so a press is one step.
        public Command trimRtSpeedCommand(double rpmDelta){
            return Commands.runOnce(() -> rtSurfaceSpeed = trimSurfaceSpeed(rtSurfaceSpeed, rpmDelta));
        }

        // Shortest-path difference between two raw readings, in raw units. A 359 -> 0.5 step
        // is +1.5, not -358.5. This is what makes the rollover a non-event: the tracker never
        // sees the discontinuity that the fixed-split interpretation turns into a 56 deg flip.
        // Package-private + static for HoodTrackingTest.
        static double shortestRawDelta(double previousRaw, double currentRaw){
            double delta = (currentRaw - previousRaw) % 360.0;
            if (delta > 180.0) {
                delta -= 360.0;
            } else if (delta < -180.0) {
                delta += 360.0;
            }
            return delta;
        }

        // Convert one loop's raw delta into degrees of hood travel, rejecting what is not
        // motion: noise below the deadband (so drift/jitter can never accumulate into phantom
        // travel -- the team's "slight changes in angle must not affect the hood") and glitches
        // above the max step (a garbled frame is not a 157 deg/sec hood).
        // Package-private + static for HoodTrackingTest.
        static double hoodDeltaDegrees(double rawDelta){
            double magnitude = Math.abs(rawDelta);
            if (magnitude < ShooterSubsystemConstants.HOOD_RAW_NOISE_DEADBAND
                    || magnitude > ShooterSubsystemConstants.HOOD_RAW_MAX_STEP) {
                return 0;
            }
            return rawDelta * ShooterSubsystemConstants.HOOD_DEG_PER_RAW_UNIT;
        }

        // Clamp a requested angle into what the hood can actually reach: never BELOW the
        // enable-time datum (hoodFloorAngle), and never more than HOOD_TRAVEL_WINDOW_DEG ABOVE
        // it. Wherever it was enabled, a setpoint can never ask for more travel than the
        // mechanism has, and never asks for a descent past where this enable started.
        //
        // THE SOFT BAND MAY NEVER DEMAND MOTION (2026-08-26, team request: "it should not move
        // at all when enabled"). The hood RESTS at ~3.2 deg, below the 5.0 deg MIN_ANGLE floor.
        // The old band clamped the enable-time seed (see captureHoodDatum) up to 5.0, so every
        // enable produced a 1.8 deg error -- past ANGLE_TOLERANCE and past HOOD_FF_FADE_DEG, so
        // the full lift feedforward fired and drove the hood UP at ~7.6 V the instant the robot
        // was enabled. Widening each limit to include the datum makes the seeded setpoint equal
        // the enable position EXACTLY, so the hood holds until something commands it.
        //
        // The band still only ever OPENS toward where the hood already is, so it cannot be used
        // to escape: enabled below MIN_ANGLE the hood can only be commanded UP, enabled above
        // MAX_ANGLE only DOWN. The hard travel guard in periodic() is what actually protects the
        // belt at the top, and it is unchanged.
        //
        // low <= baseAngleDeg <= high always holds now, so the limits can no longer cross and
        // the old degenerate-window branch is gone. Package-private + static for HoodTrackingTest.
        static double clampDesiredAngle(double angle, double baseAngleDeg, boolean datumValid){
            double low = hoodFloorAngle(baseAngleDeg, datumValid);
            double high = hoodCeilingAngle(baseAngleDeg, datumValid);
            return MathUtil.clamp(angle, low, Math.max(low, high));
        }

        // THE FLOOR THE HOOD MAY NEVER GO BELOW (team request 2026-08-27): the enable-time
        // datum itself. hoodBaseAngleDeg is re-captured from the raw encoder on every
        // disable -> enable edge (see captureHoodDatum), so the floor is wherever the hood
        // actually sat when this enable started -- not a fixed number, and not carried over
        // from the last session. Before any datum exists the fixed MIN_ANGLE soft limit stands
        // in, since there is nothing measured to floor against yet.
        //
        // This replaces the old base - HOOD_TRAVEL_WINDOW_DEG lower edge, which let the DPAD
        // walk the hood a whole window BELOW the enable position and into the bottom stop.
        // CONSEQUENCE, on purpose: enable the robot with the hood already raised and it cannot
        // be lowered past that point for the rest of the enable -- by the DPAD or by a
        // commanded shot angle. Enable with the hood resting (~3.2 deg), which is the normal
        // case, and nothing changes. The one exception is a hood enabled above MAX_ANGLE --
        // see the cap below. Used by BOTH clampDesiredAngle and the hard travel guard in
        // periodic(), so the open-loop DPAD drive is bounded by it too.
        // Package-private + static for HoodTrackingTest.
        static double hoodFloorAngle(double baseAngleDeg, boolean datumValid){
            // The MAX_ANGLE cap that used to sit here is GONE (2026-08-27). MAX_ANGLE is a
            // PHYSICAL degree (38) and the datum is now a RAW ENCODER reading (~311), so the
            // min() collapsed the floor to 38 -- the down-guard would never fire and DPAD-left
            // would drive the hood ~270 units into its bottom stop. The floor is the datum,
            // full stop; nothing on this scale can be compared against MAX_ANGLE any more.
            return datumValid ? baseAngleDeg : ShooterSubsystemConstants.MIN_ANGLE;
        }

        // THE CEILING: the enable-time datum plus the hood's full travel, in raw units. This
        // replaces every MAX_ANGLE comparison the hood used to make -- see hoodFloorAngle for
        // why a physical degree cannot bound a raw reading. Before a datum exists nothing
        // drives (the feedback gate holds 0 V), so MAX_ANGLE stands in as it always did.
        // Package-private + static for HoodTrackingTest.
        static double hoodCeilingAngle(double baseAngleDeg, boolean datumValid){
            return datumValid ? baseAngleDeg + ShooterSubsystemConstants.HOOD_TRAVEL_WINDOW_DEG
                              : ShooterSubsystemConstants.MAX_ANGLE;
        }

        // Live hood angle, in RAW ENCODER UNITS: the datum (the raw reading at enable) plus
        // everything travelled since, so this tracks the raw encoder 1:1. Before a datum exists
        // (first loops after boot, before any valid raw reading) there is nothing measured to
        // report; the physical bottom stop stands in, and the feedback gate holds the hood at
        // 0 V through those loops anyway.
        private double getShooterAngleDegrees(){
            if (hoodDatumValid) {
                return hoodBaseAngleDeg + hoodRelativeDeg;
            }
            return ShooterSubsystemConstants.HOOD_DEG_AT_FULL_DOWN;
        }

        // Capture a fresh datum HERE: this position becomes the reference every later reading
        // is measured against. Called on each disable -> enable transition, so a session's
        // accumulated encoder drift is discarded rather than carried into the next enable.
        // Accumulate one loop's travel, holding the tracked angle inside the PHYSICAL stops.
        // The hood cannot pass its stops, so an accumulation that would take the angle outside
        // them is slip or noise, not motion -- letting it run is what produced a reported
        // -32.96 deg on a hood whose travel is 3.2 to 44.5. Saturating instead of accumulating
        // also means the travel guard still knows which way is blocked.
        // Package-private + static for HoodTrackingTest.
        static double accumulateHoodTravel(double relativeDeg, double baseAngleDeg, double deltaDeg){
            // Saturate at the SAME two limits the guards use -- the enable-time floor and the
            // travel window above it. It used to saturate at the physical-degree anchors
            // (3.224 / 44.5); with the datum now being the raw encoder reading (~311) those
            // anchors are not on this scale at all, and the first loop would have pinned the
            // tracked angle at 44.5 forever.
            double angle = MathUtil.clamp(baseAngleDeg + relativeDeg + deltaDeg,
                hoodFloorAngle(baseAngleDeg, true), hoodCeilingAngle(baseAngleDeg, true));
            return angle - baseAngleDeg;
        }

        private void captureHoodDatum(double raw){
            // THE DATUM IS THE RAW ENCODER READING (2026-08-27, team request: "make the hood
            // angle the exact same as the raw encoder angle ... the raw encoder is checked so
            // raw encoder when enabled = base angle"). The tracked angle is therefore the raw
            // reading itself: base is raw at enable, and every later loop adds the raw delta
            // 1:1 (HOOD_DEG_PER_RAW_UNIT = 1.0). Nothing on the hood is in physical degrees any
            // more -- the bands and gains were converted once, in HOOD_UNITS_PER_DEG.
            //
            // Re-read on every disable -> enable edge, so an encoder re-zero between sessions
            // is a non-event: whatever it reads at enable IS the base, and the floor and
            // ceiling are placed around it.
            //
            // ponytail: the CEILING is still only as true as the assumption that the hood is
            // enabled resting on its bottom stop -- the window is the full stroke above the
            // datum. Enable with the hood ALREADY RAISED and the ceiling sits that much too
            // high, so a large commanded rise can reach the top stop. Drop the hood to its rest
            // before enabling. Upgrade path if that bites: a bottom limit switch, or a slow
            // find-the-stop homing move on the first enable.
            hoodBaseAngleDeg = raw;
            hoodRelativeDeg = 0;
            hoodLastRaw = raw;
            hoodDatumValid = true;
            // SEED THE SETPOINT TO WHERE THE HOOD ACTUALLY IS (2026-08-22). Without this the
            // setpoint keeps whatever stale value it had -- measured on the robot: the hood sat
            // at 34.84 deg with desired_Angle still 17 from an earlier session, so eight +2 deg
            // clicks only walked the target from 17 to 33, still BELOW the hood. Every click
            // registered and the hood never moved up, because the loop was trying to go DOWN
            // the whole time. Seeding here means a click is always 2 deg from where the hood is.
            setDesired_Angle(hoodBaseAngleDeg);
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
            // TEST-ONLY alias, checked FIRST so it wins over tag 17's normal FEED class.
            if (isAliasedScoreTag(tagId)) return TargetClass.SCORE;
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
            desired_Angle = clampDesiredAngle(angle, hoodBaseAngleDeg, hoodDatumValid);
            // Record the DIRECTION this setpoint asks for, against where the hood is right now.
            // Every setpoint write goes through here, so the arrival latch in periodic() always
            // has a fresh direction -- see hoodMoveReached.
            hoodMoveRising = desired_Angle > getShooterAngleDegrees();
            // A new setpoint is the ONLY thing that un-parks the hood (2026-08-28). The arrival
            // latch in periodic() calls this and re-latches immediately afterwards, in the same
            // loop, so the re-seed does not restart the drive.
            hoodHolding = false;
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

        // The field heading the RB aim hold servos to. With no live aim solution this returns
        // the CURRENT heading, so a lost tag makes the drivetrain hold still rather than chase a
        // stale bearing. Live value -- bind it as a method reference, never as a snapshot.
        public double getAutoAimFieldHeadingDegrees(){
            return aimHeadingValid ? autoAimServoHeadingDeg
                                   : drivetrain.getState().Pose.getRotation().getDegrees();
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

        // Kicker gate, SPEED ONLY (team request 2026-07-18: "kicker only starts after the
        // flywheels are up to speed"). True once the measured flywheel surface speed is within
        // SPEED_TOLERANCE of a NONZERO commanded speed -- a refused/absent shot commands 0 and
        // can never read at-speed, so this also keeps the kicker off stopped wheels. Deliberately
        // ignores hood angle + aim (unlike isReadyToShoot) so it still opens while the hood
        // encoder is down; tighten to isReadyToShoot once the hood feedback is trusted.
        // NOTE: ball contact briefly slows the wheels below tolerance, pausing the kicker until
        // they recover -- that paces fuel so every ball leaves at speed. If the kicker never
        // opens at all, SPEED_TOLERANCE (0.2 m/s) is likely too tight for the untuned velocity
        // loop -- widen it before blaming the gate.
        public boolean isFlywheelAtSpeed(){
            return velReached;
        }

        // Shot model core (SHOT MODEL block in ShooterSubsystemConstants has the derivation).
        // No-drag closed form for exit speed through (distance, deltaH) at the FIXED max hood
        // angle (38 deg; fits + envelope refit at 38 on 2026-07-21 -- docs/shot-model/
        // refit_38.py), times the distance-fitted drag multiplier, converted to flywheel
        // SURFACE speed (m/s). Returns 0 (= refuse: CoastOut path, at-speed gate never opens,
        // kicker never feeds) when the ball cannot cross the target height DESCENDING or the
        // motor velocity ceiling is exceeded.
        // Package-private for ShotModelTest (pins this against the drag integrator's truth).
        static double modelSurfaceSpeed(double distance, double deltaH, double dragMult){
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
        // so belts/kicker never feed). THE HOOD IS DELIBERATELY NOT TOUCHED (2026-08-26, team
        // request: no auto-stow anywhere). This used to take a hood argument and every caller
        // passed MIN_ANGLE, so a refused or released shot yanked the hood down and threw away
        // wherever the driver had jogged it.
        private void refuseShot(){
            hasShotTarget = false;
            movingCompMeters = 0;
            setDesiredFlywheelVelocity(0);
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
                refuseShot();
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
                    // Hood untouched -- this used to stage it to MAX, which is the robot moving
                    // the hood on its own (no auto motion, 2026-08-26).
                    target_distance = 0;
                    refuseShot();
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
                refuseShot();
                return;
            }
            // Degenerate-pose guard: a garbage botpose sitting ON the target point would divide
            // by ~0 above and float a NaN through every envelope gate (NaN comparisons are all
            // false) straight into VelocityVoltage. Refuse instead.
            if (!(distance > 0.01)) {   // also catches NaN
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot();
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
                // refuse. The hood used to be staged to MAX here; it is now left alone, since
                // nothing may move the hood without a driver command (2026-08-26).
                if (dEff < ShooterSubsystemConstants.MIN_SCORE_DISTANCE_METERS
                        || dEff > ShooterSubsystemConstants.MAX_SCORE_DISTANCE_METERS) {
                    refuseShot();
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
            // DEAD PATH, and it must stay dead until the units are fixed. MAX_ANGLE is the
            // PHYSICAL degree the shot model was fit at (38) while the hood setpoint is in raw
            // units, so clampDesiredAngle floors this at the enable-time base -- i.e. it would
            // command the hood all the way back DOWN to where it was enabled, undoing whatever
            // the driver set on the DPAD. Harmless today only because visionShotCommand (the
            // sole caller of runVisionTargeting) is not bound to anything: RT runs
            // flywheelOnlyShotCommand. Express the model's angle in hood units BEFORE binding
            // this, or auto-aim will fight the driver's hood every loop.
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
                // Leaving live-data mode: zero the VELOCITY ONCE here (this used to happen
                // every loop in periodic(), which also stomped preset shot commands). The hood
                // is left alone -- this line also drove it to MIN_ANGLE, and it wrote the field
                // directly, so it bypassed setDesired_Angle's clamp entirely (2026-08-26).
                if (!isEnabled) {
                    desired_Velocity = 0;
                }
            }
            );
        }

        // Coast the flywheels down. The HOOD IS LEFT WHERE IT IS (2026-08-26, team request: no
        // auto-stow anywhere) -- this used to drop it to MIN_ANGLE, throwing away whatever angle
        // the driver had jogged it to every time a shot was released.
        public Command stopShooterCommand(){
            return runOnce(() -> setDesiredFlywheelVelocity(0));
        }

        // RT (hold) = precision vision shot. Every loop: classify the primary tag (SCORE own
        // alliance / FEED / NONE), pick the distance source (botpose, else camera-space for
        // SCORE only), moving-comp, envelope gates, then hood MAX + model velocity. NONE or any
        // refusal keeps velocity 0 so belts/kicker never feed. Release: flywheels coast
        // (0 -> CoastOut), hood LEFT WHERE IT IS (no auto-stow, 2026-08-26), target flags clear.
        // (The old testSpinCommand/hoodJogCommand and their RT/DPAD bindings are superseded and
        // deleted per team spec 2026-07-16.)
        public Command visionShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();   // shot commands re-enable per team directive 2026-07-16
                    runVisionTargeting();
                },
                () -> {
                    refuseShot();
                    target_distance = 0;
                    aimHeadingValid = false;
                    heldClass = TargetClass.NONE; // next press must acquire a fresh tag
                })
                // Clear any pre-press residue so the spec holds exactly: pressing RT with no
                // tag found does nothing, even if one was seen moments before the press.
                .beforeStarting(() -> heldClass = TargetClass.NONE);
        }

        // RT (hold) = FLYWHEELS ONLY (team request 2026-08-22): spin to a fixed surface speed
        // and NEVER touch the hood -- the angle is whatever the DPAD jog left it at, which is
        // the whole point (manual range control). No vision, no model, no aim. The RT binding
        // runs belts always and gates the kicker on isFlywheelAtSpeed(). Release: velocity 0
        // (CoastOut), hood deliberately left where it is. (stopShooterCommand no longer moves
        // the hood either, so nothing can undo the driver's setting any more.)
        public Command flywheelOnlyShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();
                    // Every loop, so a DPAD trim mid-hold takes effect immediately.
                    setDesiredFlywheelVelocity(rtSurfaceSpeed);
                },
                () -> {
                    hasShotTarget = false;
                    setDesiredFlywheelVelocity(0);
                });
        }

        // Y (hold) = FIXED FEED SHOT (team request 2026-08-29): hood all the way up, flywheels
        // at a fixed RPM. Same principles as RT -- the binding runs the belts always and opens
        // the kicker only after the spin-up delay -- the only difference is that this one DOES
        // command the hood, once.
        //
        // ONCE, deliberately: the hood setpoint is written in beforeStarting, not in the loop
        // body. setDesired_Angle clears hoodHolding, so re-writing it every loop would un-park
        // the hood on every arrival and re-open the up/down limit cycle the arrival latch
        // exists to close (see periodic). The flywheel target IS re-written every loop, exactly
        // as flywheelOnlyShotCommand does it.
        //
        // The target is read off the hood floor at the press (getHoodFloorAngle = the
        // enable-time datum), so it means the same thing whatever the raw encoder happened to
        // read at enable. clampDesiredAngle and the hard travel guard still bound it.
        // Release: flywheels coast, hood stays where the shot put it (nothing lowers it).
        public Command feedShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();
                    setDesiredFlywheelVelocity(
                        surfaceSpeedForMotorRpm(ShooterSubsystemConstants.FEED_SHOT_MOTOR_RPM));
                },
                () -> {
                    hasShotTarget = false;
                    setDesiredFlywheelVelocity(0);
                })
                .beforeStarting(() -> setDesired_Angle(
                    getHoodFloorAngle() + ShooterSubsystemConstants.FEED_SHOT_HOOD_UNITS));
        }

        // RB (hold) = AUTO-AIM SHOT (team request 2026-08-29). CONSTANT flywheel speed
        // (AUTOAIM_FLYWHEEL_MOTOR_RPM); the ONLY thing that varies with distance is the hood
        // angle, looked up in AUTOAIM_HOOD_TABLE. No auto-rotate -- the driver still aims the
        // robot (A does search-align), this only sets the hood.
        //
        // THE HOOD MOVES WITHOUT A DRIVER PRESS HERE. That is a deliberate exception to the
        // 2026-08-26 "no automatic hood motion" rule, approved by the team on 2026-08-29 for
        // this binding only. Everything protecting the hood still applies: the setpoint goes
        // through setDesired_Angle (so clampDesiredAngle bounds it to [enable datum, datum +
        // travel]), the travel guards in periodic() still cut the volts at both ends, and the
        // retarget deadband below stops the loop from re-driving the hood every 20 ms.
        //
        // Release: flywheels coast, hood LEFT WHERE IT IS (no auto-stow, 2026-08-26).
        public Command autoAimShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();
                    runAutoAimTargeting();
                },
                () -> {
                    autoAimHadTag = false;
                    autoAimHasCommanded = false;
                    autoAimServoSeeded = false;
                    autoAimTagId = -1;
                    hasShotTarget = false;
                    target_distance = 0;
                    aimHeadingValid = false;
                    setDesiredFlywheelVelocity(0);
                })
                // Clear the ride-through before the first loop too: pressing RB with no tag in
                // view must do nothing, even if one was seen moments before the press.
                .beforeStarting(() -> {
                    autoAimHadTag = false;
                    autoAimServoSeeded = false;
                    // Every press starts fresh, so the first good frame always commands the
                    // hood -- the hood may have been moved by the DPAD since the last hold.
                    autoAimHasCommanded = false;
                });
        }

        // One pass per loop while autoAimShotCommand() is scheduled. periodic() has already
        // refreshed targetClass and tagCameraPose this loop (the scheduler runs subsystem
        // periodic() before command execute()).
        private void runAutoAimTargeting(){
            // No OWN scoring tag in view (the test alias makes 17 count).
            if (targetClass != TargetClass.SCORE) {
                // RIDE-THROUGH first (team request 2026-08-29, "some delay before going down if
                // a tag isn't seen"): for AUTOAIM_TAG_HOLD_SEC after the last good frame, do
                // NOTHING AT ALL -- returning here leaves the hood setpoint, the flywheel target
                // and hasShotTarget exactly as the last good frame left them. That is what stops
                // a one-frame flicker from turning into a hood move, since it is the RE-COMMAND
                // on reacquire that lurches, not the dropout itself.
                if (autoAimHoldingThroughDropout(autoAimHadTag, autoAimTagTimer.get())) {
                    return;
                }
                // Hold expired: refuse -- velocity 0, so the at-speed gate never opens and the
                // kicker never feeds. The hood is left wherever it is, deliberately -- losing
                // the tag must not yank it down.
                autoAimHadTag = false;
                target_distance = 0;
                refuseShot();
                return;
            }
            // A good frame: this is what the hold counts from.
            autoAimHadTag = true;
            autoAimTagTimer.restart();
            // PICK THE TAG rather than taking the Limelight's primary: a centred tag (10, 26)
            // needs no lateral correction, so it is preferred over a side tag (9, 25) whenever
            // both are in frame. Geometry comes from that tag's own raw fiducial, which is the
            // only source that works for a NON-primary tag (targetpose_cameraspace always
            // describes the primary one).
            LimelightHelpers.RawFiducial picked = pickAutoAimTag(
                LimelightHelpers.getRawFiducials(Vision.CAM_LIMELIGHT),
                FieldConstants.ownScoreTags(ownAlliance()));
            if (picked == null || !(picked.distToCamera > 0.01)) {
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot();
                return;
            }
            autoAimTagId = picked.id;
            double groundRange = fiducialGroundRange(picked.tync, picked.distToCamera);
            double camX = fiducialCamX(picked.txnc, groundRange);
            double camZ = fiducialCamZ(picked.txnc, groundRange);
            double lateral = FieldConstants.tagLateralOffsetMeters(picked.id);
            // Rotation of the tag in camera space about the VERTICAL axis = how far off to one
            // side we are looking at it from. Camera space is x right / y DOWN / z forward, so
            // the vertical axis is y and Rotation3d.getY() is the angle wanted.
            autoAimViewAngleDeg = autoAimViewAngleDegrees(picked.id, visibleTagId,
                Math.toDegrees(tagCameraPose.getRotation().getY()));
            double[] hub = hubPointInCameraFrame(camX, camZ, lateral, autoAimViewAngleDeg);
            double distance = Math.hypot(hub[0], hub[1]);
            if (!(distance > 0.01)) {   // also catches NaN from a garbage solve
                target_distance = 0;
                aimHeadingValid = false;
                refuseShot();
                return;
            }
            target_distance = distance;
            // AUTO-ROTATE (team request 2026-08-29): the blue-origin field heading that puts the
            // SHOOTER on the hub CENTRE. Camera x is RIGHT-positive while field heading is
            // CCW-positive, so the bearing is SUBTRACTED from the current heading. Re-sampled
            // every loop, so the servo converges as the robot turns; consumed by the RB binding's
            // aim hold and by isAimedAtTarget(), which gates the kicker.
            double bearingDeg = clampTurnToKeepTagInView(
                Math.toDegrees(Math.atan2(hub[0], hub[1])), picked.txnc);
            autoAimBearingDeg = bearingDeg;
            double headingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
            // TWO headings, deliberately, and they must not be collapsed into one:
            //  - aimHeadingDeg is the TRUE absolute bearing to the hub centre. isAimedAtTarget()
            //    measures against it, so it is what reports the aim. Damping this would widen
            //    that report by 1/scale and it would read "aimed" while visibly off target.
            //  - autoAimServoHeadingDeg is what the drivetrain servo chases: the same absolute
            //    target, LOW-PASS FILTERED toward it.
            //
            // THE FILTER IS ON AN ABSOLUTE HEADING, and that is the whole point (overshoot fix,
            // 2026-08-29). It used to be (live heading - scaled bearing), rebuilt from scratch
            // every loop -- and the heading term updates instantly while the camera bearing lags
            // by a frame or two of vision latency. So as the robot turned, the target ran ahead
            // of it: the servo was chasing a goal that moved because the servo was moving. That
            // is a positive feedback loop, and it overshoots however small the gain is. Filtering
            // toward a target that stands still in the field frame removes the feedback entirely
            // -- the hub is not moving, so neither should the target be.
            // SIGN: PLUS, determined ON THE ROBOT (team observation 2026-08-29: "when the robot
            // needs to correct more to the right it goes left, and vice versa"). Minus is what
            // the conventions on paper say -- camera +x is right, WPILib field heading is
            // CCW-positive, so facing a target to the right should mean a SMALLER heading -- and
            // that is what this was. It turned the wrong way on the real robot, which also
            // explains the runaway: turning away from the target makes the bearing GROW, so it
            // drove round until the tag left frame. The hardware wins over the paper convention.
            //
            // WHAT IS ACTUALLY INVERTED IS NOT ESTABLISHED -- candidates are the Pigeon's yaw
            // sense against WPILib's CCW-positive convention, or the camera's x axis. Worth
            // finding, because whatever it is affects more than this line:
            //   - ShooterSubsystem.degreesToAlignToTarget carries the SAME latent sign
            //     (atan2(-camX, camZ), callers ADD it), so searchAndAlignCommand will turn the
            //     wrong way too if it is ever re-bound. It is currently unbound, so it is not
            //     wrong today -- but do not restore it without re-checking this.
            //   - if it is the gyro, FIELD-CENTRIC DRIVING is mirrored as well. Worth a check:
            //     push the stick left and see whether the robot goes left.
            aimHeadingDeg = headingDeg + bearingDeg;
            // HOLD THE TARGET WHILE TURNING FAST. This is the fix for the robot driving itself
            // round until the tag left frame (2026-08-29). aimHeadingDeg is only a true absolute
            // heading if the bearing and the heading were sampled at the SAME instant -- and they
            // are not: the heading is live while the camera is a few frames behind. Mid-turn,
            // subtracting a stale bearing from a fresh heading produces a target that advances
            // WITH the robot, so the robot chases it round. Refreshing only while nearly still
            // breaks that loop; the target is then a fixed field heading, which is what the hub
            // actually is.
            double omegaDegPerSec =
                Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);
            if (!autoAimServoSeeded) {
                autoAimServoHeadingDeg = aimHeadingDeg;
                autoAimServoSeeded = true;
            } else if (autoAimTargetIsTrustworthy(omegaDegPerSec)) {
                // Wrapped error, so a target either side of +-180 takes the short way round.
                double err = MathUtil.inputModulus(aimHeadingDeg - autoAimServoHeadingDeg, -180, 180);
                autoAimServoHeadingDeg = MathUtil.inputModulus(
                    autoAimServoHeadingDeg + ShooterSubsystemConstants.AUTOAIM_ROTATION_SCALE * err,
                    -180, 180);
            }
            aimHeadingValid = true;
            // Table value is raw units ABOVE THE ENABLE DATUM, so anchor it to the datum.
            // getHoodFloorAngle() IS that datum (and the floor clampDesiredAngle enforces).
            // INHERITED ASSUMPTION: the datum is only the hood's rest position if the robot was
            // enabled with the hood resting down -- see captureHoodDatum. Enable with the hood
            // already raised and every angle here is biased by the same amount.
            // Table lookup, then the flat AUTOAIM_HOOD_SUBTRACT_UNITS bias (team direction
            // 2026-08-29) -- applied to EVERY auto-aim angle, at every distance, on every tag.
            // The bias can only FLATTEN toward the datum, never past it (team direction
            // 2026-08-30): a bias bigger than the tabled angle would otherwise ask for a
            // negative rise. setDesired_Angle clamps to the floor too, but clamping here is
            // what keeps autoAimLastTarget equal to the angle actually commanded -- otherwise
            // the retarget deadband compares against a target the hood was never sent to.
            double hoodRise = Math.max(0.0, autoAimHoodTable.get(distance)
                - ShooterSubsystemConstants.AUTOAIM_HOOD_SUBTRACT_UNITS);
            double hoodTarget = getHoodFloorAngle() + hoodRise;
            // First command of the hold always goes through; after that only a real change does.
            if (!autoAimHasCommanded || autoAimShouldRetarget(hoodTarget, autoAimLastTarget)) {
                setDesired_Angle(hoodTarget);
                autoAimLastTarget = hoodTarget;
                autoAimHasCommanded = true;
            }
            setDesiredFlywheelVelocity(
                surfaceSpeedForMotorRpm(ShooterSubsystemConstants.AUTOAIM_FLYWHEEL_MOTOR_RPM));
            hasShotTarget = true;
        }

        // RB (hold) = fixed blind feed: fixed hood angle + fixed tunable feed surface speed.
        // The RB binding runs belts always and gates the kicker on isFlywheelAtSpeed() (team
        // request 2026-07-18) -- fuel feeds only once the wheels actually reach the commanded
        // speed, never into wheels still spinning up. Release: flywheels coast, hood unchanged.
        public Command feedAngleShotCommand(){
            return runEnd(
                () -> {
                    enableSubsystem();
                    setDesired_Angle(ShooterSubsystemConstants.RB_FEED_ANGLE);
                    setDesiredFlywheelVelocity(ShooterSubsystemConstants.RB_FEED_SURFACE_SPEED);
                },
                () -> refuseShot());
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
                // NO AUTOMATIC HOOD MOTION (team request 2026-08-26). The hood moves only when
                // something explicitly commands an angle -- in the match bindings that is the
                // DPAD jog and nothing else. Enabling re-datums to wherever the hood sits and
                // seeds the setpoint there, so the hood holds; the old hood-lower interlock and
                // the coast-down auto-recess it gated are both gone.
                double rawHood = shooterAngleEncoder.getPosition();
                // CONTINUOUS TRACKING. Datum on the disable -> enable edge (a session's drift is
                // discarded, never carried across enables), then accumulate shortest-path deltas.
                // Noise and glitches are filtered out in hoodDeltaDegrees, so a still hood cannot
                // drift its own angle reading.
                boolean nowEnabled = DriverStation.isEnabled();
                if (isHoodFeedbackValid(rawHood) && (!hoodDatumValid || (nowEnabled && !hoodWasEnabled))) {
                    captureHoodDatum(rawHood);
                } else if (hoodDatumValid && isHoodFeedbackValid(rawHood)) {
                    hoodRelativeDeg = accumulateHoodTravel(hoodRelativeDeg, hoodBaseAngleDeg,
                        hoodDeltaDegrees(shortestRawDelta(hoodLastRaw, rawHood)));
                    hoodLastRaw = rawHood;
                }
                hoodWasEnabled = nowEnabled;
                double currentHoodAngle = getShooterAngleDegrees();
                // SAME band as setDesired_Angle -- one definition, both callers. A raw
                // clamp(desired_Angle, MIN_ANGLE, MAX_ANGLE) here would re-introduce the
                // enable-time lurch even with the setter fixed, by pulling a hood parked below
                // the 5.0 deg floor back up to it every loop.
                double hoodTarget = clampDesiredAngle(desired_Angle, hoodBaseAngleDeg, hoodDatumValid);
                // The stow interlock that used to sit here (hold the hood up while the flywheels
                // coast down) is GONE with auto-stow itself (2026-08-26): nothing lowers the hood
                // on its own any more, so the only thing it could still block was a DPAD-down
                // press while the wheels spun -- which is exactly the one motion the driver is
                // supposed to always get.
                // FEEDBACK SANITY GATE (verify-review 2026-07-18): only close the loop on a raw
                // reading inside the calibrated band. An unplugged/silent absolute encoder reads 0,
                // which the two-point map turns into ~105 deg -- the PID would then drive DOWN at
                // 6 V into the bottom hard stop forever (the MIN guard never fires because the
                // mapped angle reads high, not low) and stall the fragile NEO 550 at its 20 A
                // limit. Out-of-band or NaN (comparisons fail) -> 0 V; brake idle holds the hood.
                boolean hoodFeedbackValid = isHoodFeedbackValid(rawHood);
                double hoodVolts = 0; // kill switch / invalid feedback -> 0 V (brake idle holds)
                if (HOOD_CLOSED_LOOP_ENABLED && hoodFeedbackValid) {
                    // SETTLE BAND (2026-08-22, team requirement: a DPAD click moves the hood
                    // 2 deg and it STAYS there). Once inside ANGLE_TOLERANCE the hood is driven
                    // with 0 V and the brake idle mode holds it; without this it HUNTS -- it
                    // breaks free at ~7.5 V, carries past the target, and the loop drives it
                    // back down with up to 6 V plus gravity, a limit cycle around the setpoint.
                    // The hysteresis band that used to re-engage this drive is GONE: it turned
                    // an unavoidable droop into a permanent up/down limit cycle.
                    // ARRIVAL LATCH: the move is over the moment the hood is at or past what
                    // was asked for, whatever the remaining error says. Re-seed the setpoint to
                    // where it actually landed so the overshoot is never chased back down.
                    // See hoodMoveReached for why the settle band cannot do this on its own.
                    if (!hoodHolding && hoodMoveReached(currentHoodAngle, hoodTarget, hoodMoveRising)) {
                        setDesired_Angle(currentHoodAngle);
                        hoodTarget = currentHoodAngle;
                        shooterAnglePID.reset();
                        hoodHolding = true;
                    }
                    // NO re-evaluation here any more. Once parked, only a new setpoint
                    // (setDesired_Angle) can un-park the hood -- see the deleted hoodShouldHold.
                    // RESET WHILE HELD (2026-08-28, needed now that kD is non-zero): the
                    // controller is not called at all inside the band, so its stored previous
                    // error goes stale. Without this reset the first loop after a press would
                    // differentiate against an error from seconds ago and fire a D spike into
                    // the hood. Resetting every held loop means every move starts from rest.
                    double anglePID;
                    if (hoodHolding) {
                        anglePID = 0;
                        shooterAnglePID.reset();
                    } else {
                        anglePID = shooterAnglePID.calculate(currentHoodAngle, hoodTarget);
                    }
                    // Gravity LIFT feedforward: the P term alone under-drives against gravity at
                    // modest errors (kP 0.18 x a 10-unit step is 1.8 V, far below the ~7.5 V
                    // breakaway), so add an
                    // up-bias while the hood is still BELOW its target. FADED IN across
                    // HOOD_FF_FADE_DEG rather than stepped at ANGLE_TOLERANCE (2026-08-22) -- the step
                    // made the DPAD jog lurch. Still 0 V at/inside the target, so the hood can never be
                    // driven PAST the target into the MAX_ANGLE belt-skip zone by this term, and still
                    // off while lowering (error <= 0), so the gravity-assisted descent stays gentle.
                    double liftFF = hoodHolding ? 0 : hoodLiftFeedforward(hoodTarget - currentHoodAngle);
                    // ASYMMETRIC speed cap (2026-07-18): raising fights gravity, lowering is gravity-
                    // assisted, so the UP stroke gets more voltage (HOOD_MAX_UP_VOLTAGE) than the DOWN
                    // stroke (HOOD_MAX_DOWN_VOLTAGE). A symmetric 6 V could not lift the hood at all
                    // (it sat fully down during the feed shot). Positive output raises the hood, so it
                    // is capped at +UP; negative lowers, capped at -DOWN.
                    hoodVolts = MathUtil.clamp(
                        hoodBreakawayFloor(anglePID + liftFF, hoodTarget - currentHoodAngle, hoodHolding),
                        -ShooterSubsystemConstants.HOOD_MAX_DOWN_VOLTAGE, ShooterSubsystemConstants.HOOD_MAX_UP_VOLTAGE);
                }
                // HARD travel guard (team request 2026-07-18: "by no means exceed the max angle"
                // -- the hood was over-extending and skipping the belt). Positive volts raise the
                // hood: once at/above MAX_ANGLE never drive UP, once at/below the enable-time floor
                // (hoodFloorAngle -- the datum, re-read from the raw encoder every enable) never
                // drive DOWN, whatever the PID or the DPAD jog asks. MAX_ANGLE sits ~6.5 deg below the
                // physical 44.5 stop, so a little coast after the cut never reaches the stop /
                // skips the belt. Outside the branches above so it bounds BOTH the closed loop
                // and the open-loop jog.
                if (currentHoodAngle >= hoodCeilingAngle(hoodBaseAngleDeg, hoodDatumValid) && hoodVolts > 0) {
                    hoodVolts = 0;
                }
                if (currentHoodAngle <= hoodFloorAngle(hoodBaseAngleDeg, hoodDatumValid) && hoodVolts < 0) {
                    hoodVolts = 0;
                }
                shooterAngle.setVoltage(hoodVolts);
                lastHoodVolts = hoodVolts;
            //At-speed / at-angle kicker gates -- instance state (the mutable statics in
            // ShooterSubsystemConstants are DELETED; the hopper polls isReadyToShoot()).
                velReached =
                    desired_Velocity != 0 ? Math.abs(desired_Velocity-getShooterFlywheelVelocity()) < ShooterSubsystemConstants.SPEED_TOLERANCE : false;
                angleReached =
                     desired_Angle != 0 ? Math.abs(desired_Angle-getShooterAngleDegrees()) < ShooterSubsystemConstants.ANGLE_TOLERANCE : false;
            //Data. The six gain entries are INPUTS ONLY -- they are seeded once at construction
            // from the constants and never written again. They used to be echoed here every
            // loop, a few lines above the block that READS them back, so a typed value survived
            // only if it landed in the gap between the echo and the read in the same loop:
            // dashboard tuning appeared to work, then silently reverted. Same fix the Desired
            // Velocity/Angle entries already got. The gains still revert to the constants on
            // reboot or redeploy -- write a keeper into SubsystemConstants.
                debugEntry.setBoolean(enableComp);

            //Physics Lab: live-data mode reads setpoints straight off Shuffleboard. No else --
            // zeroing on exit happens ONCE in enableLiveData(false); an every-loop else here
            // would stomp the shot commands (visionShotCommand/feedAngleShotCommand) every loop.
                if(enableComp){
                    desired_Velocity = desiredVelEntry.getDouble(0);
                    // Through the SETTER, not the field: the field write alone cannot clear
                    // hoodHolding, so a live-data hood move would be silently ignored.
                    setDesired_Angle(desiredAngleEntry.getDouble(0));
                }

            //PID + FF Tuning -- gains come from the driver-companion PID panel over NT (frc.robot.util.Tunable).
            // No-ops entirely when SubsystemConstants.TUNING_MODE is false.
                //Speed
                    // Every hasChanged() is evaluated into a local FIRST: || short-circuits, so
                    // OR-ing the calls inline would leave later gains unpolled and their change
                    // consumed-but-unapplied on the next pass.
                    boolean kPChanged = tuneSpeedKp.hasChanged();
                    boolean kIChanged = tuneSpeedKi.hasChanged();
                    boolean kDChanged = tuneSpeedKd.hasChanged();
                    boolean kSChanged = tuneSpeedKs.hasChanged();
                    boolean kVChanged = tuneSpeedKv.hasChanged();
                    boolean kAChanged = tuneSpeedKa.hasChanged();
                    if(kPChanged || kIChanged || kDChanged || kSChanged || kVChanged || kAChanged){
                        shooterVelConfigs.kP = tuneSpeedKp.get();
                        shooterVelConfigs.kI = tuneSpeedKi.get();
                        shooterVelConfigs.kD = tuneSpeedKd.get();
                        shooterVelConfigs.kS = tuneSpeedKs.get();
                        shooterVelConfigs.kV = tuneSpeedKv.get();
                        shooterVelConfigs.kA = tuneSpeedKa.get();
                        // Slot0Configs ONLY -- applying a whole TalonFXConfiguration here would
                        // reset the stator/supply limits and Coast neutral mode set in the ctor.
                        shooterA.getConfigurator().apply(shooterVelConfigs);
                    }
                //Angle
                    boolean angleKPChanged = tuneAngleKp.hasChanged();
                    boolean angleKIChanged = tuneAngleKi.hasChanged();
                    boolean angleKDChanged = tuneAngleKd.hasChanged();
                    if(angleKPChanged || angleKIChanged || angleKDChanged){
                        shooterAnglePID.setPID(tuneAngleKp.get(), tuneAngleKi.get(), tuneAngleKd.get());
                    }
            } else {
                // Subsystem disabled while the robot is still enabled: VelocityVoltage LATCHES on
                // the TalonFX, so just skipping the body would leave the flywheels spinning at the
                // last setpoint forever. Explicitly coast the flywheels and stop the hood (its
                // Brake idle mode holds the angle).
                shooterA.setControl(m_flywheelCoast);
                shooterAngle.setVoltage(0);
                lastHoodVolts = 0;
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
            currentRpmEntry.setDouble(getShooterFlywheelVelocity() / surfaceSpeedForMotorRpm(1.0));
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
            rawHoodEncoderEntry.setDouble(shooterAngleEncoder.getPosition());
            hoodVoltsEntry.setDouble(lastHoodVolts);
            hoodCurrentEntry.setDouble(shooterAngle.getOutputCurrent());
            hoodBaseAngleEntry.setDouble(hoodBaseAngleDeg);
            hoodRelativeEntry.setDouble(hoodRelativeDeg);
            rtSpeedEntry.setDouble(rtSurfaceSpeed);
            rtSpeedRpmEntry.setDouble(rtSurfaceSpeed / surfaceSpeedForMotorRpm(1.0));
            autoAimTagEntry.setDouble(autoAimTagId);
            autoAimBearingEntry.setDouble(autoAimBearingDeg);
            autoAimViewAngleEntry.setDouble(autoAimViewAngleDeg);
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
