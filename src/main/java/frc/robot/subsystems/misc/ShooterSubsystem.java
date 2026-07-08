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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
            private final GenericEntry targetHeightEntry;
            private final GenericEntry subsystemStateEntry;
            private final GenericEntry visionStateEntry;
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
       private boolean enableVision;
       private double desired_Velocity;
       private double desired_Angle;
       private double target_distance;
       private double target_height;

       public ShooterSubsystem(boolean enableVision, CommandSwerveDrivetrain drivetrain){

            //Coniguring Motors
                shooterA.getConfigurator().apply(shooterVelConfigs);

                // Conservative-start current limits + neutral mode (Hardware-Data-Sheet sec.7). TODO tune on robot.
                // Flywheels: Kraken X60 - stator 40A / supply 25A, Coast (let them spin down freely).
                CurrentLimitsConfigs flywheelLimits = new CurrentLimitsConfigs();
                flywheelLimits.StatorCurrentLimit = 40;
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

                shooterB.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Aligned));
                shooterC.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Opposed));
                shooterD.setControl(new Follower(shooterA.getDeviceID(), MotorAlignmentValue.Opposed));

            //Initializing Drivetrain
                this.drivetrain = drivetrain;
                this.degreesToAlignToTarget = 0.0;
        
            //Initializing Tracker Variables
                enableSubsystem = true;
                this.enableVision = enableVision;
                desired_Velocity = 0.0;
                desired_Angle = 0.0;
                target_distance = 0.0;
                target_height = 0.0;

            //Initializing Shuffleboard Entries
                currentVelEntry = ShooterSubsystemTab.add("Current Velocity", 0.0).getEntry();
                desiredVelEntry = ShooterSubsystemTab.add("Desired Velocity", 0.0).getEntry();
                currentAngleEntry = ShooterSubsystemTab.add("Current Angle", 0.0).getEntry();
                desiredAngleEntry = ShooterSubsystemTab.add("Desired Angle", 0.0).getEntry();
                targetDistanceEntry = ShooterSubsystemTab.add("Target Distance", 0.0).getEntry();
                targetHeightEntry = ShooterSubsystemTab.add("Target Height", 0.0).getEntry();
                subsystemStateEntry = ShooterSubsystemTab.add("Subsystem State", true).getEntry();
                visionStateEntry = ShooterSubsystemTab.add("Vision State", false).getEntry();
                degreedToAlignToTargEntry = ShooterSubsystemTab.add("Degrees to Align to Target", 0.0).getEntry();
                shooterSpeed_kP = ShooterSubsystemTab.add("SHOOTER KP", shooterVelConfigs.kP).getEntry();
                shooterSpeed_kD = ShooterSubsystemTab.add("SHOOTER KD", shooterVelConfigs.kD).getEntry();
                shooterSpeed_kV = ShooterSubsystemTab.add("SHOOTER KV", shooterVelConfigs.kV).getEntry();
                shooterAngle_kP = ShooterSubsystemTab.add("SHOOTER ANGLE KP", shooterAnglePID.getP()).getEntry();
                shooterAngle_kI = ShooterSubsystemTab.add("SHOOTER ANGLE KI", shooterAnglePID.getI()).getEntry();
                shooterAngle_kD = ShooterSubsystemTab.add("SHOOTER ANGLE KD", shooterAnglePID.getD()).getEntry();
                desiredVelReachedEntry = ShooterSubsystemTab.add("Desired Velocity Reached", true).getEntry();
                desiredAngleReachedEntry = ShooterSubsystemTab.add("Desired Angle Reached", true).getEntry();
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

        // True if the tag is any HUB or TRENCH tag (either alliance) on the 2026 REBUILT field.
        // TODO filter to the CURRENT alliance's HUB once the shooting model (generate*) is real.
        private static boolean isHubOrTrenchTag(int tagId){
            for (int[] set : new int[][] {
                    ShooterSubsystemConstants.APRILTAG_RED_HUB_IDS,
                    ShooterSubsystemConstants.APRILTAG_BLUE_HUB_IDS,
                    ShooterSubsystemConstants.APRILTAG_RED_TRENCH_IDS,
                    ShooterSubsystemConstants.APRILTAG_BLUE_TRENCH_IDS }) {
                for (int id : set) {
                    if (id == tagId) return true;
                }
            }
            return false;
        }
    //Subsystem Methods
        public void enableSubsystem(){
            enableSubsystem = true;
        }

        public void disableSubsystem(){
            enableSubsystem = false;
        }

        public void enableVisionBasedScoring(){
            enableVision = true;
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
        public double getDegreesToAlignToTarget(){
            return degreesToAlignToTarget;
        }

        public double generateShooterFlywheelVelocity(double distance, double height){
            return 0;
        }

        public double generateAngle(double distance, double height){
            //Generate mathematical model based on distance and height to target, this is a placeholder and should be replaced with an actual model based on testing
            return 0.0; // Placeholder return value
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
            if(!enableVision){
                return Commands.runOnce(()->{
                        this.setDesired_Angle(angle);
                        this.setDesiredFlywheelVelocity(velocity);
                });
            }
            return Commands.none(); 
        }

        public Command enableLiveData(boolean isEnabled){
           return Commands.runOnce(
            ()->{
                this.enableComp = isEnabled;
            }
            );
        }

    @Override
    public void periodic(){
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
                double anglePID = shooterAnglePID.calculate(getShooterAngleDegrees(), MathUtil.clamp(desired_Angle, ShooterSubsystemConstants.MIN_ANGLE, ShooterSubsystemConstants.MAX_ANGLE));
                    shooterAngle.setVoltage(MathUtil.clamp(anglePID, -12, 12));
            //Set Global Constants
                ShooterSubsystemConstants.desiredVelReached = 
                    desired_Velocity != 0 ? Math.abs(desired_Velocity-getShooterFlywheelVelocity()) < ShooterSubsystemConstants.SPEED_TOLERANCE : false;
                ShooterSubsystemConstants.desiredAngleReached = 
                     desired_Angle != 0 ? Math.abs(desired_Angle-getShooterAngleDegrees()) < ShooterSubsystemConstants.ANGLE_TOLERANCE : false;
            //Vision
            if(enableVision){
                boolean foundTarget = false;
                // Cheap NetworkTables reads for the PRIMARY target only -- the old
                // getLatestResults() call deserialized the Limelight's full JSON dump with
                // Jackson every 20 ms loop, which stalls the whole robot loop.
                if (LimelightHelpers.getTV(Vision.CAM_LIMELIGHT)) {
                    int tagId = (int) LimelightHelpers.getFiducialID(Vision.CAM_LIMELIGHT);
                    if (isHubOrTrenchTag(tagId)) {
                        Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(Vision.CAM_LIMELIGHT);
                        // Limelight camera space: X = right, Y = DOWN, Z = forward (depth).
                        // Old code read X as distance, Z as height, and computed the bearing in
                        // the vertical plane. TODO verify signs/axes on the robot with a real tag.
                        target_distance = Math.abs(targetPose.getZ() / Vision.LL_Z_MULTIPLIER);
                        target_height = Math.abs(targetPose.getY() / Vision.LL_Y__MULTIPLIER);
                        degreesToAlignToTarget = Math.toDegrees(Math.atan2(
                            targetPose.getX() / Vision.LL_X_MULTIPLIER,
                            targetPose.getZ() / Vision.LL_Z_MULTIPLIER));
                        foundTarget = true;
                    }
                }
                if(foundTarget){
                  setDesiredFlywheelVelocity(generateShooterFlywheelVelocity(target_distance, target_height));
                  setDesired_Angle(generateAngle(target_distance, target_height));
                } else {
                  setDesiredFlywheelVelocity(0);
                  setDesired_Angle(0 );
                }
            }
            //Data
                currentVelEntry.setDouble(getShooterFlywheelVelocity());
                // desiredVelEntry.setDouble(desired_VelocityRPS);
                currentAngleEntry.setDouble(getShooterAngleDegrees());
                // desiredAngleEntry.setDouble(desired_Angle);
                targetDistanceEntry.setDouble(target_distance);
                targetHeightEntry.setDouble(target_height);
                subsystemStateEntry.setBoolean(enableSubsystem);
                visionStateEntry.setBoolean(enableVision);
                degreedToAlignToTargEntry.setDouble(degreesToAlignToTarget);
                shooterSpeed_kP.setDouble(shooterVelConfigs.kP);
                shooterSpeed_kD.setDouble(shooterVelConfigs.kD);
                shooterSpeed_kV.setDouble(shooterVelConfigs.kV);
                shooterAngle_kP.setDouble(shooterAnglePID.getP());
                shooterAngle_kI.setDouble(shooterAnglePID.getI());
                shooterAngle_kD.setDouble(shooterAnglePID.getD());
                desiredVelReachedEntry.setBoolean(ShooterSubsystemConstants.desiredVelReached);
                desiredAngleReachedEntry.setBoolean(ShooterSubsystemConstants.desiredAngleReached);
                debugEntry.setBoolean(enableComp);
            
            //Physics Lab;
                if(enableComp){
                    desired_Velocity = desiredVelEntry.getDouble(0);
                    desired_Angle = desiredAngleEntry.getDouble(0);
                } else if(!enableComp) {
                    desired_Velocity = 0;
                    desired_Angle = ShooterSubsystemConstants.MIN_ANGLE;
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
            }
    }
}
