package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import util.gyro.GyroIO;
import util.gyro.GyroIOInputsAutoLogged;
import util.math.DreadbotMath;
import util.misc.DreadbotSubsystem;
import util.misc.VisionIntegration;
import util.misc.VisionPosition;
import util.misc.WaypointHelper;
import util.swerve.SwerveModule;
import util.swerve.SwerveModuleIOCAN;

public class Drive extends DreadbotSubsystem {
    
    // Location of wheel module positions
    private final Translation2d frontLeftLocation = new Translation2d(SwerveConstants.MODULE_X_OFFSET, SwerveConstants.MODULE_Y_OFFSET);
    private final Translation2d frontRightLocation = new Translation2d(SwerveConstants.MODULE_X_OFFSET, -SwerveConstants.MODULE_Y_OFFSET);
    private final Translation2d backLeftLocation = new Translation2d(-SwerveConstants.MODULE_X_OFFSET, SwerveConstants.MODULE_Y_OFFSET);
    private final Translation2d backRightLocation = new Translation2d(-SwerveConstants.MODULE_X_OFFSET, -SwerveConstants.MODULE_Y_OFFSET);

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    public boolean doLockon = false;
    public double deltaTheta = 0.0;
    public double aprilTagX;
    public double aprilTagZ;
    private final NetworkTable smartDashboard;

    private StructArraySubscriber<VisionPosition> visionPositions;
    private DoubleSubscriber poseLatency;
    private StructPublisher<Pose2d> posePub;
    private StructArrayPublisher<SwerveModuleState> swervePub;
    private StructArrayPublisher<SwerveModuleState> swerveOptimPub;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public Boolean autoAimArm = false;
    public double initialDistanceToTag = 0.0;

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(DriveConstants.SLEW , -DriveConstants.SLEW, 0);
    private SlewRateLimiter strafeSlewRateLimiter = new SlewRateLimiter(DriveConstants.SLEW, -DriveConstants.SLEW, 0);

    private PIDController turningController = new PIDController(0.017, 0.007, 0.004);

    private Field2d field2d;

    public Drive(NetworkTable table, GyroIO gyroIO) {
        this.smartDashboard = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        this.posePub = this.smartDashboard.getStructTopic("Robot Pose2d", Pose2d.struct).publish();
        this.swervePub = this.smartDashboard.getStructArrayTopic("Robot Swerve", SwerveModuleState.struct).publish();
        this.swerveOptimPub = this.smartDashboard.getStructArrayTopic("Robot Optimised Swerve", SwerveModuleState.struct).publish();

        this.visionPositions = table.getStructArrayTopic("visionPos", VisionPosition.struct).subscribe(new VisionPosition[]{}, PubSubOption.periodic(0.02));
        this.poseLatency = table.getDoubleTopic("visionLatency").subscribe(0.0, PubSubOption.periodic(0.02));
        
        this.gyroIO = gyroIO;

        this.gyroIO.reset();
        if(Constants.SubsystemConstants.DRIVE_ENABLED) {
            field2d = new Field2d();

            SmartDashboard.putData(field2d);
        
            frontLeftModule = new SwerveModule(new SwerveModuleIOCAN(
                new CANSparkMax(1, MotorType.kBrushless),
                new CANSparkMax(2, MotorType.kBrushless),
                new CANcoder(9), 
                SwerveConstants.FRONT_LEFT_ENCODER_OFFSET
            ));
            frontRightModule = new SwerveModule(new SwerveModuleIOCAN(
                new CANSparkMax(3, MotorType.kBrushless),
                new CANSparkMax(4, MotorType.kBrushless), 
                new CANcoder(10), 
                SwerveConstants.FRONT_RIGHT_ENCODER_OFFSET
            ));
            backRightModule = new SwerveModule(new SwerveModuleIOCAN(
                new CANSparkMax(5, MotorType.kBrushless),
                new CANSparkMax(6, MotorType.kBrushless), 
                new CANcoder(11), 
                SwerveConstants.BACK_RIGHT_ENCODER_OFFSET
            ));
            backLeftModule = new SwerveModule(new SwerveModuleIOCAN(
                new CANSparkMax(7, MotorType.kBrushless),
                new CANSparkMax(8, MotorType.kBrushless), 
                new CANcoder(12), 
                SwerveConstants.BACK_LEFT_ENCODER_OFFSET
            ));
            turningController.enableContinuousInput(-180, 180);
            kinematics = new SwerveDriveKinematics(
                frontLeftLocation,
                frontRightLocation,
                backLeftLocation,
                backRightLocation
            );
            //using SwerveDrivePoseEstimator because it allows us to combine vision with odometry measurements down the line
            poseEstimator = new SwerveDrivePoseEstimator(kinematics, 
                getGyroRotation(),
                new SwerveModulePosition[] {
                    frontLeftModule.getPosition(),
                    frontRightModule.getPosition(),
                    backLeftModule.getPosition(),
                    backRightModule.getPosition()
                },
                new Pose2d()
            );
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 0.0));
            AutoBuilder.configureHolonomic(
                this::getPosition, 
                this::resetOdometry,
                this::getSpeeds,
                this::followSpeeds,
                new HolonomicPathFollowerConfig(
                    new PIDConstants(2.2, 0.1), //MAKE SURE TO CHANGE THIS FOR THIS YEAR BOT!!!! (THESE ARE LAST YEARS VALUES)
                    new PIDConstants(2.2, 0.1),
                    AutonomousConstants.MAX_SPEED_METERS_PER_SECOND, // keep it slow for right now during testing
                    Math.hypot(SwerveConstants.MODULE_X_OFFSET, SwerveConstants.MODULE_Y_OFFSET),
                    new ReplanningConfig()
                ),
                () -> {
                    if(DriverStation.getAlliance().isPresent()) {
                        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        }
    }

    @Override
    public void periodic() {
        if(!Constants.SubsystemConstants.DRIVE_ENABLED) {
          return;
        }

        Logger.recordOutput("SpeakerPose", new Pose2d(WaypointHelper.getSpeakerPos(), new Rotation2d()));
        double timestamp = (RobotController.getFPGATime() / 1_000_000.0) - poseLatency.get();

        VisionPosition[] positions = visionPositions.get();
        if (positions.length > 0) {

            List<Pose2d> worldPositions = new ArrayList<Pose2d>();
            for (VisionPosition position : positions) {
                Pose2d worldToRobot = VisionIntegration.worldToRobotFromWorldFrame(VisionIntegration.robotToWorldFrame(position.x, position.y, getPoseRotation().rotateBy(Rotation2d.fromDegrees(180)).getRadians()), position.ID);
                worldToRobot = new Pose2d(worldToRobot.getTranslation(), getPoseRotation());
                worldPositions.add(worldToRobot);
            }
            double sumX = 0;
            double sumY = 0;
            double sumRot = 0;
            for (Pose2d pos : worldPositions) {
                sumX += pos.getX();
                sumY += pos.getY();
                sumRot += pos.getRotation().getRadians();
            }
            double avgX = sumX / positions.length;
            double avgY = sumY / positions.length;
            double avgRot = sumRot / positions.length;
            Pose2d averagedPose = new Pose2d(avgX, avgY, new Rotation2d(avgRot));
            double error = averagedPose.getTranslation().getDistance(getPoseEstimator().getEstimatedPosition().getTranslation());
            if(error < 5.0) {
                poseEstimator.addVisionMeasurement(averagedPose, timestamp);
            } else {
                System.out.println("The april tags get a bit quirky ;) error :" + error);
            }
            Logger.recordOutput("Drive/Pose/Vision", averagedPose);
            
            // poseEstimator.resetPosition(
            //     getGyroRotation(),
            //     new SwerveModulePosition[] {
            //         frontLeftModule.getPosition(),
            //         frontRightModule.getPosition(),
            //         backLeftModule.getPosition(),
            //         backRightModule.getPosition(),
            //     },
            //     worldToRobot
            // );
        }
        poseEstimator.updateWithTime(
            RobotController.getFPGATime() / 1_000_000.0,
            getGyroRotation(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition(),
            }
        );
        this.posePub.set(poseEstimator.getEstimatedPosition());
        this.swervePub.set(new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState(),
        });
        this.swerveOptimPub.set(new SwerveModuleState[] {
            frontLeftModule.getOptimizedState(),
            frontRightModule.getOptimizedState(),
            backLeftModule.getOptimizedState(),
            backRightModule.getOptimizedState(),
        });
        Logger.recordOutput("Drive/Pose/Estimated", poseEstimator.getEstimatedPosition());
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    // make sure to input speed, not percentage!!!!!
    // xSpeed is forward, ySpeed is strafe -- because of ChassisSpeeds
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if(!Constants.SubsystemConstants.DRIVE_ENABLED) {
          return;
        }

        if (doLockon) {
            double distToTagX = WaypointHelper.getSpeakerPos().getX() - poseEstimator.getEstimatedPosition().getX();
            double distToTagY = WaypointHelper.getSpeakerPos().getY() - poseEstimator.getEstimatedPosition().getY();
            
            double inputRotation = getPoseRotation().rotateBy(Rotation2d.fromDegrees(180)).getRadians();
            Logger.recordOutput("Input Rotation", Units.radiansToDegrees(inputRotation));

            double inputTarget = Math.atan2(distToTagY, distToTagX);
            deltaTheta = inputTarget - inputRotation; //get back of robot
            
            Logger.recordOutput("Target Rotation", Units.radiansToDegrees(inputTarget));
            Logger.recordOutput("Rotation Delta", Units.radiansToDegrees(deltaTheta));
            if (deltaTheta > Math.PI) {
                deltaTheta = deltaTheta - Math.PI * 2;
            } else if (deltaTheta < -Math.PI) {
                deltaTheta = Math.PI * 2 + deltaTheta;
            }

            if(Math.abs(deltaTheta) >= Math.PI) {
                System.out.println(deltaTheta);
            }

            rot = Math.max(-1, Math.min(1, DreadbotMath.applyDeadbandToValue(deltaTheta, .1))) * DriveConstants.ROT_SPEED_LIMITER;
        }
        // if (DriverStation.isTeleop()) {
        //     if (DreadbotMath.applyDeadbandToValue(rot, DriveConstants.DEADBAND) != 0) {
        //         targetAngle = gyro.getRotation2d().getDegrees();
        //     } else {
        //         rot = turningController.calculate(gyro.getRotation2d().getDegrees() % 360, targetAngle);
        //     }
        // }
      
        xSpeed = strafeSlewRateLimiter.calculate(xSpeed);
        ySpeed = forwardSlewRateLimiter.calculate(ySpeed);
        ChassisSpeeds desiredSpeeds = 
        ChassisSpeeds.discretize(fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroRotation()) : new ChassisSpeeds(xSpeed, ySpeed, rot), 0.02);
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.ATTAINABLE_MAX_SPEED);
        setDesiredStates(swerveModuleStates);
    }

    public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public void resetOdometry(Pose2d pose) {
        gyroIO.reset();
        if(!Constants.SubsystemConstants.DRIVE_ENABLED) {
          return;
        }
        poseEstimator.resetPosition(
            getGyroRotation(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            pose
        );
        
    }

    public Pose2d getPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    private Rotation2d getGyroRotation() {
        return gyroInputs.yaw;
    }

    private Rotation2d getPoseRotation() {
        return getPosition().getRotation();
    }

    private ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    private void followSpeeds(ChassisSpeeds speed) {
        ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(speed, 0.02); // not sure why this is here, but lets try it anyways? lol
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(targetSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.ATTAINABLE_MAX_SPEED);
        setDesiredStates(swerveModuleStates);
    }


    // private void resetModules() {
    //     frontLeftModule.resetEncoder();
    //     frontRightModule.resetEncoder();
    //     backLeftModule.resetEncoder();
    //     backRightModule.resetEncoder();

    //     resetOdometry(getPosition());
    // }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void resetGyro(){
        gyroIO.reset();
        resetOdometry(getPosition());
    }
    public void resetPose() {
        gyroIO.reset();
        resetOdometry(WaypointHelper.getResetPose());
    }

    public GyroIO getGyroIO() {
        return this.gyroIO;
    }

    public GyroIOInputsAutoLogged getGyroInputs() {
        return this.gyroInputs;
    }

    public SwerveDrivePoseEstimator getEstimator() {
        return this.poseEstimator;
    }

    @Override
    public void close() throws Exception {
        if(!Constants.SubsystemConstants.DRIVE_ENABLED) {
          return;
        }

        stopMotors();
        frontLeftModule.close();
        frontRightModule.close();
        backLeftModule.close();
        backLeftModule.close();

    }

    @Override
    public void stopMotors() {
        if(!Constants.SubsystemConstants.DRIVE_ENABLED) {
            return;
        }
        frontLeftModule.stopMotors();
        frontRightModule.stopMotors();
        backLeftModule.stopMotors();
        backRightModule.stopMotors();
    }
}
