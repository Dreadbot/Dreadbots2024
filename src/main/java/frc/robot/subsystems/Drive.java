package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import util.math.DreadbotMath;
import util.misc.DreadbotSubsystem;
import util.misc.OnceCell;
import util.misc.SwerveModule;
import util.misc.VisionIntegration;
import util.misc.WaypointHelper;

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
    private final NetworkTable table;
    private final NetworkTable smartDashboard;

    private DoubleSubscriber poseX;
    private DoubleSubscriber poseY;
    private DoubleSubscriber poseLatency;
    private BooleanSubscriber tagSeen;
    private StructPublisher<Pose2d> posePub;
    private StructPublisher<Pose2d> visionPosePub;
    private StructArrayPublisher<SwerveModuleState> swervePub;
    private StructArrayPublisher<SwerveModuleState> swerveOptimPub;
    private StructPublisher<Rotation2d> gyroPub;

    private AHRS gyro = new AHRS(Port.kMXP);

    public Boolean autoAimArm = false;
    public double initialDistanceToTag = 0.0;

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(DriveConstants.SLEW , -DriveConstants.SLEW, 0);
    private SlewRateLimiter strafeSlewRateLimiter = new SlewRateLimiter(DriveConstants.SLEW, -DriveConstants.SLEW, 0);
    private OnceCell<Double> networkTablesTimeOffset = new OnceCell<Double>();

    private PIDController turningController = new PIDController(0.017, 0.007, 0.004);
    private double targetAngle;

    private Field2d field2d;

    public Drive(NetworkTable table) {
        this.smartDashboard = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        this.posePub = this.smartDashboard.getStructTopic("Robot Pose2d", Pose2d.struct).publish();
        this.visionPosePub = this.smartDashboard.getStructTopic("Vision Pose2d", Pose2d.struct).publish();
        this.swervePub = this.smartDashboard.getStructArrayTopic("Robot Swerve", SwerveModuleState.struct).publish();
        this.swerveOptimPub = this.smartDashboard.getStructArrayTopic("Robot Optimised Swerve", SwerveModuleState.struct).publish();
        this.gyroPub = this.smartDashboard.getStructTopic("Robot Gyro", Rotation2d.struct).publish();

        this.table = table;
        this.poseX = table.getDoubleTopic("robotX").subscribe(0.0);
        this.poseY = table.getDoubleTopic("robotY").subscribe(0.0);
        this.poseLatency = table.getDoubleTopic("visionLatency").subscribe(0.0);
        this.tagSeen = table.getBooleanTopic("tagSeen").subscribe(false);
        
        gyro.reset();
        if(Constants.SubsystemConstants.DRIVE_ENABLED) {
            field2d = new Field2d();

            SmartDashboard.putData(field2d);
        
            frontLeftModule = new SwerveModule(
                new CANSparkMax(1, MotorType.kBrushless),
                new CANSparkMax(2, MotorType.kBrushless), 
                new CANcoder(9), 
                SwerveConstants.FRONT_LEFT_ENCODER_OFFSET
            );
            frontRightModule = new SwerveModule(
                new CANSparkMax(3, MotorType.kBrushless),
                new CANSparkMax(4, MotorType.kBrushless), 
                new CANcoder(10), 
                SwerveConstants.FRONT_RIGHT_ENCODER_OFFSET
            );
            backRightModule = new SwerveModule(
                new CANSparkMax(5, MotorType.kBrushless),
                new CANSparkMax(6, MotorType.kBrushless), 
                new CANcoder(11), 
                SwerveConstants.BACK_RIGHT_ENCODER_OFFSET
            );
            backLeftModule = new SwerveModule(
                new CANSparkMax(7, MotorType.kBrushless),
                new CANSparkMax(8, MotorType.kBrushless), 
                new CANcoder(12), 
                SwerveConstants.BACK_LEFT_ENCODER_OFFSET
            );
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
                    new PIDConstants(1.2, 0.1), //MAKE SURE TO CHANGE THIS FOR THIS YEAR BOT!!!! (THESE ARE LAST YEARS VALUES)
                    new PIDConstants(0.8, 0.1),
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
        //double gyroOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI : 0;
        // SmartDashboard.putNumber("gyroAngle", getGyroRotation().getRadians() - gyroOffset);
        SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
        double timestamp = (RobotController.getFPGATime() / 1_000_000.0) - poseLatency.get();
        SmartDashboard.putNumber("Timestamp", timestamp);
        if (tagSeen.get()) {
            Pose2d worldToRobot = VisionIntegration.worldToRobotFromWorldFrame(
                VisionIntegration.robotToWorldFrame(
                    poseX.get(),
                    poseY.get(),
                    getPoseRotation().rotateBy(Rotation2d.fromRadians(Math.PI)).getRadians()),
                4);
            worldToRobot = new Pose2d(worldToRobot.getTranslation(), getPoseRotation());
            this.visionPosePub.set(worldToRobot);
            poseEstimator.addVisionMeasurement(worldToRobot, timestamp);
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
        this.gyroPub.set(gyro.getRotation2d());
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
            
            deltaTheta = -Math.atan2(distToTagY, distToTagX) - Math.toRadians(gyro.getYaw());

            rot = Math.max(-1, Math.min(1, DreadbotMath.applyDeadbandToValue(deltaTheta,.1))) * DriveConstants.ROT_SPEED_LIMITER * -1;
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
        gyro.reset();
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
        return gyro.getRotation2d();
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


    private void resetModules() {
        frontLeftModule.resetEncoder();
        frontRightModule.resetEncoder();
        backLeftModule.resetEncoder();
        backRightModule.resetEncoder();

        resetOdometry(getPosition());
    }

    public AHRS getGyro() {
        return gyro;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void resetGyro(){
        gyro.reset();
        resetOdometry(getPosition());
    }
    public void resetPose() {
        gyro.reset();
        resetOdometry(new Pose2d(new Translation2d(15.25, 5.54), new Rotation2d(Units.degreesToRadians(180))));
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
