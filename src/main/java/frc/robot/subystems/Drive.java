package frc.robot.subystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.Constants.SwerveConstants;
import util.misc.DreadbotSubsystem;
import util.misc.SwerveModule;

public class Drive extends DreadbotSubsystem {
    
    // Location of wheel module positions
    private final Translation2d frontLeftLocation = new Translation2d(SwerveConstants.MODULE_X_OFFSET, SwerveConstants.MODULE_Y_OFFSET);
    private final Translation2d frontRightLocation = new Translation2d(SwerveConstants.MODULE_X_OFFSET, -SwerveConstants.MODULE_Y_OFFSET);
    private final Translation2d backLeftLocation = new Translation2d(-SwerveConstants.MODULE_X_OFFSET, SwerveConstants.MODULE_Y_OFFSET);
    private final Translation2d backRightLocation = new Translation2d(-SwerveConstants.MODULE_X_OFFSET, -SwerveConstants.MODULE_Y_OFFSET);

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;

    private AHRS gyro = new AHRS(Port.kUSB1);

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(1.5, -1.5, .2);
    private SlewRateLimiter strafeSlewRateLimiter = new SlewRateLimiter(1.5, -1.5, .2);


    public Drive() {

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
        backLeftModule = new SwerveModule(
            new CANSparkMax(5, MotorType.kBrushless),
            new CANSparkMax(6, MotorType.kBrushless), 
            new CANcoder(11), 
            SwerveConstants.BACK_LEFT_ENCODER_OFFSET
        );
        backRightModule = new SwerveModule(
            new CANSparkMax(7, MotorType.kBrushless),
            new CANSparkMax(8, MotorType.kBrushless), 
            new CANcoder(12), 
            SwerveConstants.BACK_RIGHT_ENCODER_OFFSET
        );

        gyro.reset();

        kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        );

        //using SwerveDrivePoseEstimator because it allows us to combine vision with odometry measurements down the line
        odometry = new SwerveDrivePoseEstimator(kinematics, 
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            new Pose2d()
        );
    }

    @Override
    public void periodic() {

        odometry.update(
            getGyroRotation(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );
        
    }

    // make sure to input speed, not percentage!!!!!
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        xSpeed = forwardSlewRateLimiter.calculate(xSpeed);
        ySpeed = strafeSlewRateLimiter.calculate(ySpeed);

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
            fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.ATTAINABLE_MAX_SPEED);
        setDesiredState(swerveModuleStates);;
        
    }

    public void setDesiredState(SwerveModuleState[] swerveModuleStates) {

        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);

    }

    public void resetOdometry(Pose2d pose) {

        odometry.resetPosition(gyro.getRotation2d(),
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
        return odometry.getEstimatedPosition();
    }

    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    public void followSpeed(ChassisSpeeds speed) {

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speed);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.ATTAINABLE_MAX_SPEED);
        setDesiredState(swerveModuleStates);

    }

    public void resetModules() {

        frontLeftModule.resetEncoder();
        frontRightModule.resetEncoder();
        backLeftModule.resetEncoder();
        backRightModule.resetEncoder();

        resetOdometry(getPosition());
    }

    public void resetGyro(){
        gyro.reset();
        resetOdometry(getPosition());
    }

    @Override
    public void close() throws Exception {

        stopMotors();
        frontLeftModule.close();
        frontRightModule.close();
        backLeftModule.close();
        backLeftModule.close();

    }

    @Override
    public void stopMotors() {
        drive(0, 0, 0, false);
    }
    
}
