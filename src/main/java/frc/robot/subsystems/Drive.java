package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutonomousConstants;
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
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveOdometry odometry;

    private AHRS gyro = new AHRS(Port.kMXP);

    public Boolean autoAimArm = false;
    public double initialDistanceToTag = 0.0;

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(3 , -3, 0);
    private SlewRateLimiter strafeSlewRateLimiter = new SlewRateLimiter(3, -3, 0);

    public Drive() {
        gyro.reset();
        if(Constants.SubsystemConstants.DRIVE_ENABLED) {

        
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
            

            kinematics = new SwerveDriveKinematics(
                frontLeftLocation,
                frontRightLocation,
                backLeftLocation,
                backRightLocation
            );
            //using SwerveDrivePoseEstimator because it allows us to combine vision with odometry measurements down the line
            poseEstimator = new SwerveDrivePoseEstimator(kinematics, 
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                    frontLeftModule.getPosition(),
                    frontRightModule.getPosition(),
                    backLeftModule.getPosition(),
                    backRightModule.getPosition()
                },
                PathPlannerAuto.getStaringPoseFromAutoFile("New Auto") // CHANGE THIS ON ACTUAL BOT!
            );
            odometry = new SwerveDriveOdometry(
                kinematics, 
                getGyroRotation(), 
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                }
            );
            AutoBuilder.configureHolonomic(
                this::getPosition, 
                this::resetOdometry,
                this::getSpeeds,
                this::followSpeeds,
                new HolonomicPathFollowerConfig(
                    new PIDConstants(2.6, 0.1), //MAKE SURE TO CHANGE THIS FOR THIS YEAR BOT!!!! (THESE ARE LAST YEARS VALUES)
                    new PIDConstants(2.6, 0.1),
                    AutonomousConstants.MAX_SPEED_METERS_PER_SECOND, // keep it slow for right now during testing
                    Units.inchesToMeters(30.0),
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
        poseEstimator.update(
            getGyroRotation(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );
        odometry.update(
            getGyroRotation(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );
        

        frontLeftModule.putValuesToSmartDashboard("Front Left");
        frontRightModule.putValuesToSmartDashboard("Front Right");
        backLeftModule.putValuesToSmartDashboard("Back Left");
        backRightModule.putValuesToSmartDashboard("Back Right");

    }

    // make sure to input speed, not percentage!!!!!
    //xSpeed is forward, ySpeed is strafe -- because of ChassisSpeeds
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if(!Constants.SubsystemConstants.DRIVE_ENABLED) {
          return;
        }
        xSpeed = strafeSlewRateLimiter.calculate(xSpeed);
        ySpeed = forwardSlewRateLimiter.calculate(ySpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("xSpeed", xSpeed);

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
            fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.ATTAINABLE_MAX_SPEED);
        setDesiredStates(swerveModuleStates);
    }

    public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
        if(!Constants.SubsystemConstants.DRIVE_ENABLED) {
          return;
        }
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public void resetOdometry(Pose2d pose) {
        if(!Constants.SubsystemConstants.DRIVE_ENABLED) {
          return;
        }
        poseEstimator.resetPosition(gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            pose
        );
    }

    private Pose2d getPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    private Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
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

    public void resetGyro(){
        gyro.reset();
        resetOdometry(getPosition());
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
        drive(0, 0, 0, false);
    }
    
}
