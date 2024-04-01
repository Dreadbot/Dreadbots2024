package util.misc;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;
    private CANcoder turningCanCoder;
    private PIDController turningPIDController = new PIDController(6.5, 0, 0);
    private double desiredSpeed;
    private double desiredAngle;
    public SwerveModuleState desiredState = new SwerveModuleState();
    
    public SwerveModule(CANSparkMax driveMotor, CANSparkMax turnMotor, CANcoder turningCanCoder, double canCoderOffset) {
        this.driveMotor = driveMotor;
        this.turningMotor = turnMotor;
        this.turningCanCoder = turningCanCoder;
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = -canCoderOffset;
        this.turningCanCoder.getConfigurator().apply(config);
        this.turningMotor.setInverted(true);
        this.driveMotor.setInverted(true);
        driveMotor.getPIDController().setP(0.2);
        driveMotor.getPIDController().setFF(0.23);
        this.driveMotor.getEncoder().setPositionConversionFactor(SwerveConstants.WHEEL_DIAMETER * Math.PI * SwerveConstants.DRIVE_GEAR_RATIO); //convert from revolutions to meters
        this.driveMotor.getEncoder().setVelocityConversionFactor((SwerveConstants.WHEEL_DIAMETER * Math.PI * SwerveConstants.DRIVE_GEAR_RATIO) / 60);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModule(CANSparkMax driveMotor, CANSparkMax turnMotor, CANcoder turningCanCoder, double canCoderOffset, double drivePOverride) {
        this(driveMotor, turnMotor, turningCanCoder, canCoderOffset);
        this.driveMotor.getPIDController().setP(drivePOverride);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(), 
            new Rotation2d(turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI)
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(), 
            new Rotation2d(turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI)
        );
    }

    public void resetEncoder() {
        driveMotor.getEncoder().setPosition(0);
    }

    public SwerveModuleState getOptimizedState() {
        return SwerveModuleState.optimize(getState(), new Rotation2d(turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
        this.desiredState = desiredState;
        desiredSpeed = optimizedState.speedMetersPerSecond;
        desiredAngle = optimizedState.angle.getDegrees();
        double turnOutput = turningPIDController.calculate(turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI, optimizedState.angle.getRadians());
        this.driveMotor.getPIDController().setReference(optimizedState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        turningMotor.setVoltage(turnOutput);
    }

    public void putValuesToSmartDashboard(String name) {
        SmartDashboard.putNumber(name + " CANCoder", turningCanCoder.getAbsolutePosition().getValueAsDouble());
    }

    public CANSparkMax getDriveMotor() {
        return this.driveMotor;
    }

    public CANSparkMax getTurnMotor() {
        return this.turningMotor;
    }

    public void close() throws Exception{
        driveMotor.close();
        turningMotor.close();
    }

    public  void stopMotors() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }
}