package util.misc;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private DreadbotMotor driveMotor;
    private DreadbotMotor turningMotor;
    private CANcoder turningCanCoder;
    private PIDController turningPIDController = new PIDController(6.5, 0, 0);
    
    public SwerveModule(DreadbotMotor driveMotor, DreadbotMotor turnMotor, CANcoder turningCanCoder, double canCoderOffset) {
        this.driveMotor = driveMotor;
        this.turningMotor = turnMotor;
        this.turningCanCoder = turningCanCoder;
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = -canCoderOffset;
        this.turningCanCoder.getConfigurator().apply(config);
        this.turningMotor.setInverted(true);
        driveMotor.getPIDController().setP(0.1);
        driveMotor.getPIDController().setFF(1);
        this.driveMotor.getEncoder().setPositionConversionFactor(SwerveConstants.WHEEL_DIAMETER * Math.PI * SwerveConstants.DRIVE_GEAR_RATIO); //convert from revolutions to meters
        this.driveMotor.getEncoder().setVelocityConversionFactor((SwerveConstants.WHEEL_DIAMETER * Math.PI * SwerveConstants.DRIVE_GEAR_RATIO) / 60);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModule(DreadbotMotor driveMotor, DreadbotMotor turnMotor, CANcoder turningCanCoder, double canCoderOffset, double drivePOverride) {
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
        driveMotor.resetEncoder();
    }

    public void zeroModule() {
        driveMotor.getEncoder().setPosition(0);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));

        double turnOutput = turningPIDController.calculate(turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI, optimizedState.angle.getRadians());
        this.driveMotor.getPIDController().setReference(optimizedState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        turningMotor.setVoltage(turnOutput);
    }

    public void putValuesToSmartDashboard(String name) {
        SmartDashboard.putNumber(name +" Can Coder", turningCanCoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public DreadbotMotor getDriveMotor() {
        return this.driveMotor;
    }

    public DreadbotMotor getTurnMotor() {
        return this.turningMotor;
    }

    public void close() throws Exception{
        driveMotor.close();
        turningMotor.close();
    }
}