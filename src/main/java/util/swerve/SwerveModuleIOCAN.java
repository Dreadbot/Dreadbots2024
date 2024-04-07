package util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOCAN implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;
    private CANcoder turningCANCoder;

    public SwerveModuleIOCAN(CANSparkMax driveMotor, CANSparkMax turningMotor, CANcoder turningCANCoder, double canCoderOffset) {
        this.driveMotor = driveMotor;
        this.turningMotor = turningMotor;
        this.turningCANCoder = turningCANCoder;
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = -canCoderOffset;
        this.turningCANCoder.getConfigurator().apply(config);
        this.turningMotor.setInverted(true);
        this.driveMotor.setInverted(true);
        driveMotor.getPIDController().setP(0.2);
        driveMotor.getPIDController().setFF(0.23);
        this.driveMotor.getEncoder().setPositionConversionFactor(SwerveConstants.WHEEL_DIAMETER * Math.PI * SwerveConstants.DRIVE_GEAR_RATIO); //convert from revolutions to meters
        this.driveMotor.getEncoder().setVelocityConversionFactor((SwerveConstants.WHEEL_DIAMETER * Math.PI * SwerveConstants.DRIVE_GEAR_RATIO) / 60);
    }

    public SwerveModuleIOCAN(CANSparkMax driveMotor, CANSparkMax turningMotor, CANcoder turningCANCoder, double canCoderOffset, double drivePOverride) {
        this(driveMotor, turningMotor, turningCANCoder, canCoderOffset);
        driveMotor.getPIDController().setP(drivePOverride);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.velocity = driveMotor.getEncoder().getVelocity();
        inputs.rotation = new Rotation2d(turningCANCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);

        inputs.drivePosition = driveMotor.getEncoder().getPosition();
        inputs.driveVoltage = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrent = driveMotor.getOutputCurrent();
        inputs.driveTemperature = driveMotor.getMotorTemperature();

        inputs.turningVoltage = turningMotor.getAppliedOutput() * turningMotor.getBusVoltage();
        inputs.turningCurrent = turningMotor.getOutputCurrent();
        inputs.turningTemperature = turningMotor.getMotorTemperature();
    }

    @Override
    public void setDriveReference(double reference, ControlType controlType) {
        driveMotor.getPIDController().setReference(reference, controlType);
    }

    @Override
    public void setTurnVoltage(double voltage) {
        turningMotor.setVoltage(voltage);
    }

    @Override
    public void resetEncoder() {
        driveMotor.getEncoder().setPosition(0);
    }

    @Override
    public void close() {
        driveMotor.close();
        turningMotor.close();
    }

    @Override
    public void stopMotors() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }
}
