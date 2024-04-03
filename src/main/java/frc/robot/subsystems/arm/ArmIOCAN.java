package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ArmConstants;

public class ArmIOCAN implements ArmIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public ArmIOCAN() {
        this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(ArmConstants.ARM_DUTY_CYCLE_ENCODER));
        absoluteEncoder.setPositionOffset(ArmConstants.ARM_ENCODER_OFFSET);
        absoluteEncoder.setDistancePerRotation(ArmConstants.ARM_ENCODER_SCALE);

        this.leftMotor = new CANSparkMax(ArmConstants.ARM_LEFT_MOTOR, MotorType.kBrushless);
        this.rightMotor = new CANSparkMax(ArmConstants.ARM_RIGHT_MOTOR, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor, true);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.getEncoder().setPositionConversionFactor(ArmConstants.ARM_GEAR_RATIO);
        rightMotor.getEncoder().setPositionConversionFactor(ArmConstants.ARM_GEAR_RATIO);

        leftMotor.getEncoder().setVelocityConversionFactor(ArmConstants.ARM_GEAR_RATIO / 60); // rpm -> rps
        rightMotor.getEncoder().setVelocityConversionFactor(ArmConstants.ARM_GEAR_RATIO / 60); // rpm -> rps

        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false);
        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false);

        leftMotor.getEncoder().setPosition(absoluteEncoder.get());

        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.absolutePosition = absoluteEncoder.get();

        inputs.leftVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.leftTemperature = leftMotor.getMotorTemperature();

        inputs.rightVoltage = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
        inputs.rightCurrent = rightMotor.getOutputCurrent();
        inputs.rightTemperature = rightMotor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        leftMotor.setIdleMode(idleMode);
        rightMotor.setIdleMode(idleMode);
    }

    @Override
    public void close() throws Exception {
        leftMotor.close();
        rightMotor.close();
    }

    @Override
    public void stopMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
