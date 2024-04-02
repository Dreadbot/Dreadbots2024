package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOCAN implements IntakeIO {
    private CANSparkMax intakeMotor;
    private DigitalInput beamBreakSensor;

    public IntakeIOCAN() {
        this.intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
        this.beamBreakSensor = new DigitalInput(IntakeConstants.BEAM_BREAK_SENSOR);

        this.intakeMotor.setInverted(true);
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.hasNote = !beamBreakSensor.get();
        
        inputs.velocity = intakeMotor.getEncoder().getVelocity();
        inputs.voltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.current = intakeMotor.getOutputCurrent();
        inputs.temperature = intakeMotor.getMotorTemperature();
    }
}
