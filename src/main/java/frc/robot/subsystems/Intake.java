package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.misc.DreadbotSubsystem;

public class Intake extends DreadbotSubsystem { 

    private CANSparkMax intakeMotor;
    private DigitalInput beamBreakSensor;

    public Intake() { 
        if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
        beamBreakSensor = new DigitalInput(IntakeConstants.BEAM_BREAK_SENSOR);
        
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Note", !beamBreakSensor.get());
    }

    public void intake(double speed) {
        if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        intakeMotor.set(speed);
    }

    public boolean hasNote() {
        return !beamBreakSensor.get();
    }

    @Override
    public void close() throws Exception {
         if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
       intakeMotor.close();
    }

    @Override
    public void stopMotors() {
         if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        intakeMotor.stopMotor();
    }
}

