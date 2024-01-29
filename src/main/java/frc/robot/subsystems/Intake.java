package frc.robot.subystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import util.misc.DreadbotSubsystem;

public class Intake extends DreadbotSubsystem { 
    private final CANSparkMax intakeMotor;



    public Intake() { 
        intakeMotor = new CANSparkMax(15, MotorType.kBrushless);
    }

    public void intake(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void close() throws Exception {
       intakeMotor.close();
    }

    @Override
    public void stopMotors() {
        intakeMotor.stopMotor();
    }
}

