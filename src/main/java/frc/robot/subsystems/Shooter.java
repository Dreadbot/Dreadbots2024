package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import util.misc.DreadbotSubsystem;

public class Shooter extends DreadbotSubsystem {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    public Shooter() {
        this.leftMotor = new CANSparkMax(16, MotorType.kBrushless);
        this.rightMotor = new CANSparkMax(17, MotorType.kBrushless);

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
