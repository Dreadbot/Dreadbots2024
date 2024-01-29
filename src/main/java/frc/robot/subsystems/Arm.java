package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import util.misc.DreadbotSubsystem;


public class Arm extends DreadbotSubsystem {

    private final CANSparkMax armMotorLeft;
    private final CANSparkMax armMotorRight;

    public Arm() {

        armMotorLeft = new CANSparkMax(13, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(14, MotorType.kBrushless);

    }

    @Override
    public void close() throws Exception {
        armMotorLeft.close();
        armMotorRight.close();
    }

    @Override
    public void stopMotors() {
        armMotorLeft.stopMotor();
        armMotorRight.stopMotor();

    }
    
}
