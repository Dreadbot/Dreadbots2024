package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import util.misc.DreadbotSubsystem;


public class Arm extends DreadbotSubsystem {

    private final CANSparkMax armMotorLeft;
    private final CANSparkMax armMotorRight;

    public Arm() {
        if(!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        armMotorLeft = new CANSparkMax(13, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(14, MotorType.kBrushless);

    }

    @Override
    public void close() throws Exception {
        if(!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        armMotorLeft.close();
        armMotorRight.close();
    }

    @Override
    public void stopMotors() {
        if(!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        armMotorLeft.stopMotor();
        armMotorRight.stopMotor();

    }
    
}
