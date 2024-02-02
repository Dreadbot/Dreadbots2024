package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import util.misc.DreadbotSubsystem;


public class Arm extends DreadbotSubsystem {

    private CANSparkMax armMotorLeft;
    private CANSparkMax armMotorRight;
    private SparkPIDController leftPidController;
    private SparkPIDController rightPidController;

    public Arm() {
        if(!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        armMotorLeft = new CANSparkMax(13, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(14, MotorType.kBrushless);
        leftPidController = armMotorLeft.getPIDController();
        rightPidController = armMotorRight.getPIDController();

        leftPidController.setP(0.2);
        leftPidController.setI(0.0);
        leftPidController.setD(0.0);

        rightPidController.setP(0.2);
        rightPidController.setI(0.0);
        rightPidController.setD(0.0);
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

    public void moveToPosition(double target) {
        leftPidController.setReference(target, ControlType.kPosition);
        rightPidController.setReference(target, ControlType.kPosition);
    }
    
}