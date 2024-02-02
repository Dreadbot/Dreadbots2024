package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import util.misc.DreadbotSubsystem;

public class Intake extends DreadbotSubsystem { 
    private CANSparkMax intakeMotor;



    public Intake() { 
        if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        intakeMotor = new CANSparkMax(3, MotorType.kBrushless);
    }

    public void intake(double speed) {
        if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        intakeMotor.set(speed);
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

