package frc.robot.subsystems;

import javax.swing.text.StyleConstants.ColorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.ColorSensorConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.misc.DreadbotSubsystem;

public class Intake extends DreadbotSubsystem { 

    private CANSparkMax intakeMotor;

    public Intake() { 
        if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        intakeMotor = new CANSparkMax(15, MotorType.kBrushless);
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void intake(double speed) {
        if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        intakeMotor.set(speed);
    }

    public boolean hasNote() {
        return false;
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

