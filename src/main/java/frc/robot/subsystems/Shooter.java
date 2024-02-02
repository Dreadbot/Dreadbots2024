package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import util.misc.DreadbotSubsystem;

public class Shooter extends DreadbotSubsystem {

    private CANSparkMax leaderMotor;
    private CANSparkMax followerMotor;
    private SparkPIDController pidController;

    public Shooter() {
         if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        this.leaderMotor = new CANSparkMax(1, MotorType.kBrushless);
        this.followerMotor = new CANSparkMax(2, MotorType.kBrushless);
        this.leaderMotor.setInverted(true);
        this.followerMotor.follow(leaderMotor, true);
        
        pidController = leaderMotor.getPIDController();
        pidController.setP(0.5);
        pidController.setI(0.0);
        pidController.setD(0.0);
    }

    @Override
    public void close() throws Exception {
        if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        leaderMotor.close();
        followerMotor.close();
    }

    @Override
    public void stopMotors() {
        if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        leaderMotor.stopMotor();
        
    }

    public void shoot(double speed) {
        if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        pidController.setReference(speed, ControlType.kVelocity);
    }
    

    
}
