package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.misc.DreadbotSubsystem;

public class Shooter extends DreadbotSubsystem {

    private CANSparkMax leaderMotor;
    private CANSparkMax followerMotor;
    private SparkPIDController pidController;
    private Solenoid angleSolenoid;

    public Shooter() {
         if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        this.leaderMotor = new CANSparkMax(16, MotorType.kBrushless);
        this.followerMotor = new CANSparkMax(17, MotorType.kBrushless);
        this.followerMotor.restoreFactoryDefaults();
        this.leaderMotor.restoreFactoryDefaults();
        this.leaderMotor.setInverted(false);
        this.followerMotor.follow(leaderMotor, true);

        this.angleSolenoid = new Solenoid(21, PneumaticsModuleType.REVPH, 8);
        
        pidController = leaderMotor.getPIDController();
        // this.leaderMotor.getEncoder().setPositionConversionFactor(1.0 / 3.0);
        // this.leaderMotor.getEncoder().setVelocityConversionFactor(1.0 / 3.0);

        // this.followerMotor.getEncoder().setPositionConversionFactor(1.0 / 3.0);
        // this.followerMotor.getEncoder().setVelocityConversionFactor(1.0 / 3.0);


        pidController.setP(0.0);
        pidController.setI(0.0);
        pidController.setD(0.00);
        pidController.setFF(0.00012);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Speed", leaderMotor.getEncoder().getVelocity());
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
        SmartDashboard.putNumber("Shooter Desired Speed", speed);
        // leaderMotor.set(speed);
        leaderMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }
    
    public void setSourcePickupPosition() {
        angleSolenoid.set(true);
    }
    public void setStowPosition() {
        angleSolenoid.set(false);
    }
    
}
