package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.misc.DreadbotSubsystem;

public class Shooter extends DreadbotSubsystem {

    private CANSparkMax leaderMotor;
    private CANSparkMax followerMotor;
    private SparkPIDController leaderPidController;
    private SparkPIDController followerPidController;
    private Solenoid angleSolenoid;
    private double targetSpeed = 0.0;
    private PowerDistribution hub;

    public Shooter() {
         if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        this.hub = new PowerDistribution(1, ModuleType.kRev);
        this.leaderMotor = new CANSparkMax(ShooterConstants.SHOOTER_LEADER_MOTOR, MotorType.kBrushless);
        this.followerMotor = new CANSparkMax(ShooterConstants.SHOOTER_FOLLOWER_MOTOR, MotorType.kBrushless);
        this.leaderMotor.setInverted(true);
        this.leaderMotor.setIdleMode(IdleMode.kCoast);
        this.followerMotor.setIdleMode(IdleMode.kCoast);

        this.angleSolenoid = new Solenoid(21, PneumaticsModuleType.REVPH, 8);
        
        leaderPidController = leaderMotor.getPIDController();
        followerPidController = followerMotor.getPIDController();
        // this.leaderMotor.getEncoder().setPositionConversionFactor(1.0 / 3.0);
        // this.leaderMotor.getEncoder().setVelocityConversionFactor(1.0 / 3.0);

        // this.followerMotor.getEncoder().setPositionConversionFactor(1.0 / 3.0);
        // this.followerMotor.getEncoder().setVelocityConversionFactor(1.0 / 3.0);


        leaderPidController.setP(0.000);
        leaderPidController.setI(0.0);
        leaderPidController.setD(0.00);
        leaderPidController.setFF(0.00019);
        followerPidController.setP(0.000);
        followerPidController.setI(0.0);
        followerPidController.setD(0.00);
        followerPidController.setFF(0.00019);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Shooter Speed", leaderMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Target Shooter Speed", targetSpeed);
        SmartDashboard.putNumber("Drawn Amps", hub.getCurrent(7));
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
    public boolean overDrawingAmps() {
        return hub.getCurrent(7) > 55 || hub.getCurrent(8) > 55;
    }

    public void shoot(double speed) {
        if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        // SmartDashboard.putNumber("Shooter Desired Speed", speed);
        this.targetSpeed = speed;
        // leaderMotor.set(speed);
        leaderMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        followerMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    public boolean isAtSpeed() {
        return Math.abs(leaderMotor.getEncoder().getVelocity() - targetSpeed) < Constants.ShooterConstants.FLYWHEEL_ERROR_MARGIN;
    }
    public double getFlywheelSpeedMPS() {
        return 0.0508 * leaderMotor.getEncoder().getVelocity() / 60;
    }
    
    public void setSourcePickupPosition() {
        angleSolenoid.set(true);
        
    }
    public void setStowPosition() {
        angleSolenoid.set(false);
    }
    
}
