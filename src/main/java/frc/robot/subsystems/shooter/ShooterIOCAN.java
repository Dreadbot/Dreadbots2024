package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOCAN implements ShooterIO {
    private CANSparkMax leaderMotor;
    private CANSparkMax followerMotor;
    private SparkPIDController leaderPidController;
    private SparkPIDController followerPidController;

    public ShooterIOCAN() {
        this.leaderMotor = new CANSparkMax(ShooterConstants.SHOOTER_LEADER_MOTOR, MotorType.kBrushless);
        this.followerMotor = new CANSparkMax(ShooterConstants.SHOOTER_FOLLOWER_MOTOR, MotorType.kBrushless);
        this.leaderMotor.setInverted(true);
        this.leaderMotor.setIdleMode(IdleMode.kCoast);
        this.followerMotor.setIdleMode(IdleMode.kCoast);
        
        this.leaderPidController = leaderMotor.getPIDController();
        this.followerPidController = followerMotor.getPIDController();

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
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leaderVelocity = leaderMotor.getEncoder().getVelocity();
        inputs.leaderVoltage = leaderMotor.getAppliedOutput() * leaderMotor.getBusVoltage();
        inputs.leaderCurrent = leaderMotor.getOutputCurrent();
        inputs.leaderTemperature = leaderMotor.getMotorTemperature();

        inputs.followerVelocity = followerMotor.getEncoder().getVelocity();
        inputs.followerVoltage = followerMotor.getAppliedOutput() * followerMotor.getBusVoltage();
        inputs.followerCurrent = followerMotor.getOutputCurrent();
        inputs.followerTemperature = followerMotor.getMotorTemperature();
    }

    @Override
    public void setReference(double reference, ControlType controlType) {
        leaderPidController.setReference(reference, controlType);
        followerPidController.setReference(reference, controlType);
    }

    @Override
    public void close() throws Exception {
        leaderMotor.close();
        followerMotor.close();
    }

    @Override
    public void stopMotors() {
        leaderMotor.stopMotor();
        followerMotor.stopMotor();
    }
}
