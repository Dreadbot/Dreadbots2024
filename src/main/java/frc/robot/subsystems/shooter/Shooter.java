package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import util.misc.DreadbotSubsystem;

public class Shooter extends DreadbotSubsystem {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    private double targetSpeed = 0.0;
    private PowerDistribution hub;

    public Shooter(ShooterIO io) {
         if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        this.hub = new PowerDistribution(1, ModuleType.kRev);
        
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.recordOutput("Shooter/ActualSpeed", (inputs.leaderVelocity + inputs.followerVelocity) / 2);
        Logger.recordOutput("Shooter/TargetSpeed", targetSpeed);
        Logger.recordOutput("Shooter/DrawnAmps", hub.getCurrent(7));
    }
    @Override
    public void close() throws Exception {
        if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        io.close();
    }

    @Override
    public void stopMotors() {
        if(!Constants.SubsystemConstants.SHOOTER_ENABLED) {
            return;
        }
        io.stopMotors();
        
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
        io.setReference(speed, ControlType.kVelocity);
    }

    public boolean isAtSpeed() {
        return Math.abs(inputs.leaderVelocity - targetSpeed) < Constants.ShooterConstants.FLYWHEEL_ERROR_MARGIN;
    }
    public double getFlywheelSpeedMPS() {
        return 0.0508 * inputs.leaderVelocity / 60;
    }
}
