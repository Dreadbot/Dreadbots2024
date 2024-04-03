package util.swerve;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private SwerveModuleIO io;
    private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private PIDController turningPIDController = new PIDController(6.5, 0, 0);
    public SwerveModuleState desiredState = new SwerveModuleState();

    public String name;
    
    public SwerveModule(SwerveModuleIO io, String name) {
        this.name = name;
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        this.io = io;
        io.updateInputs(inputs);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SwerveModules/" + name, inputs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.velocity,
            inputs.rotation
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePosition,
            inputs.rotation
        );
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public SwerveModuleState getOptimizedState() {
        return SwerveModuleState.optimize(getState(), inputs.rotation);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, inputs.rotation);
        this.desiredState = desiredState;
        double turnOutput = turningPIDController.calculate(inputs.rotation.getRadians(), optimizedState.angle.getRadians());
        io.setDriveReference(optimizedState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        io.setTurnVoltage(turnOutput);
    }

    public void putValuesToSmartDashboard(String name) {
        Logger.recordOutput("Drive/SwerveModules/" + name + "/CANCoder", inputs.rotation.getDegrees() / 360);
    }

    public void close() throws Exception{
        io.close();
    }

    public void stopMotors() {
        io.stopMotors();
    }
}