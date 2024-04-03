package util.swerve;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double velocity;
        public Rotation2d rotation;

        public double drivePosition;
        public double driveVoltage;
        public double driveCurrent;
        public double driveTemperature;

        public double turningVoltage;
        public double turningCurrent;
        public double turningTemperature;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}
    public default void setDriveReference(double reference, ControlType controlType) {}
    public default void setTurnVoltage(double voltage) {}
    public default void resetEncoder() {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
