package util.swerve;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double velocity = 0.0;
        public Rotation2d rotation = Rotation2d.fromDegrees(0);

        public double drivePosition = 0.0;
        public double driveVoltage = 0.0;
        public double driveCurrent = 0.0;
        public double driveTemperature = 0.0;

        public double turningVoltage = 0.0;
        public double turningCurrent = 0.0;
        public double turningTemperature = 0.0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}
    public default void setDriveReference(double reference, ControlType controlType) {}
    public default void setTurnVoltage(double voltage) {}
    public default void resetEncoder() {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
