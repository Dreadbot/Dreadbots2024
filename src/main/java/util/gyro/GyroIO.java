package util.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yaw = Rotation2d.fromDegrees(0);
        public double roll = 0.0;
        public double pitch = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
    public default void reset() {}
}
