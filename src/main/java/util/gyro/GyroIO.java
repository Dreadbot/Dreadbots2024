package util.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yaw;
        public double roll;
        public double pitch;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
    public default void reset() {}
}
