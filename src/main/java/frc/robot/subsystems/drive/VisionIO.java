package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Translation2d[] poses;
        public int[] tagIds;
        public double poseLatency;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
