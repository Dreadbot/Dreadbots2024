package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double absolutePosition = 0.0;

        public double leftVoltage = 0.0;
        public double leftCurrent = 0.0;
        public double leftTemperature = 0.0;

        public double rightVoltage = 0.0;
        public double rightCurrent = 0.0;
        public double rightTemperature = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
    public default void setIdleMode(IdleMode idleMode) {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
