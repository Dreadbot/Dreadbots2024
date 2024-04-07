package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double absolutePosition;

        public double leftVoltage;
        public double leftCurrent;
        public double leftTemperature;

        public double rightVoltage;
        public double rightCurrent;
        public double rightTemperature;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
    public default void setIdleMode(IdleMode idleMode) {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
