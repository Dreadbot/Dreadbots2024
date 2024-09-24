package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.ControlType;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leaderVelocity = 0.0;
        public double leaderVoltage = 0.0;
        public double leaderCurrent = 0.0;
        public double leaderTemperature = 0.0;

        public double followerVelocity = 0.0;
        public double followerVoltage = 0.0;
        public double followerCurrent = 0.0;
        public double followerTemperature = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setReference(double reference, ControlType controlType) {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
