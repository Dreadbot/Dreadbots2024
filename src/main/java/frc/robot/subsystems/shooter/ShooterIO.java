package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.ControlType;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leaderVelocity;
        public double leaderVoltage;
        public double leaderCurrent;
        public double leaderTemperature;

        public double followerVelocity;
        public double followerVoltage;
        public double followerCurrent;
        public double followerTemperature;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setReference(double reference, ControlType controlType) {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
