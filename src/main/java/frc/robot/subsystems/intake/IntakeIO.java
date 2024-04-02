package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        boolean hasNote;

        double velocity;
        double voltage;
        double current;
        double temperature;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void set(double speed) {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
