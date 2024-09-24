package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        boolean hasNote = false;

        double velocity = 0.0;
        double voltage = 0.0;
        double current = 0.0;
        double temperature = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void set(double speed) {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
