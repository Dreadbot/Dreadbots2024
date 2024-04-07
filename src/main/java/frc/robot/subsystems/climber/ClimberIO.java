package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInput {
        public boolean leftTopSwitch;
        public boolean leftBottomSwitch;
        public boolean rightTopSwitch;
        public boolean rightBottomSwitch;
        
        public double leftPosition;
        public double leftVoltage;
        public double leftCurrent;
        public double leftTemperature;
        
        public double rightPosition;
        public double rightVoltage;
        public double rightCurrent;
        public double rightTemperature;
    }

    public default void updateInputs(ClimberIOInput inputs) {}
    public default void arcade(double verticalSpeed, double rotationSpeed) {}
    public default void tank(double leftSpeed, double rightSpeed) {}
    public default void setPiston(boolean state) {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
