package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInput {
        public boolean leftTopSwitch = false;
        public boolean leftBottomSwitch = false;
        public boolean rightTopSwitch = false;
        public boolean rightBottomSwitch = false;
        
        public double leftPosition = 0.0;
        public double leftVoltage = 0.0;
        public double leftCurrent = 0.0;
        public double leftTemperature = 0.0;
        
        public double rightPosition = 0.0;
        public double rightVoltage = 0.0;
        public double rightCurrent = 0.0;
        public double rightTemperature = 0.0;
    }

    public default void updateInputs(ClimberIOInput inputs) {}
    public default void arcade(double verticalSpeed, double rotationSpeed) {}
    public default void tank(double leftSpeed, double rightSpeed) {}
    public default void setPiston(boolean state) {}
    public default void close() throws Exception {}
    public default void stopMotors() {}
}
