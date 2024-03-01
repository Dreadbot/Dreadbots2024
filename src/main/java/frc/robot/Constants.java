// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

@SuppressWarnings("SpellCheckingInspection")
public abstract class Constants {
    public static class OperatorConstants {
      public static final int PRIMARY_JOYSTICK_PORT   = 0;
      public static final int SECONDARY_JOYSTICK_PORT = 1;
    }
    public static class SubsystemConstants {
      public static final boolean DRIVE_ENABLED = true;
      public static final boolean ARM_ENABLED = true;
      public static final boolean CLIMBER_ENABLED = true;
      public static final boolean INTAKE_ENABLED = true;
      public static final boolean SHOOTER_ENABLED = true;

    }
    public static class DriveConstants {
      public static final double SPEED_LIMITER = 4; // !!! DANGEROUS ON 2023 BOT !!! BE CAREFUL!
      public static final double DEADBAND = 0.08;
      public static final double ROT_SPEED_LIMITER = 1 * Math.PI;
      public static final double SLEW = 17;
    }

    public static class SwerveConstants {
        public static final double ATTAINABLE_MAX_SPEED = 5;
        public static final double MODULE_Y_OFFSET = Units.inchesToMeters(24.75) / 2; // Between the left and right
        public static final double MODULE_X_OFFSET = Units.inchesToMeters(24.75) / 2; //Between the front and back

        // Encoder offsets are in rotations now???? ok
        public static final double FRONT_LEFT_ENCODER_OFFSET = 0.131591796875 + 0.5;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.0234375 + 0.5;
        public static final double BACK_LEFT_ENCODER_OFFSET = -0.357421875 + 0.5;
        public static final double BACK_RIGHT_ENCODER_OFFSET = 0.1064453125 + 0.5;
        public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double TURN_GEAR_RATIO = 150 / 7;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
      }

      public static class AutonomousConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 2.5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.25;
      }
      public static class IntakeConstants {
        public static final double INTAKE_SPEED = 0.7;
        public static final double OUTTAKE_SPEED = 0.3;
        public static final int BEAM_BREAK_SENSOR = 7;
      }
      public static class ShooterConstants {
        public static final double FLYWHEEL_ERROR_MARGIN = 50;
      }
      public static class ClimberConstants {
        public static final int LEFT_CLIMB_MOTOR = 18;
        public static final int RIGHT_CLIMB_MOTOR = 19;
        public static final int TOP_LEFT_LIMIT_SWITCH_ID = 6;
        public static final int TOP_RIGHT_LIMIT_SWITCH_ID = 4;
        public static final int BOTTOM_LEFT_LIMIT_SWITCH_ID = 5;
        public static final int BOTTOM_RIGHT_LIMIT_SWITCH_ID = 3;


        public static final double GYRO_ANGLE_CONVERSION_FACTOR = 90.0;
        public static final double GYRO_PITCH_OFFSET = -1.3;
        public static final double EXTEND_SPEED = 0.5;
        public static final double MAX_HEIGHT = 100; //placeholder number
        public static final double RETRACT_SPEED = -0.5;
        public static final double P_GAIN = 0.2;
        public static final double MIN_HEIGHT = 10; 
      }
      public static class ArmConstants {
        public static final double ARM_POSITION_ERROR_MARGIN = 0.00004;
        public static final double ARM_GEAR_RATIO = 1.0 / 100.0;
        public static final double KG = 0.15; //0.27
        public static final double AUTON_START_POSITION = (90 - 15) / 360.0; //Arm starts 4 degress from vertical
        public static final double ARM_SOURCE_PICKUP_POSITION = 0.2;
      }
      public static class ColorSensorConstants {
        public static final Color NOTE_COLOR = new Color("#9F3F1F");
        public static final double CONFIDENCE = 0.80;
      }
    private Constants() throws IllegalStateException {
        throw new IllegalStateException("Constants is a utility class. It should not be instantiated.");
    }
}
