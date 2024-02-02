// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

@SuppressWarnings("SpellCheckingInspection")
public abstract class Constants {
    public static class OperatorConstants {
      public static final int PRIMARY_JOYSTICK_PORT   = 0;
      public static final int SECONDARY_JOYSTICK_PORT = 1;
    }
    public static class SubsystemConstants {
      public static final boolean DRIVE_ENABLED = true;
    }
    public static class DriveConstants {
      public static final double SPEED_LIMITER = 1; // !!! DANGEROUS ON 2023 BOT !!! BE CAREFUL!
      public static final double DEADBAND = 0.14;
    }

    public static class SwerveConstants {
        public static final double ATTAINABLE_MAX_SPEED = 3;
        public static final double MODULE_Y_OFFSET = Units.inchesToMeters(25.0) / 2; // Between the front and back
        public static final double MODULE_X_OFFSET = Units.inchesToMeters(25.0) / 2; // Between the left and right
        // Encoder offsets are in rotations now???? ok
        public static final double FRONT_LEFT_ENCODER_OFFSET = 0.134765625 + 0.5;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.02734375 + 0.5;
        public static final double BACK_LEFT_ENCODER_OFFSET = -0.32 - 0.5;
        public static final double BACK_RIGHT_ENCODER_OFFSET = -0.389404296875;
        public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double TURN_GEAR_RATIO = 150 / 7;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
      }

      public static class AutonomousConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 2.5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.25;
      }
      
    private Constants() throws IllegalStateException {
        throw new IllegalStateException("Constants is a utility class. It should not be instantiated.");
    }
}
