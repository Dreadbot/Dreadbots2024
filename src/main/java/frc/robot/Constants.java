// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

@SuppressWarnings("SpellCheckingInspection")
public abstract class Constants {
    //region * SECTION - CONTROL PANEL
    public static final boolean DRIVE_ENABLED        = true;

    //endregion

    //region * SECTION - ROBOT PORTS
    // =================================================================================================================

    public static class OperatorConstants {
      public static final int PRIMARY_JOYSTICK_PORT   = 0;
      public static final int SECONDARY_JOYSTICK_PORT = 1;
    }
    

    // Motor Ports

    //region * SECTION - DRIVE CONSTANTS
    // =================================================================================================================

    // Drive Controller Constants
    public static final double TRAJECTORY_ERROR_CONTROLLER_P = 3.6109d;
    public static final double WHEEL_CONTROLLER_P            = 1.0d;

    // Drive Mechanical Wheel Specifications
    public static final double WHEEL_GEARING                  = 14.0d / 70.0d;
    public static final double WHEEL_CIRCUMFERENCE_METERS     = Units.inchesToMeters(3.0d) * 2 * Math.PI;
    public static final double WHEEL_ROTATIONS_TO_METERS      = WHEEL_CIRCUMFERENCE_METERS * WHEEL_GEARING;
    public static final double WHEEL_RPM_TO_METERS_PER_SECOND = WHEEL_ROTATIONS_TO_METERS / 60.0d;

    // This is how far the wheel centers are from the physical center of the robot
    // Longitudinal (forward/backward) and Lateral (side/side) distances are useful for kinematics.
    public static final double WHEEL_LONGITUDINAL_DISPLACEMENT = 0.19d;   // meters
    public static final double WHEEL_LATERAL_DISPLACEMENT      = 0.4191d; // meters

    // These are the individual wheel feedforward gains
    public static final double WHEEL_FEED_STATIC_FRICTION_GAIN = 0.09185d;
    public static final double WHEEL_FEED_VELOCITY_GAIN        = 3.1899d;
    public static final double WHEEL_FEED_ACCELERATION_GAIN    = 0.17172d;
    //endregion


    private Constants() throws IllegalStateException {
        throw new IllegalStateException("Constants is a utility class. It should not be instantiated.");
    }

    public static class DriveConstants {
      public static final double SPEED_LIMITER = 1.0; // !!! DANGEROUS ON 2023 BOT !!! BE CAREFUL!\
      public static final double DEADBAND = 0.1;
    }

    public static class SwerveConstants {
        public static final double ATTAINABLE_MAX_SPEED = 3;
        public static final double MODULE_Y_OFFSET = Units.inchesToMeters(26.0) / 2; // Between the front and back
        public static final double MODULE_X_OFFSET = Units.inchesToMeters(23.0) / 2; // Between the left and right
        // Encoder offsets are in degrees, not radians
        public static final double FRONT_LEFT_ENCODER_OFFSET = -137.021 + 180;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = -31.816 - 2;
        public static final double BACK_LEFT_ENCODER_OFFSET = -116.895 - 1;
        public static final double BACK_RIGHT_ENCODER_OFFSET = -121.904 + 180 + 2;
        public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double TURN_GEAR_RATIO = 150 / 7;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
      }
      public static class AutonomousConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 2.5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.25;
      }
}
