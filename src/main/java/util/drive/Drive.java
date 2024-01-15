// package util.drive;

// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import util.misc.DreadbotSubsystem;
// import util.misc.DreadbotMotor;

// /**
//  * The drive is the mechanism that moves the robot across the field. We are using a mecanum drive.
//  */
// public class Drive extends DreadbotSubsystem {
//     // Motor Objects
//     protected DreadbotMotor leftFrontMotor;
//     protected DreadbotMotor rightFrontMotor;
//     protected DreadbotMotor leftBackMotor;
//     protected DreadbotMotor rightBackMotor;

//     // NavX Gyroscope
//     protected AHRS gyroscope;

//     // Target ChassisSpeeds commanded by teleop directions or
//     protected ChassisSpeeds targetChassisSpeeds;

//     @Override
//     public void stopMotors() {
//         if(isDisabled()) return;

//         try {
//             //mecanumDrive.stopMotor();
//             leftFrontMotor.stopMotor();
//             rightFrontMotor.stopMotor();
//             leftBackMotor.stopMotor();
//             rightBackMotor.stopMotor();
//         } catch (IllegalStateException ignored) { disable(); }
//     }

//     @Override
//     public void close() {
//         // Stop motors before closure
//         stopMotors();

//         try {
//             leftFrontMotor.close();
//             rightFrontMotor.close();
//             leftBackMotor.close();
//             rightBackMotor.close();
//         } catch (IllegalStateException ignored) { disable(); }
//     }

//     /**
//      * Resets the encoder positions.
//      */
//     public void resetEncoders() {
//         if(isDisabled()) return;

//         rightFrontMotor.resetEncoder();
//         leftFrontMotor.resetEncoder();
//     }

//     /**
//      * Returns the gyroscope yaw.
//      *
//      * @return The gyroscope yaw
//      */
//     public double getYaw() {
//         return gyroscope.getYaw();
//     }

//     public AHRS getGyroscope() {
//         return gyroscope;
//     }
// }