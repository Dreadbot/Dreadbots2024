package frc.robot.commmands.driveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import util.math.DreadbotMath;
import util.math.Vector2D;

public class DriveCommand extends Command {

    private final Drive drive;
    private final DoubleSupplier joystickX;
    private final DoubleSupplier joystickY;
    private final DoubleSupplier joystickRotatation;
    private double speedModifier; 

    public DriveCommand(Drive drive, DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joyStickRotation) {
        this.drive = drive;
        this.joystickX = joystickX;
        this.joystickY = joystickY;
        this.joystickRotatation = joyStickRotation;
        this.speedModifier = 1;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double forward = -DreadbotMath.applyDeadbandToValue(joystickY.getAsDouble(), DriveConstants.DEADBAND) * DriveConstants.SPEED_LIMITER * speedModifier;
        double strafe = -DreadbotMath.applyDeadbandToValue(joystickX.getAsDouble(), DriveConstants.DEADBAND) * DriveConstants.SPEED_LIMITER * speedModifier;
        double rotation = -DreadbotMath.applyDeadbandToValue(joystickRotatation.getAsDouble(), DriveConstants.DEADBAND) * DriveConstants.ROT_SPEED_LIMITER;
        drive.drive(forward, strafe, rotation, true);
    }

    public void enableTurtle() {

        speedModifier = 0.5;

    }

    public void disableTurtle() {

        speedModifier = 1;
    }
}
