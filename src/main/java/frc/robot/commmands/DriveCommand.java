package frc.robot.commmands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subystems.Drive;
import util.math.DreadbotMath;
import util.math.Vector2D;

public class DriveCommand extends Command {

    private final Drive drive;
    private final DoubleSupplier joystickX;
    private final DoubleSupplier joystickY;
    private final DoubleSupplier joystickRotatation;


    public DriveCommand(Drive drive, DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joyStickRotation) {
        this.drive = drive;
        this.joystickX = joystickX;
        this.joystickY = joystickY;
        this.joystickRotatation = joyStickRotation;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Vector2D joystickValue = DreadbotMath.applyDeadbandToVector(new Vector2D(joystickX.getAsDouble(), joystickY.getAsDouble()), DriveConstants.DEADBAND);
        double rotation = DreadbotMath.applyDeadbandToValue(joystickRotatation.getAsDouble(), DriveConstants.DEADBAND) * DriveConstants.SPEED_LIMITER;
        double forward = joystickValue.x1 * DriveConstants.SPEED_LIMITER;
        double strafe = joystickValue.x2 * DriveConstants.SPEED_LIMITER;

        drive.drive(strafe, forward, rotation, true);
    }
}
