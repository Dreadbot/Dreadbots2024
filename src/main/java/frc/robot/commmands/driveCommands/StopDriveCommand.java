package frc.robot.commmands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class StopDriveCommand extends Command {
    
    private final Drive drive;

    public StopDriveCommand(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
