package frc.robot.commmands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class ResetGyroCommand extends Command {
    
    private final Drive drive;

    public ResetGyroCommand(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void execute() {
        this.drive.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
