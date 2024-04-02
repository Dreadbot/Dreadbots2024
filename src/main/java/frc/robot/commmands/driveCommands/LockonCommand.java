package frc.robot.commmands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive; 

public class LockonCommand extends Command {
    private final Drive drive;

    public LockonCommand(Drive drive) {
        this.drive = drive;
    }
    @Override
    public void execute() {
        drive.doLockon = true;
    }
    @Override
    public void end(boolean isCancelled){
        drive.doLockon = false;
    }
}
