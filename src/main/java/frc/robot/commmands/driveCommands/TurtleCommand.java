package frc.robot.commmands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class TurtleCommand extends Command {
    
    private final DriveCommand driveCommand;
    public TurtleCommand(DriveCommand command) {
        this.driveCommand = command;

    }

    @Override
    public void initialize() {
        driveCommand.enableTurtle();
    }

    @Override 
    public void end(boolean interupted) {
        driveCommand.disableTurtle();
    }

    
}
