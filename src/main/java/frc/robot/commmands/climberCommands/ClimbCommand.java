package frc.robot.commmands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbCommand extends Command {
    private final Climber climber;

    public ClimbCommand(Climber climber) {
        this.climber = climber;
    }

    
}
