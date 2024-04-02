package frc.robot.commmands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class UnbindCommand extends Command {

    private final Climber climber;
    private double speed;
    
    public UnbindCommand(Climber climber, double speed) {
        this.climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.climb(-speed, 0);
    } 

    @Override
    public void end(boolean interupted) {
        climber.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
        //return climberPositions[0] <= ClimberConstants.MIN_HEIGHT || climberPositions[1] <= ClimberConstants.MIN_HEIGHT;
    }
    
}
