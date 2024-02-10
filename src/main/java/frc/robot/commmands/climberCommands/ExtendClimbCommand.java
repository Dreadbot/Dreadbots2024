package frc.robot.commmands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ExtendClimbCommand extends Command {

    private final Climber climber;

    public ExtendClimbCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.climb(ClimberConstants.EXTEND_SPEED, 0); // no rotation on extension for obvious
        
    }


    @Override
    public void end(boolean interupted) {
        climber.stopMotors();
    }

    @Override
    public boolean isFinished() {
       // double[] climberPositions = climber.getClimberPositions();
       // return climberPositions[0] >= ClimberConstants.MAX_HEIGHT || climberPositions[1] >= ClimberConstants.MAX_HEIGHT;
       return false;
    }
    
}
