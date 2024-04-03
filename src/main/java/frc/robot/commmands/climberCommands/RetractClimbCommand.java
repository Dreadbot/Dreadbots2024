package frc.robot.commmands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;

public class RetractClimbCommand extends Command {

    private Climber climber;
    
    public RetractClimbCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }
    
    @Override
    public void execute() {
        climber.retract(-ClimberConstants.EXTEND_SPEED);
    }

    @Override
    public void end(boolean interupted) {
        climber.stopMotors();
        climber.lock();
    }
}
