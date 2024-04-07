package frc.robot.commmands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class UnlockCommand extends Command {
    private final Climber climber;

    public UnlockCommand(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void execute() {
        this.climber.unlock();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
