package frc.robot.commmands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class LockCommand extends Command {
    private final Climber climber;

    public LockCommand(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void execute() {
        this.climber.lock();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
