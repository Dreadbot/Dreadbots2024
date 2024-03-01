package frc.robot.commmands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShootCommand extends Command {
    
    Shooter shooter;

    public StopShootCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        this.shooter.shoot(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
