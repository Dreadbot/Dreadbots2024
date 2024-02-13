package frc.robot.commmands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;
    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;

    }
    
    @Override
    public void execute() {
        shooter.shoot(1000d);
    }
    
    @Override
    public void end(boolean interupted) {
        shooter.stopMotors();
    }
}