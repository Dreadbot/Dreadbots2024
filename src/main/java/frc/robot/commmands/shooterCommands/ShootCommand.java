package frc.robot.commmands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;
    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;

    }
    
    @Override
    public void initialize() {
        shooter.shoot(8000);
    }
    
    @Override
    public void end(boolean interupted) {
        shooter.stopMotors();
    }
}
