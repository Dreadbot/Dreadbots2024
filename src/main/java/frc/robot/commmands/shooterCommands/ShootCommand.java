package frc.robot.commmands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final double speed;
    public ShootCommand(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
    }
    
    @Override
    public void initialize() {
        shooter.shoot(speed);
    }
    
    @Override
    public void end(boolean interupted) {
        shooter.stopMotors();
    }
}
