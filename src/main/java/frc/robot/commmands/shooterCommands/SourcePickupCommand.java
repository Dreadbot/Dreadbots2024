package frc.robot.commmands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SourcePickupCommand extends Command {
    
    private Shooter shooter;

    public SourcePickupCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSourcePickupPosition();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setStowPosition();
    }
}
