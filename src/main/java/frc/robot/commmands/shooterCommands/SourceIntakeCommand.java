package frc.robot.commmands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SourceIntakeCommand extends Command {

    private final Shooter shooter;

    public SourceIntakeCommand(Shooter shooter) {
        this.shooter = shooter;
    }
    
    @Override
    public void initialize() {
        shooter.shoot(-1000);
    }
    
    @Override
    public void end(boolean interupted) {
        shooter.stopMotors();
    }
}
