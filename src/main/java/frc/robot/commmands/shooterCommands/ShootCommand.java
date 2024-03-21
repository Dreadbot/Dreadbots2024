package frc.robot.commmands.shooterCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commmands.controllerCommands.RumbleController;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {

    private final Shooter shooter;
    private final double speed;
    private final XboxController controller;
    private long timer;
    
    public ShootCommand(Shooter shooter, double speed, XboxController controller) {
        this.shooter = shooter;
        this.speed = speed;
        this.controller = controller;
    }
    
    @Override
    public void initialize() {
        shooter.shoot(speed);
        timer = System.currentTimeMillis();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Shooter Timer", System.currentTimeMillis() - timer);

        if (this.controller != null && !interrupted) {
            CommandScheduler.getInstance().schedule(new RumbleController(controller).raceWith(new WaitCommand(0.5)));
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.isAtSpeed();
    }
}
