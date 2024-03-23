package frc.robot.commmands.controllerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class EmergencyRumbleCommand extends Command {
    private XboxController controller;

    public EmergencyRumbleCommand(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public void execute() {
        controller.setRumble(RumbleType.kBothRumble, 2 * Math.sin(Timer.getFPGATimestamp() * Math.PI * 8));
    }
    @Override
    public void end(boolean interrupted) {
        this.controller.setRumble(RumbleType.kBothRumble, 0.0);
    }

}
