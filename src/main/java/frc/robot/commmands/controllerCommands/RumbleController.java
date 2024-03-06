package frc.robot.commmands.controllerCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleController extends Command {
    XboxController controller;

    public RumbleController(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public void execute() {
        this.controller.setRumble(RumbleType.kBothRumble, 1.0);
    }

    @Override
    public void end(boolean interrupted) {
        this.controller.setRumble(RumbleType.kBothRumble, 0.0);
    }
}
