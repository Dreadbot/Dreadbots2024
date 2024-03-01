package frc.robot.commmands.controllerCommands;

import java.sql.PseudoColumnUsage;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleController extends Command {
    PS4Controller controller;

    public RumbleController(PS4Controller controller) {
        this.controller = controller;
    }

    @Override
    public void initialize() {
        this.controller.setRumble(RumbleType.kBothRumble, 1.0);
    }

    @Override
    public void end(boolean interrupted) {
        this.controller.setRumble(RumbleType.kBothRumble, 0.0);
    }
}
