package frc.robot.commmands.controllerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class EmergencyRumbleCommand extends Command {
    private XboxController controller;
    private double startTime;

    public EmergencyRumbleCommand(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
            this.controller.setRumble(RumbleType.kLeftRumble, 1.0);
            this.controller.setRumble(RumbleType.kRightRumble, 0.0);
        //this.controller.setRumble(RumbleType.kLeftRumble, Math.sin((startTime + Math.PI) / 2));
    }
    @Override
    public void end(boolean interrupted) {
        this.controller.setRumble(RumbleType.kBothRumble, 0.0);
    }

}
