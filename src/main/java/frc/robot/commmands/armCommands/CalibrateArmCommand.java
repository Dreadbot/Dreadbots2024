package frc.robot.commmands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class CalibrateArmCommand extends Command {
    private Arm arm;

    public CalibrateArmCommand(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.setJoystickOverride(-0.35);
    }
    
    @Override 
    public void end(boolean interrupted) {
        arm.setJoystickOverride(0);
        if(!interrupted) {
            System.out.println("Setting arm state to 0.2");
            arm.overrideArmState(0.20);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.getVerticalLimitSwitch();
    }
}
