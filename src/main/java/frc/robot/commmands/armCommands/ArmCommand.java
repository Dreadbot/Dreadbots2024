package frc.robot.commmands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmCommand extends Command {
    private Arm arm;
     public ArmCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.moveToPosition(1);
    }
    
    @Override
    public void end(boolean interupted) {
        arm.stopMotors();
    }
    
}
