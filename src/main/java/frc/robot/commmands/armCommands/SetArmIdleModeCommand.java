package frc.robot.commmands.armCommands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class SetArmIdleModeCommand extends Command {

    private Arm arm;

    public SetArmIdleModeCommand(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.setIdleMode(IdleMode.kCoast);
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.setIdleMode(IdleMode.kBrake);
    }
}
