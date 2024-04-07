package frc.robot.commmands.armCommands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCommand extends Command {

    private Arm arm;
    private DoubleSupplier joystickInput;

    public ArmCommand(Arm arm, DoubleSupplier joystickInput) {
        this.arm = arm;
        this.joystickInput = joystickInput;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setJoystickOverride(joystickInput.getAsDouble());
    }
    
    @Override
    public void end(boolean interupted) {
        arm.stopMotors();
    }
}
