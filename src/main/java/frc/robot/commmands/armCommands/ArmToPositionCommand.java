package frc.robot.commmands.armCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmToPositionCommand extends Command {

    private Arm arm;
    private double position;
    private DoubleSupplier joystickOverride;

    public ArmToPositionCommand(Arm arm, double position, DoubleSupplier joystickOverride) {
        this.arm = arm;
        this.position = position;
        this.joystickOverride = joystickOverride;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setReference(new State(position, 0));
    } 

    @Override
    public void end(boolean canceled) {
        if(canceled) {
            arm.setReference(new State(arm.getEncoderPosition(), 0));
        }
    }

    @Override
    public boolean isFinished() {
        return arm.isAtDesiredState() || Math.abs(joystickOverride.getAsDouble()) > 0.08;
    }
}
