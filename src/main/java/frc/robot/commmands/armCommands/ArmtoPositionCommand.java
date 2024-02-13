package frc.robot.commmands.armCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmtoPositionCommand extends Command {
    private Arm arm;
    private double position;
     public ArmtoPositionCommand(Arm arm, double position) {
        this.arm = arm;
        this.position = position;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setReference(new State(position, 0));
    }
    
    @Override
    public void end(boolean interupted) {
        arm.stopMotors();
    }
    
}