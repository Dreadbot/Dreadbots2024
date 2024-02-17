package frc.robot.commmands.armCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class ArmTargeting extends Command{
    private final Arm arm;
    private DoubleSubscriber dist;
    private final Shooter shooter;

    public ArmTargeting(Arm arm, Shooter shooter, DoubleSubscriber distSub) {
        this.arm = arm;
        this.dist = distSub;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        double speed = shooter.getFlywheelSpeed();
        double distance = dist.get();
        double theta = -1.6090 + (7.2587/speed) + (0.4664/distance) + (0.1147*speed) + (0.3058*distance) + (-0.0312*speed*distance);
        arm.setReference(new State(theta/(2*Math.PI), 0));
    }
}
