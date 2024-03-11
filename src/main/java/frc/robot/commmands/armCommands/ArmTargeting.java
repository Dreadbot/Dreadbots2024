package frc.robot.commmands.armCommands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import util.math.DreadbotMath;

public class ArmTargeting extends Command {
    private final Arm arm;
    private final Shooter shooter;
    private final Drive drive;
    private double distance;

    public ArmTargeting(Arm arm, Shooter shooter, Drive drive) {
        this.arm = arm;
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(arm);
    }

    @Override
    public void execute() { 
        double distToTagX = 16.579342 - drive.getPosition().getX();
        double distToTagZ = 5.547868 - drive.getPosition().getY();
        distance = Math.sqrt(Math.pow(distToTagX, 2) + Math.pow(distToTagZ, 2));
        distance += 0.2032;
        double speed = shooter.getFlywheelSpeedMPS();
        double theta = -1.1555 + (4.9849 / speed) + (0.4805 / distance) + (0.0896 * speed) + (0.2774 * distance) + (-0.0285 * speed * distance);
        SmartDashboard.putNumber("Arm Angle", theta);
        SmartDashboard.putNumber("Arm Estimated Position", theta / (Math.PI * 2));

        SmartDashboard.putNumber("Distance ", distance);
        SmartDashboard.putNumber("FlyWheel tangental speed", speed);

        arm.setReference(new State(theta / (2 * Math.PI), 0));
    }
}
