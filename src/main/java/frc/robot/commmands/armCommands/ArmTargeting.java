package frc.robot.commmands.armCommands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;

public class ArmTargeting extends Command{
    private final Arm arm;
    private final DoubleSubscriber dist;
    private final Shooter shooter;
    private final BooleanSubscriber tagSeen;

    public ArmTargeting(Arm arm, Shooter shooter, NetworkTable table) {
        this.arm = arm;
        this.dist = table.getDoubleTopic("distanceToTag").subscribe(0.0);
        this.tagSeen = table.getBooleanTopic("tagSeen").subscribe(false);
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        double distance = dist.get();
        double speed = shooter.getFlywheelSpeed()*4*Math.PI*60*0.0254;
        double theta = -1.6090 + (7.2587/speed) + (0.4664/distance) + (0.1147*speed) + (0.3058*distance) + (-0.0312*speed*distance);
        AHRS gyro;
        PoseEstimator poseEstimator;
        if (tagSeen.get()) {
            double initialDistanceToTag = dist.get();
        }
        if (gyro.getVelocityX() != 0 || gyro.getVelocityZ() != 0) {
            double distToTagX = 16.579342 - poseEstimator.getEstimatedPosition().getY();
            double distToTagZ = 5.547868 - poseEstimator.getEstimatedPosition().getX();
            distance = Math.atan(distToTagX / distToTagZ) + gyro.getYaw();
        }
        arm.setReference(new State(theta/(2*Math.PI), 0));
    }
}
