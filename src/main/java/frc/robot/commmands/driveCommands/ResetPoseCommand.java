package frc.robot.commmands.driveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class ResetPoseCommand extends Command {
    private final Drive drive;
    private final DoubleSubscriber robotPosX;
    private final DoubleSubscriber robotPosY;
    private final DoubleSubscriber robotTheta;
    private final BooleanSubscriber tagSeen;

    public ResetPoseCommand(Drive drive, NetworkTable table) {
        this.drive = drive;
        this.robotPosX = table.getDoubleTopic("robotposZ").subscribe(0.0);
        this.robotPosY = table.getDoubleTopic("robotposX").subscribe(0.0);
        this.robotTheta = table.getDoubleTopic("robotposTheta").subscribe(0.0);
        this.tagSeen = table.getBooleanTopic("tagSeen").subscribe(false);
    }

    @Override
    public void execute() {
        if (tagSeen.get()) {
            this.drive.resetOdometry(
                new Pose2d(new Translation2d(this.robotPosX.get(), this.robotPosY.get()),
                new Rotation2d(robotTheta.get()))
            );
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
