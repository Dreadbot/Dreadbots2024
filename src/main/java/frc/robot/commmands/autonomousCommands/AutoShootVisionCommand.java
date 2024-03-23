package frc.robot.commmands.autonomousCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commmands.armCommands.ArmTargetCommand;
import frc.robot.commmands.intakeCommands.FeedCommand;
import frc.robot.commmands.intakeCommands.OuttakeCommand;
import frc.robot.commmands.shooterCommands.ShootCommand;
import frc.robot.commmands.shooterCommands.StopShootCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShootVisionCommand extends SequentialCommandGroup {
    public AutoShootVisionCommand(Intake intake, Arm arm, Shooter shooter, Drive drive, double rpm) {
        addCommands(
            (new OuttakeCommand(intake)
                .withTimeout(0.07)
                .andThen(new ShootCommand(shooter, rpm, null).withTimeout(3))
                .deadlineWith(new ArmTargetCommand(arm, drive.getPoseEstimator()))),
            new ArmTargetCommand(arm, drive.getPoseEstimator()),
            new FeedCommand(intake).withTimeout(.3),
            new StopShootCommand(shooter)
        );
    }
}
