package frc.robot.commmands.autonomousCommands;

import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commmands.armCommands.ArmTargetCommand;
import frc.robot.commmands.intakeCommands.FeedCommand;
import frc.robot.commmands.intakeCommands.OuttakeCommand;
import frc.robot.commmands.shooterCommands.ShootCommand;
import frc.robot.commmands.shooterCommands.StopShootCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

public class AutoShootVisionCommand extends SequentialCommandGroup {
    public AutoShootVisionCommand(Intake intake, Arm arm, Shooter shooter, Drive drive, double rpm) {
        addCommands(
            (new OuttakeCommand(intake)
                .withTimeout(0.07)
                .andThen(new ShootCommand(shooter, rpm, null).withTimeout(3))
                .deadlineWith(new ArmTargetCommand<SwerveDriveWheelPositions>(arm, drive.getPoseEstimator()))),
            new ArmTargetCommand<SwerveDriveWheelPositions>(arm, drive.getPoseEstimator()),
            new FeedCommand(intake).withTimeout(.3),
            new StopShootCommand(shooter)
        );
    }
}
