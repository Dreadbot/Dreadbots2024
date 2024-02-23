package frc.robot.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commmands.armCommands.ArmToPositionCommand;
import frc.robot.commmands.intakeCommands.FeedCommand;
import frc.robot.commmands.intakeCommands.OuttakeCommand;
import frc.robot.commmands.shooterCommands.ShootCommand;
import frc.robot.commmands.shooterCommands.StopShootCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShootCommand extends SequentialCommandGroup {

    public AutoShootCommand(Intake intake, Arm arm, Shooter shooter, double armPosition, double rpm) {
        addCommands(
            new WaitCommand(0.18)
                .deadlineWith(new OuttakeCommand(intake)),
            new ShootCommand(shooter, rpm)
                .alongWith(new ArmToPositionCommand(arm, armPosition))
                .until(() -> shooter.isAtSpeed() && arm.isAtDesiredState()),
            new FeedCommand(intake)
                .raceWith(new WaitCommand(0.4)),
            new StopShootCommand(shooter)
        );
    }
    
}
