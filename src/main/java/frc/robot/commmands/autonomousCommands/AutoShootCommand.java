package frc.robot.commmands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commmands.armCommands.ArmToPositionCommand;
import frc.robot.commmands.intakeCommands.FeedCommand;
import frc.robot.commmands.intakeCommands.OuttakeCommand;
import frc.robot.commmands.shooterCommands.ShootCommand;
import frc.robot.commmands.shooterCommands.StopShootCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShootCommand extends SequentialCommandGroup {

    public AutoShootCommand(Intake intake, Arm arm, Shooter shooter, double armPosition, double rpm) {
        addCommands(
            new OuttakeCommand(intake).raceWith(new WaitCommand(0.07)),
            (new ShootCommand(shooter, rpm, null)
                .alongWith(new ArmToPositionCommand(arm, armPosition, () -> 0)))
                .withTimeout(3),
            new FeedCommand(intake)
                .raceWith(new WaitCommand(0.3)),
            new StopShootCommand(shooter)
        );
    }
    
}
