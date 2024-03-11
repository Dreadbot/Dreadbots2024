package frc.robot.commmands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class StopIntakeCommand extends Command {

    private final Intake intake;

    public StopIntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.stopMotors();
    }
}
