package frc.robot.commmands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class OuttakeCommand extends Command{
    private final Intake intake;


    public OuttakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
       intake.intake(-IntakeConstants.OUTTAKE_SPEED);
    }
    
    @Override
    public void end(boolean interupted) {
        intake.intake(0);
    }
    
}
