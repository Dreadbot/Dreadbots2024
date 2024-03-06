package frc.robot.commmands.intakeCommands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commmands.controllerCommands.RumbleController;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private final Intake intake;
    private final XboxController controller;

    public IntakeCommand(Intake intake, XboxController controller) {
        this.intake = intake;
        this.controller = controller;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake(IntakeConstants.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMotors();
        if (!interrupted && controller != null) {
            CommandScheduler.getInstance().schedule(new RumbleController(controller).raceWith(new WaitCommand(0.5)));
        }
    }
    @Override
    public boolean isFinished() {
        return intake.hasNote();
    }
}
