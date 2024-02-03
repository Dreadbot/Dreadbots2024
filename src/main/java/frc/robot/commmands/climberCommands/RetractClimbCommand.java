package frc.robot.commmands.climberCommands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class RetractClimbCommand extends Command {
    private final Climber climber;
    private final AHRS gyro;

    public RetractClimbCommand(Climber climber, AHRS gyro) {
        this.climber = climber;
        this.gyro = gyro;
        addRequirements(climber);

    }
    @Override
    public void execute() {
        climber.climb(ClimberConstants.RETRACT_SPEED, ClimberConstants.P_GAIN * gyro.getRoll());
    } 

    @Override
    public boolean isFinished() {
        double[] climberPositions = climber.getClimberPositions();
        return climberPositions[0] <= ClimberConstants.MIN_HEIGHT || climberPositions[1] <= ClimberConstants.MIN_HEIGHT;
    }

}
