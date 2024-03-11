package frc.robot.commmands.climberCommands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimbCommand extends Command {

    private final Climber climber;
    private final AHRS gyro;
    
    public ClimbCommand(Climber climber, AHRS gyro) {
        this.climber = climber;
        this.gyro = gyro;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.climb(ClimberConstants.RETRACT_SPEED, (-gyro.getRoll()) / ClimberConstants.GYRO_ANGLE_CONVERSION_FACTOR);
        SmartDashboard.putNumber("leftClimberPosition", climber.getLeftClimberPosition());
        SmartDashboard.putNumber("rightClimberPosition", climber.getRightClimberPosition());
    } 

    @Override
    public void end(boolean interupted) {
        climber.stopMotors();
        climber.lock();
    }

    @Override
    public boolean isFinished() {
        return false;
        //return climberPositions[0] <= ClimberConstants.MIN_HEIGHT || climberPositions[1] <= ClimberConstants.MIN_HEIGHT;
    }
    
}
