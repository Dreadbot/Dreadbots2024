package frc.robot.commmands.climberCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;
import util.gyro.GyroIOInputsAutoLogged;

public class ClimbCommand extends Command {

    private final Climber climber;
    private final GyroIOInputsAutoLogged gyroInputs;
    
    public ClimbCommand(Climber climber, GyroIOInputsAutoLogged gyroInputs) {
        this.climber = climber;
        this.gyroInputs = gyroInputs;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.climb(ClimberConstants.RETRACT_SPEED, (-gyroInputs.roll) / ClimberConstants.GYRO_ANGLE_CONVERSION_FACTOR);
        Logger.recordOutput("leftClimberPosition", climber.getLeftClimberPosition());
        Logger.recordOutput("rightClimberPosition", climber.getRightClimberPosition());
    } 

    @Override
    public void end(boolean interupted) {
        climber.stopMotors();
        climber.lock();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
