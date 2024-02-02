package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import util.misc.DreadbotSubsystem;

public class Climber extends DreadbotSubsystem {
    
    private final CANSparkMax leaderClimberMotor;
    private final CANSparkMax followerClimberMotor;
    
    public Climber() { 
        this.leaderClimberMotor = new CANSparkMax(18, MotorType.kBrushless);
        this.followerClimberMotor = new CANSparkMax(19, MotorType.kBrushless);

    }
    @Override
    public void close() throws Exception {
        leaderClimberMotor.close();
        followerClimberMotor.close();
    }
    @Override
    public void stopMotors() {
        leaderClimberMotor.stopMotor();
        followerClimberMotor.stopMotor();
    }

    public void climb(double speed) {
        leaderClimberMotor.set(speed);
}
}