package frc.robot.subystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import util.misc.DreadbotSubsystem;

public class Climber extends DreadbotSubsystem {
    
    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;
    
    public Climber() { 
        leftClimberMotor = new CANSparkMax(18, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(19, MotorType.kBrushless);

    }
    @Override
    public void close() throws Exception {
        leftClimberMotor.close();
        rightClimberMotor.close();
    }
    @Override
    public void stopMotors() {
        leftClimberMotor.stopMotor();
        rightClimberMotor.stopMotor();
    }
}
