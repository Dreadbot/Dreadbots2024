package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import util.misc.DreadbotSubsystem;

public class Shooter extends DreadbotSubsystem {

    private final CANSparkMax leaderMotor;
    private final CANSparkMax followerMotor;

    public Shooter() {
        this.leaderMotor = new CANSparkMax(1, MotorType.kBrushless);
        this.followerMotor = new CANSparkMax(2, MotorType.kBrushless);
        this.leaderMotor.setInverted(true);
        this.followerMotor.follow(leaderMotor, true);
        
    }

    @Override
    public void close() throws Exception {
        leaderMotor.close();
        followerMotor.close();
    }

    @Override
    public void stopMotors() {
        leaderMotor.stopMotor();
        
    }

    public void shoot(double speed) {
        leaderMotor.set(speed);
    }
    

    
}
