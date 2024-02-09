package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;

import util.misc.DreadbotSubsystem;

public class Climber extends DreadbotSubsystem {
    
    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;
    private final DifferentialDrive climberDrive;
    
    public Climber() { 
        this.leftClimberMotor = new CANSparkMax(1, MotorType.kBrushless);
        this.rightClimberMotor = new CANSparkMax(2, MotorType.kBrushless);
        rightClimberMotor.setInverted(true);
        this.climberDrive = new DifferentialDrive(leftClimberMotor, rightClimberMotor);

        climberDrive.setSafetyEnabled(false);
        climberDrive.setExpiration(.1);
    }

    public void climb(double vetricalSpeed, double rotationSpeed) {

        climberDrive.arcadeDrive(vetricalSpeed, rotationSpeed, false); //false for not squaring inputs
        climberDrive.feed();
    }

    /**
    * Returns the climber positions in a double array
    * @returns Array of double values: [left, right]
     */
    public double[] getClimberPositions() {
        return new double[] {
            leftClimberMotor.getEncoder().getPosition(),
            rightClimberMotor.getEncoder().getPosition()
        };
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