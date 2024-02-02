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
        this.leftClimberMotor = new CANSparkMax(18, MotorType.kBrushless);
        this.rightClimberMotor = new CANSparkMax(19, MotorType.kBrushless);
        this.climberDrive = new DifferentialDrive(leftClimberMotor, rightClimberMotor);

    }

    public void climb(double vetricalSpeed, double rotationSpeed) {

        climberDrive.arcadeDrive(vetricalSpeed, rotationSpeed, false); //false for not squaring inputs
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