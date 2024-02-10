package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import util.misc.DreadbotSubsystem;

public class Climber extends DreadbotSubsystem {
    
    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;
    private final DifferentialDrive climberDrive;
    private DigitalInput leftTopSwitch;
    private DigitalInput rightTopSwitch;
    private DigitalInput leftBottomSwitch;
    private DigitalInput rightBottomSwitch;
    private AHRS gyro;

    public Climber(AHRS gyro) { 
        this.leftClimberMotor = new CANSparkMax(1, MotorType.kBrushless);
        this.rightClimberMotor = new CANSparkMax(2, MotorType.kBrushless);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);

        this.gyro = gyro;

        rightClimberMotor.setInverted(true);
        this.climberDrive = new DifferentialDrive(leftClimberMotor, rightClimberMotor);

        this.leftTopSwitch = new DigitalInput(4);
        this.rightTopSwitch = new DigitalInput(2);
        this.leftBottomSwitch = new DigitalInput(3);
        this.rightBottomSwitch = new DigitalInput(1);

        climberDrive.setSafetyEnabled(false);
        climberDrive.setExpiration(.1);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro pitch", gyro.getPitch());
        SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());

        SmartDashboard.putBoolean("leftBottomSwitch", !leftBottomSwitch.get());
        SmartDashboard.putBoolean("rightBottomSwitch", !rightBottomSwitch.get());
        SmartDashboard.putBoolean("leftTopSwitch", !leftTopSwitch.get());
        SmartDashboard.putBoolean("rightTopSwitch", !rightTopSwitch.get());
    }

    public void climb(double vetricalSpeed, double rotationSpeed) {
        if((!leftBottomSwitch.get() || !rightBottomSwitch.get()) && vetricalSpeed < 0) {
            vetricalSpeed = 0;
        }


        if((!leftTopSwitch.get() || !rightTopSwitch.get()) && vetricalSpeed > 0) {
            vetricalSpeed = 0;
        }
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