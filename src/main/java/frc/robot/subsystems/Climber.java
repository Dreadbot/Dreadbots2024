package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SubsystemConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import util.misc.DreadbotSubsystem;

public class Climber extends DreadbotSubsystem {
    
    private CANSparkMax leftClimberMotor;
    private CANSparkMax rightClimberMotor;
    private DifferentialDrive climberDrive;
    private DigitalInput leftTopSwitch;
    private DigitalInput rightTopSwitch;
    private DigitalInput leftBottomSwitch;
    private DigitalInput rightBottomSwitch;
    private AHRS gyro;

    public Climber(AHRS gyro) { 
        if(!SubsystemConstants.CLIMBER_ENABLED) {
            return;
        }
        this.leftClimberMotor = new CANSparkMax(ClimberConstants.LEFT_CLIMB_MOTOR, MotorType.kBrushless);
        this.rightClimberMotor = new CANSparkMax(ClimberConstants.RIGHT_CLIMB_MOTOR, MotorType.kBrushless);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);

        this.gyro = gyro;

        rightClimberMotor.setInverted(true);
        this.climberDrive = new DifferentialDrive(leftClimberMotor, rightClimberMotor);

        this.leftTopSwitch = new DigitalInput(ClimberConstants.TOP_LEFT_LIMIT_SWITCH_ID);
        this.rightTopSwitch = new DigitalInput(ClimberConstants.TOP_RIGHT_LIMIT_SWITCH_ID);
        this.leftBottomSwitch = new DigitalInput(ClimberConstants.BOTTOM_LEFT_LIMIT_SWITCH_ID);
        this.rightBottomSwitch = new DigitalInput(ClimberConstants.BOTTOM_RIGHT_LIMIT_SWITCH_ID);

        climberDrive.setSafetyEnabled(false);
        climberDrive.setExpiration(.1);
    }
    // @Override
    // public void periodic() {
    //     SmartDashboard.putBoolean("leftBottomSwitch", !leftBottomSwitch.get());
    //     SmartDashboard.putBoolean("rightBottomSwitch", !rightBottomSwitch.get());
    //     SmartDashboard.putBoolean("leftTopSwitch", !leftTopSwitch.get());
    //     SmartDashboard.putBoolean("rightTopSwitch", !rightTopSwitch.get());
    // }

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
           getLeftClimberPosition(), getRightClimberPosition()
        };
    }

    public double getLeftClimberPosition() {
        return leftClimberMotor.getEncoder().getPosition();
    }

    public double getRightClimberPosition() {
        return rightClimberMotor.getEncoder().getPosition();
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