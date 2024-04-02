package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOCAN implements ClimberIO {
    private CANSparkMax leftClimberMotor;
    private CANSparkMax rightClimberMotor;
    private Solenoid lockPiston;
    private DifferentialDrive climberDrive;

    private DigitalInput leftTopSwitch;
    private DigitalInput leftBottomSwitch;
    private DigitalInput rightTopSwitch;
    private DigitalInput rightBottomSwitch;

    public ClimberIOCAN() {
        this.leftClimberMotor = new CANSparkMax(ClimberConstants.LEFT_CLIMB_MOTOR, MotorType.kBrushless);
        this.rightClimberMotor = new CANSparkMax(ClimberConstants.RIGHT_CLIMB_MOTOR, MotorType.kBrushless);

        this.leftClimberMotor.setIdleMode(IdleMode.kBrake);
        this.rightClimberMotor.setIdleMode(IdleMode.kBrake);

        this.leftClimberMotor.setSmartCurrentLimit(80);
        rightClimberMotor.setSmartCurrentLimit(80);


        this.lockPiston = new Solenoid(21, PneumaticsModuleType.REVPH, 9);

        // this.gyro = gyro;

        this.rightClimberMotor.setInverted(true);
        this.climberDrive = new DifferentialDrive(leftClimberMotor, rightClimberMotor);

        this.leftTopSwitch = new DigitalInput(ClimberConstants.TOP_LEFT_LIMIT_SWITCH_ID);
        this.rightTopSwitch = new DigitalInput(ClimberConstants.TOP_RIGHT_LIMIT_SWITCH_ID);
        this.leftBottomSwitch = new DigitalInput(ClimberConstants.BOTTOM_LEFT_LIMIT_SWITCH_ID);
        this.rightBottomSwitch = new DigitalInput(ClimberConstants.BOTTOM_RIGHT_LIMIT_SWITCH_ID);

        this.climberDrive.setSafetyEnabled(false);
        this.climberDrive.setExpiration(.1);
    }

    @Override
    public void updateInputs(ClimberIOInput inputs) {
        inputs.leftTopSwitch = !leftTopSwitch.get();
        inputs.leftBottomSwitch = !leftBottomSwitch.get();
        inputs.rightTopSwitch = !rightTopSwitch.get();
        inputs.rightBottomSwitch = !rightBottomSwitch.get();

        inputs.leftPosition = leftClimberMotor.getEncoder().getPosition();
        inputs.leftVoltage = leftClimberMotor.getAppliedOutput() * leftClimberMotor.getBusVoltage();
        inputs.leftCurrent = leftClimberMotor.getOutputCurrent();
        inputs.leftTemperature = leftClimberMotor.getMotorTemperature();

        inputs.rightPosition = rightClimberMotor.getEncoder().getPosition();
        inputs.rightVoltage = rightClimberMotor.getAppliedOutput() * rightClimberMotor.getBusVoltage();
        inputs.rightCurrent = rightClimberMotor.getOutputCurrent();
        inputs.rightTemperature = rightClimberMotor.getMotorTemperature();
    }

    @Override
    public void arcade(double verticalSpeed, double rotationSpeed) {
        climberDrive.arcadeDrive(verticalSpeed, rotationSpeed, false); //false for not squaring inputs
        climberDrive.feed();
    }

    @Override
    public void tank(double leftSpeed, double rightSpeed) {
        climberDrive.tankDrive(leftSpeed, rightSpeed);
        climberDrive.feed();
    }

    @Override
    public void setPiston(boolean state) {
        lockPiston.set(state);
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
