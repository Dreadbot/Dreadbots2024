package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import util.math.DreadbotMath;
import util.misc.DreadbotSubsystem;


public class Arm extends DreadbotSubsystem {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkPIDController leftPidController;
    private SparkPIDController rightPidController;

    private DigitalInput horizontalSwitch;
    private DigitalInput verticalSwitch;
    private BooleanEvent horizontalEvent;
    private BooleanEvent verticalEvent;
    private EventLoop limitSwitchEventLoop;
    private boolean horizontalSwitchCalibrated;
    private boolean isArmInCoastMode = false;
    private boolean isUserButtonPressed = false;
    private DutyCycleEncoder absoluteEncoder;

    private TrapezoidProfile armProfile;
    private State armState;
    private State desiredArmState;
    private double joystickOverride;

    public Arm() {
        if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        leftMotor = new CANSparkMax(ArmConstants.ARM_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ArmConstants.ARM_RIGHT_MOTOR, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(8));
        this.absoluteEncoder.setPositionOffset(0.9780);

        horizontalSwitch = new DigitalInput(ArmConstants.HORIZONTAL_ARM_SWITCH);
        verticalSwitch = new DigitalInput(ArmConstants.VERTICAL_ARM_SWITCH);

        limitSwitchEventLoop = new EventLoop();
        horizontalSwitchCalibrated = false;

        horizontalEvent = new BooleanEvent(limitSwitchEventLoop, this::getHorizontalLimitSwitch);
        verticalEvent = new BooleanEvent(limitSwitchEventLoop, this::getVerticalLimitSwitch);
        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor, true);

        leftPidController = leftMotor.getPIDController();
        rightPidController = rightMotor.getPIDController();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        
        leftMotor.getEncoder().setPositionConversionFactor(ArmConstants.ARM_GEAR_RATIO);
        rightMotor.getEncoder().setPositionConversionFactor(ArmConstants.ARM_GEAR_RATIO);

        leftMotor.getEncoder().setVelocityConversionFactor(ArmConstants.ARM_GEAR_RATIO / 60); // rpm -> rps
        rightMotor.getEncoder().setVelocityConversionFactor(ArmConstants.ARM_GEAR_RATIO / 60); // rpm -> rps


        leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, 0.265f);
        leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, 0.005f);

        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

        armProfile = new TrapezoidProfile(new Constraints(1.0 / 4.0, 1.0 / 5.0)); // very slow to start
        armState = new State(0, 0); //ARM ASSUMES IT STARTS DOWN!!!!
        desiredArmState = new State(0, 0);

        leftPidController.setP(8);
        leftPidController.setI(0.0);
        leftPidController.setD(0.0);
        horizontalEvent
            .and(() -> (Math.signum(leftMotor.getEncoder().getVelocity()) < 0 && !horizontalSwitchCalibrated))
            .ifHigh(() -> { 
            horizontalSwitchCalibrated = true;
            leftMotor.getEncoder().setPosition(0.0076);
            });
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }
    @Override
    public void periodic() {
        if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        this.desiredArmState = new State(DreadbotMath.clampValue(desiredArmState.position, 0.0, 0.264), desiredArmState.velocity);
        this.armState = armProfile.calculate(0.02, armState, desiredArmState);

        if (Math.abs(joystickOverride) > 0.08) {
            //we should overrride with manual control
            leftMotor.set(DreadbotMath.applyDeadbandToValue(joystickOverride, 0.08) * 0.2 * -1); //inverted joystick
            this.armState = new State(DreadbotMath.clampValue(leftMotor.getEncoder().getPosition(), 0.0, 0.264), 0); //override the desired state with what the user wants
            this.desiredArmState = new State(DreadbotMath.clampValue(leftMotor.getEncoder().getPosition(), 0.0, 0.264), 0); //override the desired state with what the user wants
        } else {
            leftPidController.setReference(armState.position, ControlType.kPosition, 0, Math.cos(Units.rotationsToRadians(leftMotor.getEncoder().getPosition())) * ArmConstants.KG);
        }
        SmartDashboard.putNumber("Encoder position", this.leftMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Lower limit switch triggered", getHorizontalLimitSwitch());
        SmartDashboard.putBoolean("Upper limit switch triggered", getVerticalLimitSwitch());
        SmartDashboard.putBoolean("Is at position", this.isAtDesiredState());
        SmartDashboard.putNumber("Absolute Encoder", this.absoluteEncoder.getAbsolutePosition());

        limitSwitchEventLoop.poll();
    }

    public void setIdleMode(IdleMode mode) {
        leftMotor.setIdleMode(mode);
        rightMotor.setIdleMode(mode);
    }

    
    @Override
    public void close() throws Exception {
        if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        leftMotor.close();
        rightMotor.close();
    }

    @Override
    public void stopMotors() {
        if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        leftMotor.stopMotor();
        rightMotor.stopMotor();

    }

    public double getEncoderPosition() {
         if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return 0.0;
        }
        return leftMotor.getEncoder().getPosition();
    }

    public boolean isAtDesiredState() {
        return Math.abs(this.desiredArmState.position - this.armState.position) < Constants.ArmConstants.ARM_POSITION_ERROR_MARGIN;
    }

    public void setReference(State target) {
        this.desiredArmState = target;
    }
    public void setJoystickOverride(double joystickOverride) {
        this.joystickOverride = joystickOverride;
    }
    public void setArmStartState() {
        this.armState = new State(this.leftMotor.getEncoder().getPosition(), 0);
    }
    public void overrideArmState(double rotations) {
        this.leftMotor.getEncoder().setPosition(rotations);
        this.armState = new State(rotations, 0);
        this.desiredArmState = this.armState;
    }

    public boolean getHorizontalLimitSwitch() {
        return !this.horizontalSwitch.get();
    }
    public boolean getVerticalLimitSwitch() {
        return !this.verticalSwitch.get();
    }
    public void disabledPeriodic() {
        if(RobotController.getUserButton() && !isArmInCoastMode && !isUserButtonPressed) {
            isUserButtonPressed = true;
            isArmInCoastMode = true;
            setIdleMode(IdleMode.kCoast);
        } else if (RobotController.getUserButton() && isArmInCoastMode && !isUserButtonPressed) {
            isUserButtonPressed = true;
            isArmInCoastMode = false;
            setIdleMode(IdleMode.kBrake);
        } else if(!RobotController.getUserButton()) {
            isUserButtonPressed = false;
        }

    }
}
