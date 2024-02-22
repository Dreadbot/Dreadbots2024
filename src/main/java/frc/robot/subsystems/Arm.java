package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private DigitalInput horizontalSwtich;
    private DigitalInput verticalSwtich;

    private TrapezoidProfile armProfile;
    private State armState;
    private State desiredArmState;
    private double joystickOverride;

    public Arm() {
        if(!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        leftMotor = new CANSparkMax(13, MotorType.kBrushless);
        rightMotor = new CANSparkMax(14, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        horizontalSwtich = new DigitalInput(1);
        verticalSwtich = new DigitalInput(2);

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


        leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, 0.25f);
        leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, 0.006f);

        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

        armProfile = new TrapezoidProfile(new Constraints(1.0 / 4.0, 1.0 / 5.0)); // very slow to start
        armState = new State(0, 0); //ARM ASSUMES IT STARTS DOWN!!!!
        desiredArmState = new State(0, 0);

        leftPidController.setP(8);
        leftPidController.setI(0.0);
        leftPidController.setD(0.0);

    }

    @Override
    public void periodic() {
        if(!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        this.desiredArmState = new State(DreadbotMath.clampValue(desiredArmState.position, 0.0, 0.25), desiredArmState.velocity);
        this.armState = armProfile.calculate(0.02, armState, desiredArmState);

        if(Math.abs(joystickOverride) > 0.06) {
            //we should overrride with manual control
            leftMotor.set(DreadbotMath.applyDeadbandToValue(joystickOverride, 0.06) * 0.2 * -1); //inverted joystick
            System.out.println("joystick override: " + joystickOverride);
            this.armState = new State(DreadbotMath.clampValue(leftMotor.getEncoder().getPosition(), 0.0, 0.25), 0); //override the desired state with what the user wants
            this.desiredArmState = new State(DreadbotMath.clampValue(leftMotor.getEncoder().getPosition(), 0.0, 0.25), 0); //override the desired state with what the user wants
        } else {
             leftPidController.setReference(armState.position, ControlType.kPosition, 0, Math.cos(Units.rotationsToRadians(leftMotor.getEncoder().getPosition())) * ArmConstants.KG);
        }
        SmartDashboard.putNumber("Encoder position", this.leftMotor.getEncoder().getPosition());


        // check limit switches and stop motor
        if(getHorizontalLimitSwitch()) {
            this.leftMotor.getEncoder().setPosition(0);
        }
        if(getVerticalLimitSwitch()) {
            this.leftMotor.getEncoder().setPosition(0.25);
        }
        
    }

    @Override
    public void close() throws Exception {
        if(!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        leftMotor.close();
        rightMotor.close();
    }

    @Override
    public void stopMotors() {
        if(!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        leftMotor.stopMotor();
        rightMotor.stopMotor();

    }

    public double getEncoderPosition() {
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
        SmartDashboard.putNumber("Joystick override", joystickOverride);
    }
    public void setArmStartState() {
        this.armState = new State(this.leftMotor.getEncoder().getPosition(), 0);
         
    }
    public boolean getHorizontalLimitSwitch() {
        return !this.horizontalSwtich.get();
    }
    public boolean getVerticalLimitSwitch() {
        return !this.verticalSwtich.get();
    }
}
