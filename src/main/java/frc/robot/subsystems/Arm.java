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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import util.math.DreadbotMath;
import util.misc.DreadbotSubsystem;


public class Arm extends DreadbotSubsystem {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkPIDController leftPidController;
    private SparkPIDController rightPidController;
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
        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor);

        leftPidController = leftMotor.getPIDController();
        rightPidController = rightMotor.getPIDController();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        
        leftMotor.getEncoder().setPositionConversionFactor(ArmConstants.ARM_GEAR_RATIO);
        rightMotor.getEncoder().setPositionConversionFactor(ArmConstants.ARM_GEAR_RATIO);

        leftMotor.getEncoder().setVelocityConversionFactor(ArmConstants.ARM_GEAR_RATIO / 60); // rpm -> rps
        rightMotor.getEncoder().setVelocityConversionFactor(ArmConstants.ARM_GEAR_RATIO / 60); // rpm -> rps

        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

        // rightMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        // rightMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);





        armProfile = new TrapezoidProfile(new Constraints(0.25, 1 / 8)); // very slow to start
        armState = new State(); //ARM ASSUMES IT STARTS DOWN!!!!

        leftPidController.setP(0.2);
        leftPidController.setI(0.0);
        leftPidController.setD(0.0);

    }

    @Override
    public void periodic() {
        this.armState = armProfile.calculate(0.02, armState, desiredArmState);

        leftPidController.setReference(armState.position, ControlType.kPosition);
        //check limit switches and stop motor
        if(DreadbotMath.applyDeadbandToValue(joystickOverride, 0.08) > 0) {
            //we should overrride with manual control
            leftMotor.set(joystickOverride);
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
    public void setReference(State target) {
        this.desiredArmState = target;
    }
    public void setJoystickOverride(double joystickOverride) {
        this.joystickOverride = joystickOverride;
    }
}
