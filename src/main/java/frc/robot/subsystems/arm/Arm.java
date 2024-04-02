package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import util.math.DreadbotMath;
import util.misc.DreadbotSubsystem;

public class Arm extends DreadbotSubsystem {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs;

    private PIDController absolutePID;
    private boolean isArmInCoastMode = false;
    private boolean isUserButtonPressed = false;

    private TrapezoidProfile armProfile;
    private State armState;
    private State desiredArmState;
    public double joystickOverride;

    public Arm(ArmIO io) {
        this.io = io;
        if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }

        absolutePID = new PIDController(52.0, 30.0, 0.0);
        absolutePID.setIZone(0.02);
        absolutePID.setTolerance(ArmConstants.ARM_ENCODER_TOLERANCE);

        armProfile = new TrapezoidProfile(new Constraints(1.0 / 2.0, 1.0 / 1.5)); // very slow to start
        armState = new State(0, 0); //ARM ASSUMES IT STARTS DOWN!!!!
        desiredArmState = new State(0, 0);
    }

    @Override
    public void periodic() {
        if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }

        io.updateInputs(inputs);

        Logger.recordOutput("desired position", this.desiredArmState.position);
        Logger.recordOutput("Absolute Encoder Rotation", inputs.absolutePosition * 360);
        Logger.recordOutput("Absolute Encoder position", inputs.absolutePosition);
        Logger.recordOutput("At Setpoint", absolutePID.atSetpoint());
        Logger.recordOutput("Absolute PID Setpoint", absolutePID.getSetpoint());
        Logger.recordOutput("Armstate Position", armState.position);

        Logger.recordOutput("PID Error", absolutePID.getPositionError());

        if (Math.abs(joystickOverride) > 0.08) {
            // we should overrride with manual control
            //leftMotor.set(DreadbotMath.applyDeadbandToValue(joystickOverride, 0.08) * 0.2 * -1); // inverted joystick
            //this.armState = new State(DreadbotMath.clampValue(absoluteEncoder.get(), 0.0, 0.264), 0); // override the desired state with what the user wants
            //this.desiredArmState = new State(DreadbotMath.clampValue(absoluteEncoder.get(), 0.0, 0.264), 0); // override the desired state with what the user wants
            this.desiredArmState = new State(
                DreadbotMath.clampValue(
                    this.desiredArmState.position + joystickOverride * -0.00346,
                    0.000,
                    ArmConstants.ARM_UPPER_LIMIT
                ),
                0
            );
        }

        this.desiredArmState = new State(DreadbotMath.clampValue(desiredArmState.position, ArmConstants.ARM_LOWER_LIMIT, ArmConstants.ARM_UPPER_LIMIT), desiredArmState.velocity);
        this.armState = armProfile.calculate(0.02, armState, desiredArmState);
        absolutePID.setSetpoint(armState.position);
        double PIDoutput = absolutePID.calculate(inputs.absolutePosition);
        io.setVoltage(
            PIDoutput +
            Math.cos(Units.rotationsToRadians(inputs.absolutePosition)) * ArmConstants.KG
        );

        Logger.recordOutput("Is at position", this.isAtDesiredState());
        Logger.recordOutput("Motor controlled voltage", 
            PIDoutput +
            Math.cos(Units.rotationsToRadians(inputs.absolutePosition)) * ArmConstants.KG);
        Logger.recordOutput("PID Output", PIDoutput);
    }

    public void setIdleMode(IdleMode mode) {
        io.setIdleMode(mode);
    }

    
    @Override
    public void close() throws Exception {
        if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        io.close();
    }

    @Override
    public void stopMotors() {
        if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return;
        }
        io.stopMotors();
    }

    public double getEncoderPosition() {
         if (!Constants.SubsystemConstants.ARM_ENABLED) {
            return 0.0;
        }
        return inputs.absolutePosition;
    }

    public boolean isAtDesiredState() {
        double margin = DriverStation.isAutonomous() ? ArmConstants.ARM_POSITION_ERROR_MARGIN_AUTON : ArmConstants.ARM_POSITION_ERROR_MARGIN;
        return Math.abs(this.desiredArmState.position - this.getEncoderPosition()) < margin;
    }

    public void setReference(State target) {
        this.desiredArmState = target;
    }
    public void setJoystickOverride(double joystickOverride) {
        this.joystickOverride = joystickOverride;
    }
    public void setArmStartState() {
        this.armState = new State(inputs.absolutePosition, 0);
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
