package frc.robot.subsystems.climber;

import frc.robot.Constants.SubsystemConstants;
import util.misc.DreadbotSubsystem;

public class Climber extends DreadbotSubsystem {
    private ClimberIO io;
    private ClimberIOInputAutoLogged inputs;

    public Climber(ClimberIO io) { 
        if(!SubsystemConstants.CLIMBER_ENABLED) {
            return;
        }
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void climb(double verticalSpeed, double rotationSpeed) {
        if((inputs.leftBottomSwitch || inputs.rightBottomSwitch) && verticalSpeed < 0) {
            verticalSpeed = 0;
        }

        if((inputs.leftTopSwitch || inputs.rightBottomSwitch) && verticalSpeed > 0) {
            verticalSpeed = 0;
        }
        io.arcade(verticalSpeed, rotationSpeed);
    }

    public void retract(double speed) {
        double leftSpeed = speed;
        double rightSpeed = speed;

        if(inputs.leftTopSwitch && speed < 0) {
            leftSpeed = 0;
        }
        if(inputs.rightTopSwitch && speed < 0) {
            rightSpeed = 0;
        }   

        if((inputs.leftTopSwitch || inputs.rightTopSwitch) && speed > 0) {
            leftSpeed = 0;
            rightSpeed = 0;
        }

        io.tank(leftSpeed, rightSpeed);
        
    }

    /**
    * Returns the climber positions in a double array
    * @return Array of double values: [left, right]
     */
    public double[] getClimberPositions() {
        return new double[] {
           getLeftClimberPosition(), getRightClimberPosition()
        };
    }

    public double getLeftClimberPosition() {
        return inputs.leftPosition;
    }

    public double getRightClimberPosition() {
        return inputs.rightPosition;
    }

    public void lock() {
        io.setPiston(false);
    }

    public void unlock() {
        io.setPiston(true);
    }

    @Override
    public void close() throws Exception {
        io.close();
    }
    @Override
    public void stopMotors() {
        io.stopMotors();
    }
}