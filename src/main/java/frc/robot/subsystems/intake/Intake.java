package frc.robot.subsystems.intake;

import frc.robot.Constants;
import util.misc.DreadbotSubsystem;

public class Intake extends DreadbotSubsystem { 
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) { 
        if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void intake(double speed) {
        if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        io.set(speed);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    @Override
    public void close() throws Exception {
         if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
       io.close();
    }

    @Override
    public void stopMotors() {
         if(!Constants.SubsystemConstants.INTAKE_ENABLED) {
            return;
        }
        io.stopMotors();
    }
}

