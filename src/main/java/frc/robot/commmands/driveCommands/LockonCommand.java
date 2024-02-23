package frc.robot.commmands.driveCommands;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

import frc.robot.subsystems.Drive;
import util.math.DreadbotMath; 

public class LockonCommand extends Command {
    private final Drive drive;
    private final DoubleSubscriber thetaTopic;

    public LockonCommand(Drive drive, DoubleSubscriber doubleSubscriber) {
        this.drive = drive;
        this.thetaTopic = doubleSubscriber;
    }
    @Override
    public void execute() {
        double theta = thetaTopic.get();
        SmartDashboard.putNumber("Theta", theta);
        drive.lockRotationOverride = theta;
    }
    @Override
    public void end(boolean isCanceled){
        drive.lockRotationOverride = 0;
    }
}
