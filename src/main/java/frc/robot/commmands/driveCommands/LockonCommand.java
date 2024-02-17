package frc.robot.commmands.driveCommands;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import util.math.DreadbotMath; 

public class LockonCommand extends Command {
    private final Drive drive;
    private final DoubleSubscriber sub;

    public LockonCommand(Drive drive, DoubleSubscriber sub) {
        this.drive = drive;
        this.sub = sub;
    }

    public void execute() {
        double theta = sub.get();
        drive.lockRotationOverride = theta;
    }

    public void end() {
        sub.close();
    }
}
