package frc.robot.commmands.driveCommands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

import frc.robot.subsystems.Drive;
import util.math.DreadbotMath; 

public class LockonCommand extends Command {
    private final Drive drive;

    public LockonCommand(Drive drive) {
        this.drive = drive;
    }
    @Override
    public void execute() {
        drive.doLockon = true;
    }
    @Override
    public void end(boolean isCancelled){
        drive.doLockon = false;
    }
}
