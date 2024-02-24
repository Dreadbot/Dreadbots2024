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
    private final DoubleSubscriber thetaSub;
    private final BooleanSubscriber tagSeenSub;
    private final DoubleSubscriber robotPosXSub;
    private final DoubleSubscriber robotPosZSub;

    public LockonCommand(Drive drive, NetworkTable table) {
        this.drive = drive;
        this.thetaSub = table.getDoubleTopic("thetaToTag").subscribe(0.0);
        this.tagSeenSub = table.getBooleanTopic("tagSeen").subscribe(false);
        this.robotPosXSub = table.getDoubleTopic("robotposXFromTag").subscribe(0.0);
        this.robotPosZSub = table.getDoubleTopic("robotposZFromTag").subscribe(0.0);
    }
    @Override
    public void execute() {
        drive.doLockon = true;
        boolean tagSeen = tagSeenSub.get();
        if (tagSeen) {
            drive.lockonInitialRobotPosX = robotPosXSub.get();
            drive.lockonInitialRobotPosZ = robotPosZSub.get();
            drive.lockonTarget = Math.toRadians(drive.getGyro().getYaw()) + thetaSub.get();
        }
    }
    @Override
    public void end(boolean isCancelled){
        drive.doLockon = false;
    }
}
