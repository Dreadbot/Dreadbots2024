package util.vision;



import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import util.misc.TimestampedPose2d;

public class VisionInterface {
    private NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private NetworkTable ntTable = ntInstance.getTable("SmartDashboard");
    private DoubleSubscriber robotPoseX = ntTable.getDoubleTopic("robotposX").subscribe(0.0);
    private DoubleSubscriber robotPoseZ = ntTable.getDoubleTopic("robotposZ").subscribe(0.0);
    private DoubleSubscriber robotPoseTheta = ntTable.getDoubleTopic("posTheta").subscribe(1000); // Change default to something better later


    public Pose2d getVisionPose() {
        double robotX = robotPoseX.get();
        double robotZ = robotPoseZ.get();
        double robotTheta = robotPoseTheta.get();
        
        if(robotX == 0 || robotZ == 0 || robotTheta == 1000) {
            return new Pose2d();
        }

        return new Pose2d(new Translation2d(robotX, robotZ), new Rotation2d(robotTheta));
    }

    public TimestampedPose2d getTimestampedVisionPose() {
        TimestampedDouble tsValX = robotPoseX.getAtomic();
        TimestampedDouble tsValZ = robotPoseZ.getAtomic();
        TimestampedDouble tsValTheta = robotPoseTheta.getAtomic();

        if(tsValX.value == 0 || tsValZ.value == 0 || tsValTheta.value == 1000) {
            return new TimestampedPose2d(new Pose2d(), tsValX.timestamp);
        }

        return new TimestampedPose2d(new Pose2d(new Translation2d(tsValX.value, tsValZ.value), new Rotation2d(tsValTheta.value)), tsValX.timestamp);
    }


}