package util.misc;

import edu.wpi.first.math.geometry.Pose2d;

public class TimestampedPose2d extends Pose2d {
    
    private final Pose2d pose;
    private final double timestamp;

    public TimestampedPose2d(Pose2d pose, double timestamp) {
        super(pose.getTranslation(), pose.getRotation());
        this.pose = pose;
        this.timestamp = timestamp;
    }

    public double getTimestamp() {
        return this.timestamp;
    }

}
