package util.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import util.math.DreadbotMath;
import util.math.Vector2D;

public class VisionIntegration {
    public static final Pose2d[] tagVectors = new Pose2d[] {
        new Pose2d(DreadbotMath.inchesToMeters(593.68), DreadbotMath.inchesToMeters(9.68), Rotation2d.fromDegrees(120)),
        new Pose2d(DreadbotMath.inchesToMeters(637.21), DreadbotMath.inchesToMeters(34.79), Rotation2d.fromDegrees(120)),
        new Pose2d(DreadbotMath.inchesToMeters(652.73), DreadbotMath.inchesToMeters(196.17), Rotation2d.fromDegrees(180)),
        new Pose2d(DreadbotMath.inchesToMeters(652.73), DreadbotMath.inchesToMeters(218.42), Rotation2d.fromDegrees(180)),
        new Pose2d(DreadbotMath.inchesToMeters(578.77), DreadbotMath.inchesToMeters(323.00), Rotation2d.fromDegrees(270)),
        new Pose2d(DreadbotMath.inchesToMeters(72.5), DreadbotMath.inchesToMeters(323.00), Rotation2d.fromDegrees(270)),
        new Pose2d(DreadbotMath.inchesToMeters(-1.50), DreadbotMath.inchesToMeters(218.42), Rotation2d.fromDegrees(0)),
        new Pose2d(DreadbotMath.inchesToMeters(-1.50), DreadbotMath.inchesToMeters(196.17), Rotation2d.fromDegrees(0)),
        new Pose2d(DreadbotMath.inchesToMeters(14.02), DreadbotMath.inchesToMeters(34.79), Rotation2d.fromDegrees(60)),
        new Pose2d(DreadbotMath.inchesToMeters(57.54), DreadbotMath.inchesToMeters(9.68), Rotation2d.fromDegrees(60)),
        new Pose2d(DreadbotMath.inchesToMeters(468.69), DreadbotMath.inchesToMeters(146.19), Rotation2d.fromDegrees(300)),
        new Pose2d(DreadbotMath.inchesToMeters(468.69), DreadbotMath.inchesToMeters(177.10), Rotation2d.fromDegrees(60)),
        new Pose2d(DreadbotMath.inchesToMeters(441.74), DreadbotMath.inchesToMeters(161.62), Rotation2d.fromDegrees(180)),
        new Pose2d(DreadbotMath.inchesToMeters(209.48), DreadbotMath.inchesToMeters(161.62), Rotation2d.fromDegrees(0)),
        new Pose2d(DreadbotMath.inchesToMeters(182.73), DreadbotMath.inchesToMeters(177.10), Rotation2d.fromDegrees(120)),
        new Pose2d(DreadbotMath.inchesToMeters(182.73), DreadbotMath.inchesToMeters(146.19), Rotation2d.fromDegrees(240)),
    };
    public static Pose2d currentPoseInAuton = new Pose2d();

    public static Pose2d getApriltagPose(int tagId) {
        return tagVectors[tagId - 1];
    }

    public static Pose2d robotToWorldFrame(double x, double y, double rotation) {
        Vector2D robotFrame = new Vector2D(x, y).rotate(rotation);
        return new Pose2d(robotFrame.x1, robotFrame.x2, new Rotation2d(rotation));
    }

    public static Pose2d worldToRobotFromWorldFrame(Pose2d worldFrame, int tagId) {
        return new Pose2d(getApriltagPose(tagId).getTranslation().minus(worldFrame.getTranslation()), worldFrame.getRotation());
    }

    public static Rotation2d getAllianceAgnosticRotation(Rotation2d rotation) {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return rotation.rotateBy(Rotation2d.fromDegrees(180));
        }
        return rotation;
    }

    public static int getAllianceAgnosticInversion() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return 1;
        }
        return -1;
    }
}
