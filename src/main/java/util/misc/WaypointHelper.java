package util.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class WaypointHelper {

    public static Translation2d getSpeakerPos() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return new Translation2d(16.579342, 5.547868);
        }
        return new Translation2d(-0.0381, 5.547868);
    }

    public static Pose2d getResetPose() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return new Pose2d(new Translation2d(15.25, 5.54), new Rotation2d(Units.degreesToRadians(180)));
        }
        return new Pose2d(new Translation2d(1.36, 5.54), new Rotation2d());
    }
    
    public static DriverStation.Alliance getAlliance() {
        if(DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get();
        }
        return DriverStation.Alliance.Red;
    }
}
