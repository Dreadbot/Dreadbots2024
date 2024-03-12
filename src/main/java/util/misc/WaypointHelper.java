package util.misc;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class WaypointHelper {

    public static Translation2d getSpeakerPos() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return new Translation2d(16.579342, 5.547868);
        }
        return new Translation2d(-0.0381, 5.547868);
    }
}
