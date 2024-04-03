package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArraySubscriber;
import util.misc.VisionPosition;

public class VisionIOSmartDashboard implements VisionIO {
    private StructArraySubscriber<VisionPosition> visionPositions;
    private DoubleSubscriber poseLatency;

    public VisionIOSmartDashboard() {
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        NetworkTable visionTable = ntinst.getTable("azathoth");
        this.visionPositions = visionTable.getStructArrayTopic("visionPos", VisionPosition.struct).subscribe(new VisionPosition[]{}, PubSubOption.periodic(0.02));
        this.poseLatency = visionTable.getDoubleTopic("visionLatency").subscribe(0.0, PubSubOption.periodic(0.02));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        VisionPosition[] positions = visionPositions.get();
        inputs.poses = new Translation2d[positions.length];
        inputs.tagIds = new int[positions.length];
        for (int i = 0; i < positions.length; i++) {
            VisionPosition position = positions[i];
            inputs.poses[i] = new Translation2d(position.x, position.y);
            inputs.tagIds[i] = position.ID;
        }
        inputs.poseLatency = poseLatency.get();
    }
}
