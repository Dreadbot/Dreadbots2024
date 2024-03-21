package frc.robot.commmands.armCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import util.misc.WaypointHelper;

public class ArmTargetCommand extends Command {
    private final Arm arm;
    private final PoseEstimator poseEstimator;
    private double originToBase = .2032;
    private Translation2d speakerPos = WaypointHelper.getSpeakerPos();
    private DriverStation.Alliance alliance = WaypointHelper.getAlliance();
    private double speakerHoodOffset = 0.3048;
    private double speakerHeight = 2.1336;
    private final double angleBoxArm = 1.3788 ;// 1.41372
    private double armLength = .6096;
    private double armAngle;
    private double targetingBias = -0.01;
    
    private Translation2d speakerHood;
    public ArmTargetCommand(Arm arm, PoseEstimator poseEstimator) {
        this.arm = arm;
        this.poseEstimator = poseEstimator;
        addRequirements(arm);
        speakerHood = new Translation2d(speakerPos.getX() + speakerHoodOffset, speakerPos.getY());
    }

    @Override
    public void execute() {
        speakerPos = WaypointHelper.getSpeakerPos();
        if (alliance == DriverStation.Alliance.Red) {
            speakerHoodOffset = -Math.abs(speakerHoodOffset);
        }
        Pose2d pos = poseEstimator.getEstimatedPosition();
        double groundDistToSpeaker = Math.hypot(speakerHood.getX() - pos.getX(), speakerHood.getY() - pos.getY()) - originToBase;
        double pivotToHoodTheta = Math.atan2(speakerHeight, groundDistToSpeaker);
        double distPivotToHood = Math.hypot(groundDistToSpeaker, speakerHeight);
        SmartDashboard.putNumber("Distance Pivot to Hood", distPivotToHood);
        
        armAngle = Math.asin((armLength * Math.sin(angleBoxArm)) / distPivotToHood) + angleBoxArm - pivotToHoodTheta;
        arm.setReference(new State((armAngle - (targetingBias * groundDistToSpeaker)) / (2 * Math.PI), 0));
        SmartDashboard.putNumber("Vision Claculated Angle", armAngle);
    }

    @Override
    public boolean isFinished() {
        return arm.isAtDesiredState();
    }
}
