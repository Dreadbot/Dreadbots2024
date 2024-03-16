package frc.robot.commmands.armCommands;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
    private final Drive drive;
    private double distanceToSpeakerBase;
    private double originToBase = .2032;
    private Translation2d speakerPos = WaypointHelper.getSpeakerPos();
    private DriverStation.Alliance alliance = WaypointHelper.getAlliance();
    private double speakerHoodOffset = 0.3048;
    private double speakerHeight = 2.1336;
    private double angleBoxArm = 1.41372;
    private double armLength = .6096;
    private double armAngle;
    
    private Translation2d speakerHood;
    public ArmTargetCommand(Arm arm, Drive drive) {
        this.arm = arm;
        this.drive = drive;
        addRequirements(arm);
        if (alliance == DriverStation.Alliance.Red) {
            speakerHoodOffset = -0.3048;
        }
        speakerHood = new Translation2d(speakerPos.getX() + speakerHoodOffset, speakerPos.getY());
    }

    @Override
    public void execute() { 
        Pose2d pos = drive.getPoseEstimator().getEstimatedPosition();
        double groundDistToSpeaker = Math.hypot(speakerHood.getX() - pos.getX(), speakerHood.getY() - pos.getY() - 0.2032);
        double pivotToHoodTheta = Math.atan2(speakerHeight, groundDistToSpeaker);
        double distPivotToHood = Math.hypot(groundDistToSpeaker, speakerHeight);
        
        armAngle = Math.asin((armLength * Math.sin(angleBoxArm)) / distPivotToHood) + angleBoxArm - pivotToHoodTheta;
        arm.setReference(new State(armAngle, 0));
    }
}
