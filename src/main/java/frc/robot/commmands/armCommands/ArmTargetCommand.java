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
<<<<<<< HEAD
    private final Drive drive;
    private double fixedAngle = 1.41372;
    private double armToEndOfPizza = .2413; // I don't know about this number you guys probably have to measure
    private double g = -9.8;
    private double deltaX = 0; // delta x and delta h are random values that are close enough to their actual
    private double hBase = .2; // should be height of arm pivet, I don't know this value
    private double deltaH = .8;
    private double targetH = 2.1336;
    private double armLength = .6096;
    private double speakerHoodOffset = 0.3048;
    private double originToBase = .2032;
    private Translation2d speakerPos = WaypointHelper.getSpeakerPos();
    private DriverStation.Alliance alliance = WaypointHelper.getAlliance();
    private double hNought;
    private double D;
    private double a;
    private double b;
    private double c; 
    private double horizontalAngle;
    private double armAngle;
    private double armRot;
    private Translation2d speakerHood;

    private double vNought = 16; // I don't know what you guys want this to be

    public ArmTargetCommand(Arm arm, Drive drive) {
=======
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
>>>>>>> c830c5e2e242c0dbc08489c2dd2b61cbc31f5138
        this.arm = arm;
        this.poseEstimator = poseEstimator;
        addRequirements(arm);
<<<<<<< HEAD
        if(alliance == DriverStation.Alliance.Red) {
            speakerHoodOffset = -0.3048;
=======
        if (alliance == DriverStation.Alliance.Red) {
            speakerHoodOffset = -speakerHoodOffset;
>>>>>>> c830c5e2e242c0dbc08489c2dd2b61cbc31f5138
        }
        speakerHood = new Translation2d(speakerPos.getX() + speakerHoodOffset, speakerPos.getY());
    }

    @Override
<<<<<<< HEAD
    public void execute(){
        Pose2d pos = drive.getPoseEstimator().getEstimatedPosition();
        double targetBoxX = Math.hypot(speakerHood.getX() - pos.getX(), speakerHood.getY() - pos.getY() - originToBase);

        for(int i = 0; i < 20; i++) {
            hNought = hBase + deltaH;
            D = targetBoxX + deltaX;
            a = -(g * (Math.pow(D, 2))) / (2 * (Math.pow(vNought, 2)));
            b = D;
            c = hNought + a - targetH;

            horizontalAngle = Math.atan((-b + Math.sqrt(Math.pow(b, 2) - 4*a*c))/(2 * a));
            armAngle = fixedAngle - horizontalAngle;
            deltaH = armLength * Math.sin(fixedAngle - horizontalAngle) + armToEndOfPizza * Math.sin(horizontalAngle);
            deltaX = armLength * Math.cos(armAngle);
        }
        armRot = armAngle / (2 * Math.PI);
        SmartDashboard.putNumber("Vision Target Command Angle", armRot);
        arm.setReference(new State(armRot, 0));
=======
    public void execute() {
        Pose2d pos = poseEstimator.getEstimatedPosition();
        double groundDistToSpeaker = Math.hypot(speakerHood.getX() - pos.getX(), speakerHood.getY() - pos.getY() - originToBase);
        double pivotToHoodTheta = Math.atan2(speakerHeight, groundDistToSpeaker);
        double distPivotToHood = Math.hypot(groundDistToSpeaker, speakerHeight);
        
        armAngle = Math.asin((armLength * Math.sin(angleBoxArm)) / distPivotToHood) + angleBoxArm - pivotToHoodTheta;
        arm.setReference(new State((armAngle - (targetingBias * groundDistToSpeaker)) / (2 * Math.PI), 0));
        SmartDashboard.putNumber("Vision Claculated Angle", armAngle);
    }

    @Override
    public boolean isFinished() {
        return arm.isAtDesiredState();
>>>>>>> c830c5e2e242c0dbc08489c2dd2b61cbc31f5138
    }
}