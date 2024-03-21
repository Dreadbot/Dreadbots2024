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
        this.arm = arm;
        this.drive = drive;
        addRequirements(arm);
        if(alliance == DriverStation.Alliance.Red) {
            speakerHoodOffset = -0.3048;
        }
        speakerHood = new Translation2d(speakerPos.getX() + speakerHoodOffset, speakerPos.getY());
    }

    @Override
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
    }
}