package frc.robot.commmands.armCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import util.misc.WaypointHelper;

public class ArmTargetCommand<T extends WheelPositions<T>> extends Command {
    private final Arm arm;
    private final PoseEstimator<T> poseEstimator;
    private final double fixedAngle = 1.34390; // 1.41372
    private double armToEndOfPizza = .2413; // I don't know about this number you guys probably have to measure
    private double g = -9.8;
    private double deltaX = 0; // delta x and delta h are random values that are close enough to their actual
    private double hBase = .1778; // should be height of arm pivet, I don't know this value
    private double deltaH = .8;
    private double targetH = 2.1336;
    private double armLength = .6096;
    private double speakerHoodOffset = 0.3048;
    private double originToBase = .2032;
    private Translation2d speakerPos = WaypointHelper.getSpeakerPos();
    // private DriverStation.Alliance alliance = WaypointHelper.getAlliance();
    private double hNought;
    private double D;
    private double a;
    private double b;
    private double c; 
    private double horizontalAngle;
    private double armAngle;
    private double armRot;
    private Translation2d speakerHood;
    private final double targetBias = -0.007;

    private double vNought = (4 * Math.PI * 0.0254 * 4000) / 60; // I don't know what you guys want this to be

    public ArmTargetCommand(Arm arm, PoseEstimator<T> poseEstimator) {
        this.arm = arm;
        this.poseEstimator = poseEstimator;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        speakerHoodOffset = 0.3048 * (WaypointHelper.getAlliance() == DriverStation.Alliance.Red ? -1 : 1);
        speakerPos = WaypointHelper.getSpeakerPos();
        speakerHood = new Translation2d(speakerPos.getX() + speakerHoodOffset, speakerPos.getY());

        Pose2d pos = poseEstimator.getEstimatedPosition();
        double targetBoxX = Math.hypot(speakerHood.getX() - pos.getX(), speakerHood.getY() - pos.getY())- originToBase;

        for(int i = 0; i < 20; i++) {
            hNought = hBase + deltaH;
            D = targetBoxX + deltaX;
            a = -(g * (Math.pow(D, 2))) / (2 * (Math.pow(vNought, 2)));
            b = D;
            c = hNought + a - targetH;

            horizontalAngle = Math.atan2((-b + Math.sqrt(Math.pow(b, 2) - 4*a*c)),(2 * a));
            armAngle = fixedAngle - horizontalAngle;
            deltaH = armLength * Math.sin(fixedAngle - horizontalAngle) + armToEndOfPizza * Math.sin(horizontalAngle);
            deltaX = armLength * Math.cos(armAngle);
        }
        armRot = (armAngle / (2 * Math.PI)) + targetBias * targetBoxX;
        Logger.recordOutput("Vision Target Command Angle", armRot);
        Logger.recordOutput("Distance To Speaker Base", targetBoxX);
        Logger.recordOutput("Target Angle", armAngle);
        Logger.recordOutput("Target Box Height", hNought);
        arm.setReference(new State(armRot, 0));
    }

    @Override
    public boolean isFinished() {
        return arm.isAtDesiredState();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.setReference(new State(arm.getEncoderPosition(), 0));
        }
    }
}
