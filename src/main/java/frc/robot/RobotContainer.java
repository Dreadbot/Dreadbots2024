// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commmands.armCommands.ArmCommand;
import frc.robot.commmands.armCommands.ArmTargetCommand;
import frc.robot.commmands.armCommands.ArmToPositionCommand;
import frc.robot.commmands.armCommands.SetArmIdleModeCommand;
import frc.robot.commmands.autonomousCommands.AutoShootCommand;
import frc.robot.commmands.autonomousCommands.AutoShootVisionCommand;
import frc.robot.commmands.climberCommands.ExtendClimbCommand;
import frc.robot.commmands.climberCommands.RetractClimbCommand;
import frc.robot.commmands.climberCommands.UnbindCommand;
import frc.robot.commmands.climberCommands.UnlockCommand;
import frc.robot.commmands.controllerCommands.EmergencyRumbleCommand;
import frc.robot.commmands.climberCommands.ClimbCommand;
import frc.robot.commmands.driveCommands.DriveCommand;
import frc.robot.commmands.driveCommands.LockonCommand;
import frc.robot.commmands.driveCommands.ResetGyroCommand;
import frc.robot.commmands.driveCommands.StopDriveCommand;
import frc.robot.subsystems.Climber;
import frc.robot.commmands.intakeCommands.FeedCommand;
import frc.robot.commmands.intakeCommands.IntakeCommand;
import frc.robot.commmands.intakeCommands.OuttakeCommand;
import frc.robot.commmands.intakeCommands.StopIntakeCommand;
import frc.robot.commmands.shooterCommands.ShootCommand;
import frc.robot.commmands.shooterCommands.StopShootCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOCAN;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOCAN;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOCAN;
import util.gyro.GyroIONavX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final DigitalInput toPi = new DigitalInput(9);
    
    private final XboxController primaryController = new XboxController(OperatorConstants.PRIMARY_JOYSTICK_PORT);
    private final XboxController secondaryController = new XboxController(OperatorConstants.SECONDARY_JOYSTICK_PORT);
    private final Drive drive;
    private final Climber climber;

    public final LoggedDashboardChooser<Command> autoChooser; 
    private final Shooter shooter;
    private final Intake intake; 
    private final Arm arm;
    private final PneumaticHub pneumaticHub;

    private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTable visionTable = ntInst.getTable("azathoth");
   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        drive = new Drive(visionTable, new GyroIONavX());
        drive.getGyroIO().reset();
        pneumaticHub = new PneumaticHub(21);
        pneumaticHub.enableCompressorDigital();
        climber = new Climber();
        shooter = new Shooter(new ShooterIOCAN());
        intake = new Intake(new IntakeIOCAN());
        arm = new Arm(new ArmIOCAN());

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            Logger.recordOutput("Auton Pose", pose);
        });

        configureButtonBindings();
        initializeAutonCommands();
        autoChooser = new LoggedDashboardChooser<>("Auton", AutoBuilder.buildAutoChooser());
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        DriveCommand driveCommand = new DriveCommand(drive, primaryController::getLeftX, primaryController::getLeftY, primaryController::getRightX);
        drive.setDefaultCommand(driveCommand);
        // new Trigger(primaryController::getSquareButton).whileTrue(new ExtendClimbCommand(climber));
        // new Trigger(primaryController::getTriangleButton).whileTrue(new RetractClimbCommand(climber, drive.getGyro()));
        new Trigger(primaryController::getStartButton).onTrue(new ResetGyroCommand(drive));
        new Trigger(() -> primaryController.getLeftTriggerAxis() > 0.50).whileTrue(
            new UnlockCommand(climber)
                .andThen(new UnbindCommand(climber, ClimberConstants.EXTEND_SPEED).withTimeout(0.02))
                .andThen(new WaitCommand(0.25))
                .andThen(new ExtendClimbCommand(climber))
        );
        new Trigger(() -> primaryController.getRightTriggerAxis() > 0.50).whileTrue(
            new UnlockCommand(climber)
                .andThen(new UnbindCommand(climber, ClimberConstants.RETRACT_SPEED).withTimeout(0.02))
                .andThen(new WaitCommand(0.25))
                .andThen(new ClimbCommand(climber, drive.getGyroInputs()))
        );
        new Trigger(primaryController::getRightBumper).whileTrue(
            new UnlockCommand(climber)
                .andThen(new UnbindCommand(climber, ClimberConstants.RETRACT_SPEED).withTimeout(0.02))
                .andThen(new WaitCommand(0.25))
                .andThen(new RetractClimbCommand(climber))
        );
        new Trigger(secondaryController::getStartButton).whileTrue(new EmergencyRumbleCommand(secondaryController));
        
        // new Trigger(primaryController::getL1Button).whileTrue(new TurtleCommand(driveCommand));
        new Trigger(secondaryController::getAButton).onTrue(new IntakeCommand(intake, primaryController));
        new Trigger(secondaryController::getAButton).onFalse(new StopIntakeCommand(intake));

        new Trigger(secondaryController::getBButton).onTrue(new OuttakeCommand(intake));
        new Trigger(secondaryController::getBButton).onFalse(new StopIntakeCommand(intake));
        
        new Trigger(secondaryController::getLeftBumper).whileTrue(new ArmTargetCommand<SwerveDriveWheelPositions>(arm, drive.getPoseEstimator()).repeatedly());
        ArmCommand armCommand = new ArmCommand(arm, secondaryController::getLeftY);
        arm.setDefaultCommand(armCommand);
        /* secondaryController.getRightBumper().onTrue(
            (new ShootCommand(shooter, 3750))
                .alongWith(new ArmToPositionCommand(arm, 0.08261))
                .until(() -> shooter.isAtSpeed() && arm.isAtDesiredState())
                .andThen(new FeedCommand(intake)
                .raceWith(new WaitCommand(0.4)))
                .andThen(new StopShootCommand(shooter))); */
        new Trigger(secondaryController::getRightBumper).whileTrue(new OuttakeCommand(intake).raceWith(new WaitCommand(0.04)).andThen(new ShootCommand(shooter, 5250, secondaryController)));
        new Trigger(secondaryController::getRightBumper).onFalse(new StopShootCommand(shooter));
        new Trigger(secondaryController::getBackButton).whileTrue(new ShootCommand(shooter, -2000, secondaryController));
        new Trigger(secondaryController::getBackButton).onFalse(new StopShootCommand(shooter));
        new Trigger(() -> secondaryController.getLeftTriggerAxis() > 0.50).whileTrue(new ShootCommand(shooter, 2000, secondaryController).alongWith(new FeedCommand(intake)));
        new Trigger(() -> secondaryController.getLeftTriggerAxis() > 0.50).onFalse(new StopShootCommand(shooter));
        new Trigger(secondaryController::getLeftBumper).whileTrue(new ArmTargetCommand<SwerveDriveWheelPositions>(arm, drive.getPoseEstimator()).repeatedly());
        //new Trigger(secondaryController::getLeftBumper).whileTrue(new ShootCommand(shooter, 4000, secondaryController));
        //new Trigger(secondaryController::getLeftBumper).onFalse(new StopShootCommand(shooter));
        new Trigger(() -> secondaryController.getRightTriggerAxis() > 0.50).whileTrue(new FeedCommand(intake));
        // new Trigger(secondaryController::getTriangleButton).whileTrue(new SourcePickupCommand(shooter));
        new Trigger(secondaryController::getYButton).whileTrue(new ShootCommand(shooter, 1700, secondaryController));
        new Trigger(secondaryController::getYButton).onFalse(new StopShootCommand(shooter));

        new Trigger(() -> secondaryController.getPOV() == 270).onTrue(new ArmToPositionCommand(arm, 0.0806, secondaryController::getLeftY)); //center note position: 0.11285, 
        new Trigger(() -> secondaryController.getPOV() == 90).onTrue(new ArmToPositionCommand(arm, 0.09908, secondaryController::getLeftY));
        new Trigger(primaryController::getAButton).whileTrue(new LockonCommand(drive));
        // new Trigger(() -> secondaryController.getPOV() == 0).onTrue(new ArmToPositionCommand(arm, ArmConstants.ARM_SOURCE_PICKUP_POSITION));
        new Trigger(() -> secondaryController.getPOV() == 180).onTrue(new ArmToPositionCommand(arm, 0.07, secondaryController::getLeftY)); //Trap Shot 6in chain from shooter, 0.06773 from climber lined up with chain, 0.06500 with 1700 rpm
        new Trigger(() -> secondaryController.getPOV() == 0).onTrue(new ArmToPositionCommand(arm, 0.253, secondaryController::getLeftY));

        new Trigger(shooter::overDrawingAmps).whileTrue(new EmergencyRumbleCommand(secondaryController));

        new Trigger(RobotController::getUserButton).whileTrue(new SetArmIdleModeCommand(arm));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command command = autoChooser.get();
        if (!command.getName().equals("1Note-Vision")) {
            drive.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(command.getName()));
        }
        return command;
    }

    public void teleopInit() {
        // drive.getGyro().setAngleAdjustment(-drive.getPosition().getRotation().getDegrees());
        arm.setReference(new State(arm.getEncoderPosition(), 0));
        arm.setArmStartState();
    }
    public void autonInit() {
        arm.setReference(new State(arm.getEncoderPosition(), 0));
        arm.setArmStartState();
    }

    public void initializeAutonCommands() {
        NamedCommands.registerCommand("Shoot-Subwoofer", new AutoShootCommand(intake, arm, shooter, 0.0806, 4000)); // .906
        NamedCommands.registerCommand("Shoot-MiddleNote", new AutoShootCommand(intake, arm, shooter, 0.12900, 4000));
        NamedCommands.registerCommand("Shoot-AmpSide", new AutoShootCommand(intake, arm, shooter, 0.1300, 4500));
        NamedCommands.registerCommand("Shoot-SourceSide", new AutoShootCommand(intake, arm, shooter, 0.132000, 4000));
        NamedCommands.registerCommand("Shoot-Vision", new AutoShootVisionCommand(intake, arm, shooter, drive, 5250));
        NamedCommands.registerCommand("RaiseArm", new ArmToPositionCommand(arm, 0.05, () -> 0));
        NamedCommands.registerCommand("DropArm", new ArmToPositionCommand(arm, ArmConstants.ARM_LOWER_LIMIT, () -> 0));
        NamedCommands.registerCommand("Stop", new StopDriveCommand(drive));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intake, null));
        NamedCommands.registerCommand("StopIntake", new StopIntakeCommand(intake));
        NamedCommands.registerCommand("Rev-Shooter", new InstantCommand(() -> {
            shooter.shoot(6500);
        }));
        NamedCommands.registerCommand("Stop-Shooter", new InstantCommand(() -> {
            shooter.shoot(0);
        }));
    }

    public void disabledPeriodic() {
        arm.disabledPeriodic();
    }
}

