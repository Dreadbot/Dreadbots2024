// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.PS4ControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commmands.armCommands.ArmCommand;
import frc.robot.commmands.armCommands.ArmToPositionCommand;
import frc.robot.commmands.autonomousCommands.AutoShootCommand;
import frc.robot.commmands.climberCommands.ExtendClimbCommand;
import frc.robot.commmands.climberCommands.RetractClimbCommand;
import frc.robot.commmands.climberCommands.ClimbCommand;
import frc.robot.commmands.driveCommands.DriveCommand;
import frc.robot.commmands.driveCommands.ResetGyroCommand;
import frc.robot.commmands.driveCommands.StopDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.commmands.intakeCommands.FeedCommand;
import frc.robot.commmands.intakeCommands.IntakeCommand;
import frc.robot.commmands.intakeCommands.OuttakeCommand;
import frc.robot.commmands.intakeCommands.StopIntakeCommand;
import frc.robot.commmands.shooterCommands.ShootCommand;
import frc.robot.commmands.shooterCommands.SourceIntakeCommand;
import frc.robot.commmands.shooterCommands.SourcePickupCommand;
import frc.robot.commmands.shooterCommands.StopShootCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import util.controls.DreadbotController;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    
    private final PS4Controller primaryController = new PS4Controller(OperatorConstants.PRIMARY_JOYSTICK_PORT);
    private final DreadbotController secondaryController = new DreadbotController(OperatorConstants.SECONDARY_JOYSTICK_PORT);
    private final Drive drive;
    private final Climber climber;

    public final SendableChooser<Command> autoChooser; 
    private final Shooter shooter;
    private final Intake intake; 
    private final Arm arm;
    private final PneumaticHub pneumaticHub;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        drive = new Drive();
        drive.getGyro().reset();
        pneumaticHub = new PneumaticHub(21);
        pneumaticHub.enableCompressorDigital();
        climber = new Climber(drive.getGyro());
        shooter = new Shooter();
        intake = new Intake();
        arm = new Arm();
        configureButtonBindings();
        initializeAutonCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton", autoChooser);
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
        new Trigger(primaryController::getSquareButton).whileTrue(new ExtendClimbCommand(climber));
        new Trigger(primaryController::getTriangleButton).whileTrue(new ClimbCommand(climber, drive.getGyro()));
        new Trigger(primaryController::getOptionsButton).onTrue(new ResetGyroCommand(drive));
        new Trigger(primaryController::getR1Button).whileTrue(new RetractClimbCommand(climber));

        //primaryController.getLeftBumper().whileTrue(new TurtleCommand(driveCommand));
        secondaryController.getAButton().onTrue(new IntakeCommand(intake));
        secondaryController.getAButton().onFalse(new StopIntakeCommand(intake));

        secondaryController.getBButton().onTrue(new OuttakeCommand(intake));
        secondaryController.getBButton().onFalse(new StopIntakeCommand(intake));

        ArmCommand armCommand = new ArmCommand(arm, secondaryController::getYAxis);
        arm.setDefaultCommand(armCommand);
        /* secondaryController.getRightBumper().onTrue(
            (new ShootCommand(shooter, 3750))
                .alongWith(new ArmToPositionCommand(arm, 0.08261))
                .until(() -> shooter.isAtSpeed() && arm.isAtDesiredState())
                .andThen(new FeedCommand(intake)
                .raceWith(new WaitCommand(0.4)))
                .andThen(new StopShootCommand(shooter))); */
        secondaryController.getRightBumper().whileTrue(new ShootCommand(shooter, 3750));
        secondaryController.getRightBumper().onFalse(new StopShootCommand(shooter));
        secondaryController.getLeftBumper().whileTrue(new ShootCommand(shooter, -2000));
        secondaryController.getLeftBumper().onFalse(new StopShootCommand(shooter));

        secondaryController.getRightTrigger().whileTrue(new FeedCommand(intake));
        secondaryController.getYButton().whileTrue(new SourcePickupCommand(shooter));
        secondaryController.getDpadLeft().onTrue(new ArmToPositionCommand(arm, 0.09476)); //center note position: 0.11285, 

        secondaryController.getDpadUp().onTrue(new ArmToPositionCommand(arm, ArmConstants.ARM_SOURCE_PICKUP_POSITION));
        secondaryController.getDpadDown().whileTrue(new SourceIntakeCommand(shooter));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        drive.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected().getName()));
        return autoChooser.getSelected();        
    }

    public void teleopInit() {
        arm.setReference(new State(arm.getEncoderPosition(), 0));
        arm.setArmStartState();
    }
    public void autonInit() {
        arm.autonomousInit();
    }

    public void initializeAutonCommands() {
        NamedCommands.registerCommand("Shoot-Subwoofer", new AutoShootCommand(intake, arm, shooter, 0.09476, 3750));
        NamedCommands.registerCommand("Shoot-MiddleNote", new AutoShootCommand(intake, arm, shooter, 0.1216, 4750));
        NamedCommands.registerCommand("DropArm", new ArmToPositionCommand(arm, 0));
        NamedCommands.registerCommand("Stop", new StopDriveCommand(drive));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intake));
        NamedCommands.registerCommand("StopIntake", new StopIntakeCommand(intake));


    }
}
