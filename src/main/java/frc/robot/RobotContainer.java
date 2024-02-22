// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commmands.armCommands.ArmCommand;
import frc.robot.commmands.climberCommands.ExtendClimbCommand;
import frc.robot.commmands.climberCommands.RetractClimbCommand;
import frc.robot.commmands.driveCommands.DriveCommand;
import frc.robot.commmands.driveCommands.LockonCommand;
import frc.robot.commmands.driveCommands.TurtleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.commmands.intakeCommands.IntakeCommand;
import frc.robot.commmands.intakeCommands.OuttakeCommand;
import frc.robot.commmands.shooterCommands.ShootCommand;
import frc.robot.commmands.shooterCommands.SourcePickupCommand;
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

    
    private final DreadbotController primaryController = new DreadbotController(OperatorConstants.PRIMARY_JOYSTICK_PORT);
    private final DreadbotController secondaryController = new DreadbotController(OperatorConstants.SECONDARY_JOYSTICK_PORT);
   // public final SendableChooser<Command> autoChooser;
    private final Drive drive = new Drive();
    private final Climber climber;

   // public final SendableChooser<Command> autoChooser; 
    private final Shooter shooter;
    private final Intake intake; 
    private final Arm arm;
    private final PneumaticHub pneumaticHub;
    private final NetworkTable table;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        pneumaticHub = new PneumaticHub(21);
        pneumaticHub.enableCompressorDigital();
       // autoChooser = AutoBuilder.buildAutoChooser();
       // SmartDashboard.putData("Auto Chooser", autoChooser)       
        climber = new Climber(drive.getGyro());
        shooter = new Shooter();
        intake = new Intake();
        arm = new Arm();
        configureButtonBindings();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("SmartDashboard");
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
       DriveCommand driveCommand = new DriveCommand(drive, primaryController::getXAxis, primaryController::getYAxis, primaryController::getZAxis);
       drive.setDefaultCommand(driveCommand);
    //    primaryController.getXButton().whileTrue(new ExtendClimbCommand(climber));
    //    primaryController.getYButton().whileTrue(new RetractClimbCommand(climber, drive.getGyro()));

        //primaryController.getLeftBumper().whileTrue(new TurtleCommand(driveCommand));
        secondaryController.getAButton().whileTrue(new IntakeCommand(intake));
        secondaryController.getBButton().whileTrue(new OuttakeCommand(intake));
        ArmCommand armCommand = new ArmCommand(arm, secondaryController::getYAxis);
        arm.setDefaultCommand(armCommand);  
        secondaryController.getXButton().whileTrue(new ShootCommand(shooter));
        secondaryController.getYButton().whileTrue(new SourcePickupCommand(shooter));

        DoubleTopic dblTopic = table.getDoubleTopic("thetaTagPub");
        primaryController.getAButton().whileTrue(new LockonCommand(drive, dblTopic.subscribe(0, null)));
    }

    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      //  drive.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected().getName()));
        //return autoChooser.getSelected();
        return new Command() {
            ;
        };
    }
}
