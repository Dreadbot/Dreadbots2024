// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commmands.driveCommands.DriveCommand;
import frc.robot.commmands.driveCommands.TurtleCommand;
import frc.robot.commmands.intakeCommands.IntakeCommand;
import frc.robot.commmands.intakeCommands.OuttakeCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.commmands.shooterCommands.ShootCommand;
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
    Drive drive = new Drive(); 
    private final Shooter shooter;
    private final Intake intake; 

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Chooser", autoChooser);
        shooter = new Shooter();
        intake = new Intake();
        configureButtonBindings();
        
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
        primaryController.getLeftBumper().whileTrue(new TurtleCommand(driveCommand));
        primaryController.getAButton().whileTrue(new IntakeCommand(intake));
        primaryController.getBButton().whileTrue(new OuttakeCommand(intake));
        primaryController.getXButton().whileTrue(new ShootCommand(shooter));

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