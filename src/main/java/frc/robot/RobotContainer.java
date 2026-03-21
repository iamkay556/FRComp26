// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.visionAlignment;
import frc.robot.commands.appTag;

import java.util.Set;


import frc.robot.subsystems.driveKay;
import frc.robot.subsystems.VisionPID;
import frc.robot.subsystems.intakeGsun;
import frc.robot.subsystems.shooterRichard;
import frc.robot.subsystems.visionShooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import java.io.File;
import java.rmi.dgc.VMID;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.TempVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final CommandJoystick m_driverController = new CommandJoystick(0);
  private final CommandJoystick m_aimJoystick = new CommandJoystick(1);
  private final driveKay m_swerve = new driveKay();
  private final VisionPID m_vision = new VisionPID(m_swerve, "limelight");
  private final appTag m_approachReef = new appTag(m_swerve, "limelight", 0.5);
  private final appTag m_approachClimber = new appTag(m_swerve, "limelight", 0.3);
  private final intakeGsun m_intake = new intakeGsun();
  private final shooterRichard m_shooter = new shooterRichard();
  private final visionShooter m_visionShooter = new visionShooter();
  private final visionAlignment m_VisionAlignment = new visionAlignment(m_swerve);

  
  public RobotContainer() {
    
    // Configure the trigger bindings
    m_swerve.configureAutoBuilder();
    configureBindings();

 
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
     m_swerve.setDefaultCommand(m_swerve.driveCommand(
       () -> -m_driverController.getRawAxis(1), 
       ()-> -m_driverController.getRawAxis(0), 
       ()->-m_driverController.getRawAxis(2)
     ));

    //  m_swerve.setDefaultCommand(m_swerve.driveCommand(
    //    () -> -m_driverController.getRawAxis(0), 
    //    ()-> -m_driverController.getRawAxis(1), 
    //    ()->-m_driverController.getRawAxis(4)
    //  ));


     //  m_driverController.button(8).whileTrue(m_swerve.driveToPose(m_vision.findLeftBranch()));
     m_driverController.button(3).onTrue(m_swerve.zeroGyroCommand());
    //  m_driverController.button(4).onTrue(Commands.runOnce(() -> m_vision.lockIn()));
    //  m_driverController.button(5).whileTrue(m_swerve.driveToPose(m_vision.getTargetPose()));
    // m_driverController.button(5).whileTrue(Commands.defer(() -> m_swerve.driveToPose(m_vision.getTargetPose()), Set.of(m_swerve)));
    //  m_driverController.button(5).whileTrue(m_vision.alignFromVisibleTag()); 
    m_aimJoystick.button(1).whileTrue(m_VisionAlignment); 
    m_aimJoystick.button(2).onTrue(m_approachReef);
    m_aimJoystick.button(3).onTrue(m_approachClimber);
    m_aimJoystick.button(4).toggleOnTrue(m_intake.runIntake());
    m_aimJoystick.button(5).toggleOnTrue(m_shooter.runShooter());
    m_aimJoystick.button(6).toggleOnTrue(m_visionShooter.runShooter());
    
    m_aimJoystick.button(7).whileTrue(m_intake.holdPosition1());
    m_aimJoystick.button(8).whileTrue(m_intake.holdPosition2());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("ballin");
  }
}
