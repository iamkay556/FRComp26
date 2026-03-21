// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.visionAlignment;
import frc.robot.commands.appTag;

import java.util.Set;

import frc.robot.subsystems.driveKay;
import frc.robot.subsystems.VisionPID;
import frc.robot.subsystems.intakeGsun;
import frc.robot.subsystems.shooterRichard;
import frc.robot.subsystems.visionShooter;
import frc.robot.subsystems.climberAndrew;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj2.command.*;

public class RobotContainer {

  // ─── SUBSYSTEMS ───────────────────────────────────────────────────────────
  private final CommandJoystick m_driverController = new CommandJoystick(0);
  private final CommandJoystick m_aimJoystick      = new CommandJoystick(1);

  private final driveKay        m_swerve           = new driveKay();
  private final VisionPID       m_vision           = new VisionPID(m_swerve, "limelight");
  private final appTag          m_approachReef     = new appTag(m_swerve, "limelight", 0.5);
  private final appTag          m_approachClimber  = new appTag(m_swerve, "limelight", 0.3);
  private final intakeGsun      m_intake           = new intakeGsun();
  private final shooterRichard  m_shooter          = new shooterRichard();
  private final visionShooter   m_visionShooter    = new visionShooter();
  private final visionAlignment m_VisionAlignment  = new visionAlignment(m_swerve);
  private final climberAndrew   m_climber          = new climberAndrew();

  public RobotContainer() {
    m_swerve.configureAutoBuilder();
    configureBindings();
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureBindings() {

    // ─── DRIVE CONTROLLER ─────────────────────────────────────────────────
    m_swerve.setDefaultCommand(m_swerve.driveCommand(
        () -> m_driverController.getRawAxis(1),
        () -> m_driverController.getRawAxis(0),
        () -> -m_driverController.getRawAxis(2)
    ));

    m_driverController.button(3).onTrue(m_swerve.zeroGyroCommand());

    // ─── AIM JOYSTICK ─────────────────────────────────────────────────────
    m_aimJoystick.button(1).whileTrue(m_VisionAlignment);           // Vision align
    m_aimJoystick.button(2).onTrue(m_approachReef);                 // Approach reef tag
    m_aimJoystick.button(3).onTrue(m_approachClimber);              // Approach climber tag

    m_aimJoystick.button(4).toggleOnTrue(m_intake.runIntake());     // Run intake rollers
    m_aimJoystick.button(5).toggleOnTrue(m_shooter.runShooter());   // Run shooter
    m_aimJoystick.button(6).toggleOnTrue(m_visionShooter.runShooter()); // Vision shooter

    m_aimJoystick.button(7).whileTrue(m_intake.holdPosition1());    // Intake position 1
    m_aimJoystick.button(8).whileTrue(m_intake.holdPosition2());    // Intake position 2

    // ─── CLIMBER ──────────────────────────────────────────────────────────
    m_aimJoystick.button(9).whileTrue(m_climber.cmdLowerClimber()); // Lower arm manually
    m_aimJoystick.button(10).whileTrue(m_climber.cmdPullUp());      // Pull up manually
    m_aimJoystick.button(11).onTrue(m_climber.fullClimb(m_swerve)); // Full auto climb sequence
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("ballin");
  }
}