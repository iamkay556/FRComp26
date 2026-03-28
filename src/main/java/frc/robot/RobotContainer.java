// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.visionAlignment;
import frc.robot.commands.appTag;

import frc.robot.subsystems.driveKay;
import frc.robot.subsystems.VisionPID;
import frc.robot.subsystems.intakeGsun;
import frc.robot.subsystems.shooterRichard;
import frc.robot.subsystems.visionShooter;
import frc.robot.subsystems.visionShooterNew;
import frc.robot.subsystems.climberAndrew;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

public class RobotContainer {

  private static final double SHOOT_DURATION_SECONDS  = 6.0; 
  private static final double DRIVE_BACK_SPEED_MPS    = 1.0;
  private static final double DRIVE_BACK_DURATION_SEC = 1.5;

  private final CommandJoystick m_driverController = new CommandJoystick(0);
  private final CommandJoystick m_aimJoystick      = new CommandJoystick(1);

  // private final CommandXboxController m_xbox = new CommandXboxController(0);

  private final driveKay        m_swerve          = new driveKay();
  private final appTag          m_approachReef    = new appTag(m_swerve, "limelight", 0.5);
  private final appTag          m_approachClimber = new appTag(m_swerve, "limelight", 0.3);
  private final intakeGsun      m_intake          = new intakeGsun();
  private final shooterRichard  m_shooter         = new shooterRichard();
  private final visionShooterNew   m_visionShooter   = new visionShooterNew();
  private final visionAlignment m_VisionAlignment = new visionAlignment(m_swerve);
  private final climberAndrew   m_climber         = new climberAndrew();

  public RobotContainer() {
    m_swerve.configureAutoBuilder();
    configureBindings();
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureBindings() {

    m_swerve.setDefaultCommand(m_swerve.driveCommand(
        () -> -m_driverController.getRawAxis(1),  // forward/back
        () -> -m_driverController.getRawAxis(0),  // strafe
        () ->  m_driverController.getRawAxis(2)   // rotate
    ));

    // m_swerve.setDefaultCommand(m_swerve.driveCommand(
    //     () -> -m_xbox.getLeftY(),
    //     () -> -m_xbox.getLeftX(),
    //     () ->  m_xbox.getRightX()
    // ));

    m_driverController.button(3).onTrue(m_swerve.zeroGyroCommand());
    m_driverController.button(10).whileTrue(new visionAlignment(m_swerve));
    m_driverController.button(11).whileTrue(m_visionShooter.runShooter());
    m_driverController.button(12).whileTrue(m_intake.runIntake());

    // m_driverController.button(4).whileTrue(m_intake.runTempKrak());
    // m_driverController.button(5).whileTrue(m_intake.runTempKrakBackwards());
    m_driverController.button(7).whileTrue(m_intake.shaftSlowSpin(true));
    m_driverController.button(8).whileTrue(m_intake.shaftSlowSpin(false));

    // ─── SHOOTER (driver controller) ──────────────────────────────────────
    m_driverController.button(9).onTrue(m_shooter.runShooterTot());
    m_driverController.button(9).onFalse(m_shooter.stopRunShooterTot());

    // ─── AIM JOYSTICK (Joystick 1) ────────────────────────────────────────

    // Vision + approach
    m_aimJoystick.button(1).whileTrue(m_VisionAlignment);
    m_aimJoystick.button(2).onTrue(m_approachReef);
    m_aimJoystick.button(3).onTrue(m_approachClimber);

    // Intake
    m_aimJoystick.button(4).whileTrue(m_intake.runIntake());
    m_aimJoystick.button(5).whileTrue(m_visionShooter.runShooter());
    // m_aimJoystick.button(5).whileTrue(m_intake.runTempKrak());
    // m_aimJoystick.button(6).whileTrue(m_intake.runTempKrakBackwards());
    // m_aimJoystick.button(7).onTrue(m_intake.shaftForwardUntilStall());
    // m_aimJoystick.button(8).onTrue(m_intake.shaftBackwardUntilStall());
    // m_aimJoystick.button(9).whileTrue(m_intake.holdPosition1());
    // m_aimJoystick.button(10).whileTrue(m_intake.holdPosition2());

    m_aimJoystick.button(7).whileTrue(m_shooter.runShooter50in());
    m_aimJoystick.button(8).whileTrue(m_shooter.runShooter63in());
    m_aimJoystick.button(9).whileTrue(m_shooter.runShooter75in());
    m_aimJoystick.button(10).whileTrue(m_shooter.runShooter120in());

    m_aimJoystick.button(11).whileTrue(m_climber.cmdLowerClimber());
    m_aimJoystick.button(12).whileTrue(m_climber.cmdPullUp());
    // m_aimJoystick.button(X).onTrue(m_climber.fullClimb(m_swerve));

    // m_xbox.y().whileTrue(m_intake.runTempKrak());
    // m_xbox.a().whileTrue(m_intake.runIntake());
    // m_xbox.leftBumper().whileTrue(m_shooter.runShooterTot());
    // m_xbox.rightBumper().whileTrue(m_visionShooter.runShooter());
  }

  // public Command getAutonomousCommand() {
  //   return m_shooter.runShooterTot()
  //       .withTimeout(SHOOT_DURATION_SECONDS)
  //       .andThen(Commands.runOnce(m_shooter::shooterStopTot, m_shooter))
  //       .andThen(
  //           Commands.run(
  //               () -> m_swerve.driveRobotRelative(new ChassisSpeeds(-DRIVE_BACK_SPEED_MPS, 0, 0)),
  //               m_swerve
  //           ).withTimeout(DRIVE_BACK_DURATION_SEC)
  //       )
  //       .andThen(Commands.runOnce(
  //           () -> m_swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0)),
  //           m_swerve
  //       ));
  // }

  public Command getAutonomousCommand() {
      return Commands.run(
              () -> m_swerve.driveRobotRelative(new ChassisSpeeds(-DRIVE_BACK_SPEED_MPS, 0, 0)),
              m_swerve
          ).withTimeout(DRIVE_BACK_DURATION_SEC)
          .andThen(Commands.runOnce(
              () -> m_swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0)),
              m_swerve
          ))
          .andThen(m_visionShooter.runShooter()
              .withTimeout(SHOOT_DURATION_SECONDS)
          );
  }

  // public Command getAutonomousCommand() {
  //   return new PathPlannerAuto("ballin");
  // }
}