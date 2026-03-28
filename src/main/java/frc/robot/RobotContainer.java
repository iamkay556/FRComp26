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
import frc.robot.subsystems.climberAndrew;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

public class RobotContainer {

  // ─── TUNE THIS ────────────────────────────────────────────────────────────
  // How many seconds to run the shooter before starting the climb.
  // Increase if the ball isn't fully launched. Decrease if it's wasting time.
  private static final double SHOOT_DURATION_SECONDS = 2.0;
  // ─────────────────────────────────────────────────────────────────────────

  // ─── SUBSYSTEMS ───────────────────────────────────────────────────────────
  private final CommandJoystick m_driverController = new CommandJoystick(0);
  private final CommandJoystick m_aimJoystick      = new CommandJoystick(1);
  // private final XboxController m_Controller = new XboxController(0);

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
        () -> -m_driverController.getRawAxis(1),
        () -> -m_driverController.getRawAxis(0),
        () -> m_driverController.getRawAxis(2)
    ));

    // default swerve drive using normal XboxController axes
    // m_swerve.setDefaultCommand(
    //     m_swerve.driveCommand(
    //         () -> -m_Controller.getLeftY(),   // forward/back
    //         () -> -m_Controller.getLeftX(),   // strafe
    //         () -> m_Controller.getRightX()   // rotate
    //     )
    // );

    m_driverController.button(3).onTrue(m_swerve.zeroGyroCommand());
    m_driverController.button(4).whileTrue(m_intake.runTempKrak());
    m_driverController.button(5).whileTrue(m_intake.runTempKrakBackwards());
    // m_driverController.button(5).whileTrue(m_shooter.runShooter());
    m_driverController.button(7).whileTrue(m_intake.shaftSlowSpin(true));
    m_driverController.button(8).whileTrue(m_intake.shaftSlowSpin(false));
    m_driverController.button(9).toggleOnTrue(m_shooter.runShooterTot());
    m_driverController.button(9).toggleOnFalse(m_shooter.stopRunShooterTot());
    // m_driverController.button(9).onTrue(m_shooter.runShooterOrange());
    // m_driverController.button(10).onTrue(m_shooter.runShooterGreen());
    // m_driverController.button(10).whileTrue(m_VisionAlignment);
     m_driverController.button(10).whileTrue(new visionAlignment(m_swerve)); 
    m_driverController.button(11).whileTrue(m_visionShooter.runShooter());
    m_driverController.button(12).whileTrue(m_intake.runIntake());

    


      // HOLD WHILE PRESSED
    // if (m_Controller.getYButton()) {
        // m_intake.runTempKrak().schedule();
    // }
      
    // if (m_Controller.getAButton()) {
    //     m_intake.runIntake();
    // }

    // if (m_Controller.getLeftBumper()) {
    //     m_shooter.runShooter().schedule();
    // }

    // // ─── AIM JOYSTICK ─────────────────────────────────────────────────────
    // m_aimJoystick.button(1).whileTrue(m_VisionAlignment);
    // m_aimJoystick.button(2).onTrue(m_approachReef);
    // m_aimJoystick.button(3).onTrue(m_approachClimber);

    // m_aimJoystick.button(4).toggleOnTrue(m_intake.runIntake());
    // m_aimJoystick.button(5).toggleOnTrue(m_shooter.runShooter());
    // m_aimJoystick.button(6).toggleOnTrue(m_visionShooter.runShooter());

    // // m_aimJoystick.button(7).whileTrue(m_intake.holdPosition1());
    // m_aimJoystick.button(8).whileTrue(m_intake.holdPosition2());
    // m_aimJoystick.button(7).whileTrue(m_intake.runTempKrak());

    // // ─── CLIMBER ──────────────────────────────────────────────────────────
    // m_aimJoystick.button(9).whileTrue(m_climber.cmdLowerClimber());
    // m_aimJoystick.button(10).whileTrue(m_climber.cmdPullUp());
    // m_aimJoystick.button(11).onTrue(m_climber.fullClimb(m_swerve));
  }

  // ─── AUTO: SHOOT THEN CLIMB ───────────────────────────────────────────────
  // Step 1 — Run shooter for SHOOT_DURATION_SECONDS, then stop it
  // Step 2 — Run full climb sequence (lower arm → drive back → pull up)
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(""); 
  }
  // public Command getAutonomousCommand() {
  //   return m_shooter.runShooter()
  //       .withTimeout(SHOOT_DURATION_SECONDS)
  //       .andThen(Commands.runOnce(m_shooter::shooterStop, m_shooter))
  //       .andThen(m_climber.fullClimb(m_swerve));
  // }
}