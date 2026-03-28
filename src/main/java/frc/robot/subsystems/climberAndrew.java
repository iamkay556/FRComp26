package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climberAndrew extends SubsystemBase {

    // ─── HOW TO FIND POSITIONS USING PHOENIX TUNER X ─────────────────────────
    //
    // 1. Open Phoenix Tuner X and connect to the robot (USB or WiFi)
    // 2. Click on your climber motor in the device list
    // 3. Go to the "Self-Test" tab and click "Self-Test Snapshot"
    //    OR go to the "Plot" tab and enable "Position"
    // 4. Manually move the climber arm to the LOWERED position (fully extended)
    //    Read the "Position" value shown — paste it into LOWER_POSITION_ROTATIONS
    // 5. Manually move the climber arm to the CLIMBED position (fully retracted)
    //    Read the "Position" value shown — paste it into CLIMB_POSITION_ROTATIONS
    //
    // TIP: You can also zero the encoder first in Tuner X ("Zero" button on the
    // device page) with the arm at rest so your numbers start from 0.
    // ─────────────────────────────────────────────────────────────────────────

    // Climber arm fully lowered/extended — ready to hook the cage
    private static final double LOWER_POSITION_ROTATIONS = 10.0; // TUNE WITH TUNER X

    // Climber arm fully retracted — robot pulled up
    private static final double CLIMB_POSITION_ROTATIONS = -10.0; // TUNE WITH TUNER X

    // How close to target (rotations) counts as "arrived" in the auto sequence
    private static final double POSITION_TOLERANCE = 0.5;

    // How fast robot drives backward to seat against cage (robot-relative m/s)
    private static final double DRIVE_BACK_SPEED_MPS = 0.5; // TUNE THIS

    // How long (seconds) to drive backward
    private static final double DRIVE_BACK_SECONDS = 0.75; // TUNE THIS

    // ─── PID + MOTION MAGIC GAINS ─────────────────────────────────────────────
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.12;
    private static final double kA = 0.0;
    private static final double kS = 0.1;

    private static final double MM_CRUISE_VELOCITY = 5.0;
    private static final double MM_ACCELERATION    = 10.0;
    private static final double MM_JERK            = 0.0;
    // ─────────────────────────────────────────────────────────────────────────

    private final TalonFX climber = new TalonFX(26); // CHANGE CAN ID
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
    final DutyCycleOut climberPow= new DutyCycleOut(0.0);


    public climberAndrew() {
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kV = kV;
        slot0.kA = kA;
        slot0.kS = kS;

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
        mm.MotionMagicAcceleration   = MM_ACCELERATION;
        mm.MotionMagicJerk           = MM_JERK;

        climber.getConfigurator().apply(cfg);
    }

    public void periodic() {}

     public Command runDown() {
        return Commands.startEnd(
            () -> climber.setControl(climberPow.withOutput(-0.1)),
            () -> climber.setControl(climberPow.withOutput(0)),
            this
        );
    }

    public Command runUp() {
        return Commands.startEnd(
            () -> climber.setControl(climberPow.withOutput(0.1)),
            () -> climber.setControl(climberPow.withOutput(0)),
            this
        );
    }

    private Command lowerClimber() {
        return Commands.run(
            () -> climber.setControl(positionRequest.withPosition(LOWER_POSITION_ROTATIONS)),
            this
        );
    }

    private Command driveBack(driveKay drive) {
        return Commands.run(
            () -> drive.driveRobotRelative(new ChassisSpeeds(-DRIVE_BACK_SPEED_MPS, 0, 0)),
            drive
        )
        .withTimeout(DRIVE_BACK_SECONDS)
        .andThen(Commands.runOnce(
            () -> drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0)),
            drive
        ));
    }

    private Command pullUp() {
        return Commands.run(
            () -> climber.setControl(positionRequest.withPosition(CLIMB_POSITION_ROTATIONS)),
            this
        );
    }

    public Command cmdLowerClimber() {
        return lowerClimber();
    }

    public Command cmdPullUp() {
        return pullUp();
    }

    public Command climbFor() {
        return Commands.startEnd(
            () -> climber.setControl(climberPow.withOutput(-0.7)),
            () -> climber.setControl(climberPow.withOutput(0)),
            this
        );
    }
    public Command climbBack() {
        return Commands.startEnd(
            () -> climber.setControl(climberPow.withOutput(0.7)),
            () -> climber.setControl(climberPow.withOutput(0)),
            this
        );
    }


    public Command fullClimb(driveKay drive) {
        return lowerClimber()
            .until(() -> Math.abs(
                climber.getPosition().getValueAsDouble() - LOWER_POSITION_ROTATIONS
            ) < POSITION_TOLERANCE)
            .andThen(driveBack(drive))
            .andThen(pullUp());
    }
}