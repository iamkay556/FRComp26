package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;




public class intakeGsun extends SubsystemBase {


    // Target hold position in rotor rotations.
    // To find the right number: enable the robot, move intake to where you want
    // it to hold, then read intake.getPosition().getValueAsDouble() in Shuffleboard
    // or Phoenix Tuner X. Paste that number here.
    private static final double HOLD_POSITION_ROTATIONS1 = 5.0;
    private static final double HOLD_POSITION_ROTATIONS2 = 5.0;

    // kP: how hard the motor fights back when it drifts from the target.
    // Start at 1.0. If it oscillates/shakes, lower it. If it's too weak/slow, raise it.
    private static final double kP = 1.0;

    // kI: adds extra push if it's stuck slightly off target for a long time.
    // Leave at 0 to start — only add a tiny amount (e.g. 0.01) if kP alone can't
    // hold the position perfectly against gravity.
    private static final double kI = 0.0;

    // kD: dampens oscillation. If kP causes shaking, raise kD slightly (e.g. 0.1).
    private static final double kD = 0.0;

    // kV: feedforward — how much voltage per rotation/sec of velocity.
    // For a pure hold command this matters less, but 0.12 is the Kraken X60 default.
    private static final double kV = 0.12;

    // kA: acceleration feedforward. Leave at 0 unless motion is jerky.
    private static final double kA = 0.0;

    // kS: static friction compensation — minimum voltage to overcome stiction.
    // If the mechanism barely moves on low kP, raise this slightly (e.g. 0.25).
    private static final double kS = 0.1;

    // MotionMagic cruise velocity in rotations/sec.
    // How fast it moves TO the position. Lower = slower/smoother, higher = snappier.
    private static final double MM_CRUISE_VELOCITY = 5.0;

    // MotionMagic acceleration in rotations/sec^2. How quickly it ramps up to cruise.
    // Start at 2x cruise velocity (10.0). Lower if motion is jerky.
    private static final double MM_ACCELERATION = 10.0;

    // MotionMagic jerk limit in rotations/sec^3. Optional smoothing.
    // Set to 0 to disable, or e.g. 400.0 for very smooth ramping.
    private static final double MM_JERK = 0.0;

    private TalonFX intakeBall;
    private TalonFX intakeShaft;

    final DutyCycleOut intakePower = new DutyCycleOut(0.0);
    final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

    public intakeGsun() {
        this.intakeBall = new TalonFX(19);
        this.intakeShaft = new TalonFX(23); // CHANGE DEVICE ID TO WTVR WE SET
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // PID + feedforward gains on slot 0
        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kV = kV;
        slot0.kA = kA;
        slot0.kS = kS;

        // Motion Magic profile
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
        mm.MotionMagicAcceleration   = MM_ACCELERATION;
        mm.MotionMagicJerk           = MM_JERK;

        intakeShaft.getConfigurator().apply(cfg);
    }

    public void periodic() {}

    public void intakeStart() {
        intakeBall.setControl(intakePower.withOutput(1.0));
    }

    public void intakeStop() {
        intakeBall.setControl(intakePower.withOutput(0.0));
    }

    public Command runIntake() {
        return Commands.startEnd(
            () -> intakeStart(),
            () -> intakeStop(),
            this
        );
    }

    /**
     * Moves to and holds the intake at HOLD_POSITION_ROTATIONS using Motion Magic.
     * The motor will actively resist any force pushing it away from that position.
     */
    public Command holdPosition1() {
        return Commands.run(
            () -> intakeShaft.setControl(positionRequest.withPosition(HOLD_POSITION_ROTATIONS1)),
            this
        );
    }
    public Command holdPosition2() {
        return Commands.run(
            () -> intakeShaft.setControl(positionRequest.withPosition(HOLD_POSITION_ROTATIONS2)),
            this
        );
    }
}