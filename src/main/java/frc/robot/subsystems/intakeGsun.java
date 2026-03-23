package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;




public class intakeGsun extends SubsystemBase {

    // ─── POSITIONS ────────────────────────────────────────────────────────────
    // Position 1: confirmed correct angle from Phoenix Tuner X self-test
    private static final double HOLD_POSITION_ROTATIONS1 = -0.413086;

    // Position 2: tune this — move to desired angle, read "intakeShaft/Position"
    // from Shuffleboard, paste here
    private static final double HOLD_POSITION_ROTATIONS2 = 5.0; // TUNE THIS

    // 120 degrees in rotor rotations (120 / 360 = 0.333 rotations).
    // If your mechanism has a gear ratio, multiply: e.g. 0.333 * gearRatio
    private static final double NUDGE_ROTATIONS = 0.333;

    // How fast the nudge spins (duty cycle, 0.0 to 1.0). Keep small.
    private static final double NUDGE_POWER = 0.2;

    // How long (seconds) the nudge runs before stopping automatically.
    private static final double NUDGE_TIMEOUT_SECONDS = 0.5; // TUNE THIS

    // ─── PID + MOTION MAGIC ───────────────────────────────────────────────────
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

    private TalonFX intakeBall;
    private TalonFX intakeShaft;

    final DutyCycleOut intakePower   = new DutyCycleOut(0.0);
    final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

    public intakeGsun() {
        this.intakeBall  = new TalonFX(19);
        this.intakeShaft = new TalonFX(23);
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

        intakeShaft.getConfigurator().apply(cfg);
    }

    // ─── SHUFFLEBOARD TUNING ─────────────────────────────────────────────────
    // Call this from periodic() to stream live motor data to Shuffleboard.
    // Open Shuffleboard, look for the "intakeShaft" tab.
    // Move the mechanism to any position and read "intakeShaft/Position (rot)"
    // to find the right value for HOLD_POSITION_ROTATIONS1/2.
    private void publishTuningData() {
        double pos = intakeShaft.getPosition().getValueAsDouble();
        double vel = intakeShaft.getVelocity().getValueAsDouble();
        double volts = intakeShaft.getMotorVoltage().getValueAsDouble();

        SmartDashboard.putNumber("intakeShaft/Position (rot)", pos);
        SmartDashboard.putNumber("intakeShaft/Position (deg)", pos * 360.0);
        SmartDashboard.putNumber("intakeShaft/Velocity (rot-s)", vel);
        SmartDashboard.putNumber("intakeShaft/Voltage (V)", volts);

        // Shows how far off the shaft is from each target position
        SmartDashboard.putNumber("intakeShaft/Error to Pos1 (rot)", pos - HOLD_POSITION_ROTATIONS1);
        SmartDashboard.putNumber("intakeShaft/Error to Pos2 (rot)", pos - HOLD_POSITION_ROTATIONS2);
    }

    @Override
    public void periodic() {
        publishTuningData();
    }

    // ─── INTAKE ROLLER ────────────────────────────────────────────────────────

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

    // ─── NUDGE: spins ~120 degrees then stops automatically ──────────────────
    // Use this as a backup if Motion Magic isn't working or you need a quick jog.
    // NUDGE_TIMEOUT_SECONDS controls how long it runs — tune it so it stops
    // roughly at 120 degrees. Watch "intakeShaft/Position (deg)" on Shuffleboard.
    public Command nudgeIntake() {
        return Commands.startEnd(
            () -> intakeShaft.setControl(intakePower.withOutput(NUDGE_POWER)),
            () -> intakeShaft.setControl(intakePower.withOutput(0.0)),
            this
        ).withTimeout(NUDGE_TIMEOUT_SECONDS);
    }

    public Command runTempKrak(){ {
        return Commands.startEnd(
            () -> intakeShaft.setControl(intakePower.withOutput(-0.5)), 
            () -> intakeShaft.setControl(intakePower.withOutput(0)),
            this
        );
    }}

    // ─── HOLD POSITIONS ───────────────────────────────────────────────────────

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