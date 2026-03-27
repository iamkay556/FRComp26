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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class intakeGsun extends SubsystemBase {

    private static final double HOLD_POSITION_ROTATIONS1 = 0.4512;
    private static final double HOLD_POSITION_ROTATIONS2 = -3.343;

    private static final double NUDGE_POWER           = 0.2;
    private static final double NUDGE_TIMEOUT_SECONDS = 0.5;

    private static final double SLOW_MOVE_POWER = 0.1;

    private static final double STALL_CHECK_SECONDS = 0.5;
    private static final double STALL_MIN_DELTA_ROT = 0.025;

    private static final double TIMEOUT_SECONDS = 1.5;

    private static final double kP = 1.5;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.12;
    private static final double kA = 0.0;
    private static final double kS = 0.1;

    private static final double MM_CRUISE_VELOCITY = 10.0;
    private static final double MM_ACCELERATION    = 20.0;
    private static final double MM_JERK            = 1.0;

    private TalonFX intakeBall;
    private TalonFX intakeShaft;

    final DutyCycleOut intakePower           = new DutyCycleOut(0.0);
    final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

    private final Timer stallTimer    = new Timer();
    private double stallCheckStartPos = 0.0;
    private boolean stallTimerRunning = false;

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

    private void publishTuningData() {
        double pos   = intakeShaft.getPosition().getValueAsDouble();
        double vel   = intakeShaft.getVelocity().getValueAsDouble();
        double volts = intakeShaft.getMotorVoltage().getValueAsDouble();

        SmartDashboard.putNumber("intakeShaft/Position (rot)", pos);
        SmartDashboard.putNumber("intakeShaft/Position (deg)", pos * 360.0);
        SmartDashboard.putNumber("intakeShaft/Velocity (rot-s)", vel);
        SmartDashboard.putNumber("intakeShaft/Voltage (V)", volts);
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

    public Command nudgeIntake() {
        return Commands.startEnd(
            () -> intakeShaft.setControl(intakePower.withOutput(NUDGE_POWER)),
            () -> intakeShaft.setControl(intakePower.withOutput(0.0)),
            this
        ).withTimeout(NUDGE_TIMEOUT_SECONDS);
    }

    public Command runTempKrak() {
        return Commands.startEnd(
            () -> intakeShaft.setControl(intakePower.withOutput(-0.1)),
            () -> intakeShaft.setControl(intakePower.withOutput(0)),
            this
        );
    }

    public Command runTempKrakBackwards() {
        return Commands.startEnd(
            () -> intakeShaft.setControl(intakePower.withOutput(0.1)),
            () -> intakeShaft.setControl(intakePower.withOutput(0)),
            this
        );
    }

    public Command shaftSlowSpin(boolean forward) {
        double power = forward ? SLOW_MOVE_POWER : -SLOW_MOVE_POWER;
        return Commands.startEnd(
            () -> intakeShaft.setControl(intakePower.withOutput(power)),
            () -> intakeShaft.setControl(intakePower.withOutput(0.0)),
            this
        );
    }

    public Command shaftSpinTimeout(boolean forward) {
        double power = forward ? SLOW_MOVE_POWER : -SLOW_MOVE_POWER;
        return Commands.startEnd(
            () -> intakeShaft.setControl(intakePower.withOutput(power)),
            () -> intakeShaft.setControl(intakePower.withOutput(0.0)),
            this
        ).withTimeout(TIMEOUT_SECONDS);
    }

    public Command shaftSpinUntilStall(boolean forward) {
        double power = forward ? SLOW_MOVE_POWER : -SLOW_MOVE_POWER;
        return Commands.sequence(
            Commands.runOnce(() -> {
                stallTimer.reset();
                stallTimer.start();
                stallCheckStartPos = intakeShaft.getPosition().getValueAsDouble();
                stallTimerRunning  = true;
                SmartDashboard.putBoolean("intakeShaft/Stalled", false);
            }),
            Commands.run(() -> {
                intakeShaft.setControl(intakePower.withOutput(power));

                if (stallTimerRunning && stallTimer.hasElapsed(STALL_CHECK_SECONDS)) {
                    double currentPos = intakeShaft.getPosition().getValueAsDouble();
                    double delta      = Math.abs(currentPos - stallCheckStartPos);
                    SmartDashboard.putNumber("intakeShaft/Stall Delta (rot)", delta);

                    if (delta < STALL_MIN_DELTA_ROT) {
                        SmartDashboard.putBoolean("intakeShaft/Stalled", true);
                        stallTimerRunning = false;
                    } else {
                        stallCheckStartPos = currentPos;
                        stallTimer.reset();
                    }
                }
            }, this)
            .until(() -> !stallTimerRunning)
        ).finallyDo((interrupted) -> {
            intakeShaft.setControl(intakePower.withOutput(0.0));
            stallTimer.stop();
            stallTimerRunning = false;
        });
    }


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