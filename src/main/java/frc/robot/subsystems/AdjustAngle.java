package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdjustAngle extends SubsystemBase {

    private final TalonFX motor = new TalonFX(27); // CHANGE CAN ID

    private final DutyCycleOut power = new DutyCycleOut(0.0);

    private static final double ADJUST_SPEED = 0.1;

    public Command spinForward() {
        return Commands.startEnd(
            () -> motor.setControl(power.withOutput(ADJUST_SPEED)),
            () -> motor.setControl(power.withOutput(0.0)),
            this
        );
    }

    public Command spinBackward() {
        return Commands.startEnd(
            () -> motor.setControl(power.withOutput(-ADJUST_SPEED)),
            () -> motor.setControl(power.withOutput(0.0)),
            this
        );
    }
}