package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class intake extends SubsystemBase {
    private TalonFX intake;

    public intake() {
        this.intake = new TalonFX(0);
    }
    
    final DutyCycleOut intakePower = new DutyCycleOut(0.0);
    public void periodic() {}

    public void intakeStart() {
        intake.setControl(intakePower.withOutput(1.0));
    }

     public void intakeStop() {
        intake.setControl(intakePower.withOutput(0.0));
    }

    public Command runIntake() {
    return Commands.startEnd(
        () -> intakeStart(),
        () -> intakeStop(),
        this
    );
}
}
