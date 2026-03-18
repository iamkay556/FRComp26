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




public class shooterRichard extends SubsystemBase {
    private TalonFX shooter;

    public shooterRichard() {
        this.shooter = new TalonFX(0);
    }
    
    final DutyCycleOut shooterPower = new DutyCycleOut(0.0);
    public void periodic() {}

    public void shooterStart() {
        shooter.setControl(shooterPower.withOutput(1.0));
    }

     public void shooterStop() {
        shooter.setControl(shooterPower.withOutput(0.0));
    }

    public Command runShooter() {
    return Commands.startEnd(
        () -> shooterStart(),
        () -> shooterStop(),
        this
    );
}
}
