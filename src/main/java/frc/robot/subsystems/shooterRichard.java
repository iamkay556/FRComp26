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
    private TalonFX shooter1;
    private TalonFX shooter2;
    private TalonFX inmotor;

    public shooterRichard() {
        this.shooter1 = new TalonFX(22);
        this.shooter2 = new TalonFX(25);
        this.inmotor = new TalonFX(24);
    }
    
    final DutyCycleOut shooterPower = new DutyCycleOut(0.0);
    final DutyCycleOut intakePower = new DutyCycleOut(0.0);

    public void periodic() {}

    public void shooterStart() {
        shooter1.setControl(shooterPower.withOutput(.40));
        shooter2.setControl(shooterPower.withOutput(-.40));
        // inmotor.setControl(intakePower.withOutput(0.4));
    }

     public void shooterStop() {
        shooter1.setControl(shooterPower.withOutput(0.0));
        shooter2.setControl(shooterPower.withOutput(0.0));
        // inmotor.setControl(intakePower.withOutput(0.0));
    }
    public void shooterStartTot() {
        shooter1.setControl(shooterPower.withOutput(.67)); 
        shooter2.setControl(shooterPower.withOutput(-.67));
        inmotor.setControl(intakePower.withOutput(0.4));
    }

     public void shooterStopTot() {
        shooter1.setControl(shooterPower.withOutput(0.0));
        shooter2.setControl(shooterPower.withOutput(0.0));
        inmotor.setControl(intakePower.withOutput(0.0));
    }

    public Command runShooterTot() {
    return Commands.startEnd(
        () -> shooterStartTot(),
        () -> shooterStopTot(),
        this
    );
    }

    public Command stopRunShooterTot() {
        return Commands.runOnce(() -> shooterStopTot(), this);
    }


        // 50 inches
    public Command runShooter50in() {
        return Commands.startEnd(
            () -> {
                shooter1.setControl(shooterPower.withOutput(0.52));
                shooter2.setControl(shooterPower.withOutput(-0.52));
                inmotor.setControl(intakePower.withOutput(0.4));
            },
            () -> shooterStopTot(),
            this
        );
    }
    
    // 63 inches
    public Command runShooter63in() {
        return Commands.startEnd(
            () -> {
                shooter1.setControl(shooterPower.withOutput(0.55));
                shooter2.setControl(shooterPower.withOutput(-0.55));
                inmotor.setControl(intakePower.withOutput(0.4));
            },
            () -> shooterStopTot(),
            this
        );
    }
    
    // 75 inches
    public Command runShooter75in() {
        return Commands.startEnd(
            () -> {
                shooter1.setControl(shooterPower.withOutput(0.575));
                shooter2.setControl(shooterPower.withOutput(-0.575));
                inmotor.setControl(intakePower.withOutput(0.4));
            },
            () -> shooterStopTot(),
            this
        );
    }
    
    // 120 inches
    public Command runShooter120in() {
        return Commands.startEnd(
            () -> {
                shooter1.setControl(shooterPower.withOutput(0.67));
                shooter2.setControl(shooterPower.withOutput(-0.67));
                inmotor.setControl(intakePower.withOutput(0.4));
            },
            () -> shooterStopTot(),
            this
        );
    }

    public Command runShooterOrange() {
    return Commands.startEnd(
        () -> shooterStart(),
        () -> shooterStop(),
        this
    );
    }

    public Command runShooterGreen() {
    return Commands.startEnd(
        () -> inmotor.setControl(intakePower.withOutput(.4)),
        () -> inmotor.setControl(intakePower.withOutput(0.0)),
        this
    );
    }

}
