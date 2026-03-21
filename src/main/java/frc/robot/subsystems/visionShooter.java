package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.lang.Math;




public class visionShooter extends SubsystemBase {
    private TalonFX shooter1;
    private TalonFX shooter2;
    private TalonFX inmotor;

    private final String limelightName = "limelight";
    private final PIDController powerPID = new PIDController(1.0, 0.0, 0.0);

    public visionShooter() {
        this.shooter1 = new TalonFX(0);
        this.shooter2 = new TalonFX(0);
        this.inmotor = new TalonFX(0);
        powerPID.setTolerance(0.1);
    }

    final DutyCycleOut shooterPower = new DutyCycleOut(0.0);
    final DutyCycleOut intakePower = new DutyCycleOut(0.0);

    private boolean lastHadTarget = false;

    public void periodic() {}

    public double[] getTargetDistance() {
        double[] robotPoseTargetSpace = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        double x = robotPoseTargetSpace[0];
        double y = robotPoseTargetSpace[1];
        double z = robotPoseTargetSpace[2];
        double horizontalDistance = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
        double verticalDistance = y;
        double[] distances = {horizontalDistance, verticalDistance};
        return distances;
    }

    public double calculateShooterPower(double[] distances, double angle) {
        double targetVelocity = Math.sqrt((9.8 * Math.pow(distances[0], 2)) / ((2 * Math.pow(Math.cos(angle), 2)) * (distances[0] * Math.tan(angle) - distances[1])));
        double wheelRadius = 0.1524;
        double gearRatio = 1;
        double tune = 1.0;
        double targetWheelVelocity = (((targetVelocity / wheelRadius) * tune) / (2 * Math.PI)) * gearRatio;
        double currentVelocity = shooter1.getVelocity().getValueAsDouble();
        double power = powerPID.calculate(currentVelocity, targetWheelVelocity);
        power = MathUtil.clamp(power, -1.0, 1.0);
        return power;
    }

    public double calculateShooterPowerTrialandError(double[] distances) {
        double power = 1;
        if (distances[0] < 0.5) {
            power = 0.3;
        } else if (distances[0] < 1) {
            power = 0.4;
        } else if (distances[0] < 1.5) {
            power = 0.5;
        } else if (distances[0] < 2) {
            power = 0.6;
        } else if (distances[0] < 2.5) {
            power = 0.7;
        } else if (distances[0] < 3) {
            power = 0.8;
        }
        return power;
    }

    public void shooterStart(double power) {
        shooter1.setControl(shooterPower.withOutput(power));
        shooter2.setControl(shooterPower.withOutput(power));
        inmotor.setControl(intakePower.withOutput(-0.8));
    }

    public void shooterStop() {
        shooter1.setControl(shooterPower.withOutput(0.0));
        shooter2.setControl(shooterPower.withOutput(0.0));
        inmotor.setControl(intakePower.withOutput(0.0));
    }

    public Command runShooter() {
        return Commands.run(
            () -> {
                boolean tv = LimelightHelpers.getTV(limelightName);
                if (!tv) {
                    shooterStop();

                    if (lastHadTarget) {
                        SmartDashboard.putString("visionShooter/Status", "No tag found");
                    }
                    lastHadTarget = false;
                    return;
                }

                lastHadTarget = true;

                double[] distances = getTargetDistance();
                double angle = Math.toRadians(60);
                double power = calculateShooterPower(distances, angle);
                // double power = calculateShooterPowerTrialandError(distances);
                shooterStart(power);
            },
            this
        );
    }
}