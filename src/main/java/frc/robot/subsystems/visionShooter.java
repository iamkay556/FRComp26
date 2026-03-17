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
    private TalonFX shooter;
    private final String limelightName = "limelight";
    private final PIDController powerPID = new PIDController(1.0, 0.0, 0.0);

    public visionShooter() {
        this.shooter = new TalonFX(0);
        powerPID.setTolerance(0.1);
    }
    
    final DutyCycleOut shooterPower = new DutyCycleOut(0.0);

    private boolean lastHadTarget = false;

    public double[] getTargetDistance() {
        double[] robotPoseTargetSpace = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        double x = robotPoseTargetSpace[0];
        double y = robotPoseTargetSpace[1];
        double z = robotPoseTargetSpace[2];
        double horizontalDistance = Math.sqrt(Math.pow(x,2) + Math.pow(z, 2));
        double verticalDistance = y;
        double[] distances = {horizontalDistance, verticalDistance};
        return distances;
    }

    public double calculateShooterPower(double[] distances, double angle) {
        double targetVelocity = Math.sqrt((9.8 * Math.pow(distances[0], 2)) / ((2 * Math.pow(Math.cos(angle), 2)) * (distances[0] * Math.tan(angle) - distances[1])));
        double wheelRadius = 0.1524; // in meters
        double gearRatio = 1;
        double tune = 1.0; //chat gpt said to add this to account for loss in energy transfer as current equation assumes no sllip between wheel and ball along with perfect compression and energy transfer
        double targetWheelVelocity = (((targetVelocity / wheelRadius) * tune) / (2 * Math.PI)) * gearRatio;
        double currentVelocity = shooter.getVelocity().getValueAsDouble();
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
        }else if (distances[0] < 2.5) {
            power = 0.7;
        }else if (distances[0] < 3) {
            power = 0.8;
        }
        
        return power;
    }

    public void shooterShoot(double power) {
        shooter.setControl(shooterPower.withOutput(power));
    }

    public Command runShooter() {
        return Commands.run(
            () -> {
                boolean tv = LimelightHelpers.getTV(limelightName);
                 if (!tv) {
                    shooterShoot(0);

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
                shooterShoot(power);

            },
            this
        );
    }
}
