package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.LimelightHelpers;

public class visionShooterNew extends SubsystemBase {


    private static final double[][] POWER_TABLE = {
        {  50.0, 0.525  },
        {  63.0, 0.56  },
        {  75.0, 0.58 },
        { 120.0, 0.67  },
    };

    private static final double MIN_POWER = 0.50;
    private static final double MAX_POWER = 0.75;

    private static final double FEED_POWER = 0.6;

    private static final String LIMELIGHT = "limelight";


    private final TalonFX shooter1 = new TalonFX(22);
    private final TalonFX shooter2 = new TalonFX(25);
    private final TalonFX inmotor  = new TalonFX(24);

    final DutyCycleOut shooterPower = new DutyCycleOut(0.0);
    final DutyCycleOut intakePower  = new DutyCycleOut(0.0);

    public visionShooterNew() {}

    @Override
    public void periodic() {
        double distIn = getDistance();
        SmartDashboard.putNumber("visionShooter/Distance (in)", distIn);
        SmartDashboard.putNumber("visionShooter/Calculated Power", interpolatePower(distIn));
        SmartDashboard.putBoolean("visionShooter/Has Target", LimelightHelpers.getTV(LIMELIGHT));
    }

    // DISTANCE 
    // pose[2] (Z = forward distance)
    private double getDistance() {
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT);
        if (pose == null || pose.length < 6) return 0.0;
        return pose[2] * 39.3702 ;
    }

    // private double getDistanceInches() {
    //     return getDistanceMeters() * 39.3701;
    // }

    private double interpolatePower(double distInches) {
        if (distInches <= POWER_TABLE[0][0]) return POWER_TABLE[0][1];
        if (distInches >= POWER_TABLE[POWER_TABLE.length - 1][0])
            return POWER_TABLE[POWER_TABLE.length - 1][1];

        for (int i = 0; i < POWER_TABLE.length - 1; i++) {
            double d0 = POWER_TABLE[i][0],   p0 = POWER_TABLE[i][1];
            double d1 = POWER_TABLE[i+1][0], p1 = POWER_TABLE[i+1][1];
            if (distInches >= d0 && distInches <= d1) {
                double t = (distInches - d0) / (d1 - d0);
                return MathUtil.clamp(p0 + t * (p1 - p0), MIN_POWER, MAX_POWER);
            }
        }
        return MIN_POWER;
    }

    public void shooterStart(double power) {
        shooter1.setControl(shooterPower.withOutput(power));
        shooter2.setControl(shooterPower.withOutput(-power));
        inmotor.setControl(intakePower.withOutput(FEED_POWER));
    }

    public void shooterStop() {
        shooter1.setControl(shooterPower.withOutput(0.0));
        shooter2.setControl(shooterPower.withOutput(0.0));
        inmotor.setControl(intakePower.withOutput(0.0));
    }

    public Command runShooter() {
        return Commands.run(() -> {
            if (!LimelightHelpers.getTV(LIMELIGHT)) {
                shooterStop();
                SmartDashboard.putString("visionShooter/Status", "No target");
                return;
            }
            double distIn = getDistance();
            double power  = interpolatePower(distIn);
            shooterStart(power);
            SmartDashboard.putString("visionShooter/Status", "Shooting");
            SmartDashboard.putNumber("visionShooter/Applied Power", power);
        }, this)
        .finallyDo((interrupted) -> shooterStop());
    }
}