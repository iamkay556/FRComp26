package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveKay;

public class appTag extends Command {

    private final driveKay drive;
    private final String limelightName;
    private final double targetDistanceMeters;

    private final ProfiledPIDController forwardPID = new ProfiledPIDController(
        0.04, 0.0, 0.002,
        new TrapezoidProfile.Constraints(
            0.6,  
            0.4 
        )
    );

    // strafe: centers robot on tag using TX angle
    private final ProfiledPIDController strafePID = new ProfiledPIDController(
        0.03, 0.0, 0.001,
        new TrapezoidProfile.Constraints(0.5, 0.3)
    );

    // turn: squares robot to tag using TX angle
    private final ProfiledPIDController turnPID = new ProfiledPIDController(
        0.04, 0.0, 0.001,
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(90),
            Units.degreesToRadians(120)
        )
    );

    private int stableCount = 0;
    private static final int STABLE_LOOPS = 10; // 10 * 20ms = 200ms hold

    public appTag(driveKay drive, String limelightName, double targetDistanceMeters) {
        this.drive = drive;
        this.limelightName = limelightName;
        this.targetDistanceMeters = targetDistanceMeters;
        addRequirements(drive);

        forwardPID.setTolerance(0.05); // 5cm
        strafePID.setTolerance(0.8);   // degrees
        turnPID.setTolerance(Units.degreesToRadians(1.5));
    }

    private double getDistance() {
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        if (pose == null || pose.length < 6) return 0.0;
        return pose[2];
    }

    @Override
    public void initialize() {
        forwardPID.reset(getDistance());
        strafePID.reset(LimelightHelpers.getTX(limelightName));
        turnPID.reset(drive.getPose().getRotation().getRadians());
        stableCount = 0;
        SmartDashboard.putString("ApproachTag/Status", "Running");
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(limelightName)) {
            drive.driveRobotRelative(new ChassisSpeeds());
            SmartDashboard.putString("ApproachTag/Status", "No Target");
            stableCount = 0;
            return;
        }

        double currentDist = getDistance();
        double tx = LimelightHelpers.getTX(limelightName);

        double vx    = MathUtil.clamp(forwardPID.calculate(currentDist, targetDistanceMeters), -1, 1);
        double vy    = MathUtil.clamp(strafePID.calculate(tx, 0.0), -1, 1);
        double omega = MathUtil.clamp(turnPID.calculate(tx, 0.0), -0.5, 0.5);

        drive.driveRobotRelative(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public boolean isFinished() {
        if (!LimelightHelpers.getTV(limelightName)) return false;

        double distErr = Math.abs(getDistance() - targetDistanceMeters);
        double txErr   = Math.abs(LimelightHelpers.getTX(limelightName));

        if (distErr < 0.1 && txErr < 0.8) stableCount++;
        else stableCount = 0;

        return stableCount >= STABLE_LOOPS;
    }
}