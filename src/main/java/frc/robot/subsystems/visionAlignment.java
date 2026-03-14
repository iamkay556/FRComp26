package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class visionAlignment extends Command {
    private final driveKay drive;
    private final String limelightName = "limelight";
    private static final double targetDistance = Units.feetToMeters(4.0);

    private final PIDController forwardPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController strafePID  = new PIDController(1.8, 0.0, 0.0);
    private final PIDController turnPID    = new PIDController(0.03, 0.0, 0.001);

    public visionAlignment(driveKay drive) {
        this.drive = drive;
        addRequirements(drive);

        forwardPID.setTolerance(0.05);
        strafePID.setTolerance(0.05);
        turnPID.setTolerance(1.0);
    }
private int telemetryCounter = 0;
private boolean lastHadTarget = false;

@Override
public void initialize() {
    forwardPID.reset();
    strafePID.reset();
    turnPID.reset();
    telemetryCounter = 0;
    SmartDashboard.putString("VisionAlign/Status", "Initialized");
}

@Override
public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(limelightName);

    boolean tv = LimelightHelpers.getTV("limelight");
    double[] tprs = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    double[] bpts = LimelightHelpers.getBotPose_TargetSpace("limelight");

    SmartDashboard.putBoolean("LL/TV", tv);
    SmartDashboard.putString("LL/PipelineType", LimelightHelpers.getCurrentPipelineType("limelight"));
    SmartDashboard.putNumber("LL/PipelineIndex", LimelightHelpers.getCurrentPipelineIndex("limelight"));

    SmartDashboard.putNumber("LL/TPRS_Len", tprs == null ? -1 : tprs.length);
        SmartDashboard.putNumber("LL/BPTS_Len", bpts == null ? -1 : bpts.length);

    if (tprs != null && tprs.length >= 6) {
        SmartDashboard.putNumber("LL/TPRS_X", tprs[0]);
        SmartDashboard.putNumber("LL/TPRS_Y", tprs[1]);
        SmartDashboard.putNumber("LL/TPRS_Z", tprs[2]);
    }

    if (bpts != null && bpts.length >= 6) {
        SmartDashboard.putNumber("LL/BPTS_X", bpts[0]);
        SmartDashboard.putNumber("LL/BPTS_Y", bpts[1]);
        SmartDashboard.putNumber("LL/BPTS_Z", bpts[2]);
    }

    if (!hasTarget) {
        drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));

        if (lastHadTarget) {
            SmartDashboard.putString("VisionAlign/Status", "No tag found");
        }
        lastHadTarget = false;
        return;
    }

    lastHadTarget = true;

    double[] robotPoseTargetSpace = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
    if (robotPoseTargetSpace == null || robotPoseTargetSpace.length < 6) {
        drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putString("VisionAlign/Status", "Bad pose data");
        return;
    }

    double tx = LimelightHelpers.getTX(limelightName);
    double lateralErrorMeters = -robotPoseTargetSpace[0];
    double distanceMeters = robotPoseTargetSpace[2];

    double turnErrorDeg = tx;

    double vx = forwardPID.calculate(distanceMeters, targetDistance);
    double vy = strafePID.calculate(lateralErrorMeters, 0.0);
    double omega = turnPID.calculate(tx, 0.0);

    vx = MathUtil.clamp(vx, -0.8, 0.8);
    vy = MathUtil.clamp(vy, -0.8, 0.8);
    omega = MathUtil.clamp(omega, -0.5, 0.5);

    if (forwardPID.atSetpoint()) vx = 0.0;
    if (strafePID.atSetpoint()) vy = 0.0;
    if (turnPID.atSetpoint()) omega = 0.0;

    drive.driveRobotRelative(new ChassisSpeeds(vx, vy, omega));

    telemetryCounter++;
    if (telemetryCounter >= 5) {
        telemetryCounter = 0;

        SmartDashboard.putNumber("VisionAlign/DistanceMeters", distanceMeters);
        SmartDashboard.putNumber("VisionAlign/LateralErrorMeters", lateralErrorMeters);
        SmartDashboard.putNumber("VisionAlign/TurnErrorDeg", turnErrorDeg);
        SmartDashboard.putNumber("VisionAlign/CmdVx", vx);
        SmartDashboard.putNumber("VisionAlign/CmdVy", vy);
        SmartDashboard.putNumber("VisionAlign/CmdOmega", omega);
        
    }
}

@Override
public boolean isFinished() {
    return forwardPID.atSetpoint()
    && strafePID.atSetpoint()
    && turnPID.atSetpoint();
}
}