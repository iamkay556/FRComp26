package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;

public class VisionPID extends SubsystemBase {

    private final driveKay drive;
    private final String limelightName;

    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private static final double STANDOFF_DISTANCE = Units.feetToMeters(4.0);
    // reject
    private static final double MAX_SPIN_DEG_PER_SEC = 720;
    private static final double MAX_TAG_DISTANCE = 8.0;

    // alignment pid controllers
    private final PIDController xPID = new PIDController(3.0, 0, 0.2);
    private final PIDController yPID = new PIDController(3.0, 0, 0.2);
    private final ProfiledPIDController thetaPID =
            new ProfiledPIDController(
                    5.0, 0, 0.3,
                    new TrapezoidProfile.Constraints(
                            Units.degreesToRadians(540),
                            Units.degreesToRadians(720)
                    )
            );

    // tolerances
    private static final double POS_TOL = 0.05;
    private static final double ANG_TOL = Units.degreesToRadians(2);

    private double onTargetStart = -1;
        // Robot pose as seen by odometry/vision fusion
    private final StructPublisher<Pose2d> robotPosePub =
            NetworkTableInstance.getDefault()
                    .getStructTopic("AdvScope/RobotPose", Pose2d.struct).publish();

    // The target pose we're driving to
    private final StructPublisher<Pose2d> targetPosePub =
            NetworkTableInstance.getDefault()
                    .getStructTopic("AdvScope/TargetPose", Pose2d.struct).publish();

    // The raw tag pose from the field layout
    private final StructPublisher<Pose2d> tagPosePub =
            NetworkTableInstance.getDefault()
                    .getStructTopic("AdvScope/LockedTagPose", Pose2d.struct).publish();

    // Vision-estimated pose from MegaTag2
    private final StructPublisher<Pose2d> visionPosePub =
            NetworkTableInstance.getDefault()
                    .getStructTopic("AdvScope/VisionEstimate", Pose2d.struct).publish();

    // PID error values
    private final DoublePublisher xErrPub =
            NetworkTableInstance.getDefault()
                    .getDoubleTopic("AdvScope/PID/xError_m").publish();
    private final DoublePublisher yErrPub =
            NetworkTableInstance.getDefault()
                    .getDoubleTopic("AdvScope/PID/yError_m").publish();
    private final DoublePublisher thetaErrPub =
            NetworkTableInstance.getDefault()
                    .getDoubleTopic("AdvScope/PID/thetaError_deg").publish();

    // Alignment status
    private final BooleanPublisher alignActivePub =
            NetworkTableInstance.getDefault()
                    .getBooleanTopic("AdvScope/Alignment/active").publish();
    private final BooleanPublisher onTargetPub =
            NetworkTableInstance.getDefault()
                    .getBooleanTopic("AdvScope/Alignment/onTarget").publish();

    // Tag info
    private final DoublePublisher tagIDPub =
            NetworkTableInstance.getDefault()
                    .getDoubleTopic("AdvScope/Vision/tagID").publish();
    private final DoublePublisher tagDistPub =
            NetworkTableInstance.getDefault()
                    .getDoubleTopic("AdvScope/Vision/tagDist_m").publish();
    private final DoublePublisher tagCountPub =
            NetworkTableInstance.getDefault()
                    .getDoubleTopic("AdvScope/Vision/tagCount").publish();

    // Field2d for SmartDashboard (AdvantageScope can read this too)
    private final Field2d field2d = new Field2d();

    // Track current target for periodic publishing
    private Pose2d currentTarget = null;
    private boolean alignmentActive = false;

    public VisionPID(driveKay drive, String limelightName) {
        this.drive = drive;
        this.limelightName = limelightName;

        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        xPID.setTolerance(POS_TOL);
        yPID.setTolerance(POS_TOL);
        thetaPID.setTolerance(ANG_TOL);
    }

    @Override
    public void periodic() {

        LimelightHelpers.SetRobotOrientation(
                limelightName,
                drive.getPose().getRotation().getDegrees(),
                0,0,0,0,0
        );

        fuseVision();
        publishTelemetry();
    }

    // pose fusion
    private void fuseVision() {

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 == null) return;
        if (mt2.tagCount <= 0) return;
        if (mt2.avgTagDist > MAX_TAG_DISTANCE) return;

        double yawRate =
                Units.radiansToDegrees(
                        drive.getRobotVelocity().omegaRadiansPerSecond);

        if (Math.abs(yawRate) > MAX_SPIN_DEG_PER_SEC) return;

        drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        // Publish vision estimate and tag info
        visionPosePub.set(mt2.pose);
        tagDistPub.set(mt2.avgTagDist);
        tagCountPub.set(mt2.tagCount);
    }
    
    private void publishTelemetry() {
        Pose2d robotPose = drive.getPose();

        // Robot pose
        robotPosePub.set(robotPose);
        field2d.setRobotPose(robotPose);

        // Target pose (if active)
        if (currentTarget != null) {
            targetPosePub.set(currentTarget);
            field2d.getObject("target").setPose(currentTarget);

            // PID errors
            xErrPub.set(currentTarget.getX() - robotPose.getX());
            yErrPub.set(currentTarget.getY() - robotPose.getY());
            thetaErrPub.set(Units.radiansToDegrees(
                    currentTarget.getRotation().getRadians()
                    - robotPose.getRotation().getRadians()));
        }

        // Current visible tag
        int tagID = (int) LimelightHelpers.getFiducialID(limelightName);
        tagIDPub.set(tagID);
        if (tagID > 0) {
            layout.getTagPose(tagID).ifPresent(p -> {
                tagPosePub.set(p.toPose2d());
                field2d.getObject("lockedTag").setPose(p.toPose2d());
            });
        }

        alignActivePub.set(alignmentActive);
        onTargetPub.set(isStable());
    }

    public Command alignFromVisibleTag() {

        return Commands.defer(() -> {

            int tagID = (int) LimelightHelpers.getFiducialID(limelightName);

            if (tagID < 1) return Commands.none();

            Optional<Pose2d> tagPoseOpt = layout.getTagPose(tagID).map(p -> p.toPose2d());

            if (tagPoseOpt.isEmpty()) return Commands.none();

            Pose2d tagPose = tagPoseOpt.get();

            // for 4 meters
            Translation2d offset =
                    new Translation2d(STANDOFF_DISTANCE, 0);
                
            Pose2d targetPose =
                    tagPose.transformBy(
                            new Transform2d(offset, new Rotation2d())
                    );

            targetPose =
                    new Pose2d(
                            targetPose.getTranslation(),
                            tagPose.getRotation()
                                    .plus(Rotation2d.fromDegrees(180))
                    );

            return alignToPose(targetPose);

        }, Set.of(drive));
    }

    private Command alignToPose(Pose2d target) {

        return Commands.runOnce(() -> {
            resetControllers(target);
            currentTarget = target;
            alignmentActive = true;
            onTargetStart = -1;
        }).andThen(
                Commands.run(() -> {

                    Pose2d current = drive.getPose();

                    double vx = xPID.calculate(
                            current.getX(), target.getX());
                    double vy = yPID.calculate(
                            current.getY(), target.getY());
                    double omega = thetaPID.calculate(
                            current.getRotation().getRadians(),
                            target.getRotation().getRadians());

                    vx = MathUtil.clamp(vx, -3, 3);
                    vy = MathUtil.clamp(vy, -3, 3);
                    omega = MathUtil.clamp(
                            omega,
                            -Units.degreesToRadians(360),
                            Units.degreesToRadians(360));

                    ChassisSpeeds speeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    vx, vy, omega,
                                    current.getRotation());

                    drive.driveRobotRelative(speeds);

                }).until(this::isStable)
        ).finallyDo(i -> {
                drive.driveRobotRelative(new ChassisSpeeds());
                alignmentActive = false;
                currentTarget = null;
                });

    }

    private void resetControllers(Pose2d target) {
        xPID.reset();
        yPID.reset();
        thetaPID.reset(drive.getPose().getRotation().getRadians());
        xPID.setSetpoint(target.getX());
        yPID.setSetpoint(target.getY());
        thetaPID.setGoal(target.getRotation().getRadians());
    }

    private boolean isStable() {

        boolean at =
                xPID.atSetpoint() &&
                yPID.atSetpoint() &&
                thetaPID.atGoal();

        if (!at) {
            onTargetStart = -1;
            return false;
        }

        if (onTargetStart < 0)
            onTargetStart = Timer.getFPGATimestamp();

        return Timer.getFPGATimestamp() - onTargetStart > 0.15;
    }
}