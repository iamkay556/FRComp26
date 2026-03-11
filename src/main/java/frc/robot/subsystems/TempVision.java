    package frc.robot.subsystems;

    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.geometry.Transform2d;
    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.math.util.Units;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.networktables.StructPublisher;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import edu.wpi.first.apriltag.AprilTagFieldLayout;
    import edu.wpi.first.apriltag.AprilTagFields;

    import java.util.Optional;

    public class TempVision extends SubsystemBase {

        private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        private final StructPublisher<Pose2d> targetPublisher = NetworkTableInstance.getDefault().getStructTopic("TargetPose", Pose2d.struct).publish();

        private long lockedTag = 0;

        private static final double STANDOFF_DISTANCE =
                Units.feetToMeters(4.0);   // 4 feet away

        public void lockIn() {

            long tid = NetworkTableInstance.getDefault().getTable("limeli   ght").getEntry("tid").getInteger(0);

            if (tid != 0) {
                lockedTag = tid;
            }
        }

        public Pose2d getTargetPose() {

            if (lockedTag == 0) {
                return new Pose2d(); // safe default
            }

            Optional<Pose2d> tagPoseOpt = layout.getTagPose((int) lockedTag).map(p -> p.toPose2d());

            if (tagPoseOpt.isEmpty()) {
                return new Pose2d();
            }

            Pose2d tagPose = tagPoseOpt.get();

            /*
            Move straight back from tag by 4 feet.
            Offset is defined in tag coordinate frame:
            +X is forward from tag.
            So we use -STANDOFF_DISTANCE to back up.
            */

            Translation2d offset =
                    new Translation2d(-STANDOFF_DISTANCE, 0);

            Pose2d offsetPose =
                    tagPose.transformBy(
                            new Transform2d(offset, new Rotation2d())
                    );


            Rotation2d facingTag =
                    tagPose.getRotation()
                            .plus(Rotation2d.fromDegrees(180));

            return new Pose2d(offsetPose.getTranslation(), facingTag);
        }

        @Override
        public void periodic() {

            Pose2d target = getTargetPose();
            targetPublisher.set(target);
        }
    }