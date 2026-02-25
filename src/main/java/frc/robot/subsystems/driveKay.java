package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
public class driveKay extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private NetworkTable table;
    private SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();



    public driveKay() {
        double maximumSpeed = Units.feetToMeters(15.5);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean rorb = alliance.orElse(Alliance.Red) == DriverStation.Alliance.Red;
        Pose2d startingPose = rorb ? new Pose2d(new Translation2d(1,
                                                                      4),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(16,
                                                                      4),
                                                    Rotation2d.fromDegrees(180));
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, startingPose);
        } catch (IOException e) {
            throw new RuntimeException("Runtime error when creating a new swerve drive:\n" + e);
        }



        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        swerveDrive.setModuleEncoderAutoSynchronize(true, 0.50);
        swerveDrive.setAutoCenteringModules(false);
        swerveDrive.zeroGyro();

        SmartDashboard.putData("Field", field);
        //swerveDrive.getGyro().setOffset(new Rotation3d(0,0,Math.PI));
          
    }
    
    
    public void periodic() {  
      field.setRobotPose(getPose());

      SmartDashboard.putNumber("Pose X m", getPose().getX());
      SmartDashboard.putNumber("Pose Y m", getPose().getY());
      SmartDashboard.putNumber("Pose Rot deg", getPose().getRotation().getDegrees());

  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Command zeroGyroCommand(){
      return Commands.runOnce( ()-> swerveDrive.zeroGyro());
  }
  
  public SwerveModulePosition[] getModulePositions() {
    Map<String, swervelib.SwerveModule> modules = swerveDrive.getModuleMap();
    return new SwerveModulePosition[] {
        modules.get("frontleft").getPosition(),
        modules.get("frontright").getPosition(),
        modules.get("backleft").getPosition(),
        modules.get("backright").getPosition()
    };  
  }

  //   public ChassisSpeeds getChassisSpeeds(){
  //     Map<String, swervelib.SwerveModule> mapvelo = swerveDrive.getModuleMap();
  //     return swerveDrive.kinematics.toChassisSpeeds(
  //       mapvelo.get("backright").getState(), 
  //       mapvelo.get("frontleft").getState(), 
  //       mapvelo.get("frontright").getState(), 
  //       mapvelo.get("backleft").getState());
  // }

  public ChassisSpeeds getChassisSpeeds(){
      Map<String, swervelib.SwerveModule> mapvelo = swerveDrive.getModuleMap();
      return swerveDrive.kinematics.toChassisSpeeds(
        mapvelo.get("frontleft").getState(), 
        mapvelo.get("frontright").getState(), 
        mapvelo.get("backleft").getState(), 
        mapvelo.get("backright").getState());
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }
  

  public void configureAutoBuilder(){
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("PathPlanner RobotConfig missing/invalid. AutoBuilder not configured.", e.getStackTrace());
      return;
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE CHANGE TO GET CHASSIS SPEEDS IF NEEDED
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                                new PIDConstants(1, 0, 0), // Translation PID constants
                                new PIDConstants(1, 0, 0) // Rotation PID constants
                        ),
                        config, // The robot configuration
                        () -> {
                          // Boolean supplier that controls when the path will be mirrored for the red alliance
                          // This will flip the path being followed to the red side of the field.
                          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            
                          Optional<Alliance> alliance = DriverStation.getAlliance();
                          return alliance.isPresent() && alliance.get() == Alliance.Red;
                        },
                        this // Reference to this subsystem to set requirements
                );
              }


    // public BooleanSupplier isValidTarget(){
    //   return ()-> (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0) != 0);
    // }


    public Command driveToPose(Pose2d targetPose) { 
      // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
          swerveDrive.getMaximumChassisVelocity(), 4.0,
          swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(
          targetPose,
          constraints,
          edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
      );
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
     // TODO Auto-generated method stub
     swerveDrive.drive(speeds);
              }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
      SmartDashboard.putString("X: ", "heehee" + MathUtil.applyDeadband(translationX.getAsDouble(), 0.2));
      SmartDashboard.putString("Y: ", "heehee" + MathUtil.applyDeadband(translationY.getAsDouble(), 0.2));
      SmartDashboard.putString("Z: ","heehee" + Math.pow(MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.5), 3));
      return run(() -> {
        // Make the robot move
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
            MathUtil.applyDeadband(translationX.getAsDouble(), 0.2)* swerveDrive.getMaximumChassisVelocity(),
            MathUtil.applyDeadband(translationY.getAsDouble(), 0.2) * swerveDrive.getMaximumChassisVelocity()
                ),0.8),
            MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.2) 
                * swerveDrive.getMaximumChassisAngularVelocity(),
            false,
            false);
      });
    }
}
// hi