package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlInputs;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.geometry.Pose2d;

public class DriveSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;

  public DriveSubsystem() throws IOException, ParseException {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    RobotConfig config;

    //In Meters Per Second
    LinearVelocity maximumSpeed = MetersPerSecond.of(4.35864);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed.magnitude());
    config = RobotConfig.fromGUISettings();
    
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red)
          .orElse(false);
      },
      this // Reference to this subsystem to set requirements
    );
  }
  
  /**
   * Command to drive the robot from joystick input using translative values
   * and heading as angular velocity. Should be used as a default command.
   *
   * @param transform_supplier  Function that returns a translation and rotation
   * @return Drive command.
   */
  public Command teleopDriveFieldCentric() {
    return this.run(() -> {
      var input = ControlInputs.getInstance().getDriveStick();
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
      chassisSpeeds.vxMetersPerSecond = -input.y() * swerveDrive.getMaximumChassisVelocity();
      chassisSpeeds.vyMetersPerSecond = -input.x() * swerveDrive.getMaximumChassisVelocity();
      chassisSpeeds.omegaRadiansPerSecond = -input.rotation() * swerveDrive.getMaximumChassisAngularVelocity();
      swerveDrive.driveFieldOriented(chassisSpeeds);
    });
  }

  /**
   * Command to drive the robot from joystick input using translative values
   * and heading as angular velocity. Should be used as a default command.
   *
   * @param transform_supplier  Function that returns a translation and rotation
   * @return Drive command.
   */
  public Command teleopDriveRobotCentric() {
    return this.run(() -> {
      var input = ControlInputs.getInstance().getDriveStick();
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
      chassisSpeeds.vxMetersPerSecond = -input.y() * swerveDrive.getMaximumChassisVelocity();
      chassisSpeeds.vyMetersPerSecond = -input.x() * swerveDrive.getMaximumChassisVelocity();
      chassisSpeeds.omegaRadiansPerSecond = -input.rotation() * swerveDrive.getMaximumChassisAngularVelocity();
      swerveDrive.drive(chassisSpeeds);
    });
  }

  @Override
  public void periodic() {
    updatePosition();
  }

  public void updatePosition() {
    LimelightHelpers.SetRobotOrientation("limelight", swerveDrive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    if (LimelightHelpers.getTV("limelight") == true) {
      // Add vision measurement
      LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight"); 
      swerveDrive.addVisionMeasurement(poseEstimate.pose, Timer.getFPGATimestamp());
    }
  }
  
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    swerveDrive.drive(chassisSpeeds);
  }
}
