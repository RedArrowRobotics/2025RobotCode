package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.cameraserver.CameraServer;

public class DriveSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;
  private final Field2d field = new Field2d();
  boolean trustPose = false;
  boolean isPathRunning = false;
  final LinearVelocity maximumSpeed = MetersPerSecond.of(3.31);

  public DriveSubsystem() throws IOException, ParseException {
    if (DriverStation.getMatchType() == MatchType.None) {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    } else {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    }

    RobotConfig config;

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed.in(MetersPerSecond));
    config = RobotConfig.fromGUISettings();

    SmartDashboard.putData(field);
    
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPoseTrusted, // Method to reset odometry (will be called if your auto has a starting pose)
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
  
  public static enum DriveOrientation {
    /** Forwards is a constant direction. */
    FIELD_CENTRIC,
    /** Forwards is based on the robot's direction. */
    ROBOT_CENTRIC,
  }

  /**
   * Holds unitless multipliers on linear and angular velocity.
   */
  public static record DrivePower(double x,double y,double rotation) {
    public DrivePower toSwerve() {
      return new DrivePower(this.y(), this.x(), -this.rotation());
    }
    public DrivePower times(double multiplier) {
      return new DrivePower(this.x() * multiplier, this.y() * multiplier, this.rotation() * multiplier);
    }
  }
  
  /**
   * Command to drive the robot  using translative values and heading as angular velocity.
   * Should be used as a default command.
   *
   * @param input_power A function that supplies multipliers on linear and angular velocity.
   * @param orientation The type of orientation to use.
   * @return Drive command.
   */
  public Command teleopDrive(Supplier<DrivePower> input_power, DriveOrientation orientation) {
    return this.run(() -> {
      var power = input_power.get();
      manualDrive(power, orientation);
    });
  }

  public Command brake() {
    return this.runOnce(() -> {
      manualDrive(new DrivePower(0, 0, 0), DriveOrientation.FIELD_CENTRIC);
    });
  }

  public Command testDrive() {
   return run( () -> {
   
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    chassisSpeeds.vxMetersPerSecond = .1 * swerveDrive.getMaximumChassisVelocity();
    chassisSpeeds.vyMetersPerSecond = 0 * swerveDrive.getMaximumChassisVelocity();
    chassisSpeeds.omegaRadiansPerSecond = 0 * swerveDrive.getMaximumChassisAngularVelocity();
    swerveDrive.drive(chassisSpeeds, true, new Translation2d());
    //switch(orientation) {
    //  case FIELD_CENTRIC -> swerveDrive.driveFieldOriented(chassisSpeeds);
    //  case ROBOT_CENTRIC -> swerveDrive.drive(chassisSpeeds);
    //}   
   });
  }

  /**
   * Drive the robot using translative values and heading as angular velocity.
   *
   * @param power Multipliers on linear and angular velocity.
   * @param orientation The type of orientation to use.
   */
  private void manualDrive(DrivePower power, DriveOrientation orientation) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    chassisSpeeds.vxMetersPerSecond = power.x() * swerveDrive.getMaximumChassisVelocity();
    chassisSpeeds.vyMetersPerSecond = power.y() * swerveDrive.getMaximumChassisVelocity();
    chassisSpeeds.omegaRadiansPerSecond = power.rotation() * swerveDrive.getMaximumChassisAngularVelocity();
    switch(orientation) {
      case FIELD_CENTRIC -> {
        //TODO: Figure out whether YAGSL flips yaw based on alliance color
        if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
          chassisSpeeds.vxMetersPerSecond = -chassisSpeeds.vxMetersPerSecond;
          chassisSpeeds.vyMetersPerSecond = -chassisSpeeds.vyMetersPerSecond;
        } else if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
          chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond;
          chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond;
        }
        swerveDrive.driveFieldOriented(chassisSpeeds);
      }
      case ROBOT_CENTRIC -> swerveDrive.drive(chassisSpeeds);
    }
  }

  public void setPathRunning(boolean isPathRunning) {
      this.isPathRunning = isPathRunning;
  }

  @Override
  public void periodic() {
   // if (!isPathRunning) {
      updatePosition();
      field.setRobotPose(getPose());
    //}
  }

  public void updatePosition() {
    LimelightHelpers.SetRobotOrientation("limelight", swerveDrive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    if (LimelightHelpers.getTV("limelight") == true) {
      // Add vision measurement
      LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999.0)); 
      swerveDrive.addVisionMeasurement(poseEstimate.pose, Timer.getFPGATimestamp());
      trustPose = true;
    }
  }
  
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  private void resetPoseTrusted(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    trustPose = true;
  }

  public void resetPoseUntrusted(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    trustPose = false;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    swerveDrive.drive(chassisSpeeds);
  }

  public LinearVelocity getMaximumChassisVelocity() {
    return maximumSpeed;//MetersPerSecond.of(swerveDrive.getMaximumChassisVelocity());
  }

  public AngularVelocity getMaximumChassisAngularVelocity() {
    return RadiansPerSecond.of(swerveDrive.getMaximumChassisAngularVelocity());
  }
  
  /**
   * Whether the reported pose can be trusted to be reasonably accurate,
   * e.g. we have seen an AprilTag or have run a PathPlanner command with
   * a known pose.
   * 
   * @return whether the pose returned from {@link #getPose()} can be trusted
   */
  public boolean isPoseTrusted() {
    return trustPose;
  }

  public void resetGyro() {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    switch (alliance) {
      case Red -> resetPoseUntrusted(new Pose2d(0.0,0.0, new Rotation2d(Degrees.of(180))));
      case Blue -> resetPoseUntrusted(new Pose2d(0.0,0.0, new Rotation2d(Degrees.zero())));
    }
  }
}
