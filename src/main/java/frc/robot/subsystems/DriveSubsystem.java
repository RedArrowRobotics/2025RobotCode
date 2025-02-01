package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlInputs.DriveStickState;
import frc.robot.commands.AlignToReef;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.LinearVelocity;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;

public class DriveSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;

  public DriveSubsystem() throws IOException, ParseException {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    RobotConfig config;

    LinearVelocity maximumSpeed = MetersPerSecond.of(4.35864);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed.in(MetersPerSecond));
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
  
  public static enum DriveOrientation {
    /** Forwards is a constant direction. */
    FIELD_CENTRIC,
    /** Forwards is based on the robot's direction. */
    ROBOT_CENTRIC,
  }

  /**
   * Command to drive the robot from joystick input using translative values
   * and heading as angular velocity. Should be used as a default command.
   *
   * @param orientation The type of orientation to use.
   * @return Drive command.
   */
  public Command teleopDrive(Supplier<DriveStickState> controls, DriveOrientation orientation) {
    return this.run(() -> {
      var input = controls.get();
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
      chassisSpeeds.vxMetersPerSecond = -input.y() * swerveDrive.getMaximumChassisVelocity();
      chassisSpeeds.vyMetersPerSecond = -input.x() * swerveDrive.getMaximumChassisVelocity();
      chassisSpeeds.omegaRadiansPerSecond = -input.rotation() * swerveDrive.getMaximumChassisAngularVelocity();
      switch(orientation) {
        case FIELD_CENTRIC: swerveDrive.driveFieldOriented(chassisSpeeds);
        case ROBOT_CENTRIC: swerveDrive.drive(chassisSpeeds);
      }
    });
  }

  public Command pidDrive(Pose2d target) {
    var command = new Command() {    
      private ProfiledPIDController feedback_x;
      private ProfiledPIDController feedback_y;
      private ProfiledPIDController feedback_theta;

      @Override
      public void initialize() {
        feedback_x = new ProfiledPIDController(0.1, 0, 0, new Constraints(1.0,0.1));
        feedback_y = new ProfiledPIDController(0.1, 0, 0, new Constraints(1.0,0.1));
        feedback_theta = new ProfiledPIDController(0.1, 0, 0, new Constraints(1.0,0.1));
      }
      @Override
      public void execute() {        
        // Get velocity multipliers based on distance to target
        var vx = feedback_x.calculate(getPose().getX(),target.getX());
        var vy = feedback_y.calculate(getPose().getY(),target.getY());
        var vtheta = feedback_theta.calculate(getPose().getRotation().getRadians(),target.getRotation().getRadians());
        // Create chassis speeds from multipliers and maximum velocity
        var chassisSpeeds = new ChassisSpeeds();
        chassisSpeeds.vxMetersPerSecond = vx * swerveDrive.getMaximumChassisVelocity();
        chassisSpeeds.vyMetersPerSecond = vy * swerveDrive.getMaximumChassisVelocity();
        chassisSpeeds.omegaRadiansPerSecond = vtheta * swerveDrive.getMaximumChassisAngularVelocity();
        swerveDrive.drive(chassisSpeeds);
      }
      @Override
      public boolean isFinished() {
        var current = getPose();
        return current.getMeasureX().isNear(target.getMeasureX(), Inches.of(0.5))
          && current.getMeasureY().isNear(target.getMeasureY(), Inches.of(0.5))
          && current.getRotation().getMeasure().isNear(target.getRotation().getMeasure(), Degrees.of(5));
      }
    };
    command.addRequirements(this);
    return command;
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
