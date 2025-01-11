package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.io.File;
import java.io.IOError;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDriveTrain {
  SwerveDrive swerveDrive;

  public SwerveDriveTrain() throws IOException {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    //In Meters Per Second
    double maximumSpeed = 4.35864;
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public void driveFC(Double translationX, Double translationY, Double angularRotationX) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    chassisSpeeds.vxMetersPerSecond = translationX * swerveDrive.getMaximumChassisVelocity();
    chassisSpeeds.vyMetersPerSecond = translationY * swerveDrive.getMaximumChassisVelocity();
    chassisSpeeds.omegaRadiansPerSecond = angularRotationX * swerveDrive.getMaximumChassisAngularVelocity();
    swerveDrive.driveFieldOriented(chassisSpeeds);
  }

    /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public void drive(Double translationX, Double translationY, Double angularRotationX) {
    // Make the robot move
    swerveDrive.drive(
        new Translation2d(
            translationX * swerveDrive.getMaximumChassisVelocity(),
            translationY * swerveDrive.getMaximumChassisVelocity()
        ),
        angularRotationX * swerveDrive.getMaximumChassisAngularVelocity(),
        false,
        false);
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
}
