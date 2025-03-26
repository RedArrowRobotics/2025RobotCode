package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlInputs.DrivePower;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSubsystem extends SubsystemBase {
    private final Optional<SwerveDrive> swerveDrive;
    private final Field2d field = new Field2d();
    boolean trustPose = false;
    boolean isPathRunning = false;

    private final SendableChooser<DriveOrientation> driveModeChooser = new SendableChooser<>();
    final Alert swerveReadError = new Alert("Failed to read swerve drive configuration.", AlertType.kError);
    final Alert pathPlannerError = new Alert("Failed to read PathPlanner configuration.", AlertType.kError);
    final Alert poseTrustWarning = new Alert("Current pose is untrusted. On-the-fly path generation will not function.", AlertType.kWarning);

    public DriveSubsystem() {
        if (DriverStation.getMatchType() == MatchType.None) {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        } else {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
        }
        // Configure swerve drive
        {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            Optional<SwerveDrive> swerve = Optional.empty();
            try {
                var parser = new SwerveParser(swerveJsonDirectory);
                swerve = Optional.of(parser.createSwerveDrive(MetersPerSecond.of(3.31).in(MetersPerSecond)));
            } catch (Exception e) {
                swerveReadError.set(true);
            }
            swerveDrive = swerve;
        }
        // Configure PathPlanner
        try {
            swerveDrive.ifPresent((swerve) -> {
                RobotConfig config;
                try {
                    config = RobotConfig.fromGUISettings();
                } catch(Exception e) {
                    // Rethrow because Java checked exceptions don't work in lambda expressions
                    throw (RuntimeException) e;
                }
                AutoBuilder.configure(
                    // Pose Supplier
                    () -> swerve.getPose(),
                    // Reset Pose
                    (pose) -> { swerve.resetOdometry(pose); trustPose = true; },
                    // Robot-relative Speeds Supplier.
                    () -> swerve.getRobotVelocity(),
                    // Drive Output
                    (speeds, feedforwards) -> swerve.drive(speeds),
                    // Path Controller
                    new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    // PathPlanner Config
                    config,
                    // Should Path be Flipped?
                    () -> DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false),
                    // Drive Subsystem
                    this
                );
            });
        } catch (Exception e) {
            pathPlannerError.set(true);
        }
        // Dashboard
        driveModeChooser.setDefaultOption("Field", DriveOrientation.FIELD_CENTRIC);
        driveModeChooser.addOption("Field", DriveOrientation.ROBOT_CENTRIC);
        SmartDashboard.putData(driveModeChooser);
        SmartDashboard.putData(field);
    }

    public static enum DriveOrientation {
        /** Forwards is a constant direction. */
        FIELD_CENTRIC,
        /** Forwards is based on the robot's direction. */
        ROBOT_CENTRIC,
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity. Should
     * be used as a default command.
     *
     * @param input_power A function that supplies multipliers on linear and angular velocity.
     * @param orientation The type of orientation to use.
     * @return Drive command.
     */
    public Command drive(SwerveInputStream inputstream) {
        return this.run(() -> {
            swerveDrive.ifPresent((swerve) -> {
                swerve.driveFieldOriented(inputstream.get());
            });
        }).withName(Constants.Commands.DRIVE);
    }

    /**
     * Command to drive the robot to a position using PathPlanner.
     *
     * @param pose The pose to pathfind to.
     * @return Drive command.
     */
    public Command driveToPose(Pose2d pose) {
        return swerveDrive.map((swerve) -> {
            PathConstraints constraints = new PathConstraints(
                MetersPerSecond.of(swerve.getMaximumChassisVelocity()),
                MetersPerSecondPerSecond.of(4.0),
                RadiansPerSecond.of(swerve.getMaximumChassisAngularVelocity()),
                DegreesPerSecondPerSecond.of(720),
                Volts.of(12)
            );
            return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                MetersPerSecond.of(0) // Goal end velocity
            );
        }).orElse(new InstantCommand()).withName(Constants.Commands.DRIVE_TO_POSE);
    }

    public Optional<SwerveDrive> getSwerve() {
        return swerveDrive;
    }

    /**
     * Drive the robot using translative values and heading as angular velocity.
     *
     * @param power Multipliers on linear and angular velocity.
     * @param orientation The type of orientation to use.
     */
    private void manualDrive(DrivePower power, DriveOrientation orientation) {
        swerveDrive.ifPresent((swerve) -> {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
            chassisSpeeds.vxMetersPerSecond = power.x() * swerve.getMaximumChassisVelocity();
            chassisSpeeds.vyMetersPerSecond = power.y() * swerve.getMaximumChassisVelocity();
            chassisSpeeds.omegaRadiansPerSecond =
                    power.rotation() * swerve.getMaximumChassisAngularVelocity();
            switch (orientation) {
                case FIELD_CENTRIC -> {
                    // TODO: Figure out whether YAGSL flips yaw based on alliance color
                    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond;
                        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond;
                    } else if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                        chassisSpeeds.vxMetersPerSecond = -chassisSpeeds.vxMetersPerSecond;
                        chassisSpeeds.vyMetersPerSecond = -chassisSpeeds.vyMetersPerSecond;
                    }
                    swerve.driveFieldOriented(chassisSpeeds);
                }
                case ROBOT_CENTRIC -> swerve.drive(chassisSpeeds);
            }
        });
    }

    public void setPathRunning(boolean isPathRunning) {
        this.isPathRunning = isPathRunning;
    }

    public DriveOrientation driveMode() {
        return driveModeChooser.getSelected();
    }

    @Override
    public void periodic() {
        updatePosition();
        field.setRobotPose(getPose());
        poseTrustWarning.set(trustPose);
    }

    public void updatePosition() {
        swerveDrive.ifPresent((swerve) -> {
            LimelightHelpers.SetRobotOrientation("limelight",
                    swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            if (LimelightHelpers.getTV("limelight") == true) {
                // Add vision measurement
                LimelightHelpers.PoseEstimate poseEstimate =
                        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
                swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999.0));
                swerve.addVisionMeasurement(poseEstimate.pose, Timer.getFPGATimestamp());
                trustPose = true;
            }
        });
    }

    public Pose2d getPose() {
        return swerveDrive.map((swerve) -> swerve.getPose()).orElse(Pose2d.kZero);
    }

    private void resetPoseTrusted(Pose2d pose) {
        swerveDrive.ifPresent((swerve) -> {
            swerve.resetOdometry(pose);
        });
        trustPose = true;
    }

    public void resetPoseUntrusted(Pose2d pose) {
        swerveDrive.ifPresent((swerve) -> {
            swerve.resetOdometry(pose);
        });
        trustPose = false;
    }

    public LinearVelocity getMaximumChassisVelocity() {
        return swerveDrive.map((swerve) -> MetersPerSecond.of(swerve.getMaximumChassisVelocity())).orElse(MetersPerSecond.zero());
    }

    public AngularVelocity getMaximumChassisAngularVelocity() {
        return swerveDrive.map((swerve) -> RadiansPerSecond.of(swerve.getMaximumChassisAngularVelocity())).orElse(RadiansPerSecond.zero());
    }

    /**
     * Whether the reported pose can be trusted to be reasonably accurate, e.g. we have seen an
     * AprilTag or have run a PathPlanner command with a known pose.
     * 
     * @return whether the pose returned from {@link #getPose()} can be trusted
     */
    public boolean isPoseTrusted() {
        return trustPose;
    }

    public void zeroGyro() {
        swerveDrive.ifPresent((swerve) -> swerve.zeroGyro());
        /*var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        switch (alliance) {
            case Red -> resetPoseUntrusted(new Pose2d(0.0, 0.0, new Rotation2d(Degrees.of(180))));
            case Blue -> resetPoseUntrusted(new Pose2d(0.0, 0.0, new Rotation2d(Degrees.zero())));
        }*/
    }
}
