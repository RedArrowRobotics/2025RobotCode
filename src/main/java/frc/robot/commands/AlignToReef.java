package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveOrientation;
import frc.robot.subsystems.DriveSubsystem.DrivePower;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class AlignToReef {
    private final DriveSubsystem driveSubsystem;
    protected Distance translation = Meters.of(0);

    public AlignToReef(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
    }

    public Command alignToReef() {
      return new Command() {
        Command inner;

        @Override
        public void initialize() {
          Optional<Pose2d> targetPose = Optional.empty();
          if(LimelightHelpers.getTV("limelight")) {
            var fiducials = LimelightHelpers.getRawFiducials("limelight");
            List<Integer> ids;
            if(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
              ids = Constants.redReefATags;
            } else {
              ids = Constants.blueReefATags;
            }
            Optional<RawFiducial> closestFiducial = Optional.empty();
            for (var fiducial : fiducials) {
              System.out.println(ids.toString());
                if (ids.contains(fiducial.id) && (closestFiducial.map((closest -> fiducial.distToRobot < closest.distToRobot)).orElse(true))) {
                  closestFiducial = Optional.of(fiducial);
                }
            }
            if (closestFiducial.isPresent()) {
              Optional<Pose3d> targetAprilTagPose = Constants.fieldLayout.getTagPose(closestFiducial.get().id);
              targetPose = targetAprilTagPose.map((target) -> target.toPose2d().transformBy(new Transform2d(
                new Translation2d(translation, Inches.of(-35.0/2)),
                new Rotation2d(Degrees.of(180))
              )));
            }
          }
          if (targetPose.isEmpty()) {
            inner = new InstantCommand();
            return;
          }
          List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            driveSubsystem.getPose(),
            targetPose.get()
          );
          PathConstraints constraints = new PathConstraints(
            driveSubsystem.getMaximumChassisVelocity(),
            MetersPerSecondPerSecond.of(3),
            driveSubsystem.getMaximumChassisAngularVelocity(),
            DegreesPerSecondPerSecond.of(720),
            Volts.of(12)
          );
          PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(MetersPerSecond.of(0), Rotation2d.fromDegrees(0))
          );
          path.preventFlipping = true;
          inner = AutoBuilder.followPath(path);
          //inner = targetPose.map((pose) -> driveSubsystem.pidDrive(pose)).orElse(new InstantCommand());
          inner.initialize();
        }
        @Override
        public void execute() {
          inner.execute();
        }
        @Override
        public void end(boolean interrupted) {
          driveSubsystem.brake();
        }
        @Override
        public boolean isFinished() {
          return inner.isFinished();
        }
      };
    }
}