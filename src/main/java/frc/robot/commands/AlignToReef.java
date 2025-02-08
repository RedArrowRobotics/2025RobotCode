package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class AlignToReef {
    private final DriveSubsystem driveSubsystem;
    private final Distance translation = Meters.of(0);

    public AlignToReef(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
    }

    public Command alignToReef() {
        return Commands.defer(() -> {
            List<Integer> ids = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
                case Blue -> Constants.blueReefATags;
                case Red -> Constants.redReefATags;
                default -> List.of();
            };
            return Arrays.stream(LimelightHelpers.getRawFiducials("limelight"))
                // Only track relevant fiducials
                .filter((fiducial) -> ids.contains(fiducial.id))
                // Sort fiducials by distance, then get the closest one
                .sorted((fiducialA,fiducialB) -> Double.compare(fiducialA.distToRobot, fiducialB.distToCamera))
                .findFirst()
                // Get the pose corresponding to the target fiducial
                .flatMap((fiducial) -> Constants.fieldLayout.getTagPose(fiducial.id))
                .map((target) -> target.toPose2d().transformBy(new Transform2d(
                    new Translation2d(Inches.of(-35.0 / 2), translation),
                    new Rotation2d(Degrees.of(0))
                )))
                // Create a PathPlanner Command from the target pose
                .map((targetPose) -> {
                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                        driveSubsystem.getPose(),
                        targetPose
                    );
                    PathConstraints constraints = new PathConstraints(
                        driveSubsystem.getMaximumChassisVelocity(),
                        MetersPerSecondPerSecond.of(3),
                        driveSubsystem.getMaximumChassisAngularVelocity(),
                        DegreesPerSecondPerSecond.of(720),
                        Volts.of(12));
                    PathPlannerPath path = new PathPlannerPath(
                        waypoints,
                        constraints,
                        null,
                        new GoalEndState(MetersPerSecond.of(0), targetPose.getRotation()));
                    path.preventFlipping = true;
                    return AutoBuilder.followPath(path);
                })
                // If we don't have a target, return a no-op Command
                .orElse(new InstantCommand());
        }, Set.of(driveSubsystem));
    }
}