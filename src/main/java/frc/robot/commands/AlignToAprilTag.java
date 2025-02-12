package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
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

import java.util.Arrays;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class AlignToAprilTag {
    private final DriveSubsystem driveSubsystem;

    public AlignToAprilTag(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
    }

    public alignToSource() {
        
    }

    public Command alignToReef(Distance translation) {
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
                .sorted((fiducialA,fiducialB) -> Double.compare(fiducialA.distToRobot, fiducialB.distToRobot))
                .findFirst()
                // Get the pose corresponding to the target fiducial
                .flatMap((fiducial) -> Constants.fieldLayout.getTagPose(fiducial.id))
                .map((target) -> target.toPose2d().transformBy(new Transform2d(
                    new Translation2d(Inches.of(-35.0 / 2), translation),
                    new Rotation2d(Degrees.of(180))
                )))
                // Create a PathPlanner command from the target pose
                .map((targetPose) -> {
                    var currentPose = driveSubsystem.getPose();
                    var targetRotation = targetPose.getRotation();
                    // Create waypoints. As per PathPlanner documentation, pose rotation is
                    // the direction of travel, not the rotation of the robot
                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                        new Pose2d(currentPose.getTranslation(),Rotation2d.kZero),
                        new Pose2d(targetPose.getTranslation(),targetPose.minus(currentPose).getTranslation().getAngle())
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
                        new GoalEndState(MetersPerSecond.of(0), targetRotation));
                    path.preventFlipping = true;
                    return AutoBuilder.followPath(path);
                })
                // If we don't have a target, return a no-op command
                .orElse(new InstantCommand());
        }, Set.of(driveSubsystem));
    }
}