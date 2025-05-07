package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class AlignToAprilTag {
    private final DriveSubsystem driveSubsystem;
    private final Field2d field = new Field2d();
    private final PathConstraints constraints;

    public AlignToAprilTag(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        constraints = new PathConstraints(
            subsystem.getMaximumChassisVelocity().times(0.25),
            MetersPerSecondPerSecond.of(10.6).times(0.25),
            subsystem.getMaximumChassisAngularVelocity(),
            DegreesPerSecondPerSecond.of(861),
            Volts.of(12));

        // We only need to put this object in the smart dashboard once. The 
        // dashboard will automatically update when the underlying object 
        // (the field) updates.
        SmartDashboard.putData(field);
    }

    private Command align(List<Integer> ids, Distance translation, Rotation2d rotation) {
        // Using the current robot pose, determine which of the AprilTag IDs
        // in the 'ids' list is closest. Then find the target pose by transforming  
        // into a pose with the robot facing the AprilTag. Optionally translate
        // and/or rotate the robot relative to this pose.
        Pose2d targetPose = driveSubsystem.getPose()
                .nearest(ids.stream().map((Integer id) -> Constants.fieldLayout.getTagPose(id).orElseThrow().toPose2d()).toList())
                .transformBy(new Transform2d(
                        new Translation2d(Inches.of(18), translation),
                        new Rotation2d(Degrees.of(180)).plus(rotation)));

        // Create a PathPlanner command from the target pose
        var currentPose = driveSubsystem.getPose();
        var targetRotation = targetPose.getRotation();

        // Create waypoints. As per PathPlanner documentation, pose rotation is
        // the direction of travel - relative to the current rotation of the 
        // robot - not the rotation of the robot

        // We want the rotation to be pointed towards the target. This could 
        // result in a sudden change of direction but is faster than trying to 
        // have PathPlanner create a 'smooth' transition, because it can create
        // a significant deviation in a direction that does not advance us 
        // toward the end pose, and this deviation is not proportional to the 
        // current speed of the robot in that direction.
        Pose2d start = new Pose2d(currentPose.getTranslation(), currentPose.getRotation().rotateBy(new Transform2d(currentPose, targetPose).getTranslation().getAngle()));

        // If we set the end pose's rotation to the goal rotation, the path will
        // always try to end with the robot driving directly forward into the 
        // target state. This seems to have better behavior in most cases, because 
        // rotations at the end of the path can intersect field elements.
        Pose2d end = new Pose2d(targetPose.getTranslation(), targetRotation.minus(rotation));

        // Create the path
        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(start, end),
                constraints,
                null,
                new GoalEndState(MetersPerSecond.of(0), targetRotation));
        path.preventFlipping = true;

        // Update the field object with the current path
        field.getObject("align").setPoses(path.getPathPoses());
        field.getObject("align_endpoints").setPoses(start, end);
        driveSubsystem.setPathRunning(true);
        return AutoBuilder.followPath(path).andThen( () -> {driveSubsystem.setPathRunning(false);});
    }

    public Command alignToSource() {
        return Commands.defer(() -> {
            List<Integer> ids = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
                case Blue -> Constants.blueSourceATags;
                case Red -> Constants.redSourceATags;
                default -> List.of();
            };
            // When aligning to the source, the robot needs to be rotated 90 deg
            // counter-clockwise, relative to when aligning to the reef. 
            return align(ids, Inches.of(0), Rotation2d.kCCW_90deg);
        }, Set.of(driveSubsystem));
    }

    public Command alignToReef(Distance translation) {
        return Commands.defer(() -> {
            List<Integer> ids = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
                case Blue -> Constants.blueReefATags;
                case Red -> Constants.redReefATags;
                default -> List.of();
            };
            return align(ids, translation, Rotation2d.kZero);
        }, Set.of(driveSubsystem));
    }
}