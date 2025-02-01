package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
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
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

import java.util.Optional;
import java.util.function.Function;

public class AlignToReef {
    private final DriveSubsystem driveSubsystem;
    private final Optional<Alliance> alliance;
    Pose2d startPose = Pose2d.kZero;
    protected Distance translation = Meters.of(0);

    public AlignToReef(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        alliance = DriverStation.getAlliance();
    }

    public Command alignToReef() {
      return new Command() {
        Command inner;

        @Override
        public void initialize() {
          Optional<Pose2d> targetPose = Optional.empty();
          if(LimelightHelpers.getTV("limelight")) {
            var fiducials = LimelightHelpers.getRawFiducials("limelight");
            var ids = Constants.redReefATags;            
            if(alliance.orElse(Alliance.Red) == Alliance.Blue) {
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
          inner = targetPose.map((pose) -> driveSubsystem.pidDrive(pose)).orElse(new InstantCommand());
          inner.initialize();
        }
        @Override
        public void execute() {
          inner.execute();
        }
        @Override
        public boolean isFinished() {
          return inner.isFinished();
        }
      };
    }
}