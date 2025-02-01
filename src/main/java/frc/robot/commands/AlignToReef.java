package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

import java.util.Optional;

public class AlignToReef extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Optional<Alliance> alliance;
    private Integer targetFiducialId = null;

    public AlignToReef(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
        alliance = DriverStation.getAlliance();
    }

    @Override
    public void initialize() {
        if(LimelightHelpers.getTV("limelight")) {
          var fiducials = LimelightHelpers.getRawFiducials("limelight");
          var ids = Constants.redReefATags;            
            if(alliance.get() == Alliance.Blue) {
              ids = Constants.blueReefATags;
            }
            RawFiducial closestFiducial = null;
            for (var fiducial : fiducials) {
                if (ids.contains(fiducial.id) && (closestFiducial == null || fiducial.distToRobot < closestFiducial.distToRobot)) {
                  closestFiducial = fiducial;
                }
            }
            if (closestFiducial != null) {
              targetFiducialId = closestFiducial.id;
            }
        }
      
    }

    @Override
    public void execute() {
      if (targetFiducialId == null) {
        return;
      }
      
    }
  
  
    @Override
    public boolean isFinished() {
      return targetFiducialId == null;
    }
}