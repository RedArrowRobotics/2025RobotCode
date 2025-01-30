package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import java.util.Optional;

public class AlignToReef extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Optional<Alliance> alliance;

    public AlignToReef(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
        alliance = DriverStation.getAlliance();
    }

    @Override
    public void initialize() {
        if(LimelightHelpers.getTV("limelight")) {
            if(alliance.get() == Alliance.Blue) {
                
            } else if(alliance.get() == Alliance.Red) {

            }
        }
      
      LimelightHelpers.getRawFiducials("limelight");
    }

    @Override
    public void execute() {

    }
  
  
    @Override
    public boolean isFinished() {
      return true;
    }
}