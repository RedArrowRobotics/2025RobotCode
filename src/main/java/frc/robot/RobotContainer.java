// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveOrientation;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class RobotContainer {
  private final ControlInputs controlInputs = new ControlInputs();
  private final DriveSubsystem swerveDriveTrain;
  private final SensorInputs sensorInputs = new SensorInputs();
  private final SendableChooser<Command> autoChooser;
  private Command autoSelected;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public RobotContainer() throws IOException, Exception{
    swerveDriveTrain = new DriveSubsystem();
    swerveDriveTrain.setDefaultCommand(swerveDriveTrain.teleopDrive(
      () -> controlInputs.getDriveStick().toSwerve(),
      DriveOrientation.FIELD_CENTRIC
    ));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void robotPeriodic() {
    sensorInputs.readSensors();
  }

  public void teleopPeriodic() {
    
  }

  public Optional<Command> getAutonomousCommand() {
    // Fetch the selected autonomous command from the dashoard and put it in an Optional
    return Optional.ofNullable(autoChooser.getSelected());
  }
  
}