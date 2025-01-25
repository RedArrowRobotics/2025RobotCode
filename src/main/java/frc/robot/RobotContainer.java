// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveDriveTrain;
import swervelib.SwerveDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class RobotContainer {
  private final SwerveDriveTrain swerveDriveTrain;
  private final ControlInputs controlInputs = new ControlInputs();
  private final SensorInputs sensorInputs = new SensorInputs();
  private Components components = new Components();
  private final ComponentsControl componentsControl = new ComponentsControl();
  private final SendableChooser<Command> autoChooser;
  private Command autoSelected;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public RobotContainer() throws IOException, Exception{
    swerveDriveTrain = new SwerveDriveTrain();
    swerveDriveTrain.setDefaultCommand(swerveDriveTrain.driveFieldCentric(() -> {
      return new Transform2d(-controlInputs.driveStickY, -controlInputs.driveStickX, Rotation2d.fromRadians(-controlInputs.driveStickZrotation));
    }));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void robotPeriodic() {
    controlInputs.readControls();
    sensorInputs.readSensors();
  }

  public void teleopPeriodic() {
    componentsControl.runComponents(components, controlInputs, sensorInputs);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
}