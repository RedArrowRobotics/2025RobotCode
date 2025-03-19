// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Optional<Command> autonomousCommand = Optional.empty();

    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        try {
            robotContainer = new RobotContainer();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        if (robotContainer != null) {
            robotContainer.robotPeriodic();
        }
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link container} class. */
    @Override
    public void autonomousInit() {
        if (robotContainer != null) {
            robotContainer.resetGyro();
            // If we have an autonomous command selected, schedule it to be ran.
            autonomousCommand = robotContainer.getAutonomousCommand();
            autonomousCommand.ifPresent(Command::schedule);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once each time the robot enters Teleop mode. */
    @Override
    public void teleopInit() {
        // If we were running an autonomous command, cancel it when teleop starts.
        autonomousCommand.ifPresent(Command::cancel);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        robotContainer.teleopPeriodic();
    }

    /** This function is called once each time the robot enters Test mode. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
