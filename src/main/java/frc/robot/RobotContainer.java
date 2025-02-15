// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.hal.MatchInfoData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CageSubsystem;
import frc.robot.subsystems.CoralScoringDeviceSubsystem;
import frc.robot.commands.AlignToReef;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveOrientation;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class RobotContainer {

    private final ControlInputs controlInputs = new ControlInputs();
    private final ControlInputs.Triggers controlTriggers = controlInputs.new Triggers();
    private final DriveSubsystem swerveDriveTrain;
    private final CoralScoringDeviceSubsystem coralScoringDevice;
    private final ElevatorSubsystem elevator;
    private final SensorInputs sensorInputs = new SensorInputs();
    private final CageSubsystem cage;
        private final SendableChooser<Command> autoChooser;
        private Command autoSelected;
    
        /**
         * This function is run when the robot is first started up and should be used
         * for any
         * initialization code.
         */
    
        public RobotContainer() throws IOException, Exception {
            swerveDriveTrain = new DriveSubsystem();
            swerveDriveTrain.setDefaultCommand(swerveDriveTrain.teleopDrive(
                    () -> controlInputs.getdriveController().toSwerve(),
                    DriveOrientation.FIELD_CENTRIC));
            
            coralScoringDevice = new CoralScoringDeviceSubsystem();
            controlTriggers.driveControllerButtonB.toggleOnTrue(coralScoringDevice.grabCoral());
            coralScoringDevice.reefTrigger.toggleOnTrue(coralScoringDevice.dropCoral());
            elevator = new ElevatorSubsystem();
            controlTriggers.sysOpControllerDpadUp.onTrue(elevator.elevatorHome());
            controlTriggers.sysOpControllerDpadDown.onTrue(elevator.elevatorL2());
            controlTriggers.sysOpControllerDpadLeft.onTrue(elevator.elevatorL3());
            controlTriggers.sysOpControllerDpadRight.onTrue(elevator.elevatorL4());
            controlTriggers.sysOpControllerLeftTrigger.onTrue(elevator.dealgaeExtend());
            controlTriggers.sysOpControllerRightTrigger.onTrue(elevator.dealgaeRetract());
            controlTriggers.sysOpControllerLeftBumper.onTrue(elevator.dealgaeStartSpin());
            controlTriggers.sysOpControllerRightBumper.onTrue(elevator.dealgaeStopSpin());
            cage = new CageSubsystem();
            controlTriggers.driveControllerButtonY.toggleOnTrue(cage.ascend());
            controlTriggers.driveControllerButtonX.toggleOnTrue(cage.descend());
            controlTriggers.driveControllerDpadDown.onTrue(cage.holdCage());
            controlTriggers.driveControllerDpadUp.onTrue(cage.releaseCage());
            var commands = new AlignToReef(swerveDriveTrain);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Resets the robot's pose to the alliance default.
     * Blue alliance robots start facing at 180° (towards the blue alliance wall).
     */
    public void resetPoseToDefault() {
        swerveDriveTrain.resetPose(switch (DriverStation.getAlliance().orElse(Alliance.Red)) {
            case Blue -> new Pose2d(0.0,0.0,Rotation2d.k180deg);
            case Red -> new Pose2d(0.0,0.0,Rotation2d.kZero);
            default -> new Pose2d(0.0,0.0,Rotation2d.kZero);
        });
    }

    public void robotPeriodic() {
        sensorInputs.readSensors();
        SmartDashboard.putData(coralScoringDevice);
    }

    public void teleopPeriodic() {
    
    }

    public Optional<Command> getAutonomousCommand() {
        // Fetch the selected autonomous command from the dashoard and put it in an
        // Optional
        return Optional.ofNullable(autoChooser.getSelected());
    }
}