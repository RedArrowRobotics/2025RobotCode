// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToAprilTag;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CageSubsystem;
import frc.robot.subsystems.CoralScoringDeviceSubsystem;
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
    private final CoralScoringDeviceSubsystem coralArm;
    private final ElevatorSubsystem elevator;
    private final SensorInputs sensorInputs = new SensorInputs();
    private final CageSubsystem cage;
    private final SendableChooser<Command> autoChooser;

    public Trigger reefTrigger;


    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */

    public RobotContainer() throws IOException, Exception {
        swerveDriveTrain = new DriveSubsystem();
        swerveDriveTrain.setDefaultCommand(swerveDriveTrain.teleopDrive(
                () -> {
                    var power = controlInputs.getdriveController().toSwerve();
                    if (controlTriggers.driveButtonA.getAsBoolean()) {
                        power = power.times(0.5);
                    }
                    return power;
                },
                DriveOrientation.FIELD_CENTRIC));
      

            
        var commands = new AlignToAprilTag(swerveDriveTrain);
        coralArm = new CoralScoringDeviceSubsystem();
        elevator = new ElevatorSubsystem();
        cage = new CageSubsystem();
      
        // PathPlanner Commands
        NamedCommands.registerCommand(Constants.ALIGN_REEF_LEFT, commands.alignToReef(Inches.of(-6.5)));
        NamedCommands.registerCommand(Constants.ALIGN_REEF_RIGHT, commands.alignToReef(Inches.of(6.5)));
        NamedCommands.registerCommand(Constants.ALIGN_SOURCE, commands.alignToSource());

        //Coral Scoring Device
        NamedCommands.registerCommand("Intake Coral", coralArm.grabCoral());
        NamedCommands.registerCommand("Outtake Coral", coralArm.dropCoral());
        NamedCommands.registerCommand("Stop Coral Wheels", coralArm.stopScorerSpin());
        NamedCommands.registerCommand("Coral Loading Position", coralArm.loadCoralPosition());
        NamedCommands.registerCommand("Coral Scoring Position", coralArm.scoreCoralPosition());
        //Cage
        NamedCommands.registerCommand("Ascend Cage", cage.ascend());
        NamedCommands.registerCommand("Descend Cage", cage.descend());
        //Elevator
        NamedCommands.registerCommand("Score L1", new WaitCommand(5));
        NamedCommands.registerCommand("Score L2", new WaitCommand(5));
        NamedCommands.registerCommand("Score L3", new WaitCommand(5));
        NamedCommands.registerCommand("Score L4", new WaitCommand(5));
        NamedCommands.registerCommand("Extend Algae Remover", elevator.dealgaeExtend());
        NamedCommands.registerCommand("Retract Algae Remover", elevator.dealgaeRetract());
        NamedCommands.registerCommand("Turn on Dealgae Wheels", elevator.dealgaeStartSpin());
        NamedCommands.registerCommand("Turn off Dealgae Wheels", elevator.dealgaeStopSpin());


        reefTrigger = new Trigger(() -> {return coralArm.armIsInPosition() && coralArm.isCoralOverReef() && elevator.elevatorIsInPosition();});

        controlTriggers.elevatorHome.onTrue(Commands.deadline(coralArm.loadCoralPosition(), elevator.elevatorL2()).andThen(elevator.elevatorHome()));
        controlTriggers.elevatorL2.onTrue(elevator.elevatorL2().alongWith(Commands.waitUntil(() -> elevator.isElevatorAtL2()).andThen(coralArm.scoreCoralPosition())));
        controlTriggers.elevatorL3.onTrue(elevator.elevatorL3().alongWith(Commands.waitUntil(() -> elevator.isElevatorAtL2()).andThen(coralArm.scoreCoralPosition())));
        controlTriggers.elevatorL4.onTrue(elevator.elevatorL4().alongWith(Commands.waitUntil(() -> elevator.isElevatorAtL2()).andThen(coralArm.scoreCoralPosition())));

        controlTriggers.deAlgae.whileTrue(elevator.dealgaeStartSpin());
        reefTrigger.toggleOnTrue(coralArm.dropCoral());

        controlTriggers.climberDescend.whileTrue(cage.descend());
        controlTriggers.climberAscend.whileTrue(cage.ascend());

        controlTriggers.alignReefLeft.and(swerveDriveTrain::isPoseTrusted).whileTrue(NamedCommands.getCommand(Constants.ALIGN_REEF_LEFT));
        controlTriggers.alignReefRight.and(swerveDriveTrain::isPoseTrusted).whileTrue(NamedCommands.getCommand(Constants.ALIGN_REEF_RIGHT));
        controlTriggers.alignSource.and(swerveDriveTrain::isPoseTrusted).whileTrue(NamedCommands.getCommand(Constants.ALIGN_SOURCE));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void robotPeriodic() {
        sensorInputs.readSensors();
        SmartDashboard.putData(coralArm);
        SmartDashboard.putData(elevator);
        SmartDashboard.putData(cage);
        SmartDashboard.putData(swerveDriveTrain);
    }

    public void teleopPeriodic() {
    
    }

    public Optional<Command> getAutonomousCommand() {
        // Fetch the selected autonomous command from the dashoard and put it in an
        // Optional
        return Optional.ofNullable(autoChooser.getSelected());
    }
}