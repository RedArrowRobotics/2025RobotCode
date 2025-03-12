package frc.robot;

import frc.robot.subsystems.DriveSubsystem.DrivePower;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControlInputs {
    // Joysticks
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final GenericHID componentsBoard = new GenericHID(1);
    public DrivePower getdriveController() {
        // Multipliers for the drive stick axes
        final double driveControllerLinearMultiplier = 0.8;
        final double driveControllerRotationMultiplier = 0.5;

        // Derive joystick values. Inputs are squared to make precice control at low speeds easier
        var x = ( driveController.getLeftX() * Math.abs( driveController.getLeftX()) ) * driveControllerLinearMultiplier;
        var y = ( driveController.getLeftY() * Math.abs( driveController.getLeftY()) ) * driveControllerLinearMultiplier;
        var rotation = ( driveController.getRightX() * Math.abs( driveController.getRightX()) ) * driveControllerRotationMultiplier;
        // Compose the seperate components into a state record
        return new DrivePower(x, y, rotation);
    }

    // For later
    public final void setRumble(double value) {
        driveController.setRumble(RumbleType.kBothRumble, value);
    }
    
    public class Triggers {
        public Triggers() {
            componentsBoard.setOutput(1, false);
        }
        public final Trigger elevatorHome = new Trigger(() -> componentsBoard.getRawButtonPressed(1));
        public final Trigger elevatorL2 = new Trigger(() -> componentsBoard.getRawButtonPressed(2));
        public final Trigger elevatorL3 = new Trigger(() -> componentsBoard.getRawButtonPressed(3));
        public final Trigger elevatorL4 = new Trigger(() -> componentsBoard.getRawButtonPressed(4));
        public final Trigger deAlgae = new Trigger(() -> componentsBoard.getRawButtonPressed(5));
        public final Trigger climberDescend = new Trigger(() -> componentsBoard.getRawButton(6));
        public final Trigger climberAscend = new Trigger(() -> componentsBoard.getRawButton(7));
        public final Trigger alignReefLeft = new Trigger(() -> componentsBoard.getRawButton(10));
        public final Trigger alignReefRight = new Trigger(() -> componentsBoard.getRawButton(11));
        public final Trigger alignSource = new Trigger(() -> componentsBoard.getRawButton(12));
        public final Trigger cageGrabber = new Trigger(() -> componentsBoard.getRawButton(12));

        public final Trigger driveButtonA = driveController.a();
        public final Trigger driveButtonB = driveController.b();
        public final Trigger driveButtonX = driveController.x();
        public final Trigger driveButtonY = driveController.y();
    }
}