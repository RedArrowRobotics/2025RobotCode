package frc.robot;

import frc.robot.subsystems.DriveSubsystem.DrivePower;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class ControlInputs {
    // Joysticks
    private final Joystick driveController = new Joystick(0);
    private final GenericHID componentsBoard = new GenericHID(1);
    public DrivePower getdriveController() {
        // Multipliers for the drive stick axes
        final double driveControllerLinearMultiplier = 0.8;
        final double driveControllerRotationMultiplier = 0.5;

        // Derive joystick values. Inputs are squared to make precice control at low speeds easier
        var x = ( driveController.getX() * Math.abs( driveController.getX()) ) * driveControllerLinearMultiplier;
        var y = ( driveController.getY() * Math.abs( driveController.getY()) ) * driveControllerLinearMultiplier;
        var rotation = ( driveController.getZ() * Math.abs( driveController.getZ()) ) * driveControllerRotationMultiplier;
        // Compose the seperate components into a state record
        return new DrivePower(-x, y, rotation);
    }

    // For later
    public final void setRumble(double value) {
        driveController.setRumble(RumbleType.kBothRumble, value);
    }
    
    public class Triggers {
        public Triggers() {
            componentsBoard.setOutput(1, false);
        }
        public final Trigger elevatorHome = new Trigger(() -> componentsBoard.getRawButton(1));
        public final Trigger elevatorL2 = new Trigger(() -> componentsBoard.getRawButton(2));
        public final Trigger elevatorL3 = new Trigger(() -> componentsBoard.getRawButton(3));
        public final Trigger elevatorL4 = new Trigger(() -> componentsBoard.getRawButton(4));
        public final Trigger manualElevatorUp = new Trigger(() ->componentsBoard.getRawAxis(0) < -0.5);
        public final Trigger manualElevatorDown = new Trigger(() -> componentsBoard.getRawAxis(0) > 0.5);
        public final Trigger intakeCoral = new Trigger(() -> componentsBoard.getRawButton(5));
        public final Trigger climberDescend = new Trigger(() -> componentsBoard.getRawButton(6));
        public final Trigger climberAscend = new Trigger(() -> componentsBoard.getRawButton(7));
        public final Trigger alignSource = new Trigger(() -> componentsBoard.getRawButton(8));
        public final Trigger alignReefRight = new Trigger(() -> componentsBoard.getRawButton(9));
        public final Trigger alignReefLeft = new Trigger(() -> componentsBoard.getRawButton(10));
        public final Trigger manualDropCoral = new Trigger(() -> componentsBoard.getRawButton(11));
        public final Trigger cageGrabber = new Trigger(() -> componentsBoard.getRawButton(12));
        public final Trigger manualCoralArmLoad = new Trigger(() -> componentsBoard.getRawAxis(1) < -0.5);
        public final Trigger manualCoralArmScore = new Trigger(() -> componentsBoard.getRawAxis(1) > 0.5);

        public final Trigger slowSpeed = new Trigger(() -> driveController.getRawButton(1) );
    }
}