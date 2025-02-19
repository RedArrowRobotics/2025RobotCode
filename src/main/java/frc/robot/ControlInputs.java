package frc.robot;

import frc.robot.subsystems.DriveSubsystem.DrivePower;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControlInputs {
    // Joysticks
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController sysOpController = new CommandXboxController(3);
    private final CommandJoystick componentsBoardLeft = new CommandJoystick(1);
    private final CommandJoystick componentsBoardRight = new CommandJoystick(2);

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
        public final Trigger driveButtonA = driveController.a();
        public final Trigger driveButtonB = driveController.b();
        public final Trigger driveButtonX = driveController.x();
        public final Trigger driveButtonY = driveController.y();

        public final Trigger sysOpDpadUp = sysOpController.povUp();
        public final Trigger sysOpDpadDown = sysOpController.povDown();
        public final Trigger sysOpDpadLeft = sysOpController.povLeft();
        public final Trigger sysOpDpadRight = sysOpController.povRight();
        public final Trigger sysOpLeftBumper = sysOpController.leftBumper();
        public final Trigger sysOpRightBumper = sysOpController.rightBumper();
        public final Trigger sysOpLeftTrigger = sysOpController.leftTrigger();
        public final Trigger sysOpRightTrigger = sysOpController.rightTrigger();
    }
}