package frc.robot;

import frc.robot.subsystems.DriveSubsystem.DrivePower;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControlInputs {
    // Joysticks
    private final CommandXboxController driveStick = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(3);
    private final CommandJoystick componentsBoardLeft = new CommandJoystick(1);
    private final CommandJoystick componentsBoardRight = new CommandJoystick(2);

    public DrivePower getDriveStick() {
        // Multipliers for the drive stick axes
        final double driveStickLinearMultiplier = 0.8;
        final double driveStickRotationMultiplier = 0.5;

        // Derive joystick values. Inputs are squared to make precice control at low speeds easier
        var x = ( driveStick.getLeftX() * Math.abs( driveStick.getLeftX()) ) * driveStickLinearMultiplier;
        var y = ( driveStick.getLeftY() * Math.abs( driveStick.getLeftY()) ) * driveStickLinearMultiplier;
        var rotation = ( driveStick.getRightX() * Math.abs( driveStick.getRightX()) ) * driveStickRotationMultiplier;
        // Compose the seperate components into a state record
        return new DrivePower(x, y, rotation);
    }

    // For later
    public final void setRumble(double value) {
        driveStick.setRumble(RumbleType.kBothRumble, value);
    }

    public class Triggers {
        public final Trigger controllerButtonA = driveStick.a();
        public final Trigger controllerButtonB = driveStick.b();
        public final Trigger upButton = driveStick.povUp();
        public final Trigger downButton = driveStick.povDown();
        public final Trigger leftButton = driveStick.povLeft();
        public final Trigger rightButton = driveStick.povRight();
        public final Trigger leftTrigger = driveStick.leftTrigger();
        public final Trigger rightTrigger = driveStick.rightTrigger();
        public final Trigger leftBumper = driveStick.leftBumper();
        public final Trigger rightBumper = driveStick.rightBumper();
        public final Trigger controllerButtonY = driveStick.y();
        public final Trigger controllerButtonX = driveStick.x();
        public final Trigger controllerDpadUp = driveStick.povUp();
        public final Trigger controllerDpadDown = driveStick.povDown();
    }
}