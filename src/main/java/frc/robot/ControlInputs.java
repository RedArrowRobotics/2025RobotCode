package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem.DrivePower;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class ControlInputs {
    // Joysticks
    private final XboxController driveStick = new XboxController(0);
    private final Joystick componentsBoardLeft = new Joystick(1);
    private final Joystick componentsBoardRight = new Joystick(2);

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
}