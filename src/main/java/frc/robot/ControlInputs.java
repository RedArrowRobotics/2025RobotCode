package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

public class ControlInputs {
    // Singleton Instance
    private static ControlInputs instance;

    // Joysticks IDs
    private final XboxController driveStick = new XboxController(0);
    private final Joystick componentsBoardLeft = new Joystick(1);
    private final Joystick componentsBoardRight = new Joystick(2);

    public DriveStickState getDriveStick() {
        /**
         * 
         */
        // Multipliers for the drive stick axes
        final double driveStickLinearMultiplier = 0.8;
        final double driveStickRotationMultiplier = 0.5;

        // Get joystick values
        var x = ( driveStick.getLeftX() * Math.abs(driveStick.getLeftX()) ) * driveStickLinearMultiplier;
        var y = ( driveStick.getLeftY() * Math.abs(driveStick.getLeftY()) ) * driveStickLinearMultiplier;
        var theta = ( driveStick.getRightX() * Math.abs(driveStick.getRightX()) ) * driveStickRotationMultiplier;
        // Compose the seperate components into a state record
        return new DriveStickState(x, y, theta);
    }

    // For later
    public final void setRumble(double value) {
        driveStick.setRumble(RumbleType.kBothRumble, value);
    }

    /**
     * Returns the ControlInputs instance.
     *
     * @return the instance
     */
    public static synchronized ControlInputs getInstance() {
        if (instance == null) {
            instance = new ControlInputs();
        }
        return instance;
    }

    public static record DriveStickState(double x,double y,double theta) {}
}