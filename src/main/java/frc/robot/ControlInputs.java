package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class ControlInputs {
    //Joysticks IDs
    private final int driveStickDeviceId = 0;
    private final int componentsBoardLeftId = 1;
    private final int componentsBoardRightId = 2;

    //Max Input
    private final double driveStickMaxMovement = 0.8;
    private final double driveStickMaxRotation = 0.5;

    //Buttons IDs
        //Components Board Right

        //Components Board Left

    //Joystick Definitions
    private final XboxController driveStick = new XboxController(driveStickDeviceId);
    private final Joystick componentsBoardLeft = new Joystick(componentsBoardLeftId);
    private final Joystick componentsBoardRight = new Joystick(componentsBoardRightId);

    //Variable Defintions
    public double driveStickX = 0.0;
    public double driveStickY = 0.0;
    public double driveStickZrotation = 0.0;

    //Reading the controls
    public final void readControls() {
        //Drivestick
        driveStickX = ( driveStick.getLeftX() * Math.abs(driveStick.getLeftX()) ) * driveStickMaxMovement;
        driveStickY = ( driveStick.getLeftY() * Math.abs(driveStick.getLeftY()) ) * driveStickMaxMovement;
        driveStickZrotation = ( driveStick.getRightX() * Math.abs(driveStick.getRightX()) ) * driveStickMaxRotation;
        //Components Board Right
        
        //Components Board Left
       
        //Drive Controller Climb
       
    }

    //For later
    public final void setRumble(double value) {
        driveStick.setRumble(RumbleType.kBothRumble, value);
    }
}