package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

public class SensorInputs {
    //Sensor Definitions
    private DigitalInput intakeLimitSwitchHome = new DigitalInput(0);
    private final AHRS navxAhrs = new AHRS(Port.kMXP);
    public final Encoder intakeEncoder = new Encoder(8, 9);
    
    //Variable Defintions
    public boolean intakeLimitHome = false;
    public float currentPitchDegrees = (float) 0.0;
    public float currentYawDegrees = (float) 0.0;
    public float currentRollDegrees = (float) 0.0;
    public Rotation2d drivetrainRotation = Rotation2d.fromDegrees(0.0);

    //Reading the sensors
    public void readSensors() {
        //Intake
        intakeLimitHome = intakeLimitSwitchHome.get();
        
        //NavX
        currentPitchDegrees = convertTo360(navxAhrs.getPitch());
        currentYawDegrees = convertTo360(navxAhrs.getYaw());
        currentRollDegrees = convertTo360(navxAhrs.getRoll());
        drivetrainRotation = Rotation2d.fromDegrees(currentYawDegrees);
        SmartDashboard.putNumber("NavX Pitch", currentPitchDegrees);
        SmartDashboard.putNumber("NavX Yaw", currentYawDegrees);
        SmartDashboard.putNumber("NavX Roll", currentRollDegrees);

    }

    public final float convertTo360(float input) {
        return (input + 360) % 360;
    }
}